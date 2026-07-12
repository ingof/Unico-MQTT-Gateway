// =====================================================
// V_1_3_1.ino
// ESP32 RS485 Modbus -> Web + MQTT Gateway fuer UNICO
// Version: V1.3.2
// Build marker: V1.3.2_SETTINGS_PANEL_ACTIVE - search this text in Arduino IDE to verify the correct sketch file.
// Hardware: ESP32 + RS485-Board, RX=GPIO16, TX=GPIO17, DIR=GPIO18
// Modbus: 9600 8N1, Slave-ID 1, FC03 Lesen, FC06 Schreiben
// MQTT: PubSubClient Library erforderlich
// =====================================================

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <esp_system.h>
#include <math.h>
#include "UGW_secrets.h"

// =====================================================
// Version
// =====================================================
const char* FW_VERSION = "V1.3.2";
const char* FW_NAME    = "Unico-Gateway";

// =====================================================
// Laufzeit-Konfiguration
// UGW_secrets.h liefert die Flash-/Fallback-Defaults.
// Webinterface-Aenderungen werden in ESP32 Preferences/NVS gespeichert
// und ueberschreiben die Defaults zur Laufzeit.
// =====================================================
String cfgWifiSsid;
String cfgWifiPass;
String cfgApSsid;
String cfgApPass;
String cfgMqttHost;
uint16_t cfgMqttPort = 1883;
String cfgMqttUser;
String cfgMqttPass;
String cfgMqttClientId;
String cfgMqttRoot;

// MQTT-Sendeintervall fuer zyklische State-/Sensor-Topics.
// Die Mess-/Statusdaten werden nicht bei jedem Modbus-Poll an MQTT geschickt,
// sondern gebuendelt in diesem Intervall. Wichtige Bedienzustandsaenderungen
// wie Betriebsart und Solltemperatur werden zusaetzlich sofort veroeffentlicht.
uint16_t cfgMqttPublishIntervalSec = 10;

// Modbus-/RS485-Parameter. Defaults sind die bekannten UNICO-Werte;
// per Webinterface gespeicherte Werte ueberschreiben sie zur Laufzeit.
uint8_t  cfgModbusSlaveId      = 1;
uint32_t cfgModbusBaudrate     = 9600;
uint8_t  cfgModbusDataBits     = 8;
char     cfgModbusParity       = 'N';   // N=None, E=Even, O=Odd
uint8_t  cfgModbusStopBits     = 1;
uint16_t cfgModbusTimeoutMs    = 700;
uint16_t cfgModbusPollIntervalMs = 2000;

String cfgConfigPassword = "";
bool cfgClimateProtected = false;
// Schaltet die komplette Web-Bedienung frei oder sperrt sie.
// Das ist getrennt vom Passwortschutz: Aus = keine schreibenden Webbefehle,
// Ein = schreiben erlaubt; optional zusaetzlich per Passwort gesichert.
bool cfgWebCommandsEnabled = true;
bool cfgMqttCommandsEnabled = false;
String configSessionToken = "";

String lastMqttError = "Noch nicht verbunden";
int lastMqttState = -1;

// Forward declarations for functions used before their definition
void sendNoCache();
bool readHolding(uint8_t slave, uint16_t startAddr, uint16_t count, uint16_t* values);
bool writeHoldingVerified(uint16_t addr, uint16_t value, uint16_t verifyMask, String label);
void mqttCallback(char* topicChars, byte* payload, unsigned int length);
void clearSerial2();
String modbusParityText();
void applyModbusSerial();
void publishControlChangesIfNeeded(bool force);
String publicStatusText();



// =====================================================
// Hardware / Modbus Parameter
// =====================================================
#define MODBUS_RX_PIN   16
#define MODBUS_TX_PIN   17
#define MODBUS_DIR_PIN  18

// Schreibbefehle bekommen bewusst kleine Ruhezeiten.
// Die UNICO-Steuerplatine ignoriert sonst gelegentlich direkt aufeinanderfolgende Frames.
const uint16_t WRITE_PAUSE_BEFORE   = 300;
const uint16_t WRITE_PAUSE_AFTER    = 400;
const uint8_t  WRITE_RETRIES        = 3;

// =====================================================
// Server / MQTT
// =====================================================
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Preferences prefs;

// =====================================================
// Registerspeicher
// =====================================================
const uint16_t MAX_REG_STORE = 400;
uint16_t regValue[MAX_REG_STORE];
bool regValid[MAX_REG_STORE];

// =====================================================
// Status / Zaehler
// =====================================================
uint32_t lastPollMs = 0;
uint32_t okPolls = 0;
uint32_t timeoutErrors = 0;
uint32_t crcErrors = 0;
uint32_t formatErrors = 0;
uint32_t modbusErrors = 0;
uint32_t writeOk = 0;
uint32_t writeErrors = 0;
uint32_t mqttPublishes = 0;
uint32_t mqttReconnectAttempts = 0;
uint32_t lastMqttTryMs = 0;
uint32_t lastMqttPublishMs = 0;

// Merker fuer sofortige MQTT-Updates der wichtigen Bedienzustandswerte.
// Damit landen Moduswechsel und Solltemperaturaenderungen sofort in openHAB,
// waehrend die normalen Messwerte nur im eingestellten Intervall gesendet werden.
bool lastMqttModeValid = false;
uint16_t lastMqttModeReg14 = 0;
bool lastMqttSetpointValid = false;
uint16_t lastMqttSetpointReg8 = 0;

bool writeBusy = false;
String lastStatus = "Start";
String wifiModeText = "";

// =====================================================
// Hilfsdaten 0014 Aktionen
// Maskierung konservativ:
// - einfache Aktionen aendern nur MODE BIT07..BIT05
// - '+ Luefter Auto' aendert MODE und Luefter BIT04..BIT03
// Andere Bits in 0014 bleiben bei maskiertem Schreiben erhalten.
// =====================================================
struct Action14 {
  const char* name;
  uint16_t value;
  uint16_t mask;
};

const Action14 actions14[] = {
  {"Aus / Standby",           0x0000, 0x00E0},
  {"Kuehlen",                 0x0020, 0x00E0},
  {"Heizen",                  0x0040, 0x00E0},
  {"Entfeuchten",             0x0060, 0x00E0},
  {"Lueften",                 0x0080, 0x00E0},
  {"Automatik",               0x00A0, 0x00E0},
  {"Kuehlen + Luefter Auto",  0x0038, 0x00F8},
  {"Heizen + Luefter Auto",   0x0058, 0x00F8},
  {"Entfeuchten + Luefter Auto", 0x0078, 0x00F8},
  {"Lueften + Luefter Auto",  0x0098, 0x00F8}
};
const uint8_t ACTION14_COUNT = sizeof(actions14) / sizeof(actions14[0]);

// =====================================================
// HTML
// =====================================================
const char MAIN_page[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Unico-Gateway V1.3.2</title>
<style>
:root { --footerH: 46px; --tabH: 48px; }
html { height: 100%; }

* {
  scrollbar-width: thin;
  scrollbar-color: #3a3a3a #202020;
}
*::-webkit-scrollbar {
  width: 10px;
  height: 10px;
}
*::-webkit-scrollbar-track {
  background: #202020;
}
*::-webkit-scrollbar-thumb {
  background: #3a3a3a;
  border-radius: 8px;
  border: 2px solid #202020;
}
*::-webkit-scrollbar-thumb:hover {
  background: #404040;
}
*::-webkit-scrollbar-corner {
  background: #202020;
}
body {
  font-family: Arial, sans-serif;
  background: #111;
  color: #eee;
  margin: 0;
  height: 100vh;
  height: 100dvh;
  overflow: hidden;
  box-sizing: border-box;
  padding-bottom: var(--footerH);
  display: flex;
  flex-direction: column;
}
header {
  background: #1d1d1d;
  border-bottom: 1px solid #444;
  box-sizing: border-box;
}
.topbar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  padding: 8px 12px 6px 12px;
}
h1 { margin: 0; font-size: 18px; }
.subtitle { color: #aaa; font-size: 12px; white-space: nowrap; }
.tabs {
  display: flex;
  gap: 6px;
  padding: 0 12px 8px 12px;
  overflow-x: auto;
  -webkit-overflow-scrolling: touch;
}
.tabbtn {
  width: auto;
  min-width: 126px;
  padding: 8px 14px;
  background: #262626;
  color: #ddd;
  border: 1px solid #555;
  border-radius: 6px 6px 0 0;
  cursor: pointer;
  font-weight: 600;
}
.tabbtn.active {
  background: #3a3a3a;
  color: #fff;
  border-bottom-color: #3a3a3a;
}

.top-separator {
  height: 4px;
  background: #050505;
  border-top: 1px solid #000;
  border-bottom: 1px solid #181818;
}
.footer-separator {
  position: fixed;
  left: 0;
  right: 0;
  bottom: var(--footerH);
  height: 4px;
  z-index: 9998;
  background: #050505;
  border-top: 1px solid #000;
  border-bottom: 1px solid #181818;
}

main {
  flex: 1 1 auto;
  min-height: 0;
  overflow: auto;
  -webkit-overflow-scrolling: touch;
  padding: 10px 12px 0 12px;
  box-sizing: border-box;
  background: #111;
}
.tabpage { display: none; }
.tabpage.active { display: block; }
.section {
  border: 1px solid #333;
  background: #181818;
  margin-bottom: 12px;
  border-radius: 6px;
  overflow: hidden;
}
.section h2 {
  margin: 0;
  padding: 8px 10px;
  background: #222;
  border-bottom: 1px solid #333;
  font-size: 15px;
}
.content { padding: 10px; }
.control-layout {
  display: grid;
  grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
  gap: 12px;
  align-items: start;
}
.grid2 {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 12px;
}
.grid3 {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 12px;
}
.card {
  border: 1px solid #2d2d2d;
  border-radius: 6px;
  overflow: hidden;
  background: #151515;
}
.card h3 {
  margin: 0;
  padding: 7px 9px;
  background: #202020;
  font-size: 14px;
  border-bottom: 1px solid #2d2d2d;
}
table {
  width: 100%;
  border-collapse: collapse;
  font-size: 13px;
}
th, td {
  padding: 6px 7px;
  border-bottom: 1px solid #292929;
  text-align: left;
  white-space: nowrap;
}
th { color: #ccc; background: #202020; }
td.value { font-family: Consolas, monospace; text-align: right; }
tr.gap td { padding-top: 13px; border-top: 1px solid #444; }
input, select, button {
  width: 100%;
  box-sizing: border-box;
  padding: 7px;
  background: #222;
  color: #eee;
  border: 1px solid #555;
  border-radius: 4px;
}
select:disabled, input:disabled {
  color: #ddd;
  background: #1d1d1d;
  opacity: 1;
}
button { cursor: pointer; background: #333; }
button:hover { background: #444; }
button:disabled { color: #777; cursor: not-allowed; }
.control-row {
  display: grid;
  grid-template-columns: 190px minmax(160px, 1fr) 92px;
  gap: 8px;
  align-items: end;
  margin-bottom: 10px;
}
.current-grid {
  display: grid;
  grid-template-columns: repeat(5, minmax(100px, 1fr));
  gap: 8px;
  margin-top: 12px;
}
.current-overview {
  color: #f4f4f4;
  font-family: Arial, sans-serif;
  font-size: 17px;
  line-height: 1.35;
  font-weight: 600;
  margin: 0 0 12px 0;
}
.data-layout {
  display: grid;
  grid-template-columns: repeat(3, minmax(260px, 1fr));
  gap: 12px;
  align-items: start;
}
.data-col {
  min-width: 0;
}
.data-layout table td:first-child {
  white-space: normal;
}
.data-layout td.value {
  white-space: nowrap;
}
label { font-size: 12px; color: #bbb; display: block; margin-bottom: 4px; }
.sep { border-top: 2px solid #555; padding-top: 12px; margin-top: 14px; }
.help { color: #aaa; font-size: 12px; line-height: 1.35; margin-top: 8px; }
.kv td:first-child { color: #ccc; }
footer {
  position: fixed;
  left: 0;
  right: 0;
  bottom: 0;
  height: var(--footerH);
  z-index: 9999;
  background: #1d1d1d;
  border-top: 0;
  padding: 6px 8px;
  box-sizing: border-box;
  overflow: hidden;
}
.statusline {
  display: flex;
  gap: 16px;
  align-items: center;
  height: 100%;
  overflow-x: auto;
  overflow-y: hidden;
  white-space: nowrap;
  -webkit-overflow-scrolling: touch;
  font-size: 13px;
}
.statusline > span { flex: 0 0 auto; }
.mono { font-family: Consolas, monospace; }
.good { color: #8fd18f; }
.bad { color: #ff8080; }
.warn { color: #ffd37a; }
.muted { color: #999; }

.formgrid {
  display: grid;
  grid-template-columns: 180px minmax(170px, 1fr);
  gap: 8px 10px;
  align-items: center;
}
.formgrid label { color: #bbb; font-size: 12px; }
.formgrid .full { grid-column: 1 / -1; }
.actions {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin-top: 10px;
}
.actions button { width: auto; min-width: 120px; }
.lockbox {
  border: 1px solid #5c4a22;
  background: #211b10;
  border-radius: 6px;
  padding: 10px;
  margin-bottom: 12px;
}
.lockrow {
  display: grid;
  grid-template-columns: minmax(160px, 1fr) 110px;
  gap: 8px;
  align-items: end;
}
.statusnote { color: #ffd37a; font-size: 12px; margin-top: 8px; }

@media (max-width: 1100px) {
  .grid2, .grid3 { grid-template-columns: 1fr; }
  .data-layout { grid-template-columns: repeat(2, minmax(260px, 1fr)); }
  .current-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .control-row { grid-template-columns: 1fr 1fr 90px; }
}
@media (max-width: 900px) {
  .control-layout { grid-template-columns: 1fr; }
}
@media (max-width: 700px) {
  header { padding: 0; }
  .topbar { align-items: flex-start; flex-direction: column; gap: 2px; padding: 7px 8px 5px 8px; }
  .subtitle { white-space: normal; }
  .tabs { padding: 0 8px 7px 8px; }
  .tabbtn { min-width: 108px; padding: 7px 10px; }
  main { padding: 8px 8px 0 8px; }
  th, td { padding: 5px 6px; font-size: 12px; }
  .content { padding: 8px; }
  .control-row { grid-template-columns: 1fr; }
  .data-layout { grid-template-columns: 1fr; }
  .current-grid { grid-template-columns: 1fr; }
}
</style>
</head>
<body>
<header>
  <div class="topbar">
    <h1>Unico-Gateway</h1>
    <div class="subtitle">Modbus RTU → MQTT Gateway | V1.3.2</div>
  </div>
  <nav class="tabs">
    <button id="btn_bedienen" class="tabbtn" onclick="showTab('bedienen')">Bedienen</button>
    <button id="btn_daten" class="tabbtn active" onclick="showTab('daten')">Daten</button>
    <button id="btn_konfiguration" class="tabbtn" onclick="showTab('konfiguration')">Konfiguration</button>
  </nav>
  <div class="top-separator"></div>
</header>

<main>
  <section id="tab_bedienen" class="tabpage">
    <div class="control-layout">
      <div class="section">
        <h2>Bedienen</h2>
        <div class="content">
          <div class="current-overview" id="current_summary">-</div>

          <div id="command_lock" class="lockbox" style="display:none;">
            <div class="lockrow">
              <div><label>Klimabefehle-Passwort</label><input id="cmd_pass" type="password" autocomplete="current-password" placeholder="Passwort"></div>
              <div><label>&nbsp;</label><button onclick="clearCommandPassword()">Leeren</button></div>
            </div>
            <div class="statusnote">Klimabefehle sind geschützt. Alle Senden-Aktionen benötigen dieses Passwort.</div>
          </div>

          <div class="control-row">
            <div><label>Aktion</label><select id="act14" onchange="markActionDirty();"></select></div>
            <div><label>Betriebsart aktuell</label><div id="current_mode" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendAction14()">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Solltemperatur</label><input id="wr8" type="number" min="18" max="30" step="1" value="22" onchange="markSetpointDirty()"></div>
            <div><label>Aktueller Sollwert</label><div id="current_setpoint" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendTemp(8)">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Lüfterstufe</label><select id="wr_fan" onchange="markFanDirty()"></select></div>
            <div><label>Aktueller Lüfter</label><div id="current_fan" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart14('fan')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Silent Mode</label><select id="wr_silent" onchange="markSilentDirty()"></select></div>
            <div><label>Aktuell</label><div id="current_silent" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart14('silent')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Flap Swing</label><select id="wr_swing" onchange="markSwingDirty()"></select></div>
            <div><label>Aktuell</label><div id="current_swing" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart14('swing')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Economy</label><select id="wr_eco" onchange="markEcoDirty()"></select></div>
            <div><label>Aktuell</label><div id="current_eco" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart14('eco')">Senden</button></div>
          </div>

          <div class="sep">
            <div class="control-row">
              <div><label>Raumtemperatur Fernsensor</label><input id="wr5" type="number" min="-10" max="50" step="0.1" value="22.0"></div>
              <div><label>Hinweis</label><div class="muted">wirkt nur, wenn Fernsensor in den Einstellungen aktiv ist</div></div>
              <div><label>&nbsp;</label><button onclick="sendTemp(5)">Senden</button></div>
            </div>
          </div>

          <div class="help">Die Bedienelemente werden aus den Poll-Daten aktualisiert. Aktualisiert wird nur, wenn sich der Zustand der Klimaanlage geändert hat.</div>
        </div>
      </div>

      <!-- UGW_SETTINGS_CONTROL_BLOCK V1.3.2_SETTINGS_PANEL_ACTIVE: schreibbare Register-0012-Einstellungen im Reiter Bedienen -->
      <div class="section">
        <h2>Einstellungen</h2>
        <div class="content">
          <div class="control-row">
            <div><label>Displaybeleuchtung</label><select id="wr12_display" onchange="markPart12Dirty('display')"></select></div>
            <div><label>Aktuell</label><div id="current_12_display" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart12('display')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Wärmepumpe / nur Kühlen</label><select id="wr12_heatpump" onchange="markPart12Dirty('heatpump')"></select></div>
            <div><label>Aktuell</label><div id="current_12_heatpump" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart12('heatpump')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Tastatursperre</label><select id="wr12_keylock" onchange="markPart12Dirty('keylock')"></select></div>
            <div><label>Aktuell</label><div id="current_12_keylock" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart12('keylock')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Decken-/Bodeninstallation</label><select id="wr12_install" onchange="markPart12Dirty('install')"></select></div>
            <div><label>Aktuell</label><div id="current_12_install" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart12('install')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label title="Energy Boost / System Enable Eingang">Logik Eingangseinstellung</label><select id="wr12_inputlogic" onchange="markPart12Dirty('inputlogic')" title="Energy Boost / System Enable Eingang"></select></div>
            <div><label>Aktuell</label><div id="current_12_inputlogic" class="mono" title="Energy Boost / System Enable Eingang">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart12('inputlogic')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Raumtemp. vom Fernsensor</label><select id="wr12_remote" onchange="markPart12Dirty('remote')"></select></div>
            <div><label>Aktuell</label><div id="current_12_remote" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart12('remote')">Senden</button></div>
          </div>

          <div class="control-row">
            <div><label>Celsius / Fahrenheit</label><select id="wr12_unit" onchange="markPart12Dirty('unit')"></select></div>
            <div><label>Aktuell</label><div id="current_12_unit" class="mono">-</div></div>
            <div><label>&nbsp;</label><button onclick="sendPart12('unit')">Senden</button></div>
          </div>

          <div class="help">Diese Werte schreiben Register 0012 per Read-Modify-Write. Unbeteiligte Bits bleiben erhalten.</div>
        </div>
      </div>
    </div>
  </section>

  <section id="tab_daten" class="tabpage active">
    <div class="data-layout">
      <div class="data-col">
        <div class="section">
          <h2>Temperaturen</h2>
          <div class="content">
            <table><tbody>
              <tr><td>Raum</td><td id="t0" class="value">-</td></tr>
              <tr><td>Solltemperatur</td><td id="t8" class="value">-</td></tr>
              <tr><td>WT Innen</td><td id="t1" class="value">-</td></tr>
              <tr><td>WT Außen</td><td id="t2" class="value">-</td></tr>
              <tr><td>Ausblas</td><td id="t3" class="value">-</td></tr>
              <tr><td>Außen</td><td id="t4" class="value">-</td></tr>
              <tr class="gap"><td>Unbekannt</td><td id="t6" class="value">-</td></tr>
            </tbody></table>
          </div>
        </div>

        <div class="section">
          <h2>Messwerte</h2>
          <div class="content">
            <table><tbody>
              <tr><td>Kompressorfrequenz</td><td id="m300" class="value">-</td></tr>
              <tr><td>IDU Lüfter RPM</td><td id="m301" class="value">-</td></tr>
              <tr><td>ODU Lüfter RPM</td><td id="m302" class="value">-</td></tr>
              <tr><td>EEV Ventilöffnung</td><td id="m303" class="value">-</td></tr>
              <tr><td>Fehlercode</td><td id="m304" class="value">-</td></tr>
              <tr><td>Schutzcode</td><td id="m305" class="value">-</td></tr>
              <tr><td>DC-Strom</td><td id="m306" class="value">-</td></tr>
              <tr><td>DC-Spannung</td><td id="m307" class="value">-</td></tr>
              <tr><td>AC-Strom</td><td id="m308" class="value">-</td></tr>
              <tr><td>AC-Spannung</td><td id="m309" class="value">-</td></tr>
            </tbody></table>
          </div>
        </div>
      </div>

      <div class="data-col">
        <div class="section">
          <h2>Sollwerte</h2>
          <div class="content">
            <table><tbody>
              <tr><td>Kühlen Min/Max</td><td id="sw_cool" class="value">-</td></tr>
              <tr><td>Heizen Min/Max</td><td id="sw_heat" class="value">-</td></tr>
            </tbody></table>
          </div>
        </div>

        <div class="section">
          <h2>Zustand</h2>
          <div class="content">
            <table><tbody>
              <tr><td>Kondensat-Wasserventil</td><td id="z10_0" class="value">-</td></tr>
              <tr><td>Kondensat-Wasserpumpe</td><td id="z10_1" class="value">-</td></tr>
              <tr><td>Heat R</td><td id="z10_2" class="value">-</td></tr>
              <tr><td>AUX1</td><td id="z10_3" class="value">-</td></tr>
              <tr><td>AUX2</td><td id="z10_4" class="value">-</td></tr>
              <tr><td>4-Wege-Ventil</td><td id="z10_5" class="value">-</td></tr>
            </tbody></table>
          </div>
        </div>

        <div class="section">
          <h2>Status</h2>
          <div class="content">
            <table><tbody>
              <tr><td>Kondensatwasser-Alarm</td><td id="s11_0" class="value">-</td></tr>
              <tr><td>Pumpen-Wasserstand</td><td id="s11_1" class="value">-</td></tr>
              <tr><td>System Enable / Energy Boost Kontakt</td><td id="s11_2" class="value">-</td></tr>
            </tbody></table>
          </div>
        </div>
      </div>

      <div class="data-col">
        <div class="section">
          <h2>Einstellungen</h2>
          <div class="content">
            <table><tbody>
              <tr><td>Displaybeleuchtung</td><td id="e12_0" class="value">-</td></tr>
              <tr><td>Wärmepumpe / nur Kühlen</td><td id="e12_1" class="value">-</td></tr>
              <tr><td>Tastatursperre</td><td id="e12_2" class="value">-</td></tr>
              <tr><td>Decken-/Bodeninstallation</td><td id="e12_3" class="value">-</td></tr>
              <tr><td title="Energy Boost / System Enable Eingang: je nach Verdrahtung normal offen oder normal geschlossen">Logik Eingangseinstellung</td><td id="e12_4" class="value" title="Energy Boost / System Enable Eingang: normal offen = Kontakt schließt, normal geschlossen = Kontakt öffnet">-</td></tr>
              <tr><td>Raumtemp. vom Fernsensor</td><td id="e12_5" class="value">-</td></tr>
              <tr><td>Celsius / Fahrenheit</td><td id="e12_6" class="value">-</td></tr>
            </tbody></table>
          </div>
        </div>
      </div>
    </div>
  </section>

  <section id="tab_konfiguration" class="tabpage">
    <div id="config_lock" class="lockbox" style="display:none;">
      <h2>Konfiguration geschützt</h2>
      <div class="lockrow">
        <div><label>Passwort</label><input id="cfg_login_pass" type="password" autocomplete="current-password" placeholder="Passwort"></div>
        <div><label>&nbsp;</label><button onclick="loginConfig()">Öffnen</button></div>
      </div>
      <div id="cfg_login_msg" class="statusnote"></div>
    </div>

    <div id="config_forms">
      <div class="grid2">
        <div class="section">
          <h2>MQTT</h2>
          <div class="content">
            <table class="kv"><tbody>
              <tr><td>Verbindung</td><td id="mqtt_conn" class="value">-</td></tr>
              <tr><td>Fehler</td><td id="mqtt_error" class="value">-</td></tr>
              <tr><td>Publishes</td><td id="mqtt_pub" class="value">-</td></tr>
              <tr><td>Reconnect-Versuche</td><td id="mqtt_try" class="value">-</td></tr>
            </tbody></table>
            <div class="formgrid sep">
              <label for="cfg_mqtt_host">Broker</label><input id="cfg_mqtt_host" type="text">
              <label for="cfg_mqtt_port">Port</label><input id="cfg_mqtt_port" type="number" min="1" max="65535" step="1">
              <label for="cfg_mqtt_clientid">Client-ID</label><input id="cfg_mqtt_clientid" type="text">
              <label for="cfg_mqtt_root">Topic Prefix</label><input id="cfg_mqtt_root" type="text">
              <label for="cfg_mqtt_interval">MQTT Intervall</label><input id="cfg_mqtt_interval" type="number" min="5" max="900" step="1">
              <label for="cfg_mqtt_user">Benutzer</label><input id="cfg_mqtt_user" type="text" autocomplete="username">
              <label for="cfg_mqtt_pass">Passwort</label><input id="cfg_mqtt_pass" type="password" autocomplete="new-password" placeholder="leer = unverändert">
              <label for="cfg_mqtt_commands">MQTT-Befehle</label><select id="cfg_mqtt_commands"><option value="0">nicht aktiv</option><option value="1">aktiv</option></select>
            </div>
          </div>
        </div>

        <div class="section">
          <h2>Modbus / RS485</h2>
          <div class="content">
            <table class="kv"><tbody>
              <tr><td>Slave-ID</td><td id="mb_slave" class="value">-</td></tr>
              <tr><td>Baudrate</td><td id="mb_baud" class="value">-</td></tr>
              <tr><td>Datenbits</td><td id="mb_bits" class="value">-</td></tr>
              <tr><td>Parität</td><td id="mb_parity" class="value">-</td></tr>
              <tr><td>Stoppbits</td><td id="mb_stop" class="value">-</td></tr>
              <tr><td>Pollintervall</td><td id="mb_poll" class="value">-</td></tr>
              <tr><td>Timeout</td><td id="mb_timeout" class="value">-</td></tr>
            </tbody></table>
            <div class="formgrid sep">
              <label for="cfg_mb_slave">Slave-ID</label><input id="cfg_mb_slave" type="number" min="1" max="247" step="1">
              <label for="cfg_mb_baud">Baudrate</label><select id="cfg_mb_baud"><option>1200</option><option>2400</option><option>4800</option><option selected>9600</option><option>19200</option><option>38400</option><option>57600</option><option>115200</option></select>
              <label for="cfg_mb_bits">Datenbits</label><select id="cfg_mb_bits"><option>7</option><option selected>8</option></select>
              <label for="cfg_mb_parity">Parität</label><select id="cfg_mb_parity"><option value="N">Keine</option><option value="E">Gerade</option><option value="O">Ungerade</option></select>
              <label for="cfg_mb_stop">Stoppbits</label><select id="cfg_mb_stop"><option>1</option><option>2</option></select>
              <label for="cfg_mb_poll">Pollintervall ms</label><input id="cfg_mb_poll" type="number" min="500" max="60000" step="100">
              <label for="cfg_mb_timeout">Timeout ms</label><input id="cfg_mb_timeout" type="number" min="100" max="5000" step="50">
            </div>
            <div class="help">Modbus-/RS485-Änderungen werden gespeichert und sofort für neue Telegramme verwendet.</div>
          </div>
        </div>
      </div>

      <div class="grid2">
        <div class="section">
          <h2>WLAN / Access Point</h2>
          <div class="content">
            <table class="kv"><tbody>
              <tr><td>Betrieb</td><td id="cfg_wifi_mode" class="value">-</td></tr>
              <tr><td>IP-Adresse</td><td id="cfg_ip" class="value">-</td></tr>
              <tr><td>Signal</td><td id="cfg_rssi" class="value">-</td></tr>
            </tbody></table>
            <div class="formgrid sep">
              <label for="cfg_wifi_ssid">WLAN SSID</label><input id="cfg_wifi_ssid" type="text" autocomplete="username">
              <label for="cfg_wifi_pass">WLAN Passwort</label><input id="cfg_wifi_pass" type="password" autocomplete="new-password" placeholder="leer = unverändert">
              <label for="cfg_ap_ssid">AP Name</label><input id="cfg_ap_ssid" type="text">
              <label for="cfg_ap_pass">AP Passwort</label><input id="cfg_ap_pass" type="password" autocomplete="new-password" placeholder="leer = unverändert, min. 8 Zeichen">
            </div>
            <div class="help">WLAN- und AP-Änderungen werden gespeichert und nach Neustart aktiv. Leeres Passwortfeld bedeutet: vorhandenes Passwort behalten.</div>
          </div>
        </div>

        <div class="section">
          <h2>Sicherheit</h2>
          <div class="content">
            <table class="kv"><tbody>
              <tr><td>Konfigurationspasswort</td><td id="sec_pass_set" class="value">-</td></tr>
              <tr><td>Klimabefehle geschützt</td><td id="sec_climate" class="value">-</td></tr>
            </tbody></table>
            <div class="formgrid sep">
              <label for="cfg_web_commands">Web-Bedienung erlauben</label><select id="cfg_web_commands"><option value="0">Aus</option><option value="1">Ein</option></select>
              <label for="cfg_climate_protected">Web-Bedienung schützen</label><select id="cfg_climate_protected"><option value="0">Aus</option><option value="1">Ein</option></select>
            </div>
            <div class="help">Klimabefehl = alles, was schreibt: Ein/Aus, Modus, Solltemperatur, Lüfter, Silent, Swing, Economy, Fernsensor. MQTT-Befehle werden getrennt im MQTT-Block aktiviert.</div>

            <div class="formgrid sep">
              <label id="pwd_old_label" for="pwd_old">Altes Passwort</label><input id="pwd_old" type="password" autocomplete="current-password" placeholder="nur wenn bereits gesetzt">
              <label for="pwd_new1">Neues Passwort</label><input id="pwd_new1" type="password" autocomplete="new-password">
              <label for="pwd_new2">Neues Passwort wiederholen</label><input id="pwd_new2" type="password" autocomplete="new-password">
            </div>
            <div class="actions">
              <button onclick="changeConfigPassword()">Passwort speichern</button>
              <button onclick="clearConfigPassword()">Passwortschutz entfernen</button>
            </div>
            <div id="sec_msg" class="statusnote"></div>
          </div>
        </div>
      </div>

      <div class="section">
        <h2>Speichern / Neustart</h2>
        <div class="content">
          <div class="actions">
            <button onclick="saveConfig()">Konfiguration speichern</button>
            <button onclick="restartDevice()">Neustart</button>
            <button onclick="resetConfig()">Konfiguration zurücksetzen</button>
          </div>
          <div id="cfg_save_msg" class="statusnote"></div>
        </div>
      </div>
    </div>
  </section>
</main>

<div class="footer-separator"></div>
<footer>
  <div class="statusline">
    <span id="st_version" class="mono">V1.3.2</span>
    <span>Modbus: <span id="st_modbus" class="mono">-</span></span>
    <span>MQTT: <span id="st_mqtt" class="mono">-</span></span>
    <span>WLAN: <span id="st_wifi" class="mono">-</span></span>
    <span>OK: <span id="st_ok" class="mono">0</span></span>
    <span>Timeout: <span id="st_timeout" class="mono">0</span></span>
  </div>
</footer>

<script>
const actions = [
  {n:"Aus / Standby", v:0x0000, m:0x00E0},
  {n:"Kühlen", v:0x0020, m:0x00E0},
  {n:"Heizen", v:0x0040, m:0x00E0},
  {n:"Entfeuchten", v:0x0060, m:0x00E0},
  {n:"Lüften", v:0x0080, m:0x00E0},
  {n:"Automatik", v:0x00A0, m:0x00E0},
  {n:"Kühlen + Lüfter Auto", v:0x0038, m:0x00F8},
  {n:"Heizen + Lüfter Auto", v:0x0058, m:0x00F8},
  {n:"Entfeuchten + Lüfter Auto", v:0x0078, m:0x00F8},
  {n:"Lüften + Lüfter Auto", v:0x0098, m:0x00F8}
];
const modes = ["Standby/Aus", "Kühlen", "Heizen", "Entfeuchten", "Lüften", "Automatik", "6", "7"];
const fans = ["Niedrig", "Mittel", "Hoch", "Automatik"];
const onoff = ["aus", "ein"];
const setting12Options = {
  display: ["aus", "ein"],
  heatpump: ["nur Kühlen", "Wärmepumpe"],
  keylock: ["freigegeben", "gesperrt"],
  install: ["Boden", "Decke"],
  inputlogic: ["normal geschlossen", "normal offen"],
  remote: ["aus", "ein"],
  unit: ["Celsius", "Fahrenheit"]
};
let dirty12 = {};
let lastControlSig = "";
let dirtyAction = false;
let dirtySetpoint = false;
let dirtyFan = false;
let dirtySilent = false;
let dirtySwing = false;
let dirtyEco = false;
let configPasswordSet = false;
let climateProtected = false;

function $(id){ return document.getElementById(id); }
function hasReg(d,a){ return d.regs && Object.prototype.hasOwnProperty.call(d.regs, String(a)); }
function reg(d,a){ return hasReg(d,a) ? d.regs[String(a)] : null; }
function signed16(v){ v &= 0xFFFF; return (v & 0x8000) ? v - 0x10000 : v; }
function bit(v,b){ return (v >> b) & 1; }
function temp1(d,a){ const v=reg(d,a); return v===null ? "-" : (signed16(v)/10).toFixed(1) + " °C"; }
function temp0(d,a){ const v=reg(d,a); return v===null ? "-" : Math.round(signed16(v)/10) + " °C"; }
function num(d,a){ const v=reg(d,a); return v===null ? "-" : String(signed16(v)); }
function setCls(id, ok){ const e=$(id); e.className = "mono " + (ok ? "good" : "bad"); }
function setOptions(id, arr){ const s=$(id); if(!s) return; s.innerHTML=""; arr.forEach((x,i)=>{ const o=document.createElement("option"); o.value=i; o.textContent=x; s.appendChild(o); }); }
function bitCount(x){ let c=0; while(x){ c += x & 1; x >>= 1; } return c; }
function val(id){ const e=$(id); return e ? e.value : ""; }
function setVal(id,v){ const e=$(id); if(e) e.value = v == null ? "" : v; }
function setText(id,v){ const e=$(id); if(e) e.textContent = v == null ? "-" : v; }
function postForm(url, obj){
  const body = new URLSearchParams();
  Object.keys(obj).forEach(k => body.append(k, obj[k]));
  return fetch(url, {method:"POST", headers:{"Content-Type":"application/x-www-form-urlencoded"}, body});
}

function showTab(name){
  ["bedienen","daten","konfiguration"].forEach(t=>{
    const page = $("tab_"+t);
    const btn = $("btn_"+t);
    if(page) page.classList.toggle("active", t===name);
    if(btn) btn.classList.toggle("active", t===name);
  });
}

function markActionDirty(){ dirtyAction = true; }
function markSetpointDirty(){ dirtySetpoint = true; }
function markFanDirty(){ dirtyFan = true; }
function markSilentDirty(){ dirtySilent = true; }
function markSwingDirty(){ dirtySwing = true; }
function markEcoDirty(){ dirtyEco = true; }
function markPart12Dirty(part){ dirty12[part] = true; }

function initControls(){
  const a=$("act14");
  actions.forEach((x,i)=>{ const o=document.createElement("option"); o.value=i; o.textContent=x.n; a.appendChild(o); });
  setOptions("wr_fan", fans);
  setOptions("wr_silent", onoff);
  setOptions("wr_swing", onoff);
  setOptions("wr_eco", onoff);
  setOptions("wr12_display", setting12Options.display);
  setOptions("wr12_heatpump", setting12Options.heatpump);
  setOptions("wr12_keylock", setting12Options.keylock);
  setOptions("wr12_install", setting12Options.install);
  setOptions("wr12_inputlogic", setting12Options.inputlogic);
  setOptions("wr12_remote", setting12Options.remote);
  setOptions("wr12_unit", setting12Options.unit);
  setCurrentSummaryFrom14(null);
}

function actionIndexForValue(v){
  let best = 0;
  let bestBits = -1;
  actions.forEach((a,i)=>{
    if((v & a.m) === (a.v & a.m)){
      const b = bitCount(a.m);
      if(b > bestBits){ bestBits = b; best = i; }
    }
  });
  return best;
}

function capState(x){ return x === "ein" ? "Ein" : "Aus"; }
function statusSummaryFrom14(r14){
  if(r14 === null) return "-";
  const modeVal = (r14 >> 5) & 7;
  const fanVal = (r14 >> 3) & 3;
  const silentVal = r14 & 1;
  const swingVal = (r14 >> 1) & 1;
  const ecoVal = (r14 >> 8) & 1;
  const fanTxt = modeVal === 0 ? "Aus" : fans[fanVal];
  return modes[modeVal] + " | Lüfter " + fanTxt + " | Silent " + capState(onoff[silentVal]) + " | Swing " + capState(onoff[swingVal]) + " | ECO " + capState(onoff[ecoVal]);
}
function setCurrentSummaryFrom14(r14){
  const txt = statusSummaryFrom14(r14);
  if($("current_summary")) $("current_summary").textContent = txt;
}
function decodeAction(){
  // Keine Vorschau der Aktionswirkung mehr. Angezeigt wird nur der Istzustand aus Poll-Daten.
}

function updateControlsFromDevice(d){
  const r14 = reg(d,14);
  const r8 = reg(d,8);
  const sig = String(r14 === null ? "-" : r14) + "|" + String(r8 === null ? "-" : r8);
  if(sig === lastControlSig) return;
  lastControlSig = sig;

  if(r14 !== null){
    const modeVal = (r14 >> 5) & 7;
    const fanVal = (r14 >> 3) & 3;
    const silentVal = r14 & 1;
    const swingVal = (r14 >> 1) & 1;
    const ecoVal = (r14 >> 8) & 1;
    if($("current_mode")) $("current_mode").textContent = modes[modeVal];
    $("current_fan").textContent = fans[fanVal];
    $("current_silent").textContent = onoff[silentVal];
    $("current_swing").textContent = onoff[swingVal];
    $("current_eco").textContent = onoff[ecoVal];
    setCurrentSummaryFrom14(r14);
    if(!dirtyFan) $("wr_fan").value = fanVal;
    if(!dirtySilent) $("wr_silent").value = silentVal;
    if(!dirtySwing) $("wr_swing").value = swingVal;
    if(!dirtyEco) $("wr_eco").value = ecoVal;
    if(!dirtyAction){
      $("act14").value = actionIndexForValue(r14);
    }
  }
  if(r8 !== null){
    const setp = Math.round(signed16(r8)/10);
    $("current_setpoint").textContent = setp + " °C";
    if(!dirtySetpoint) $("wr8").value = setp;
  }
}

async function sendTemp(addr){
  const input = $(addr === 5 ? "wr5" : "wr8");
  const value = input.value;
  try {
    let url = `/api/write?type=temp&addr=${addr}&value=${encodeURIComponent(value)}`;
    if(climateProtected) url += `&pass=${encodeURIComponent(val("cmd_pass"))}`;
    const r = await fetch(url);
    const t = await r.text();
    if(!r.ok) alert(t);
    else if(addr === 8) dirtySetpoint = false;
  } catch(e) { alert("Senden fehlgeschlagen"); }
}

async function sendAction14(){
  const idx = $("act14").value;
  try {
    let url = `/api/write?type=action14&idx=${idx}`;
    if(climateProtected) url += `&pass=${encodeURIComponent(val("cmd_pass"))}`;
    const r = await fetch(url);
    const t = await r.text();
    if(!r.ok) alert(t);
    else dirtyAction = false;
  } catch(e) { alert("Senden fehlgeschlagen"); }
}

async function sendPart14(part){
  const id = part === "fan" ? "wr_fan" : part === "silent" ? "wr_silent" : part === "swing" ? "wr_swing" : "wr_eco";
  const value = val(id);
  try {
    let url = `/api/write?type=part14&part=${encodeURIComponent(part)}&value=${encodeURIComponent(value)}`;
    if(climateProtected) url += `&pass=${encodeURIComponent(val("cmd_pass"))}`;
    const r = await fetch(url);
    const t = await r.text();
    if(!r.ok) alert(t);
    else {
      if(part === "fan") dirtyFan = false;
      else if(part === "silent") dirtySilent = false;
      else if(part === "swing") dirtySwing = false;
      else if(part === "eco") dirtyEco = false;
    }
  } catch(e) { alert("Senden fehlgeschlagen"); }
}

async function sendPart12(part){
  const id = "wr12_" + part;
  const value = val(id);
  try {
    let url = `/api/write?type=part12&part=${encodeURIComponent(part)}&value=${encodeURIComponent(value)}`;
    if(climateProtected) url += `&pass=${encodeURIComponent(val("cmd_pass"))}`;
    const r = await fetch(url);
    const t = await r.text();
    if(!r.ok) alert(t);
    else dirty12[part] = false;
  } catch(e) { alert("Senden fehlgeschlagen"); }
}

function clearCommandPassword(){ setVal("cmd_pass", ""); }

function updateBits(d){
  const r10 = reg(d,10);
  if(r10 !== null){
    for(let i=0;i<=5;i++) $("z10_"+i).textContent = bit(r10,i) ? "EIN" : "AUS";
  }
  const r11 = reg(d,11);
  if(r11 !== null){
    $("s11_0").textContent = bit(r11,0) ? "kein Alarm" : "0";
    $("s11_1").textContent = bit(r11,1) ? "1" : "aktiv";
    $("s11_2").textContent = bit(r11,2) ? "Kontakt geschlossen" : "0";
  }
  const r12 = reg(d,12);
  if(r12 !== null){
    const s12 = {
      display: bit(r12,0),
      heatpump: bit(r12,1),
      keylock: bit(r12,2),
      install: bit(r12,3),
      inputlogic: bit(r12,4),
      remote: bit(r12,5),
      unit: bit(r12,6)
    };
    $("e12_0").textContent = setting12Options.display[s12.display];
    $("e12_1").textContent = setting12Options.heatpump[s12.heatpump];
    $("e12_2").textContent = setting12Options.keylock[s12.keylock];
    $("e12_3").textContent = setting12Options.install[s12.install];
    $("e12_4").textContent = setting12Options.inputlogic[s12.inputlogic];
    $("e12_4").title = s12.inputlogic ? "Energy Boost / System Enable: Kontakt schließt" : "Energy Boost / System Enable: Kontakt öffnet";
    $("e12_5").textContent = setting12Options.remote[s12.remote];
    $("e12_6").textContent = setting12Options.unit[s12.unit];

    Object.keys(s12).forEach(part => {
      setText("current_12_" + part, setting12Options[part][s12[part]]);
      if(!dirty12[part]) setVal("wr12_" + part, s12[part]);
    });
    const il = $("current_12_inputlogic");
    if(il) il.title = s12.inputlogic ? "Energy Boost / System Enable: Kontakt schließt" : "Energy Boost / System Enable: Kontakt öffnet";
  }
}

function updateValues(d){
  $("t0").textContent = temp1(d,0);
  $("t8").textContent = temp0(d,8);
  $("t1").textContent = temp1(d,1);
  $("t2").textContent = temp1(d,2);
  $("t3").textContent = temp1(d,3);
  $("t4").textContent = temp1(d,4);
  $("t6").textContent = temp1(d,6);

  const cMinV = reg(d,46), cMaxV = reg(d,48), hMinV = reg(d,47), hMaxV = reg(d,49);
  $("sw_cool").textContent = (cMinV === null || cMaxV === null) ? "-" : signed16(cMinV) + " / " + signed16(cMaxV) + " °C";
  $("sw_heat").textContent = (hMinV === null || hMaxV === null) ? "-" : signed16(hMinV) + " / " + signed16(hMaxV) + " °C";

  for(let a=300; a<=309; a++){
    const v = reg(d,a);
    let txt = "-";
    if(v !== null){
      if(a===306 || a===308) txt = (signed16(v)/10).toFixed(1) + " A";
      else if(a===301 || a===302) txt = signed16(v) + " rpm";
      else if(a===307 || a===309) txt = signed16(v) + " V";
      else txt = String(signed16(v));
    }
    $("m"+a).textContent = txt;
  }
}

function updateMeta(d){
  $("st_version").textContent = d.version;
  $("mqtt_conn").textContent = d.mqttConnected ? "verbunden" : "getrennt";
  setText("mqtt_error", d.mqttError);
  $("mqtt_pub").textContent = d.mqttPublishes;
  $("mqtt_try").textContent = d.mqttReconnectAttempts;
  $("mb_slave").textContent = d.modbusSlave;
  $("mb_baud").textContent = d.modbusBaud;
  $("mb_bits").textContent = d.modbusBits;
  $("mb_parity").textContent = d.modbusParity;
  $("mb_stop").textContent = d.modbusStop;
  $("mb_poll").textContent = d.pollInterval + " ms";
  $("mb_timeout").textContent = d.modbusTimeout + " ms";
  $("cfg_wifi_mode").textContent = d.wifiMode;
  $("cfg_ip").textContent = d.ip;
  $("cfg_rssi").textContent = d.rssi + " dBm";

  $("st_modbus").textContent = d.status;
  $("st_mqtt").textContent = d.mqttConnected ? "OK" : d.mqttError;
  $("st_wifi").textContent = (d.wifiMode === "AP" ? "AP " : "IP ") + d.ip;
  $("st_ok").textContent = d.okPolls;
  $("st_timeout").textContent = d.timeoutErrors;
  setCls("st_mqtt", d.mqttConnected);
  climateProtected = !!d.climateProtected;
  configPasswordSet = !!d.configPasswordSet;
  const cl = $("command_lock");
  if(cl) cl.style.display = climateProtected ? "block" : "none";
}

async function loadConfig(){
  try{
    const r = await fetch("/api/config");
    if(r.status === 403){
      $("config_lock").style.display = "block";
      $("config_forms").style.display = "none";
      return;
    }
    const c = await r.json();
    $("config_lock").style.display = "none";
    $("config_forms").style.display = "block";
    configPasswordSet = !!c.configPasswordSet;
    climateProtected = !!c.climateProtected;
    setVal("cfg_wifi_ssid", c.wifiSsid);
    setVal("cfg_ap_ssid", c.apSsid);
    setVal("cfg_mqtt_host", c.mqttHost);
    setVal("cfg_mqtt_port", c.mqttPort);
    setVal("cfg_mqtt_user", c.mqttUser);
    setVal("cfg_mqtt_clientid", c.mqttClientId);
    setVal("cfg_mqtt_root", c.mqttRoot);
    setVal("cfg_mqtt_interval", c.mqttIntervalSec);
    setVal("cfg_mqtt_commands", c.mqttCommandsEnabled ? "1" : "0");
    setVal("cfg_mb_slave", c.modbusSlave);
    setVal("cfg_mb_baud", c.modbusBaud);
    setVal("cfg_mb_bits", c.modbusBits);
    setVal("cfg_mb_parity", c.modbusParityCode);
    setVal("cfg_mb_stop", c.modbusStop);
    setVal("cfg_mb_poll", c.modbusPollIntervalMs);
    setVal("cfg_mb_timeout", c.modbusTimeoutMs);
    setVal("cfg_web_commands", c.webCommandsEnabled ? "1" : "0");
    setVal("cfg_climate_protected", c.climateProtected ? "1" : "0");
    setText("sec_pass_set", c.configPasswordSet ? "gesetzt" : "nicht gesetzt");
    setText("sec_climate", c.climateProtected ? "Ein" : "Aus");
    $("pwd_old").style.display = c.configPasswordSet ? "block" : "none";
    $("pwd_old_label").style.display = c.configPasswordSet ? "block" : "none";
  } catch(e){ setText("cfg_save_msg", "Konfiguration konnte nicht gelesen werden"); }
}

async function loginConfig(){
  try{
    const r = await postForm("/api/config_login", {pass: val("cfg_login_pass")});
    const t = await r.text();
    if(!r.ok){ setText("cfg_login_msg", t); return; }
    setText("cfg_login_msg", "OK");
    await loadConfig();
  } catch(e){ setText("cfg_login_msg", "Login fehlgeschlagen"); }
}

async function saveConfig(){
  try{
    const r = await postForm("/api/config_save", {
      wifi_ssid: val("cfg_wifi_ssid"),
      wifi_pass: val("cfg_wifi_pass"),
      ap_ssid: val("cfg_ap_ssid"),
      ap_pass: val("cfg_ap_pass"),
      mqtt_host: val("cfg_mqtt_host"),
      mqtt_port: val("cfg_mqtt_port"),
      mqtt_user: val("cfg_mqtt_user"),
      mqtt_pass: val("cfg_mqtt_pass"),
      mqtt_clientid: val("cfg_mqtt_clientid"),
      mqtt_root: val("cfg_mqtt_root"),
      mqtt_interval: val("cfg_mqtt_interval"),
      mqtt_commands: val("cfg_mqtt_commands"),
      mb_slave: val("cfg_mb_slave"),
      mb_baud: val("cfg_mb_baud"),
      mb_bits: val("cfg_mb_bits"),
      mb_parity: val("cfg_mb_parity"),
      mb_stop: val("cfg_mb_stop"),
      mb_poll: val("cfg_mb_poll"),
      mb_timeout: val("cfg_mb_timeout"),
      web_commands: val("cfg_web_commands"),
      climate_protected: val("cfg_climate_protected")
    });
    const t = await r.text();
    setText("cfg_save_msg", t);
    if(r.ok){
      setVal("cfg_wifi_pass", ""); setVal("cfg_ap_pass", ""); setVal("cfg_mqtt_pass", "");
      await loadConfig();
    }
  } catch(e){ setText("cfg_save_msg", "Speichern fehlgeschlagen"); }
}

async function changeConfigPassword(){
  try{
    const r = await postForm("/api/password", {old: val("pwd_old"), new1: val("pwd_new1"), new2: val("pwd_new2")});
    const t = await r.text();
    setText("sec_msg", t);
    if(r.ok){ setVal("pwd_old", ""); setVal("pwd_new1", ""); setVal("pwd_new2", ""); await loadConfig(); }
  } catch(e){ setText("sec_msg", "Passwort konnte nicht gespeichert werden"); }
}

async function clearConfigPassword(){
  try{
    const r = await postForm("/api/password_clear", {old: val("pwd_old")});
    const t = await r.text();
    setText("sec_msg", t);
    if(r.ok){ setVal("pwd_old", ""); setVal("pwd_new1", ""); setVal("pwd_new2", ""); await loadConfig(); }
  } catch(e){ setText("sec_msg", "Passwortschutz konnte nicht entfernt werden"); }
}

async function restartDevice(){
  if(!confirm("ESP32 wirklich neu starten?")) return;
  try{
    const r = await postForm("/api/restart", {});
    const t = await r.text();
    setText("cfg_save_msg", t);
  } catch(e){ setText("cfg_save_msg", "Neustart-Befehl fehlgeschlagen"); }
}

async function resetConfig(){
  if(!confirm("Gespeicherte Web-Konfiguration wirklich löschen und auf UGW_secrets.h zurückfallen?")) return;
  try{
    const r = await postForm("/api/config_reset", {});
    const t = await r.text();
    setText("cfg_save_msg", t);
  } catch(e){ setText("cfg_save_msg", "Reset-Befehl fehlgeschlagen"); }
}

async function loadState(){
  try{
    const r = await fetch("/api/state");
    const d = await r.json();
    updateValues(d);
    updateBits(d);
    updateControlsFromDevice(d);
    updateMeta(d);
  } catch(e){
    $("st_modbus").textContent = "API Fehler";
  }
}

showTab('daten');
initControls();
loadState();
loadConfig();
setInterval(loadState, 2000);
</script>
</body>
</html>
)rawliteral";

// =====================================================
// Modbus CRC / Hex
// =====================================================
uint16_t crc16_modbus(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

String bytesToHex(const uint8_t* data, uint16_t len) {
  String s;
  for (uint16_t i = 0; i < len; i++) {
    if (i) s += ' ';
    if (data[i] < 16) s += '0';
    s += String(data[i], HEX);
  }
  s.toUpperCase();
  return s;
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (uint16_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

int16_t signed16(uint16_t v) {
  return (v & 0x8000) ? (int16_t)(v - 0x10000) : (int16_t)v;
}

String oneDecimalFromRaw10(uint16_t raw) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", signed16(raw) / 10.0f);
  return String(buf);
}


String currentIpText() {
  return (wifiModeText == "AP") ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
}

String mqttStateText(int st) {
  switch (st) {
    case 0: return "OK";
    case -4: return "Timeout";
    case -3: return "Verbindung verloren";
    case -2: return "Connect fehlgeschlagen";
    case -1: return "Getrennt";
    case 1: return "Falsche MQTT-Protokollversion";
    case 2: return "Client-ID abgelehnt";
    case 3: return "Broker nicht verfuegbar";
    case 4: return "Benutzer/Passwort falsch";
    case 5: return "Nicht autorisiert";
    default: return String("MQTT Status ") + String(st);
  }
}

// Liefert den bewusst bereinigten Status fuer Weboberflaeche und MQTT.
// Interne Modbus-Fehler bleiben in lastStatus erhalten, aber technische Write-/Telegrammtexte
// sollen in der normalen Gateway-Ansicht nicht erscheinen. Die Oberflaeche bleibt damit
// eine Bedien- und Datenansicht und kein Modbus-Diagnoselog.
String publicStatusText() {
  if (lastStatus.startsWith("Write ")) return "Befehl Fehler";
  if (lastStatus.startsWith("WR ")) return "Befehl";
  if (lastStatus.indexOf("TX=") >= 0 || lastStatus.indexOf("RX=") >= 0) return "Befehl";
  return lastStatus;
}

// Liefert den Text fuer die aktuell gewaehlte Paritaet.
// Dieser Text wird nur fuer Weboberflaeche und MQTT-Konfigurationswerte verwendet.
String modbusParityText() {
  if (cfgModbusParity == 'E') return "Even";
  if (cfgModbusParity == 'O') return "Odd";
  return "None";
}

// ESP32-HardwareSerial erwartet eine Konstante wie SERIAL_8N1.
// Diese Funktion baut daraus die passende serielle Konfiguration fuer die GUI-Werte.
uint32_t serialConfigFromSettings() {
  bool bits7 = (cfgModbusDataBits == 7);
  bool stop2 = (cfgModbusStopBits == 2);
  if (cfgModbusParity == 'E') {
    if (bits7) return stop2 ? SERIAL_7E2 : SERIAL_7E1;
    return stop2 ? SERIAL_8E2 : SERIAL_8E1;
  }
  if (cfgModbusParity == 'O') {
    if (bits7) return stop2 ? SERIAL_7O2 : SERIAL_7O1;
    return stop2 ? SERIAL_8O2 : SERIAL_8O1;
  }
  if (bits7) return stop2 ? SERIAL_7N2 : SERIAL_7N1;
  return stop2 ? SERIAL_8N2 : SERIAL_8N1;
}

// Begrenzung der Konfigurationswerte aus NVS/Webinterface.
// Damit bleibt auch nach einer falschen Eingabe ein sinnvoller Modbus-Startzustand erhalten.
void normalizeRuntimeConfig() {
  if (cfgMqttPublishIntervalSec < 5) cfgMqttPublishIntervalSec = 5;
  if (cfgMqttPublishIntervalSec > 900) cfgMqttPublishIntervalSec = 900;

  if (cfgModbusSlaveId < 1 || cfgModbusSlaveId > 247) cfgModbusSlaveId = 1;
  if (cfgModbusBaudrate < 1200 || cfgModbusBaudrate > 115200) cfgModbusBaudrate = 9600;
  if (cfgModbusDataBits != 7 && cfgModbusDataBits != 8) cfgModbusDataBits = 8;
  if (cfgModbusParity != 'N' && cfgModbusParity != 'E' && cfgModbusParity != 'O') cfgModbusParity = 'N';
  if (cfgModbusStopBits != 1 && cfgModbusStopBits != 2) cfgModbusStopBits = 1;
  if (cfgModbusTimeoutMs < 100 || cfgModbusTimeoutMs > 5000) cfgModbusTimeoutMs = 700;
  if (cfgModbusPollIntervalMs < 500 || cfgModbusPollIntervalMs > 60000) cfgModbusPollIntervalMs = 2000;
}

// Wendet geaenderte Modbus-/RS485-Parameter auf Serial2 an.
// Nach dem Speichern im Webinterface gelten neue Werte damit ohne Neu-Flash.
void applyModbusSerial() {
  Serial2.end();
  delay(30);
  Serial2.begin(cfgModbusBaudrate, serialConfigFromSettings(), MODBUS_RX_PIN, MODBUS_TX_PIN);
  clearSerial2();
}

bool hasConfigPassword() {
  return cfgConfigPassword.length() > 0;
}

String makeSessionToken() {
  return String((uint32_t)esp_random(), HEX) + String((uint32_t)millis(), HEX);
}

bool cookieAuthorized() {
  if (!hasConfigPassword()) return true;
  if (configSessionToken.length() == 0) return false;
  String c = server.header("Cookie");
  return c.indexOf(String("ugw_auth=") + configSessionToken) >= 0;
}

bool requireConfigAuth() {
  if (cookieAuthorized()) return true;
  sendNoCache();
  server.send(403, "text/plain; charset=utf-8", "Konfiguration gesperrt");
  return false;
}

// Prueft, ob ein schreibender Klimabefehl ausgefuehrt werden darf.
// Wenn der Bedienbereich geschuetzt ist, muss die HTTP-API das Konfigurationspasswort
// als Parameter pass=... mitliefern. Ohne Schutz bleibt die Bedienung bewusst frei.
bool climateCommandAuthorized() {
  if (!cfgClimateProtected || !hasConfigPassword()) return true;
  if (!server.hasArg("pass")) return false;
  return server.arg("pass") == cfgConfigPassword;
}

// Laedt die dauerhaft gespeicherte Laufzeitkonfiguration aus Preferences/NVS.
// Falls ein Wert dort fehlt, wird der Flash-Default aus UGW_secrets.h genutzt.
// Dadurch funktionieren frisch geflashte Geraete sofort, bleiben aber per Webinterface anpassbar.
void loadConfig() {
  prefs.begin("ugw", false);
  cfgWifiSsid = prefs.getString("wifi_ssid", UGW_WIFI_SSID);
  cfgWifiPass = prefs.getString("wifi_pass", UGW_WIFI_PASSWORD);
  cfgApSsid = prefs.getString("ap_ssid", UGW_AP_SSID);
  cfgApPass = prefs.getString("ap_pass", UGW_AP_PASSWORD);
  cfgMqttHost = prefs.getString("mqtt_host", UGW_MQTT_HOST);
  cfgMqttPort = prefs.getUShort("mqtt_port", UGW_MQTT_PORT);
  cfgMqttUser = prefs.getString("mqtt_user", UGW_MQTT_USER);
  cfgMqttPass = prefs.getString("mqtt_pass", UGW_MQTT_PASSWORD);
  cfgMqttClientId = prefs.getString("mqtt_cid", UGW_MQTT_CLIENTID);
  cfgMqttRoot = prefs.getString("mqtt_root", UGW_MQTT_ROOT);
  cfgConfigPassword = prefs.getString("cfg_pass", "");
  cfgClimateProtected = prefs.getBool("clim_prot", false);
  cfgWebCommandsEnabled = prefs.getBool("web_cmd", true);
  cfgMqttCommandsEnabled = prefs.getBool("mqtt_cmd", false);
  cfgMqttPublishIntervalSec = prefs.getUShort("mqtt_int", 10);

  cfgModbusSlaveId = prefs.getUChar("mb_slave", 1);
  cfgModbusBaudrate = prefs.getUInt("mb_baud", 9600);
  cfgModbusDataBits = prefs.getUChar("mb_bits", 8);
  cfgModbusParity = (char)prefs.getUChar("mb_parity", 'N');
  cfgModbusStopBits = prefs.getUChar("mb_stop", 1);
  cfgModbusTimeoutMs = prefs.getUShort("mb_timeout", 700);
  cfgModbusPollIntervalMs = prefs.getUShort("mb_poll", 2000);

  if (cfgApSsid.length() == 0) cfgApSsid = UGW_AP_SSID;
  if (cfgApPass.length() > 0 && cfgApPass.length() < 8) cfgApPass = UGW_AP_PASSWORD;
  if (cfgMqttHost.length() == 0) cfgMqttHost = UGW_MQTT_HOST;
  if (cfgMqttClientId.length() == 0) cfgMqttClientId = UGW_MQTT_CLIENTID;
  if (cfgMqttRoot.length() == 0) cfgMqttRoot = UGW_MQTT_ROOT;
  if (!hasConfigPassword()) cfgClimateProtected = false;
  normalizeRuntimeConfig();
}

void saveStringPref(const char* key, const String& value) {
  prefs.putString(key, value);
}


void clearSerial2() {
  while (Serial2.available()) Serial2.read();
}

// Sendet ein komplettes Modbus-RTU-Frame ueber den RS485-Transceiver.
// MODBUS_DIR_PIN schaltet DE/RE: vor dem Senden auf TX, danach wieder auf RX.
// Die kleinen Pausen vermeiden, dass der Transceiver oder die UNICO-Platine Frames abschneidet.
void sendFrame(const uint8_t* request, uint8_t len) {
  clearSerial2();
  digitalWrite(MODBUS_DIR_PIN, HIGH);
  delayMicroseconds(250);
  Serial2.write(request, len);
  Serial2.flush();
  delayMicroseconds(1200);
  digitalWrite(MODBUS_DIR_PIN, LOW);
}

// =====================================================
// Modbus lesen / schreiben
// =====================================================
// Liest Holding-Register mit FC03.
// Die Funktion validiert Laenge, Slave-ID, Funktion, Bytezahl und CRC. Nur komplett
// gueltige Antworten werden in values[] uebernommen; alles andere erhoeht Fehlerzaehler.
bool readHolding(uint8_t slave, uint16_t startAddr, uint16_t count, uint16_t* values) {
  if (count < 1 || count > 60) {
    lastStatus = "Ungueltige Registeranzahl";
    formatErrors++;
    return false;
  }

  uint8_t request[8];
  request[0] = slave;
  request[1] = 0x03;
  request[2] = highByte(startAddr);
  request[3] = lowByte(startAddr);
  request[4] = highByte(count);
  request[5] = lowByte(count);
  uint16_t crc = crc16_modbus(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  sendFrame(request, 8);

  uint8_t response[128];
  uint16_t index = 0;
  uint16_t expectedLen = 5 + count * 2;
  uint32_t start = millis();
  while (millis() - start < cfgModbusTimeoutMs) {
    while (Serial2.available() && index < sizeof(response)) {
      response[index++] = Serial2.read();
      if (index >= expectedLen) break;
    }
    if (index >= expectedLen) break;
    delay(1);
  }

  if (index == 0) {
    lastStatus = "Timeout";
    timeoutErrors++;
    return false;
  }


  if (index < 5) {
    lastStatus = "Antwort zu kurz";
    formatErrors++;
    return false;
  }

  uint16_t rxCrc = response[index - 2] | ((uint16_t)response[index - 1] << 8);
  uint16_t calcCrc = crc16_modbus(response, index - 2);
  if (rxCrc != calcCrc) {
    lastStatus = "CRC Fehler";
    crcErrors++;
    return false;
  }

  if (response[0] != slave) {
    lastStatus = "Falsche Slave-ID";
    formatErrors++;
    return false;
  }

  if (response[1] == 0x83) {
    lastStatus = String("Modbus Exception ") + String(response[2]);
    modbusErrors++;
    return false;
  }

  if (response[1] != 0x03) {
    lastStatus = "Falsche Funktion";
    formatErrors++;
    return false;
  }

  uint8_t byteCount = response[2];
  if (byteCount != count * 2 || index != expectedLen) {
    lastStatus = "Falsche Bytezahl";
    formatErrors++;
    return false;
  }

  for (uint16_t i = 0; i < count; i++) {
    values[i] = ((uint16_t)response[3 + i * 2] << 8) | response[4 + i * 2];
  }

  lastStatus = "OK";
  return true;
}

// Schreibt genau ein Holding-Register mit FC06 und prueft das Echo-Telegramm.
// Diese Funktion macht nur den einzelnen Schreibversuch. Readback und Wiederholungen
// liegen in writeHoldingVerified().
bool writeHoldingOnce(uint8_t slave, uint16_t addr, uint16_t value) {
  uint8_t request[8];
  request[0] = slave;
  request[1] = 0x06;
  request[2] = highByte(addr);
  request[3] = lowByte(addr);
  request[4] = highByte(value);
  request[5] = lowByte(value);
  uint16_t crc = crc16_modbus(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  sendFrame(request, 8);

  uint8_t response[16];
  uint16_t index = 0;
  uint32_t start = millis();
  while (millis() - start < cfgModbusTimeoutMs) {
    while (Serial2.available() && index < sizeof(response)) {
      response[index++] = Serial2.read();
      if (index >= 8) break;
    }
    if (index >= 8) break;
    delay(1);
  }

  if (index == 0) {
    lastStatus = "Write Timeout";
    timeoutErrors++;
    return false;
  }

  if (index != 8) {
    lastStatus = "Write Antwortlaenge";
    formatErrors++;
    return false;
  }

  uint16_t rxCrc = response[6] | ((uint16_t)response[7] << 8);
  uint16_t calcCrc = crc16_modbus(response, 6);
  if (rxCrc != calcCrc) {
    lastStatus = "Write CRC Fehler";
    crcErrors++;
    return false;
  }

  for (uint8_t i = 0; i < 6; i++) {
    if (response[i] != request[i]) {
      lastStatus = "Write Echo falsch";
      formatErrors++;
      return false;
    }
  }

  return true;
}

// Robustes Schreiben mit mehreren Versuchen und Readback.
// verifyMask erlaubt Read-Modify-Write fuer Register 0014: Es werden nur die Bits
// verglichen, die ein Bedienbefehl wirklich aendern sollte. Andere Bits duerfen erhalten bleiben.
bool writeHoldingVerified(uint16_t addr, uint16_t value, uint16_t verifyMask, String label) {
  bool ok = false;
  for (uint8_t attempt = 1; attempt <= WRITE_RETRIES; attempt++) {
    delay(WRITE_PAUSE_BEFORE);
    bool wr = writeHoldingOnce(cfgModbusSlaveId, addr, value);
    delay(WRITE_PAUSE_AFTER);

    uint16_t rb[1];
    bool rd = readHolding(cfgModbusSlaveId, addr, 1, rb);
    if (wr && rd && ((rb[0] & verifyMask) == (value & verifyMask))) {
      if (addr < MAX_REG_STORE) {
        regValue[addr] = rb[0];
        regValid[addr] = true;
      }
      ok = true;
      break;
    }
  }

  if (ok) writeOk++;
  else writeErrors++;
  return ok;
}

// =====================================================
// MQTT
// =====================================================
String topic(const char* suffix) {
  String t = cfgMqttRoot;
  t += "/";
  t += suffix;
  return t;
}

// Veröffentlicht ein MQTT-Topic unterhalb des konfigurierten Prefix.
// State-/Status-/Config-Werte werden retained gesendet, damit openHAB nach einem
// Reconnect sofort den letzten bekannten Zustand sieht. Command-Topics werden nie
// retained gesendet; eingehende retained Commands werden beim Connect gezielt geleert.
void mqttPublish(const char* suffix, const String& payload, bool retained = true) {
  if (!mqtt.connected()) return;
  String t = topic(suffix);
  if (mqtt.publish(t.c_str(), payload.c_str(), retained)) mqttPublishes++;
}

void mqttPublishNumber(const char* suffix, long value) {
  mqttPublish(suffix, String(value), true);
}

void mqttPublishTemp(const char* suffix, uint16_t raw) {
  mqttPublish(suffix, oneDecimalFromRaw10(raw), true);
}

void mqttPublishTempWholeDegrees(const char* suffix, uint16_t raw) {
  mqttPublish(suffix, String(signed16(raw) / 10), true);
}

// Maschinenfreundliche MQTT-Payloads fuer openHAB/Influx.
// Die deutsche Weboberflaeche uebersetzt diese Werte optisch, MQTT bleibt bewusst englisch.
const char* modeTextFrom14(uint16_t v) {
  switch ((v >> 5) & 0x07) {
    case 0: return "off";
    case 1: return "cool";
    case 2: return "heat";
    case 3: return "dry";
    case 4: return "fan";
    case 5: return "auto";
    default: return "unknown";
  }
}

const char* fanTextFrom14(uint16_t v) {
  switch ((v >> 3) & 0x03) {
    case 0: return "low";
    case 1: return "medium";
    case 2: return "high";
    case 3: return "auto";
    default: return "unknown";
  }
}

String bitTextOnOff(uint16_t v, uint8_t b) {
  return (v & (1u << b)) ? "ON" : "OFF";
}

// Loescht retained Command-Topics, bevor abonniert wird.
// Dadurch wird ein alter, retained gespeicherter Befehl nach Reconnect nicht nochmals ausgefuehrt.
void mqttClearRetained(const char* suffix) {
  if (!mqtt.connected()) return;
  String t = topic(suffix);
  mqtt.publish(t.c_str(), "", true);
}

void mqttClearRetainedCommands() {
  const char* commands[] = {
    "command/setpoint", "command/mode", "command/fan", "command/silent", "command/swing", "command/economy"
  };
  for (uint8_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) mqttClearRetained(commands[i]);
}

// Einheitliche Rueckmeldung fuer openHAB/Automatisierungen.
// Nach jedem MQTT-Befehl steht hier, welcher Befehl bearbeitet wurde und ob er erfolgreich war.
String mqttCommandResultCode(bool ok, const String& msg) {
  if (ok) return "ok";
  if (msg.indexOf("disabled") >= 0) return "rejected_mqtt_command_disabled";
  if (msg.indexOf("busy") >= 0) return "rejected_write_busy";
  if (msg.indexOf("unknown") >= 0) return "rejected_unknown_topic";
  if (msg.indexOf("range") >= 0 || msg.indexOf("whole degrees") >= 0) return "rejected_out_of_range";
  if (msg.indexOf("invalid") >= 0 || msg.indexOf("not a pure number") >= 0) return "rejected_invalid_value";
  return "modbus_failed";
}

void mqttPublishCommandResult(const String& command, bool ok, const String& msg) {
  if (!mqtt.connected()) return;
  mqttPublish("status/last_command", command, true);
  mqttPublish("status/last_command_result", mqttCommandResultCode(ok, msg), true);
  mqttPublish("status/last_error", ok ? "none" : msg, true);
}

bool strictFloatFromString(String text, float &out) {
  text.trim();
  text.replace(',', '.');
  if (text.length() == 0) return false;

  bool seenDigit = false;
  bool seenDot = false;
  for (uint16_t i = 0; i < text.length(); i++) {
    char c = text[i];
    if (c >= '0' && c <= '9') {
      seenDigit = true;
    } else if (c == '.' && !seenDot) {
      seenDot = true;
    } else if ((c == '+' || c == '-') && i == 0) {
      // Vorzeichen erlaubt, Bereichspruefung folgt separat
    } else {
      return false;
    }
  }
  if (!seenDigit) return false;
  out = text.toFloat();
  return true;
}

bool parseSetpointCommandToRaw(String payload, uint16_t &raw, String &msg) {
  float f = 0.0f;
  if (!strictFloatFromString(payload, f)) {
    msg = "MQTT setpoint rejected: not a pure number";
    return false;
  }

  long n = lroundf(f);
  if (fabsf(f - (float)n) > 0.001f) {
    msg = "MQTT setpoint rejected: whole degrees only";
    return false;
  }

  // Normaler MQTT-Befehl: Grad Celsius 18..30, z. B. 22.
  if (n >= 18 && n <= 30) {
    raw = (uint16_t)(n * 10);
    msg = String("setpoint ") + String(n);
    return true;
  }

  // Zusaetzlich toleriert: Rohwert 180..300, aber nur volle Grad => letzte Dezimalstelle 0.
  if (n >= 180 && n <= 300 && (n % 10) == 0) {
    raw = (uint16_t)n;
    msg = String("setpoint raw ") + String(n);
    return true;
  }

  msg = "MQTT setpoint rejected: range 18..30 C or raw 180..300";
  return false;
}

bool writeMasked14(uint16_t mask, uint16_t bits, const String& label, String &msg) {
  uint16_t live[1];
  delay(WRITE_PAUSE_BEFORE);
  if (!readHolding(cfgModbusSlaveId, 14, 1, live)) {
    msg = "Register 0014 konnte nicht gelesen werden";
    return false;
  }
  uint16_t target = (live[0] & ~mask) | (bits & mask);
  bool ok = writeHoldingVerified(14, target, mask, label);
  msg = ok ? "OK" : "NOK";
  return ok;
}

bool parseOnOff(String payload, uint16_t &bit) {
  payload.trim();
  payload.toLowerCase();
  if (payload == "1" || payload == "on" || payload == "true") {
    bit = 1;
    return true;
  }
  if (payload == "0" || payload == "off" || payload == "false") {
    bit = 0;
    return true;
  }
  return false;
}

bool parseModeCommand(String payload, uint16_t &bits) {
  payload.trim();
  payload.toLowerCase();
  if (payload == "0" || payload == "standby" || payload == "off") { bits = 0x0000; return true; }
  if (payload == "1" || payload == "cool" || payload == "cooling") { bits = 0x0020; return true; }
  if (payload == "2" || payload == "heat" || payload == "heating") { bits = 0x0040; return true; }
  if (payload == "3" || payload == "dry" || payload == "dehumidify") { bits = 0x0060; return true; }
  if (payload == "4" || payload == "fan") { bits = 0x0080; return true; }
  if (payload == "5" || payload == "auto" || payload == "automatic") { bits = 0x00A0; return true; }
  return false;
}

bool parseFanCommand(String payload, uint16_t &bits) {
  payload.trim();
  payload.toLowerCase();
  if (payload == "0" || payload == "low") { bits = 0x0000; return true; }
  if (payload == "1" || payload == "medium" || payload == "mid") { bits = 0x0008; return true; }
  if (payload == "2" || payload == "high") { bits = 0x0010; return true; }
  if (payload == "3" || payload == "auto") { bits = 0x0018; return true; }
  return false;
}

void mqttSubscribeCommands() {
  if (!mqtt.connected()) return;
  if (!cfgMqttCommandsEnabled) return;
  String t1 = topic("command/#");
  mqtt.subscribe(t1.c_str());
}

// Verarbeitet eingehende MQTT-Command-Topics.
// State-/Sensor-Topics werden ausdruecklich ignoriert, damit ein Rueckkanal aus openHAB
// nicht versehentlich als Befehl interpretiert wird. Befehle liegen primaer unter
// Ausschliesslich command/... wird als Eingang akzeptiert; alle state/status/config-Topics sind reine Ausgaenge.
void mqttCallback(char* topicChars, byte* payload, unsigned int length) {
  String incomingTopic(topicChars);
  String payloadText;
  payloadText.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) payloadText += (char)payload[i];
  payloadText.trim();
  if (payloadText.length() == 0) return; // geloeschte retained Commands ignorieren

  // State-/Status-/Config-Topics sind nur Ausgang. Eingehende Werte darauf niemals als Befehl verwenden.
  if (incomingTopic.startsWith(topic("state/")) || incomingTopic.startsWith(topic("status/")) || incomingTopic.startsWith(topic("config/"))) {
    return;
  }

  String cmdPrefix = cfgMqttRoot + "/command/";
  String cmd;
  if (incomingTopic.startsWith(cmdPrefix)) {
    cmd = incomingTopic.substring(cmdPrefix.length());
  } else {
    return;
  }

  if (!cfgMqttCommandsEnabled) {
    mqttPublishCommandResult(cmd, false, "MQTT commands disabled");
    return;
  }

  if (writeBusy) {
    mqttPublishCommandResult(cmd, false, "write busy");
    return;
  }

  bool ok = false;
  String msg;

  writeBusy = true;

  if (cmd == "setpoint") {
    uint16_t raw = 0;
    if (parseSetpointCommandToRaw(payloadText, raw, msg)) {
      ok = writeHoldingVerified(8, raw, 0xFFFF, "MQTT SETP");
      msg = ok ? msg + " OK" : msg + " NOK";
    }
  } else if (cmd == "mode") {
    uint16_t bits = 0;
    if (parseModeCommand(payloadText, bits)) ok = writeMasked14(0x00E0, bits, "MQTT MODE", msg);
    else msg = "MQTT mode rejected: invalid value";
  } else if (cmd == "fan") {
    uint16_t bits = 0;
    if (parseFanCommand(payloadText, bits)) ok = writeMasked14(0x0018, bits, "MQTT FAN", msg);
    else msg = "MQTT fan rejected: invalid value";
  } else if (cmd == "silent") {
    uint16_t bit = 0;
    if (parseOnOff(payloadText, bit)) ok = writeMasked14(0x0001, bit, "MQTT SIL", msg);
    else msg = "MQTT silent rejected: invalid value";
  } else if (cmd == "swing") {
    uint16_t bit = 0;
    if (parseOnOff(payloadText, bit)) ok = writeMasked14(0x0002, bit << 1, "MQTT SWG", msg);
    else msg = "MQTT swing rejected: invalid value";
  } else if (cmd == "economy") {
    uint16_t bit = 0;
    if (parseOnOff(payloadText, bit)) ok = writeMasked14(0x0100, bit << 8, "MQTT ECO", msg);
    else msg = "MQTT economy rejected: invalid value";
  } else {
    msg = "MQTT command rejected: unknown topic";
  }

  writeBusy = false;
  lastStatus = ok ? "MQTT Befehl OK" : msg;
  mqttPublishCommandResult(cmd, ok, msg);
}

void mqttConnectIfNeeded() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttTryMs < 5000) return;

  lastMqttTryMs = millis();
  mqttReconnectAttempts++;

  String willTopic = topic("status/availability");
  bool ok;
  if (cfgMqttUser.length() > 0) {
    ok = mqtt.connect(cfgMqttClientId.c_str(), cfgMqttUser.c_str(), cfgMqttPass.c_str(), willTopic.c_str(), 0, true, "offline");
  } else {
    ok = mqtt.connect(cfgMqttClientId.c_str(), willTopic.c_str(), 0, true, "offline");
  }

  lastMqttState = mqtt.state();
  lastMqttError = ok ? "OK" : mqttStateText(lastMqttState);

  if (ok) {
    mqttPublish("status/availability", "online", true);
    mqttPublish("status/version", FW_VERSION, true);
    mqttPublish("status/ip", currentIpText(), true);
    mqttPublish("status/mqtt", "OK", true);
    mqttPublish("status/modbus", publicStatusText(), true);
    mqttPublish("status/last_error", "none", true);
    mqttPublish("config/modbus/slave_id", String(cfgModbusSlaveId), true);
    mqttPublish("config/modbus/baudrate", String(cfgModbusBaudrate), true);
    mqttPublish("config/modbus/data_bits", String(cfgModbusDataBits), true);
    mqttPublish("config/modbus/parity", modbusParityText(), true);
    mqttPublish("config/modbus/stop_bits", String(cfgModbusStopBits), true);
    mqttPublish("config/modbus/poll_interval", String(cfgModbusPollIntervalMs), true);
    mqttPublish("config/modbus/timeout", String(cfgModbusTimeoutMs), true);
    mqttPublish("config/mqtt/publish_interval", String(cfgMqttPublishIntervalSec), true);
    mqttClearRetainedCommands();
    mqttSubscribeCommands();
    publishControlChangesIfNeeded(true);
    lastMqttPublishMs = 0;
  }
}

// Veröffentlicht die wichtigsten Bedienzustände nur bei Änderung.
// Dadurch springen openHAB-Items fuer Ein/Aus, Betriebsart und Solltemperatur sofort,
// ohne dass alle Messwerte bei jedem Poll erneut publiziert werden.
void publishControlChangesIfNeeded(bool force) {
  if (!mqtt.connected()) return;

  if (regValid[14] && (force || !lastMqttModeValid || regValue[14] != lastMqttModeReg14)) {
    uint16_t v = regValue[14];
    mqttPublish("state/control/mode", modeTextFrom14(v), true);
    mqttPublish("state/control/fan", fanTextFrom14(v), true);
    mqttPublish("state/control/silent", (v & 0x0001) ? "ON" : "OFF", true);
    mqttPublish("state/control/swing", (v & 0x0002) ? "ON" : "OFF", true);
    mqttPublish("state/control/economy", (v & 0x0100) ? "ON" : "OFF", true);
    lastMqttModeReg14 = v;
    lastMqttModeValid = true;
  }

  if (regValid[8] && (force || !lastMqttSetpointValid || regValue[8] != lastMqttSetpointReg8)) {
    mqttPublishTempWholeDegrees("state/temperature/setpoint", regValue[8]);
    lastMqttSetpointReg8 = regValue[8];
    lastMqttSetpointValid = true;
  }
}

// Veroeffentlicht die vollstaendige Werteliste im eingestellten MQTT-Intervall.
// Einheiten stehen nicht im Payload, damit openHAB/Influx saubere Number-Items
// speichern kann. Die Einheit wird in openHAB am Item definiert.
void publishAllMqtt() {
  if (!mqtt.connected()) return;

  if (regValid[0]) mqttPublishTemp("state/temperature/room", regValue[0]);
  if (regValid[1]) mqttPublishTemp("state/temperature/indoor_heat_exchanger", regValue[1]);
  if (regValid[2]) mqttPublishTemp("state/temperature/outdoor_heat_exchanger", regValue[2]);
  if (regValid[3]) mqttPublishTemp("state/temperature/discharge", regValue[3]);
  if (regValid[4]) mqttPublishTemp("state/temperature/outdoor", regValue[4]);
  if (regValid[6]) mqttPublishTemp("state/temperature/unknown", regValue[6]);
  if (regValid[8]) {
    mqttPublishTempWholeDegrees("state/temperature/setpoint", regValue[8]);
  }

  if (regValid[10]) {
    uint16_t v = regValue[10];
    mqttPublish("state/output/condensate_water_valve", bitTextOnOff(v,0));
    mqttPublish("state/output/condensate_water_pump", bitTextOnOff(v,1));
    mqttPublish("state/output/heat_r", bitTextOnOff(v,2));
    mqttPublish("state/output/aux1", bitTextOnOff(v,3));
    mqttPublish("state/output/aux2", bitTextOnOff(v,4));
    mqttPublish("state/output/four_way_valve", bitTextOnOff(v,5));
  }

  if (regValid[11]) {
    uint16_t v = regValue[11];
    mqttPublish("state/status/condensate_water_alarm", (v & 0x0001) ? "no_alarm" : "0");
    mqttPublish("state/status/pump_water_level", (v & 0x0002) ? "1" : "active");
    mqttPublish("state/status/system_enable_energy_boost", (v & 0x0004) ? "closed" : "0");
  }

  if (regValid[12]) {
    uint16_t v = regValue[12];
    mqttPublish("state/setting/display_light", (v & 0x0001) ? "ON" : "OFF");
    mqttPublish("state/setting/heat_pump", (v & 0x0002) ? "heat_pump" : "cooling_only");
    mqttPublish("state/setting/key_lock", (v & 0x0004) ? "locked" : "unlocked");
    mqttPublish("state/setting/ceiling_floor_installation", (v & 0x0008) ? "ceiling" : "floor");
    mqttPublish("state/setting/input_logic", (v & 0x0010) ? "normally_open" : "normally_closed");
    mqttPublish("state/setting/remote_room_sensor", (v & 0x0020) ? "ON" : "OFF");
    mqttPublish("state/setting/temperature_unit", (v & 0x0040) ? "fahrenheit" : "celsius");
  }

  if (regValid[46]) mqttPublishNumber("state/setpoint_limit/cooling_min", signed16(regValue[46]));
  if (regValid[47]) mqttPublishNumber("state/setpoint_limit/heating_min", signed16(regValue[47]));
  if (regValid[48]) mqttPublishNumber("state/setpoint_limit/cooling_max", signed16(regValue[48]));
  if (regValid[49]) mqttPublishNumber("state/setpoint_limit/heating_max", signed16(regValue[49]));

  if (regValid[300]) mqttPublishNumber("state/measurement/compressor_frequency", signed16(regValue[300]));
  if (regValid[301]) mqttPublishNumber("state/measurement/indoor_fan_rpm", signed16(regValue[301]));
  if (regValid[302]) mqttPublishNumber("state/measurement/outdoor_fan_rpm", signed16(regValue[302]));
  if (regValid[303]) mqttPublishNumber("state/measurement/eev_opening", signed16(regValue[303]));
  if (regValid[304]) mqttPublishNumber("state/measurement/error_code", signed16(regValue[304]));
  if (regValid[305]) mqttPublishNumber("state/measurement/protection_code", signed16(regValue[305]));
  if (regValid[306]) mqttPublish("state/measurement/dc_current", oneDecimalFromRaw10(regValue[306]), true);
  if (regValid[307]) mqttPublishNumber("state/measurement/dc_voltage", signed16(regValue[307]));
  if (regValid[308]) mqttPublish("state/measurement/ac_current", oneDecimalFromRaw10(regValue[308]), true);
  if (regValid[309]) mqttPublishNumber("state/measurement/ac_voltage", signed16(regValue[309]));

  // Gateway-Status im Intervall. Bedienzustand und Solltemperatur werden getrennt
  // nur bei tatsaechlicher Aenderung veroeffentlicht.
  mqttPublish("status/modbus", publicStatusText(), true);
  mqttPublish("status/mqtt", mqtt.connected() ? "OK" : lastMqttError, true);
  mqttPublish("status/ip", currentIpText(), true);
}

// =====================================================
// Polling
// =====================================================
// Uebernimmt einen erfolgreich gelesenen Registerblock in den lokalen Speicher.
// Die Weboberflaeche und MQTT-Ausgabe greifen nur auf diesen gueltigen Zwischenspeicher zu.
void storeBlock(uint16_t start, uint16_t count, uint16_t* vals) {
  for (uint16_t i = 0; i < count; i++) {
    uint16_t addr = start + i;
    if (addr < MAX_REG_STORE) {
      regValue[addr] = vals[i];
      regValid[addr] = true;
    }
  }
}

void pollUnico() {
  if (writeBusy) return;

  bool anyOk = false;
  uint16_t vals[32];

  // 0000..0014: Temperaturen 0..4, 6, 8 + Zustand 10 + Status 11 + Einstellungen 12 + Bedienen 14
  if (readHolding(cfgModbusSlaveId, 0, 15, vals)) {
    storeBlock(0, 15, vals);
    anyOk = true;
    delay(80);
  }

  // 0046..0049: Sollwerte
  if (readHolding(cfgModbusSlaveId, 46, 4, vals)) {
    storeBlock(46, 4, vals);
    anyOk = true;
    delay(80);
  }

  // 0300..0309: Messwerte
  if (readHolding(cfgModbusSlaveId, 300, 10, vals)) {
    storeBlock(300, 10, vals);
    anyOk = true;
  }

  if (anyOk) {
    okPolls++;
    lastPollMs = millis();

    // Wichtig fuer openHAB: Der Bedienzustand wird sofort geschickt,
    // aber nur wenn sich 0014 oder Solltemperatur 0008 tatsaechlich geaendert hat.
    publishControlChangesIfNeeded(false);

    // Die vollstaendige Sensor-/Messwertliste wird nur zyklisch im MQTT-Intervall gesendet.
    if (mqtt.connected() && (lastMqttPublishMs == 0 || millis() - lastMqttPublishMs >= (uint32_t)cfgMqttPublishIntervalSec * 1000UL)) {
      publishAllMqtt();
      lastMqttPublishMs = millis();
    }
  }
}

// =====================================================
// Web API
// =====================================================
void sendNoCache() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
}

void handleRoot() {
  sendNoCache();
  server.send_P(200, "text/html; charset=utf-8", MAIN_page);
}

// HTTP-State-API fuer die Weboberflaeche.
// Liefert kompaktes JSON mit Anzeigenwerten, Zaehlern und Metadaten. Sie ist prima
// fuer das Webinterface, fuer openHAB bleibt MQTT die bevorzugte Schnittstelle.
void handleState() {
  String s;
  s.reserve(4096);
  s += "{";
  s += "\"version\":\""; s += FW_VERSION; s += "\",";
  s += "\"status\":\""; s += jsonEscape(publicStatusText()); s += "\",";
  s += "\"ip\":\""; s += currentIpText(); s += "\",";
  s += "\"wifiMode\":\""; s += jsonEscape(wifiModeText); s += "\",";
  s += "\"rssi\":"; s += String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0); s += ",";
  s += "\"mqttConnected\":"; s += (mqtt.connected() ? "true" : "false"); s += ",";
  int mqttStateNow = mqtt.connected() ? 0 : mqtt.state();
  String mqttErrNow = mqtt.connected() ? String("OK") : lastMqttError;
  if (!mqtt.connected() && mqttErrNow.length() == 0) mqttErrNow = mqttStateText(mqttStateNow);
  s += "\"mqttState\":"; s += String(mqttStateNow); s += ",";
  s += "\"mqttError\":\""; s += jsonEscape(mqttErrNow); s += "\",";
  s += "\"configPasswordSet\":"; s += (hasConfigPassword() ? "true" : "false"); s += ",";
  s += "\"climateProtected\":"; s += (cfgClimateProtected ? "true" : "false"); s += ",";
  s += "\"mqttBroker\":\""; s += jsonEscape(cfgMqttHost + ":" + String(cfgMqttPort)); s += "\",";
  s += "\"mqttRoot\":\""; s += jsonEscape(cfgMqttRoot); s += "\",";
  s += "\"mqttIntervalSec\":"; s += String(cfgMqttPublishIntervalSec); s += ",";
  s += "\"mqttPublishes\":"; s += String(mqttPublishes); s += ",";
  s += "\"mqttReconnectAttempts\":"; s += String(mqttReconnectAttempts); s += ",";
  s += "\"modbusSlave\":"; s += String(cfgModbusSlaveId); s += ",";
  s += "\"modbusBaud\":"; s += String(cfgModbusBaudrate); s += ",";
  s += "\"modbusBits\":"; s += String(cfgModbusDataBits); s += ",";
  s += "\"modbusParity\":\""; s += modbusParityText(); s += "\",";
  s += "\"modbusStop\":"; s += String(cfgModbusStopBits); s += ",";
  s += "\"pollInterval\":"; s += String(cfgModbusPollIntervalMs); s += ",";
  s += "\"modbusTimeout\":"; s += String(cfgModbusTimeoutMs); s += ",";
  s += "\"okPolls\":"; s += String(okPolls); s += ",";
  s += "\"timeoutErrors\":"; s += String(timeoutErrors); s += ",";
  s += "\"crcErrors\":"; s += String(crcErrors); s += ",";
  s += "\"formatErrors\":"; s += String(formatErrors); s += ",";
  s += "\"modbusErrors\":"; s += String(modbusErrors); s += ",";
  s += "\"writeOk\":"; s += String(writeOk); s += ",";
  s += "\"writeErrors\":"; s += String(writeErrors); s += ",";
  s += "\"regs\":{";
  bool first = true;
  for (uint16_t i = 0; i < MAX_REG_STORE; i++) {
    if (!regValid[i]) continue;
    if (!first) s += ",";
    first = false;
    s += "\""; s += String(i); s += "\":"; s += String(regValue[i]);
  }
  s += "}}";

  sendNoCache();
  server.send(200, "application/json; charset=utf-8", s);
}

// HTTP-API fuer alle schreibenden Klimabefehle und Einstellungen aus dem Reiter Bedienen.
// Wenn die Web-Bedienung deaktiviert ist, wird jedes Schreiben verworfen.
// Wenn Klimabefehle geschuetzt sind, wird vor jedem Schreiben das Passwort
// aus dem Parameter pass=... geprueft.
void handleWrite() {
  if (!server.hasArg("type")) {
    sendNoCache();
    server.send(400, "text/plain; charset=utf-8", "type fehlt");
    return;
  }

  if (!cfgWebCommandsEnabled) {
    sendNoCache();
    server.send(403, "text/plain; charset=utf-8", "Web-Bedienung ist deaktiviert");
    return;
  }

  if (!climateCommandAuthorized()) {
    sendNoCache();
    server.send(403, "text/plain; charset=utf-8", "Klimabefehl gesperrt: Passwort fehlt oder ist falsch");
    return;
  }

  writeBusy = true;
  String type = server.arg("type");
  bool ok = false;
  String msg = "";

  if (type == "temp") {
    if (!server.hasArg("addr") || !server.hasArg("value")) {
      msg = "addr oder value fehlt";
    } else {
      int addr = server.arg("addr").toInt();
      float f = server.arg("value").toFloat();
      bool rangeOk = true;
      if (addr == 5) rangeOk = (f >= -10.0f && f <= 50.0f);
      else if (addr == 8) rangeOk = (f >= 18.0f && f <= 30.0f && fabsf(f - roundf(f)) <= 0.001f);
      else rangeOk = false;

      if (!rangeOk) {
        msg = (addr == 8) ? "Solltemperatur nur volle Grad 18..30 erlaubt" : "Wert ausserhalb Bereich oder Register nicht erlaubt";
      } else {
        uint16_t raw = (uint16_t)((int16_t)roundf(f * 10.0f));
        ok = writeHoldingVerified((uint16_t)addr, raw, 0xFFFF, String("WR ") + String(addr));
        msg = ok ? "OK" : "NOK";
      }
    }
  }
  else if (type == "action14") {
    if (!server.hasArg("idx")) {
      msg = "idx fehlt";
    } else {
      int idx = server.arg("idx").toInt();
      if (idx < 0 || idx >= ACTION14_COUNT) {
        msg = "idx ungueltig";
      } else {
        uint16_t live[1];
        delay(WRITE_PAUSE_BEFORE);
        if (!readHolding(cfgModbusSlaveId, 14, 1, live)) {
          msg = "Register 0014 konnte nicht gelesen werden";
        } else {
          uint16_t target = (live[0] & ~actions14[idx].mask) | (actions14[idx].value & actions14[idx].mask);
          ok = writeHoldingVerified(14, target, actions14[idx].mask, String("ACT14 ") + String(idx));
          msg = ok ? "OK" : "NOK";
        }
      }
    }
  }
  else if (type == "part14") {
    if (!server.hasArg("part") || !server.hasArg("value")) {
      msg = "part oder value fehlt";
    } else {
      String part = server.arg("part");
      int value = server.arg("value").toInt();
      uint16_t mask = 0;
      uint16_t bits = 0;
      String label = "PART14";

      if (part == "fan") {
        if (value < 0 || value > 3) msg = "Luefterwert ungueltig";
        else { mask = 0x0018; bits = ((uint16_t)value & 0x03) << 3; label = "FAN14"; }
      } else if (part == "silent") {
        if (value < 0 || value > 1) msg = "Silent-Wert ungueltig";
        else { mask = 0x0001; bits = (uint16_t)value; label = "SIL14"; }
      } else if (part == "swing") {
        if (value < 0 || value > 1) msg = "Swing-Wert ungueltig";
        else { mask = 0x0002; bits = ((uint16_t)value) << 1; label = "SWG14"; }
      } else if (part == "eco") {
        if (value < 0 || value > 1) msg = "Economy-Wert ungueltig";
        else { mask = 0x0100; bits = ((uint16_t)value) << 8; label = "ECO14"; }
      } else {
        msg = "part ungueltig";
      }

      if (mask != 0) {
        uint16_t live[1];
        delay(WRITE_PAUSE_BEFORE);
        if (!readHolding(cfgModbusSlaveId, 14, 1, live)) {
          msg = "Register 0014 konnte nicht gelesen werden";
        } else {
          uint16_t target = (live[0] & ~mask) | (bits & mask);
          ok = writeHoldingVerified(14, target, mask, label);
          msg = ok ? "OK" : "NOK";
        }
      }
    }
  }
  else if (type == "part12") {
    if (!server.hasArg("part") || !server.hasArg("value")) {
      msg = "part oder value fehlt";
    } else {
      String part = server.arg("part");
      int value = server.arg("value").toInt();
      uint16_t mask = 0;
      uint16_t bits = 0;
      String label = "PART12";

      if (value < 0 || value > 1) {
        msg = "Einstellwert ungueltig";
      } else if (part == "display") {
        mask = 0x0001; bits = (uint16_t)value; label = "DISP12";
      } else if (part == "heatpump") {
        mask = 0x0002; bits = ((uint16_t)value) << 1; label = "HP12";
      } else if (part == "keylock") {
        mask = 0x0004; bits = ((uint16_t)value) << 2; label = "KEY12";
      } else if (part == "install") {
        mask = 0x0008; bits = ((uint16_t)value) << 3; label = "INST12";
      } else if (part == "inputlogic") {
        mask = 0x0010; bits = ((uint16_t)value) << 4; label = "INP12";
      } else if (part == "remote") {
        mask = 0x0020; bits = ((uint16_t)value) << 5; label = "REM12";
      } else if (part == "unit") {
        mask = 0x0040; bits = ((uint16_t)value) << 6; label = "UNIT12";
      } else {
        msg = "part ungueltig";
      }

      if (mask != 0) {
        uint16_t live[1];
        delay(WRITE_PAUSE_BEFORE);
        if (!readHolding(cfgModbusSlaveId, 12, 1, live)) {
          msg = "Register 0012 konnte nicht gelesen werden";
        } else {
          uint16_t target = (live[0] & ~mask) | (bits & mask);
          ok = writeHoldingVerified(12, target, mask, label);
          msg = ok ? "OK" : "NOK";
        }
      }
    }
  }
  else {
    msg = "type ungueltig";
  }

  writeBusy = false;
  sendNoCache();
  server.send(ok ? 200 : 500, "text/plain; charset=utf-8", msg);
}


void handleConfigLogin() {
  if (!hasConfigPassword()) {
    configSessionToken = makeSessionToken();
    sendNoCache();
    server.sendHeader("Set-Cookie", String("ugw_auth=") + configSessionToken + "; Path=/");
    server.send(200, "text/plain; charset=utf-8", "OK");
    return;
  }
  if (!server.hasArg("pass") || server.arg("pass") != cfgConfigPassword) {
    sendNoCache();
    server.send(403, "text/plain; charset=utf-8", "Passwort falsch");
    return;
  }
  configSessionToken = makeSessionToken();
  sendNoCache();
  server.sendHeader("Set-Cookie", String("ugw_auth=") + configSessionToken + "; Path=/");
  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleConfigGet() {
  if (!requireConfigAuth()) return;
  String s;
  s.reserve(2400);
  s += "{";
  s += "\"configPasswordSet\":"; s += (hasConfigPassword() ? "true" : "false"); s += ",";
  s += "\"climateProtected\":"; s += (cfgClimateProtected ? "true" : "false"); s += ",";
  s += "\"webCommandsEnabled\":"; s += (cfgWebCommandsEnabled ? "true" : "false"); s += ",";
  s += "\"mqttCommandsEnabled\":"; s += (cfgMqttCommandsEnabled ? "true" : "false"); s += ",";
  s += "\"wifiSsid\":\""; s += jsonEscape(cfgWifiSsid); s += "\",";
  s += "\"wifiPassSet\":"; s += (cfgWifiPass.length() ? "true" : "false"); s += ",";
  s += "\"apSsid\":\""; s += jsonEscape(cfgApSsid); s += "\",";
  s += "\"apPassSet\":"; s += (cfgApPass.length() ? "true" : "false"); s += ",";
  s += "\"mqttHost\":\""; s += jsonEscape(cfgMqttHost); s += "\",";
  s += "\"mqttPort\":"; s += String(cfgMqttPort); s += ",";
  s += "\"mqttUser\":\""; s += jsonEscape(cfgMqttUser); s += "\",";
  s += "\"mqttPassSet\":"; s += (cfgMqttPass.length() ? "true" : "false"); s += ",";
  s += "\"mqttClientId\":\""; s += jsonEscape(cfgMqttClientId); s += "\",";
  s += "\"mqttRoot\":\""; s += jsonEscape(cfgMqttRoot); s += "\",";
  s += "\"mqttIntervalSec\":"; s += String(cfgMqttPublishIntervalSec); s += ",";
  s += "\"modbusSlave\":"; s += String(cfgModbusSlaveId); s += ",";
  s += "\"modbusBaud\":"; s += String(cfgModbusBaudrate); s += ",";
  s += "\"modbusBits\":"; s += String(cfgModbusDataBits); s += ",";
  s += "\"modbusParityCode\":\""; s += cfgModbusParity; s += "\",";
  s += "\"modbusStop\":"; s += String(cfgModbusStopBits); s += ",";
  s += "\"modbusPollIntervalMs\":"; s += String(cfgModbusPollIntervalMs); s += ",";
  s += "\"modbusTimeoutMs\":"; s += String(cfgModbusTimeoutMs);
  s += "}";
  sendNoCache();
  server.send(200, "application/json; charset=utf-8", s);
}

// Speichert die Konfigurationsseite in Preferences/NVS.
// MQTT- und Modbus-Werte werden direkt uebernommen; WLAN/AP-Werte gelten nach Neustart,
// damit sich der ESP nicht mitten in einer Websession aus dem Netz trennt.
void handleConfigSave() {
  if (!requireConfigAuth()) return;

  String newWifiSsid = server.arg("wifi_ssid");
  String newApSsid = server.arg("ap_ssid");
  String newMqttHost = server.arg("mqtt_host");
  String newMqttClientId = server.arg("mqtt_clientid");
  String newMqttRoot = server.arg("mqtt_root");
  int newMqttPort = server.arg("mqtt_port").toInt();
  int newMqttInterval = server.arg("mqtt_interval").toInt();

  int newMbSlave = server.arg("mb_slave").toInt();
  uint32_t newMbBaud = (uint32_t)server.arg("mb_baud").toInt();
  int newMbBits = server.arg("mb_bits").toInt();
  String newMbParityText = server.arg("mb_parity");
  char newMbParity = newMbParityText.length() ? newMbParityText[0] : 'N';
  int newMbStop = server.arg("mb_stop").toInt();
  int newMbPoll = server.arg("mb_poll").toInt();
  int newMbTimeout = server.arg("mb_timeout").toInt();

  if (newWifiSsid.length() == 0) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "WLAN SSID fehlt"); return; }
  if (newApSsid.length() == 0) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "AP Name fehlt"); return; }
  if (newMqttHost.length() == 0) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "MQTT Broker fehlt"); return; }
  if (newMqttPort < 1 || newMqttPort > 65535) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "MQTT Port ungueltig"); return; }
  if (newMqttClientId.length() == 0) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "MQTT Client-ID fehlt"); return; }
  if (newMqttRoot.length() == 0) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "MQTT Topic Prefix fehlt"); return; }
  if (newMqttInterval < 5 || newMqttInterval > 900) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "MQTT Intervall 5..900 Sekunden"); return; }

  if (newMbSlave < 1 || newMbSlave > 247) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Modbus Slave-ID 1..247"); return; }
  if (newMbBaud < 1200 || newMbBaud > 115200) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Baudrate ungueltig"); return; }
  if (newMbBits != 7 && newMbBits != 8) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Datenbits 7 oder 8"); return; }
  if (newMbParity != 'N' && newMbParity != 'E' && newMbParity != 'O') { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Paritaet ungueltig"); return; }
  if (newMbStop != 1 && newMbStop != 2) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Stoppbits 1 oder 2"); return; }
  if (newMbPoll < 500 || newMbPoll > 60000) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Pollintervall 500..60000 ms"); return; }
  if (newMbTimeout < 100 || newMbTimeout > 5000) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Timeout 100..5000 ms"); return; }

  String newApPass = server.arg("ap_pass");
  if (newApPass.length() > 0 && newApPass.length() < 8) { sendNoCache(); server.send(400, "text/plain; charset=utf-8", "AP Passwort mindestens 8 Zeichen oder leer lassen"); return; }

  bool newClimateProtected = server.arg("climate_protected") == "1";
  if (newClimateProtected && !hasConfigPassword()) {
    sendNoCache();
    server.send(400, "text/plain; charset=utf-8", "Erst Konfigurationspasswort setzen, dann Klimabefehle schuetzen");
    return;
  }

  cfgWifiSsid = newWifiSsid;
  cfgApSsid = newApSsid;
  cfgMqttHost = newMqttHost;
  cfgMqttPort = (uint16_t)newMqttPort;
  cfgMqttUser = server.arg("mqtt_user");
  cfgMqttClientId = newMqttClientId;
  cfgMqttRoot = newMqttRoot;
  cfgMqttPublishIntervalSec = (uint16_t)newMqttInterval;
  cfgMqttCommandsEnabled = server.arg("mqtt_commands") == "1";
  cfgWebCommandsEnabled = server.arg("web_commands") != "0";
  cfgClimateProtected = newClimateProtected;

  cfgModbusSlaveId = (uint8_t)newMbSlave;
  cfgModbusBaudrate = newMbBaud;
  cfgModbusDataBits = (uint8_t)newMbBits;
  cfgModbusParity = newMbParity;
  cfgModbusStopBits = (uint8_t)newMbStop;
  cfgModbusPollIntervalMs = (uint16_t)newMbPoll;
  cfgModbusTimeoutMs = (uint16_t)newMbTimeout;
  normalizeRuntimeConfig();

  if (server.hasArg("wifi_pass") && server.arg("wifi_pass").length() > 0) cfgWifiPass = server.arg("wifi_pass");
  if (newApPass.length() > 0) cfgApPass = newApPass;
  if (server.hasArg("mqtt_pass") && server.arg("mqtt_pass").length() > 0) cfgMqttPass = server.arg("mqtt_pass");

  prefs.putString("wifi_ssid", cfgWifiSsid);
  prefs.putString("wifi_pass", cfgWifiPass);
  prefs.putString("ap_ssid", cfgApSsid);
  prefs.putString("ap_pass", cfgApPass);
  prefs.putString("mqtt_host", cfgMqttHost);
  prefs.putUShort("mqtt_port", cfgMqttPort);
  prefs.putString("mqtt_user", cfgMqttUser);
  prefs.putString("mqtt_pass", cfgMqttPass);
  prefs.putString("mqtt_cid", cfgMqttClientId);
  prefs.putString("mqtt_root", cfgMqttRoot);
  prefs.putUShort("mqtt_int", cfgMqttPublishIntervalSec);
  prefs.putBool("clim_prot", cfgClimateProtected);
  prefs.putBool("web_cmd", cfgWebCommandsEnabled);
  prefs.putBool("mqtt_cmd", cfgMqttCommandsEnabled);
  prefs.putUChar("mb_slave", cfgModbusSlaveId);
  prefs.putUInt("mb_baud", cfgModbusBaudrate);
  prefs.putUChar("mb_bits", cfgModbusDataBits);
  prefs.putUChar("mb_parity", (uint8_t)cfgModbusParity);
  prefs.putUChar("mb_stop", cfgModbusStopBits);
  prefs.putUShort("mb_poll", cfgModbusPollIntervalMs);
  prefs.putUShort("mb_timeout", cfgModbusTimeoutMs);

  applyModbusSerial();
  lastPollMs = 0;

  mqtt.disconnect();
  mqtt.setServer(cfgMqttHost.c_str(), cfgMqttPort);
  lastMqttTryMs = 0;
  lastMqttError = "Konfiguration gespeichert, MQTT reconnect folgt";

  sendNoCache();
  server.send(200, "text/plain; charset=utf-8", "Gespeichert. MQTT wird neu verbunden. WLAN/AP nach Neustart aktiv. Modbus gilt sofort.");
}

void handlePasswordSet() {
  if (!requireConfigAuth()) return;
  String oldPass = server.arg("old");
  String n1 = server.arg("new1");
  String n2 = server.arg("new2");

  if (hasConfigPassword() && oldPass != cfgConfigPassword) {
    sendNoCache(); server.send(403, "text/plain; charset=utf-8", "Altes Passwort falsch"); return;
  }
  if (n1.length() < 4) {
    sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Neues Passwort mindestens 4 Zeichen"); return;
  }
  if (n1 != n2) {
    sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Neue Passwoerter stimmen nicht ueberein"); return;
  }

  cfgConfigPassword = n1;
  prefs.putString("cfg_pass", cfgConfigPassword);
  configSessionToken = makeSessionToken();
  sendNoCache();
  server.sendHeader("Set-Cookie", String("ugw_auth=") + configSessionToken + "; Path=/");
  server.send(200, "text/plain; charset=utf-8", "Passwort gespeichert");
}

// Entfernt den Konfigurationsschutz nur nach Authentifizierung.
// Bei gesetztem Passwort ist sowohl eine gueltige Konfigurationssession als auch
// das alte Passwort erforderlich. Danach werden auch geschuetzte Klimabefehle deaktiviert.
void handlePasswordClear() {
  if (!requireConfigAuth()) return;
  if (!hasConfigPassword()) {
    sendNoCache(); server.send(400, "text/plain; charset=utf-8", "Kein Passwortschutz gesetzt"); return;
  }
  if (server.arg("old") != cfgConfigPassword) {
    sendNoCache(); server.send(403, "text/plain; charset=utf-8", "Altes Passwort falsch"); return;
  }
  cfgConfigPassword = "";
  cfgClimateProtected = false;
  prefs.remove("cfg_pass");
  prefs.putBool("clim_prot", false);
  configSessionToken = "";
  sendNoCache();
  server.sendHeader("Set-Cookie", "ugw_auth=; Path=/; Max-Age=0");
  server.send(200, "text/plain; charset=utf-8", "Passwortschutz entfernt. Klimabefehle sind wieder ungeschuetzt.");
}

// Loescht die komplette Web-/NVS-Konfiguration. Nach dem Neustart gelten wieder
// die Flash-Defaults aus UGW_secrets.h. Das ist der Rettungsweg bei falschem WLAN,
// falschem MQTT oder vergessener Schutzkonfiguration.
void handleConfigReset() {
  if (!requireConfigAuth()) return;
  prefs.clear();
  sendNoCache();
  server.send(200, "text/plain; charset=utf-8", "Konfiguration geloescht. Neustart wird ausgefuehrt...");
  delay(400);
  ESP.restart();
}

void handleRestart() {
  if (!requireConfigAuth()) return;
  sendNoCache();
  server.send(200, "text/plain; charset=utf-8", "Neustart wird ausgefuehrt...");
  delay(300);
  ESP.restart();
}

// =====================================================
// WLAN Setup
// =====================================================
// Baut zuerst die normale WLAN-Verbindung auf.
// Gelingt das nicht innerhalb des Zeitfensters, startet der ESP einen Fallback-AP,
// damit WLAN-/MQTT-Daten ueber die Konfigurationsseite korrigiert werden koennen.
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfgWifiSsid.c_str(), cfgWifiPass.c_str());
  Serial.print("WLAN verbinden");

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiModeText = "STA";
    Serial.print("WLAN OK: ");
    Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    bool apOk;
    if (cfgApPass.length() >= 8) apOk = WiFi.softAP(cfgApSsid.c_str(), cfgApPass.c_str());
    else apOk = WiFi.softAP(cfgApSsid.c_str());
    wifiModeText = apOk ? "AP" : "AP Fehler";
    Serial.print("Fallback AP: ");
    Serial.println(WiFi.softAPIP());
  }
}

// =====================================================
// Arduino Setup / Loop
// =====================================================
// Arduino-Initialisierung: Registerspeicher leeren, Konfiguration laden,
// RS485, WLAN, MQTT und Webserver starten. Ab hier arbeitet loop() ereignisgetrieben.
void setup() {
  Serial.begin(115200);
  delay(300);

  for (uint16_t i = 0; i < MAX_REG_STORE; i++) {
    regValue[i] = 0;
    regValid[i] = false;
  }

  pinMode(MODBUS_DIR_PIN, OUTPUT);
  digitalWrite(MODBUS_DIR_PIN, LOW);

  loadConfig();
  applyModbusSerial();
  setupWiFi();

  mqtt.setServer(cfgMqttHost.c_str(), cfgMqttPort);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);

  const char* headerKeys[] = {"Cookie"};
  server.collectHeaders(headerKeys, 1);

  server.on("/", handleRoot);
  server.on("/api/state", handleState);
  server.on("/api/write", handleWrite);
  server.on("/api/config", HTTP_GET, handleConfigGet);
  server.on("/api/config_login", HTTP_POST, handleConfigLogin);
  server.on("/api/config_save", HTTP_POST, handleConfigSave);
  server.on("/api/password", HTTP_POST, handlePasswordSet);
  server.on("/api/password_clear", HTTP_POST, handlePasswordClear);
  server.on("/api/config_reset", HTTP_POST, handleConfigReset);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.begin();

  lastStatus = "Bereit";
  Serial.println("Unico-Gateway V1.3.2 bereit - SETTINGS_PANEL_ACTIVE / Bedienen plus Einstellungen");
}

// Hauptschleife: Webserver bedienen, MQTT-Verbindung pflegen und Modbus zyklisch pollen.
// Waehrend eines Schreibbefehls wird Polling pausiert, damit die UNICO-Platine nicht
// mit Lese- und Schreibtelegrammen gleichzeitig belastet wird.
void loop() {
  server.handleClient();

  mqttConnectIfNeeded();
  if (mqtt.connected()) mqtt.loop();

  if (!writeBusy && millis() - lastPollMs >= cfgModbusPollIntervalMs) {
    lastPollMs = millis();
    pollUnico();
  }
}
