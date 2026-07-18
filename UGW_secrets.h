#pragma once

// WLAN Client
const char* UGW_WIFI_SSID     = "DEIN_WLAN_NAME";
const char* UGW_WIFI_PASSWORD = "DEIN_WLAN_PASSWORT";

// Fallback Access Point, if WLAN-connection fails
const char* UGW_AP_SSID       = "Unico-Gateway";
const char* UGW_AP_PASSWORD   = "12345678";

// MQTT Broker
const char* UGW_MQTT_HOST     = "192.168.178.10";
const uint16_t UGW_MQTT_PORT  = 1883;
const char* UGW_MQTT_USER     = "";
const char* UGW_MQTT_PASSWORD = "";
const char* UGW_MQTT_CLIENTID = "unico-gateway-esp32";
const char* UGW_MQTT_ROOT     = "unico";

// NTP-Zeitserver
// possibel are one hostname or one IPv4-Adress.
const char* UGW_NTP_SERVER  = "time.cloudflare.com";
//const char* UGW_NTP_SERVER  = "192.168.1.1";
