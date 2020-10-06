#include "neo_wifi.h"
#include <SPIFFS.h>
#include <sstream>
#include "Motor.h"

int NeoWifi::linear_x_ = 0;
int NeoWifi::angular_z_ = 0;
bool NeoWifi::wifi_control_ = false;



NeoWifi::NeoWifi() : server_(80),
                     ws_("/controller") {}

void NeoWifi::init()
{
    ws_.onEvent(onWsEvent);
    server_.addHandler(&ws_);

    server_.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", String(), false, process_html);
        // request->send(SPIFFS, "/ws.html", "text/html");
    });

    // Route to load style.css file
    server_.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/style.css", "text/css");
    });

    // Route to load style.css file
    server_.on("/controls.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/controls.js", "text/javascript");
    });

    server_.begin();
}

String NeoWifi::process_html(const String &var)
{
    if (var == "ESP32_IP")
    {
        IPAddress IP = WiFi.softAPIP();
        return IP.toString();
    }
    return String();
}

void NeoWifi::onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{


    if (type == WS_EVT_CONNECT)
    {

        Serial.println("Websocket client connection received");
        NeoWifi::wifi_control_ = true;
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        Serial.println("Client disconnected");
        Serial.println("-----------------------");
        NeoWifi::wifi_control_ = false;
    }
    else if (type == WS_EVT_DATA && len > 0)
    {

        int number;
        std::ostringstream convert;
        for (int i = 1; i < len; i++)
        {
            convert << data[i];
        }
        std::istringstream iss(convert.str());

        iss >> number;
        if (iss.fail())
        {
            // something wrong happened
            Serial.printf("iss failed: %s\n", data);
            return;
        }

        if (data[0] == 'y')
        {
            Serial.print("Speed received: ");
            NeoWifi::linear_x_ = number;
        }
        else if (data[0] == 'x')
        {
            Serial.print("Steering received: ");
            NeoWifi::angular_z_ = number;
        }
        else
        {
            Serial.printf("UNKNOWN: %s\n", data);
        }
        Serial.println(number);
    }
}