
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

class NeoWifi
{

public:
    NeoWifi();

    void init();
    bool wifi_has_control() { return wifi_control_; }
    void update_vel(int &linear_x, int &angular_z)
    {
        linear_x = linear_x_;
        angular_z = angular_z_;
    }

    // network credentials
    constexpr static char *ssid = (char *)"Neo";
    constexpr static char *password = (char *)"123456789";

private:
    static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

    static String process_html(const String &var);

    // Variable to store the HTTP request
    String header;

    AsyncWebServer server_;
    AsyncWebSocket ws_;
   static bool wifi_control_;
    static int linear_x_;
    static int angular_z_;
};