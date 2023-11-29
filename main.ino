/***********************************************************************
  Adafruit MQTT Library ESP32 Adafruit IO SSL/TLS example

  Use the latest version of the ESP32 Arduino Core:
    https://github.com/espressif/arduino-esp32

  Works great with Adafruit Huzzah32 Feather and Breakout Board:
    https://www.adafruit.com/product/3405
    https://www.adafruit.com/products/4172

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  Modified by Brent Rubell for Adafruit Industries
  MIT license, all text above must be included in any redistribution
 **********************************************************************/
#include <WiFi.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <SoftwareSerial.h>
#include <Thread.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/************************* FreeRTOS *********************************/
// Khai báo hàm cho các task
void writeSignal(void *parameter);
void readDataSensor(void *parameter);

// Định nghĩa task handlers
TaskHandle_t writeSignalHandle = NULL;
TaskHandle_t readDataSensorHandle = NULL;

/************************* Software Serial *********************************/

#define RX_PIN 18
#define TX_PIN 19

SoftwareSerial mySerial(RX_PIN, TX_PIN);

/************************* WiFi Access Point *********************************/

#define WLAN_SSID "Redmi Note 9S"
#define WLAN_PASS "11102002"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER "io.adafruit.com"

// Using port 8883 for MQTTS
#define AIO_SERVERPORT 8883

// Adafruit IO Account Configuration
// (to obtain these values, visit https://io.adafruit.com and click on Active Key)
#define AIO_USERNAME "haole1110"
#define AIO_KEY "aio_ZLBh61asqnKW4oWnfRsQCjby6YXB"

/************ Global State (you don't need to change this!) ******************/

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// io.adafruit.com root CA
const char* adafruitio_root_ca =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
  "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
  "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
  "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
  "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
  "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
  "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
  "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
  "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
  "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
  "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
  "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
  "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
  "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
  "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
  "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
  "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
  "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
  "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
  "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
  "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
  "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
  "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
  "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
  "-----END CERTIFICATE-----\n";

/****************************** Feeds ***************************************/

// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish door = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/door");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");

Adafruit_MQTT_Subscribe led = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led");
Adafruit_MQTT_Subscribe thief = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/thief");

/*************************** Sketch Code ************************************/

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(10);

    // Tạo các task
  xTaskCreatePinnedToCore(writeSignal,"Task1",10000,NULL,1,&writeSignalHandle,0);  delay(500);   
  xTaskCreatePinnedToCore(readDataSensor,"Task2",10000,NULL,1,&readDataSensorHandle,1);  delay(500); 

  // Cài đặt chân RX/TX
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  Serial.println(F("Adafruit IO MQTTS (SSL/TLS) Example"));

  // Connect to WiFi access point.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);
  mqtt.subscribe(&led);
  mqtt.subscribe(&thief);
}

uint32_t x = 0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.

  // MQTT_connect();
  // readThread.run();
  // writeThread.run();

  //writeSignal();
  //readDataSensor();

  // wait a couple seconds to avoid rate limit
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1)
        ;
    }
  }

  Serial.println("MQTT Connected!");
}

static String temp1 = "", temp2 = "", temp3 = "";

void readDataSensor(void *parameter) {
  while (true){
    MQTT_connect();

    // Serial.println(0);
    if (Serial.available() > 0) {
      // Serial.println(1);
      String string_data = "";
      char data = Serial.read();
      if (data == '!') {
        // Serial.println(2);
        string_data += data;
        while (data != '#' && string_data.length() < 10) {
          // Serial.println(3);
          if (Serial.available() > 0) {
            data = Serial.read();
            string_data += data;
          }
        }
      }

      Serial.println(string_data);

      if (string_data[string_data.length() - 1] != '#') return;

      char feed_data = string_data[1]; 
      string_data = string_data.substring(2, string_data.length() - 1);

      // temp1 = temp2;
      // temp2 = temp3;
      // temp3 = string_data;



      if (string_data == "") return;
      Serial.println(string_data);

      if (1) {
        switch(feed_data) {
        case 'D':
          // Now we can publish stuff!
          Serial.print(F("\nSending val "));
          Serial.print(string_data);
          Serial.print(F(" to Door feed..."));
          if (!door.publish(&string_data[0])) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
            temp1 = "";
            temp2 = "";
            temp3 = "";
          }
          break;
        case 'H':
          // Now we can publish stuff!
          Serial.print(F("\nSending val "));
          Serial.print(string_data);
          Serial.print(F(" to humidity feed..."));
          if (!humidity.publish(&string_data[0])) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
            temp1 = "";
            temp2 = "";
            temp3 = "";
          }
          break;
        case 'T':
          // Now we can publish stuff!
          Serial.print(F("\nSending val "));
          Serial.print(string_data);
          Serial.print(F(" to temperature feed..."));
          if (!temperature.publish(&string_data[0])) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
            temp1 = "";
            temp2 = "";
            temp3 = "";
          }
          break;
        default:
          break;
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 1 giây
  }
}

void writeSignal(void *parameter) {
  while (true){
    MQTT_connect();

    Adafruit_MQTT_Subscribe* subscription;
    while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &led) {
        Serial.print("Received data from Led feed: ");
        String data_recieved = "!L" + String((char*)led.lastread) + "#";
        Serial.println(data_recieved);
        // Xử lý dữ liệu ở đây (nếu cần)

      }

      if (subscription == &thief) {
        Serial.print("Received data from Thief feed: ");
        String data_recieved = "!F" + String((char*)thief.lastread) + "#";
        Serial.println(data_recieved);
        // Xử lý dữ liệu ở đây (nếu cần)
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 1 giây
  }
}