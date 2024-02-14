//LoRa libraries
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 6
#define RFM95_INT 7
#define RF95_FREQ 915.0

//Wifi and HHTP libraries
#include <HttpClient.h>
#include <WiFiNINA.h>

//char ssid[] = "GORMAN";
//char pass[] = "Kaipothedogwashere1!";

//Wifi SSID and password
char ssid[] = "VM4425032";
char pass[] = "ckp5rqynKdQv";

//char ssid[] = "leah";
//char pass[] = "123456789";

//Start WIFI
int status = WL_IDLE_STATUS;

WiFiClient client;

IPAddress server(192, 168, 0, 101);  // IP address of localhost found from ipconfig
HttpClient http(client, server, 4534);

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

// Set up variable to hold sheepData
String Temp;
String Activity;
String BPM;
String Contract;
String prevData;
String Battery;

void setup() {
  // connect to WIFI
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  delay(1000);

  //Set up LoRa
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial)
    ;
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  //23dB
  rf95.setTxPower(23, false);
}

void loop() {
  delay(1000);
  //checks for message
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      String received_id = "X";
      Serial.println("string test:");
      Serial.println(received_id + (char*)buf);

      String myString = String((char*)buf);
      Serial.println(myString);
      const char* receivedData = myString.c_str();
      String ReceivedString = receivedData;


      //Decode Received message
      for (int i = 0; i < ReceivedString.length(); i++) {
        switch (ReceivedString[i]) {
          case 'T':
            Temp = String(ReceivedString[i + 1]);
            Temp = Temp + String(ReceivedString[i + 2]);
            break;
          case 'A':
            Activity = String(ReceivedString[i + 1]);
            break;
          case 'H':
            BPM = String(ReceivedString[i + 1]);
            BPM = BPM + String(ReceivedString[i + 2]);
            break;
          case 'C':
            Contract = String(ReceivedString[i + 1]);
            Contract = Contract + String(ReceivedString[i + 2]);
            break;
          case 'B':
            Battery = String(ReceivedString[i + 1]);
            Battery = Battery + String(ReceivedString[i + 2]);
            break;
        }
      }

      // Send a reply
      uint8_t data[] = "Message Received";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);


      //Change values into Json string
      Serial.println("SendData");
      String sendData = "{\"sheepId\": \"string\",\"temperature\": " + Temp + ",\"accelerometer\": " + Activity + ",\"contractionTime\": " + Contract + ",\"heartRate\": " + BPM + ",\"batteryLife\": " + Battery + "}";
      Serial.println(sendData);

      Serial.println("Making POST request...");

      http.beginRequest();
      http.post("/api/Alert");
      http.sendHeader("Content-Type", "application/json");
      http.sendHeader("Content-Length", sendData.length());

      http.sendHeader("Host", "192.168.0.101:4534");
      http.beginBody();
      http.print(sendData);
      http.endRequest();

      int statusCode = http.responseStatusCode();
      String response = http.responseBody();

      Serial.print("Status Code: ");
      Serial.println(statusCode);
      Serial.print("Response: ");
      Serial.println(response);

    } else {
      Serial.println("Receive failed");
    }
  }
}