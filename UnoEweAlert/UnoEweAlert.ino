// Setting up Lora
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Pulse Sensor
float BPM = 60;
float BPM_temp = 60;
int PulseSensorPurplePin = A0;  // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int Signal;                     // holds the incoming raw data. Signal value can range from 0-1024
int Threshold_low = 600;
int Threshold_high = 520;
bool next = true;
float Time = 0;
float prev = 0;

//Accelerometer
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();
int Activity = 0;
uint8_t prevOrientation = 0;

//Battery setup
#define BLUE 6
#define GREEN 7
#define RED 5
#define battery A1
int batteryPercent = 90;

int temp = 20;
//strap led
#define strap_LED 8

//Contraction sensor
#include <ArduinoJson.h>

//This should be the same value of the used resistor
#define RESISTOR 10000

//This is the pin where the cord is connected to
#define RUBBERCORDPIN A0

int value=0;
int raw = 0;             // Raw input value
int vin = 5;             // Store input voltage, this should be 5
float vout = 0;          // Store output voltage
float refresistor1 = 1;  // Variable to store the R1 value
float refresistor2 = 0;  // represents unknown resistor.
float buffer = 0;        // buffer variable for calculation

bool next_contract = true;
int contraction = 0;
unsigned long time_between_cont = 0;
unsigned long previous_contract = 0;
unsigned long previous_time = 0;

int contractionTime = 10;

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial)
    ;
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa TX Test!");

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

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);



  //accelerometer
  Serial.begin(9600);

  Serial.println("Adafruit MMA8451 test!");


  if (!mma.begin()) {
    Serial.println("Couldnt start");
  }
  Serial.println("MMA8451 found!");

  mma.setRange(MMA8451_RANGE_2_G);

  Serial.print("Range = ");
  Serial.print(2 << mma.getRange());
  Serial.println("G");

  //Set up LED's
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(strap_LED, OUTPUT);

  digitalWrite(BLUE, HIGH);
}
// packet counter, we increment per xmission
int16_t packetnum = 0;

void loop() {
  //convert each value to  a char array
  char Temp_char[3];
  itoa(temp, Temp_char, 10);

  char Activity_char[2];
  itoa(Activity, Activity_char, 10);

  char Heart_char[3];
  itoa(BPM, Heart_char, 10);

  char Contract_char[3];
  itoa(contractionTime, Contract_char, 10);

  char Battery_char[3];
  itoa(batteryPercent, Battery_char, 10);

  //Set up identifiers
  char radiopacket[45];
  const char T[] = "T";
  const char A[] = "A";
  const char H[] = "H";
  const char C[] = "C";
  const char B[] = "B";

  //Add together to one char array
  strcpy(radiopacket, T);
  strcat(radiopacket, Temp_char);
  strcat(radiopacket, A);
  strcat(radiopacket, Activity_char);
  strcat(radiopacket, H);
  strcat(radiopacket, Heart_char);
  strcat(radiopacket, C);
  strcat(radiopacket, Contract_char);
  strcat(radiopacket, B);
  strcat(radiopacket, Battery_char);

  //send data
  Serial.println("SendDATa:");
  Serial.print(radiopacket);

  Serial.println("Sending to rf95_server");

  Serial.print("Sending ");
  Serial.println(radiopacket);
  radiopacket[19] = 0;

  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  delay(10);
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char *)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }

  //Check threshold
  if (Activity > 3) {
    digitalWrite(strap_LED, HIGH);
  }


  ///Battery status check
  //get value from analog pin
  int sensorValue = analogRead(battery);
  //convert value to voltage
  float voltage = sensorValue * (5.00 / 1023.00) * 2;
  //convert to peercentage
  batteryPercent = (voltage * 100) / 9;
  Serial.print("battery:");
  Serial.println(batteryPercent);

  //checks if battery is low or high
  if (voltage < 8.00) {
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
  } else {
    digitalWrite(GREEN, HIGH);
    digitalWrite(RED, LOW);
  }

  //accelerometer
  // Read the 'raw' data in 14-bit counts
  mma.read();
  Serial.print("X:\t");
  Serial.print(mma.x);
  Serial.print("\tY:\t");
  Serial.print(mma.y);
  Serial.print("\tZ:\t");
  Serial.print(mma.z);
  Serial.println();

  //Get a new sensor event
  sensors_event_t event;
  mma.getEvent(&event);

  //Display the results (acceleration is measured in m/s^2)
  Serial.print("X: \t");
  Serial.print(event.acceleration.x);
  Serial.print("\t");
  Serial.print("Y: \t");
  Serial.print(event.acceleration.y);
  Serial.print("\t");
  Serial.print("Z: \t");
  Serial.print(event.acceleration.z);
  Serial.print("\t");
  Serial.println("m/s^2 ");

  //Get the orientation of the sensor
  uint8_t o = mma.getOrientation();
  //if its different then increase the activity
  if (prevOrientation != o) {
    Activity++;

    if (Activity == 10) {
      Activity = 0;
    }
    prevOrientation = o;

    Serial.print("Activity:");
    Serial.println(Activity);
  }


  //Pulse Sensor
  float startPulseTime = millis();
  unsigned long Time = millis();
  //while((Time-startPulseTime)<5000){

  //Read in value from PulseSensor
  int Signal = analogRead(PulseSensorPurplePin);
  if ((Signal > Threshold_high) && (next == true)) {
    next = false;
    Time = millis() - prev;
    prev = millis();
    //Calculate BPM based on time between beats
    BPM_temp = 60000 / Time;
    if (BPM_temp > 60 && BPM_temp < 100) {
      BPM = BPM_temp;
      Serial.print("BPM:");
      Serial.println(BPM);
    }
    Time = millis();
  }
  if ((Signal < Threshold_low) && (next == false)) {
    next = true;
  }

  //Temporary place holder for temperature
  if (temp > 20) {
    temp--;
  } else {
    temp++;
  }

  //Contraction
  //read value
  value = analogRead(RUBBERCORDPIN);
  //calculate voltage on pin
  vout = (5.0 / 1023.0) * value;
  buffer = (vin / vout) - 1;
  refresistor2 = refresistor1 / buffer;

  //Output values
  Serial.print("Voltage: ");
  Serial.println(vout);
  Serial.print("Resistance: ");
  Serial.println(refresistor2);

  //if sheet is pulled and as been let go in between
  if ((refresistor2 > 7) && (next_contract == true)) {
    contraction = contraction++;
    next_contract = false;
    unsigned long time_now = millis();
    time_between_cont = time_now - previous_contract;
    previous_contract = time_now;

    //ouput data
    Serial.println("time now:");
    Serial.println(time_now);
    Serial.println("TIME BETWEEN:");
    Serial.println(time_between_cont / (1000));

    //calculate in terms of percentage with 10 minutes =0%, 1 min=99% and check is 2 char
    int contraction_Val = ((time_between_cont / (1000)) / (60) * 100);
    contraction_Val = 100 - contraction_Val;
    if (((time_between_cont / (1000)) < 99) && ((time_between_cont / (1000)) > 10)) {
      int contractionTime = (time_between_cont / (1000));
    }
  } else {
    if (refresistor2 < 7) {
      next_contract = true;
    }
  }

  delay(1000);
}
