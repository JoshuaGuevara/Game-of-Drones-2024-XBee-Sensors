/*
IMPORTANT Note: UPLOAD CODE TO ARDUINO BEFORE CONNECTING TFMINI-S 
*/
#include <AltSoftSerial.h>
#include <Time.h>
#include <TimeLib.h>
#include <XBee.h>
#include <Printers.h>
//#include <SoftwareSerial.h>
#include <TFMini.h>
#include "binary.h"

#include <PinChangeInterrupt.h>

// TFMini-S pins
#define TFMINI_RX_PIN 0 // RX pin of Arduino (connected to TX of TFMini-S)
#define TFMINI_TX_PIN 1 // TX pin of Arduino (connected to RX of TFMini-S)

//TFMini tfmini;

//SoftwareSerial SerialTFMini(11, 12);

// create the XBee object
XBeeWithCallbacks xbee;
AltSoftSerial SoftSerial;

#define DebugSerial Serial // used to send/receive from MATLAB and/or Arduino serial monitor debugging 
#define XBeeSerial SoftSerial // using software serial for Xbee comms

//XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x41C165F8); // Current Actual Coordinator

//photodiode output - pin 4 input ----------------------
const int inputPin = 4;  // Change this to the pin you want to use (4, 5, 6, or 7)
volatile unsigned long pulseCount = 0;
unsigned long startTime;

//connect pin 13 to A0 ----------------------------------
const int jamVerifyLight = 13; //digital pin 13
const int jamButton = A0; //digital pin 14

long duration;
int val;
// ----------------------------------------------------------

// Payload 
int sensorID = 12; //////////////////////////////////////////////////////////// UPDATE ID #
int distance = 0;
time_t t = 0;

long lastSeen = 0; //Tracks the time when we last detected a tag

// For receiving real-world time & updating Arduino internal clock
String currTime = "00:00:00";
uint32_t currT = 0;


int getTFminiData() {
  static char i = 0;
  char j = 0;
  int checksum = 0; 
  static int rx[9];
  int distance = 0;
  
  if(Serial.available()) {  
    rx[i] = Serial.read();
    
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        distance = rx[2] + rx[3] * 256;
      }
      i = 0;
    } else {
      i++;
    } 
  }  

  return distance;
}

void zbReceive(ZBRxResponse& rx, uintptr_t) {
  // Establish memory buffer 
  Buffer b(rx.getData(), rx.getDataLength()); 

  // Note b.len() should be equal to the amount of data bytes expected in the payload
  if(b.len() == 5) //this number MUST match send allocation buffer 
  {
    // Remove each data item from the payload and then save them into temp variables    
    uint8_t type = b.remove<uint8_t>();
    currT = b.remove<uint32_t>();

    // Cast time received from MATLAB as a double to feed into the setTime() fcn
    currT = (double)currT;
    
    if (type == 1) 
    {
      // Update Arduino internal clock
      updateCurrTimeStr();
      return;
    }
    
  }

  DebugSerial.println(F("Unknown or invalid packet"));
  printResponse(rx, DebugSerial);
}


void setup() {
  // Define Jammer Inputs/Outputs ----------------------------
  pinMode(jamVerifyLight, OUTPUT);
  digitalWrite(jamVerifyLight, LOW); 
  pinMode(jamButton, INPUT); 
  pinMode(inputPin, INPUT);
  attachPCINT(digitalPinToPCINT(inputPin), countPulse, RISING);
  startTime = millis();
  
  // Step 1: Initialize hardware serial port (serial debug port)
  //Serial.begin(115200);
  DebugSerial.begin(115200);
  
  XBeeSerial.begin(9600);
  xbee.setSerial(XBeeSerial);
  delay(1);

  // Make sure that any errors are logged to Serial. The address of
  // Serial is first cast to Print*, since that's what the callback
  // expects, and then to uintptr_t to fit it inside the data parameter.
  xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&DebugSerial);
  xbee.onTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&DebugSerial);
  xbee.onZBRxResponse(zbReceive);
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial);
     
  DebugSerial.println ("Initializing...");

  // Step 2: Initialize the data rate for the SoftwareSerial port
  //SerialTFMini.begin(TFMINI_BAUDRATE);

  // Step 3: Initialize the TF Mini sensor
  //tfmini.begin(&SerialTFMini);    

}

void updateCurrTimeStr()
{
  setTime(currT); 
}

void xbeeSendPacket(int distance){
  //DebugSerial.println("Sending packet");
  
  ZBTxRequest txRequest;
  //txRequest.setAddress64(0x0013A20041C19C0F); // address of test coordinator
  txRequest.setAddress64(0x0013A20041C165F8); // address of actual coordinator

  // Allocate <X> payload bytes: 1 type byte plus however many other data bytes needed
  AllocBuffer<5> packet; // <X> = 1 1-byte (uint_8) + 5 2-bytes (integers)
  
  // Packet type & payload data items
  // format is:
  // |type (1)| |sensorID| |distance|
  packet.append<uint8_t>(1);
  packet.append<int>(sensorID);
  packet.append<int>(distance);
  
  // Set payload
  txRequest.setPayload(packet.head, packet.len());

  // Send the packet
  xbee.send(txRequest);
  
}

// Jam Packet Function -----------------------------------------------
void sendJamPacket(){
  val = digitalRead(jamButton);
  // For more in-depth code explaination, read through sendDistancePacket(), as
  // it is very similar to this fcn.

  ZBTxRequest txRequest;
  txRequest.setAddress64(0x0013A20041C165F8); // address of Mario coordinator

  AllocBuffer<7> packet;

  int targetType = 0; // 0 is all sensor types targeted
  int targetID = 12;   // 0 is all sensor ids targeted ////////////////////////////////////////////////// UPDATE

  // Construct a jam packet in the format:
  // |type (3)| |sensorID| |target type| |target id|
  packet.append<uint8_t>(3);
  packet.append<int>(sensorID);
  packet.append<int>(targetType);
  packet.append<int>(targetID);
  
  // prepare the packet
  txRequest.setPayload(packet.head, packet.len());

  // send the packet over the xbee network
  xbee.send(txRequest);
}

void xbeeSendNullPacket(){
  // For debugging purposes 
  DebugSerial.println("Sending NULL packet");
  
  // Allocate <X> payload bytes: 1 type byte plus however many other data bytes needed
  // Prepare the Zigbee Transmit Request API packet
  ZBTxRequest txRequest;
  //txRequest.setAddress64(0x0013A20041C19C0F); // address of test coordinator
  txRequest.setAddress64(0x0013A20041C165F8); // address of actual coordinator


  // Allocate <X> payload bytes: 1 type byte plus however many other data bytes needed
  AllocBuffer<3> packet; // <X> = 1 1-byte (uint_8) + 5 2-bytes (integers)
  
  // Packet type & payload data items
  packet.append<uint8_t>(2);
  packet.append<int>(sensorID); 

  // Set payload
  txRequest.setPayload(packet.head, packet.len());

  // Send the packet
  xbee.send(txRequest);

}

unsigned long last_tx_time = 0; //must use to avoid potential latency issues with xbeeloop()
// **** Sending packet on an infinite loop ****
void loop() {
  xbee.loop();

  val = digitalRead(jamButton);

  unsigned long currentTime = millis();  // Current time in milliseconds
  unsigned long elapsedTime = currentTime - startTime;
  
  distance = getTFminiData();
  
  if(distance && (millis() - lastSeen) > 100) {

    float frequency = (pulseCount * 1000.0) / (millis() - lastSeen);  // Pulses per second

    if(frequency > 90 && frequency < 110) { // jams sensor when diode read 90-110 Hz (Can be adjusted)
      //digitalWrite(jamVerifyLight, HIGH);
      sendJamPacket();    
    } 
    else if (distance < 175){ // Max dist: 175cm (can be adjusted)
      xbeeSendPacket(distance);
    } 

    if (frequency > 20){
      Serial.print(frequency);
      Serial.println(" Hz");
    } else {
      Serial.print(distance);
      Serial.println(" cm");
    }
      
    distance = 0;

    pulseCount = 0;
    startTime = currentTime;
      
    // Timestamp when distance was received
    t = now();
      
    lastSeen = millis();
  }
    
  if((millis() - lastSeen) > 3000){
    xbeeSendNullPacket();
    lastSeen = millis();
  }
  //delay(100);
}

void countPulse(){
  pulseCount++;
}

/*
Future work: 
  -Read Freq at a faster/accurate rate, match speed of sensor (now receives data every 100ms)
  -Implement Jamming code to ultrasonic sensors
  -Organize arduino code for TFMini-S, TFLuna, LIDAR-Lite v3
  -Implement easy way to change frequency values, maybe add a switch to arduino to preset freq
*/