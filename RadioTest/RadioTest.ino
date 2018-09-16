/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read new NUID from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * Example sketch/program showing how to the read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the type, and the NUID if a new card has been detected. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 */

#include <SPI.h>
#include <MFRC522.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(2, 3); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

#define rgbRed    6
#define rgbGreen  5
#define rgbBlue   4
#define buttonPin 7
#define buttonLed 8
#define SS_PIN    10
#define RST_PIN   9
#define battery   A0
#define _DEBUG

bool button;
bool buttonWasPressed;
bool paused;

unsigned long currentTime;
unsigned long buttonStartTime;
unsigned long buttonEndTime;
unsigned long batteryTime;

String uid;
 
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class

MFRC522::MIFARE_Key key; 

// Init array that will store new NUID 
byte nuidPICC[4];

void setup() 
{ 
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  pinMode(buttonLed, OUTPUT);
  pinMode(rgbGreen, OUTPUT);
  pinMode(rgbBlue, OUTPUT);
  pinMode(rgbRed, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(battery, INPUT);

  analogWrite(rgbGreen, 10);

  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  Serial.println(F("This code scan the MIFARE Classsic NUID."));
  Serial.print(F("Using the following key:"));
  printHex(key.keyByte, MFRC522::MF_KEY_SIZE);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.setTimeOut(500);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  myDFPlayer.volume(15);  //Set volume value (0~30)
  delay(100);
  paused = true;
}

int getBatteryVoltage() {
  int batteryVoltage = 0;
  for (int n = 0; n < 20; ++n) {
    batteryVoltage += analogRead(battery);
  }
  return batteryVoltage / 20;
}

void setBatteryLed() {
  int batteryConstrained = constrain(getBatteryVoltage(), 80, 800);
  int intensity = map(batteryConstrained, 80, 800, 0, 25);
  analogWrite(rgbGreen, intensity);
  analogWrite(rgbRed, 25 - intensity);
} 

float getVoltageFromAnalog(int analogValue) {
  return analogValue * 0.00488759;
}
 
void loop() 
{
  button = digitalRead(buttonPin);
  int lastRead = millis();

  if (myDFPlayer.available()) {
    Serial.println(".");
  }

  if (button and not buttonWasPressed){
    buttonWasPressed = true;
    buttonStartTime = millis();
    Serial.println("Taster wird gedrückt.");
    
  } else if (not button and buttonWasPressed){
    if (paused){
      myDFPlayer.start();
      paused = false;
    } else {
      myDFPlayer.pause();  //pause the mp3
      paused = true;
    }
    buttonWasPressed = false;
    buttonEndTime = millis();
    int timePressed = buttonEndTime - buttonStartTime;
    Serial.print("Der Taster wurde: ");
    Serial.print(timePressed);
    Serial.println(" Millisekunden gedrückt.");
    if (timePressed > 3000){
      myDFPlayer.next();
      paused = false;
    }
  }

  if (paused){
    digitalWrite(buttonLed, LOW);
  } else {
    digitalWrite(buttonLed, HIGH);
  }
  
  if (uid == "14424591212"){
    myDFPlayer.play(1);
    paused = false;
    uid = "";
  }
  if (uid == "192160120212"){
    myDFPlayer.play(2);
    Serial.println(myDFPlayer.readCurrentFileNumber(DFPLAYER_DEVICE_SD));
    paused = false;
    uid = "";
  }
  if (uid == "22416692212"){
    myDFPlayer.play(3);
    paused = false;
    uid = "";
  }
  if (uid == "96181228164"){
    myDFPlayer.pause();  //pause the mp3
    paused = true;
    uid = "";
  }
  if (uid == "19223171212"){
    myDFPlayer.start();
    paused = false;
    uid = "";
  }
  
  // Look for new cards
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()){
    digitalWrite(rgbBlue, HIGH);
    delay(50);
    digitalWrite(rgbBlue, LOW);
    readUID();
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

}


void readUID(){
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));

  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }

  if (rfid.uid.uidByte[0] != nuidPICC[0] || 
    rfid.uid.uidByte[1] != nuidPICC[1] || 
    rfid.uid.uidByte[2] != nuidPICC[2] || 
    rfid.uid.uidByte[3] != nuidPICC[3] ) {
    Serial.println(F("A new card has been detected."));

    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
    }
   
    Serial.println(F("The NUID tag is:"));
    Serial.print(F("In hex: "));
    printHex(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
    Serial.print(F("In dec: "));
    printDec(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
  }
  else Serial.println(F("Card read previously."));

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
}

/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
void printDec(byte *buffer, byte bufferSize) {
  uid = "";
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
    uid += (buffer[i] < 0x10 ? " 0" : "");
    uid += (buffer[i]);
  }
  Serial.println("");
  Serial.println(uid);
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}




