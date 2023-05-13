#include <TinyGPS++.h>      //NMEA(GPS) parsing library
//#include <SharpIR.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Arduino SPI library#include <Adafruit_GFX.h>



const unsigned char UBLOX_INIT[] PROGMEM = {
  // Rate (pick one)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 //(1Hz)
};

//-----------------------------TFT Display Parameters--------------------------------------

// ST7789 TFT display pin map for MKR 1000
#define TFT_CS    -1  // define chip select pin
#define TFT_DC     4  // define data/command pin
#define TFT_RST    5  // define reset pin, or set to -1 and connect to Arduino RESET pin

//Pin map + instantiate display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


//-------------------------------------------------------------------------------------------



//-----------------------------Distance Sensor Parameters-------------------------------------

//Sharp 2Y0A710 IR sensor pin map  
#define leftSensorPin  A0
#define rightSensorPin  A2

//instantiate left/right sensor of SharpIR class
//SharpIR leftSensor = SharpIR(leftSensorPin, 100500);
//SharpIR rightSensor = SharpIR(rightSensorPin, 100500);

//threshold value (in cm) that sensor will use to
// determine if object is too close and trigger alert
const int THRESHOLD = 120;

//-------------------------------------------------------------------------------------------



//Instantiate TinyGPSPlus class
TinyGPSPlus gps;

//function prototypes
void SpeedometerTask();
void DetectionTask();
void DisplaySpeed();
void displayspeed();

void setup() {


Serial.begin(9600);




Serial1.begin(9600);
  // send configuration data in UBX protocol
  for(unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    Serial1.write( pgm_read_byte(UBLOX_INIT+i) );
  }


 // changeBaudrate();
  //delay(100); Serial1.flush();
//Serial1.begin(19200);   //new 115k boost mode

// Initialize ST7789 display 240x240 pixel w/ SPI
  tft.init(240, 240, SPI_MODE2);  
 
  // Screen orientation
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(5);
  tft.println("ON");
  

}

void loop() {

  SpeedometerTask();
  
  DetectionTask();

}


//------------------------------------------------------------------
//This function polls Serial1 (pin 13)for RX from the GPS module
//On successful RX the speed data is printed to the TFT display
//------------------------------------------------------------------
void SpeedometerTask(){
  
  if (Serial1.available() > 0){
  tft.setCursor(40, 70);
      
  if (gps.encode(Serial1.read())){
    //tft.print("gps reading");
      displayspeed();
    }
  }
  
  }



//--------------------------------------------------------------------------------------------------
//This function polls the distance sensors to check for objects closer than the threshold
//  Once detected, arrow indicators are sent to TFT display to alert operator
//  
// For any object detected being too close, a set sequence of flashes happens using timer comparison
//   to keep from being loop locked. Once the sequence is done, the flag is then set low to enable
//   further detection.
//
// vars:
//        detectFlag            —     signals whether object has been detected
//
//        leftDetectTime        —     captures when first detection occured (left) 
//                                     and is also used to compare time for flasing the arrows
//
//        rightDetectTime       —     captures when first detection occured (right)
//                                     and is also used to compare time for flasing the arrows
//
//        arrowState            —     state of the arrow on display: 1 = on, 0 = off
//
//        arrowCounter          —     tracks one on or one off transition of the arrow 
//    
//-------------------------------------------------------------------------------------------------
 void DetectionTask(){
  static bool leftDetectFlag = false;
  static int leftDetectTime, leftSensorVal, leftDistance;
  static int leftArrowState, leftArrowCounter = 0;
  
  static bool rightDetectFlag = false;
  static int rightDetectTime, rightSensorVal, rightDistance;
  static int rightArrowState, rightArrowCounter = 0;


    Serial.print("right dis = ");
    Serial.println(rightDistance);
 
  //if rightDetectFlag is low check the sensor
  if(!rightDetectFlag){

 
    //poll the sensor
    rightSensorVal = analogRead(rightSensorPin);
    rightDistance = 28250/(rightSensorVal - 229.5);
       //Serial.println(rightDistance);
       //delay(200);
  //if right sensor detects object too close, raise the rightDetectFlag
  if(rightDistance < 115 && rightDistance >= 0){
      //Serial.println("distance crossed");
      rightDetectFlag = true;
      rightDetectTime = millis();
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(100,200);
      tft.print(">>>");
      rightArrowState = 1;
         
    }
 }

  
  //else if rightDetectFlag is high
  else {

    //if arrow has been on or off for 200ms, change state of display to make arrow flash
      if(millis() - rightDetectTime > 300){

          //Serial.println("timer met");
          //Serial.println(rightArrowState);

    switch(rightArrowState) {

      case 1 :
          rightArrowState = 0;
          tft.setTextColor(ST77XX_BLACK);
          tft.setCursor(100,200);
          //Serial.println("Print BLACK");
          tft.print(">>>");
          rightArrowCounter++; 
          break;
  
      case 0 :
          rightArrowState = 1;
          tft.setTextColor(ST77XX_GREEN);
          //Serial.println("Print GREEN");
          tft.setCursor(100,200);
          tft.print(">>>");
          rightArrowCounter++; 
  
       /* you can have any number of case statements */
      default : /* Optional */
      rightArrowState = 1;
}
          rightDetectTime = millis();  
      }  
  }

  //check if the arrow has flashed the appropriate amount of times
  // if so, reset rightArrowCounter and lower the rightDetectFlag
  // so that the sensor can be polled again
  if(rightArrowCounter >= 5){
    rightDetectFlag = false;
    rightArrowCounter = 0;   
    }

    



    //Serial.print("left dis = ");
    //Serial.println(leftDistance);
 
  //if leftDetectFlag is low check the sensor
  if(!leftDetectFlag){


    //poll the sensor
    leftSensorVal = analogRead(leftSensorPin);
    leftDistance = 28250/(leftSensorVal - 229.5);

  //if left sensor detects object too close, raise the leftDetectFlag
  if(leftDistance < 80){
//      Serial.println("initial left time set");
      leftDetectFlag = true;
      leftDetectTime = millis();
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(0,200);
      tft.print("<<<");
      leftArrowState = 1;
         
    }
  }

  
  //else if leftDetectFlag is high
  else {

    //if arrow has been on or off for 200ms, change state of display to make arrow flash
      if(millis() - leftDetectTime > 300){

//          Serial.println("timer met");
//          Serial.println(leftArrowState);

    switch(leftArrowState) {

      case 1 :
          leftArrowState = 0;
          tft.setTextColor(ST77XX_BLACK);
          tft.setCursor(0,200);
//          Serial.println("Print BLACK");
          tft.print("<<<");
          leftArrowCounter++; 
          break;
  
      case 0 :
          leftArrowState = 1;
          tft.setTextColor(ST77XX_GREEN);
//          Serial.println("Print GREEN");
          tft.setCursor(0,200);
          tft.print("<<<");
          leftArrowCounter++; 
  
       
      default : 
      leftArrowState = 1;
}
          leftDetectTime = millis();  
      }  
  }

  //check if the arrow has flashed the appropriate amount of times
  // if so, reset leftArrowCounter and lower the leftDetectFlag
  // so that the sensor can be polled again
  if(leftArrowCounter >= 5){
    leftDetectFlag = false;
    leftArrowCounter = 0;   
    }

    
}

void displayspeed()
{
  static double previousReading = 0.00;

  if (gps.speed.isValid())
  {

    tft.setTextSize(5);
    tft.setCursor(40, 90);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(previousReading);

    tft.setTextSize(5);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(40, 90);

    previousReading = gps.speed.mph();
    tft.print(previousReading);       

    
 }
  else
  {
    tft.setTextSize(5);
    tft.setCursor(40, 90);
   // tft.print("No Data!!!");
//    Serial.print("No Data!!!");
 
  }
  
}


// Send a packet to the receiver to change baudrate to 115200.
void changeBaudrate() {
    // CFG-PRT
    byte packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x00, // id
        0x14, // length
        0x00, // 
        0x01, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0xD0, // payload
        0x08, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0x4B, // payload
        0x00, // payload
        0x00, // payload
        0x07, // payload
        0x00, // payload
        0x03, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        
        0xC0, // CK_A
        0x7E, // CK_B
    };
    sendPacket(packet, sizeof(packet));
}


void sendPacket(byte *packet, byte len) {
    for (byte i = 0; i < len; i++)
    {
        Serial1.write(packet[i]);
    }
}
