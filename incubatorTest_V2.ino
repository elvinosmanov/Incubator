
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <DS3231.h>
#include "DHT.h"
#include <Adafruit_BMP085.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
int bluePin = 8;    //IN1 on the ULN2003 Board, BLUE end of the Blue/Yellow motor coil
int pinkPin = 9;    //IN2 on the ULN2003 Board, PINK end of the Pink/Orange motor coil
int yellowPin = 10;  //IN3 on the ULN2003 Board, YELLOW end of the Blue/Yellow motor coil
int orangePin = 11;  //IN4 on the ULN2003 Board, ORANGE end of the Pink/Orange motor coil
int relayPin = 6;
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
DHT dht(DHTPIN, DHTTYPE);
DS3231  rtc(SDA, SCL);
Adafruit_BMP085 bmp;
//VARIABLES--------------------
int h;
float t;
bool tempHigh = false;
bool startMotor = false;
bool directionMotor = false;
float setPointT = 37.5;
int currentStep = 0;
int saat=0;
int minute = 0;
int second = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long rememberTurn = 0; 
unsigned long turnInterval = 600000;

//unsigned long turnInterval = (2 * 3600)* 1000;

//testing values:
//interval motorun hereketinden cox olmalidi;

unsigned long motorInterval = 75000;
byte centigrade[8] = {
  0x10,
  0x06,
  0x09,
  0x08,
  0x08,
  0x09,
  0x06,
  0x00
};
byte motorOn[8] = {
  0x00,
  0x00,
  0x11,
  0x1B,
  0x15,
  0x11,
  0x11,
  0x00
};
byte light[8] = { //icon for sun
  0b00000,
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100
};
byte highT[8] ={B00100,B01010,B01010,B01110,B01110,B11111,B11111,B01110}; //thermometer icon
byte highH[8] ={B00100,B00100,B01010,B01010,B10001,B10001,B10001,B01110}; //drop icon

//------------------------------------------------------------------------SETUP------------------------------------------------------
void setup(){
  Serial.begin(9600);
  dht.begin();
  rtc.begin();
  lcd.init();// initialize the lcd 
  lcd.setBacklight(2);
  lcd.createChar(1, centigrade);
  lcd.createChar(2, highT);
  lcd.createChar(3, highH);
  lcd.createChar(4, motorOn);
  lcd.createChar(5, light);

  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  //  rtc.setDOW(SUNDAY);     // Set Day-of-Week to SUNDAY
  //  rtc.setTime(9, 58, 35);     // Set the time to 12:00:00 (24hr format)
  //  rtc.setDate(7, 26, 20);   // Set the date to January 1st, 2014
  }
  pinMode(bluePin, OUTPUT);
  pinMode(pinkPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(orangePin,OUTPUT);
  pinMode(relayPin,OUTPUT);
  
  digitalWrite(bluePin, LOW);
  digitalWrite(pinkPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(orangePin, LOW);
}
//--------------------------------------------------------------------------LOOP------------------------------------------------------
void loop(){
  currentMillis = millis();
  updateSensor();
  showMenu();
  Maintain();
  if(!startMotor){
    if((millis() - rememberTurn) > turnInterval){
    startMotor = true;
    rememberTurn = millis();
  }
  }
  
  turnEgg();
//  lcd.print("rem:");
//  lcd.setCursor(4,1);
//  lcd.print(hourLeft);
//  lcd.setCursor(5,1);
//  lcd.print(":");
//  lcd.setCursor(6,1);
//  lcd.print(minLeft);
 }
// -----------------------------------------------------------TURN EGG----------------------------------------------
 void turnEgg(){
  if(startMotor){
    if((millis() - rememberTurn) <= motorInterval){
      lcd.setCursor(15,1);
      lcd.write(4); 
      step(directionMotor);
      
    }
    else if((millis() - rememberTurn) > motorInterval){
      directionMotor = !directionMotor;
      startMotor = false;
    }
  }
 }
//---------------------------------------------------------------UDPATE SENSOR-----------------------------------------------
 void updateSensor(){
  if ((currentMillis - previousMillis) > 700 )
  {
    h = dht.readHumidity();
    t = bmp.readTemperature();

    if (isnan(h) || isnan(t)) 
    {
      Serial.println(F("Failed to read from sensor!"));
      return;
    }
    else
    {
      if (t >= setPointT+0.2)
      {
        tempHigh = true;
      }
      else if(t <=setPointT-0.2){
        tempHigh = false;
      }
    }
    previousMillis = currentMillis;  
  }
  
}
//---------------------------------------------------------------MAINTAIN-----------------------------------------
void Maintain(){
  if(tempHigh){
    digitalWrite(relayPin, LOW);
//    Serial.print("Relay is not working");
  }
  else{
    digitalWrite(relayPin, HIGH);
    lcd.setCursor(14,1);
    lcd.write(5);
//    Serial.print("Relay is working");
  }
}
//------------------------------------------------------------SHOW MENU------------------------------------------
 void showMenu(){
  if (currentMillis - previousMillis2 > 1000)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.write(2);
  lcd.print(":");
  lcd.print(t,1);
  lcd.write(1);
  
  lcd.setCursor(9,0);
  lcd.write(3);
  lcd.print(":");
  lcd.print(h,1);
  lcd.print("%");
  lcd.setCursor(0,1);
  lcd.print("rem:");
  second = (turnInterval-(currentMillis-rememberTurn))/1000;
  minute = second/60;
  saat = minute/60; 
  lcd.print(saat);
  lcd.print(":");
  lcd.print(minute%60);
  lcd.print(":");
  lcd.print((second%60));
  previousMillis2=currentMillis;

  }
}
//----------------------------------------------------------------STEP MOTOR----------------------------------------
void step(boolean directionMotor){
  switch(currentStep){
    case 0:
      digitalWrite(bluePin, HIGH);
      digitalWrite(pinkPin, LOW);
      digitalWrite(yellowPin, LOW);
      digitalWrite(orangePin, LOW);

      break;
    case 1:
      digitalWrite(bluePin, LOW);
      digitalWrite(pinkPin, HIGH);
      digitalWrite(yellowPin, LOW);
      digitalWrite(orangePin, LOW);

      break;
    case 2:
      digitalWrite(bluePin, LOW);
      digitalWrite(pinkPin, LOW);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(orangePin, LOW);
      break;
    case 3:

      digitalWrite(bluePin, LOW);
      digitalWrite(pinkPin, LOW);
      digitalWrite(yellowPin, LOW);
      digitalWrite(orangePin, HIGH);
      break;
  }
  //CCW
  if(directionMotor){
      currentStep = (--currentStep >= 0) ? currentStep : 3;
  }
  else{
    currentStep = (++currentStep < 4) ? currentStep : 0;
  }
delay(3);
}
