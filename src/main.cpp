
#include <Arduino.h>

#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)



#include <pins_arduino.h>
#include <Wire.h>
// #include <EEPROM.h>
#include <ArduinoUniqueID.h>


#define ENABLE_WATER_RELAYS 1
#define ENABLE_BME280 1
#define ENABLE_BH1750 1
#define ENABLE_LED 1


#define ENABLE_BUZZER 59


#ifdef ENABLE_BME280
  #include "Adafruit_Sensor.h"
  #include "Adafruit_BME280.h"
  Adafruit_BME280 bme; // I2C
  #define SEALEVELPRESSURE_HPA (1013.25)
#endif



#ifdef ENABLE_WATER_RELAYS
  #define PIN_WATER_SENSOR1_PIN_A 54
  #define PIN_WATER_SENSOR1_PIN_D (2)
  
  #define PIN_WATER_SENSOR2_PIN_A 55
  #define PIN_WATER_SENSOR2_PIN_D (3)
  
  #define PIN_RELAY1  4
  #define PIN_RELAY2  5
  #define PIN_RELAY3  6
  #define PIN_RELAY4  7
  
#endif


#ifdef ENABLE_BH1750
  
  #include "BH1750Lib.h"
  
  BH1750Lib lightmeter;

#endif


#ifdef ENABLE_LED

  #include <FastLED.h>
  
  #define PIXEL_COUNT_RING 232
  #define PIXEL_COUNT_SIDE 45
  CRGB leds[PIXEL_COUNT_RING];
  CRGB leds_side[PIXEL_COUNT_SIDE];

  #define PIN_LED_RING  8
  #define PIN_LED_STRIP1  9
  #define PIN_LED_STRIP2  10
  #define PIN_LED_STRIP3  11
  #define PIN_LED_STRIP4  12
  
  int rgb0[3] = {0,0,255};
  int rgb1[3] = {255,0,0};
  int rgb_alpha = 0;
  int rgb_ratio = 1;
  


CRGB Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
  return CRGB(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
  return CRGB(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return CRGB(WheelPos * 3, 255 - WheelPos * 3, 0);
}

#endif


void(* resetFunc) (void) = 0;


String sID = "";
// int checksum = 0;

// String dID = "";

// #include "pitches.h"


unsigned long previousMillisA = 0;
const long intervalA = 5000;


unsigned long previousMillisB = 0;
const long intervalB = 1000;

unsigned long previousMillisC = 0;
const long intervalC = 1000;


void setup()
{
  
  Serial.begin(115200);
  
  analogReadResolution(12);
  
  pinMode(57, OUTPUT);
  // analogWrite(57,4096);
  analogWrite(57,0);

  #ifdef ENABLE_BUZZER
    pinMode(ENABLE_BUZZER,OUTPUT);
    digitalWrite(ENABLE_BUZZER,HIGH);
    delay(1000);        // ...for 1 sec
    digitalWrite(ENABLE_BUZZER,LOW);
    delay(1000);        // ...for 1 sec
    digitalWrite(ENABLE_BUZZER,HIGH);
    delay(1000);        // ...for 1sec
  #endif
  
  
  for (size_t i = 0; i < UniqueIDsize; i++)
	{
		if (UniqueID[i] < 0x10)
			sID += "0";
		sID += UniqueID[i];
	}
	
  
  // sID = EEPROM.read(0);

  // for (int i=0; i<6; i++) {
  //   sID = EEPROM.read(i);
  // }
  
  // if(sID == 0){
  //   Serial.println("Writing Serial");
  //   int k = -1;
  //   int checksum = 0;
  //   uint16_t j = 0;
    
  //   for(; j < (341*1); j++) {
  //     checksum += * ( (byte *) j );//checksum += the byte number u in the ram
  //   }
  //   EEPROM.write(k++,checksum);
    
  //   for (int i=0; i<6; i++) {
  //     sID = EEPROM.read(i);
  //   }
  // }
  // Serial.println(sID,HEX);
   
  // if(checksum == 0)
  // for(uint16_t u = 0; u < 2048; u++)
  // {
  //   checksum += * ( (byte *) u );//checksum += the byte number u in the ram
  // }
  
// 	Serial.println(dID);
  
  #ifdef ENABLE_BME280
    bme.begin(); 
  #endif
    
  #ifdef ENABLE_BH1750
    lightmeter.begin(BH1750LIB_MODE_CONTINUOUSHIGHRES);
  #endif
    
    
  #ifdef ENABLE_LED
    FastLED.addLeds<WS2812B, PIN_LED_RING, GRB>(leds, PIXEL_COUNT_RING); 
    FastLED.addLeds<WS2812B, PIN_LED_STRIP1, GRB>(leds_side, PIXEL_COUNT_SIDE); 
    FastLED.addLeds<WS2812B, PIN_LED_STRIP2, GRB>(leds_side, PIXEL_COUNT_SIDE); 
    FastLED.addLeds<WS2812B, PIN_LED_STRIP3, GRB>(leds_side, PIXEL_COUNT_SIDE); 
    FastLED.addLeds<WS2812B, PIN_LED_STRIP4, GRB>(leds_side, PIXEL_COUNT_SIDE); 
  #endif

  #ifdef ENABLE_WATER_RELAYS
    // pinMode(PIN_WATER_SENSOR1_PIN_A, INPUT);
    pinMode(PIN_WATER_SENSOR1_PIN_D, INPUT);
    // pinMode(PIN_WATER_SENSOR2_PIN_A, INPUT);
    pinMode(PIN_WATER_SENSOR2_PIN_D, INPUT); 
    
    pinMode(PIN_RELAY1,OUTPUT);
    digitalWrite(PIN_RELAY1,LOW);
    
    pinMode(PIN_RELAY2,OUTPUT);
    digitalWrite(PIN_RELAY2,LOW);
    
    pinMode(PIN_RELAY3,OUTPUT);
    digitalWrite(PIN_RELAY3,LOW);
    
    pinMode(PIN_RELAY4,OUTPUT);
    digitalWrite(PIN_RELAY4,LOW);
  #endif
  
}


#ifdef ENABLE_BME280

double dewPoint(double tempf, double humidity) {
    double AAA0= 373.15/(273.15 + tempf);
    double SUM = -7.90298 * (AAA0-1);
    SUM += 5.02808 * log10(AAA0);
    SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/AAA0)))-1) ;
    SUM += 8.1328e-3 * (pow(10,(-3.49149*(AAA0-1)))-1) ;
    SUM += log10(1013.246);
    double VP = pow(10, SUM-3) * humidity;
    double T = log(VP/0.61078);   
    return (241.88 * T) / (17.558-T);
}

#endif

String readString_key;
String str_empty = "";
bool readString_key_complete = false;
String readString_value[3] = {"","",""};
int readString_valueI = 1;
int msgID = 0;
String str_deviderA = ":";
String str_deviderB = ",";

void parseSerialCommand(String key,int val1,int val2,int val3){
  if(key == "") return;
  
  if(key == "id"){
    // EEPROM.write(0,val1);
    resetFunc();
  }else
  
  if(key == "reset"){
    // resetFunc();
    REQUEST_EXTERNAL_RESET;
  }else 
  
  #ifdef ENABLE_WATER_RELAYS
    if(key == "r1"){
      if(val1 == 1){
        digitalWrite(4,HIGH);
      }else{
        digitalWrite(4,LOW);
      } 
    } else if(key == "r2"){
      if(val1 == 1){
        digitalWrite(5,HIGH);
      }else{
        digitalWrite(5,LOW);
      } 
    } else if(key == "r3"){
      if(val1 == 1){
        digitalWrite(6,HIGH);
      }else{
        digitalWrite(6,LOW);
      } 
    } else if(key == "r4"){
      if(val1 == 1){
        digitalWrite(7,HIGH);
      }else{
        digitalWrite(7,LOW);
      } 
    } else 
  #endif
  
  #ifdef ENABLE_LED
    if(key == "rgb0"){
      rgb0[0] = val1;
      rgb0[1] = val2;
      rgb0[2] = val3;
    } else if(key == "rgb1"){
      rgb1[0] = val1;
      rgb1[1] = val2;
      rgb1[2] = val3;
    } else if(key == "rgb_r"){
      rgb_ratio = val1;
    } else if(key == "rgb_a"){
      rgb_alpha = val1;
    } else 
  #endif

  {
    Serial.println("Invalid Command");
    // return;
  }
    
    Serial.print(key);
    Serial.print(str_deviderA);
    Serial.print(val1);
    Serial.print(str_deviderB);
    Serial.print(val2);
    Serial.print(str_deviderB);
    Serial.println(val3); 
}


void resetInput(){
  readString_key=str_empty;
  readString_value[0]=str_empty;
  readString_value[1]=str_empty;
  readString_value[2]=str_empty;
  readString_key_complete = false;
  readString_valueI = 1;
}
void inputCheck(){
  if(readString_key.length() >30)
    resetInput();
  if(readString_value[0].length() >30)
    resetInput();
  if(readString_value[1].length() >30)
    resetInput();
  if(readString_value[3].length() >30)
    resetInput();
}


#ifdef ENABLE_WATER_RELAYS
  
  int water1sensorAValue;
  int water1sensorDValue;
  int water2sensorAValue;
  int water2sensorAValue_AVG[11] = {0,0,0,0,0,0,0,0,0,0};
  int water2sensorAValue_AVG_C = 0;
  int water2sensorAValue_average = -1;
  int water2sensorDValue;
  
  int relay1;
  int relay2;
  int relay3;
  int relay4;

#endif


#ifdef ENABLE_BH1750
  
  int lux;

#endif


#ifdef ENABLE_BME280
  
  float tempc;
  float tempf;
  float humidity;
  float pressure;
  double due_point;

#endif

void loop()
{
  unsigned long currentMillis = millis();
  
  // if (false) {
  if (currentMillis - previousMillisA >= intervalA) {
    previousMillisA = currentMillis;
    
    //do stuff here
    
    #ifdef ENABLE_WATER_RELAYS
      Serial.print("w1:");
      Serial.print(water1sensorAValue);
      Serial.print("-");
      Serial.print(water1sensorDValue);
      Serial.print(str_deviderB);
      
      Serial.print("w2:");
      Serial.print(water2sensorAValue);
      Serial.print("-");
      Serial.print(water2sensorDValue);
      Serial.print(str_deviderB);
        
      Serial.print("w2a:");
      Serial.print(water2sensorAValue_average);
      Serial.print(str_deviderB);
        
      Serial.print("r1:");
      Serial.print(relay1);
      Serial.print(str_deviderB);
      Serial.print("r2:");
      Serial.print(relay2);
      Serial.print(str_deviderB);
      Serial.print("r3:");
      Serial.print(relay3);
      Serial.print(str_deviderB);
      Serial.print("r4:");
      Serial.print(relay4);
      Serial.print(str_deviderB);
      
    
    #endif
    
    #ifdef ENABLE_BH1750
      Serial.print("l:");
      Serial.print(lux);
      Serial.print(str_deviderB);
    #endif
    
    #ifdef ENABLE_BME280
      Serial.print("t-c:");
      Serial.print(String(tempc));
      Serial.print(str_deviderB);
      
      Serial.print("t-f:");
      Serial.print(String(tempf));
      Serial.print(str_deviderB);
      
      Serial.print("p:");
      Serial.print(String(pressure));
      Serial.print(str_deviderB);
      
      Serial.print("h:");
      Serial.print(String(humidity));
      Serial.print(str_deviderB);
      
      due_point = dewPoint(tempf,humidity);
      
      Serial.print("d:");
      Serial.print(String(due_point));
      Serial.print(str_deviderB);
    #endif
    
    
    #ifdef ENABLE_LED
      Serial.print("rgb0:");
      Serial.print(rgb0[0]);Serial.print("-");Serial.print(rgb0[1]);Serial.print("-");Serial.print(rgb0[2]);
      Serial.print(str_deviderB);
      
      
      Serial.print("rgb1:");
      Serial.print(rgb1[0]);Serial.print("-");Serial.print(rgb1[1]);Serial.print("-");Serial.print(rgb1[2]);
      Serial.print(str_deviderB);
  
      Serial.print("rgb_a:");
      Serial.print(rgb_alpha);
      Serial.print(str_deviderB);
      
      Serial.print("rgb_r:");
      Serial.print(rgb_ratio);
      Serial.print(str_deviderB);
    #endif
      
    Serial.print("id");
    Serial.print(str_deviderA);
    Serial.print(sID);
      
    Serial.print("-");
    Serial.println(msgID);
  }
  
  
  msgID++;
  bool runningWater = false;
  
    // digitalWrite(ENABLE_BUZZER,LOW);
    // delay(1000);        // ...for 1 sec
    // digitalWrite(ENABLE_BUZZER,HIGH);
  inputCheck();
  
  
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Serial.println(c);
    if (c == '=' && readString_key.length() >0) {
      // Serial.println(readString_key);
      readString_key_complete = true;
    }else if (c == ';') {
      inputCheck();
    // }else if (c == '\r' || c == '\n') {
    
      // Serial.print(readString_key);
      // Serial.print("=");
      // Serial.println(readString_value1);
      if(readString_value[0].length() >0){
        int n1 = readString_value[0].toInt();
        int n2 = readString_value[1].toInt();
        int n3 = readString_value[2].toInt();
        parseSerialCommand(readString_key, n1, n2, n3);
    
      }
      resetInput();
      // Serial.flush();
      return;
    } 
    else if (c == ','){
      readString_valueI += 1;
    }
    else {     
      if(!readString_key_complete)
        readString_key += c;
      else if(readString_valueI == 1)
        readString_value[0] += c;
      else if(readString_valueI == 2)
        readString_value[1] += c;
      else if(readString_valueI == 3)
        readString_value[2] += c;
    }
    inputCheck();
  }
  
    
  #ifdef ENABLE_WATER_RELAYS
    
    // analogReadResolution(12);
    water1sensorAValue = analogRead(PIN_WATER_SENSOR1_PIN_A); 
    
    // analogReadResolution(10);
    
    water1sensorDValue = (digitalRead(PIN_WATER_SENSOR1_PIN_D) == HIGH ? 0 : 1);
    
    water2sensorAValue = analogRead(PIN_WATER_SENSOR2_PIN_A); 
    water2sensorDValue = (digitalRead(PIN_WATER_SENSOR2_PIN_D) == HIGH ? 0 : 1); 
    
    // water2sensorAValue avg
    
    // if(water2sensorAValue_AVG_C > 4){
    //   water2sensorAValue_AVG[water2sensorAValue_AVG_C++] = water2sensorAValue;
    // }else{
    //   water2sensorAValue_AVG[0] = water2sensorAValue_AVG[1];
    //   water2sensorAValue_AVG[1] = water2sensorAValue_AVG[2];
    //   water2sensorAValue_AVG[2] = water2sensorAValue_AVG[3];
    //   water2sensorAValue_AVG[3] = water2sensorAValue_AVG[4];
    //   water2sensorAValue_AVG[4] = water2sensorAValue;
      
    //   water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4] ) / 5;
    // }
    
    
    
    if(water2sensorAValue > (4095 - 55)){
      digitalWrite(4,HIGH);
    }else if(water2sensorDValue == 1){
      digitalWrite(4,LOW);
      // delay(100);
      runningWater = true;
    }
    
    
    relay1 = digitalRead(4);
    relay2 = digitalRead(5);
    relay3 = digitalRead(6);
    relay4 = digitalRead(7);
    
    if(relay1 || relay2 || relay3 || relay4){
      delay(500);
      runningWater = true;
      // return;
    }
  #endif
  
  
  if (currentMillis - previousMillisC >= intervalC) {
    previousMillisC = currentMillis;
    #ifdef ENABLE_WATER_RELAYS
      
      // water2sensorAValue avg
      // water2sensorAValue_AVG_C++;
      
      // if(water2sensorAValue_AVG_C > 4){
      //   water2sensorAValue_AVG[water2sensorAValue_AVG_C++] = water2sensorAValue;
      // }else{
      
      switch(water2sensorAValue_AVG_C){
        case 0:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[0] = water2sensorAValue;
          water2sensorAValue_average = water2sensorAValue;
          break;
        case 1:
          water2sensorAValue_AVG_C++;
          
          water2sensorAValue_AVG[1] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1]) / 2;
          
          break;
        case 2:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[2] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2]) / 3;
          break;
        case 3:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[3] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3]) / 4;
          break;
        case 4:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[4] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4]) / 5;
          break;
        case 5:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[5] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4] + water2sensorAValue_AVG[5]) / 6;
          break;
        case 6:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[6] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4] + water2sensorAValue_AVG[5] + water2sensorAValue_AVG[6]) / 7;
          break;
        case 7:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[7] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4] + water2sensorAValue_AVG[5] + water2sensorAValue_AVG[6] + water2sensorAValue_AVG[7]) / 8;
          break;
        case 8:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[8] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4] + water2sensorAValue_AVG[5] + water2sensorAValue_AVG[6] + water2sensorAValue_AVG[7] + water2sensorAValue_AVG[8]) / 9;
          break;
        case 9:
          water2sensorAValue_AVG_C++;
          water2sensorAValue_AVG[9] = water2sensorAValue;
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4] + water2sensorAValue_AVG[5] + water2sensorAValue_AVG[6] + water2sensorAValue_AVG[7] + water2sensorAValue_AVG[8] + water2sensorAValue_AVG[9]) / 10;
          break;
        default:
          water2sensorAValue_AVG[0] = water2sensorAValue_AVG[1];
          water2sensorAValue_AVG[1] = water2sensorAValue_AVG[2];
          water2sensorAValue_AVG[2] = water2sensorAValue_AVG[3];
          water2sensorAValue_AVG[3] = water2sensorAValue_AVG[4];
          water2sensorAValue_AVG[4] = water2sensorAValue_AVG[5];
          water2sensorAValue_AVG[5] = water2sensorAValue_AVG[6];
          water2sensorAValue_AVG[6] = water2sensorAValue_AVG[7];
          water2sensorAValue_AVG[7] = water2sensorAValue_AVG[8];
          water2sensorAValue_AVG[7] = water2sensorAValue_AVG[9];
          water2sensorAValue_AVG[9] = water2sensorAValue;
          
          water2sensorAValue_average = (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4] + water2sensorAValue_AVG[5] + water2sensorAValue_AVG[6] + water2sensorAValue_AVG[7] + water2sensorAValue_AVG[8] + water2sensorAValue_AVG[9]) / 10;
            
          break;
      }
      
      //   if(water2sensorAValue_AVG_C > 10)
      //   water2sensorAValue_average = water2sensorAValue;// (water2sensorAValue_AVG[0] + water2sensorAValue_AVG[1] + water2sensorAValue_AVG[2] + water2sensorAValue_AVG[3] + water2sensorAValue_AVG[4]  +  water2sensorAValue_AVG[5] + water2sensorAValue_AVG[6] + water2sensorAValue_AVG[7] + water2sensorAValue_AVG[8] + water2sensorAValue_AVG[9] ) / 10;
      // // }
      
    #endif
  }
  

  // if (false) {
  if (currentMillis - previousMillisB >= intervalB) {
    previousMillisB = currentMillis;
    
  if(!runningWater){
    analogWrite(57,0);
    delay(500);
    
    #ifdef ENABLE_BH1750
    
      lightmeter.begin(BH1750LIB_MODE_CONTINUOUSHIGHRES);
      uint16_t lux_ = lightmeter.lightLevel();
      lux = lux_;
      // if(lux = 54612){
      //   lux = -1;
      // }else
      //   lux = lux_;
    #endif
    
    #ifdef ENABLE_BME280
    
      if(bme.begin()){ 
        tempc = bme.readTemperature();
        tempf = 1.8*tempc+32;
        humidity = bme.readHumidity();
        pressure = bme.readPressure() / 3386;
        
        if(humidity != 0){
          
        }
      }
      // delay(100);
    #endif
    
    
    #ifdef ENABLE_LED
    
      int j = 1;
    	for(int i = 0; i < PIXEL_COUNT_RING; i++) {
    	  if(j == 1){
    	    if(rgb0[0] > -1)
    	      leds[i] = CRGB(rgb0[0],rgb0[1],rgb0[2]);
    	    else leds[i] = Wheel(rgb0[1]);
    	  }else{
    	    if(rgb1[0] > -1)
    	      leds[i] = CRGB(rgb1[0],rgb1[1],rgb1[2]);
    	    else leds[i] = Wheel(rgb1[1]);
    	  }
    	  if(j == rgb_ratio){
    	    j = 1;
    	  }else{
    	    j++;
    	  }
    	  
    	}
      
      j=1;
    	for(int i = 0; i < PIXEL_COUNT_SIDE; i++) {
    	  if(j == 1){
    	    if(rgb0[0] > -1)
    	      leds_side[i] = CRGB(rgb0[0],rgb0[1],rgb0[2]);
    	    else leds_side[i] = Wheel(rgb0[1]);
    	  }else{
    	    if(rgb1[0] > -1)
    	      leds_side[i] = CRGB(rgb1[0],rgb1[1],rgb1[2]);
    	    else leds_side[i] = Wheel(rgb1[1]);
    	  }
    	  if(j == rgb_ratio){
    	    j = 1;
    	  }else{
    	    j++;
    	  }
    	  
    	}
    	
    	
    	
      FastLED.setBrightness(rgb_alpha);
      FastLED.show();
    
  
    #endif
    
  }
  
  
  }
	
// 	Serial.print(checksum);
// for (int i = 0; i < 3; i++){
// 	Serial.print(sID);
// }
  // Serial.println("");
  
  // delay(500);
  
  analogWrite(57,4096);
}
