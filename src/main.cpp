#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FlashStorage.h>
#include "detect/detect.h"
#include <ISM330DHCT.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

typedef struct {
  boolean valid;
  char password[20];
} Password;

FlashStorage(my_flash_store, Password);

int a_x;
int a_y;
int a_z;
uint8_t menu;
uint8_t count;
bool unlocking;
bool recording;
bool demo;
String sensor;
String recorded;
Password passkey;

ISM330DHCT ism330dhct;

void button_reset(void){
  unlocking = 0;
  recording = 0;
  demo = 0;
  menu = 0;
  sensor = "";
  recorded = "";
  count = 0;

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
 
  display.clearDisplay();
  display.display();
 
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
 
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("A - Unlock");
  display.println("B - Record");
  display.println("C - Demo");
  display.setCursor(0,16);
  display.display();
}

void setup(void) {
  
  Serial.begin(9600);

  detect_setup();
  button_reset();
  
  Wire.begin();
  
  ism330dhct.begin_I2C();

  ism330dhct.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  ism330dhct.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);

  ism330dhct.configInt1(false, false, true); // accelerometer DRDY on INT1
}

void init_sensor(void){
  sensors_event_t accel;
  ism330dhct.get_accel_Event(&accel);
  a_x = accel.acceleration.x;
  a_y = accel.acceleration.y;
  a_z = accel.acceleration.z;
}

void loop() {
  init_sensor();
  switch (menu)
  {
  case 0:
    if((!digitalRead(BUTTON_A)) && (unlocking == false)){
    unlocking = 1;
    menu = 1;
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Unlocking...");
    display.display();
    delay(500);
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("A - Cancel");
    display.setCursor(0,16);
    };
    if((!digitalRead(BUTTON_B)) && (recording == false)){
    recording = 1;
    menu = 1;
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Recording...");
    display.display();
    delay(500);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("A - Cancel");
    display.setCursor(0,16);
    };
    if((!digitalRead(BUTTON_C)) && (demo == false)){
    demo = 1;
    menu = 1;
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Demo Mode");
    display.display();
    delay(500);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("A - Cancel");
    display.setCursor(0,16);
    };
  break;
  case 1:
    sensor = detect(a_x, a_y, a_z);
    if ((sensor) != ""){
      count = count + 1;
      display.print(sensor + ",");
      recorded = recorded + sensor;
      if ((count >=4) && (demo == false)) {
        menu = 2;
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Wait...");
        display.display();
        delay(500);
      };
    };
    if((!digitalRead(BUTTON_A)) && (unlocking == true)){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Cancelling...");
      display.display();
      delay(500);
      button_reset();
    };
    if((!digitalRead(BUTTON_A)) && (recording == true)){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Cancelling...");
      display.display();
      delay(500);
      button_reset();
    };
    if((!digitalRead(BUTTON_A)) && (demo == true)){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Cancelling...");
      display.display();
      delay(500);
      button_reset();
    };
  break;
  case 2:
    if (unlocking == true){
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("A - Unlock");
      display.println("B - Cancel");
      display.setCursor(0,16);
      display.print(recorded);
    };
    if (recording == true){
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("A - Save");
      display.println("B - Cancel");
      display.setCursor(0,16);
      display.print(recorded);
    };
    if((!digitalRead(BUTTON_A)) && (unlocking == true)){
      passkey = my_flash_store.read();
      if ((String(passkey.password) == recorded) && (passkey.valid == true)) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Unlocked");
        display.display();
        delay(500);
        button_reset();
      }
      else if (passkey.valid == true) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Retry");
        display.display();
        delay(500);
        button_reset();
      }
      else {
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Record Password");
        display.display();
        delay(1000);
        button_reset();
      }

    };
    if((!digitalRead(BUTTON_B)) && (unlocking == true)){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Cancelling...");
      display.display();
      delay(500);
      button_reset();
    };
    if((!digitalRead(BUTTON_A)) && (recording == true)){
      recorded.toCharArray(passkey.password, 20);
      passkey.valid = true;
      my_flash_store.write(passkey);
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Saved");
      display.display();
      delay(1000);
      button_reset();
    };
    if((!digitalRead(BUTTON_B)) && (recording == true)){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Cancelling...");
      display.display();
      delay(500);
      button_reset();
    };
  break;
  default :
    button_reset();
  break;
  }
  
  yield();
  display.display();
  delayMicroseconds(10);
}