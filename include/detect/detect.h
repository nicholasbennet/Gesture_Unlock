#include "timer.h"

bool up;
bool down;
bool left;
bool right;
bool in;
bool out;
bool started;

uint32_t sampleRate = 1000;

String s;

void detect_setup(void){
  up = 0;
  left = 0;
  right = 0;
  in = 0;
  out = 0;
  down = 0;
  started = 0;
  tcDisable();
}

String detect(int a_x, int a_y, int a_z){
  Serial.println(timer_int());
  if (started == false){
    if ((a_z >= 15) && (up == false)){
      up = 1;
      started = 1;
      s = "";
    }
    else if ((a_z <= -10) && (down == false)){
      down = 1;
      started = 1;
      s = "";
    }
    else if ((a_y >= 15) && (left == false)){
      left = 1;
      started = 1;
      s = "";
    }
    else if ((a_y <= -15) && (right == false)){
      right = 1;
      started = 1;
      s = "";
    }
    else if ((a_x >= 15) && (out == false)){
      out = 1;
      started = 1;
      s = "";
    }
    else if ((a_x <= -15) && (in == false)){
      in = 1;
      started = 1;
      s = "";
    }
    else {
      s = "";
    };
  }
  else if (started == true){
    if ((up == true) && (a_z <= -10)){
      s = "UP";
      detect_setup();
      delay(500);
    }
    else if ((down == true) && (a_z >= 15)){
      s = "DOWN";
      detect_setup();
      delay(500);
    }
    else if ((left == true) && (a_y <= -5)){
      s = "LEFT";
      detect_setup();
      delay(500);
    }
    else if ((right == true) && (a_y >= 10)){
      s = "RIGHT";
      detect_setup();
      delay(500);
    }
    else if ((out == true) && (a_x <= -5)){
      s = "OUT";
      detect_setup();
      delay(500);
    }
    else if ((in == true) && (a_x <= 10)){
      s = "IN";
      detect_setup();
      delay(500);
    }
    else {
      s = "";
      if (timer_int() == 0){
        tcConfigure(sampleRate);
        tcStartCounter();
      }
      else if (timer_int() >=3){
        detect_setup();
      };
    };
  }
  else {
    s ="";
  };
  return s;
}