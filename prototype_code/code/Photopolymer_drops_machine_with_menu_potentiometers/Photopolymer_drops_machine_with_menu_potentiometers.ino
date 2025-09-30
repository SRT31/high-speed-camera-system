	// PHOTOPOLYMER_DROP_PROJECT
// wrote by: Elad Shapira eladap2@gmail.com ; Amir Ben-Shalom amirb@mada.org.il
// last update: 26.11.2023
// the code support an OLED screen and 3 potentiometers for a menu,
// and activate stepper-motor when button (Start_Pb_Pin (3)) is pressed, the motor is connected to an peristaltic pump,
// drop of photopolymer liquid if pushed, then a sensor (SENSOR_IN (A0)) detect it and activate LEDs (PULSE_OUT (9)) to polymerizing the drop.
// on regular mode if the sensor dont detect a drop, a timer is activating the LEDs.
// ***** if the potentiometers is on "1 - 2 - 3" the timer will not work! it is for filling the tubes of the pump ***** 

#include <Adafruit_SH110X.h> // for OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>   // for OLED
#include <Arduino.h>
#include "variables.h"

/*
 0 - DrpRt, drop rate [sec] time after light to next drop
 1 - SusTm, suspention time [ms, the showed number * 10] after detection befour light
 2 - ExpTm, Exposure time [ms, the showed number * 10]
 3 - DrNum, number of drops in one run (after big PB Pressed)
 4 - StpRt, ms betweeb steps = 20             (200step for full rotation, RPM = 60*(1000/(200*Step_Rate)) = 300/Step_Rate
 5 - LngTm, min between long cycle
 6 - LngDN, number of drops in long cycle
 */

int pot_1_val ; //DrNum, number of drops in one run (after big PB Pressed)
int pot_2_val ; //DrpRt, drop rate [sec] time after light to next dropt
int pot_3_val ; //LngTm, [min] between long cycle
int pot_1_map ;
int pot_2_map ;
int pot_3_map ;
int last_pot_1_map ;
int last_pot_2_map ;
int last_pot_3_map ;

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

///////////////// Without Knobs and Without PB ////////////////////
// === Serial-replaceable settings ===

int drnum = 10;      // drops per run (replaces pot_1_map)
int drprt = 2;       // seconds between drops (replaces pot_2_map)
int lngtm = 11;      // minutes between long cycles (replaces pot_3_map)
int lngdn = 10;      // drops in long cycle

int sus_ms = 48;     // suspension time [ms] before LEDs
int exp_ms = 105;    // exposure time [ms]
int stprt_ms = 20;   // stepper step interval [ms]
////////////////////////////////////////////////////////////////


//**********************************************************************************************************************************

void setup() {
  
  Serial.begin(9600);
  delay (500);// wait to make sure serial begin 
  Serial.println("STARTING");
  handleSerial();
  pinMode(PULSE_OUT, OUTPUT);
  digitalWrite(PULSE_OUT, LOW);          // make sure UV led not on
  
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  digitalWrite(STEPPER_STEP_PIN,  LOW);  
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  digitalWrite(STEPPER_DIR_PIN,  LOW);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Start_Pb_Pin, INPUT_PULLUP);
  pinMode(SENSOR_IN, INPUT);

  
  Start_Display();
  //last_pot_1_map = (map(analogRead(Pot_1_pin), 1, 1023, 1, 20)); // number of drops in one run (after big PB Pressed)
  //last_pot_2_map = (map(analogRead(Pot_2_pin), 1, 1023, 0, 10)); // drop rate [sec] time after light to next dropt
  //last_pot_3_map = (map(analogRead(Pot_3_pin), 1, 1023, 5, 60)); // time int [sec] for long cycle

  ////EDITED
  pot_1_map = drnum;
  pot_2_map = drprt;
  pot_3_map = lngtm;

}

void loop() {
  handleSerial();
//  if (!long_active_flag && !active_flag && ( (millis() - pull_back_timer) > (2*pull_back_time))){  // 2*60 minuts has passed
//    move_motor_back() ; // sucking some resin back so no exposed resin stay
//  }
  
  if (!long_active_flag && !active_flag && ( (millis() - long_timer) > pot_3_map*60000)){  // "pot_3_map" minuts has passed 
    long_active_flag = true;                                                                // long active do a cycle of show_num[6] drops every show_num[5] minuts
  }
  
  if (!active_flag && !long_active_flag){
    Refresh_display();                                // update main/submain menu
    if (!check_start_PB()){                           // chacking if start_PB is pressed
      active_flag = true ;
//      led_safe_timer = millis();                        // reset the led safe timer
    }
  }
	

    if (analogRead(SENSOR_IN) <= sensor_threshold){
      Serial.print('Yesss');
      delay (sus_ms);    //same hardcoded value                                  // * SusTm suspention time = 48 ms
      digitalWrite(LED_BUILTIN, HIGH);                 // switch (on) internal led
      digitalWrite(PULSE_OUT, HIGH);                   // switch (on) UV led
      delay (exp_ms);    //same hardcoded value                                 // * ExpTm exposure time = 105 ms
      digitalWrite(PULSE_OUT, LOW);                    // switch (off) UV led
      digitalWrite(LED_BUILTIN, LOW);                  // switch (off) internal led
    
      display.clearDisplay();
      display.setTextSize(1);  
      display.setCursor(25, 25);
      display.println("polymerized");
      display.setCursor(25, 35);
      display.println("drop number: ");
      display.setCursor(98, 35);
      display.println(loop_counter +1);
      display.display(); 
      delay (drprt*1000);       ///ALSO HERE EDITED                   // DrRt, drop rate  = 2000 ms
      loop_counter++ ;

      drop_num = (long_active_flag) ? lngdn : drnum;  // if long_active_flag == false --> serial set values
      led_safe_timer = millis();                        // reset the led safe timer
      
      if (loop_counter == drop_num){                   // DrNum, number of drops in one run
        loop_counter = 0;
        long_timer = millis();
        timer = 0 ;
//        pull_back_timer = millis();
        active_flag = false;
        long_active_flag = false;
        Refresh_display();
      }
    }
  }
  
void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("help")) {
    Serial.println(F("Commands:"));
    Serial.println(F("  go | stop | status | help"));
    Serial.println(F("  set drnum <n>   - drops per run"));
    Serial.println(F("  set drprt <sec> - seconds between drops"));
    Serial.println(F("  set sus <ms>    - suspension ms before LEDs"));
    Serial.println(F("  set exp <ms>    - exposure ms (LEDs ON)"));
    Serial.println(F("  set stprt <ms>  - stepper step interval"));
    Serial.println(F("  set lngtm <min> - minutes between long cycles"));
    Serial.println(F("  set lngdn <n>   - drops in long cycle"));
    return;
  }

  if (cmd.equalsIgnoreCase("status")) {
    Serial.print(F("drnum=")); Serial.print(drnum);
    Serial.print(F(" drprt=")); Serial.print(drprt);
    Serial.print(F(" sus="));   Serial.print(sus_ms);
    Serial.print(F(" exp="));   Serial.print(exp_ms);
    Serial.print(F(" stprt=")); Serial.print(stprt_ms);
    Serial.print(F(" lngtm=")); Serial.print(lngtm);
    Serial.print(F(" lngdn=")); Serial.println(lngdn);
    return;
  }

  if (cmd.equalsIgnoreCase("go")) {
    active_flag = true;
    loop_counter = 0;
    long_active_flag = false;
    pot_1_map = drnum;      // keep older symbols in sync (if used elsewhere)
    pot_2_map = drprt;
    pot_3_map = lngtm;
    move_motor(true);
    Serial.println(F("GO"));
    return;
  }

  if (cmd.equalsIgnoreCase("stop")) {
    active_flag = false;
    long_active_flag = false;
    Serial.println(F("STOP"));
    return;
  }

  // --- parse: set <name> <value> ---
  if (cmd.startsWith("set ")) {
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 == -1) { Serial.println(F("ERR: use 'set <name> <value>'")); return; }

    String name = cmd.substring(sp1 + 1, sp2); name.toLowerCase();
    long val = cmd.substring(sp2 + 1).toInt();

    if (name == "drnum") { drnum = constrain((int)val, 1, 100); }
    else if (name == "drprt") { drprt = constrain((int)val, 0, 3600); }
    else if (name == "sus") { sus_ms = constrain((int)val, 0, 5000); }
    else if (name == "exp") { exp_ms = constrain((int)val, 0, 5000); }
    else if (name == "stprt") { stprt_ms = constrain((int)val, 1, 1000); }
    else if (name == "lngtm") { lngtm = constrain((int)val, 0, 1440); }
    else if (name == "lngdn") { lngdn = constrain((int)val, 1, 1000); }
    else { Serial.println(F("ERR: unknown name")); return; }

    // keep legacy vars aligned if the rest of your code still uses them:
    pot_1_map = drnum;
    pot_2_map = drprt;
    pot_3_map = lngtm;

    Serial.println(F("OK"));
    return;
  }

  Serial.println(F("ERR: unknown command. Type 'help'."));
}

