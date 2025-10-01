// PHOTOPOLYMER_DROP_PROJECT
// wrote by: Elad Shapira eladap2@gmail.com ; Amir Ben-Shalom amirb@mada.org.il
// last update: 26.11.2023

#include <Adafruit_SH110X.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>
#include "variables.h"


int pot_1_val, pot_2_val, pot_3_val;
int pot_1_map, pot_2_map, pot_3_map;
int last_pot_1_map, last_pot_2_map, last_pot_3_map;

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === Serial-replaceable settings (no knobs / no PB) ===
int drnum   = 10;   // drops per run
int drprt   = 2;    // seconds between drops
int lngtm   = 11;   // minutes between long cycles
int lngdn   = 10;   // drops in long cycle
int sus_ms  = 48;   // suspension time [ms]
int exp_ms  = 120;  // exposure time [ms]
int stprt_ms= 20;   // stepper step interval [ms]

// keep Serial responsive during waits
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    handleSerial();
    delay(1);
  }
}

void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("STARTING");

  pinMode(PULSE_OUT, OUTPUT);
  digitalWrite(PULSE_OUT, LOW);

  pinMode(STEPPER_STEP_PIN, OUTPUT);
  digitalWrite(STEPPER_STEP_PIN, LOW);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  digitalWrite(STEPPER_DIR_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Start_Pb_Pin, INPUT_PULLUP); // harmless to keep even without PB
  pinMode(SENSOR_IN, INPUT);

  pinMode(PI_3, OUTPUT);	// A3 is configred as output to be connected with a voltage divider and eventually to PI
  digitalWrite(PI_3, LOW);	// Make sure its not high

  Start_Display();

  // initialize legacy maps from serial-set values
  pot_1_map = drnum;
  pot_2_map = drprt;
  pot_3_map = lngtm;
}

void loop() {
  handleSerial();

  // long-cycle timer
  if (!long_active_flag && !active_flag &&
      ((millis() - long_timer) > (unsigned long)pot_3_map * 60000UL)) {
    long_active_flag = true;
  }

  // idle UI (no auto-start from PB)
  if (!active_flag && !long_active_flag) {
    Refresh_display();
    // If you truly have no PB, keep this disabled:
    // if (!check_start_PB()) { active_flag = true; }
  }
  
  //Serial.println("DETECTED\n");
  //Serial.println(analogRead(SENSOR_IN));
  if (active_flag){
    move_motor(true);
  }
  
  

  if (active_flag && analogRead(SENSOR_IN) <= sensor_threshold) {
    digitalWrite(PI_3, HIGH);
    Serial.println(digitalRead(PI_3));
    

    //Serial.println(analogRead(SENSOR_IN));
    smartDelay(sus_ms);

    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(PULSE_OUT, HIGH);
    smartDelay(exp_ms);
    digitalWrite(PULSE_OUT, LOW);
    digitalWrite(LED_BUILTIN, LOW);

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(25, 25);
    display.println("polymerized");
    display.setCursor(25, 35);
    display.println("drop number: ");
    display.setCursor(98, 35);
    display.println(loop_counter + 1);
    display.display();

    smartDelay((unsigned long)drprt * 1000UL);
    loop_counter++;

    drop_num = (long_active_flag) ? lngdn : drnum;
    led_safe_timer = millis();

    if (loop_counter == drop_num) {
      loop_counter = 0;
      long_timer = millis();
      timer = 0;
      active_flag = false;
      long_active_flag = false;
      Refresh_display();
    }
  }
  digitalWrite(PI_3, LOW);
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

      // NEW: state info
      Serial.print(F(" | active=")); Serial.print(active_flag ? 1 : 0);
      Serial.print(F(" long="));     Serial.print(long_active_flag ? 1 : 0);
      Serial.print(F(" loop="));     Serial.print(loop_counter);
      Serial.print(F(" drop_num=")); Serial.print((long_active_flag) ? lngdn : drnum);
      Serial.print(F(" sensor="));   Serial.println(analogRead(SENSOR_IN));
    return;
  }

  if (cmd.equalsIgnoreCase("go")) {
    active_flag = true;
    loop_counter = 0;
    long_active_flag = false;

    // keep legacy vars aligned if used elsewhere
    pot_1_map = drnum;
    pot_2_map = drprt;
    pot_3_map = lngtm;

    //move_motor(true); // kick off the first drop like PB press
    Serial.println(F("GO"));
    return;
  }

  if (cmd.equalsIgnoreCase("stop")) {
    active_flag = false;
    long_active_flag = false;
    digitalWrite(PULSE_OUT, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(PI_3, LOW);   // make sure the Pi line is low too
    Serial.println(F("STOP"));
    return;
  }

  // set <name> <value>
  if (cmd.startsWith("set ")) {
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 == -1) { Serial.println(F("ERR: use 'set <name> <value>'")); return; }

    String name = cmd.substring(sp1 + 1, sp2); name.toLowerCase();
    long val = cmd.substring(sp2 + 1).toInt();

    if (name == "drnum")      drnum    = constrain((int)val, 1, 100);
    else if (name == "drprt") drprt    = constrain((int)val, 0, 3600);
    else if (name == "sus")   sus_ms   = constrain((int)val, 0, 5000);
    else if (name == "exp")   exp_ms   = constrain((int)val, 0, 5000);
    else if (name == "stprt") stprt_ms = constrain((int)val, 1, 1000);
    else if (name == "lngtm") lngtm    = constrain((int)val, 0, 1440);
    else if (name == "lngdn") lngdn    = constrain((int)val, 1, 1000);
    else { Serial.println(F("ERR: unknown name")); return; }

    // keep legacy vars aligned
    pot_1_map = drnum;
    pot_2_map = drprt;
    pot_3_map = lngtm;

    Serial.println(F("OK"));
    return;
  }

  Serial.println(F("ERR: unknown command. Type 'help'."));
}