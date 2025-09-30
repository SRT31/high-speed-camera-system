// PHOTOPOLYMER_DROP_PROJECT
// wrote by: Elad Shapira eladap2@gmail.com ; Amir Ben-Shalom amirb@mada.org.il
// last update: 26.11.2023
// the code support an OLED screen and 3 potentiometers for a menu,
// and activate stepper-motor when button (Start_Pb_Pin (3)) is pressed, the motor is connected to an peristaltic pump,
// drop of photopolymer liquid if pushed, then a sensor (SENSOR_IN (A0)) detect it and activate LEDs (PULSE_OUT (9)) to polymerizing the drop.
// on regular mode if the sensor dont detect a drop, a timer is activating the LEDs.
// ***** if the potentiometers is on "1 - 2 - 3" the timer will not work! it is for filling the tubes of the pump ***** 

#include <Adafruit_SH110X.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
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



//**********************************************************************************************************************************

void setup() {
  
  Serial.begin(9600);
  delay (500);// wait to make sure serial begin 
  Serial.println("STARTING");
  
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
  last_pot_1_map = (map(analogRead(Pot_1_pin), 1, 1023, 1, 20)); // number of drops in one run (after big PB Pressed)
  last_pot_2_map = (map(analogRead(Pot_2_pin), 1, 1023, 0, 10)); // drop rate [sec] time after light to next dropt
  last_pot_3_map = (map(analogRead(Pot_3_pin), 1, 1023, 5, 60)); // time int [sec] for long cycle
  pot_3_map = 11 ;
}

void loop() {
  
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
  
  if(active_flag || long_active_flag){
    
    move_motor(true);                                  // moving the motor one step , false = forward
//    if ((pot_1_map == 1)&&(pot_2_map == 2)&&(pot_3_map == 3)){
//      led_safe_timer = millis();                       // reset the led safe timer0.0.
    }
    if (analogRead(SENSOR_IN) <= sensor_threshold){
      delay (48);                                      // * SusTm suspention time = 48 ms
      digitalWrite(LED_BUILTIN, HIGH);                 // switch (on) internal led
      digitalWrite(PULSE_OUT, HIGH);                   // switch (on) UV led
      delay (105);                                     // * ExpTm exposure time = 105 ms
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
      delay (pot_2_map*1000);                          // DrRt, drop rate  = 2000 ms
      loop_counter++ ;

      drop_num = (long_active_flag) ? long_drop_num : pot_1_map;  // if long_active_flag == false --> pot_1_val = normal cycle
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
