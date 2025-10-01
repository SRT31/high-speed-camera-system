//........................................................ functions .................................................................

void move_motor (bool dir) {    //    כששמים  show_num[4]=40 המכשיר נתקע ולא מתחיל בכלל להזיז את המנוע. נתקע לפני ה one_step()
  digitalWrite(STEPPER_DIR_PIN, dir);
  if(micros() - pulse_timer > (20*1000)){
      one_step();
      pulse_timer = micros();
  }
} 
void move_motor_back () {    // after one hour without touching the motor suck back the resin.
  for (int i=0 ; i<50 ; i++){
  digitalWrite(STEPPER_DIR_PIN, true); // true = backwards
  delay(20);
  one_step();
  }
}
void one_step(){        // rotating one step of the motor ( 1.8° per step)
  digitalWrite(STEPPER_STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEPPER_STEP_PIN, LOW);
  delayMicroseconds(2);
}

// ..............................................................  menu  ........................................................

//------------------------------------------------------ print main screan without the cursor (">"/"X")
void Start_Display(){
  display.begin(i2c_Address, true); // Address 0x3C default
  display.clearDisplay();
  display.display();

  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F(__FILE__ " " __DATE__ " " __TIME__));
  display.display();
  delay(5000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("START");
  display.display();
  delay(500);
  display.clearDisplay();
  display.display();
}


void Refresh_display() {
  
  pot_1_val = analogRead(Pot_1_pin);
  pot_2_val = analogRead(Pot_2_pin);
  pot_3_val = analogRead(Pot_3_pin);
  pot_1_map = (map(pot_1_val, 1, 1023, 1, 15));  // number of drops in one run (after big PB Pressed)
  pot_2_map = (map(pot_2_val, 1, 1023, 0, 10));  // drop rate [sec] time after light to next dropt
  pot_3_map = (map(pot_3_val, 1, 1023, 3, 40)); // time int [sec] for long cycle
  
  display.clearDisplay();
  display.setTextSize(1);  
  display.setCursor(25, 8);
  display.print("Drop number: ");
  display.println(pot_1_map);
  display.setCursor(25, 24);
  display.print("Drop Rate: ");
  display.println(pot_2_map);
  display.setCursor(25, 40);
  display.print("Long Time: ");
  display.println(pot_3_map);
  display.setCursor(30,54);
  display.print("time: ");
  display.println(timer);
  display.display(); 
  delay(10);
  if ((millis() - menu_timer ) > 1000 ){            // printing the time passed every second
    timer =(millis() - long_timer)/1000 ; 
    menu_timer=millis();
  }
}


// ------------------------------------------------- check if start (Big) PB pressed, if yes start drop and cure cycle 
bool check_start_PB(){
  bool Start_Pb = digitalRead(Start_Pb_Pin);
  if (Start_Pb == false) {                            // false is pressed 
   delay (debounce_time);
   while (digitalRead(Start_Pb_Pin)== false){}        // wait PB release 
   delay (debounce_time);
   }
   return Start_Pb;
}
