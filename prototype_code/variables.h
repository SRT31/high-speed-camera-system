//-------------arduino IO define for Encoder, Switch, sensor, Stepper and UV led output. see also PCB scheam 
#define Pot_3_pin (A7)      //was A6	// buttons
#define Pot_2_pin (A6)      //was A7	// buttons	
#define Pot_1_pin (A1)      //was A3	// buttons
#define Start_Pb_Pin (3)    //was D10	// Power of the Exhibit
#define SENSOR_IN (A0)      //was A1	// Drop detect sensor 
#define PULSE_OUT (9)       //was D9	// Activate UV led
#define STEPPER_DIR_PIN (2) //was D7	// Stepper direction pin 
#define STEPPER_STEP_PIN (5)//was D6	// Stepper pin
#define PI_3 (A3)


//------------ OLED display
#define i2c_Address 0x3c //initialize with the I2C Oled's addr 0x3C. Typically aliexpress, 0x3D Adafrout
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO

//------------ menu
int long_drop_num = 10;


//------------ Others
int last_sensor_value;
int sensor_value ;

const int sensor_threshold = 200 ;           // optical sensor level for detect drope lower = low sensetivity 
const int debounce_time = 50 ;               // ms wait to check pb
int loop_counter = 0 ;                       // count the number of drops in one cycle
int sens_time = 0;
bool active_flag = false;
bool long_active_flag = false;
int drop_num ;
unsigned long last_BIG_pb_press = 0;
unsigned long previous_time;
unsigned long previous_time_for_loop;

int menu_counter = 0;                        // current menu item changed 
uint32_t Start_Drop = 0;                     // absolute time of starting drop
int old_menu_counter = 0;                    // for clearing the cursor
const int parameters_count = 7 ;             // number of parameters in the menu (and machine) original 8 
bool main_submain_menu = true ;              // start at main menue 

byte Old_Encoder_Read = 0;                   // old 2 bits read from first encoder 
byte New_Encoder_Read = 0;                   // new 2 bits read from first encoder 
int encoder_move_counter = 0 ;               // counting encoder move  
int encoder_correct_move = 0 ;               // encoder move after prescalar
const int encoder_prescalar = 2 ;            // prescalar to divide encoder move to overcome mechanical notch 

int long unsigned pulse_timer = micros();
int long unsigned long_timer = millis();
int long unsigned menu_timer = millis();
int long unsigned led_safe_timer = millis();

//int long unsigned pull_back_timer = millis();
//int pull_back_time = 6300000 ;               // one hour in ms
int timer ;
