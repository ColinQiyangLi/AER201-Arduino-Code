#define _TASK_SLEEP_ON_IDLE_RUN  // For task scheduler
#define _TASK_STATUS_REQUEST

#define TOTAL_BOTTLES 10         // Max bottle count

// Define name for each sensor and motor
#define IR_ENCODER 1             // IR sensor
#define OPAC 2                   // TSL2560
#define DIST1 3                  // VL53L0X
#define DIST2 4                  // APDS9930

#define TOP_WHEEL 0              // Loading wheel DC motor
#define BOTTOM_WHEEL 1           // Sorting wheel DC motor
#define GATE1 2                  // Gate 3 servo
#define GATE2 3                  // Gate 4 servo
#define GATE3 4                  // Gate 5 servo

// Threshold calibration parameters
#define BOTTLE_IN_PLACE_THRESHOLD 200
#define ENCODER_THRESHOLD 200

// Delay time calibration parameters

#define MAX_LOADING_TIME 85.0
#define MAX_SORTING_TIME 120.0  

#define GATE_WAIT_TIME 100

#define STOP_WHEEL_WAIT_TIME 350

// PWM multipliers
#define TOP_WHEEL_SPD 200         // Top wheel speed multiplier
#define BOTTOM_WHEEL_SPD 1        // Bottom wheel speed multiplier

#define LASOR_WAIT_TIME 100

// Servo position calibration parameters

#define PULSE_DURATION 7
#define PULSE_INTERVAL 200


#define TOP_WHEEL_DURATION 1500
#define TOP_WHEEL_WAIT_TIME 7000
#define FIRST_ROTATION_TIME 2000

#define BOTTLE_DROP_WAIT_TIME 300

// Servo position calibration parameters

#define SERVO1_OPEN 0
#define SERVO2_OPEN 90
#define SERVO3_OPEN 0
#define SERVO1_CLOSE 84
#define SERVO2_CLOSE 4
#define SERVO3_CLOSE 90

#define ROWS 4
#define COLS 4
#define MOTORS 5

// Import all the libraries

#include <MemoryFree.h>
#include <avr/pgmspace.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TaskScheduler.h>
#include <EEPROM.h>
#include <MD_DS3231.h>
#include <Servo.h>
#include <APDS9930.h>
#include "TSL2561.h"
#include <VL53L0X.h>

// Constant strings stored in program memory to save dynamic memory space

char buffer[30];   // buffer from program memory

const char string_0[] PROGMEM = ">";     // all the display strings are stored in the program memory
const char string_1[] PROGMEM = "Start";
const char string_2[] PROGMEM = "Results"; 
const char string_3[] PROGMEM = "Date and Time";
const char string_4[] PROGMEM = "Yop w/ Cap:";
const char string_5[] PROGMEM = "Eska w/ Cap:";
const char string_6[] PROGMEM = "Eska w/o Cap:"; 
const char string_7[] PROGMEM = "Yop w/o Cap:";
const char string_8[] PROGMEM = "Total:";
const char string_9[] PROGMEM = "Time:";
const char string_10[] PROGMEM = "Standby Mode!"; 
const char string_11[] PROGMEM = "<Instruction>";
const char string_12[] PROGMEM = "1: Start";
const char string_13[] PROGMEM = "2: Results";
const char string_14[] PROGMEM = "3: Perm Log"; 
const char string_15[] PROGMEM = "4: Sys Time";
const char string_16[] PROGMEM = "5: Emergency Stop";
const char string_17[] PROGMEM = "---";
const char string_18[] PROGMEM = "Sun";
const char string_19[] PROGMEM = "Mon";
const char string_20[] PROGMEM = "Tue";
const char string_21[] PROGMEM = "Wed";
const char string_22[] PROGMEM = "Thu";
const char string_23[] PROGMEM = "Fri";
const char string_24[] PROGMEM = "Sat";
const char string_25[] PROGMEM = "<Current Trial>";
const char string_26[] PROGMEM = "<Recorded Trials>";
const char string_27[] PROGMEM = "Current Trial";
const char string_28[] PROGMEM = "Previous Trial #";
const char string_29[] PROGMEM = "<System Time>";
const char string_30[] PROGMEM = "Running...";
const char string_31[] PROGMEM = "Complete!";
const char string_32[] PROGMEM = "Press any key";
const char string_33[] PROGMEM = "Free Memory = ";
const char string_34[] PROGMEM = " ";
const char string_35[] PROGMEM = "Completed at:";
const char string_36[] PROGMEM = "Started at:";

const char* const string_table[] PROGMEM =     // string table stores all the strings in the program memory
{   
  string_0,
  string_1,
  string_2,
  string_3,
  string_4,
  string_5,
  string_6,
  string_7,
  string_8,
  string_9,
  string_10,
  string_11,
  string_12,
  string_13,
  string_14,
  string_15,
  string_16,
  string_17,
  string_18,
  string_19,
  string_20,
  string_21,
  string_22,
  string_23,
  string_24,
  string_25,
  string_26,
  string_27,
  string_28,
  string_29,
  string_30,
  string_31,
  string_32,
  string_33,
  string_34,
  string_35};

// Temporary Variables
uint16_t proximity_data = 0;
int sensorValue = 0;        // value read from the pot

//Define the symbols on the buttons of the keypads (key map)
const char hexaKeys[ROWS][COLS] = {
  {'B', 'C', '0', '0'},
  {'A', 'D', '0', '0'},
  {'0', '0', '0', '0'},
  {'0', '0', '0', '0'}
};

// Pin Assignments
const byte rowPins[ROWS] = {7, 8}; //connect to the row pinouts of the keypad
const byte colPins[COLS] = {9, 10}; //connect to the column pinouts of the keypad
const byte motorPins[MOTORS] = {5, 6, 11, 12, 13};    // pin assignment for motors
const byte emergency_Interrupt_Pin = 3;   // emergency interrupt pin
const byte lasor_Pin = 4;        // laser pin
const byte IR_encoder_PIN = A0;

// User Interface Information
byte menu = 0, scrn = 0, curs = 0;   // index of menu, screen position and cursor position
byte pre_scrn = 0, pre_curs = 0;      // previous screen position and cursor position
byte len = 3;                         // length of the current menu
  
String temp_result[12];          // temp storage for display strings
String RTC_Display[2];          // temp storage for RTC display message
String RTC_store[4];

byte curr_status = 0; // 0: menu, 1: running, 2: finished
float recorded_time = 0; // Operation time
unsigned long start_time = 0, end_time = 0;  // recorded starting time in milis

int category[6] = {};      // bottle type for each slot
byte data[5] = {};       // Type 1/ Type 2/ Type 3/ Type 4/ Total
String message[2] = {}; // message[0]: first line of LCD, meesage[1]: second line of LCD

byte motors[MOTORS];   //motor[0]: top motor, motor[1]: bottom motor

bool bottom_flag = false;

int global_cnt;
int pulse_cnt;
bool pulse_pause;

// Temporary Sensor Reading
int global_s_data_1_p;
int s_data_1, s_data_2, s_data_o;

// Temporary Logic Counters
int top_wheel_cnt = 0;
bool top_wheel_turning = true;

int intake_cnt = 0;

// Temporary Flags for the second operating stream
bool first_turning = false;
bool pure_sorting = false;
bool first_reading = true;
bool first_reading_after = true;

// Temporary storage for operation starting time and operation ending time
unsigned long st_time, ed_time;

// Sensor Instance Definitions
APDS9930 apds = APDS9930();    
TSL2561 tsl(TSL2561_ADDR_HIGH); 
VL53L0X sensor;

// Motor Instance Definitions
Servo servo1, servo2, servo3; 

// UI Instance Definitions
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize task scheduler and task instances

StatusRequest LCD_Task_request; // initialize each request
Scheduler ts;       // initialize task scheduler

/* Define all the Low-level Tasks */
Task LCD_Task(&LCD_Task_Callback, &ts);                                               // Task to update LCD and it can be triggered by LCD_Task_request
Task keypad_Task(10, TASK_FOREVER, &keypad_Task_Callback, &ts, NULL);                 // Task to update key changes every 10ms
Task RTC_Task(200, TASK_FOREVER, &RTC_Task_Callback, &ts, NULL);                      // Task to update RTC data every 200ms
Task PC_interface_Task(100, TASK_FOREVER, &PC_interface_Task_Callback, &ts, NULL);    // Task to update PC Interface every 100ms
Task first_stream_Task(20, TASK_FOREVER, &topwheel_Callback, &ts, true);              // First operating stream task 
Task second_stream_Task(1000, TASK_FOREVER, &main_initialize_Callback, &ts, false);   // Second operating stream task 



void start_sorting(){                                   // Instruct the machine to start sorting
  curr_status = 1;
  servo1.attach(motorPins[GATE1]);                      // Attach servo motors to desired pins
  servo2.attach(motorPins[GATE2]);
  servo3.attach(motorPins[GATE3]);
  
  start_time = millis();                                // Record starting time
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[30])));
  Serial.println(buffer);                              // Display starting message
  update_Messages();
  second_stream_Task.enable();                          // Activate second operating stream
  second_stream_Task.forceNextIteration();
}

void emergency_stop(){
  second_stream_Task.setCallback(&main_initialize_Callback);    // Reset the second operating stream and deactivate it
  second_stream_Task.disable();
  servo1.detach();                      // Detach all the servo motors 
  servo2.detach();
  servo3.detach();
  pinMode(motorPins[GATE1], INPUT);     // Set the pin mode to input so that the servo will not be controlled by any signals
  pinMode(motorPins[GATE2], INPUT);
  pinMode(motorPins[GATE3], INPUT);
  set_motor(TOP_WHEEL, 0);              // Stop all the motors
  set_motor(BOTTOM_WHEEL, 0); 
  set_lasor(0);                         // Turn off the laser
  curr_status = 2;
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[31])));    // PC interface message
  Serial.println(buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[32])));
  Serial.println(buffer);
  update_Messages();                                              // Update LCD Message
  end_time = millis();
  recorded_time = (end_time - start_time) / 1000.0;               // Record operation time

  // record RTC information 
  generate_RTC_Message();                                         // Generate RTC message and store them temporarily
  RTC_store[0] = RTC_Display[0];
  RTC_store[1] = RTC_Display[1];
  
  log_data();                                                     // Permanent log
}


int read_sensor(int sensor_id){
  int sensor_read = 0;                                // Temporary variables for storing and calculating the readings
  int lum, ir, full;
  
  switch(sensor_id){
    case IR_ENCODER:                                  // Read IR sensor
      sensor_read = analogRead(IR_encoder_PIN);
      break;
    case OPAC:
      lum = tsl.getFullLuminosity();                  // Read TSL sensor to get if the bottle is transparent (1: Transparent)
      ir = lum >> 16;
      full = lum & 0xFFFF;
      sensor_read = full;
      break;
    case DIST2:                                       // Read APDS sensor data with the sensitive setting
      apds.readProximity(proximity_data);
      sensor_read = proximity_data;
      break;    
    case DIST1:                                       // Read the range sensor to detect the distance.
      sensor_read = sensor.readRangeContinuousMillimeters();
      break;
  }
  return sensor_read;                                 // Return the reading as an integer
}

void set_lasor(int value){                            // Turn on/off the laser (1: on, 0: off)
  if(value == 1) digitalWrite(lasor_Pin, HIGH);
  else digitalWrite(lasor_Pin, LOW);
}

void set_motor(int motor_id, int value){              // Set the motors (including servos and DC motors)
  switch(motor_id){
    case TOP_WHEEL:
      analogWrite(motorPins[TOP_WHEEL], TOP_WHEEL_SPD * value);   // DC motor for the top wheel
      break;
    case BOTTOM_WHEEL:
      analogWrite(motorPins[BOTTOM_WHEEL], BOTTOM_WHEEL_SPD * value);  // DC motor for the bottom wheel
      break;
    case GATE1:                                       // Three servos corresponding to the servos
      servo1.write(value);
      break;
    case GATE2:
      servo2.write(value);
      break;
    case GATE3:
      servo3.write(value);
      break;
  }
}

void emergency_interrupt_Callback() {                   // Emergency interrupt callback (triggered when emergency button is pressed)
  set_motor(TOP_WHEEL, 0);                              // Stop all the motors first
  set_motor(BOTTOM_WHEEL, 0);
  set_lasor(0);
  if(curr_status != 1) return;
  second_stream_Task.setCallback(&main_initialize_Callback);  // Deactivate the second stream task immediately
  second_stream_Task.disable();
  emergency_stop();                                           // Proceed to the normal stopping procedure
}

void motor_init(){                                      // Initialize motor settings
  pinMode(motorPins[GATE1], INPUT);
  pinMode(motorPins[GATE2], INPUT);
  pinMode(motorPins[GATE3], INPUT);
  pinMode(motorPins[BOTTOM_WHEEL], OUTPUT);             // Set the pins to output for controlling bottom/top wheels
  pinMode(motorPins[TOP_WHEEL], OUTPUT);
}

void sensor_init(){
  
  pinMode(lasor_Pin, OUTPUT);      // Initialize lasor pin
  apds.init();                     // Initialize APDS-9930 (configure I2C and initial values)
  delay(100);  
  apds.enableProximitySensor(false);
  
  Wire.begin();

  sensor.init();                   // Initialize VL53L0X
  sensor.setTimeout(500);
  sensor.startContinuous(150);
  sensor.setAddress(21);
  
  tsl.begin();                     // Initialize TSL2560
  
  tsl.setGain(TSL2561_GAIN_0X);                 // Set no gain (for bright situtations)
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // Shortest integration time (bright light
}

void enable_tasks(){                // Enable all the tasks that are initially disabled for safety
  keypad_Task.enable();
  RTC_Task.enable();
  PC_interface_Task.enable();
  first_stream_Task.enable();
}

void UI_init(){
  lcd.begin();                                          // setup LCD and serial input/output
  Serial.begin(115200);
  lcd.backlight();
  pinMode(emergency_Interrupt_Pin, INPUT_PULLUP);
}

void setup() {                                          // setup after the machine is powered on
  
  UI_init();                                            // initialize all components
  sensor_init();
  motor_init();

  prepare_LCD_Task();                                   // setup request
  update_Messages();                                    // update the message to LCD
  
  PC_Display_Instruction();                             // output initial PC output
  
  attachInterrupt(digitalPinToInterrupt(emergency_Interrupt_Pin), emergency_interrupt_Callback, FALLING);
                                                        // initialize emergency button
  enable_tasks();                                       // enable all the tasks as soon as the initialization is complete
}

void loop() {                                           // main loop
  ts.execute();                                         // use task scheduler to schedule all six tasks
}
