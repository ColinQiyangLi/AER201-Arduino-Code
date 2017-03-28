#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STATUS_REQUEST

#define PROX1 0
#define PROX2 1
#define OPAC 2
#define DIST1 3
#define DIST2 4

#define PROX1PIN A0

#define TOP_WHEEL 0
#define BOTTOM_WHEEL 1
#define GATE1 2
#define GATE2 3
#define GATE3 4

#define MAX_SORTING_TIME 150.0

#define GATE_WAIT_TIME 1000

#define STOP_WHEEL_WAIT_TIME 350
#define TOP_WHEEL_SPD 165
#define BOTTOM_WHEEL_SPD 1

#define LASOR_WAIT_TIME 200

#define ENCODER_THRESHOLD 200

#define PULSE_DURATION 7
#define PULSE_INTERVAL 200

#define TOP_WHEEL_DURATION 1500
#define TOP_WHEEL_WAIT_TIME 7000

#define BOTTLE_DROP_WAIT_TIME 700

#define EXPECTED_READING_VARIANCE 30

#define SERVO1_OPEN 0
#define SERVO2_OPEN 90
#define SERVO3_OPEN 0
#define SERVO1_CLOSE 84
#define SERVO2_CLOSE 4
#define SERVO3_CLOSE 79

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


// Global Variables
APDS9930 apds = APDS9930();     // define sensors
TSL2561 tsl(TSL2561_ADDR_HIGH); 
VL53L0X sensor;
Servo servo1, servo2, servo3; 

uint16_t proximity_data = 0;
int sensorValue = 0;        // value read from the pot

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
const byte MOTORS = 5; // two motors
//define the cymbols on the buttons of the keypads
const char hexaKeys[ROWS][COLS] = {     // define key map
  {'B', 'C', '0', '0'},
  {'A', 'D', '0', '0'},
  {'0', '0', '0', '0'},
  {'0', '0', '0', '0'}
};
byte rowPins[ROWS] = {7, 8}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {9, 10}; //connect to the column pinouts of the keypad
byte motorPins[MOTORS] = {5, 6, 11, 12, 13};    // pin assignment for motors
//byte sensor_Interrupt_Pin = 2;    //interrupt pin assignment
byte emergency_Interrupt_Pin = 3;
byte lasor_Pin = 4;

// user interface information
byte menu = 0, scrn = 0, curs = 0;   // index of menu, screen position and cursor position
byte pre_scrn = 0, pre_curs = 0;      // previous screen position and cursor position
byte len = 3;                         // length of the current menu

byte debug = 1;

int sensor_mock[5];

int temp_reading_queue[5];
int queue_size;

/*uint8_t LCD_cur[8] = {
  0x0, 
  0xa, 
  0x1f, 
  0x1f, 
  0xe, 
  0x4, 
  0x0
};
uint8_t LCD_up[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t LCD_down[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};*/

// sensor info
int pre_readings;

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
const char string_29[] PROGMEM = "System Time";
const char string_30[] PROGMEM = "Running...";
const char string_31[] PROGMEM = "Complete!";
const char string_32[] PROGMEM = "Press any key";
const char string_33[] PROGMEM = "Free Memory = ";
const char string_34[] PROGMEM = " ";



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
  string_34};
  
String temp_result[6];          // temp storage for display strings
String RTC_Display[2];          // temp storage for RTC display message

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

int global_s_data_1_p;
int global_s_data_o_p;
int s_data_1, s_data_2, s_data_o;

int top_wheel_cnt = 0;
bool top_wheel_turning = true;

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

StatusRequest update_LCD_request; // initialize each request
Scheduler ts;       // initialize task scheduler

void check_Keys_Callback(bool);

/* Define all the Low-level Tasks */
Task update_LCD(&update_LCD_Callback, &ts);                                       // Task to update LCD triggered by update_LCD_request
Task check_Keys(10, TASK_FOREVER, &check_Keys_Callback, &ts, NULL);               // Task to update key changes every 10ms
Task update_RTC(200, TASK_FOREVER, &update_RTC_Callback, &ts, NULL);              // Task to update RTC data every 200ms
Task update_PC_Interface(100, TASK_FOREVER, &update_PC_Interface_Callback, &ts, NULL);    // Task to update PC Interface every 100ms

Task main_Task(1000, TASK_FOREVER, &main_initialize_Callback, &ts, false);        // Main task (responsible for the entire sorting process)
Task topwheel_Task(200, TASK_FOREVER, &topwheel_Callback, &ts, true);

//Task sensor_test(1000, TASK_FOREVER, &sensor_test_Callback, &ts, true, NULL);    // For debugging purpose to see the readings for each sensor.

void print_memory(){
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[33])));
  Serial.print(buffer);
  Serial.println(freeMemory());
}

int read_sensor(int sensor_id){
  int sensor_read = 0;
  int lum, ir, full;
  
  switch(sensor_id){
    case PROX1:                                       // Read apds with less sensitive setting to detect if bottle is in place
      apds.readProximity(proximity_data);
      sensor_read = proximity_data;
      //Serial.print(sensor_read);
      if(sensor_read > 100)
        sensor_read = 0;
      else
        sensor_read = 1;        
      break;
    case PROX2:                                       // Read IR sensor to detect if there is a change in color (black to white)
      sensor_read = analogRead(PROX1PIN);
      //Serial.println(sensor_read);
      break;
    case OPAC:
      lum = tsl.getFullLuminosity();                  // Read TSL sensor to get if the bottle is transparent (1: Transparent)
      ir = lum >> 16;
      full = lum & 0xFFFF;
      Serial.print("Full:");
      Serial.print(full);
      Serial.print(" IR:");
      Serial.println(ir);
      sensor_read = full;
      break;
    case DIST1:                                       // Read APDS sensor data with the sensitive setting
      apds.readProximity(proximity_data);
      sensor_read = proximity_data;
      break;    
    case DIST2:                                      // Read the range sensor to detect the distance.
      // change setting for VXL (more sensitive)
      sensor_read = sensor.readRangeContinuousMillimeters();
      //Serial.println(sensor_read);
      break;
    //Serial.println("Actual Sernsor Read #" + String(sensor_id) + ":" + String(sensor_read));
  }
  return sensor_read;
}

void set_lasor(int value){
  Serial.println("Set Lasor:" + String(value));
  if(value == 1)
    digitalWrite(lasor_Pin, HIGH);
  else
    digitalWrite(lasor_Pin, LOW);
}

void set_motor(int motor_id, int value){              // Set the motors (including servos and DC motors)
  //Serial.println("Set Motor " + String(motor_id) + ":" + String(value));
  switch(motor_id){
    case TOP_WHEEL:
      analogWrite(motorPins[TOP_WHEEL], TOP_WHEEL_SPD * value);   // DC motor for the top wheel
      break;
    case BOTTOM_WHEEL:
      analogWrite(motorPins[BOTTOM_WHEEL], BOTTOM_WHEEL_SPD * value);  // DC motor for the bottom wheel
      break;
    case GATE1:                 // Three servos corresponding to the servos
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

//// TEST ////
void sensor_test_Callback2(){                               // This is a secondary sensor test for debugging purpose. It will print out all the processed data for all sensors
  debug = 0;
  for(int i=0;i<5;i++){
    read_sensor(i);
  }
  debug = 1;
}

void sensor_test_Callback(){                                // This is a sensor test for debugging purpose. It will print out all the raw data for all sensors
  sensorValue = analogRead(PROX1PIN);
  Serial.print(sensorValue);
  Serial.print("  ");
  if ( !apds.readProximity(proximity_data) ) {
    Serial.println("Error reading proximity value");
  } else {
    Serial.print("Proximity: ");
    Serial.println(proximity_data);
  }

  Serial.println(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }

  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(" IR: "); Serial.print(ir); 
  Serial.print(" Full: "); Serial.print(full); 
  Serial.print(" Visible: "); Serial.print(full - ir);
  
  Serial.print(" Lux: "); Serial.println(tsl.calculateLux(full, ir));
}

/* Main Task */

void main_initialize_Callback(){        // Initializing for sorting 
  Serial.println(F("Initializing..."));
  global_cnt = 6;
  global_s_data_1_p = 0;                // Set previous cap reading to 1 (no cap)
  bottom_flag = false;
  set_motor(TOP_WHEEL, 1);              // Start bottom/top wheels and close all the gates
  set_motor(BOTTOM_WHEEL, 255);
  set_motor(GATE1, SERVO1_CLOSE);
  set_motor(GATE2, SERVO2_CLOSE);
  set_motor(GATE3, SERVO3_CLOSE);
  //pre_readings = read_sensor(PROX2);        // Pre-set the IR encoder reading
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;    // Reset sorting result
  category[0] = category[1] = category[2] = category[3] = category[4] = category[5] = -1;

  top_wheel_turning = true;
  top_wheel_cnt = 0;
  /*topwheel_Task.setCallback(&topwheel_Callback_start); // reset the callback function
  topwheel_Task.enable();
  topwheel_Task.forceNextIteration();   // Start the top wheel task on the next iteration
  */
  //main_Task.setCallback(&main_before_in_place_Callback);
  main_Task.setCallback(&main_before_mark_detect_Callback);
  main_Task.delay(20);
}

/*
void main_initialize_loop_Callback(){   // Dummy callback for looping purpose
  main_Task.setCallback(&main_initialize_check_Callback);
  main_Task.delay(25);
}

void main_initialize_check_Callback(){
  int sensor_data = read_sensor(PROX2);
  if(sensor_data != pre_readings)     // If the reading is not as same as the previous reading, stop the bottom wheel immediately
  {
    set_motor(BOTTOM_WHEEL, 0);       // stop the bottom wheel
    main_Task.setCallback(&main_before_in_place_Callback);
    main_Task.delay(200);
  }else
  {
    main_Task.setCallback(&main_initialize_loop_Callback);
    main_Task.delay(25);
  }
}*/

void main_before_in_place_Callback(){         // Since we have no mechanism to detect if a bottle is in place, this procedure will lead to "main_in_place_loop_Callback".
  Serial.println(F("Detecting bottles.."));
  //bool loop = true;
  /*if(data[4] >= 10){        // Check if the total number of bottles is already 10
    loop = false;
    //global_cnt --;          // Count down for remaining turns for clear out stage
  }else
  {
    
    int readings = read_sensor(PROX1);
    if(readings == 1){  // in place
      loop = false;
      data[4] ++;
    }
    
    loop = false;
  }*/
  bool should_stop = true;
  for(int i=0;i<6;i++) if(category[i] != -1) should_stop = false;
  //if(global_cnt == 0)       // If the clear out stage is finished jump to the finished callback
  
  double curr_time = millis();
  recorded_time = (curr_time - start_time) / 1000.0;
  if(recorded_time > MAX_SORTING_TIME || (should_stop && data[4] >= 10))
  {
    main_Task.setCallback(&main_finished);
    main_Task.delay(1000);
  }else                     // Otherwise, keep looping or start reading the sensors
  {
    main_Task.setCallback(&main_read_opacity_Callback);
    set_lasor(1);                           // Turn on the lasor to allow opacity check in the next callback
    main_Task.delay(LASOR_WAIT_TIME);                     // Wait until lasor to turn on
  }
}


void main_in_place_loop_Callback(){           // Dummy callback for looping purpose
  main_Task.setCallback(&main_before_in_place_Callback);
  main_Task.delay(250);
}

void main_read_opacity_Callback(){            // Read the opacity
  Serial.println("Read Opacity");
  s_data_o = read_sensor(OPAC);
  set_lasor(0);                               // Turn off the sensor to prevent any influence to other sensors
  queue_size = 0;
  main_Task.setCallback(&main_read_other_sensors_Callback);
  //main_Task.setCallback(&main_wait_reading_stablized_Callback);
  main_Task.delay(LASOR_WAIT_TIME);                       // Wait until lasor to fully shut down
}
/*
void main_wait_reading_stablized_Callback(){
  
int temp_reading_queue[5];
int queue_size;

  if(queue_size == READING_QUEUE_SIZE)
  {
    int avg = 0, var = 0;
    int multiplier = 100;
    for(int i=0;i<READING_QUEUE_SIZE;i++){
      avg += (temp_reading_queue[i] * multiplier);
    }
    avg /= READING_QUEUE_SIZE;
    for(int i=0;i<READING_QUEUE_SIZE;i++){
      var += (avg - temp_reading_queue[i] * multiplier) * (avg - temp_reading_queue[i] * multiplier);
    }
    if(var < EXPECTED_READING_VARIANCE * multiplier * multiplier))
    {
      main_Task.setCallback(&main_read_other_sensors_Callback);
      main_Task.delay(1);
    }
    for(int i=0;i<READING_QUEUE_SIZE-1;i++){
      temp_reading_queue[i] = temp_reading_queue[i+1];
    }
    temp_reading_queue[READING_QUEUE_SIZE-1] = read_sensor(DIST1);
  }else
  {
    temp_reading_queue[queue_size] = read_sensor(DIST1);
    queue_size ++;
  }
  
}

void main_wait_reading_stablized_loop_Callback(){
  
}
*/
void main_read_other_sensors_Callback(){        // Read the other sensors other than opacity
  Serial.println(F("Bottle Detected! Start the bottom wheel"));
  
  s_data_1 = read_sensor(DIST1); // Eska: <100 => depth, 300 => close, Yop: <500 => depth, ~1023 => close
  s_data_2 = read_sensor(DIST2); // 1: depth, 0: close
  //s_data_o = read_sensor(OPAC);  // 1: transparent
                                                 
  /*if (debug)                                   // Display the readings
  {
    Serial.print("Current Readings:");
    Serial.print(" OP:");
    Serial.print(s_data_o);
    Serial.print(" CAP1:");
    Serial.print(s_data_1);
    Serial.print(" CAP2:");
    Serial.println(s_data_2);
  }*/
  
  if (debug)                                    // Display the readings adjusted for the bottle in the second slot
  {    
    Serial.print("Processed Readings:");
    Serial.print(" OP:");
    Serial.print(s_data_o);
    Serial.print(" CAP1:");
    Serial.print(global_s_data_1_p);
    Serial.print(" CAP2:");
    Serial.println(s_data_2);
  }
  category[0] = -1; // undefined category for the bottle just entered the sorting wheel

  /*
   * Category Information
   * yop with cap: 0
   * eska with cap: 1
   * eska without cap: 2
   * yop without cap: 3
   * empty: -1
   */
  
  bool cap1, cap2;
  if (s_data_o < 1000) // The current one is yop
  {
    if(global_s_data_1_p > 800) cap1 = true;   // Check for one end (previous reading is used)
    else cap1 = false;
    if(s_data_2 < 150) cap2 = true;             // Check for the other end (current reading is used)
    else cap2 = false;
    if(cap1 && cap2)                           // If both ends are blocked
    {
      category[1] = 0; // yop with cap
      Serial.println("Detect Yop with Cap!");
      data[0] ++; data[4] ++;
    }else if(cap1 || cap2)                           // If one of the end is blocked
    {
      category[1] = 3; // yop without cap
      Serial.println("Detect Yop without Cap!");
      data[3] ++; data[4] ++;
    }else
    {
      category[1] = -1;
    }
  }else   // The current one is eska
  {
    if(global_s_data_1_p > 200) cap1 = true;  // Check for one end (previous reading is used)
    else cap1 = false;
    if(s_data_2 < 150) cap2 = true;            // Check for the other end (current reading is used)
    else cap2 = false;
    if(cap1 && cap2)                          // If both ends are blocked
    {
      category[1] = 1; // eska with cap
      data[1] ++; data[4] ++;
      Serial.println("Detect Eska with Cap!");
    }else if(cap1 || cap2)                          // If one of the ends is blocked
    {
      category[1] = 2; // eska without cap
      data[2] ++; data[4] ++;
      Serial.println("Detect Eska without Cap!");
    }else
    {
      category[1] = -1;
    }
  }

  if(debug)                                       // Display the current sorting information under the debug mode.
  {
    Serial.println("Current Results");
    Serial.println("Yc:" + String(data[0]));            
    Serial.println("Ec :" + String(data[1]));
    Serial.println("Ew:" + String(data[2]));
    Serial.println("Yw:" + String(data[3]));
    Serial.println("Total:" + String(data[4])); 
  } 
  
  Serial.print(F("Next Slots:"));
  for(int i = 5;i > 0;i--)
  {                           
    category[i] = category[i-1];                        // Shift all the slots for gate control in the next step (-1 stands for undefined)
    Serial.print(String(category[i]) + " ");
  }
  
  global_s_data_1_p = s_data_1;                         // Record the current readings (bottom sensor and opacity sensor)
  global_s_data_o_p = s_data_o;

  
  set_motor(BOTTOM_WHEEL, 255);                         // Start the bottom wheel
  pre_readings = read_sensor(PROX2);
  main_Task.setCallback(&main_before_mark_detect_Callback);
  main_Task.delay(100);                                  // Immediately start checking for encoder
}

void main_before_mark_detect_Callback(){  // IR Encoder Loop

  int sensor_data = read_sensor(PROX2);   // Read the IR sensor for mark detecting
  //Serial.println(sensor_data);
  if(bottom_flag)                         // If the IR detects low already, check for raising edge
  {
    if(sensor_data > ENCODER_THRESHOLD)   // Raising edge occurs
    {
      set_motor(BOTTOM_WHEEL, 0);                                   // Stop the motor
      main_Task.setCallback(&main_after_mark_detect_Callback);
      main_Task.delay(STOP_WHEEL_WAIT_TIME);                                         // Wait until the motor is fully stopped
      bottom_flag = false;
    }else
    {
      pulse_cnt -= 2;    // goes down by 2 because one callback loop is around 2ms
      if(pulse_pause)
      {
        if(pulse_cnt <= 0)
        {
          pulse_pause = false;
          pulse_cnt = PULSE_INTERVAL;
          set_motor(BOTTOM_WHEEL, 0);  
        }
      }else
      {
        if(pulse_cnt <= 0)
        {
          pulse_pause = true;
          pulse_cnt = PULSE_DURATION;
          set_motor(BOTTOM_WHEEL, 255); 
        }
      }                               // Two-step simulation for pulse signal for motors
      main_Task.setCallback(&main_before_mark_detect_loop_Callback); // Next step for simulating pulse signal
      main_Task.delay(1);                                           // Pulse duration
    }
  }else
  {
    if(sensor_data <= ENCODER_THRESHOLD)
    {
      set_motor(BOTTOM_WHEEL, 0);
      if(category[2] == 0){ set_motor(GATE1, SERVO1_OPEN); category[2] = -1;}            // Open the corresponding gates when the bottom wheel is doing pulsed regulated motion
      if(category[3] == 1){ set_motor(GATE2, SERVO2_OPEN); category[3] = -1;}
      if(category[4] == 2){ set_motor(GATE3, SERVO3_OPEN); category[4] = -1;}
      bottom_flag = true;
      
      pulse_cnt = PULSE_DURATION;     // setup for pulse pause motion for the bottom wheel
      pulse_pause = true;
      
      main_Task.setCallback(&main_before_mark_detect_loop_Callback);  
      main_Task.delay(STOP_WHEEL_WAIT_TIME);                                 // Wait until the wheel is fully stopped
    }else
    {
      main_Task.setCallback(&main_before_mark_detect_loop_Callback);
      main_Task.delay(1);
    }
  }
}

void main_before_mark_detect_dot_Callback(){                // Second step of the pulse simulation
  set_motor(BOTTOM_WHEEL, 0);
  main_Task.setCallback(&main_before_mark_detect_Callback); // Stop the motor and wait for the next pulse
  main_Task.delay(PULSE_INTERVAL);
}

void main_before_mark_detect_loop_Callback(){               // Dummy callback function for looping purpose
  main_Task.setCallback(&main_before_mark_detect_Callback);
  main_Task.delay(1);
}

void main_after_mark_detect_Callback(){                // When the bottom wheel is stopped, the gates are opened (only the corresponding gates)

  Serial.println(F("Bottom wheel is stopped"));
  Serial.println();
  
  main_Task.setCallback(&main_after_gate_open_Callback);
  main_Task.delay(BOTTLE_DROP_WAIT_TIME);                                    // execute the next callback after 2s
  
}

void main_after_gate_open_Callback(){                   // When the gates are opened after 2s, the gates are instructed to be closed.
  Serial.println(F("Gates start to close"));
  set_motor(GATE1, SERVO1_CLOSE);
  set_motor(GATE2, SERVO2_CLOSE);
  set_motor(GATE3, SERVO3_CLOSE);
  main_Task.setCallback(&main_before_in_place_Callback);  // Loop back to the before bottle in place callback
  main_Task.delay(GATE_WAIT_TIME);
}

void main_finished(){                                   // When all bottles get sorted and cleared out, this callback gets executed
  emergency_stop();
  main_Task.setCallback(&main_initialize_Callback);
}

/*void topwheel_Callback_start(){                         // Start the bottom wheel when it is instructed
  if(curr_status == 1) set_motor(TOP_WHEEL, 1);
  topwheel_Task.setCallback(&topwheel_Callback_stop);
  topwheel_Task.delay(TOP_WHEEL_DURATION);
}

void topwheel_Callback_stop(){
  if(curr_status == 1) set_motor(TOP_WHEEL, 0);
  topwheel_Task.setCallback(&topwheel_Callback_start);
  topwheel_Task.delay(TOP_WHEEL_WAIT_TIME);
}*/

void topwheel_Callback(){
  if(curr_status != 1)return;
  top_wheel_cnt += 200;
  if(top_wheel_turning)
  {
    if(top_wheel_cnt >= TOP_WHEEL_DURATION)
    {
      set_motor(TOP_WHEEL, 0);
      top_wheel_turning = false;
      top_wheel_cnt = 0;
    }
  }else{
    if(top_wheel_cnt >= TOP_WHEEL_WAIT_TIME)
    {
      set_motor(TOP_WHEEL, 1);
      top_wheel_turning = true;
      top_wheel_cnt = 0;
    }
  }
}

void emergency_interrupt_Callback() {                   // Emergency interrupt callback (triggered when emergency button is pressed)
  if(curr_status != 1) return;
  set_motor(TOP_WHEEL, 0);
  set_motor(BOTTOM_WHEEL, 0);
  set_lasor(0);
  main_Task.setCallback(&main_initialize_Callback);
  main_Task.disable();
  emergency_stop();
}

void motor_init(){                                      // Initialize motor settings
  servo1.attach(motorPins[GATE1]);                      // Attach servo motors to desired pins
  servo2.attach(motorPins[GATE2]);
  servo3.attach(motorPins[GATE3]);
  set_motor(GATE1, SERVO1_CLOSE);
  set_motor(GATE2, SERVO2_CLOSE);
  set_motor(GATE3, SERVO2_CLOSE);
  pinMode(motorPins[BOTTOM_WHEEL], OUTPUT);             // Set the pins to output for controlling bottom/top wheels
  pinMode(motorPins[TOP_WHEEL], OUTPUT);
}

void sensor_init(){
  
  pinMode(lasor_Pin, OUTPUT);  // initialize lasor pin
  
  Serial.println();
  Serial.println(F("---------------------------"));
  Serial.println(F("APDS-9930 - ProximitySensor"));
  Serial.println(F("---------------------------"));
  
  // Initialize APDS-9930 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9930 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9930 init!"));
  }
  delay(100);
  
  print_memory();
  // Start running the APDS-9930 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
  
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous(150);
  sensor.setAddress(21);
  
  if (tsl.begin()) {
    Serial.println(F("Found sensor"));
  } else {
    Serial.println(F("No sensor?"));
    while (1);
  }
  
  tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  
}

void enable_tasks(){
  check_Keys.enable();
  update_RTC.enable();
  update_PC_Interface.enable();
  topwheel_Task.enable();
  //sensor_test.enable();
}

void UI_init(){
  lcd.begin();                                          // setup LCD and serial input/output
  Serial.begin(115200);
  lcd.backlight();
  //lcd.createChar(0, LCD_cur);
  //lcd.createChar(1, LCD_up);
  //lcd.createChar(2, LCD_down);
  pinMode(emergency_Interrupt_Pin, INPUT_PULLUP);
}

void setup() {                                          // setup after the machine is powered on
  
  UI_init();
  sensor_init();
  motor_init();
  
  prepare_Update_LCD();     
  update_Messages();                                    // update the message to LCD
  
  PC_Display_Instruction();
  
  attachInterrupt(digitalPinToInterrupt(emergency_Interrupt_Pin), emergency_interrupt_Callback, FALLING);
  
  enable_tasks();                            // prepare the LCD to be updated as soon as any key is pressed
}

void loop() {                                           // main loop
  ts.execute();                                         // use task scheduler to schedule everything
}
