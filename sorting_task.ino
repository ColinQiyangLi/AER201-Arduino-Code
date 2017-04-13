void main_initialize_Callback(){        // Initializing for sorting 
  global_s_data_1_p = 180;                // Set previous cap reading to 1 (no cap)
  bottom_flag = false;                  // The flag that keeps track of the falling/rising edge of the IR encoder reading
  
  set_motor(TOP_WHEEL, 1);              // Start bottom/top wheels and close all the gates
  set_motor(BOTTOM_WHEEL, 255); 
  set_motor(GATE1, SERVO1_CLOSE);
  set_motor(GATE2, SERVO2_CLOSE);
  set_motor(GATE3, SERVO3_CLOSE);
  
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;    // Reset sorting result and in-take count
  intake_cnt = 0;
  category[0] = category[1] = category[2] = category[3] = category[4] = category[5] = -1;   // Reset the slot information

  pure_sorting = false;                  // Proceed to clear-out stage if this is true 
  top_wheel_turning = true;              // Reset the status of the top wheel so that the top wheel always starts immediately after the sorting starts
  top_wheel_cnt = FIRST_ROTATION_TIME;   // Set the first rotation time
  first_reading = true;                  // This is used to ignore the first set of reading 
  first_reading_after =  true;           // This is used to ignore the first set of reading right after the clear-out stage begins
  second_stream_Task.setCallback(&main_before_mark_detect_Callback);     // Proceed to the next procedure in 20 ms
  second_stream_Task.delay(20);
}

void main_before_in_place_Callback(){   
        
  bool no_bottle = true;
  for(int i=0;i<6;i++) if(category[i] != -1) no_bottle = false;     // Check if there is no bottle in the slots

  double curr_time = millis();
  recorded_time = (curr_time - start_time) / 1000.0;                // Retrieve the current operation time

  if(recorded_time > MAX_LOADING_TIME && !pure_sorting)             // If time exceed maximum loading time, proceed to clear-out stage
  {
    pure_sorting = true;
    set_motor(TOP_WHEEL, 0);
  }
  
  if(recorded_time > MAX_SORTING_TIME ||                            // If time exceed maximum sorting time
    (no_bottle && intake_cnt >= TOTAL_BOTTLES) ||                   // or there is bottle in the slots and the maximum bottles count is reached
    (no_bottle && recorded_time > MAX_LOADING_TIME))                // or there is bottle in the slots and the maximum loading time is reached
  {                                                                 // then the sorting completes
    main_Task.setCallback(&main_finished);
    main_Task.delay(1000);
  }else                                                             // Otherwise, keep looping or start reading the sensors
  {
    if(read_sensor(DIST1) < BOTTLE_IN_PLACE_THRESHOLD || intake_cnt >= TOTAL_BOTTLES - 1 || pure_sorting)   // If the sensor detects a bottle or it is at clear-out stage, 
    {                                                                                                       // proceed to the sensor reading procedure
      main_Task.setCallback(&main_read_opacity_Callback);
      set_lasor(1);                                                 // Turn on the lasor to allow opacity check in the next callback
      main_Task.delay(LASOR_WAIT_TIME + 300);                       // 300ms is estimated to be the time required for the bottle to drop into the slot
    }else
    {
      main_Task.setCallback(&main_in_place_loop_Callback);          // Otherwise, keep looping
      main_Task.delay(1);
    }
  }
}


void main_in_place_loop_Callback(){           // Dummy callback for looping purpose
  main_Task.setCallback(&main_before_in_place_Callback);
  main_Task.delay(1);
}

void main_read_opacity_Callback(){            // Read the opacity
  s_data_o = read_sensor(OPAC);
  set_lasor(0);                               // Turn off the sensor to prevent any influence to other sensors
  main_Task.setCallback(&main_read_other_sensors_Callback);
  main_Task.delay(LASOR_WAIT_TIME);           // Wait until lasor to fully shut down
}

void main_read_other_sensors_Callback(){        // Read the other sensors other than opacity
  s_data_1 = read_sensor(DIST1);          // VL53L0X
  s_data_2 = read_sensor(DIST2);          // APDS9930
  
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
  if(first_reading || intake_cnt >= TOTAL_BOTTLES || (pure_sorting && !first_reading_after))  // Ignore this set of reading in some edge cases
  {
    category[1] = -1;
    first_reading = false;
  }else
  {
    if(pure_sorting) first_reading_after = false;   
    if (s_data_o < 300) // The current one is yop
    {
      if(global_s_data_1_p < 150) cap1 = true;   // Check for one end (previous reading is used)
      else cap1 = false;
      if(s_data_2 > 150) cap2 = true;             // Check for the other end (current reading is used)
      else cap2 = false;
      if(cap1 && cap2)                           // If both ends are blocked
      {
        category[1] = 0; // yop with cap
        intake_cnt ++; 
      }else                     // If one of the end is blocked
      {
        category[1] = 3; // yop without cap
        intake_cnt ++; 
      }
    }else   // The current one is eska
    {      
      if(global_s_data_1_p < 150) cap1 = true;  // Check for one end (previous reading is used)
      else cap1 = false;
      if(s_data_2 > 20) cap2 = true;            // Check for the other end (current reading is used)
      else cap2 = false;
  
      if(cap1 && cap2)                          // If both ends are blocked
      {
        category[1] = 1; // eska with cap
        intake_cnt ++;
      }else                        // If one of the ends is blocked
      {
        category[1] = 2; // eska without cap
        intake_cnt ++;
      }
    }  
  }
  
  for(int i = 5;i > 0;i--)
    category[i] = category[i-1];                        // Shift all the slots for gate control in the next step (-1 stands for undefined)
  
  global_s_data_1_p = s_data_1;                         // Record the current readings (bottom sensor and opacity sensor)
  
  set_motor(BOTTOM_WHEEL, 255);                         // Start the bottom wheel
  main_Task.setCallback(&main_before_mark_detect_Callback);
  main_Task.delay(100);                                 // Start checking for encoder after 100ms (give the bottom wheel enough time to move away from its old position)
}

void main_before_mark_detect_Callback(){  // IR Encoder Loop

  int sensor_data = read_sensor(PROX2);   // Read the IR sensor for mark detecting
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
      pulse_cnt -= 2;    // The pulse count goes down by 2 for each time because the time of one callback loop is around 2ms
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
      main_Task.delay(1);                                            // Pulse duration
    }
  }else
  {
    if(sensor_data <= ENCODER_THRESHOLD)
    {
      set_motor(BOTTOM_WHEEL, 0);
      if(category[2] == 0){ set_motor(GATE1, SERVO1_OPEN); data[0]++; data[4]++; category[2] = -1;}            // Open the corresponding gates when the bottom wheel is doing pulsed regulated motion
      if(category[3] == 1){ set_motor(GATE2, SERVO2_OPEN); data[1]++; data[4]++; category[3] = -1;}
      if(category[4] == 2){ set_motor(GATE3, SERVO3_OPEN); data[2]++; data[4]++; category[4] = -1;}
      if(category[5] == 3){ data[3]++; data[4]++; category[5] = -1;}
      bottom_flag = true;
      
      pulse_cnt = PULSE_DURATION;     // setup for pulse pause motion for the bottom wheel
      pulse_pause = true;
      
      main_Task.setCallback(&main_before_mark_detect_loop_Callback);  
      main_Task.delay(STOP_WHEEL_WAIT_TIME);                                 // Wait until the wheel is fully stopped
    }else
    {
      main_Task.setCallback(&main_before_mark_detect_loop_Callback);         // Keep looping until a falling edge is detected
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
  main_Task.setCallback(&main_after_gate_open_Callback);
  main_Task.delay(BOTTLE_DROP_WAIT_TIME);                                    // execute the next callback after 2s
}

void main_after_gate_open_Callback(){                   // When the gates are opened after 2s, the gates are instructed to be closed.
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

void topwheel_Callback(){                           // Keep track of the count down to make sure the motor turns for a fixed amount of time
                                                    // and stops for a fixed amount of time
  if(curr_status != 1 || pure_sorting) return;
  top_wheel_cnt -= 20;                              // The count down is decreased by 20ms because the callback function is executed approx. every 20ms
  if(top_wheel_turning)
  {
    if(top_wheel_cnt < 0)
    {
      set_motor(TOP_WHEEL, 0);
      top_wheel_turning = false;
      top_wheel_cnt = TOP_WHEEL_WAIT_TIME;          // Resting Time
    }
  }else{
    if(top_wheel_cnt < 0)
    {
      set_motor(TOP_WHEEL, 1);
      top_wheel_turning = true;
      top_wheel_cnt = TOP_WHEEL_DURATION;           // Running Time
    }
  }
}
