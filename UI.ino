
void PC_Display_Instruction(){      // Display instruction message for PC interface  
  for(int i=0;i<7;i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[10+i])));
    Serial.println( buffer );
  }
}

void PC_Display_Result(){          // Display result message for PC interface
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[25]))); 
  Serial.println( buffer );
  for(int i=0;i<5;i++){
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4+i])));
    Serial.println( buffer + String(data[i]));
  }
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[9]))); 
  Serial.println( buffer + String(recorded_time));
}


void log_data(){                  // log data in EEPROM
  int block_size = 5+sizeof(float);
  float time_value;
  byte value;
  int address;
  for(int i=3;i>=0;i--){        // shift the first 4 records by one slot
    address = block_size * i;
    for(int j=0;j<=4;j++)
    {
      value = EEPROM.read(address+j);
      EEPROM.write(address+j+block_size, value);
    }
    EEPROM.get(address+5, time_value);
    EEPROM.put(address+5+block_size, time_value);
  }
  for(int j=0;j<=4;j++)EEPROM.write(j, data[j]);    // store the new record
  EEPROM.put(5, recorded_time);
}

void PC_Display_Permenant_Log(){      // read and display the past trial information
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[26]))); 
  Serial.println( buffer );
  int address = 0;
  byte value;
  float time_value;
  for(int i=0;i<5;i++){               // iterate through each record and read each byte in the record separately.
    if(i==0)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[27])));
      Serial.println(buffer);
    }else{
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[28])));
      Serial.println(buffer + String(i));
    }
    value = EEPROM.read(address);
    address ++;
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
    Serial.println(buffer + String(value));
    value = EEPROM.read(address);
    address ++;
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[5])));
    Serial.println(buffer + String(value));
    value = EEPROM.read(address);
    address ++;
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[6])));
    Serial.println(buffer + String(value));
    value = EEPROM.read(address);
    address ++;
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[7])));
    Serial.println(buffer + String(value));
    value = EEPROM.read(address);
    address ++;
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[8])));
    Serial.println(buffer + String(value));
    EEPROM.get(address, time_value);
    address += sizeof(float);
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[9])));
    Serial.println(buffer + String(time_value));
  }
}

void PC_Display_Time(){                                           // Display the RTC time
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[29])));
  Serial.println(buffer);
  generate_RTC_Message();
  Serial.println(RTC_Display[0]);
  Serial.println(RTC_Display[1]);
}

void start_sorting(){
  curr_status = 1;
  start_time = millis();
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[30])));
  Serial.println(buffer);
  update_Messages();
  main_Task.enable();
  main_Task.forceNextIteration();
}

void emergency_stop(){
  main_Task.setCallback(&main_initialize_Callback);
  main_Task.disable();
  set_motor(TOP_WHEEL, 0);
  set_motor(BOTTOM_WHEEL, 0); 
  set_lasor(0); 
  curr_status = 2;
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[31])));
  Serial.println(buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[32])));
  Serial.println(buffer);
  update_Messages();
  end_time = millis();
  recorded_time = (end_time - start_time) / 1000.0;
  log_data();
}

void update_PC_Interface_Callback() {                             // Check if there is new input from the user through serial port
  if(Serial.available()){
    
    char inst = Serial.read();
    if(curr_status == 2)                                          // PC interface for each instruction
    {
      if(digitalRead(emergency_Interrupt_Pin) != 0)
      {
          curr_status = 0;
          PC_Display_Instruction();
          update_Messages(); 
      }
    }else{
      Serial.println(inst);
      Serial.println(String(curr_status));
      if(inst == '1' && curr_status == 0)                               // start the machine
      {
        start_sorting();
      }else if(inst == '2')                                             // show results
      {
        PC_Display_Result();
        PC_Display_Instruction();
      }else if(inst == '3')                                             // permenant log
      {
        PC_Display_Permenant_Log();
        PC_Display_Instruction();
      }else if(inst == '4')                                             // system time
      {
        PC_Display_Time();
        PC_Display_Instruction();
      }else if(inst == '5' && curr_status == 1)                        // emergency stop 
      {
        emergency_stop();        
      }else if(inst == 'q' && debug){
        sensor_mock[0] = 1 - sensor_mock[0];
        Serial.println("PROX1:" + String(sensor_mock[0]));
      }else if(inst == 'w' && debug){
        sensor_mock[1] = 1 - sensor_mock[1];
        Serial.println("PROX2:" + String(sensor_mock[1]));
      }else if(inst == 'e' && debug){
        sensor_mock[2] = 1 - sensor_mock[2];
        Serial.println("OPAC:" + String(sensor_mock[2]));
      }else if(inst == 'f' && debug){
        sensor_mock[3] = 1 - sensor_mock[3];
        Serial.println("DIST1:" + String(sensor_mock[3]));
      }else if(inst == 'g' && debug){
        sensor_mock[4] = 1 - sensor_mock[4];
        Serial.println("DIST2:" + String(sensor_mock[4]));
      }
    }
  }
}



/* LCD Task */

// Update the message shown on the LCD

void prepare_Update_LCD() {
  update_LCD_request.setWaiting();
  update_LCD.waitFor(&update_LCD_request);
}

void update_LCD_Callback() {
  lcd.clear();
  
  if(curr_status == 0)
  {
    if(scrn == curs) lcd.print(">");
    else lcd.print(" ");
  }
  lcd.print(message[0]);
  if(curr_status == 0)
  {
    if(scrn > 0) {
      lcd.setCursor(15,0);
      lcd.print("^");
    }
  }
  
  lcd.setCursor(0, 1);
  if(curr_status == 0)
  {
    if(scrn + 1 == curs) lcd.print(">");
    else lcd.print(" ");
  }
  lcd.print(message[1]);
  if(curr_status == 0)
  {
    if(scrn + 1 < len - 1){
      lcd.setCursor(15,1);
      lcd.print("v");
    }
  }
  prepare_Update_LCD();
}

/* RTC Task */

// Update the time

String dow2String(uint8_t code)                           // convert dow data to string
// Day of week to string. DOW 1=Sunday, 0 is undefined
{
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[17+code])));
  return String(buffer);
}

String p2dig(uint8_t v)                                   // convert number to 2-digit string
{
  if (v < 10)return "0" + String(v);
  return String(v);
}

void update_RTC_Callback(void)                            // retrive and update RTC display message
// update the display
{
  RTC.readTime();
  if (curr_status == 0 && menu == 2) {
    generate_RTC_Message();
    generate_message_from_list(RTC_Display);
    update_LCD_request.signalComplete();
  }
}

void generate_RTC_Message()                               // generate RTC message by reading RTC code
// Print the current time and date to the LCD display
{
  RTC_Display[1] = p2dig(RTC.h) + ":" + p2dig(RTC.m) + ":" + p2dig(RTC.s);
  RTC_Display[0] = dow2String(RTC.dow) + " " + RTC.yyyy + "-" + p2dig(RTC.mm) + "-" + p2dig(RTC.dd);
  if (RTC.status(DS3231_12H) == DS3231_ON)
    RTC_Display[1] = RTC_Display[1] + (RTC.pm ? " pm" : " am");
}

void resetTime(void)                                    // reset RTC to desired time (only used when resetting RTC)
{
  RTC.yyyy = 2017;
  RTC.mm = 2;
  RTC.dd = 28;
  RTC.h = 14;
  RTC.m = 38;
  RTC.s = 00;
  RTC.dow = 5;
  RTC.pm = 0;
  RTC.writeTime();
}

/* Keypad Task */

void generate_message_from_list(String *list) {                 // generate message from a list and add cursor to the screen.
  message[0] = list[scrn];
  message[1] = list[scrn + 1];
  /*if(scrn == curs)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[0])));
    message[0] = buffer + message[0];
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[34])));
    message[1] = buffer + message[1]
  }
  if(scrn + 1 == curs)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[34])));
    message[0] = buffer + message[0];
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[0])));
    message[1] = buffer + message[1]
  }*/
}

void update_Messages(){                                       // update display message for LCD
  if(curr_status == 0)
    {
      if (menu == 0) // Main menu
      {
        for(int i=0;i<3;i++)
        {
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table[1+i])));
            temp_result[i] = buffer;
        }
        generate_message_from_list(temp_result);
      } else if (menu == 1)
      {
        for(int i=0;i<6;i++)
        {
          if(i<5)
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4+i])));
            temp_result[i] = buffer + String(data[i]);
          }else
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4+i])));
            temp_result[i] = buffer + String(recorded_time) + "s";
          }
        }
        generate_message_from_list(temp_result);
      } else if (menu == 2)
      {
        generate_RTC_Message();
        generate_message_from_list(RTC_Display);
        len = 2;
      }
    } else if(curr_status == 1)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[30])));
      message[0] = buffer;
      message[1] = "";
    } else if(curr_status == 2)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[31])));
      message[0] = buffer;
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[32])));
      message[1] = buffer;
    }
    update_LCD_request.signalComplete();
}

void check_Keys_Callback() {                                // check user's key input and responds correspondingly
  char customKey = customKeypad.getKey();
  if (customKey)
  {
    //resetTime();
    if (curr_status == 0)
    {
      if (customKey == 'A')
      {
        if (menu == 0)
        {
          if (curs == 0)
          {
            start_sorting();
          } else if (curs == 1)
          {
            // Result
            menu = 1;
            pre_scrn = scrn;
            pre_curs = curs;
            scrn = 0;
            curs = 0;
            len = 6;
          } else if (curs == 2)
          {
            // Date and Time
            menu = 2;
            pre_scrn = scrn;
            pre_curs = curs;
            scrn = 0;
            curs = 0;
            len = 2;
          }
        }
      }else if (customKey == 'D')
      {
        if(menu != 0)
        {
          menu = 0;
          curs = pre_curs;
          scrn = pre_scrn;  
          len = 3;        
        }
      }else if (customKey == 'B')
      {
        if(curs > 0) curs --;
        if(curs < scrn) scrn = curs;
      }else if (customKey == 'C')
      {
        if(curs < len - 1) curs ++;
        if(curs - 1 > scrn) scrn = curs - 1;
      }
    }else if (curr_status == 1)
    {
       if (customKey == 'D')
       {
          // Stop!!
          emergency_stop();
       }
    }else if (curr_status == 2 && customKey)
    {
      if(digitalRead(emergency_Interrupt_Pin) != 0)
      {
          curr_status = 0;
          curs = scrn = menu = 0;
          PC_Display_Instruction(); 
      }
    }
    update_Messages();
    
  }
}


