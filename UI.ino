/* PC Interface Task */

void PC_interface_Task_Callback() {                             // Check if there is new input from the user through serial port
  if(Serial.available()){
    
    char inst = Serial.read();
    
    if(digitalRead(emergency_Interrupt_Pin) == 0) return;
    if(curr_status == 2)                                          // PC interface for each instruction
    {
      if(digitalRead(emergency_Interrupt_Pin) != 0)
      {
          curr_status = 0;
          PC_Display_Instruction();
          update_Messages(); 
      }
    }else{
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
      }
    }
  }
}

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
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4+i])));     // Bottle Counts
    Serial.println( buffer + String(data[i]));
  }
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[9])));         // Operation Time
  Serial.println( buffer + String(recorded_time));
  
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[35])));        // RTC
  Serial.println(buffer + RTC_store[0] + " " + RTC_store[1]);
  
}


void log_data(){                  // log data in EEPROM
  //int block_size = 5+sizeof(float);
  
  int block_size = 5 + sizeof(float) + sizeof(char) * 22;
  float time_value;
  byte value;
  char char_value;
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
    for(int j=0;j<22;j++)       // Real-time information
    {
      EEPROM.get(address+5+sizeof(float) + j*sizeof(char), char_value);
      EEPROM.put(address+5+sizeof(float) + j*sizeof(char) + block_size, char_value);
    }
  }
  for(int j=0;j<=4;j++)EEPROM.write(j, data[j]);                      // store the new sorting information in
  EEPROM.put(5, recorded_time);

                                                                      // put RTC information into EEPROM
  for(int j=0;j<14;j++)
  {
    EEPROM.put(5+sizeof(float) + j * sizeof(char), RTC_store[0][j]);
  }
  for(int j=0;j<8;j++)
  {
    EEPROM.put(5+sizeof(float) + (14+j) * sizeof(char), RTC_store[1][j]);
  }
  
}

void PC_Display_Permenant_Log(){      // read and display the past trial information
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[26]))); 
  Serial.println( buffer );
  int address = 0;
  byte value;
  float time_value;
  char char_value;
  for(int i=0;i<5;i++){               // iterate through each record and read each byte in the record separately.
    if(i==0)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[27])));    // Current Trial
      Serial.println(buffer);
    }else{
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[28])));    // Previous Trials
      Serial.println(buffer + String(i));
    }
    value = EEPROM.read(address);                                     // Bottle Count Information Output for PC interface
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

                                                                      // RTC information display for each trial
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[35])));
    Serial.println(buffer);
    for(int j=0;j<22;j++)
    {
      EEPROM.get(address, char_value);
      address += sizeof(char);
      Serial.print(char_value);
      if(j == 13) Serial.print(" ");
    }
    Serial.println();
  }
}

void PC_Display_Time(){                                           // Display the RTC time
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[29])));
  Serial.println(buffer);
  generate_RTC_Message();
  Serial.println(RTC_Display[0]);
  Serial.println(RTC_Display[1]);
}

/* LCD Task */

void LCD_Task_Callback() {              // Update the message shown on the LCD      
  lcd.clear();                          // Clear the LCD screen before update
  
  if(curr_status == 0)                   // If the user is in a menu, add ">".
  {
    if(scrn == curs) lcd.print(">");
    else lcd.print(" ");
  }
  lcd.print(message[0]);
  if(curr_status == 0)
  {
    if(scrn > 0) {                       // Add up arrow if there are more options on top
      lcd.setCursor(15,0);
      lcd.print("^");
    }
  }
  
  lcd.setCursor(0, 1);                   // Repeat the procedure thing for the second line of the display
  if(curr_status == 0)
  {
    if(scrn + 1 == curs) lcd.print(">"); // If the user is in a menu, add ">".
    else lcd.print(" ");
  }
  lcd.print(message[1]);
  if(curr_status == 0)                   // Add down arrow if there are more options at the bottom
  {
    if(scrn + 1 < len - 1){
      lcd.setCursor(15,1);
      lcd.print("v");
    }
  }
  prepare_LCD_Task();                    // Prepare for the next request
}

void prepare_LCD_Task() {
  LCD_Task_request.setWaiting();
  LCD_Task.waitFor(&LCD_Task_request);
}

/* RTC Task */

void RTC_Task_Callback(void)                            // retrive and update RTC display message
{
  RTC.readTime();
  if (curr_status == 0 && menu == 2) {
    generate_RTC_Message();
    generate_message_from_list(RTC_Display);
    LCD_Task_request.signalComplete();
  }
}

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

void generate_RTC_Message()                               // generate RTC message by reading RTC code
// Print the current time and date to the LCD display
{
  RTC_Display[1] = p2dig(RTC.h) + ":" + p2dig(RTC.m) + ":" + p2dig(RTC.s);                           // HH:MM:SS  　Time
  RTC_Display[0] = dow2String(RTC.dow) + " " + RTC.yyyy + "-" + p2dig(RTC.mm) + "-" + p2dig(RTC.dd); // YYYY:MM:DD  Date
}

void resetTime(void)                                    // reset RTC to desired time (only used when resetting RTC)
{
  RTC.yyyy = 2017;
  RTC.mm = 4;
  RTC.dd = 10;
  RTC.h = 18;
  RTC.m = 23;
  RTC.s = 00;
  RTC.dow = 2;
  RTC.pm = 0;
  RTC.writeTime();
}

/* Keypad Task */

void keypad_Task_Callback() {                                // check user's key input and responds correspondingly
  char customKey = customKeypad.getKey();
  if(digitalRead(emergency_Interrupt_Pin) == 0) return;
  if (customKey)
  {
    if (curr_status == 0)
    {
      if (customKey == 'A')             // Confirm key is pressed
      {
        if (menu == 0)
        {
          if (curs == 0)
          {
            start_sorting();            // Instruct the machine to start sorting
          } else if (curs == 1)         // Enter result sub-menu
          {
            // Result
            menu = 1;
            pre_scrn = scrn;
            pre_curs = curs;
            scrn = 0;
            curs = 0;
            len = 9;
          } else if (curs == 2)         // Enter Date and Time sub-menu
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
      }else if (customKey == 'D')       // Cancel key is pressed
      {
        if(menu != 0)                   // Return from sub-menu
        {
          menu = 0;
          curs = pre_curs;
          scrn = pre_scrn;  
          len = 3;        
        }
      }else if (customKey == 'B')        // Scroll up 
      {
        if(curs > 0) curs --;
        if(curs < scrn) scrn = curs;
      }else if (customKey == 'C')       // Scroll down
      {
        if(curs < len - 1) curs ++;
        if(curs - 1 > scrn) scrn = curs - 1;
      }
    }else if (curr_status == 1)        // Machine is running 
    {
       if (customKey == 'D')            // Stop the machine if return key is pressed
       {
          emergency_stop();
       }
    }else if (curr_status == 2 && customKey)    // Complete mode
    {
      curr_status = 0;                          //  Return to main menu if any key is pressed
      curs = scrn = menu = 0;
      PC_Display_Instruction();                 // Request PC interface to output instruction message
    }
    update_Messages();                          // Update LCD
    
  }
}

void generate_message_from_list(String *list) {                 // generate message from a list and add cursor to the screen.
  message[0] = list[scrn];
  message[1] = list[scrn + 1];
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
      } else if (menu == 1)                           // Result Menu
      {
        for(int i=0;i<6;i++)                          // Display Bottle Counts on LCD
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

        strcpy_P(buffer, (char*)pgm_read_word(&(string_table[35])));
        temp_result[6] = buffer;
        temp_result[7] = RTC_store[0];                  // Display stored RTC information on LCD
        temp_result[8] = RTC_store[1];
        generate_message_from_list(temp_result);
      } else if (menu == 2)                             // Display RTC information on LCD
      {
        generate_RTC_Message();
        generate_message_from_list(RTC_Display);
        len = 2;
      }
    } else if(curr_status == 1)                         // Running
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[30])));
      message[0] = buffer;
      message[1] = "";
    } else if(curr_status == 2)                         // Complete Mode
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[31])));
      message[0] = buffer;
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[32])));
      message[1] = buffer;
    }
    LCD_Task_request.signalComplete();                // Request the LCD to update
}


