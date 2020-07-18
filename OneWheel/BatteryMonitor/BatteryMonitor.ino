void setup() {}

void loop() {}

// this will get called at 5Hz so we don't ruin angles being calculated
void update_battery_monitor()
{
  // these will be tracked
  static int analog_v[10];
  static int bar_levels[10];
  static double voltages[10];
  
  // read analog voltages, throw in array
  analog_v[9] = analogRead(A9);
  analog_v[8] = analogRead(A8);
  analog_v[7] = analogRead(A7);
  analog_v[6] = analogRead(A6);
  analog_v[5] = analogRead(A5);
  analog_v[4] = analogRead(A4);
  analog_v[3] = analogRead(A3);
  analog_v[2] = analogRead(A2);
  analog_v[1] = analogRead(A1);
  analog_v[0] = analogRead(A0);

  // TODO: each line will require precise calibration, bias needs to be added
  double bias[10] = {   0.31,    0.10,    0.21,    0.23,    0.35,    0.175,   0.24,   0.27,   0.29,   0.345};

  // ratio of each voltage div, hand measured
  double v_divs[10] = { 1,    5.72,    5.84,    5.73,    5.79,       11.07,  11.15,  11.17,  11.27,  11.10};

  // 9-5 all have the same circuit, others will need a different equation
  for (int i = 0; i < 10; i ++)
  {
    // comp filter
    double adc_voltage = 5.0*(double)analog_v[i]/1023.0 - bias[i];

    // first time, do quick update
    if (abs(adc_voltage*v_divs[i] - voltages[i]) < 1)  
      voltages[i] = 0.75*voltages[i] + 0.25*v_divs[i]*adc_voltage;
    else
      voltages[i] = v_divs[i]*adc_voltage;
    if (voltages[i] < 0) // cleanup output
      voltages[i] = 0;

//    Serial.print(adc_voltage);
//    Serial.print(" ");
  }
//  Serial.println("");

    
  // draw each cell: assume 2 - 4.2 voltage range
  for (int i = 0; i < 10; i ++)
  {
    // see how 'full' each cell is - 10.91 = 24/2.2
    int fill_height = 0;
    if (i != 0)
      fill_height = 10.91*((voltages[i]-voltages[i-1]) - 2);
    else
      fill_height = 10.91*(voltages[i] - 2);

    // makes it prettier, don't overfill
    if (fill_height > 24)
      fill_height = 24;

    // if this has changed, update and redraw, otherwise, leave it
    if (fill_height != bar_levels[i])
    {
      bar_levels[i] = fill_height;
      display.fillRoundRect(i*9+1, 1, 6, 22, 2, SSD1306_BLACK);
      display.fillRoundRect(i*9, 24-fill_height, 8, fill_height, 2, SSD1306_WHITE);
      display.display();
    }
  }
  
  // print out total voltage and display it, if it has changed
  // first, wipe old one
  display.fillRect(95, 12, 144, 17, SSD1306_BLACK);
  display.setCursor(95, 12);
  display.write(voltages[9]/10 + 48);
  display.write((int)voltages[9]%10 + 48);
  display.write('.');
  display.write((int)(voltages[9]*10) % 10 + 48);
  display.write('V');
  display.display();

//  print_voltages(voltages);
}

void print_voltages(double *voltages)
{
  for (int i = 0; i < 10; i ++)
  {
    Serial.print(voltages[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void init_display()
{
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  
    // Clear the buffer
    display.clearDisplay();
  
    // setup text stuff
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    
    // print outlines, just update fill in main method
    for (int i = 0; i < 10; i ++)
    {
      display.drawRoundRect(i*9, 0, 8, 24, 2, SSD1306_WHITE);
  
      // write the number under the cell - zero index so it's pretty
      display.setCursor(2+i*9, 25);
      display.write(i+48);
    }
  
    Serial.println("Display initialized.");
}
