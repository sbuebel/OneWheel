#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// global variable to hold bat voltages
double voltages[10];

void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  voltages[0] = 4.2;
  voltages[1] = 8.1;
  voltages[2] = 11.9;
  voltages[3] = 15.25;
  voltages[4] = 19.1;
  voltages[5] = 23.0;
  voltages[6] = 26.7;
  voltages[7] = 30.1;
  voltages[8] = 33.5;
  voltages[9] = 37.5;
  
  // setup text stuff
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

}

void loop()
{
  display.clearDisplay();

  // read analog voltages
  int v9 = analogRead(A0);
  int v8 = analogRead(A1);
  int v7 = analogRead(A2);
  int v6 = analogRead(A3);
  int v5 = analogRead(A4);
  int v4 = analogRead(A5);
  int v3 = analogRead(A6);
  int v2 = analogRead(A7);
  int v1 = analogRead(A8);
  int v0 = analogRead(A9);

  // TODO: ideally, use different resistors for each cell number, so we max precision
  
  // for some reason, seems to be bias of ~ 0.315. in final circuit, calculate this
  double bias = 0.315; // note- bias doesn't work for low voltages, overruns 0!
  // throw a comp filter in
  voltages[9] = 0.75*voltages[9] + 0.25*(((5.0*(double)v9/1023.0) - bias) * 10.99);

  // assume 3.3 - 4.2 safe voltage range
  for (int i = 0; i < 10; i ++)
  {
    // draw each 'cell'
    float fill_height = 0;
    if (i != 0)
      fill_height = 24*(((voltages[i]-voltages[i-1]) - 3.3) / (4.2-3.3));
    else
      fill_height = 24*((voltages[i] - 3.3) / (4.2-3.3));

    // makes it prettier
    if (fill_height > 24)
      fill_height = 24;

    display.drawRoundRect(i*9, 0, 8, 24, 2, SSD1306_WHITE);
    display.fillRoundRect(i*9, 24-fill_height, 8, fill_height, 2, SSD1306_WHITE);

    // write the number under the cell - zero index so it's pretty
    display.setCursor(2+i*9, 25);
    display.write(i+48);
  }
  

  // print out total voltage and display it
  display.setCursor(95, 12);
  display.write(voltages[9]/10 + 48);
  display.write((int)voltages[9]%10 + 48);
  display.write('.');
  display.write((int)(voltages[9]*10) % 10 + 48);
  display.write('V');
  
  display.display();
  delay(100);

}
