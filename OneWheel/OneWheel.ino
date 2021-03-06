#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// function stubs
void init_mpu_6050();
void read_mpu_6050();
void update_state();
void print_angles();
void print_raw();
void calibrate_gyro();
void init_display();
void update_display();
bool fsr_board_start();
bool fsr_board_stay_on(int pwm_signal);

// gyro stuff
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
double angle_pitch, angle_roll, angle_yaw;
double angle_roll_acc, angle_pitch_acc;
double angle_pitch_output, angle_roll_output, angle_yaw_output;

// calibration values from our calibration function
long gyro_x_cal = 219;
long gyro_y_cal = 187;
long gyro_z_cal = -114;
long acc_x_cal = -244;
long acc_y_cal = 71;
long acc_z_cal = 372;

// global variable to help us implement 'pushback'
float CENTER_ANGLE = 0.0;

// motor control
double __P = 120;
double __I = 0.25;
double __D = 350;
double pid_total, pid_i;
int PWM_PIN = 11;

// FSR stuff - pin definitions
int FSR_1 = 0;
int FSR_2 = 1;

// save time, just use library, maybe it's slow
//Adafruit_SSD1306 display(128, 32, &Wire, 4);  // reset pin 4 for some reason
bool display_active = false;
bool _verbose = true;

// easy to config as a servo, note: library modified to run at 250Hz
Servo vesc;

// LEDs to indicate states: subject to change
int led_off = 8;
int led_start = 9;
int led_ride = 10;

// states to dictate what mode the wheel is in
#define OFF                     1
#define START                   2
#define RIDE                    3
#define FSR1_THRESHOLD_ON       300       // threshold for FSR being pressed (analog)
#define FSR2_THRESHOLD_ON       300       // threshold for FSR being pressed (analog)
#define CENTER_THROTTLE         1500      // send 1500 signal for no motion
#define START_STABLE_THRESH     2         // safe start only within 2 deg of level
#define CUTOFF_THRESHOLD        20        // num loops before FSR not pressed turns off board
#define PITCH_CUTOFF            22        // angle at which cutoff occurs
#define ROLL_CUTOFF             28        // angle at which cutoff occurs (primary!)
#define PID_REFRESH_RATE        250       // Hz - rate that we run PID loop

// 'pushback' like onewheel when we're near 100% duty cycle
// when we get back to a reasonable value, turnoff pushback
#define PUSHBACK_CUTOFF_F       1800
#define PUSHBACK_CUTOFF_R       1200
#define PUSHBACK_TURNOFF_F      1700
#define PUSHBACK_TURNOFF_R      1300

void setup()
{
  // configure LEDs to be outputs
  pinMode(led_off, OUTPUT);
  pinMode(led_start, OUTPUT);
  pinMode(led_ride, OUTPUT);
  digitalWrite(led_off, LOW);
  digitalWrite(led_start, LOW);
  digitalWrite(led_ride, LOW);
  
  // write center_throttle to begin
  vesc.attach(PWM_PIN);
  vesc.writeMicroseconds(CENTER_THROTTLE);

  // debugging
  Serial.begin(115200);
//  Serial.println("init");

  // init display if we're using it
  if (display_active)
  {
    init_display();
//    Serial.println("Display initialized.");
  }

  // start I2C
  Wire.begin();
  
  // i2c clock speed set to 400kHz
  TWBR = 12;

  // setup for the gyro, calibrate if necessary
  init_mpu_6050();
//  calibrate_gyro();

//  Serial.println("IMU initialized.");
//  Serial.println("LOOP BEGIN");

  // signal we're good to go
  digitalWrite(led_off, HIGH);
  digitalWrite(led_start, LOW);
  digitalWrite(led_ride, LOW);
}

void loop()
{
  // determines if we want to turn anything on
  static int STATE = OFF;
//  static int loop_count = 1;
  static int hysteresis = 0; // use this to track if we need to cutoff
  
  // use this to make sure we update at the correct rate (250 Hz)
  long loop_start = micros();

  // update display at 0.5Hz
//  if (loop_count++ % 500 == 0)
//  {
//    if (display_active)
//      update_display();
//    loop_count = 1;
//  }

  // use this for D
  double last_roll = angle_roll_output;
  
  ///////////////////////////////////////////////////////////////////////////////
  // gyro stuff
  ///////////////////////////////////////////////////////////////////////////////
  // read directly from the MPU to update state
  read_mpu_6050();

  // take values from read_mpu_6050 and update our state
  update_state();

  // now, based on the angles, we need to command the motor
  // control loop purely depending on pitch
  // to generate the PWM signal to drive the motor
  // calculate based on CENTER_ANGLE, so that pushback of 5 degrees works
  pid_i += __I*(angle_roll_output - CENTER_ANGLE); // note: reset i term when we start
  pid_i = constrain(pid_i, -200, 200);
  
  pid_total = -1*(__P*(angle_roll_output - CENTER_ANGLE) + pid_i + __D*(angle_roll_output-last_roll));

  // make sure we limit appropriately
  pid_total = constrain(pid_total, -500, 500);
  
  // pid_total will max at +/- pid_total
  double pwm_signal = CENTER_THROTTLE + pid_total;

  /////////////////////////////////////////////////////////////////////////////////////
  // Now, update motor speed based on state and PID
  /////////////////////////////////////////////////////////////////////////////////////
  switch (STATE)
  {
    case OFF:
      pwm_signal = CENTER_THROTTLE;
      digitalWrite(led_off, HIGH);
      digitalWrite(led_start, LOW);
      digitalWrite(led_ride, LOW);

      vesc.writeMicroseconds(CENTER_THROTTLE);
      pid_i = 0;
      CENTER_ANGLE = 0.0;
      hysteresis = 0;

      // see if we need to turn on
      if (fsr_board_start())
        STATE = START;

      break;

    case START:
      digitalWrite(led_off, LOW);
      digitalWrite(led_start, HIGH);
      digitalWrite(led_ride, LOW);
      
      // this is the tricky part...
      // go with naive approach to start, where I get the board level,
      // then we just jump into normal loop, as expected

      // by default, assume we're not ready yet
      vesc.writeMicroseconds(CENTER_THROTTLE);

      // make sure we can return to OFF if necessary
      if (!fsr_board_start())
        STATE = OFF;
      // this means we have gotten the board level, we can jump to ride now
      else if (abs(angle_roll_output) < START_STABLE_THRESH)
      {
        // reset PID i term and we're going to start next loop
        CENTER_ANGLE = 0.0;
        STATE = RIDE;
        pid_i = 0;
        hysteresis = 0;
      }
      break;

    case RIDE:
      digitalWrite(led_off, LOW);
      digitalWrite(led_start, LOW);
      digitalWrite(led_ride, HIGH);
      
      vesc.writeMicroseconds(pwm_signal);

      // see if we need to shutdown
      if (!fsr_board_stay_on(pwm_signal))
        hysteresis ++;
      else
        hysteresis = 0;

      if (ridin_dirty > CUTOFF_THRESHOLD)
        STATE = OFF;

      // TODO
//      // see if pushback needs to kick in
//      if (pwm_signal > PUSHBACK_CUTOFF_F)
//        CENTER_ANGLE = 1.0*((1800 - pwm_signal) / 200);
//      else if (pwm_signal < PUSHBACK_CUTOFF_R)
//        CENTER_ANGLE = -1.0*((pwm_signal - 1200) / 200);
//      
//      // see if pushback can turn off
//      if (pwm_signal > PUSHBACK_TURNOFF_R && pwm_signal < PUSHBACK_TURNOFF_F)
//        CENTER_ANGLE = 0.0;
        
      break;
  }

  // print an update if we need it
  if (_verbose)
  {
    String s = " ";
    String output = STATE+s+analogRead(FSR_1)+s+analogRead(FSR_2)+s+angle_roll_output+s+
                    angle_pitch_output+s+pwm_signal+s+CENTER_ANGLE+s+(micros()-loop_start);
    Serial.println(output);
  }
  
  /////////////////////////////////////////////////////////////////////////////////////
  // make sure we keep loop time
  /////////////////////////////////////////////////////////////////////////////////////
  bool on_time = false;
  while (micros() - loop_start < int(1000000 / PID_REFRESH_RATE))
  {
    on_time = true;
    digitalWrite(13, LOW);
  }
  if (!on_time)
    digitalWrite(13, HIGH);
  /////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
}

// helper function to reliably check whether the board TURN ON based on FSRs
bool fsr_board_start()
{
  // if angles are too extreme, return false too
  if (abs(angle_pitch_output) > PITCH_CUTOFF)
    return false;
  if (abs(angle_roll_output) > ROLL_CUTOFF)
    return false;
    
  //goal: return true if both are pressed, else rerturn false
  if (analogRead(FSR_1) < FSR1_THRESHOLD_ON && analogRead(FSR_2) < FSR2_THRESHOLD_ON)
    return true;
  else
    return false;
}

// helper function to reliably check whether the board TURN ON based on FSRs
bool fsr_board_stay_on(int pwm_signal)
{
  // if angles are too extreme, return false too
  if (abs(angle_pitch_output) > PITCH_CUTOFF)
    return false;
  if (abs(angle_roll_output) > ROLL_CUTOFF)
    return false;
  // turn off if we've been ridin dirty for 1 second
  if (ridin_dirty > PID_REFRESH_RATE)
    return false;
 
  // only turn off if we're within 100 of center and one or more is not pressed
  uint8_t FSRs = ((analogRead(FSR_1) > FSR1_THRESHOLD_ON) << 1) | (analogRead(FSR_2) > FSR2_THRESHOLD_ON);

  // this means they're both NOT pressed - turn OFF immediately
  if (FSRs == 0b11)
    return false;
    
  // this means one IS pressed and one IS NOT
  // if we're close to center throttle, turn off. otherwise, keep going
  else if (FSRs == 0b10 || FSRs == 0b01)
  {
    // check pwm signal if only one is pressed
    if (abs(pwm_signal - CENTER_THROTTLE) < 100)
      return false;
    else
    {
      // if we stay in this state for 1 second, turn off the board.
      ridin_dirty ++;
      return true;
    }
  }
  // this means BOTH are PRESSED
  else if (FSRs == 0)
    return true;

  // this should never happen... so turn off if it does
  return false;
}

/*
// print the display outline
void init_display()
{
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  // Clear the buffer
  display.clearDisplay();

  // make sure we're not dimmed
  display.dim(false);

  // setup text stuff
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  //////////////////////////////////////////////////////////////////////////////
  // BATTERY INDICATOR
  //////////////////////////////////////////////////////////////////////////////
  display.drawRoundRect(81, 2, 8, 23, 2, SSD1306_WHITE);

  // top cap of 'battery'
  display.drawRoundRect(83, 0, 3, 2, 1, SSD1306_WHITE);

  // write the number under the cell - zero index so it's pretty
  display.setCursor(95, 17);
  display.write('t');
  display.write('o');
  display.write('t');
  display.write('a');
  display.write('l');

  //////////////////////////////////////////////////////////////////////////////
  // TILT INDICATOR
  //////////////////////////////////////////////////////////////////////////////
  
}

// call this periodically to display useful things
void update_display()
{  
  // 10.9 voltage divide multiplier, 0.04 is offset
  double unscaled_voltage = 5.0*(double)analogRead(A1)/1023.0 + 0.04;
  double voltage = 10.9 * unscaled_voltage;

//  Serial.print(unscaled_voltage);
//  Serial.print(" ");
//  Serial.println(voltage);

  // print out total voltage and display it
  // first, wipe old one
  display.fillRect(95, 4, 30, 12, SSD1306_BLACK);
  display.setCursor(95, 4);
  display.write(voltage/10 + 48);
  display.write((int)voltage%10 + 48);
  display.write('.');
  display.write((int)(voltage*10) % 10 + 48);
  display.write('V');
  display.display();

  // see how 'full' each cell is - 10.91 = 24/2.2
  int fill_height = (voltage-33) * 2.6667;
  fill_height = constrain(fill_height, 0, 22);
  display.fillRoundRect(82, 3, 6, 20, 2, SSD1306_BLACK);
  display.fillRoundRect(81, 24-fill_height, 8, fill_height, 2, SSD1306_WHITE);
  
}
*/

// start communication with the 6050
void init_mpu_6050() 
{
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x6B);                                                     // Send the requested starting register
  Wire.write(0x00);                                                     // Set the requested starting register
  Wire.endTransmission();                                               // End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x1C);                                                     // Send the requested starting register
  Wire.write(0x10);                                                     // Set the requested starting register
  Wire.endTransmission();                                               // End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x1B);                                                     // Send the requested starting register
  Wire.write(0x08);                                                     // Set the requested starting register
  Wire.endTransmission();                                               // End the transmission
}

// read values from 6050
void read_mpu_6050() 
{
  Wire.beginTransmission(0x68);                      // Start communicating with the MPU-6050
  Wire.write(0x3B);                                  // Send the requested starting register
  Wire.endTransmission();                            // End the transmission
  Wire.requestFrom(0x68, 14);                        // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                     // Wait until all the bytes are received
  acc_x = (Wire.read() << 8 | Wire.read());          // Add the low and high byte to the acc_x variable
  acc_y = (Wire.read() << 8 | Wire.read());          // Add the low and high byte to the acc_y variable
  acc_z = (Wire.read() << 8 | Wire.read());          // Add the low and high byte to the acc_z variable
  int temperature = Wire.read() << 8 | Wire.read();  // don't need to keep this value
  gyro_x = (Wire.read() << 8 | Wire.read());         // Add the low and high byte to the gyro_x variable
  gyro_y = (Wire.read() << 8 | Wire.read());         // Add the low and high byte to the gyro_y variable
  gyro_z = (Wire.read() << 8 | Wire.read());         // Add the low and high byte to the gyro_z variable
}

// update angle based on readings
void update_state()
{
  // update for calibration errors
  acc_x += acc_x_cal;
  acc_y += acc_y_cal;
  acc_z += acc_z_cal;
  gyro_x += gyro_x_cal;
  gyro_y += gyro_y_cal;
//  gyro_z += gyro_z_cal;
  
  // AFS - 4096 = 1 g!

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz * 65.5)
  angle_roll += gyro_x * 0.0000611;                                     // Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_y * 0.0000611;                                    // Calculate the traveled roll angle and add this to the angle_roll variable
//  angle_yaw += gyro_z * 0.0000611;                                      // Will drift over time, but shouldn't be a big deal because it's all relative

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  // If the IMU has yawed transfer the pitch angle to the roll angle, and vice versa
//  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);
//  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector
  // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_roll_acc = asin((float)acc_y / acc_total_vector) * 57.296;      // Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_x / acc_total_vector) * -57.296;    // Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_roll_acc += 0.00;                                               // Accelerometer calibration value for pitch
  angle_pitch_acc += 0.00;                                              // Accelerometer calibration value for roll

  angle_pitch = angle_pitch * 0.99 + angle_pitch_acc * 0.01;      // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  angle_roll = angle_roll * 0.99 + angle_roll_acc * 0.01;         // Correct the drift of the gyro roll angle with the accelerometer roll angle
 
  // To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;    // Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;       // Take 90% of the output roll value and add 10% of the raw roll value
//  angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;          // Might as well stick a comp filter on the yaw too!
}

// manually called function to calibrate
void calibrate_gyro()
{
  // these are all globals
  gyro_x_cal = 0;
  gyro_y_cal = 0;
  gyro_z_cal = 0;

  acc_x_cal = 0;
  acc_y_cal = 0;
  acc_z_cal = 0;

  int cal_num = 250;
  for (int cal_int = 0; cal_int < cal_num ; cal_int ++)
  { 
    read_mpu_6050();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z-4096; // we expect 4096 reading if level
  }

  // should be negative to offset the averages we counted (we add this value)
  gyro_x_cal /= -cal_num;
  gyro_y_cal /= -cal_num;
  gyro_z_cal /= -cal_num;
  
  acc_x_cal /= -cal_num;
  acc_y_cal /= -cal_num;
  acc_z_cal /= -cal_num;

  // debug
  if (_verbose)
  {
    Serial.println(gyro_x_cal);
    Serial.println(gyro_y_cal);
    Serial.println(gyro_z_cal);
  
    Serial.println(acc_x_cal);
    Serial.println(acc_y_cal);
    Serial.println(acc_z_cal);
    
    // help debug - so we can see it and stop autoscroll
    delay(3000);
  }

}
