#include <Wire.h>
#include <Servo.h>

#include "VescUart.h"

/*
TODO:
 - Add bluetooth link to phone for status on the go

 */
 
VescUart UART;

// packet that we read from the read_mpu function
struct IMU_packet
{
  int gx;
  int gy;
  int gz;
  long ax;
  long ay;
  long az;
};

// function stubs
void init_mpu_6050();
struct IMU_packet read_mpu_6050();
void update_state(struct IMU_packet p);
void print_angles();
void print_raw();
void calibrate_gyro();
bool fsr_board_start();
bool fsr_board_stay_on();

// output IMU global values
double pitch, roll, yaw;

bool battery_dead = false;

// calibration values from our calibration function
long gx_cal = 219;
long gy_cal = 187;
long gz_cal = -114;
long ax_cal = -244;
long ay_cal = 71;
long az_cal = 372;

// global variable to help us implement 'pushback'
double CENTER_ANGLE = 0.0;

// motor control
// double __P = 37.5;
// double __I = 0.1;
// double __D = 260;

double __P = 35;
double __I = 3;
double __D = 400;
double pid_total, pid_i;
int PWM_PIN = 11;

// FSR stuff - pin definitions (analog lines)
int FSR_1 = 2;
int FSR_2 = 1;

bool _verbose = 1;

// easy to config as a servo
Servo vesc;

// LEDs to indicate states: subject to change
int led_off = 8;
int led_start = 9;
int led_ride = 10;

// these work ok
int FSR1_THRESHOLD_ON = 350;
int FSR2_THRESHOLD_ON = 350;

// states to dictate what mode the wheel is in
#define OFF                     1
#define START                   2
#define RIDE                    3
#define ERROR_                  4
#define DEAD                    0         // battery death
#define NUM_CELLS               12        // 12s pack
#define LOW_LOW_V_CUTOFF        2.8
#define LOW_V_CUTOFF            3.0
#define CENTER_THROTTLE         1500      // send 1500 signal for no motion
#define START_STABLE_THRESH     2         // safe start only within 2 deg of level
#define CUTOFF_THRESHOLD        20        // num loops before FSR not pressed turns off board
#define PITCH_CUTOFF            22        // angle at which cutoff occurs
#define ROLL_CUTOFF             28        // angle at which cutoff occurs (primary!)
#define PID_REFRESH_RATE        250       // Hz - rate that we run PID loop
#define eRPM_CUTOFF_LIMIT       300       // only turn off if motor is moving less than this


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

  // UART for VESC
  Serial1.begin(115200);
  while (!Serial1) {;}
  UART.setSerialPort(&Serial1);

  // start I2C
  Wire.begin();
  
  // i2c clock speed set to 400kHz
  TWBR = 12;

  // setup for the gyro, calibrate if necessary
  init_mpu_6050();
  //calibrate_gyro();
    
  // signal we're good to go
  digitalWrite(led_off, HIGH);
  digitalWrite(led_start, LOW);
  digitalWrite(led_ride, LOW);
}

void loop()
{
  // determines if we want to turn anything on
  static int STATE = OFF;
  static int hysteresis = 0;  // use this to track if we need to cutoff
  static int lc = 0;  // loop counter
  
  // use this to make sure we update at the correct rate (250 Hz)
  long loop_start = micros();

  // use this for D
  double last_roll = roll;
  
  ///////////////////////////////////////////////////////////////////////////////
  // gyro stuff
  ///////////////////////////////////////////////////////////////////////////////
  // take values from read_mpu_6050 and update our state
  update_state();

  // now, based on the angles, we need to command the motor
  // control loop purely depending on pitch
  // to generate the PWM signal to drive the motor
  // calculate based on CENTER_ANGLE, so that pushback of 5 degrees works
  pid_i += __I*(roll - CENTER_ANGLE); // note: reset i term when we start
  pid_i = constrain(pid_i, -200, 200);
  
  pid_total = -1*(__P*(roll - CENTER_ANGLE) + pid_i + __D*(roll - last_roll));

  // make sure we limit appropriately
  pid_total = constrain(pid_total, -500, 500);
  
  // pid_total will max at +/- pid_total
  double pwm_signal = CENTER_THROTTLE + pid_total;

  // summarize state in a nice string - maybe send to BLE module one day
  String s = " ";
  String t = "\t";
  String output = STATE+t+FSR1_THRESHOLD_ON+s+analogRead(FSR_1)+s+FSR2_THRESHOLD_ON+
                    s+analogRead(FSR_2)+t+roll+s+pitch+t+pwm_signal;

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
      else if (abs(roll) < START_STABLE_THRESH)
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
      if (!fsr_board_stay_on())
        hysteresis ++;
      else
        hysteresis = 0;

      if (hysteresis > CUTOFF_THRESHOLD)
        STATE = OFF;

      // only turn off if we're going at a safe speed (slowly...)
      if (battery_dead && abs(UART.data.rpm) < eRPM_CUTOFF_LIMIT)
        STATE = DEAD;
        
      break;

    case ERROR_:
      vesc.writeMicroseconds(CENTER_THROTTLE);
      break;

    case DEAD:
      vesc.writeMicroseconds(CENTER_THROTTLE);
      if (lc % 60 < 20)
      {
        digitalWrite(led_off, HIGH);
        digitalWrite(led_start, LOW);
        digitalWrite(led_ride, LOW);
      }
      else if (lc % 60 < 40)
      {
        digitalWrite(led_off, LOW);
        digitalWrite(led_start, HIGH);
        digitalWrite(led_ride, LOW);
      }
      else
      {
        digitalWrite(led_off, LOW);
        digitalWrite(led_start, LOW);
        digitalWrite(led_ride, HIGH);
      }
      output = output+"\tBATTERY DEAD, PLEASE CHARGE.";
      break;
  }

  // this takes a long time, so only do it at 1Hz, or every 250 loops
  if (lc % 250 == 0)
  {
    if (UART.getVescValues())
    {
      output = output+t+UART.data.rpm+s+UART.data.inpVoltage+s+UART.data.dutyCycleNow+
                s+UART.data.tachometer+s+UART.data.avgMotorCurrent;
      if (STATE == ERROR_) // we can break out of the error condition
        STATE = OFF;

      /*
          Low-Battery cutoff:
            - 2.8v/cell: turn off no matter what - safer than sorry
            - 3.0v/cell: don't turn off unless duty cycle is < 25%
      */
      if (UART.data.inpVoltage < NUM_CELLS * LOW_LOW_V_CUTOFF)  // protects us in hill climb scenario
        battery_dead = true;
      if (UART.data.inpVoltage < NUM_CELLS * LOW_V_CUTOFF && abs(UART.data.dutyCycleNow) < 25)
        battery_dead = true;
    }
    else
    {
      // toggle LED to show error -- VESC not connected
      digitalWrite(led_off, !digitalRead(led_off));
      output = output+"\tNO VESC DETECTED (check UART)";
      STATE = ERROR_;
    } 
    lc = 0;
  }
  lc ++;
  
  output = output+t+(micros()-loop_start);

  // only print if we asked for it
  if (_verbose)
    Serial.println(output);
  
  // make sure we keep loop time
  bool on_time = false;
  while (micros() - loop_start < int(1000000 / PID_REFRESH_RATE))
  {
    on_time = true;
    digitalWrite(13, LOW);
  }
  if (!on_time)
    digitalWrite(13, HIGH);
}


// helper function to reliably check whether the board TURN ON based on FSRs
bool fsr_board_start()
{
  // if angles are too extreme, return false too
  if (abs(pitch) > PITCH_CUTOFF)
    return false;
  if (abs(roll) > ROLL_CUTOFF)
    return false;
    
  //goal: return true if both are pressed, else rerturn false
  if (analogRead(FSR_1) < FSR1_THRESHOLD_ON && analogRead(FSR_2) < FSR2_THRESHOLD_ON)
    return true;
  else
    return false;
}


// helper function to reliably check whether the board TURN ON based on FSRs
bool fsr_board_stay_on()
{
  // if angles are too extreme, return false too
  if (abs(pitch) > PITCH_CUTOFF)
    return false;
  if (abs(roll) > ROLL_CUTOFF)
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
    // if only one is pressed and we're moving slowly, turn off
    if (abs(UART.data.rpm) < eRPM_CUTOFF_LIMIT)
      return false;
    else
      return true;
  }
  // this means BOTH are PRESSED
  else if (FSRs == 0)
    return true;

  // this should never happen... so turn off if it does
  return false;
}


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
struct IMU_packet read_mpu_6050() 
{
  struct IMU_packet p;
  Wire.beginTransmission(0x68);                      // Start communicating with the MPU-6050
  Wire.write(0x3B);                                  // Send the requested starting register
  Wire.endTransmission();                            // End the transmission
  Wire.requestFrom(0x68, 14);                        // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                     // Wait until all the bytes are received
  p.ax = (Wire.read() << 8 | Wire.read());          // Add the low and high byte to the acc_x variable
  p.ay = (Wire.read() << 8 | Wire.read());          // Add the low and high byte to the acc_y variable
  p.az = (Wire.read() << 8 | Wire.read());          // Add the low and high byte to the acc_z variable
  int temperature = Wire.read() << 8 | Wire.read();  // don't need to keep this value
  p.gx = (Wire.read() << 8 | Wire.read());         // Add the low and high byte to the gyro_x variable
  p.gy = (Wire.read() << 8 | Wire.read());         // Add the low and high byte to the gyro_y variable
  p.gz = (Wire.read() << 8 | Wire.read());         // Add the low and high byte to the gyro_z variable

  return p;
}


// update angle based on readings
void update_state()
{
  // gyro and accel angles
  static double a_p_gyro = 0;
  static double a_r_gyro = 0;
  static double a_y_gyro = 0;
  static double a_r_accel = 0;
  static double a_p_accel = 0;
  
  // start by reading mpu and storing in struct
  struct IMU_packet p = read_mpu_6050();

  // update for calibration errors
  p.ax += ax_cal;
  p.ay += ay_cal;
  p.az += az_cal;
  p.gx += gx_cal;
  p.gy += gy_cal;
  p.gz += gz_cal;
  
  // AFS - 4096 = 1 g!

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz * 65.5)
  a_r_gyro += p.gx * 0.0000611;                                     // Calculate the traveled pitch angle and add this to the angle_pitch variable
  a_p_gyro += p.gy * 0.0000611;                                    // Calculate the traveled roll angle and add this to the angle_roll variable
  a_y_gyro += p.gz * 0.0000611;                                      // Will drift over time, but shouldn't be a big deal because it's all relative

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  // If the IMU has yawed transfer the pitch angle to the roll angle, and vice versa
  a_r_gyro += a_p_gyro * sin(p.gz * 0.000001066);
  a_p_gyro -= a_r_gyro * sin(p.gz * 0.000001066);

  // calculate accelerometer angles
  float a_total = sqrt((p.ax * p.ax) + (p.ay * p.ay) + (p.az * p.az));
  // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  a_r_accel = asin((float)p.ay / a_total) * 57.296;
  a_p_accel = asin((float)p.ax / a_total) * -57.296;

  // accel calibration values
  a_r_accel += 0.00;
  a_p_accel += 0.00;

  // comp filter between gyro and accel
  a_p_gyro = a_p_gyro * 0.99 + a_p_accel * 0.01;
  a_r_gyro = a_r_gyro * 0.99 + a_r_accel * 0.01;
 
  // another comp filter on outputs
  pitch = pitch * 0.9 + a_p_gyro * 0.1;    // Take 90% of the output pitch value and add 10% of the raw pitch value
  roll = roll * 0.9 + a_r_gyro * 0.1;       // Take 90% of the output roll value and add 10% of the raw roll value
  yaw = yaw * 0.9 + a_y_gyro * 0.1;          // Might as well stick a comp filter on the yaw too!
}


// manually called function to calibrate
void calibrate_gyro()
{
  // these are all globals
  gx_cal = 0;
  gy_cal = 0;
  gz_cal = 0;

  ax_cal = 0;
  ay_cal = 0;
  az_cal = 0;

  int cal_num = 250;
  for (int cal_int = 0; cal_int < cal_num ; cal_int ++)
  { 
    struct IMU_packet p = read_mpu_6050();
    gx_cal += p.gx;
    gy_cal += p.gy;
    gz_cal += p.gz;

    ax_cal += p.ax;
    ay_cal += p.ay;
    az_cal += p.az-4096; // we expect 4096 reading if level
  }

  // should be negative to offset the averages we counted (we add this value)
  gx_cal /= -cal_num;
  gy_cal /= -cal_num;
  gz_cal /= -cal_num;
  
  ax_cal /= -cal_num;
  ay_cal /= -cal_num;
  az_cal /= -cal_num;

  // debug
  if (_verbose)
  {
    Serial.println(gx_cal);
    Serial.println(gy_cal);
    Serial.println(gz_cal);
  
    Serial.println(ax_cal);
    Serial.println(ay_cal);
    Serial.println(az_cal);
    
    // help debug - so we can see it and stop autoscroll
    delay(3000);
  }
}
