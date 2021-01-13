#include <Wire.h>
#include <Servo.h>

bool fsr_board_on();

int PWM_PIN = 11;

// FSR stuff - pin definitions
int FSR_1 = 0;
int FSR_2 = 1;

// save time, just use library, maybe it's slow
bool _verbose = true;

// easy to config as a servo, note: library modified to run at 250Hz
Servo vesc;

// LEDs to indicate states: subject to change
int led_off = 8;
int led_start = 9;
int led_ride = 10;

// states to dictate what mode the wheel is in
#define OFF                     1
#define ON                      2
#define FSR1_THRESHOLD_ON       170
#define FSR2_THRESHOLD_ON       170
#define CENTER_THROTTLE         1500

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

  Serial.println("LOOP BEGIN");

  // signal we're good to go
  digitalWrite(led_off, HIGH);
  digitalWrite(led_start, LOW);
  digitalWrite(led_ride, LOW);
}

void loop()
{
  static int STATE = OFF;

  /////////////////////////////////////////////////////////////////////////////////////
  // Now, update motor speed based on state and PID
  /////////////////////////////////////////////////////////////////////////////////////
  switch (STATE)
  {
    case OFF:
      digitalWrite(led_off, HIGH);
      digitalWrite(led_start, LOW);
      digitalWrite(led_ride, LOW);

      vesc.writeMicroseconds(CENTER_THROTTLE);

      // see if we need to turn on
      if (fsr_board_on())
        STATE = ON;
      break;

    case ON:
      digitalWrite(led_off, HIGH);
      digitalWrite(led_start, LOW);
      digitalWrite(led_ride, LOW);

      vesc.writeMicroseconds(CENTER_THROTTLE+100);

      // see if we need to turn on
      if (!fsr_board_on())
        STATE = OFF;
      break;
  }

  Serial.println(STATE);
  /////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
}

// helper function to reliably check whether the board should be on based on FSRs
bool fsr_board_on()
{
  int fsr1 = analogRead(FSR_1);
  int fsr2 = analogRead(FSR_2);

  Serial.print(fsr1);
  Serial.print(" ");
  Serial.println(fsr2);
  
  //goal: return true if both are pressed, else rerturn false
  if (fsr1 < FSR1_THRESHOLD_ON && fsr2 < FSR2_THRESHOLD_ON)
    return true;
  else
    return false;
}
