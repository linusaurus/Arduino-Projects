/*  
  Skylight control program.
  David Odell 2014
  dvid.odell@gmail.com
*/

#include <Encoder.h>

/* Constants to change in different setups */
#define RAIN_THRESHOLD  -1 // Reading on 'wet' sensor (Sensor setup property).
#define ACTUATOR_CLOSED 1000 // Pot reading on closed actuator (Actuator property).
#define ACTUATOR_OPEN 7   // Pot reading on fully open actuator (Actuator property).
#define ENCODER_P_R  100    // Points per revolution (Encoder property).
#define ROTATION_RANGE  5   // Number of turns for full open/closed.
#define SPEED  1010         // motor speed (0 - 1023).
#define WAIT_TIME 500      // wait for user to input, milliseconds.
#define DEBUG  0            // If 1, will print to serial helpful debugging data. 9600.

/* Constants DO NOT CHANGE */
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3

/* Pin definitions */
// Actuator
#define MOTOR_A_PIN 2
#define MOTOR_B_PIN 4
#define PWM_PIN     9
#define CS_PIN      0
#define EN_PIN      6
#define POT_PIN     5
// Rain
#define RAIN_PIN    4
// Encoder
#define ENC_A_PIN   3
#define ENC_B_PIN   11

// Variables
int encoderpos = 0;
int old_encoderpos = 0;
boolean reacted_to_rain = false;

// Computed constant MOTIONFACTOR translates encoder steps to 'actuator units'.
float MOTIONFACTOR = (float) (ACTUATOR_CLOSED - ACTUATOR_OPEN) / (float) (ROTATION_RANGE * 4 * ENCODER_P_R);

// Initializes encoder object.
Encoder encoder(ENC_A_PIN, ENC_B_PIN);

void setup()
{ 
  
  // Initialize digital outputs.
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  // Initialize braked.
  digitalWrite(MOTOR_A_PIN, LOW);
  digitalWrite(MOTOR_B_PIN, LOW);
  analogWrite(PWM_PIN, 0);
  
  // Only use Serial if debugging.
  if (DEBUG == 1) {
    Serial.begin(9600);
    delay(100); 
    Serial.println("Ready");
    Serial.print("Motion factor: ");
    Serial.println(MOTIONFACTOR);
  }
  
  // Close actuator if not already closed.
  motorGoTo(ACTUATOR_CLOSED);
  encoder.write(0);
}

void loop()
{

  encoderpos = encoder.read();
  // Ignore small changes in encoder position, could be error/fluctuation.
  if ((encoderpos < (old_encoderpos - 3)) || (encoderpos > (old_encoderpos + 3))) {
    // Wait for user to set target, smooths motion.
    delay(WAIT_TIME); 
    // Clip the encoder value to a ceiling
    encoderpos = encoder.read();
    motorGoTo((int) ACTUATOR_CLOSED + (MOTIONFACTOR * encoderpos));
    old_encoderpos = encoderpos;
    
    if (DEBUG == 1) {
      Serial.println("Encoder triggered");
      Serial.print("Encoder position: ");
      Serial.println(encoderpos);
      Serial.print("Actuator target: ");
      Serial.println((int) ACTUATOR_CLOSED + (MOTIONFACTOR * encoderpos));
    }
  }
  
  // Slow loop cycle to sensible level.
  delay(1);
}

/* 
  Motor functions:

 motorGo() will set a motor going in a specific direction.
 The motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motorGoTo(target) moves the actuator until the potentiometer reading
 from the actuator is equal to the target value.
 */
void motorGo(uint8_t direct, uint8_t pwm)
{
  if (direct <=4)
  {
    // Set H-bridge signals.
    if (direct <=1)
      digitalWrite(MOTOR_A_PIN, HIGH);
    else
      digitalWrite(MOTOR_A_PIN, LOW);

    if ((direct==0)||(direct==2))
      digitalWrite(MOTOR_B_PIN, HIGH);
    else
      digitalWrite(MOTOR_B_PIN, LOW);

    analogWrite(PWM_PIN, pwm);
  } 
}

void motorGoTo(int targetpos)
{
  // Clip values to the allowable range.
  if (targetpos > ACTUATOR_CLOSED) {
    targetpos = ACTUATOR_CLOSED;
  }
  if (targetpos < ACTUATOR_OPEN) {
    targetpos = ACTUATOR_OPEN;
  }
  
  // Blocking while loop. Move until in the right position.
  while (analogRead(POT_PIN) != targetpos)
  {
    if (targetpos > analogRead(POT_PIN)) {
      motorGo(CW, SPEED);
    }
    if (targetpos < analogRead(POT_PIN)) {
      motorGo(CCW, SPEED);
    }
  }
  // When finished, stop the motion.
  motorGo(BRAKEGND, 0);
}

