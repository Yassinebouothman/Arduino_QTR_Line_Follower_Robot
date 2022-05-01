
#include <QTRSensors.h>

#define Kp 0.1 //2 experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 2  //6
// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)  2.3
#define rightMaxSpeed 90 // max speed of the robot
#define leftMaxSpeed 90// max speed of the robot
#define rightBaseSpeed 70 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 70  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define speedturn 90
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
//#define QTR_NO_EMITTER_PIN
//#define EMITTER_PIN

#define rightMotor1 15
#define rightMotor2 14

#define leftMotor1 18
#define leftMotor2 16

#define rightMotorPWM 2
#define leftMotorPWM 3






QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
} , NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN); // sensor connected through analog pins A0 - A7 i.e. digital pins 14-19 in uno

unsigned int sensorValues[NUM_SENSORS];
unsigned int sensors[8];

void setup()
{

  //Serial.begin(9600);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(13, INPUT_PULLUP);
  int i;
  for (int i = 0; i < 250; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead


    qtrrc.calibrate();
  delay(20);
  wait();
  delay(1000); // wait for 2s to position the bot before entering the main loop

  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

}

int lastError = 0;
bool action = false, blanc = false;
void loop()
{

  //while(!digitalRead(13));

  int position = qtrrc.readLine(sensors);
  ////
  //    while(true)
  //    {
  //      position = qtrrc.readLine(sensorValues);
  //      for(int i=0;i<8;i++)
  //      { Serial.print(sensorValues[i]);
  //      Serial.print(" ");
  //      }
  //      Serial.print(" ");
  //      Serial.println(position);
  //    }


  int error;
  error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

  move(1, rightMotorSpeed, 1);//
  move(0, leftMotorSpeed, 1);//forward


///droite
if(sensors[0]>750 && sensors[1]>750 &&  sensors[2]>750 && sensors[3]>750 && sensors[4]>750 && sensors[5]>750 && sensors[6]<400 && sensors[7]<400 && blanc == false )
{
  stop();
  delay(1000);
  droite();
  delay(175);
  stop();
  delay(200);
  avance();
  delay(200);
}

if(sensors[0]>750 && sensors[1]>750 &&  sensors[2]>750 && sensors[3]>750 && sensors[4]>750 && sensors[5]>750 && sensors[6]<400 && sensors[7]<400 && blanc == true )
{
  stop();
  delay(1000);
  droite();
  delay(175);
  stop();
  delay(200);
  avance();
  delay(200);
  if (action == false) action = true;
  else return;
}

////tout noir
if(sensors[0]>750 && sensors[1]>750 &&  sensors[2]>750 && sensors[3]>750&& sensors[4]>750 && sensors[5]>750 && sensors[6]>750 && sensors[7]>750  )
{
  
  blanc = true;
  action = false;

}

////tout blanc
if(sensors[0]<400 && sensors[1]<400 &&  sensors[2]<400 && sensors[3]<400 && sensors[4]<400 && sensors[5]<400 && sensors[6]<400 && sensors[7]<400 && action == false  )
{
  
  avance();
  delay(50);

}

if(sensors[0]<400 && sensors[1]<400 &&  sensors[2]<400 && sensors[3]<400 && sensors[4]<400 && sensors[5]<400 && sensors[6]<400 && sensors[7]<400 && action == true  )
{
 avance();
 delay(200);
 ar();
 delay(50);
 stop();
 delay(1000);
 
 
 if(sensors[0]<400 && sensors[1]<400 &&  sensors[2]<400 && sensors[3]<400 && sensors[4]<400 && sensors[5]<400 && sensors[6]<400 && sensors[7]<400  )
{ 
  ar();
  delay(350);
  droite();
  delay(200);
  action = false;
  }
  else return;
  
}




 
}

void move(int motor, int speed, int direction)
{

  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if (motor == 1) {
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }
}

void wait() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}
void droite()//// correcte
{
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite (leftMotorPWM, 100);
}
void gauche () {
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 0);
}
void avance () {
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 100);
}

void ar () {
  digitalWrite(rightMotor1, LOW );
  digitalWrite(rightMotor2, HIGH );
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 100);
}
void stop () {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 0);
  }
