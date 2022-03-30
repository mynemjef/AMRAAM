#include <Wire.h>
#include <Pixy2.h>
#include <Adafruit_PWMServoDriver.h>
#include <GY521.h>

#define MPU_addr1             0x68
#define MIN_PULSE_WIDTH_Y     275
#define MAX_PULSE_WIDTH_Y     475
#define MIN_PULSE_WIDTH_ROLL  -25
#define MAX_PULSE_WIDTH_ROLL  25
#define FREQUENCY             60
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Pixy2 pixy;

float xa, ya, za, gyroRoll;

int i;
int MIN_PULSE_WIDTH_X = 225, MAX_PULSE_WIDTH_X = 375;
int lastY = 103;
int lastX = 157;

bool relStatus = false;
bool multiLaunch = false; //set to true if 2 missiles will be launched
bool side = false; //set to false if this is the left missile, true if it is the right one, leave false if only 1 missile is attached

//IDs 
//1 - L = left, R = right, T = top, B = bottom, Rel = missile release motor; 
//2 - R = has to be mapped in reverse, S = has to be mapped normally; 

int motorL_R = 0;
int motorR_S = 1;
int motorT_R = 2;
int motorB_S = 3;
int motorRel = 4;
 
void setup() 
{
  Serial.begin(115200); 
  Wire.begin();
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pixy.init();
  //delay(500);
  //pwm.setPWM(motorRel, 0, 315);
  //delay(500);
  if (multiLaunch) {
    if(side) {
      MIN_PULSE_WIDTH_X = 200;
      MAX_PULSE_WIDTH_X = 350;
    }
    else {
      MIN_PULSE_WIDTH_X = 250;
      MAX_PULSE_WIDTH_X = 400;
    }
  }
}

void moveStraight(int controlX, int controlY, int motorR_S, int motorB_S, int roll) {
  int pulse_wideX, pulse_wideY, pulse_wide_roll, posX, posY, posRoll;

  posX = controlX;
  posY = controlY;
  posRoll = roll;

  pulse_wideX = map(posX, 0, 315, MIN_PULSE_WIDTH_X, MAX_PULSE_WIDTH_X);
  pulse_wideY = map(posY, 0, 207, MAX_PULSE_WIDTH_Y, MIN_PULSE_WIDTH_Y);
  pulse_wide_roll = map(roll, -60, 60, MIN_PULSE_WIDTH_ROLL, MAX_PULSE_WIDTH_ROLL);
  
  pulse_wideX += pulse_wide_roll;
  pulse_wideY += pulse_wide_roll;

  pwm.setPWM(motorR_S, 0, pulse_wideY);
  pwm.setPWM(motorB_S, 0, pulse_wideX);
}

void moveReverse(int controlX, int controlY, int motorL_R, int motorT_R, int roll) {
  int pulse_wideX, pulse_wideY, pulse_wide_roll, posX, posY, posRoll;
  
  posX = controlX;
  posY = controlY;
  posRoll = roll;

  pulse_wideX = map(posX, 0, 315, MAX_PULSE_WIDTH_X, MIN_PULSE_WIDTH_X);
  pulse_wideY = map(posY, 0, 207, MIN_PULSE_WIDTH_Y, MAX_PULSE_WIDTH_Y);
  pulse_wide_roll = map(roll, -60, 60, MIN_PULSE_WIDTH_ROLL, MAX_PULSE_WIDTH_ROLL);
  
  pulse_wideX += pulse_wide_roll;
  pulse_wideY += pulse_wide_roll;

  pwm.setPWM(motorL_R, 0, pulse_wideY);
  pwm.setPWM(motorT_R, 0, pulse_wideX);
}

void relMissile(int motorRel) {
  relStatus = true;
  pwm.setPWM(motorRel, 0, 140);
  delay(500);
  //pwm.setPWM(motorRel, 0, 500);
  delay(500);
  //ignite
}

void getRickRolled(){
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1, 6, true);

  xa = Wire.read() << 8 | Wire.read();
  ya = Wire.read() << 8 | Wire.read();
  za = Wire.read() << 8 | Wire.read();
  
  gyroRoll = atan2(ya , za) * (180.0 / PI) - 2.2;
}


void loop() {
  
  getRickRolled();

  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    for (i=0; i<pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1){
        moveStraight(pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y, motorR_S, motorB_S, gyroRoll);
        moveReverse(pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y, motorL_R, motorT_R, gyroRoll);
        lastX = pixy.ccc.blocks[i].m_x;
        lastY = pixy.ccc.blocks[i].m_y;
      }
      else if (pixy.ccc.blocks[i].m_signature == 2 && pixy.ccc.blocks[i].m_age >= 180) {
        relMissile(motorRel);
      }
    }
    //if (relStatus && proximity <= 1000) {explode();}
  }
  else {
    moveStraight(lastX, lastY, motorR_S, motorB_S, gyroRoll);
    moveReverse(lastX, lastY, motorL_R, motorT_R, gyroRoll);
  }
}
 
