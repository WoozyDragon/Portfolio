#include <Wire.h>
#include <math.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//where all the remote controller values are held
float RC_in[6];

//where the motor output values are held. 
//starts on the front motor and moves clockwise
float motorOut[8];

float motorMin[8]{1024,1024,1024,1024,1024,1024,1024,1024};

//the event for the gyroscope
sensors_event_t event0;

//needed for the controller update loop
long rc_update;
long now_time;

//the angle of the robot 
double xAngle;
double yAngle;
double zAngle;

void setup()
{
  // setup the pwm file 
  setup_pwmRead();
  pwmSetup();
  Wire.setClock(400000);

  //set up debugging stuff. Comment out for actual 'release'
  setup_debug();

  //set up the gyro 
  BnoSetup();

  //set up the PID stuff
  PIDSetup();

  //Serial.println("1\t2\t3\t4\t5\t6");

  //wait, to allow any setup stuff to finish (motor intializing, imu calibration, etc)
  delay(3000);
}

//a copy of the arudino map funtion, but with doubles as inputs instead of ints
double dmap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//debug setup (Serial)
void setup_debug()
{
  //Initalize serial
  Serial.begin(9600);
  //Serial.println("Hi!");
}

//setup the pwm board
void pwmSetup()
{
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(250);

  //turn off all motors, to satisfy failsafe conditions. 
  for (uint8_t pwmnum = 0; pwmnum < 16; ++pwmnum) {
    pwm.setPWM(pwmnum, 0, 1024);
  }
}


//setup the gyroscopes.
void BnoSetup()
{
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);

}

double xSpeed;
double xLast {0};

// a cumulative angle, from (-infinity, infinity). 
// as opposed to the standard angles, which are [-180, 180]
double xGlobal {0};

//partially taken from the example from adafruit's website
void getAngle()
{
  bno.getEvent(&event0);

  imu::Vector<3> velocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  xAngle = event0.orientation.x;
  xSpeed = velocity.x();
  yAngle = event0.orientation.y;
  zAngle = event0.orientation.z;

  double deltaAngle = (xAngle) - (xLast);

  if (deltaAngle < -180)
      deltaAngle += 360;
  else if (deltaAngle > 180)
      deltaAngle -= 360;

  xGlobal += deltaAngle;
  xLast = xAngle;
  //Serial.print(xGlobal);
  //Serial.println('\t');
}

//returns x mod y, but works with floats and doubles, too. 
double fancyMod(double x, double y)
{
  return x - (y * floor(x/y));
}

void loop() {
  getControllerInput();

  getAngle();

  //if the arming swtich is off...
  //turn off all the motors and exit the loop
  if (RC_in[4] <= 0) {
    setZero();
    PIDOff();
    return;
  }

  standardLoop();
  //PIDLoop();

  setToMotors();

}
