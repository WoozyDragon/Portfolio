#include <AutoPID.h>

//for moving
#define KP 2
#define KD 0
#define KI 0

double yawSet {0};
double pitchSet {0};
double rollSet {0};

double yawOut {0};
double pitchOut {0};
double rollOut {0};

#define PID_MIN -0.15
#define PID_MAX 0.15

AutoPID yaw(&xGlobal, &yawSet, &yawOut, PID_MIN, PID_MAX, KP, KI, KD);

AutoPID pitch(&yAngle, &pitchSet, &pitchOut, PID_MIN, PID_MAX, KP, KI, KD);
AutoPID roll(&zAngle, &rollSet, &rollOut, PID_MIN, PID_MAX, KP, KI, KD);

//setup to run before loop
void PIDSetup() {
  yaw.setTimeStep(50);
  
  pitch.setTimeStep(50);
  
  roll.setTimeStep(50);
}

//turn off the PID loops
void PIDOff() {
  yaw.stop();
  pitch.stop();
  roll.stop();
}

//update the PID loops
void PIDUpdate() {
  yaw.run();
  pitch.run();
  roll.run();
}

//adjust how tilty the drone is going to be
#define MAX_PID_TILT 0.2
#define MAX_PID_YAW 0.5

void PIDSetpoint() {
  yawSet = RC_in[3] * MAX_PID_YAW;

  //if (abs(yawSet - xGlobal) > 40) yawSet = xGlobal;

  pitchSet = RC_in[1] * MAX_PID_TILT;

  rollSet = -RC_in[0] * MAX_PID_TILT;
}

//take the pid output and send it to the motors
void PIDOutput() {
  setThrottle();
  
  //yaw
  //increase the power of the clockwise (even) motors 
  //and decrease the power of the counterclockwise (odd) motors
  for (int i = 0; i < 8; i += 2) {
    motorOut[i] += yawOut;
  }
  for (int i = 1; i < 8; i += 2) {
    motorOut[i] -= yawOut;
  }

  //pitch
  motorOut[0] += pitchOut;
  motorOut[1] += pitchOut * 0.707;
  
  motorOut[3] -= pitchOut * 0.707;
  motorOut[4] -= pitchOut;
  motorOut[5] -= pitchOut * 0.707;

  motorOut[7] += pitchOut * 0.707;

  //roll
  motorOut[1] += rollOut * 0.707;
  motorOut[2] += rollOut;
  motorOut[3] += rollOut * 0.707;

  motorOut[5] -= rollOut * 0.707;
  motorOut[6] -= rollOut;
  motorOut[7] -= rollOut * 0.707;  
}


//loop to call from main
void PIDLoop() {
  PIDSetpoint();
  PIDUpdate();
  PIDOutput();
}
