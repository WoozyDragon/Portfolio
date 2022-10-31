/*
The code file dealing with setting the motors without PID
Useful for testing and making sure that everything works before adding more complex things
*/

//the half range of the motion of the stick, in pwm pulse
//full range is 2x this number.
//adjust until drone is appropriately sensetive
#define MOTION_MULTIPLIER 0.05

void setMotion()
{
  //figure out what angle the right stick is pointing at
  //also rotate it by 90 degrees so that 90 is facing forward, then make sure that its [0,360]
  //TODO, rewrite the code to not need fancyMod. 
  float stickAngle = fancyMod(atan2(RC_in[1], RC_in[0]) + M_PI_2, 2*M_PI);
  float stickAngle45 = fancyMod(stickAngle - M_PI_4, 2*M_PI);
  float stickDistance = hypot(RC_in[0], RC_in[1]);

  motorOut[0] += MOTION_MULTIPLIER * stickDistance * -cos(stickAngle);
  motorOut[1] += MOTION_MULTIPLIER * stickDistance * -cos(stickAngle45);
  motorOut[2] += MOTION_MULTIPLIER * stickDistance * -sin(stickAngle);
  motorOut[3] += MOTION_MULTIPLIER * stickDistance * -sin(stickAngle45);
  
  motorOut[4] += MOTION_MULTIPLIER * stickDistance * cos(stickAngle);
  motorOut[5] += MOTION_MULTIPLIER * stickDistance * cos(stickAngle45);
  motorOut[6] += MOTION_MULTIPLIER * stickDistance * sin(stickAngle);
  motorOut[7] += MOTION_MULTIPLIER * stickDistance * sin(stickAngle45);
  
}


//the half range of the powers for turning, in terms of pwm signal.
//The full range is twice this
#define TURN_MULTIPLIER 0.2

//increase the power of all the clockwise motors
//decrease the power of all the counterclockwise motors
void setTurn()
{  
  motorOut[0] += TURN_MULTIPLIER * RC_in[3] * -1;
  motorOut[1] += TURN_MULTIPLIER * RC_in[3];
  motorOut[2] += TURN_MULTIPLIER * RC_in[3] * -1;
  motorOut[3] += TURN_MULTIPLIER * RC_in[3];
  
  motorOut[4] += TURN_MULTIPLIER * RC_in[3] * -1;
  motorOut[5] += TURN_MULTIPLIER * RC_in[3];
  motorOut[6] += TURN_MULTIPLIER * RC_in[3] * -1;
  motorOut[7] += TURN_MULTIPLIER * RC_in[3];
}

//the loop for no PID.
void standardLoop()
{
  setThrottle();
  setMotion();
  setTurn();
}
