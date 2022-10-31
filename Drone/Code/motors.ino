//get input from the controller
void getControllerInput()
{
  now_time = millis();

  if(RC_avail() || now_time - rc_update > 25) 
  {
    rc_update = now_time;
    
    for (int i = 0; i < 6; ++i)
    {
      //range -1, 1
      RC_in[i] = RC_decode(i+1);
      //Serial.print(RC_in[i]);
      //Serial.print('\t');
    }
    //Serial.println();
    //Serial.print((RC_in[2] + 1)/2);
    RC_in[1] = -RC_in[1];
    RC_in[2] = (RC_in[2] + 1)/2;
  }
}

//set turn off all motors
void setZero()
{
  for (uint8_t pwmnum = 0; pwmnum < 16; ++pwmnum) {
    pwm.setPWM(pwmnum, 0, 1000);
  }
}

//set the motors to the powers recorder in motor out.
void setToMotors()
{
  for(int i = 0; i < 8; ++i)
  {
    if (motorOut[i] < 0) motorOut[i] = 0;
    if (motorOut[i] > 1) motorOut[i] = 1;
    
    pwm.setPWM(i, 0, map(motorOut[i], 0, 1, 1024, 2048));

    
    //Serial.print(dmap(motorOut[i], 0, 1, 1024, 2048));
    Serial.print(pwm.getPWM(i));
    Serial.print('\t');
  }
  Serial.println();
}

//set the power of the motrs as determined by the throttle
void setThrottle()
{
  //set all the motors to the desired throttle. 
  for (int i = 0; i < 8; i+=1)
  {
    motorOut[i] = RC_in[2];
  }
}
