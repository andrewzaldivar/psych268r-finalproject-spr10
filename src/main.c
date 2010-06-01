#pragma config(Sensor, S1,     touchSensor,         sensorTouch)
#pragma config(Sensor, S4,     soundSensor,         sensorSoundDB)

//globals
#define soundBuffer   13
#define foundSoundLvl 90
#define turnRadius    720
#define turnIncrement 90
#define turnBack      0
#define turnSpeed     20
#define gototurnSpeed 40
#define goSpeed       30

//initiate variables
short oldsoundMax;
short peakMotorEncoder;
short state;

//states
#define TURNFORSOUND   1
#define GOTOSOUND      2
#define FOUNDPRINCESS  3
#define IMTHEPRINCESS  4

//Sound-seeking method
//1 to find loudest sound
//2 to find quitest sound
//3 to use vector addition population encoder
#define USE_LOUDEST    1
#define USE_SOFTEST    2
#define USE_VECTORADD  3

void DisplayData(short data, bool firstDisplay)
{
  if(firstDisplay)
  {
    nxtDisplayCenteredTextLine(0, "Sound Reading");
    nxtDisplayCenteredBigTextLine(1, "%d", data);
  }
  else
  {
    nxtDisplayCenteredTextLine(4, "Max Sound Reading");
    nxtDisplayCenteredBigTextLine(5, "%d", data);
  }
}

void VectorAddition(short &oldDegrees, short &oldMagnitude,short newDegrees, short newMagnitude)
{
  short xDecomp = (oldMagnitude * cosDegrees(oldDegrees)) + (newMagnitude * cosDegrees(newDegrees));
  short yDecomp = (oldMagnitude * sinDegrees(oldDegrees)) + (newMagnitude * sinDegrees(newDegrees));
  oldMagnitude = (xDecomp^2 + yDecomp^2)^.5;
  oldDegrees = radiansToDegrees(acos(xDecomp/oldMagnitude));
  if (xDecomp < 0)
  {
    if (yDecomp < 0) {oldDegrees = oldDegrees + 180;}
    else {oldDegrees = oldDegrees + 90;}
  }
  else { if (yDecomp < 0) {oldDegrees = oldDegrees + 270;} }
  //DisplayData(oldDegrees,false);
}


void TurnRight(short degree) //Function to rotate the robot "degree" degrees to the right
{
  nSyncedTurnRatio = -100; //Makes the wheels turn in opposite directions
  nMotorEncoder[motorA] = 0; // Initialize wheel counter
  while(nMotorEncoder[motorA] < degree)
  {
    motor[motorA] = 25; //Establish turn power
  }
  motor[motorA] = 0;
  nSyncedTurnRatio = 100;
  wait1Msec(100); //Pause to reduce jerking between operations
}

void GoReverse(short distance)  //input is the distance we wish to go in inches
{
  nMotorEncoder[motorA] = 0;
  nSyncedTurnRatio = 100;

  //1 rotation is approximately 7.25 inches
  while(nMotorEncoder[motorA] > -1*distance*(360/7.25))  //Calculates distance in inches
  {                                                      //as a function of wheel rotations
    motor[motorA] = -30;  //Reverse Motor speed
  }
  motor[motorA] = 0;
  wait1Msec(100);
}

task main ()
{
  state = TURNFORSOUND;
  motor[motorA] = goSpeed;
  motor[motorB] = goSpeed;
  wait10Msec(50);
  motor[motorA] = 0;
  motor[motorB] = 0;
  nDatalogSize = 2000;
  while(1)
  {

    switch (state)
    {

    case TURNFORSOUND:

      short TURNFORSOUND_METHOD = USE_LOUDEST;
      //Sound-seeking method
      //USE_LOUDEST: go towards the loudest sound
      //USE_SOFTEST: go away from the quitest sound
      //USE_VECTORADD: use vector addition, averaging all readings


      AddToDatalog(101);

      motor[motorA] = 0;
      nSyncedMotors = synchNone;                                              //Will occasionally throw "cannot update slave sync" in nMotorEncoder otherwise
      nMotorEncoder[motorA] = 0;                                              //reset nMotorEncoder motorA
      nMotorEncoder[motorB] = 0;                                              //reset nMotorEncoder motorB - redundant
      nSyncedMotors         = synchAB;                                        //Sync Motors for (CW/CCW?)turn - A master, B slave
      nSyncedTurnRatio      = -100;                                           //Sync for in place turning
      oldsoundMax           = 0;                                              //reset oldsoundMax
      if(TURNFORSOUND_METHOD == USE_SOFTEST) {oldsoundMax = 100;}
      peakMotorEncoder      = 0;                                              //reset turn1peakdBmotorValue



      wait10Msec(50);
      short magnitude = 0;
      short motorEncoderTarget = 0;
      int soundReading = 0;


      //Loop through 1 circle and sample sound levels
      while(nMotorEncoder[motorA] <= turnRadius)
      {
        short ave = 0;
        AddToDatalog(1,nMotorEncoder[motorA]/2);
        for (short i=0;i < 33;i++)                                            //Average sound readings over 2/3 of a second
        {
          soundReading = SensorValue[soundSensor];
          AddToDatalog(2,soundReading);
          DisplayData(soundReading,true);
          ave = ave+soundReading;
          wait10Msec(2);
        }
        ave = ave/33;

        //Decision block for what to do with recorded sound levels
        if (TURNFORSOUND_METHOD==USE_LOUDEST)
        {
          DisplayData(oldsoundMax,false);
          if(ave > oldsoundMax)                                                 //compare sound sensor values current vs. old.  If higher
          {
            oldsoundMax = ave;                                                  //update value to current
            peakMotorEncoder = nMotorEncoder[motorA];                           //record motor position
          }
        }
        else if (TURNFORSOUND_METHOD==USE_SOFTEST)
        {
          DisplayData(oldsoundMax,false);
          if(ave < oldsoundMax)                                                 //compare sound sensor values current vs. old.  If higher
          {
            oldsoundMax = ave;                                                  //update value to current
            peakMotorEncoder = nMotorEncoder[motorA];                           //record motor position
          }
        }
        else
        {
          VectorAddition(peakMotorEncoder,magnitude,nMotorEncoder[motorA]/2,ave^2);
          DisplayData(ave,true);
          DisplayData(peakMotorEncoder,false);

        }

        //Incremental turn motor control
        motorEncoderTarget = motorEncoderTarget + turnIncrement;
        motor[motorA] = turnSpeed;                                            //turn at turn speed
        while(nMotorEncoder[motorA] < motorEncoderTarget)                     //loop for incremental turn
        {
          wait1Msec(1);
        }
        motor[motorA] = 0;                                                    //Estop
        wait10Msec(30);                                                       //Wait for motor noise to die down


      }
      oldsoundMax           = 0;                                              //reset oldsoundMax
      switch(TURNFORSOUND_METHOD)
      {
      case USE_LOUDEST:
        motorEncoderTarget = turnRadius - peakMotorEncoder;
        break;
      case USE_SOFTEST:
        motorEncoderTarget = turnRadius - peakMotorEncoder + 360;
        break;
      case USE_VECTORADD:
        motorEncoderTarget = turnRadius - (peakMotorEncoder*2);
        break;
      }
      SaveNxtDatalog();


      //Turn to selected direction (value in motorEncoderTarget)
      nSyncedMotors = synchNone;                                              //Will occasionally throw "cannot update slave sync" in nMotorEncoder otherwise
      nMotorEncoder[motorA] = 0;
      nMotorEncoder[motorB] = 0;
      nSyncedMotors         = synchBA;                                        //Sync Motors for (CW/CCW?)turn - B master, A slave
      nSyncedTurnRatio      = -100;
      while(nMotorEncoder[motorB] < motorEncoderTarget)
      {
        motor(motorB) = gototurnSpeed;                                        //Turn at gototurnSpeed rate
      }
      motor(motorB) = 0;                                                      //Estop
      nSyncedMotors         = synchAB;
      nSyncedTurnRatio      = 100;                                            //Sync for forward motion
      state = GOTOSOUND;                                                      //turnforsound complete, go to state (GOTOSOUND)
      break;




    case GOTOSOUND:                                                           //GOTOSOUND state

      motor(motorA) = goSpeed;
      wait10Msec(10);                                                         //Get going before checking sound
      oldsoundMax = SensorValue[soundSensor];                                 //Set old sound max == current sensor lvl
      while (state==GOTOSOUND)                                                //go loop - go forward while current sound > oldsound - soundbuffer
      {
        motor(motorA) = goSpeed;
        DisplayData(SensorValue[soundSensor],true);
        if (SensorValue[soundSensor] > oldsoundMax) { oldsoundMax = SensorValue(soundSensor); }
        DisplayData(oldsoundMax,false);
        if (SensorValue[touchSensor]==1)
        {
          motor(motorA) = 0;
          GoReverse(4);
          TurnRight(270);
          motor(motorA) = goSpeed;
          wait10Msec(200);
        }
        if (SensorValue[soundSensor] > foundSoundLvl)
        {
          state = FOUNDPRINCESS;                                              //Go to state (FOUNDPRINCESS)
        }
        if(SensorValue[soundSensor] < oldsoundMax - soundBuffer)
        {
          state = TURNFORSOUND;                                               //Go to state (TURNFORSOUND)
        }
      }
      break;




    case FOUNDPRINCESS:                                                       //FOUNDPRINCESS state

      // code for FOUNDPRINCESS here
      // stop, wiggle, squeel, whatever...
      // state = NEWROUND;                                                    // end of FOUNDPRINCESS code, return bots to NEWROUND state
      while(true)
      {
        DisplayData(0,false);
      }
      break;


    case IMTHEPRINCESS:

      //Do Princess like stuff

      //if(all bots have found me)
      //{
      // happy beeping
      // state = NEWROUND;
      //{
      //elseif(time limit)
      //{
      // state = NEWROUND;
      //{
      break;
    }                                                                           //end switch
  }                                                                           //end main while(1) loop
}                                                                             //end task main
