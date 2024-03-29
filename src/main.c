#pragma config(Sensor, S4,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S1,     soundSensor,         sensorSoundDB)
#pragma config(Sensor, S2,     touchSensor,         sensorTouch)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//globals
#define soundBuffer   15
#define soundLv_hi    70
#define soundLv_low   38
#define turnRadius    730
#define maxGoLength   3000
#define searchBuffer  10

//bot 8
//#define turnIncrement 106
//bot 10 & 12
//#define turnIncrement 102
//bot 5 & 6
//#define turnIncrement 99
//bot 2
//#define turnIncrement 95

//bot ?
#define turnIncrement 100

#define turnBack      0
#define turnSpeed     30
#define gototurnSpeed 30
#define goSpeed       30

#define TRAP_LIGHT_THRESHOLD 41
#define DISTRESS_TIMEOUT 35000

//initiate variables
short oldsoundMax;
short peakMotorEncoder;
short state;
bool light = true;
int goLength;
float minAve = 100;
float maxAve = 0;

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
#define USE_3_SOFTEST  4




void DisplayData(short data, bool firstDisplay)
{
  if(firstDisplay)
  {
    nxtDisplayCenteredTextLine(0, "Sound Reading");
    nxtDisplayCenteredBigTextLine(1, "%d", data);
  }
  else
  {
    nxtDisplayCenteredTextLine(4, "Maximum Sound");
    nxtDisplayCenteredBigTextLine(5, "%d", data);
  }
}

bool inTrap() {
  if (light) {
    if(SensorValue[lightSensor] <= TRAP_LIGHT_THRESHOLD) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }
}

// Check to see if the behavior state should change
// returns behavior state
short CheckBehaviorState() {

  if (inTrap() && SensorValue[soundSensor] < soundLv_low)
  {
    return IMTHEPRINCESS;
  }
else if( ( (SensorValue[touchSensor] ==1)&&(SensorValue[soundSensor] >=soundLv_low) ) || (inTrap()) ||(SensorValue[soundSensor] >=soundLv_hi) )
  {
    return FOUNDPRINCESS;
  }
  else
  {
    return TURNFORSOUND;
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

void WallDetectionBehavior() {
  motor(motorA) = 0;
  GoReverse(4);
  TurnRight(270);
  motor(motorA) = goSpeed;
  wait10Msec(200);
}

void produceDistressSignal() {

  nVolume = 4;
  // note: intrinsic void PlayTone(const int frequency,  const int durationIn10MsecTicks)
  //short tmp = DISTRESS_TIMEOUT;
  for(short i=0; i < DISTRESS_TIMEOUT/1000; i++)
  {
  PlayTone(1184,(1000));
  wait10Msec(1000);
}
}

void moveAwayFromTrap() {
  // Try to get away from the trap
  // by travelling in reverse for 5 secs
  wait10Msec(200);
  motor(motorA) = -1*goSpeed;
  wait1Msec(1300);
  motor(motorA) = 0;
  TurnRight(180);
  TurnRight(180);
  state = TURNFORSOUND;
}

void imThePrincess() {
  motor(motorA) = 0;
  motor(motorB) = 0;
  wait10Msec(200);
  produceDistressSignal();
  moveAwayFromTrap();
  state = GOTOSOUND;
}

void foundPrincess() {
  DisplayData(SensorValue[soundSensor],false);
  motor(motorA) = 0;
  motor(motorB) = 0;
  wait10Msec(200);
  while(SensorValue[soundSensor] > 25) {
    DisplayData(SensorValue[soundSensor],true);
    wait10Msec(10);
  }
  moveAwayFromTrap();
  state = GOTOSOUND;
}

void soundSeeking() {

  /*
  motor[motorA] = 0; //Quiet the motors so we can decide if we hear anything
  wait1Msec(500); //Wait for the motors to STFU
  if (SensorValue[soundSensor] < searchBuffer)
  {
    maxAve = 75; //This is set so it knows how far to go (75 is relatively short)
    state = GOTOSOUND;
    return; //If we're not hearing anything, just keep trucking (no actual search)
  }
*/
  int soundReadingIndex[10] = {0,1,2,3,4,5,6,7,0,1};
  float soundReadings[8];

  short TURNFORSOUND_METHOD = USE_3_SOFTEST;
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
  short numTurns = 0;
  if(TURNFORSOUND_METHOD == USE_SOFTEST)
  {
    oldsoundMax = 100;
  }

  peakMotorEncoder      = 0;                                      //reset turn1peakdBmotorValue

  wait10Msec(50);
  short magnitude = 0;
  short motorEncoderTarget = 0;
  int soundReading = 0;


  //Loop through 1 circle and sample sound levels
  while((numTurns<8)&&(CheckBehaviorState()==TURNFORSOUND)&&SensorValue[touchSensor]==0)
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

    soundReadings[numTurns] = ave;

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
    else if (TURNFORSOUND_METHOD==USE_VECTORADD)
    {
      VectorAddition(peakMotorEncoder,magnitude,nMotorEncoder[motorA]/2,ave);
      DisplayData(ave,true);
    }

    //Incremental turn motor control
    motorEncoderTarget = motorEncoderTarget + turnIncrement;
    motor[motorA] = turnSpeed;                                            //turn at turn speed
    while(nMotorEncoder[motorA] < motorEncoderTarget)                     //loop for incremental turn
    {
      wait1Msec(1);
    } // end incremental turn motor control loop

    motor[motorA] = 0;                                                    //Estop
    wait10Msec(30);                                                       //Wait for motor noise to die down
    numTurns++;
  } // end while loop

  SaveNxtDatalog();
  state = CheckBehaviorState();
  if (state!=TURNFORSOUND)
  {
    return;
  }
  else if (SensorValue[touchSensor]==1) {
    WallDetectionBehavior();
    return;
  }
  else
  {
    oldsoundMax = 0; // reset old sound
    switch(TURNFORSOUND_METHOD)
    {
    case USE_3_SOFTEST:
      // Initialize temporary parameters for checking the lowest readings of sounds
      float tmp = 0;
      minAve = 100;
      maxAve = 0;
      int minAveIndex;
      for (int i = 0; i < 8; i++)
      {
        tmp = soundReadings[soundReadingIndex[i]]
        + soundReadings[soundReadingIndex[i + 1]]
        + soundReadings[soundReadingIndex[i + 2]];

        if (tmp < minAve)
        {
          minAve = tmp;
          minAveIndex = i+1;
        }

//        if (tmp > maxAve)
//        {
//          maxAve = tmp;
//        }
      }

//      if (maxAve - minAve > soundBuffer)
//      {
        motorEncoderTarget = (turnIncrement*8) - ((soundReadingIndex[minAveIndex]+4)*turnIncrement);
//      }
//      else
//      {
//        motorEncoderTarget = 0; //Just go straight if there's nothing but crap sound readings
//      }
      AddToDatalog(3,(soundReadingIndex[minAveIndex]+1)*45);
      AddToDatalog(4,20);
      break;
    case USE_LOUDEST:
      motorEncoderTarget = turnRadius - peakMotorEncoder;
      break;
    case USE_SOFTEST:
      motorEncoderTarget = turnRadius - peakMotorEncoder + 360;
      break;
    case USE_VECTORADD:
      motorEncoderTarget = 720 - (peakMotorEncoder*2);
      break;
    }

    //      	SaveNxtDatalog();

    //Turn to selected direction (value in motorEncoderTarget)
    nSyncedMotors = synchNone;                                              //Will occasionally throw "cannot update slave sync" in nMotorEncoder otherwise
    nMotorEncoder[motorA] = 0;
    nMotorEncoder[motorB] = 0;
    nSyncedMotors         = synchBA;                                        //Sync Motors for (CW/CCW?)turn - B master, A slave
    nSyncedTurnRatio      = -100;
    if(motorEncoderTarget < 0)
    {
      motorEncoderTarget = motorEncoderTarget+720;
    }
    while(nMotorEncoder[motorB] < motorEncoderTarget)
    {
      motor(motorB) = gototurnSpeed;                                        //Turn at gototurnSpeed rate
    }
    motor(motorB) = 0;                                                      //Estop
    nSyncedMotors         = synchAB;
    nSyncedTurnRatio      = 100;                                            //Sync for forward motion
    state = GOTOSOUND;
  }                                                   //turnforsound complete, go to state (GOTOSOUND)
}

task main() {
  wait10Msec(1);
  wait10Msec(20);
  nDatalogSize = 2000;
  motor[motorA] = goSpeed;
  motor[motorB] = goSpeed;
  wait10Msec(50);
  motor[motorA] = 0;
  motor[motorB] = 0;
  //  nDatalogSize = 2000;
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  state = TURNFORSOUND;


  while(1) {
    wait10Msec(20);
    switch (state) {

      //GOTOSOUND State
    case GOTOSOUND:                                             //GOTOSOUND state
      //motor(motorA) = goSpeed;
      wait10Msec(10);                                          //Get going before checking sound
      motor(motorA) = 0;
      oldsoundMax = 0;
      nMotorEncoder[motorA] = 0;

      //goLength = maxGoLength * 1 - (maxAve / 100);
      goLength = 1440;

      while((nMotorEncoder[motorA] < goLength)&& (CheckBehaviorState()==TURNFORSOUND))
      {
        motor(motorA) = goSpeed;

        DisplayData(SensorValue[soundSensor],true);
        if(SensorValue[soundSensor] > oldsoundMax)
        {oldsoundMax = SensorValue[soundSensor];}
        DisplayData(oldsoundMax,false);

        if (SensorValue[touchSensor]==1) {
          WallDetectionBehavior();
        }

        state = CheckBehaviorState();
      } // end GOTOSOUND while loop

      break;
    case IMTHEPRINCESS:
      imThePrincess();
      break;
    case FOUNDPRINCESS:
      foundPrincess();
      break;
    case TURNFORSOUND:
      soundSeeking();
      break;
    }
  }
}
