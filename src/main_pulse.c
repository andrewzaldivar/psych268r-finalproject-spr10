#pragma config(Sensor, S1,     soundSensor,         sensorSoundDB)
#pragma config(Sensor, S2,     touchSensor,         sensorTouch)
#pragma config(Sensor, S3,     lineSensor,          sensorLightActive)
#pragma config(Sensor, S4,     sonar,               sensorSONAR)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//globals
#define soundBuffer   13
#define turnRadius    750
#define turnIncrement 90
#define turnBack      0
#define turnSpeed     20
#define gototurnSpeed 40
#define goSpeed       30


//initiate variables
short oldsoundMax;
short peakMotorEncoder;
short state;
int rescueBotCount = 0;

//states
#define TEST           0
#define DISPERSAL      1
#define TURNFORSOUND   2
#define GOTOSOUND      3
#define FOUNDPRINCESS  4
#define IMTHEPRINCESS  5
#define BOT_COUNT 5 //Number of rescue bots to assemble before princess is rescued.

#define RESCUEBOT_SOUND_THRESHOLD 50
#define PRINCESS_SILENCE_THRESHOLD 20
#define BEEP_DETECTION_DIFFERENCE 30
#define PRINCESS_FOUND_THRESHOLD 90

//Sound-seeking method
//1 to find loudest sound
//2 to find quitest sound
//3 to use vector addition population encoder
#define USE_LOUDEST    1
#define USE_SOFTEST    2
#define USE_VECTORADD  3
#define TURNFORSOUND_METHOD USE_LOUDEST


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

void DisplayData(short data, bool firstDisplay)
{
	if(firstDisplay)
	{
		nxtDisplayCenteredTextLine(0, "State");
		nxtDisplayCenteredBigTextLine(1, "%d", data);
	}
	else
	{
		nxtDisplayCenteredTextLine(4, "State Value");
		nxtDisplayCenteredBigTextLine(5, "%d", data);
	}
}

void StopMotors() {
	motor[motorA] = 0;
}
void TurnRightIndefinite(short turn_speed) { //Function to rotate the robot to the right. Returns immediately, robot still turns.
	nSyncedTurnRatio = -100; //Makes the wheels turn in opposite directions
	motor[motorA] = turn_speed; //Establish turn power
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
	while(nMotorEncoder[motorA] > -1 * distance * (360/7.25))  //Calculates distance in inches
	{                                                   //as a function of wheel rotations
		motor[motorA] = -30;  //Reverse Motor speed
	}
	motor[motorA] = 0;
	wait1Msec(100);
}

int getMaxSoundForInterval (int interval)
{
	int soundValue = 0;
	int checkValue = 0;
	ClearTimer(T1);
	while (time1[T1] < 500)
	{
		checkValue = SensorValue[soundSensor];
		if (soundValue < checkValue && !bSoundActive)
		{
			soundValue = checkValue;
		}
	}
	return soundValue;
}

int fBeepDetection () //Check and return if the detected sound is in a beep pattern.
{
	int beepDetection = 0;
	ClearTimer (T2);
	wait1Msec (100);
	int initialSound = SensorValue[soundSensor];
	while (time1[T2] < 1000)
	{
		wait1Msec (50);
		int diff = abs (SensorValue[soundSensor] - initialSound);
		if (diff > BEEP_DETECTION_DIFFERENCE)
		{
			beepDetection = 1;
			break;
		}
	}
	return beepDetection;
}

task playRescueTone () // Rescue bots play a tone for 500 msec when the princess is in silent mode.
{
	wait1Msec(100);
	while (1)
	{
		srand(random(13456));
		wait1Msec(random(500)); 
		//To randomly choose an arbitrary wait time to make the rescue calls discrete and not synchronized in case there are more than one rescue bot in the arena.
		if (SensorValue[soundSensor] < PRINCESS_SILENCE_THRESHOLD && !bSoundActive)
		{
			nVolume = 4;
			PlayTone(1184, 50);
			wait1Msec(510);
			break;
		}
	}
}

task main ()
{
	//StartTask(boundaryDetect);
	//state = FOUNDPRINCESS;
	state = IMTHEPRINCESS;
	//state = TEST;
	while(1)
	{
		DisplayData(state,false);
		switch (state)
		{
			case TEST:
				DisplayData(TEST, true);
				DisplayData(SensorValue[soundSensor], false);
				wait1Msec (500);
				break;
				
			case DISPERSAL:
				
				nSyncedMotors         = synchAB;
				nSyncedTurnRatio      = 100;
				motor(motorA) = goSpeed;
				GoReverse(20);
				TurnRight (180);
				state = TURNFORSOUND;
				//wait10Msec(2000);
				break;
				
				
			case TURNFORSOUND:
				//Sound-seeking method
				//USE_LOUDEST: go towards the loudest sound
				//USE_SOFTEST: go away from the quitest sound
				//USE_VECTORADD: use vector addition, averaging all readings
				nSyncedMotors = synchNone;                                              //Will occasionally throw "cannot update slave sync" in nMotorEncoder otherwise
				nMotorEncoder[motorA] = 0;                                              //reset nMotorEncoder motorA
				nMotorEncoder[motorB] = 0;                                              //reset nMotorEncoder motorB - redundant
				nSyncedMotors         = synchAB;                                        //Sync Motors for (CW/CCW?)turn - A master, B slave
				nSyncedTurnRatio      = -100;                                           //Sync for in place turning
				oldsoundMax           = 0;                                              //reset oldsoundMax
				peakMotorEncoder      = 0;                                              //reset turn1peakdBmotorValue
				
				wait10Msec(50);
				short magnitude = 0;
				short motorEncoderTarget = 0;
				int soundReading = 0;
				
				while(nMotorEncoder[motorA] < turnRadius)                               //loop for Turn direction 1 and sample
				{
					motorEncoderTarget = motorEncoderTarget + turnIncrement;
					motor[motorA] = turnSpeed;                                            //turn at turn speed
					while(nMotorEncoder[motorA] < motorEncoderTarget)                     //loop for incremental turn
					{
						wait1Msec(1);
					}
					motor[motorA] = 0;                                                    //Estop
					wait10Msec(30);                                                       //Wait for motor noise to die down
					short ave = 0;
					
					for (short i=0;i < 33;i++)                                            //Average sound readings over 2/3 of a second
					{
						soundReading = SensorValue[soundSensor];
						DisplayData(soundReading,true);
						ave = ave+soundReading;
						wait10Msec(2);
					}
					ave = ave/33;
					
					
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
				}
				oldsoundMax           = 0;                                              //reset oldsoundMax
				switch(TURNFORSOUND_METHOD)
			{
				case USE_LOUDEST:
					motorEncoderTarget = turnRadius - turnIncrement - (peakMotorEncoder*2);
					break;
				case USE_SOFTEST:
					motorEncoderTarget = turnRadius - turnIncrement - peakMotorEncoder + 360;
					break;
				case USE_VECTORADD:
					motorEncoderTarget = turnRadius - turnIncrement - (peakMotorEncoder*2);
					break;
			}
				
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
					if (SensorValue[soundSensor] > oldsoundMax)
					{
						oldsoundMax = SensorValue(soundSensor);
					}
					DisplayData(oldsoundMax,false);
					if (SensorValue[touchSensor]==1)
					{
						motor(motorA) = 0;
						GoReverse(4);
						TurnRight(270);
						motor(motorA) = goSpeed;
						wait10Msec(200);
					}
					if (SensorValue[soundSensor] > PRINCESS_FOUND_THRESHOLD)
					{
						state = FOUNDPRINCESS;                                              //Go to state (FOUNDPRINCESS)
					}
					if(SensorValue[soundSensor] < oldsoundMax - soundBuffer)
					{
						state = TURNFORSOUND;                                               //Go to state (TURNFORSOUND)
					}
				}
				break;
				
				
			case FOUNDPRINCESS:                                                  //FOUNDPRINCESS state
				
				// code for FOUNDPRINCESS here
				StartTask (playRescueTone); //Wait for Princess to shut up and play a tone once to inform her rescue status
				while(1)
				{
					int beepValue = fBeepDetection ();
					if (beepValue == 1) //Check for Beep vs Continuous noise made by the princess. Check 3 times.
					{
						beepValue = fBeepDetection ();
						if (beepValue == 1)
						{
							beepValue = fBeepDetection ();
							if (beepValue == 1)
							{
								DisplayData(state, true);
								DisplayData(1, false);
								motor(motorA) = 0;
							}
						}
					}
					else //When a continuous noise for 3 sec is made by the princess. Check 3 times for continuous tone to make sure the mode is changed to continuous.
					{
						beepValue = fBeepDetection ();
						if (beepValue == 0)
						{
							beepValue = fBeepDetection ();
							if (beepValue == 0)
							{
								DisplayData(state, true);
								DisplayData(0, false);
								state = DISPERSAL;
								break;
							}
						}
					}
				}
				break;
				
				
			case IMTHEPRINCESS:
				
				//Do Princess like stuff
				while (1)
				{
					DisplayData(state, true);
					DisplayData(rescueBotCount, false);
					
					// If the robot is not sufficiently surrounded (rescued) by other robots,
					if (rescueBotCount < BOT_COUNT)
					{
						// Play-mode : short pulse for 500 msec
						nVolume = 4;
						PlayTone(1184, 85); ///Async?!
						wait1Msec (850);
						
						// Silent-mode : listen to sound for 500 msec
						int rescueSoundValue = getMaxSoundForInterval(500);
						if (rescueSoundValue > RESCUEBOT_SOUND_THRESHOLD) //if there is a rescue-bot around making a noise
						{
							rescueBotCount++; //Increment rescue-bot count.
							DisplayData(rescueBotCount, false);
						}
						wait1Msec (100);
					}
					// Else, the robot is fully surrounded/rescued,
					else
					{
						wait1Msec (5100); //Once rescued go silent to kick the rescue bots into disperse mode.
						rescueBotCount = 0; //Reset rescue bot count
						state = DISPERSAL;
						break;
					}
				}
				break;
		}                                                                           //end switch
	}                                                                           //end main while(1) loop
}                                                                             //end task main
