#pragma config(Sensor, S1,     mic,                 sensorSoundDBA)
#pragma config(Sensor, S2,     touch,               sensorTouch)
#pragma config(Sensor, S3,     sonar,               sensorSONAR)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// By Kalin Agrawal
// Last edit: 2010-05-30

// GENERAL INSTRUCTIONS:
// Based on our assigned work, we should only need to do the following
//  - Add your new #define state with a new number for it.
//  - Add fixed parameters (e.g. thresholds, etc.) to the #define section below.
//  - Add any global variable defs.
//  - Add your new task, using something like the name of your state.
//  - Add your new function to provide state transition, E.g. int GetNextState_WAIT_START().
//  - Add any background tasks (e.g. continous sensory processing)
//  - Add your state as a new case to these three generic methods:
//     - GetStateChange
//     - StartState
//     - StopState
//  - Add any initialization code you might need to InitRobotRunOnce().



//////////////////////////////////////////////////////
////// GLOBALS ///////////////////////////////////////
//////////////////////////////////////////////////////


// ADD YOUR STATE CUSTOMIZATION
//
// State enumeration definitions. (Note: no "=" sign or ";")
// Since these are just labels, the order does not matter!
//
// INSTRUCTIONS:
// Do not reorder or change their numbers, just add a new one.
// USE ALL CAPS.
//
#define WAIT_START 1
#define GAME_OVER 2
#define WAIT 3
#define AID 4


// ADD YOUR STATE CUSTOMIZATION
//
// Parameters for the robot should all go here.
//
// INSTRUCTIONS:
// In the rest of the code do NOT use hard-coded constants.
// Use a CAPITALIZED_VARIABLE and define it and describe it here.
//
#define WAIT_START_TIME_MS 1000 // time to wait in WAIT_START
#define DISP_UPDATE_PERIOD_MS 10  // how often the display updates
#define INIT_ROBOT_DELAY_MS 500   // delay after program starts

#define SCREAM_LENGTH_TMS 100 // * 10 msecs
#define SCREAM_VOL 1
#define SCREAM_FREQ 1066

#define SUFFICIENTLY_SURROUNDED_RATIO 0.25
#define CLOSE_ENOUGH_DIST_CM 40
#define NUM_SURR_RATIOS 5

// constant rotation encoder click values
float INCH=(360.00/7.00);
float TILE=(360.00/7.00)*9.00*(10.00/9.80); //a tile with correction
float TURN=(360.00/7.00)*3.14*(15.00/14.00); // 1/4 turn with correction
int TURNSPEED = 30;



// ADD YOUR STATE CUSTOMIZATION
//
// Global variables
//
// INSTRUCTIONS:
// Add any additional global variables here only.
//
int state = -1;
int current_ratio = 0;
float surrounded_ratios[NUM_SURR_RATIOS];
bool is_surrounded = false;



//start turning right
void right(){
  motor[motorB] = -TURNSPEED;
  motor[motorC] = +TURNSPEED;
}

void resetSurroundedRatios() {
  current_ratio = 0; // current ratio in storage;
  for (int i=0; i<NUM_SURR_RATIOS; i++) {
    surrounded_ratios[i] = 0.0;
  }
}



//////////////////////////////////////////////////////
////// STATE TASKS and STATE TRANSITION checks  //////
//////////////////////////////////////////////////////


// STATE
//
// Wait for a touch on the touch sensor to set this off
// as an Agent in Distress (AID).
task Wait() {
  while ( true ) {
    wait1Msec( 100 );
  }
}
// TRANSITION
//
int GetNextState_WAIT() {
  if ( SensorValue(touch) ) {
    return AID;
  }
  else {
    return 0;
  }
}


// STATE
//
task Aid() {

  // Start out assuming robot is not surrounded.
  is_surrounded = false;

  while ( true ) {

    // Reset the counts of samples
    int num_total_samples = 0;
    int num_close_enough_samples = 0;

    // Reset the motor encoder so this starts spinning
    // around another revolution.
    nMotorEncoder[motorB] = 0;
    int clicks = 4 * TURN; // 360 degree turn

    resetSurroundedRatios();

    bool local_is_surrounded = false;

    // Keep doing these things until the robot is surrounded
    while ( !local_is_surrounded ) {

      // Play a "scream" tone if one is not already playing
      if ( !bSoundActive ) {
        // PlayImmediateTone( SCREAM_FREQ , SCREAM_LENGTH_TMS );
      }

      // Increment our sample count per 360-degree turn.
      num_total_samples++;

      // If the sample was close enough, increment the close
      // enough count.
      if ( SensorValue(sonar) < CLOSE_ENOUGH_DIST_CM ) {
        num_close_enough_samples++;
      }

      // Calculate the current surrounded ratio
      surrounded_ratios[current_ratio] = (float)num_close_enough_samples / (float)num_total_samples;

      // keep turning
      right();


      // If a full circle has been turned
      if ( nMotorEncoder[motorB] < 0 - clicks ) {

        // We turned a full circle around.

        // Only on full turns do we check if the
        // robot is sufficiently surrounded
        if ( surrounded_ratios[current_ratio] > (float) SUFFICIENTLY_SURROUNDED_RATIO ) {
          local_is_surrounded = true;

          motor[motorB] = 0; //turn off both motors
          motor[motorC] = 0; //turn off both motors

    is_surrounded = true;
          wait1Msec( 10 );
          break;
        }
        else {

        // Reset the counts of samples
        num_total_samples = 0;
        num_close_enough_samples = 0;

        // Reset the motor encoder so this starts spinning
        // around another revolution.
        nMotorEncoder[motorB] = 0;

        // increment which surrounded_ratios we are using (mod'd)
        current_ratio = (current_ratio + 1) % NUM_SURR_RATIOS;
        }
      }



      wait1Msec( 10 );

    }

    // The robot is sufficiently surrounded

    // Shut down the motors and hang out while the
    // operate reads the values on the display
    // for use in setting thresholds for actual use.

    motor[motorB] = 0; //turn off both motors
    motor[motorC] = 0; //turn off both motors

    wait1Msec( 10 );

    is_surrounded = true;

  }
}
// TRANSITION
//
// Switch out of the AID state only when the robot
// is sufficiently surrounded.
int GetNextState_AID() {
  if ( is_surrounded ) {
    return WAIT;
  }
  else {
    return 0;
  }
}








// STATE
//
// The intial state where the robot is waiting
// for something to start it.  See getChangeState_WAIT_START().
task WaitStart() {
  while ( true ) {
    wait1Msec( 100 );
  }
}

// TRANSITION
//
// At the moment, if we are in WAIT_START
// then we should always transition to the next state.
int GetNextState_WAIT_START() {
  if ( time1[T1] > WAIT_START_TIME_MS ) {
    return WAIT;
  }
  else {
    return 0;
  }
}



// STATE
//
// The intial state where the robot is waiting
// for something to start it.  See getChangeState_WAIT_START().
task GameOver() {
  while ( true ) {
    wait1Msec( 100 );
  }
}
// TRANSITION
//
// At the moment, if we are in GAME_OVER
// then we should never transition to another state.
int GetNextState_GAME_OVER() {
  return 0;
}





///////////////////////////////////////////////////////
////// BACKGROUND TASKS, E.G. SENSORY PROCESSING //////
///////////////////////////////////////////////////////


// BACKGROUND TASK
//
// Displays any information that is useful.
task UpdateDisplay() {
  while ( true ) {

    nxtDisplayTextLine( 7 , "State: %d", state );
    nxtDisplayTextLine( 6 , "Sonar: %d", SensorValue(sonar) );
    nxtDisplayTextLine( 5 , "IsSur: %d", (int)is_surrounded );

    // Display the accumulated surrounded ratios
    for (int j=0; j<NUM_SURR_RATIOS ; j++) {
      if (current_ratio==j) {
        nxtDisplayTextLine( j , "Curr R: %f", surrounded_ratios[j] );
      }
      else {
        nxtDisplayTextLine( j , "     R: %f", surrounded_ratios[j] );
      }
    }


    wait1Msec( DISP_UPDATE_PERIOD_MS ); // wait a bit

  }
}



//////////////////////////////////////////////////
////// ADD CASE STATEMENTS TO THESE METHODS //////
//////////////////////////////////////////////////


// ADD YOUR STATE CUSTOMIZATION
// DO NOT USE (called automatically)
//
// INSTRUCTIONS:
// Make sure you just enter a new case in using the given example.
// It should only set the "response" variable to the state that
// the robot should be set TO based on current conditions of the
// robot.  What those current conditions are is up to the coders.
// Please use the suggested function naming convention for
// checking the conditions of state transition.
//
int GetStateChange( int s ) {

  // this method will return "response" as the state to send
  // the robot to, given that it is in state "s".
  int response = -1; // this should never be returned as -1. It should change below.

  // checks if should switch from state "s" to something else.
  switch ( s ) {
    case WAIT_START:
      response = GetNextState_WAIT_START();
      break;
    case GAME_OVER:
      response = GetNextState_GAME_OVER();
      break;
    case WAIT:
      response = GetNextState_WAIT();
      break;
    case AID:
      response = GetNextState_AID();
      break;
    //case FOO:
      //response = getNextState_FOO();
      //break;
    // ... (ADD NEW CASES / STATES HERE)
    default:
      break;
  }

  return response;

}


// ADD YOUR STATE CUSTOMIZATION
// DO NOT USE (called automatically)
//
// This is automatically called by the main task's state transitor loop.
//
// INSTRUCTIONS:
//  - Make sure you just enter a new case in using the given example.
//  - Make sure you call StartTask for your state's given task.
//
void StartState( int s ) {
  // Start the appropriate task for state s
  switch ( s ) {
    case WAIT_START:
      StartTask( WaitStart );
      break;
    case GAME_OVER:
      StartTask( GameOver );
      break;
    case WAIT:
      StartTask( Wait );
      break;
    case AID:
      StartTask( Aid );
      break;
    //case FOO:
      //StartTask( Foo );
      //break;
    // ... (ADD NEW CASES / STATES HERE)
    default:
      break;
  }
}


// ADD YOUR STATE CUSTOMIZATION
// DO NOT USE (called automatically)
//
// This is automatically called by the main task's state transitor loop.
//
// INSTRUCTIONS:
//  - Make sure you just enter a new case in using the given example.
//  - Make sure you call StopTask for your state's given task.
//  - Make sure your task shuts down "cleanly".
//  - Make sure to shut down all the motors your task was using.
//
void StopState( int s ) {
  // Stop the appropriate task for state s
  switch ( s ) {
    case WAIT_START:
      StopTask( WaitStart );
      break;
    case GAME_OVER:
      StopTask( GameOver );
      break;
    case WAIT:
      StopTask( Wait );
      break;
    case AID:
      StopTask( Aid );
      break;
    //case FOO:
      //motor[motorA] = 0;
      //motor[motorB] = 0;
      //motor[motorC] = 0;
      //StopTask( Foo );
      //break;
    // ... (ADD NEW CASES / STATES HERE)
    default:
      break;
  }
}





///////////////////////////////////////////////
////// ADD INITIALIZATION TO THIS METHOD //////
///////////////////////////////////////////////


// TOUCH MINIMALLY, ONLY IF NECESSARY
// DO NOT USE (called automatically)
//
// This is called only once at the very beginning of the
// robots main task.
//
// "Contracts" wit respect to motor encoder initializations,
// global variable initialization, etc. is established here.
//
// The "*" comments should remain as headers for major
// things this method needs to accomplish before returning
// control to the main task's while loop.
void InitRobotRunOnce() {

  // * Initial wait for 1/2 second to ignore initial readings of the Sensors.
  // Allow operator to move away.
  wait1Msec(INIT_ROBOT_DELAY_MS);

  // * Clear encoders
  nMotorEncoder[motorA] = 0;
  nMotorEncoder[motorB] = 0;
  nMotorEncoder[motorC] = 0;

  // * Setup motors sync (?)
  //
  // TBD

  // * Initialize global variables
  //
  // TBD
  //
  ClearTimer(T1); // timer for waiting to start game
  nVolume = SCREAM_VOL;
  resetSurroundedRatios();

  // * Start Background Tasks
  StartTask( UpdateDisplay );

  // * Set the beginning state and start its task
  state = WAIT_START;
  StartState( state );

}





////////////////////////////////////////////
////// DO NOT TOUCH FOLLOWING METHODS //////
////////////////////////////////////////////


// DO NOT TOUCH
// DO NOT USE (called automatically)
//
// Calls initialization code.
// Then runs an infinite loop that checks a generic method
// to determine if and when a state should be switched.
// This task handles the timing of calling the startState and
// stopState methods.
//
// No specialization should be done in this method.  Leave it
// alone.  See the instructions at the top of the file.
task main() {

  // Initialization and setup
  // (remember that "contracts" wrt motor encoders, timers,
  // motors, etc. should be established).
  InitRobotRunOnce();

  // State Transitor
  while ( true ) {

    // Generic method for getting if we should change from the current state
    // and what the next state should be.
    // ( next_state == 0 means do not change state.)
    int next_state = GetStateChange( state );

    // Only if there is a change in state do we stop and start tasks, etc.
    // Assume that next_state == 0 or negative means that we should
    // not change the state.
    if ( next_state ) {
      StopState( state ); // stops the current state
      state = next_state; // updates the current state variable
      StartState( next_state ); // starts the given state
    }

  }

}