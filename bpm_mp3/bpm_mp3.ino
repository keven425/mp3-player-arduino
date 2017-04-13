/*
 * State Machine example using structs + EventManager library.
 * 
 * Alex Olwal 2017
 * 
 */

#include <Wire.h>
#include "EventManager.h"

//The system has three events
const int EVENT_0 = 0;
const int EVENT_1 = 1;
const int EVENT_2 = 2;

EventManager eventManager;

//Defines a custom structure StateMachineSingle (data type), with one state
struct StateMachineSingle
{
  enum State { UNDEFINED, STATE_ONE }; //One state + an (optional) undefined state
  int currentState = UNDEFINED; //The default is undefined
  void (* listener)(int event, int parameter); //Function pointer that can point to our callback
};
//Create an "alias" [StateMachineSingle] to [struct StateMachineSingle] (better syntax)
typedef struct StateMachineSingle StateMachineSingle; 

//Defines a custom structure StateMachineDuo (data type), with two states
struct StateMachineDuo
{
  enum State { UNDEFINED, STATE_ONE, STATE_TWO }; //Two states + an (optional) undefined state
  int currentState = UNDEFINED; //The default is undefined
  void (* listener)(int event, int parameter); //Function pointer that can point to our callback
};
//Create an "alias" [StateMachineDuo] to [struct StateMachineDuo] (better syntax)
typedef struct StateMachineDuo StateMachineDuo; 

//Defines a custom structure StateMachineDuo (data type), with three states
struct StateMachineTrio
{
  enum State { UNDEFINED, STATE_ONE, STATE_TWO, STATE_THREE };
  int currentState = UNDEFINED;
  void (* listener)(int event, int parameter);
};
//Create an "alias" [StateMachineTrio] to [struct StateMachineTrio] (better syntax)
typedef struct StateMachineTrio StateMachineTrio;

//Create three variables representing state machines, using our custom data types.
//Note that we are using the StateMachineTrio data type twice. (this is the benefit of the struct)
StateMachineDuo player; // states: [PLAYING, STOPPED]
StateMachineDuo bpm_controller; // states: [MANUAL, AUTO]

void setup() {
  Serial.begin(115200);
  Serial.readString(); //Read "away" any potentially buffered data

  //Point the listener function pointer to the respective callback
  player.listener = on_play_input;
  bpm_controller.listener = on_bpm_input;

  //Map the different events to the listeners
  //Note that the toggle state machine is only listening to two events
  eventManager.addListener(EVENT_BTN_PLAY_PAUSE, player.listener); // start/pause
  eventManager.addListener(EVENT_BTN_NEXT, player.listener); // next
  eventManager.addListener(EVENT_BTN_PREV, player.listener); // prev
  eventManager.addListener(EVENT_BTN_FAV, player.listener); // favorite song
  eventManager.addListener(EVENT_DIAL_UP, player.listener); // volume up
  eventManager.addListener(EVENT_DIAL_DOWN, player.listener); // volume down
  eventManager.addListener(EVENT_SET_BPM, player.listener); // set BPM

  eventManager.addListener(EVENT_SLIDER_MOVE, bpm_controller.listener); // bpm slider moved
  eventManager.addListener(EVENT_BPM, bpm_controller.listener); // bpm periodically from IMU
  
  Serial.println();
  Serial.println("--------------------------------");

  //Initialize all state machines (Each listener will handle the UNDEFINED state in this example).
  player.listener(0, 0);
  bpm_controller.listener(0, 0);
}

void loop() {

  //Keep track of which of the three events we should be posting
  static int index = 0;

  //Handle any events that are in the queue
  eventManager.processEvent();

  // poll sensors, post events
  // TODO

  //Post events
  // eventManager.queueEvent(EVENT_0, 0); 

}

// ************************
// * The following needs to be changed
// ************************
void onPlay( int event, int param )
{
  Serial.print("onToggle(   ");
  Serial.print(event);
  Serial.print(", ");  
  Serial.print(param);
  Serial.print(") current state[");
  Serial.print(toggle.currentState);
  Serial.print("]");

  //choose the code block based on the current state
  // UNDEFINED -> STATE_ONE -> STATE_TWO -> STATE_ONE ...
  switch (toggle.currentState) {
    
    case toggle.UNDEFINED:
      // Do Intialization things
      toggle.currentState = toggle.STATE_ONE; //move to the next state
      break;

    case toggle.STATE_ONE:
      if (event == EVENT_0) {
        toggle.currentState = toggle.STATE_TWO;
      }
      break;

    case toggle.STATE_TWO:
      if (event == EVENT_1) {
        toggle.currentState = toggle.STATE_ONE;
      }
      break;
  }

  Serial.print("->[");
  Serial.print(toggle.currentState);
  Serial.println("]");
}

void onForward( int event, int param )
{
  Serial.print("onForward(  ");
  Serial.print(event);
  Serial.print(", ");  
  Serial.print(param);
  Serial.print(") current state[");
  Serial.print(forward.currentState);
  Serial.print("]");

  //choose the code block based on the current state
  // UNDEFINED -> STATE_ONE -> STATE_TWO -> STATE_THREE -> STATE_ONE ...
  switch (forward.currentState) {

    case forward.UNDEFINED:
      // Do Intialization things
      forward.currentState = forward.STATE_ONE;
      break;

    case forward.STATE_ONE:
      if (event == EVENT_0) {
        forward.currentState = forward.STATE_TWO;
      }
      break;

    case forward.STATE_TWO:
      if (event == EVENT_1) {
        forward.currentState = forward.STATE_THREE;
      }
      break;

    case forward.STATE_THREE:
      if (event == EVENT_2) {
        forward.currentState = forward.STATE_ONE;
      }
      break;
  }

  Serial.print("->[");
  Serial.print(forward.currentState);
  Serial.println("]");
}

void onBackwards( int event, int param )
{
  Serial.print("onBackwards(");
  Serial.print(event);
  Serial.print(", ");  
  Serial.print(param);
  Serial.print(") current state[");
  Serial.print(backwards.currentState);
  Serial.print("]");

  //choose the code block based on the current state
  // UNDEFINED -> STATE_THREE -> STATE_TWO -> STATE_ONE -> STATE_THREE ...
  switch (backwards.currentState) {

    case backwards.UNDEFINED:
      // Do Intialization things
      backwards.currentState = backwards.STATE_THREE;
      break;

    case backwards.STATE_ONE:
      if (event == EVENT_2) {
        backwards.currentState = backwards.STATE_THREE;
      }
      break;

    case backwards.STATE_TWO:
      if (event == EVENT_1) {
        backwards.currentState = backwards.STATE_ONE;
      }
      break;

    case backwards.STATE_THREE:
      if (event == EVENT_0) {
        backwards.currentState = backwards.STATE_TWO;
      }
      break;
  }

  Serial.print("->[");
  Serial.print(backwards.currentState);
  Serial.println("]");
}

