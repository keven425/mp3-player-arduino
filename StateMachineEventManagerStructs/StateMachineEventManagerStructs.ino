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
StateMachineDuo toggle;
StateMachineTrio forward;
StateMachineTrio backwards;

void onToggle( int event, int param )
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

void setup() {
  Serial.begin(115200);
  Serial.readString(); //Read "away" any potentially buffered data

  //Point the listener function pointer to the respective callback
  toggle.listener = onToggle;
  forward.listener = onForward;
  backwards.listener = onBackwards;

  //Map the different events to the listeners
  //Note that the toggle state machine is only listening to two events
  eventManager.addListener(EVENT_0, toggle.listener);
  eventManager.addListener(EVENT_1, toggle.listener);
  
  eventManager.addListener(EVENT_0, forward.listener);
  eventManager.addListener(EVENT_1, forward.listener);
  eventManager.addListener(EVENT_2, forward.listener);
  
  eventManager.addListener(EVENT_0, backwards.listener);
  eventManager.addListener(EVENT_1, backwards.listener);
  eventManager.addListener(EVENT_2, backwards.listener);

  Serial.println();
  Serial.println("--------------------------------");

  //Initialize all state machines (Each listener will handle the UNDEFINED state in this example).
  toggle.listener(0, 0);
  forward.listener(0, 0);
  backwards.listener(0, 0);
}

void loop() {

  //Keep track of which of the three events we should be posting
  static int index = 0;

  //Handle any events that are in the queue
  eventManager.processEvent();

  //Post events

  Serial.print("------------ [");
  Serial.print(index);
  Serial.println("] ------------");

  switch (index)
  {
    case 0:
      eventManager.queueEvent(EVENT_0, 0); 
      break;

    case 1:
      eventManager.queueEvent(EVENT_1, 0); 
      break;

    case 2:
      eventManager.queueEvent(EVENT_2, 0); 
      break;
  }

  //Cycle through { 0, 1, 2 }
  index = (index + 1) % 3;

  //Wait two seconds 
  delay(2000);

}
