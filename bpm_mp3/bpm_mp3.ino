/*
 * State Machine example using structs + EventManager library.
 * 
 */

#include <Wire.h>
#include "EventManager.h"
//#include <EventManager.h>

// pins
const int PIN_BTN_PLAY_PAUSE = 2;
const int PIN_BTN_NEXT = 3;
const int PIN_BTN_PREV = 4;

//The system has three events
const int EVENT_BTN_PLAY_PAUSE = 0; // start/pause
const int EVENT_BTN_NEXT = 1; // next
const int EVENT_BTN_PREV = 2; // prev
const int EVENT_SET_BPM = 3; // set BPM
const int EVENT_DIAL_UP = 4; // volume up
const int EVENT_DIAL_DOWN = 5; // volume down
const int EVENT_SLIDER_MOVE = 6; // bpm slider moved
const int EVENT_BPM = 7; // bpm periodically from IMU

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
StateMachineDuo player; // states: [PAUSED, PLAYING]
StateMachineSingle volume_controller; // states: [VOLUME]
StateMachineDuo mode_controller; // states: [MANUAL_BPM, AUTO_BPM]

void setup() {
  Serial.begin(115200);
  Serial.readString(); //Read "away" any potentially buffered data

  // set up pins
  pinMode(PIN_BTN_PLAY_PAUSE, INPUT);
  pinMode(PIN_BTN_NEXT, INPUT);
  pinMode(PIN_BTN_PREV, INPUT);

  //Point the listener function pointer to the respective callback
  player.listener = on_player_input;
  volume_controller.listener = on_volume_input;
  mode_controller.listener = on_mode_input;

  //Map the different events to the listeners
  //Note that the toggle state machine is only listening to two events
  eventManager.addListener(EVENT_BTN_PLAY_PAUSE, player.listener); // start/pause
  eventManager.addListener(EVENT_BTN_NEXT, player.listener); // next
  eventManager.addListener(EVENT_BTN_PREV, player.listener); // prev
  eventManager.addListener(EVENT_SET_BPM, player.listener); // set BPM
  
  eventManager.addListener(EVENT_DIAL_UP, volume_controller.listener); // volume up
  eventManager.addListener(EVENT_DIAL_DOWN, volume_controller.listener); // volume down
  
  eventManager.addListener(EVENT_SLIDER_MOVE, mode_controller.listener); // bpm slider moved
  eventManager.addListener(EVENT_BPM, mode_controller.listener); // bpm periodically from IMU
  
  Serial.println();
  Serial.println("--------------------------------");

  //Initialize all state machines (Each listener will handle the UNDEFINED state in this example).
  player.listener(0, 0);
  volume_controller.listener(0, 0);
  mode_controller.listener(0, 0);
}

void loop() {

  // read the state of the pushbutton value:
  int btn_play_pause = digitalRead(PIN_BTN_PLAY_PAUSE);

  if (btn_play_pause == HIGH) {
      Serial.println("button play pause pressed");
  }
  
  //Handle any events that are in the queue
//  eventManager.processEvent();

  // poll sensors, post events
  // TODO

  //Post events
  // eventManager.queueEvent(EVENT_0, 0); 

}

// ************************
// * The following needs to be changed
// ************************
void on_player_input( int event, int param )
{
  Serial.print("on_player_input(   ");
  Serial.print(event);
  Serial.print(", ");  
  Serial.print(param);
  Serial.print(") current state[");
  Serial.print(player.currentState);
  Serial.print("]");

  //choose the code block based on the current state
  // UNDEFINED -> STATE_ONE -> STATE_TWO -> STATE_ONE ...
  switch (player.currentState) {
    
    case player.UNDEFINED:
      // Do Intialization things
      player.currentState = player.STATE_ONE; //move to paused state
      break;

    case player.STATE_ONE: // paused
      if (event == EVENT_BTN_PLAY_PAUSE) {
        player.currentState = player.STATE_TWO; // toggle to playing state
      }
      if (event == EVENT_BTN_NEXT) {
        // no-op
      }
      if (event == EVENT_BTN_PREV) {
        // no-op
      }
      if (event == EVENT_SET_BPM) {
        // no-op
      }
      break;

    case player.STATE_TWO: // playing
      if (event == EVENT_BTN_PLAY_PAUSE) {
        player.currentState = player.STATE_ONE; // toggle to paused state
      }
      if (event == EVENT_BTN_NEXT) {
        // TODO
      }
      if (event == EVENT_BTN_PREV) {
        // TODO
      }
      if (event == EVENT_SET_BPM) {
        // TODO
      }
      break;
  }

  Serial.print("->[");
  Serial.print(player.currentState);
  Serial.println("]");
}

void on_volume_input( int event, int param )
{
  Serial.print("on_volume_input(  ");
  Serial.print(event);
  Serial.print(", ");  
  Serial.print(param);
  Serial.print(") current state[");
  Serial.print(volume_controller.currentState);
  Serial.print("]");

  //choose the code block based on the current state
  // UNDEFINED -> STATE_ONE -> STATE_TWO -> STATE_THREE -> STATE_ONE ...
  switch (volume_controller.currentState) {

    case volume_controller.UNDEFINED:
      // Do Intialization things
      volume_controller.currentState = volume_controller.STATE_ONE;
      break;

    case volume_controller.STATE_ONE:
      if (event == EVENT_DIAL_UP) {
        // stay in current state
        // TODO
      }
      if (event == EVENT_DIAL_DOWN) {
        // stay in current state
        // TODO
      }
      break;
  }

  Serial.print("->[");
  Serial.print(volume_controller.currentState);
  Serial.println("]");
}

void on_mode_input( int event, int param )
{
  Serial.print("on_mode_input(");
  Serial.print(event);
  Serial.print(", ");  
  Serial.print(param);
  Serial.print(") current state[");
  Serial.print(mode_controller.currentState);
  Serial.print("]");

  //choose the code block based on the current state
  // UNDEFINED -> STATE_THREE -> STATE_TWO -> STATE_ONE -> STATE_THREE ...
  switch (mode_controller.currentState) {

    case mode_controller.UNDEFINED:
      // Do Intialization things
      // TODO: initialize based on slider position
      mode_controller.currentState = mode_controller.STATE_ONE;
      break;

    case mode_controller.STATE_ONE: // manual BPM
      if (event == EVENT_SLIDER_MOVE) {
        // TODO: 
        // if bpm in manual range
        //    convert to BPM
        //    emit EVENT_SET_BPM event to player
        // else if bpm in auto range
        //    change to AUTO state
      }
      if (event == EVENT_BPM) {
        // no-op
      }

    case mode_controller.STATE_TWO: // auto BPM
      if (event == EVENT_SLIDER_MOVE) {
        // TODO
        // if bpm in manual range
        //    change to MANUAL state
        // else if bpm in auto range
        //    no-op
        mode_controller.currentState = mode_controller.STATE_ONE;
      }
      if (event == EVENT_BPM) {
        // TODO
        // read BPM from IMU arm swing
        // emit EVENT_SET_BPM event to player
      }
      break;
  }

  Serial.print("->[");
  Serial.print(mode_controller.currentState);
  Serial.println("]");
}

