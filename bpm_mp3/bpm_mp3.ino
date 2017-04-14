/*
 * State Machine example using structs + EventManager library.
 * 
 */

#include <Wire.h>
#include "EventManager.h"
#include "AudioDevice.h"
//#include "Imu2Bpm.h"

// constants
const int DEBOUNCE_DELAY = 50; // 50 millisec

// pins
const int PIN_BTN_PLAY_PAUSE = 2;
const int PIN_BTN_NEXT = 3;
const int PIN_BTN_PREV = 4;
const int PIN_MP3_TX = 6;
const int PIN_MP3_RX = 7;
const int PIN_SLIDER = A0; // sliding potentiometer
const int PIN_SLIDER_LED = A1;

const int ROTARY_ANGLE_SENSOR = A0;
const float ADC_REF = 3.3; //reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
                 //board switches to 3V3, the ADC_REF should be 3.3
const float GROVE_VCC = 3.3; //VCC of the grove interface is normally 5v
const int FULL_ANGLE = 300; //full value of the rotary angle is 300 degrees

const int EVENT_BTN_PLAY_PAUSE_DOWN = 0; // start/pause
const int EVENT_BTN_NEXT_DOWN = 1; // next
const int EVENT_BTN_PREV_DOWN = 2; // prev
const int EVENT_SET_BPM = 3; // set BPM
const int EVENT_DIAL_CHANGE = 4; // volume change
const int EVENT_SLIDER_CHANGE = 5; // bpm slider moved
const int EVENT_BPM = 6; // bpm periodically from IMU
const int EVENT_BTN_PLAY_PAUSE = 7; // button high/low events
const int EVENT_SLIDER = 8;

EventManager eventManager;
AudioDevice audio(PIN_MP3_TX, PIN_MP3_RX, mp3a);
// imu to bpm converter
//Imu2Bpm imu_2_bpm;

int volume = 127;


//Defines a custom structure StateMachineSingle (data type), with one state
struct StateMachineSingle
{
  enum State { UNDEFINED, STATE_ONE }; //One state + an (optional) undefined state
  int currentState = UNDEFINED; //The default is undefined
  void (* listener)(int event, int parameter); //Function pointer that can point to our callback
};
typedef struct StateMachineSingle StateMachineSingle; 

//Defines a custom structure StateMachineDuo (data type), with two states
struct StateMachineDuo
{
  enum State { UNDEFINED, STATE_ONE, STATE_TWO }; //Two states + an (optional) undefined state
  int currentState = UNDEFINED; //The default is undefined
  void (* listener)(int event, int parameter); //Function pointer that can point to our callback
};
typedef struct StateMachineDuo StateMachineDuo; 

//Defines a custom structure StateMachineDuo (data type), with three states
struct StateMachineTrio
{
  enum State { UNDEFINED, STATE_ONE, STATE_TWO, STATE_THREE };
  int currentState = UNDEFINED;
  void (* listener)(int event, int parameter);
};
typedef struct StateMachineTrio StateMachineTrio;

//Defines a custom structure StateMachineBtn (data type), with two states
struct StateMachineBtn
{
  enum State { UNDEFINED, UP, DOWN };
  int currentState = UNDEFINED; //The default is undefined
  int lastDebounceTime = 0;
  void (* listener)(int event, int parameter); //Function pointer that can point to our callback
};
typedef struct StateMachineBtn StateMachineBtn; 

//Defines a custom structure StateMachineBtn (data type), with two states
struct PotentiometerController
{
  int currentValue = 0; // slider or rotary potentiometer value
  void (* listener)(int event, int parameter); //Function pointer that can point to our callback
};
typedef struct PotentiometerController PotentiometerController; 

//Create three variables representing state machines, using our custom data types.
//Note that we are using the StateMachineTrio data type twice. (this is the benefit of the struct)
StateMachineDuo player; // states: [PAUSED, PLAYING]
StateMachineSingle volume_controller; // states: [VOLUME]
StateMachineDuo mode_controller; // states: [MANUAL_BPM, AUTO_BPM]
StateMachineBtn play_pause_btn; // states: [UP, DOWN]
PotentiometerController slider_controller; // keep internal state of slider reading


void setup() {
  Serial.begin(115200);
  Serial.readString(); //Read "away" any potentially buffered data

  audio.initHardware();
  audio.setVolume(volume);  

  // set up pins
  pinMode(PIN_BTN_PLAY_PAUSE, INPUT);
  pinMode(PIN_BTN_NEXT, INPUT);
  pinMode(PIN_BTN_PREV, INPUT);
  pinMode(PIN_SLIDER_LED, OUTPUT);
  // digitalWrite(PIN_SLIDER_LED, HIGH);
  // pinMode(PIN_SLIDER, INPUT);

  //Point the listener function pointer to the respective callback
  player.listener = on_player_input;
  volume_controller.listener = on_volume_input;
  mode_controller.listener = on_mode_input;
  play_pause_btn.listener = on_play_pause_btn_input;
  slider_controller.listener = on_slider_input;

  //Map the different events to the listeners
  //Note that the toggle state machine is only listening to two events
  eventManager.addListener(EVENT_BTN_PLAY_PAUSE_DOWN, player.listener); // start/pause
  eventManager.addListener(EVENT_BTN_NEXT_DOWN, player.listener); // next
  eventManager.addListener(EVENT_BTN_PREV_DOWN, player.listener); // prev
  eventManager.addListener(EVENT_SET_BPM, player.listener); // set BPM
  
  eventManager.addListener(EVENT_DIAL_CHANGE, volume_controller.listener); // volume change
  
  eventManager.addListener(EVENT_SLIDER_CHANGE, mode_controller.listener); // bpm slider moved
  eventManager.addListener(EVENT_BPM, mode_controller.listener); // bpm periodically from IMU
  
  // listener for buttons, edge detector
  eventManager.addListener(EVENT_BTN_PLAY_PAUSE, play_pause_btn.listener); // button high/low from polling

  // listener for slider, rotary potentiometer. detect value changes
  eventManager.addListener(EVENT_SLIDER, slider_controller.listener);

  Serial.println();
  Serial.println("--------------------------------");

  //Initialize all state machines (Each listener will handle the UNDEFINED state in this example).
  player.listener(0, 0);
  volume_controller.listener(0, 0);
  mode_controller.listener(0, 0);
  play_pause_btn.listener(0, 0);
  slider_controller.listener(0, 0);

}

void loop() {
  
  //Handle any events that are in the queue
 eventManager.processAllEvents();

  // poll sensors, post events
  // TODO
  // read the state of the pushbutton value:
  int btn_play_pause = digitalRead(PIN_BTN_PLAY_PAUSE);
  int slider = analogRead(PIN_SLIDER);
  
  eventManager.queueEvent(EVENT_BTN_PLAY_PAUSE, btn_play_pause); 
  eventManager.queueEvent(EVENT_SLIDER, slider); 

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
      if (event == EVENT_BTN_PLAY_PAUSE_DOWN) {
        Serial.println("start playing");
        audio.play();
        player.currentState = player.STATE_TWO; // toggle to playing state
      }
      if (event == EVENT_BTN_NEXT_DOWN) {
        // no-op
      }
      if (event == EVENT_BTN_PREV_DOWN) {
        // no-op
      }
      if (event == EVENT_SET_BPM) {
        // no-op
      }
      break;

    case player.STATE_TWO: // playing
      if (event == EVENT_BTN_PLAY_PAUSE_DOWN) {
        Serial.println("stop playing");
        audio.pause();
        player.currentState = player.STATE_ONE; // toggle to paused state
      }
      if (event == EVENT_BTN_NEXT_DOWN) {
        // TODO
      }
      if (event == EVENT_BTN_PREV_DOWN) {
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
      if (event == EVENT_DIAL_CHANGE) {
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
      if (event == EVENT_SLIDER_CHANGE) {
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
      break;

    case mode_controller.STATE_TWO: // auto BPM
      if (event == EVENT_SLIDER_CHANGE) {
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

void on_play_pause_btn_input( int event, int param ) {
  // debounce
  if ((millis() - play_pause_btn.lastDebounceTime) <= DEBOUNCE_DELAY) {
    return;
  }
  
  switch (play_pause_btn.currentState) {

    case play_pause_btn.UNDEFINED:
      play_pause_btn.currentState = play_pause_btn.UP;
      break;

    case play_pause_btn.UP: 
      if (param == HIGH) {
        play_pause_btn.currentState = play_pause_btn.DOWN;
      } 
      break;

    case play_pause_btn.DOWN: 
      if (param != HIGH) {
        play_pause_btn.currentState = play_pause_btn.UP;
        // emit up edge event / button press
        eventManager.queueEvent(EVENT_BTN_PLAY_PAUSE_DOWN, 0); 
      }
      break;
  }

  play_pause_btn.lastDebounceTime = millis();
}

void on_slider_input(int event, int param) {
  // only fire change, if delta greater than 5
  // equivalent to debouncing
  if (param >= (slider_controller.currentValue + 5) || param <= (slider_controller.currentValue - 5)) {
    Serial.print("value changed: ");
    Serial.println(param);
    slider_controller.currentValue = param;
  }
}
