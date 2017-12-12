#include <SimbleeForMobile.h>

#include <PID_v1.h>

#include <SPI.h>

#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(19, 17, 18, 16); 
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0!
#define RREF 2700.0

double t;

double setpoint, input, output;

double kp = 5, ki = 1, kd = 0; //PID values

PID myPID(&t, &output, &setpoint, kp, ki, kd, DIRECT); //PID intialization

int offTime; //Amount of time the cal rod should be off for the current cycle

int onTime; //Amount of time the cal rod should be on for the current cycle

int minRelayTime = 2; //Minimum time before a relay is switched. Prevents relay chattering

int cycleTime = 20; //Time of a heating cycle. Each cycle has an on time and off time, a new one is calculated after each. 

int cadBuffer = 1200; //Amount of time the smoke eliminator is allowed to heat up before anything else turns on

double updateRate = .15;

unsigned long onStart;

unsigned long offStart;

unsigned long lastCalc;

unsigned long lastHeat;

unsigned long lastUpdated;

unsigned long switchOnTime;

int calcTime = 1;

int state; // State 0 - Off | State 1 - Preheat | State 3 - PID off cycle | State 4 - PID on cycle

int on = 1;

int off = 0;

bool lastSwitch = 0;

double temp;

int set;

int minus;

int plus;

int editText;

int switchOn;

bool relay1State;

bool relay2State;

bool relay3State;

int rel1; //Heating element

int rel2; //Smoke Eliminator

int rel3; //Smoke Generator

int swit;



#define MAX_HEIGHT SimbleeForMobile.screenHeight //Sets GUI Dimensions
#define MAX_WIDTH SimbleeForMobile.screenWidth

uint8_t ui_button;
uint8_t ui_text;
uint8_t ui_textField;
uint8_t ui_switch;
uint8_t ui_segment;
uint8_t ui_slider;
uint8_t ui_stepper;
uint8_t ui_rectangle;



// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");


void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(10, OUTPUT); 
  digitalWrite(10, LOW); //Initializes relay pins and sets them low
  pinMode(28, INPUT_PULLUP);

  //Serial.begin(115200);

  SimbleeForMobile.begin();

  max.begin(MAX31865_2WIRE);  //RTD sensor board initilization
  setpoint = 220;

  
  myPID.SetOutputLimits(0, 100); //tell the PID to range between 0 and the full window size


  myPID.SetMode(AUTOMATIC); //turn the PID on

  t = (max.temperature(1000, RREF)) * 1.8 + 32; //Reads RTD temp and converts to F
  myPID.Compute(); //Gives temperature value to the PID
  cycleCalc(); //Function that calculates the on/ off cycle length
//  Serial.println("First"); //For debugging
//  Serial.println(t);
//  Serial.println(output);
//  Serial.println(onTime);
//  Serial.println(offTime);
}


void loop() {

  SimbleeForMobile.process();

  if (SimbleeForMobile.updatable && (millis() > (lastUpdated + updateRate * 1000))) { //Everything in this "if" updates the GUI with accurate information
    lastUpdated = millis();


    SimbleeForMobile.updateValue(temp, t);
    SimbleeForMobile.updateValue(set, setpoint);

    if (relay1State == 1) {
      SimbleeForMobile.updateText(rel1, "on");
    }
    else {
      SimbleeForMobile.updateText(rel1, "off");
    }

    if (relay2State == 1) {
      SimbleeForMobile.updateText(rel2, "on");
    }
    else {
      SimbleeForMobile.updateText(rel2, "off");
    }

    if (relay3State == 1) {
      SimbleeForMobile.updateText(rel3, "on");
    }
    else {
      SimbleeForMobile.updateText(rel3, "off");
    }

    if (switchOn == 1) {
      SimbleeForMobile.updateText(swit, "on");
    }
    else {
      SimbleeForMobile.updateText(swit, "off");
    }
  }
  if (millis() > (lastCalc + calcTime * 1000)) {


    t =  (max.temperature(1000, RREF)) * 1.8 + 32;

    myPID.Compute();
//    Serial.println(t);
//    Serial.println(output);
//    Serial.println(onTime);
//    Serial.println(offTime);
//    Serial.println(lastHeat);
    lastCalc = millis();
  }




  if (digitalRead(28) == 0) { //If the on/off switch is put in the on position 
    lastHeat = millis();
    relay2(on);

    if (switchOn == 0) { //If this is the first time through this loop
      switchOnTime = millis();
      state = 1; //Changes state to preheating
      switchOn = 1;
    }

    if (state == 1) {  
      relay1(on);
      relay2(on);
      relay3(off);
      if (t > setpoint - 10) {//While preheating, CAL rod stays on until the temperature is within 10 degrees of the target
        state = 3; //State is then changed to PID control
      }
    }

    if (state == 3 || state == 4) { //Once in PID control smoke generator can turn on
      relay3(on);
    }


    if (state == 3) { //If in a PID off cycle 
      if (millis() > (offStart + offTime)) { //Checks if it is time to switch to a PID on cycle
        cycleCalc(); //calculates the next cycle
        if (onTime > 0) { //if there is any time on in the next cycle, turns the heating element on
          relay1(on);
        }
        state = 4; //switches to on state
        onStart = millis();
      }
    }

    else if (state == 4) { //If in a PID on cycle 
      if (millis() > (onStart + onTime)) { //Checks if it is time to switch to a PID on cycle
        if (offTime > 0) { //if there is any time off in the next cycle, turns the heating element off
          relay1(off);
        }
        state = 3; //switches to off state
        offStart = millis();

      }
    }
  }

  else {
    switchOn = 0;
    relay1(off);
    relay3(off);
    if (millis() > (lastHeat + cadBuffer * 1000)) {
      relay2(off);
    }
    state = 0;
  }

}


void relay1(int x) { //function to turn heating element on/off
  digitalWrite(14, x); 
  relay1State = x;
}

void relay2(int x) { //function to turn smoke elimnator on/off
  digitalWrite(13, x);
  relay2State = x;
}

void relay3(int x) { //function to turn smoke generator on/off 
  digitalWrite(10, x);
  relay3State = x;
}

float cycleCalc() { //calculates next cycle
  onTime = (cycleTime * 1000) * (output / 100);
  offTime = (cycleTime * 1000) - onTime;
  if (onTime < (minRelayTime * 1000)) {
    onTime = 0;
    offTime = (cycleTime * 1000);
  }

  if (offTime < (minRelayTime * 1000)) {
    offTime = 0;
    onTime = (cycleTime * 1000);
  }
}

void ui()
{
  color_t grayBackgroundColor = rgb(240, 240, 240);
  SimbleeForMobile.beginScreen(grayBackgroundColor);

  ui_text = SimbleeForMobile.drawText(30, 90, "Current Temperature:", BLACK, 22);
  temp = SimbleeForMobile.drawText(250, 90, t, BLACK, 22);

  ui_text = SimbleeForMobile.drawText(50, 120, "Current Setpoint:", BLACK, 22);
  set = SimbleeForMobile.drawText(230, 120, setpoint, BLACK, 22);
  minus = SimbleeForMobile.drawButton(110, 160, 40, "-", BLACK, 26);
  plus = SimbleeForMobile.drawButton(160, 160, 40, "+", BLACK, 26);

  ui_text = SimbleeForMobile.drawText(75, 350, "Smoker is", BLACK, 26);
  swit = SimbleeForMobile.drawText(205, 350, " ", BLACK, 26);

  ui_text = SimbleeForMobile.drawText(65, 400, "Heating Element is");
  rel1 = SimbleeForMobile.drawText(215, 400, " ");

  ui_text = SimbleeForMobile.drawText(60, 420, "Smoke Eliminator is");
  rel2 = SimbleeForMobile.drawText(220, 420, " ");

  ui_text = SimbleeForMobile.drawText(60, 440, "Smoke Generator is");
  rel3 = SimbleeForMobile.drawText(220, 440, " ");


  SimbleeForMobile.endScreen();
}


void ui_event(event_t &event)
{
  if (event.id == minus && event.type == EVENT_PRESS) { //changes setpoint if minus button is pressed
    setpoint = setpoint - 1;
  }

  if (event.id == plus && event.type == EVENT_PRESS) { //changes setpoint if minus button is pressed
    setpoint = setpoint + 1;
  }

}
