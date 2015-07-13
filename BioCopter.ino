/*
NHC Carriage Controller
by M. Emre Caliskan
2014

Control RC Helicopter.
Uses accelerometer and sonar sensor as inputs to determine yaw, pitch and
throttle of RC Helicopter.

Uses Infrared LED to control RC Helicopter.
*/


  #include <TimerOne.h>
 #include <IRCommunication.h>

 //comment this out to see the demodulated waveform
//it is useful for debugging purpose.
#define MODULATED 1

//Digital PIN  for reading potentiometer to adjust throttle
#define trigPin 12
#define echoPin 13

//Analog read pins
const int throttlePin = 3;
const int xPin = 0;
const int yPin = 1;

//Lock throttle button
const int throttleButton = 2;

const int xCal = 333;
const int yCal = 333;
//const int tCal = 500;
volatile int tCal=0;

const double xMax = 95;
const double yMax = 95;
const double tMax = 50;

int xRead = 0;
int yRead = 0;
int throttleRead = 0;


volatile double x= 0;
volatile double y = 0;
volatile double t = 0;
volatile double oldt = 0;

volatile int duration, distance;

int Throttle = 0;
int LeftRight = 0;
int FwdBack = 0;

long debouncing_time = 10; //Debouncing Time in Milliseconds
volatile unsigned long last_micros, last_micros2;

volatile boolean risingEdgeISR=false;
volatile boolean updateThrottle=false;
volatile boolean calibrateThrottle=false;

void getSonarDistance()
//Sends a pulse with the sonar and records the time taken to receive the pulse.
//Updates the duration variable in the function.
{
     digitalWrite(trigPin, HIGH);
     delayMicroseconds(60);
     digitalWrite(trigPin, LOW);
     duration = pulseIn(echoPin, HIGH);
}

void timerISR()
{
   //read control values from potentiometers
   xRead = analogRead(xPin);
   yRead = analogRead(yPin);

   //Conditions to Keep x-Values withing bounds recognized by helicopter
    if(xRead - xCal > xMax)
    x = 1;
      else if(xRead - xCal < -1*xMax)
    x = -1;
      else
    x = ((double)((xRead - xCal)%(int)xMax))/xMax;

    //Conditions to Keep y-Values withing bounds recognized by helicopter
    if(yRead - yCal > yMax)
       y = 1;
    else if(yRead - yCal < -1*yMax)
       y = -1;
    else
      y = ((double)((yRead - yCal)%(int)yMax))/yMax;

    //Convert x-sensor percentage values into the Full Scale Output recognized by the helicopter
    x = (int)(x*120);
    y = (int)(y*60);


   if(updateThrottle)//Only Change throttle if the throttle toggle button is pressed
   {
      distance = getDistance();
      if(calibrateThrottle)
      {
        tCal=distance;
        calibrateThrottle=false;
      }

      t = ((double)(distance - tCal))/tMax;
      t = (int)(t*250);

      Throttle=Throttle+(t-oldt);
      oldt=t;

      if(Throttle>250)
        Throttle=250;
      if(Throttle<0)
        Throttle=0;
   }

  //Calibrate according to the orientation of the accelerometer
  LeftRight = -1*y  ;//convert to -64 to 63
  FwdBack = x*-1  ;//convert to -128 to 127

  //Set of conditions to save power when throttle is low and prevent
  //activating yaw & pitch motors on helicopter
  if(Throttle < 30)
  {
    LeftRight = 0;
    FwdBack = 0;
  }

  if(Throttle <30)
    Throttle = 0;

  sendCommand(Throttle, LeftRight, FwdBack);  //Send Data Packet to Helicopter

}

int getDistance()
//Sends a pulse with the sonar and records the time taken to receive the pulse.
//Converts the time taken (duration) into a distance and returns this value
{
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(60);
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 return (duration/2) / 29.1;
}

void throttleISR ()
//Interrupt Service Routine Called on every change in edge from the limit switch input
//On rising edge; calculated distance and sets a flag to re-calibrate the throttle (to
//create a relative measurement point at the height the limit switch is pressed). A flag
//is also set to allow the throttle to be updated in the timer interrupt.
//On Falling edge; old throttle value is reset to 0, and throttle update flag is set to false
//to prevent throttle from being changed while button is not pressed.
{
       if(!risingEdgeISR)
       {
         getSonarDistance();
         calibrateThrottle=true;
         updateThrottle=true;
       }
       else
       {
         oldt = 0;
         updateThrottle=false;
       }
       risingEdgeISR=!risingEdgeISR;    //Internal Variable to keep track of rising/falling edge
}


// Setup pins on Arduino
void setup()
{
  pinMode(trigPin, OUTPUT);             //Configure triggerPin as an Output
  pinMode(echoPin, INPUT);              //Configure echoPin as an Input

  pinMode(11, OUTPUT);                  //Configure Pin 11 as Output (to be used as VCC for Sonar IC)
  digitalWrite(11, HIGH);               //Write HIGH (5V) to pin 11

  pinMode(throttleButton,INPUT);       //Configure Pin 2 as Throttle Activation Button Input
  attachInterrupt(0, throttleISR, CHANGE);    //Attach an interrupt to Pin2

  //setup interrupt interval: 180ms
  Timer1.initialize(DURATION);
  Timer1.attachInterrupt(timerISR);

  initializeIR();
}

void loop()
{
}
