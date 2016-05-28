#include "SoftwareSerial.h"
#include "TinyGPS++.h"

SoftwareSerial ss(5, 6);
TinyGPSPlus gps;
TinyGPSCustom steerDirection(gps, "GPRMC", 2);

#define bitrate 9600  //Bitrate for the serial interface
#define vs 5.0        //Supply voltage
#define loopTime 500  //Duration of one loop

//***** Sensor calibration *****

// NTC Temp sensor
#define r1 10000.0
const float a = 3.354016E-3;
const float b = 2.569850E-4;
const float c = 2.620131E-6;
const float d = 6.383091E-8;

// Pressure sensor
#define pressureSens 0.009
#define pressureOff 0.095
float prs;

// LM35DZ Temp sensor
#define tmpSens 0.01      //Sensitivity
#define tmpOff 0.0733138  //Offset

// Accelerometer
#define accSensMode true  //Accelerometer sensitivity mode. false: 1.5 g, true: 6 g
#define acc15 0.8         //Sensitivity at 1.5 g (from datasheet)
#define acc6 0.206        //Sensitivity at 6 g (from datasheet)
#define accXOff 1.61      //0 g outputs. Slightly wrong, since
#define accYOff 1.68      //the sensor is very hard to calibrate
#define accZOff 1.71      //due to the inconsistency of these values.

//***** Altitude calculation *****
#define tmpGrad -0.0065   //Temperature gradient
const float R = 287.06;   //Specific gas constant
const float g = 9.81;     //Gravitational acceleration

#define alertAltitude 100.0

#define ledPin 3              //Pin of LED
#define ledPin2 10            //Pin of LED 2
#define ledDelayOn1 250       //Blink delay of LED 1 in ms
#define ledDelayOff1 1750     //Blink delay of LED 1 in ms
#define ledDelayOn2 250       //Blink delay of LED 2 in ms
#define ledDelayOff2Real 1750 //Blink delay of LED 2 in ms
bool ledToggle1 = false;      //If true, LED 1 will toggle when led1() is called.
bool ledToggle2 = false;      //If true, LED 2 will toggle when led2() is called.
bool led1On = true;           //
bool led2On = false;          //

#define buzzerPin 2      //Pin of buzzer
#define buzzerDelayOn 250
#define buzzerDelayOff 1750
bool buzzerToggle = false;
bool buzzerOn = true;

bool useLed1 = true;
bool useLed2 = true;
bool useBuzzer = true;
bool forceAlert = false;
bool forceSilence = false;

#define switchPinHelp 7
#define switchPinOk 8

unsigned long counter = 0;//Used to check how many times the program has run
float alt = 0.0;

void setup()
{
  Serial.begin(bitrate);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  //analogWrite(buzzerPin, 255);
  pinMode(switchPinHelp, INPUT_PULLUP);
  pinMode(switchPinOk, INPUT_PULLUP);
  Serial.print("Counter,Time / ms,Pressure,Temperature (LM35),Temperature (NTC),Acceleration X-axis,Acceleration Y-axis,");                                                                 //Heading row
  Serial.println("Acceleration Z-axis,Pressure / kPa,Temperature (LM35) / K,Temperature (NTC) / K,Acceleration X-axis / g,Acceleration Y-axis / g,Acceleration Z-axis / g,Altitude / m");  //for the output
}

float bitToVolt(int n)  //Function to convert raw ADC-data (0-1023) to volt (from shield test)
{
  int raw = analogRead(n);
  float volt = (float)raw * vs / 1023;
  return volt;
}

float ntc()             //Function to calculate temperature in K
{
  float v = bitToVolt(0);
  float r = vs * r1 / v - r1;
  float t = 1.0 / (a + b * log(r / r1) + c * pow(log(r / r1), 2.0) + d * pow(log(r / r1), 3.0));
  return t;
}

float pressure()        //Function to calculate pressure in Pa
{
  float v = bitToVolt(1);
  float p = ((v / vs + pressureOff) / pressureSens) * 1000.0;
  return p;
}

float temp()            //Function to calculate temperature in K
{
  float v = bitToVolt(5);
  float tmp = (v - tmpOff) / tmpSens + 273.5;
  return tmp;
}

float accCalc(int axis) //Function to calculate acceleration in g
{
  float off, sens;
  switch (axis)
  {
    case 2:             //x-axis
      off = accXOff;
      break;
    case 3:             //y-axis
      off = accYOff;
      break;
    case 4:             //z-axis
      off = accZOff;
      break;
  }
  if (accSensMode)
  {
    sens = acc6;
  }
  else
  {
    sens = acc15;
  }
  float acc = (bitToVolt(axis) - off) / sens;
  return acc;
}

float altitude()        //Function to calculate altitude in m
{
  static float startTmp = temp();      //Measure start temperature
  static float startPrs = pressure();  //Measure start pressure
  float alt = (startTmp / tmpGrad) * (pow(prs / startPrs, -tmpGrad * R / g) - 1.0);
  return alt;
}

int switchRead()
{
  if (digitalRead(switchPinHelp) == LOW)
  {
    return 0;
  }
  else if (digitalRead(switchPinOk) == LOW)
  {
    return 1;
  }
  else
  {
    return 2;
  }
}

void printData()
{
  Serial.print(switchRead());
  Serial.print(",");
  Serial.print(counter);
  Serial.print(",");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(analogRead(1));
  Serial.print(",");
  Serial.print(analogRead(5));
  Serial.print(",");
  Serial.print(analogRead(0));
  Serial.print(",");
  Serial.print(analogRead(2));
  Serial.print(",");
  Serial.print(analogRead(3));
  Serial.print(",");
  Serial.print(analogRead(4));
  Serial.print(",");
  prs = pressure();
  Serial.print(prs);
  Serial.print(",");
  Serial.print(temp());
  Serial.print(",");
  Serial.print(ntc());
  Serial.print(",");
  Serial.print(accCalc(2));
  Serial.print(",");
  Serial.print(accCalc(3));
  Serial.print(",");
  Serial.print(accCalc(4));
  Serial.print(",");
  alt = altitude();
  Serial.print(alt);
  Serial.print(",");
  while (ss.available() > 0)
    gps.encode(ss.read());
  Serial.print(gps.location.rawLat().billionths);
  Serial.print(",");
  Serial.print(gps.location.rawLng().billionths);
  Serial.print(",");
  Serial.println(gps.altitude.value());
}

void led1()
{
  if (led1On)
  {
    digitalWrite(ledPin, LOW);
    led1On = false;
  }
  else
  {
    if (!forceSilence)
    {
      if (useLed1)
      {
        if (alt < alertAltitude)
          digitalWrite(ledPin, HIGH);
        else if (forceAlert)
          digitalWrite(ledPin, HIGH);
      }
    }
    led1On = true;
  }
  ledToggle1 = false;
}

void led2()
{
  if (led2On)
  {
    digitalWrite(ledPin2, LOW);
    led2On = false;
  }
  else
  {
    if (!forceSilence)
    {
      if (useLed2)
      {
        if (alt < alertAltitude)
          digitalWrite(ledPin2, HIGH);
        else if (forceAlert)
          digitalWrite(ledPin2, HIGH);
      }
    }
    led2On = true;
  }
  ledToggle2 = false;
}

void buzzer()
{
  if (buzzerOn)
  {
    digitalWrite(buzzerPin, LOW);
    buzzerOn = false;
  }
  else
  {
    if (!forceSilence)
    {
      if (useBuzzer)
      {
        if (alt < alertAltitude)
          digitalWrite(buzzerPin, HIGH);
        else if (forceAlert)
          digitalWrite(buzzerPin, HIGH);
      }
    }
    buzzerOn = true;
  }
  buzzerToggle = false;
}

void manual()
{
  while (Serial.available())
  {
    char rxChar = (char)Serial.read();
    switch (rxChar)
    {
      case '1':
        if (useLed1)
          useLed1 = false;
        else
          useLed1 = true;
        break;
      case '2':
        if (useLed2)
          useLed2 = false;
        else
          useLed2 = true;
        break;
      case 'b':
        if (useBuzzer)
          useBuzzer = false;
        else
          useBuzzer = true;
        break;
      case 't':
        if (forceAlert)
          forceAlert = false;
        else
          forceAlert = true;
        break;
      case 's':
        if (forceSilence)
          forceSilence = false;
        else
          forceSilence = true;
        break;
    }
  }
}

void loop()
{
  static unsigned long loopStart = 0;                                       //Determines whether data should be printed.
  static unsigned long ledStart1 = 0;                                       //Determines whether LED 1 state should change when altitude < 100.
  static unsigned long ledStart2 = 0;                                       //Determines whether LED 2 state should change when altitude < 100.
  static unsigned long buzzerStart = 0;
  static unsigned long loopEnd = 0;                                         //Initialized as an arbitrary high value to make everything run the first time.
  static unsigned long ledDelayOff2 = (ledDelayOn2 + ledDelayOff2Real) / 2; //This sets the first off delay so that the LEDs are in opposite phase.
  
  if (led1On)                               //If LED 1 is on…
  {
    if (loopEnd - ledStart1 > ledDelayOn1)  //…and enough time has passed since it last toggled…
    {
      ledToggle1 = true;                    //…set ledToggle1 to true, which will make LED 1 toggle the next time led1() is called.
      ledStart1 = millis();                 //Assign a new starting point to compare the loop end time with. This lets us know how long it's been since the LED toggle.
    }                                       //The following conditionals do the same thing, but check if LED 1 is off and if LED 2 is on or off. also buzzer
  }
  else
  {
    if (loopEnd - ledStart1 > ledDelayOff1)
    {
      ledToggle1 = true;
      ledStart1 = millis();
    }
  }
  
  if (led2On)
  {
    if (loopEnd - ledStart2 > ledDelayOn2)
    {
      ledToggle2 = true;
      ledStart2 = millis();
    }
  }
  else
  {
    if (loopEnd - ledStart2 > ledDelayOff2)
    {
      ledToggle2 = true;
      ledStart2 = millis();
      ledDelayOff2 = ledDelayOff2Real;
    }
  }

  if (buzzerOn)
  {
    if (loopEnd - buzzerStart > buzzerDelayOn)
    {
      buzzerToggle = true;
      buzzerStart = millis();
    }
  }
  else
  {
    if (loopEnd - buzzerStart > buzzerDelayOff)
    {
      buzzerToggle = true;
      buzzerStart = millis();
    }
  }

  if (loopEnd - loopStart > loopTime)
  {
    printData();
    counter++;
    loopStart = millis();
  }
  
  manual();

  if (ledToggle1)
    led1();
  if (ledToggle2)
    led2();
  if (buzzerToggle)
    buzzer();
  
  loopEnd = millis();
}
