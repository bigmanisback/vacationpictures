#define bitrate 9600  //Bitrate for the serial interface
#define vs 5.0        //Supply voltage

/***** Sensor calibration *****
 * Temperature offset may be wrong; sensor has not been tested */

// NTC Temp sensor
#define r1 10000.0
const float a = 3.354016E-3;
const float b = 2.569850E-4;
const float c = 2.620131E-6;
const float d = 6.383091E-8;

//Pressure sensor
#define pressureSens 0.009
#define pressureOff -0.095

// LM35DZ Temp sensor
#define tmpSens 0.01      //Sensitivity
#define tmpOff 0.0        //Offset

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

#define ledPin 9          //Pin of LED
#define ledPin2 10        //Pin of LED 2
#define toggleDelay 250   //toggle delay in ms
int loopStart = 0;        //Used to determine whether LED and speaker
int loopEnd = 0;          //states should change when altitude < 100 to
bool toggle = false;      //make the LED blink and the speaker play a tone.
bool state = false;       //If true, LED and speaker are on.

#define speakerPin 8      //Pin of speaker

unsigned long counter = 0;  //Used to check how many times the program has run
float alt = 0.0;

void setup()
{
  Serial.begin(bitrate);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  Serial.print("Counter,Time / ms,Pressure,Temperature (LM35),Temperature (NTC),Acceleration X-axis,Acceleration Y-axis,");                                                                 //Heading row
  Serial.println("Acceleration Z-axis,Pressure / kPa,Temperature (LM35) / °C,Temperature (NTC) / K,Acceleration X-axis / g,Acceleration Y-axis / g,Acceleration Z-axis / g,Altitude / m");  //for the output
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

float temp()            //Function to calculate temperature in deg C
{
  float v = bitToVolt(5);
  float tmp = (v - tmpOff) / tmpSens;
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
  float p = pressure();
  float alt = (startTmp / tmpGrad) * (pow(p / startPrs, -tmpGrad * R / g) - 1.0);
  return alt;
}

void printData()
{
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
  Serial.print(pressure());
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
  Serial.println(alt);
}

void alert() //
{
  if (alt < 100)
  {
    if (toggle)
    {
      if (!state)
      {
        digitalWrite(ledPin, HIGH);
        digitalWrite(ledPin2, HIGH);
        tone(speakerPin, 523, toggleDelay);
        state = true;
      }
      else
      {
        digitalWrite(ledPin, LOW);
        digitalWrite(ledPin2, LOW);
        state = false;
      }
      toggle = false;
    }
  }
  else
  {
    digitalWrite(ledPin, LOW);
    noTone(speakerPin);
  }
}

void loop()
{
  if (loopEnd - loopStart >= toggleDelay)
  {
    toggle = true;
    loopStart = millis();
  }
  printData();
  alert();
  counter++;
  loopEnd = millis();
}
