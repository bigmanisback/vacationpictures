#define bitrate 9600  //Bitrate for the serial interface
#define vs 5.0        //Supply voltage
#define looptime 500  //Loop time in ms

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
#define tmpGrad -0.0065             //Temperature gradient
const float R = 287.06;             //Specific gas constant
const float g = 9.81;               //Gravitational acceleration
#define startAlt 1000.0             //Temporary value. I don't know what this should be.

unsigned long int counter = 0;      //Used to check how many times the program has run

void setup() {
  Serial.begin(bitrate);
  Serial.print("Counter,Time / ms,Pressure,Temperature (LM35),Temperature (NTC),Acceleration X-axis,Acceleration Y-axis,"); //Heading row
  Serial.println("Acceleration Z-axis,Pressure / kPa,Temperature (LM35) / °C,Temperature (NTC) / K,Altitude / m");         //for the output
  //Insert between °C, and Alt when available: Acceleration X-axis / m s⁻²,Acceleration Y-axis / m s⁻²,Acceleration Z-axis / m s⁻²,
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

float pressure()        //Function to calculate pressure in kPa
{
  float v = bitToVolt(1);
  float p = (v / vs + pressureOff) / pressureSens;
  return p;
}

float temp()            //Function to calculate temperature in deg C
{
  float v = bitToVolt(5);
  float tmp = (v - tmpOff) / tmpSens;
  return tmp;
}

float accCalc(int axis) //Function to calculate acceleration in m s⁻²
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
  float alt = (startTmp / tmpGrad) * (pow(p / startPrs, -tmpGrad * R / g) - 1.0) + startAlt;
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
  Serial.println(altitude());
}

void loop()
{
  unsigned long int loop_start = millis(), loop_end;
  printData();
  counter++;
  loop_end = millis();
  if (looptime>(loop_end-loop_start)){      //Sets the delay to aquire right loop time. Taken from shield test.
    delay(looptime-(millis()-loop_start));
  }
}
