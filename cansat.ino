#define bitrate 9600  //Bitrate for the serial interface
#define vs 5.0        //Supply voltage
#define looptime 500  //Loop time in ms

/***** Sensor calibration *****
 * Temperature offset may be wrong; sensor has not been tested */

// NTC Temp sensor
/* Sensor datasheet found
#define ntcRes 
#define r1 10000
  */

//Pressure sensor
#define pressureSens 0.009
#define pressureOff -0.095

// LM35DZ Temp sensor
#define tmpSens 0.01    //Sensitivity
#define tmpOff 0.0      //Offset

//***** Altitude calculation *****
#define tmpGrad -0.0065             //Temperature gradient
const float R = 287.06;             //Specific gas constant
const float g = 9.81;               //Gravitational acceleration
#define startAlt = 1000.0           //Temporary value. I don't know what this should be.

unsigned long int counter = 0;      //Used to check how many times the program has run

void setup() {
  Serial.begin(bitrate);
  Serial.println("Counter,Time / ms,Pressure,Temperature (LM35),Temperature (NTC),Acceleration X-axis,Acceleration Y-axis,Acceleration Z-axis,Pressure / kPa,Temperature (LM35) / °C,Altitude / m");
  //Insert between / °C and ,Alt when available: Temperature (NTC) / °C,Acceleration X-axis / m s⁻²,Acceleration Y-axis / m s⁻²,Acceleration Z-axis / m s⁻²,
}

float bitToVolt(int n) {    //Function to convert raw ADC-data (0-255) to volt (from shield test)
  int raw = analogRead(n);
  float volt = (float)raw * 5.000 / 1023; //should 5.000 be vs?
  return volt;
}

/*
float ntc() {
  int v = bitToVolt(0);
}
*/

float pressure() {         //Function to calculate pressure in kPa
  float v = bitToVolt(1);
  float p = (v / vs + pressureOff) / pressureSens;
  return p;
}

float temp() {             //Function to calculate temperature in deg C
  float v = bitToVolt(5);
  float tmp = (v - tmpOff) / tmpSens;
  return tmp;
}

float altitude() {                     //Function to calculate altitude in m
  static float startTmp = temp();      //Measure start temperature
  static float startPrs = pressure();  //Measure start pressure
  float T = temp();
  float p = pressure();
  float alt = (startTmp / tmpGrad) * (pow(p / startPrs, -tmpGrad * R / g) - 1.0) + startAlt;
  return alt;
}

void printData() {
  Serial.print(counter);
  Serial.print(",");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(analogRead(1));
  Serial.print(analogRead(5));
  Serial.print(analogRead(0));
  Serial.print(analogRead(2));
  Serial.print(analogRead(3));
  Serial.print(analogRead(4));
  Serial.print(pressure());
  Serial.print(",");
  Serial.print(temp());
  Serial.print(",");
  //Serial.print(ntc());
  //Serial.print(",");
  //Serial.print(accx());
  //Serial.print(",");
  //Serial.print(accy();
  //Serial.print(",");
  //Serial.print(accz());
  //Serial.print(",");
  Serial.print(altitude());
  Serial.println();
}

void loop() {
  unsigned long int loop_start = millis(), loop_end;
  printData();
  counter++;
  loop_end = millis();
  if (looptime>(loop_end-loop_start)){      //Sets the delay to aquire right loop time. Taken from shield test.
    delay(looptime-(millis()-loop_start));
  }
}
