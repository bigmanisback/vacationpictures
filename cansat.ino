#define bitrate 9600  //Bitrate for the serial interface
#define vs 5.0        //Supply voltage

/***** Sensor calibration *****
 * Sensors have not been tested; values are taken from shield test program */

// NTC Temp sensor
/* We do not know the relation between the resistance and the temperature.
 * Can be found in the sensor datasheet, whatever that is.
#define ntcRes 
#define r1 10000
  */

//Pressure sensor
#define pressureSens 0.009
#define pressureOff -0.095

// LM35DZ Temp sensor
#define tmpSens 0.01    //Sensitivity
#define tmpOff 0.0      //Offset

// Altitude calculation
#define tmpGrad -0.0065       //Temperature gradient
const float R = 287.06;             //Specific gas constant
const float g = 9.81;               //Gravitational acceleration
const float startTmp = temp();      //Calculate start temperature
const float startPrs = pressure();  //Calculate start pressure
const float startAlt = 1000.0;        //Temporary value. I don't know what this should be.

void setup() {
  Serial.begin(bitrate);
}

float bitToVolt(int n) {    //Function to convert raw ADC-data (0-255) to volt (from shield test)
  int raw = analogRead(n);
  float volt = (float)raw*5.000/1023; //should 5.000 be vs?
  return volt;
}

/* We do not know the relation between the resistance and the temperature. Can be found in the sensor datasheet.
float ntc() {
  int v = bitToVolt(0);
}
*/

float pressure() {         //Function to calculate pressure in kPa
  int v = bitToVolt(1);
  float p = (v/vs+pressureOff)/pressureSens;
  return p;
}

float temp() {             //Function to calculate temperature in K
  int v = bitToVolt(5);
  float tmp = (v-tmpOff)/tmpSens;
  return tmp;
}

float altitude() {         //Function to calculate altitude in m
  float T = temp();
  float p = pressure();
  float alt = (startTmp/tmpGrad)*(pow(p/startPrs, -tmpGrad * R / g) - 1.0) + startAlt;
  return alt;
}

void loop() {
  // put your main code here, to run repeatedly:

}
