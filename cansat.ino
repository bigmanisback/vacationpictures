#define bitrate 9600  //Bitrate for the serial interface
#define vs 5          //Supply voltage

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
#define tmpSens 0.01  //Sensitivity
#define tmpOff 0      //Offset

// Altitude calculation
#define tmpGrad -0.0065
float R = 287.06;
float g = 9.81;
float startTmp = temp();
float startPrs = pressure();
//float startAltitude

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

float temp() {
  int v = bitToVolt(5);
  float tmp = (v-tmpOff)/tmpSens;
  return tmp;
}

float altitude() {
  float T = temp();
  float p = pressure();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
