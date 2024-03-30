
/*************************************************************************************************************************************************************************************
|||||||        initialize for use with six analog QTR sensors
// *********************************************************************************************************************************************************************************/

#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];


int lastError = 0;
// boolean onoff = 0;
int val, cnt = 0;
// int v[3];
const uint16_t threshold = 500; // adjustable - can take values between 0 and 1000

/************************************************************************************************************************************************************************************/




/*************************************************************************************************************************************************************************************
 |||||||     IR Analysis Function  for BlackLine And White line
*************************************************************************************************************************************************************************************/
void BIRAnalysis(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  // int error = 3500 - position;

  int cnt = 0;
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    if(sensorValues[i] >= threshold) {
      cnt++;
      sum = sum + i;
    }
  }
}



void WIRAnalysis(){
 uint16_t position = qtr.readLineWhite(sensorValues);
  // int error = 3500 - position;

  int cnt = 0;
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    if(sensorValues[i] <= threshold) {
      cnt++;
      sum = sum + i;
    }
  }
}
/************************************************************************************************************************************************************************************/



void setup(){
/*************************************************************************************************************************************************************************************
        Setup for use with six analog QTR sensors
************************************************************************************************************************************************************************************/

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  // qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

/***********************************************************************************************************************************************************************************/



}



void loop(){






}