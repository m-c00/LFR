#include <QTRSensors.h>

#define NUM_SENSORS 6
#define SENSOR_PIN_0 A0
#define SENSOR_PIN_1 A1
#define SENSOR_PIN_2 A2
#define SENSOR_PIN_3 A3
#define SENSOR_PIN_4 A4
#define SENSOR_PIN_5 A5

QTRSensorsRC qtrrc((unsigned char[]){SENSOR_PIN_0, SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3, SENSOR_PIN_4, SENSOR_PIN_5},
                   NUM_SENSORS);

unsigned int sensorValues[NUM_SENSORS];
int linePosition; // -2500 to 2500 represents line position

void setup()
{
  Serial.begin(9600);
  qtrrc.init();
}

void loop()
{
  qtrrc.read(sensorValues);
  
  // Calculate line position using sensor values
  int weightedSum = 0;
  int sum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    weightedSum += sensorValues[i] * (i * 1000);
    sum += sensorValues[i];
  }
  
  if (sum != 0)
    linePosition = ((weightedSum * 5000) / (sum * (NUM_SENSORS - 1))) - 2500;
  else
    linePosition = 0;
  
  // Print sensor values and line position
  Serial.print("Sensor values: ");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print("Line position: ");
  Serial.println(linePosition);
  
  delay(100); // Delay between readings
}
