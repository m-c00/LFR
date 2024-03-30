#include <NewPing.h>

#define TRIGGER_PIN_1  2
#define ECHO_PIN_1     3
#define MAX_DISTANCE_1 200

#define TRIGGER_PIN_2  4
#define ECHO_PIN_2     5
#define MAX_DISTANCE_2 200

// Define more sensors if needed

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE_1);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE_2);
// Define more NewPing objects for additional sensors


unsigned int distance1 = sonar1.ping_cm();
unsigned int distance2 = sonar2.ping_cm();
// Measure distances from more sensors if needed



void setup() {
  Serial.begin(9600);
}

void loop() {
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  
  Serial.print("Distance from Sensor 1: ");
  Serial.print(distance1);
  Serial.println(" cm");

  Serial.print("Distance from Sensor 2: ");
  Serial.print(distance2);
  Serial.println(" cm");

  delay(1000); // Delay for readability, adjust as needed
}
