#define trigPinC 5 // Arduino pin connected to the trigger pin of the sensor
#define echoPinL 2 // Arduino pin connected to the echo pin of the sensor
#define echoPinR 3 // Arduino pin connected to the echo pin of the sensor
#define echoPinF 4 // Arduino pin connected to the echo pin of the sensor

float averageDistance;

void setup() {
  Serial.begin(9600);
  pinMode(trigPinC, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(echoPinR, INPUT);
  pinMode(echoPinF, INPUT);
}

void loop() {
  delay(1000);
  
  // Calculate and print the average distances for each sensor
  Serial.print("Average Distance: Left ");
  Serial.print(Average(trigPinC, echoPinL));
  Serial.println(" cm");
  delay(1000);
  Serial.print("Average Distance: Right ");
  Serial.print(Average(trigPinC, echoPinR));
  Serial.println(" cm");
  delay(1000);
  Serial.print("Average Distance: Front ");
  Serial.print(Average(trigPinC, echoPinF));
  Serial.println(" cm");
}

float Average(int trigPin, int echoPin) {
  int numReadings = 5; // Number of readings to take
  long totalDistance = 0; // Total distance for averaging

  for (int i = 0; i < numReadings; i++) {
    // Trigger a pulse to start the measurement
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the duration of the echo pulse
    long duration = pulseIn(echoPin, HIGH);
    
    // Calculate the distance based on the speed of sound (343 m/s or 0.0343 cm/us)
    // and the time it took for the echo to return
    float distance_cm = duration * 0.0343 / 2.0; // Divide by 2 for one-way distance
    
    totalDistance += distance_cm;
  }
  
  // Calculate the average distance
  averageDistance = totalDistance / numReadings;
  
  return averageDistance;
}
