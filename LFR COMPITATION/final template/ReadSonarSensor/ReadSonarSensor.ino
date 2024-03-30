#include <NewPing.h>

#define MAX_DIS 200
#define EL 2
#define EF 3
#define ER 4
#define TFLR 5

NewPing sonarFront(TFLR, EF, MAX_DIS);
NewPing sonarLeft(TFLR, EL, MAX_DIS);
NewPing sonarRight(TFLR, ER, MAX_DIS);

unsigned int disFront = 0;
unsigned int disLeft = 0;
unsigned int disRight = 0;

void setup() {
  pinMode(TFLR, OUTPUT);
  pinMode(EF, INPUT);
  pinMode(EL, INPUT);
  pinMode(ER, INPUT);
  Serial.begin(9600);
}

void ReadSonar() {
  disFront = sonarFront.ping_cm();
  disLeft = sonarLeft.ping_cm();
  disRight = sonarRight.ping_cm();
}

void loop() {
  ReadSonar();
  Serial.print("disFront: ");
  Serial.println(disFront);
  Serial.print("disLeft: ");
  Serial.println(disLeft);
  Serial.print("disRight: ");
  Serial.println(disRight);
  delay(1000);
  Serial.println();
  Serial.println();
}
