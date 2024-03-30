 int refline = 0;
  int refwall = 0;
  int refobs = 0;



#include <NewPing.h> //include NewPing library to deal with sonar sensor

#define MAX_DIS 200  //Set maximum distance for every ssonar sensor
#define TFLR    11   // set common trigger pin for every sonar sensor
#define EF      4    //set front sonar eco pin
#define EL      2    // set left sonar eco pin
#define ER      3    //set  right sonar eco pin
int LRD = 0;

NewPing sonarFront(TFLR, EF, MAX_DIS);
NewPing sonarLeft(TFLR, EL, MAX_DIS);
NewPing sonarRight(TFLR, ER, MAX_DIS);

unsigned int disFront = 0; // Initialize distance variables
unsigned int disLeft = 0;
unsigned int disRight = 0;



#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// boolean onoff = 0;
int val, cnt = 0;
float sum = 0;
// int v[3];
const uint16_t threshold = 500; // adjustable - can take values between 0 and 1000
int  position;
int oldsensorValues[7];





void WIRAnalysis(){
 uint16_t position = qtr.readLineWhite(sensorValues); //cheak var type
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


void BIRAnalysis(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  // int error = 3500 - position;

 

  int cnt = 0;
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    if(sensorValues[i] >= threshold ) {
      cnt++;
      sum = sum + i;
    }
  }

}



void toggle(){

   int toggle = 0;
  for (int i = 0; i < 8; i++) {
    if((sensorValues[i] > threshold && oldsensorValues[i] <threshold ) || (sensorValues[i] < threshold && oldsensorValues[i] > threshold )) {
      toggle++;
      
    }
     for (int i = 0; i < 8; i++){
    oldsensorValues[i] = sensorValues[i];
  }
  }
}





void IRAnalysis(){
  int black;
   int white;
  
  while(1){
    if(white == 1){
          WIRAnalysis();
          toggle();
          if(toggle > 5 ){
            black =1;
            white =0;

          }else{ break; }


      }else if(black == 1){
          BIRAnalysis();
          toggle();
          if(toggle > 5 ){
            black =1;
            white =0;

          }else{ break;}

      }else {      //use different ir analysis for choice black or white line
          
           BIRAnalysis();
      
          if(3<=cnt>=1 && cnt  || cnt ==6 ){
           black =1;
           white =0;
             }else{           
           black =1;
           white =0; }

      

      }
  }
}




void ReadSonar(){

  

  unsigned int ping1 = sonarFront.ping_cm(); // Measure distance from Sensor front
  unsigned int ping2 = sonarLeft.ping_cm(); // Measure distance from Sensor left
  unsigned int ping3 = sonarRight.ping_cm(); // Measure distance from Sensor right
  
  // Check if the distance is valid (greater than 0 and less than or equal to max distance)
  //cheak for front
  if (ping1 > 0 && ping1 <= MAX_DIS) {
    disFront = ping1; // Update the distance if it's valid
  } else {
    disFront = MAX_DIS; // Set distance to max distance if no echo or out of range
  }
  // cheak for left
  if (ping2 > 0 && ping2 <= MAX_DIS) {
     disLeft = ping2; // Update the distance if it's valid
  } else {
    disLeft = MAX_DIS; // Set distance to max distance if no echo or out of range
  }
  // cheak for right
  if (ping3 > 0 && ping3 <= MAX_DIS) {
    disRight = ping3; // Update the distance if it's valid
  } else {
    disRight = MAX_DIS; // Set distance to max distance if no echo or out of range
  }
}



void Find_reference(){
  // int refline = 0;
  // int refwall = 0;
  // int refobs = 0;
  IRAnalysis();
  ReadSonar();
  if( disFront <= 10){
    refobs = 1;
  }else{refobs = 0;}

  if( disLeft <= 40 || disRight){
    refwall = 1;
  }else{refwall = 0;}

  if( cnt >= 1){
    refobs = 1;
  }else{refobs = 0;}
}




void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
