
 int refline = 0;
  int refwall = 0;
  int refobs = 0;

/*************************************************************************************************************************************************************************************
        initialize PID control 
************************************************************************************************************************************************************************************/

int lastError = 0; // last  error for pid
int Error = 0; //   calculated error before entering pid
//the speed can be between 0 and 255 - 0 is LOW and 255 is HIGH. At a high value, 
//you may risk burning the motors, because the voltage supplied by the PWM control
//is higher than 6V.
int maxspeeda = 150;
int maxspeedb = 150;
const int basespeeda = 100;
const int basespeedb = 100;
/*If your robot can't take tight curves, you can set up the robot
  to revolve around the base (and not around one of the wheels) by
  setting the minspeed to a negative value (-100), so the motors will go 
  forward and backward. Doing this the motors can wear out faster. 
  If you don't want to do this, set the minspeed to 0, so the motors 
  will only go forward.
*/
//const int minspeeda = 0;
//const int minspeedb = 0;
const int minspeeda = -100;
const int minspeedb = -100;


float WKp = 0;  //KP KI KD VALUE FOR WALL OR OBSTACKLE
float WKi = 0;
float WKd = 0;


float LKp = 0;  //KP KI KD VALUE FOR WALL OR OBSTACKLE
float LKi = 0;
float LKd = 0;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
int P;
int I;
int D;
float Pvalue;
float Ivalue;
float Dvalue;

/************************************************************************************************************************************************************************************/



/*************************************************************************************************************************************************************************************
      initialize motor driver
*************************************************************************************************************************************************************************************/

#define INAL 9
#define INBL 8
#define INCR 7
#define INDR 6
#define ENAL 10
#define ENBR 5
/************************************************************************************************************************************************************************************/



/*************************************************************************************************************************************************************************************
||||||||||||||||||||||||||||||||||||||||||||||      initialize for use with six analog QTR sensors       ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// *********************************************************************************************************************************************************************************/

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

/************************************************************************************************************************************************************************************/




/*******************************************************************************************************************************************************************
  ||||||||||||||||||||||||||||||||||||||||||||||||||||||||     Initialize Three Sonar Sensor       |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
***************************************************************************************************************************************************************************/

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
/******************************************************************************************************************************************************************************/





/*************************************************************************************************************************************************************************************
 |||||||     IR Analysis Function  for BlackLine And White line
*************************************************************************************************************************************************************************************/

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




/************************************************************************************************************************************************************************************/





/************************************************************************************************************************************************************************************
||||||||          ReadSonar  This function will take data of distance from ||front, left and right|| and assign them in ||disFront, disLeft, disRight||
**************************************************************************************************************************************************************************************/
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



/*******************************************************************************************************************************************************************************/





/***************************************************************************************************************************************************************************
||||||||||||||||||||| function for pid controlsystem  |||||||||||||||||||||||||||||||||||||
**********************************************************************************************************************************************************************/
//Make sure that this values are assigned correctly
void forward_brake(int posa, int posb) {
  digitalWrite(INAL,LOW);
  digitalWrite(INBL,HIGH);
  digitalWrite(INCR,HIGH);
  digitalWrite(INDR,LOW);
  analogWrite(ENAL, posa);
  analogWrite(ENBR, posb);
}
void left_brake(int posa, int posb) {
  digitalWrite(INAL,HIGH);
  digitalWrite(INBL,LOW);
  digitalWrite(INCR,HIGH);
  digitalWrite(INDR,LOW);
  analogWrite(ENAL, posa);
  analogWrite(ENBR, posb);
}
void right_brake(int posa, int posb) {
  digitalWrite(INAL,LOW);
  digitalWrite(INBL,HIGH);
  digitalWrite(INCR,LOW);
  digitalWrite(INDR,HIGH);
  analogWrite(ENAL, posa);
  analogWrite(ENBR, posb);
}


void PID(int error, float Kp, float Ki, float Kd) {
  int P = error;
  int I = I + error;
  int D = error - lastError;
  lastError = error;
  Pvalue = (Kp/pow(10,multiP))*P;
  Ivalue = (Ki/pow(10,multiI))*I;
  Dvalue = (Kd/pow(10,multiD))*D;

  float motorspeed = Pvalue + Ivalue + Dvalue;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < minspeeda) {
    motorspeeda = minspeeda;
  }
  if (motorspeedb < minspeedb) {
    motorspeedb = minspeedb;
  }
  //Serial.print(motorspeeda); Serial.print(" "); Serial.println(motorspeedb);
  speedcontrol(motorspeeda, motorspeedb);
}

void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    forward_brake(mota, motb);
  }
  if (mota < 0 && motb >= 0) {
    //dreapta
    mota = 0 - mota;
    right_brake(mota, motb);
     
  }
  if (mota >= 0 && motb < 0) {
    //stanga
    motb = 0 - motb;
    left_brake(mota, motb);
  }
}


/************************************************************************************************************************************************************************************/


void setup(){


 
/************************************************************************************************************************************************************************************
                  setup motor driver pinmode
*************************************************************************************************************************************************************************************/

  // Set motor connection as output
  pinMode(INAL,OUTPUT);
  pinMode(INBL,OUTPUT);
  pinMode(INCR,OUTPUT);
  pinMode(INDR,OUTPUT);
  pinMode(ENAL,OUTPUT);
  pinMode(ENBR,OUTPUT);
    
  // start with both motor off
  analogWrite(ENAL,0);
  analogWrite(ENBR,0);
  digitalWrite(INAL,LOW);
  digitalWrite(INBL,LOW);
  digitalWrite(INCR,LOW);
  digitalWrite(INDR,LOW);

/************************************************************************************************************************************************************************************/




/**********************************************************************************************************************************************************************
||||||||||||||||||||||||||| Pinmode setup for three sonar sensor  ||||||||||||||||||||||||||||||||||||||||||||||||
****************************************************************************************************************************************************************/

/**************************************************************************************************************************************************************/


/*************************************************************************************************************************************************************************************
        Setup for use with six analog QTR sensors ALSO CALIBRATION SETUP
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
Find_reference();

}



void loop(){

if(refobs == 1 ){  bridge_obstackle_control();}
  // if qnt ==0 && go wall control wall not found
if(refline == 1 ){ line_robot_control();}

else if(refwall == 1 ){  wall_robot_control();}

else{ } //qnt =0 stay certain time find reference then turn around






}




/**********************************************************************************************************************************************************************************
||||||||||||||||||||||||    Control Function |||||||||||||||||||||||||||||||||
*********************************************************************************************************************************************************************************/


void line_robot_control() {
  IRAnalysis();

  
  //I made a case where the robot can cross the line (when the curve is too tight) and the position can be 3500
  //even though it is not on the center of the line. If you don't want your motors to rotate in 2 directions
  //comment the right_brake(100,100) / left_brake(100, 100) and uncomment the forward_brake(0,100) / forward_brake(0,100)
  if(cnt=6){} //stay to mid point of stop point then stop when stop code match set as motor speed
   
  if ( 6>cnt >= 3) {  //3 or more sensor see the line
    int motorspeeda = 0;
    int motorspeedb = 0;
    int val = sum/cnt;
    if(val < 3.5) {   // line see in left side 0 to 3 or 4 dependino on there connection
      //turn right
      right_brake(100, 100);
      //forward_brake(0,100);
    }
    if(val > 3.5) {  // line see in left side but it also the condition when all sensor can see the line
      //turn left
      left_brake(100, 100);
      //forward_brake(100,0);
    }
    if(val == 3.5) { //symmetric sensor can see the line
      cnt = cnt/2;
      uint16_t mini = 1000;
      uint8_t minpos = 0;
      for (int i = 4 - cnt; i <= 3 + cnt; i++) {//identi fing which symmetric sensor could see line
         if (mini > sensorValues[i]) {
            mini = sensorValues[i];// identifying which sensor can see lowest reflectance among them
            minpos = i;
         }
      }
      if(minpos < 3.5) { //3,2,1 may see low reftance as they stey left so we have to turn right
        //turn right
        right_brake(100, 100);
        //forward_brake(0,100);
      }
      if(minpos > 3.5) {
        //turn left
        left_brake(100, 100);
        //forward_brake(100,0);
      }
    }
  }
  else {
     Error = 3500- position;
     PID(Error, LKp, LKi, LKd );
  }
}




void wall_robot_control(){
  ReadSonar();

  if( disLeft <=40 &&  disRight<=40 ){ // max distance of wall assume 40cm

    Error = disLeft - disRight;
    PID(Error, WKp, WKi, WKd );


  } else if( disLeft <=40 ){   

     LRD = disLeft;
    lastError = 0;
    int motorspeeda = 0;
      int motorspeedb = 0;
    while (1) {
      
     
      
      IRAnalysis();
      ReadSonar();

      if(cnt>=1){
        break;  // Exit the loop
      }else if( disFront<=4 && disLeft< 5){ // mere may a logical error cheak it later
        //turn right
        right_brake(100, 100);

      }else if( disLeft >=40) { // sonar lost obstackle
      //turn left
        left_brake(100, 100);
        //forward_brake(100,0);
      }else {
         Error = disLeft-LRD;
        PID(Error, WKp, WKi, WKd );
      }
   }
  }
}

// disFront, disLeft, disRight

void bridge_obstackle_control(){//incomplete
  ReadSonar();
  if(disFront <= 10){
     maxspeeda = 80; //adjust based on base speed and set type int
     maxspeedb = 80;

   }else{ 
    maxspeeda = 150;
    maxspeedb = 150;
    }
    if(disFront <= 5){ //find suitable value for obstackle
       
        if(disLeft >=30 ){ 
          lastError = 0;
          while (1){
           while ( disLeft > 5){ //find sutable value based on front distance
          left_brake(100,100);

          }
          if( disLeft >=30){}
          
          Error = disLeft -5;//
          PID(Error, WKp, WKi, WKd );
          
        }

      }
    }

  // if(disFront <= 8 && )

}








/***************************************************************************************************************************************************************************/












