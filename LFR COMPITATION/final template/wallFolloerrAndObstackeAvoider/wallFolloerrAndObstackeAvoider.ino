// Define the pin for the start button***************************************************************
const int startButtonPin = 12;
int position = 3500;


// *****************************************************************************************
// variable for find reference function*************************************************************
  int refline = 0;
  int refwall = 0;
  int refobs = 0;

// library function for ir arrray***********************************************************
#include <QTRSensors.h>

QTRSensors qtr;  // Use QTRSensorsAnalog for analog sensors

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t calibratedMin[SensorCount];
uint16_t calibratedMax[SensorCount];

int val, cnt, sum = 0;
uint16_t threshold = 500; // This will store the calculated threshold

// Add a global array for storing old sensor values
uint16_t oldsensorValues[SensorCount];
//*****************************************************************************************************

//variable and library function for read sonar sensor***********************************************
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

//**************************************************************************************************
//initialize motor driver **************************************************************************
#define INAL 6
#define INBL 7
#define INCR 8
#define INDR 9
#define ENAL 10
#define ENBR 11
//**************************************************************************************************
//initialize PID control ***************************************************************************
int lastError = 0; // last  error for pid
int Error = 0; //   calculated error before entering pid
//the speed can be between 0 and 255 - 0 is LOW and 255 is HIGH. At a high value, 
//you may risk burning the motors, because the voltage supplied by the PWM control
//is higher than 6V.
 int maxspeeda = 100;
 int maxspeedb =100;
 int basespeeda = 65;
 int basespeedb = 65;
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


float LKp = .8;  //KP KI KD VALUE FOR WALL OR OBSTACKLE
float LKi = 0.0007;
float LKd = 0.03;

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

//*******************************************************************************************************


void setup() {
    Serial.begin(9600);
  
//setup motor driver pinmode*********************************************************************
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

//***********************************************************************
//setup soner sensor pins************************************************
  pinMode(TFLR, OUTPUT);
  pinMode(EF, INPUT);
  pinMode(EL, INPUT);
  pinMode(ER, INPUT);
  Serial.begin(9600);
//**********************************************************************
// Setup code for ir array  *************************************************************************
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // Calibrate the sensors
  Serial.print("calibration start");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // Copy the calibration values to calibratedMin and calibratedMax
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    calibratedMin[i] = qtr.calibrationOn.minimum[i];
    calibratedMax[i] = qtr.calibrationOn.maximum[i];
  }

  // Calculate the threshold as the average middle value between calibrated min and max
  threshold = 0;
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold += (calibratedMax[i] + calibratedMin[i]) / 2;
  }
  threshold /= SensorCount;
 Serial.print("calibration end");
  // print the calibration minimum values measured when emitters were on
  // Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(calibratedMin[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(calibratedMax[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println(threshold);
  delay(1000);

// *************************************************************************************************
// wait for start button ****************************************************************
// Set the start button pin as an input
   Serial.print("starting loop");
  pinMode(startButtonPin, INPUT);

  Serial.println("Waiting for the start button...");
  Serial.print("starting loop");

  waitForStartButton();

  Serial.println("Start button pressed. Starting the robot...");
  Serial.print("starting loop end");
}

// // Define the pin for the start button
// const int startButtonPin = 12;

// void setup() {
//     // Serial.begin(9600);

//     // Set the start button pin as an input
//     pinMode(startButtonPin, INPUT);

//     // Wait for the start button to be pressed
//     // waitForStartButton();
    
//     // Start your robot's main functionality here
//     Serial.println("Start button pressed. Starting the robot...");
// }


void waitForStartButton() {
    Serial.println("Waiting for the start button...");
    while (true) {
        if (digitalRead(startButtonPin) == HIGH) {
           break;
            }
            return; // Start button pressed, exit the function
        }
    }




void loop() {

  

// forward_brake(60, 50);
// delay(2000);
// left_brake(150, 150);
// delay(2000);
// right_brake(140, 160);
// delay(2000);
//  ReadSonar();
//   Serial.print("disFront: ");
//   Serial.println(disFront);
//   Serial.print("disLeft: ");
//   Serial.println(disLeft);
//   Serial.print("disRight: ");
//   Serial.println(disRight);
//   delay(1000);
//   Serial.println();
//   Serial.println();



// if(refobs == 1 ){  bridge_obstackle_control();}
//   // if qnt ==0 && go wall control wall not found
// // if(refline == 1 ){ line_robot_control();}

// else if(refwall == 1 ){  wall_robot_control();}

// // else{ } //qnt =0 stay certain time find reference then turn around

// Serial.print("main  loop");

// //  IRAnalysis();
BIRAnalysis();

//  for (uint8_t i = 0; i < SensorCount; i++)
//   {
//     Serial.print(sensorValues[i]);
//     Serial.print('\t');
//   }
//   Serial.println(position);

  // delay(250);

 Error = position - 3500;
//   // Error =0;
//   Serial.println("Error");
//  Serial.println(Error);
     PID(Error, LKp, LKi, LKd );

    //  line_robot_control();


}


// function for pid controlsystem ************************************************************************

void forward_brake(int posa, int posb) {
  digitalWrite(INAL,LOW);
  digitalWrite(INBL,HIGH);
  digitalWrite(INCR,LOW);
  digitalWrite(INDR,HIGH);
  analogWrite(ENAL, posa);
  analogWrite(ENBR, posb);
}
void left_brake(int posa, int posb) {
  digitalWrite(INAL,HIGH); //left motor backword
  digitalWrite(INBL,LOW);
  digitalWrite(INCR,LOW); //right motor forword
  digitalWrite(INDR,HIGH);
  analogWrite(ENAL, posa);
  analogWrite(ENBR, posb);
}
void right_brake(int posa, int posb) {
  // left motor forword
  digitalWrite(INAL,LOW);
  digitalWrite(INBL,HIGH);
   //right motor backword
  digitalWrite(INCR,HIGH);
  digitalWrite(INDR,LOW);
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
  
  int motorspeeda = basespeeda - motorspeed; // left motor speed   --error negative mean i have to go right increase left speed
  int motorspeedb = basespeedb + motorspeed; // right motor speed
 
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
  Serial.println(motorspeeda);
    Serial.println( motorspeedb);
}

void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    forward_brake(mota, motb-2);
  }
  if (mota < 0 && motb >= 0) {
    //dreapta
    mota = 0 - mota;
    // right_brake(mota, motb);
     left_brake(mota, motb);
  }
  if (mota >= 0 && motb < 0) {
    //stanga
    motb = 0 - motb;
    // left_brake(mota, motb);
    right_brake(mota, motb);
  }
}

//*********************************************************************************
//function to get distance from three soner***************************************

void ReadSonar() {
  disFront = sonarFront.ping_cm();
  disLeft = sonarLeft.ping_cm();
  disRight = sonarRight.ping_cm();
}
//******************************************************************************
//Ir Sensor reading function*******************************************


void WIRAnalysis(){
  position = qtr.readLineWhite(sensorValues); //cheak var type
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
 position = qtr.readLineBlack(sensorValues);
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
  for (int i = 0; i < 8; i++) 
  {
    if((sensorValues[i] > threshold && oldsensorValues[i] <threshold ) || (sensorValues[i] < threshold && oldsensorValues[i] > threshold ))
     {
      toggle++;
     }
     for (int i = 0; i < 8; i++)
     {
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



//********************************************************************88
// this function will find possible reference  ***********************


void Find_reference(){
  // int refline = 0;
  // int refwall = 0;
  // int refobs = 0;
  // IRAnalysis();
  ReadSonar();
  if( disFront <= 10 && disFront > 0){  // measure sutable distance for obstackle
    refobs = 1;
  }else{refobs = 0;}

  if( disLeft <= 40 && disLeft > 0 || disRight <= 40 && disRight > 0){
    refwall = 1;
  }else{refwall = 0;}

  if( cnt >= 1){
    refline = 1;
  }else{refline = 0;}
}


//********************************************************************************
//bridge and obstackle control function*******************************************

void bridge_obstackle_control(){//incomplete

forward_brake(0, 0);

  // ReadSonar();
  // if(disFront <= 10){
  //    maxspeeda = 80; //adjust based on base speed and set type int
  //    maxspeedb = 80;

  //  }else{ 
  //   maxspeeda = 100;
  //   maxspeedb = 100;
  //   }
  //   if(disFront <= 5){ //find suitable value for obstackle
       
  //       if(disLeft >=30 ){ 
  //         lastError = 0;
  //         while (1){
  //          while ( disLeft > 5){ //find sutable value based on front distance
  //         left_brake(50,65);

  //         }
  //         if( disLeft >=30){}
          
  //         Error = disLeft -5;//
  //         PID(Error, WKp, WKi, WKd );
          
  //       }

  //     }
  //   }

  // // if(disFront <= 8 && )

}
//*********************************************************************************************
// this function is able to follow any wall**********************************************

void wall_robot_control(){
  ReadSonar();

  if( disLeft <=40  && disLeft > 0 && disRight<=40 && disRight > 0 ){ // max distance of wall assume 40cm

    Error = disLeft - disRight;
    PID(Error, WKp, WKi, WKd );


  } else if( disLeft <=40 && disLeft > 0 ){   

    int LRD = disLeft;
    int motorspeeda = 0;
    int motorspeedb = 0;

    lastError = 0;
    while (1) {
      
      // IRAnalysis();
      ReadSonar();

      // if(cnt>=1){
      //   break;  // Exit the loop
      // }
      // else 
      if( disFront <=5 && disFront > 0 ){ // if find obstackle
        //turn right
        // right_brake(100, 100);
        break;

      }
      // else if( disLeft >=40) { // sonar lost obstackle
      // //turn left
      //   left_brake(100, 100);
      //   //forward_brake(100,0);
      // }
      else {
         Error = disLeft-LRD;
        //  Error =0;
        PID(Error, WKp, WKi, WKd );
      }
   }
  }
}
//**********************************************************************************


// function for array based control

void line_robot_control() {
  BIRAnalysis();

  
  //I made a case where the robot can cross the line (when the curve is too tight) and the position can be 3500
  //even though it is not on the center of the line. If you don't want your motors to rotate in 2 directions
  //comment the right_brake(100,100) / left_brake(100, 100) and uncomment the forward_brake(0,100) / forward_brake(0,100)
  if(cnt=6){} //stay to mid point of stop point then stop when stop code match set as motor speed
   
  if ( 6>cnt >= 3) {  //3 or more sensor see the line
   
     val = sum/cnt;
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
     Error = position - 3500;
    //  Error = 3500- position;
     PID(Error, LKp, LKi, LKd );
  }
}

// **********************************************************************************************************




