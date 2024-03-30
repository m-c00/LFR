
/*************************************************************************************************************************************************************************************
      initialize motor driver
*************************************************************************************************************************************************************************************/

#define INAL 6
#define INBL 7
#define INCR 8
#define INDR 9
#define ENAL 10
#define ENBR 11
/************************************************************************************************************************************************************************************/




/*************************************************************************************************************************************************************************************
        initialize PID control 
************************************************************************************************************************************************************************************/

int lastError = 0; // last  error for pid
int Error = 0; //   calculated error before entering pid
//the speed can be between 0 and 255 - 0 is LOW and 255 is HIGH. At a high value, 
//you may risk burning the motors, because the voltage supplied by the PWM control
//is higher than 6V.
const int maxspeeda = 150;
const int maxspeedb = 150;
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



/***************************************************************************************************************************************************************************
||||||||||||||||||||| function for pid controlsystem  |||||||||||||||||||||||||||||||||||||
**********************************************************************************************************************************************************************/
//Make sure that this values are assigned correctly
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
}

void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    forward_brake(mota, motb);
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


/************************************************************************************************************************************************************************************/


void setup() {
  
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


}

void loop() {

// forward_brake(80, 80);
// left_brake(150, 150);
// right_brake(150, 160);
}
