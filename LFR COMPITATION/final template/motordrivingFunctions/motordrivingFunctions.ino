

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
                motor driving functions
*************************************************************************************************************************************************************************************/
 // set motors forword

void setforword(){
  digitalWrite(INAL,LOW);
  digitalWrite(INBL,HIGH);
  digitalWrite(INCR,HIGH);
  digitalWrite(INDR,LOW);
}
  
// set motors backword
void setbackword(){
  digitalWrite(INAL,HIGH);
  digitalWrite(INBL,LOW);
  digitalWrite(INCR,LOW);
  digitalWrite(INDR,HIGH);
}
//setspeed funtion take two integer value to set speed left and right
void setspeed(int left, int right){
  analogWrite(ENAL, left);
  analogWrite(ENBR, right);

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


setforword();
setspeed(100,100);


}
