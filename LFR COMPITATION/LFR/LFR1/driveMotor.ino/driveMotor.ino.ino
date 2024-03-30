#define INA 10
#define INB 11
#define INC 12
#define IND 13
#define ENA 3
#define ENB 5

// accelarate both motor
  void motorAccelarate()
  {
    for (int i =100; i<256; i++)
    {

      analogWrite(ENA, i);
      analogWrite(ENB, i);
      delay(5);
    }
  }
// decelarate both motor
  
  void motorDecelarate()
  {
    for (int i =200; i>100; --i)
    {

      analogWrite(ENA, i);
      analogWrite(ENB, i);
      delay(5);
    }
  }


void setup() {
  // Set motor connection as output
  pinMode(INA,OUTPUT);
  pinMode(INB,OUTPUT);
  pinMode(INC,OUTPUT);
  pinMode(IND,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
    



  // start with both motor off
  analogWrite(INA,0);
  analogWrite(INB,0);
  digitalWrite(INA,LOW);
  digitalWrite(INB,LOW);
  digitalWrite(INC,LOW);
  digitalWrite(IND,LOW);




}

void loop() {
  // set motors forword
  
  digitalWrite(INA,LOW);
  digitalWrite(INB,HIGH);
  digitalWrite(INC,HIGH);
  digitalWrite(IND,LOW);
  // accelarate & decelarate both motor
  motorAccelarate();
  motorDecelarate();
  

   // set motors backword
  
  digitalWrite(INA,HIGH);
  digitalWrite(INB,LOW);
  digitalWrite(INC,LOW);
  digitalWrite(IND,HIGH);
  // accelarate & decelarate both motor
  motorAccelarate();
  motorDecelarate();








}
