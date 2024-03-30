/*************************************************************************************************************************************************************
                      // initialize LCD Display
***********************************************************************************************************************************************************/
#include <Wire.h>     // For standard Wire library functions
#include <SoftwareWire.h> // Include the SoftwareWire library
#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library

                     // Create SoftwareWire object with specified digital pins (SDA, SCL)
#define SDA_PIN 10
#define SCL_PIN 11   // Replace SDA_PIN and SCL_PIN with your desired pin numbers
SoftwareWire myWire(SDA_PIN, SCL_PIN); 

                     // Create an instance of the LiquidCrystal_I2C library
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the I2C LCD address to 0x27, 16 columns, and 2 rows
/********************************************************************************************************************************************************/

void setup() {

/**************************************************************************************************************************************************************
                    // initialize LCD Display in set up
**********************************************************************************************************************************************************/
  myWire.begin(); // Initialize the SoftwareWire library
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
/********************************************************************************************************************************************************/


/*************************************************************************************************************************************************************
        printing command in lcd display
************************************************************************************************************************************************************************/  
  
  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set the cursor to the first row, first column
  lcd.print("Hello, World!"); // Print "Hello, World!" on the LCD
  delay(2000); // Wait for 2 seconds

  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set the cursor to the first row, first column
  lcd.print("LFR"); // Print "LFR" on the LCD
  delay(2000); // Wait for 2 seconds

/**************************************************************************************************************************************************************************/

}

void loop() {



  // Your code here...
}
