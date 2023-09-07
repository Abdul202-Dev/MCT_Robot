#include "LCDdisplay.h"
#include "Motoren.h"

#include <LiquidCrystal_I2C.h>

// set the LCD number of columns and rows
volatile  int lcdColumns = 17;
volatile  int lcdRows = 2;

// set LCD address, number of columns and rows
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void SETUPLCD() {
  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
}


//print in the first line
void Printfirst(const char* MSG) {
//clear the Line
  for (int n = sizeof(MSG)+1; n < 18; n++) { // 20 indicates symbols in line. For 2x16 LCD write - 16
    lcd.setCursor(n, 0);
    lcd.print(" ");
  }
    // set cursor to first column, first row
  lcd.setCursor(0, 0);
      // print message
    lcd.print(MSG);
}
//print in the second line
void Printsecond(const char* MSG) {

  //clear the Line
  for (int n = sizeof(MSG)+1; n < 18; n++) { // 20 indicates symbols in line. For 2x16 LCD write - 16
    lcd.setCursor(n, 1);
    lcd.print(" ");
  }
    // set cursor to first column, first row
  lcd.setCursor(0, 1);
      // print message
    lcd.print(MSG);
}
