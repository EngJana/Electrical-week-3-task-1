#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR 0x27
#define LCD_COLUMNS 20
#define LCD_LINES 4

const int analogPin = 34; // GPIO pin for analog input
int ldrValue = 0; // Variable to store the analog value

LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES); // Create an instance of the LiquidCrystal_I2C class

void setup() {
  Serial.begin(115200); // Initialize serial communication
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the LCD backlight
}

void loop() {
  ldrValue = analogRead(analogPin); // Read the analog value from the LDR
  float voltage = ldrValue * (3.3 / 4095.0); // Convert the analog value to voltage

  Serial.print("LDR Voltage: "); // Print the voltage to the serial monitor
  Serial.print(voltage);
  Serial.println(" V");

  lcd.setCursor(0, 0); // Set the cursor to the first column, first row
  lcd.print("LDR Voltage: "); // Print "LDR Voltage:" on the LCD
  lcd.setCursor(0, 1); // Set the cursor to the first column, second row
  lcd.print(voltage); // Print the voltage value on the LCD
  lcd.print(" V"); // Print " V" after the voltage value

  delay(500); // Wait for 500 milliseconds before repeating the loop
}