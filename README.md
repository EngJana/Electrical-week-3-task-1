# Electrical-week-3-task-1

Week 3 task 1: design, code, and explain ESP32 circuits one with a digital sensor and the other with an analog sensors 

# ESP32 Sensor Interfacing Project
This repository contains code for interfacing an ESP32 microcontroller with a DHT22 temperature and humidity sensor, an I2C LCD display, and an example of reading analog values from a Light-Dependent Resistor (LDR) using the ESP32.

# Table of Contents
-	Components
-	How It Works
    - ESP32 with DHT22 and I2C LCD
      -	ESP32
      -	DHT22 Sensor
      - I2C LCD Display
    - ESP32 Analog Read Example with LDR
      -	ESP32 Microcontroller
      -	Light-Dependent Resistor (LDR)
      -	Serial Communication
-	Circuit Diagrams
-	Code Explanation
    - ESP32 with DHT22 and I2C LCD
      - Libraries and Definitions
      -	Pin Definition and Object Initialization
      -	Setup Function
      -	Loop Function	
    -	ESP32 Analog Read Example with LDR
      -	Libraries and Definitions
      -	Pin Definition and Object Initialization
      -	Setup Function
      -	Loop Function

## Components
-	ESP32 Development Board
-	DHT22 Temperature and Humidity Sensor
-	I2C LCD Display (20x4)
-	Light-Dependent Resistor (LDR)
-	10k Ohm Resistor
-	Jumper Wires

## How It Works
### ESP32 with DHT22 and I2C LCD
This project demonstrates how to read digital values from an DHT22 using an ESP32 and displays those readings on the I2C LCD. 
#### ESP32
The ESP32 is a powerful microcontroller with integrated Wi-Fi and Bluetooth capabilities. It has numerous GPIO pins for digital and analog input/output operations. In this project, the ESP32:
Reads Data from DHT22: Uses the DHTesp library to read temperature and humidity data.
Displays Data on LCD: Uses the LiquidCrystal_I2C library to output data to an I2C LCD display.
Serial Communication: Sends data to the serial monitor for debugging and monitoring.
#### DHT22 Sensor
The DHT22 is a low-cost digital sensor for measuring temperature and humidity. It:
Measures Environmental Data: Provides accurate temperature and humidity readings.
Digital Output: Sends data to the ESP32 via a single data line.
#### I2C LCD Display
The I2C LCD display shows the temperature and humidity readings. It:
Initialization: Uses lcd.init() and lcd.backlight() to initialize the LCD and turn on the backlight.
Displaying Data: Uses lcd.setCursor() and lcd.print() to set the cursor position and print data on the LCD.

### ESP32 Analog Read Example with LDR
This project demonstrates how to read analog values from an LDR using an ESP32 and convert those readings to voltage. The readings are printed to the Serial Monitor.
#### ESP32 
The ESP32 uses its ADC to read analog signals from sensors. In this project, GPIO34 reads the analog value from the LDR.
analogRead(pin): Reads the analog value from the specified pin.
Serial.begin(baud_rate): Initializes serial communication.
Serial.print() / Serial.println(): Outputs data to the Serial Monitor.
#### Light-Dependent Resistor (LDR)
An LDR's resistance decreases as light intensity increases. It forms a voltage divider with a fixed resistor, producing a varying voltage.
Voltage Divider: Produces a varying voltage based on light intensity.
Analog Reading: The ESP32 reads this voltage at its analog input.
Conversion to Voltage: Converts the raw ADC value to voltage using:
Voltage = analog value × (3.3 / 4095.0)
#### Serial Communication
The ESP32 sends voltage readings to the Serial Monitor for real-time observation.
Baud Rate: Set to 115200 baud.
Output: Voltage values are printed every 500 milliseconds.

## Circuit Diagrams
### digital sensor circuit (DHT22)
<img width="440" alt="digitalon" src="https://github.com/user-attachments/assets/83b92cdd-d4d8-42c4-af49-37d5c1c81391">

### analog sensor circuit (LDR)
<img width="328" alt="analogon" src="https://github.com/user-attachments/assets/1f91db64-da53-4e80-910c-5df92daf7c8a">


## Code Explanation
### ESP32 with DHT22 and I2C LCD
#### Libraries and Definitions
-	#include "DHTesp.h": Includes the DHTesp library for the DHT22 sensor.
-	#include <LiquidCrystal_I2C.h>: Includes the LiquidCrystal_I2C library for the I2C LCD display.
-	#define I2C_ADDR 0x27: Defines the I2C address of the LCD.
-	#define LCD_COLUMNS 20: Defines the number of columns on the LCD display.
-	#define LCD_LINES 4: Defines the number of lines on the LCD display.
#### Pin Definition and Object Initialization
const int DHT_PIN = 15: Defines the GPIO pin connected to the DHT22 sensor's data pin.
-	DHTesp dhtSensor: Creates an instance of the DHTesp class.
-	LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES): Creates an instance of the LiquidCrystal_I2C class.
-	Setup Function
-	Serial.begin(115200): Initializes serial communication at a baud rate of 115200.
-	dhtSensor.setup(DHT_PIN, DHTesp::DHT22): Initializes the DHT22 sensor.
-	lcd.init(): Initializes the LCD display.
-	lcd.backlight(): Turns on the LCD backlight.
#### Loop Function
-	TempAndHumidity data = dhtSensor.getTempAndHumidity(): Reads data from the DHT22 sensor.
-	Serial.println("Temp: " + String(data.temperature, 1) + "°C"): Prints the temperature to the serial monitor.
-	Serial.println("Humidity: " + String(data.humidity, 1) + "%"): Prints the humidity to the serial monitor.
-	Serial.println("---"): Prints a separator line to the serial monitor.
-	lcd.setCursor(0, 0): Sets the cursor position on the LCD.
-	lcd.print("  Temp: " + String(data.temperature, 1) + "\xDF"+"C  "): Displays the temperature on the LCD.
-	lcd.setCursor(0, 1): Sets the cursor position on the LCD.
-	lcd.print(" Humidity: " + String(data.humidity, 1) + "% "): Displays the humidity on the LCD.
-	lcd.setCursor(0, 2): Sets the cursor position on the LCD.
-	lcd.print("Wokwi Online IoT"): Displays "Wokwi Online IoT" on the LCD.
-	delay(1000): Waits for 1 second before repeating the loop.
### ESP32 Analog Read Example with LDR
#### Libraries and Definitions
-	#include <Arduino.h>: Includes the Arduino core library for basic functionality.
-	const int analogPin = 34;: Defines GPIO pin 34 for the analog input.
-	int ldrValue = 0;: Declares a variable to store the analog value read from the LDR.
#### Pin Definition and Object Initialization
-	const int analogPin = 34: Defines the GPIO pin connected to the LDR.
#### Setup Function
-	Serial.begin(115200): Initializes serial communication at a baud rate of 115200 for debugging and monitoring the analog values.
#### Loop Function
-	ldrValue = analogRead(analogPin): Reads the analog value from GPIO pin 34 and stores it in the ldrValue variable.
-	float voltage = ldrValue * (3.3 / 4095.0): Converts the analog value to a voltage level by using the formula voltage = analog value * (3.3 / 4095.0).
-	Serial.print("LDR Voltage: "): Prints the string "LDR Voltage: " to the serial monitor.
-	Serial.print(voltage): Prints the calculated voltage value to the serial monitor.
-	Serial.println(" V"): Prints the string " V" followed by a newline character to the serial monitor.
-	delay(500): Pauses the program for 500 milliseconds before repeating the loop.
