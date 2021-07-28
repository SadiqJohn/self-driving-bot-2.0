#define CUSTOM_SETTINGS
#define INCLUDE_LEDCONTROL_MODULE
#define INCLUDE_SENSOR_MODULE
//#define INCLUDE_TERMINAL_MODULE
#include <Dabble.h>

// Imports
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_GPS.h>

#include <Wire.h>
#include <Servo.h>

#include <SoftwareSerial.h>
#include <TinyGPS.h>

#include "BotDefinitions.h"

#define DEST_RADIUS 1 // In meters

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TinyGPS gps;
SoftwareSerial gps_ss(GPS_TX_PIN, 255);

long phoneLat;
long phoneLon;

long botLat;
long botLon;
float botHeading;

float newHeading;

float getHeadingInRadians(){
    sensors_event_t event;
    mag.getEvent(&event);

    float heading = atan2(event.magnetic.y, event.magnetic.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.19;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
    
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;

    return heading;
}

float getHeadingInDegrees(){
    return getHeadingInRadians() * 180 / PI;
}

float calculateAngleInRadians(float x1, float y1, float x2, float y2){
    return atan2(y2 - y1, x2 - x1);
}

float calculateAngleInDegrees(float x1, float y1, float x2, float y2){
    return calculateAngleInRadians(x1, y1, x2, y2) * 180 / PI;
}

void updateGPS(){
    while(gps_ss.available() > 0){
        gps.encode(gps_ss.read());
        gps.get_position(&botLat, &botLon);
    }
}

// Bot Control
void turnRight(){
    digitalWrite(MOTOR_A_IN_1_PIN, LOW);
    digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
    
    digitalWrite(MOTOR_B_IN_1_PIN, LOW);
    digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
}

void turnLeft(){
    digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN_2_PIN, LOW);
    
    digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void moveForward(){
    digitalWrite(MOTOR_A_IN_1_PIN, LOW);
    digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
    
    digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void moveBackward(){
    digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN_2_PIN, LOW);
    
    digitalWrite(MOTOR_B_IN_1_PIN, LOW);
    digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
}

void stopMoving(){
    digitalWrite(MOTOR_A_IN_1_PIN, LOW);
    digitalWrite(MOTOR_A_IN_2_PIN, LOW);
    
    digitalWrite(MOTOR_B_IN_1_PIN, LOW);
    digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void setup(){
    // Motor pins
    pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
    pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
    pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
    pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  
    // Compass
    mag.begin();

    //GPS
    Serial.begin(9600);
    gps_ss.begin(9600);  

    //Bluetooth
    Dabble.begin(9600);
}

void loop(){
    Dabble.processInput();

    while(LedControl.getpinState()){
        // GPS of phone
        phoneLat = Sensor.getGPSlatitude()  * 100000;
        phoneLon = Sensor.getGPSlongitude() * 100000;

        Serial.print(phoneLat);
        Serial.print(phoneLon);


        // GPS of bot
        updateGPS();
        
        // Calculate direction
        newHeading = calculateAngleInRadians(botLon, botLat, phoneLon, phoneLat);
        
        // Face direction
        botHeading = getHeadingInRadians();

        if(newHeading - botHeading > PI){
            // Turn right
            turnRight();
            do{
                delay(1);
                getHeadingInRadians();
            }while(botHeading != newHeading);
            stopMoving();
        }
        else{
            // Turn left
            turnLeft();
            do{
                delay(1);
                getHeadingInRadians();
            }while(botHeading != newHeading);
            stopMoving();
        }

        // Move forward (by speed set in app)
        moveForward();
        int moveTick = 0;
        do{
            delay(1);
            updateGPS();
            ++moveTick;
        }while(abs(abs(phoneLat) - abs(botLat)) - DEST_RADIUS <= 0 && abs(abs(phoneLon) - abs(botLon)) - DEST_RADIUS  <= 0 || moveTick < 500);
        stopMoving();
    }

    delay(1);
}
