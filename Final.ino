#include <AFMotor.h> //Import library to control motor shield
#include <Servo.h>   //Import library to control the servo
#define Sensor A2
#define RELAY_PIN 7

int LED = A5;

AF_DCMotor rightBack(2); // Create an object to control each motor
AF_DCMotor rightFront(1);
AF_DCMotor leftFront(4);
AF_DCMotor leftBack(3);
Servo servoLook; // Create an object to control the servo
String readvoice;
byte trig = A0; // Assign the ultrasonic sensor pins
byte echo = A1;
byte Sound_sen = A5; // Assign Sound Sensor pins
byte prev = 0;
byte maxDist = 80;                                        // Maximum sensing distance (Objects further than this distance are ignored)
byte stopDist = 50;                                       // Minimum distance from an object to stop in cm
float timeOut = 2 * (maxDist + 10) / 100 / 340 * 1000000; // Maximum time to wait for a return signal

byte motorSpeed = 190; // The maximum motor speed
int motorOffset = 0;   // Factor to account for one side being more powerful
int turnSpeed = 50;    // Amount to add to motor speed when turning
int count = 0;
char value;

int clap = 0;
long detection_range_start = 0;
long detection_range = 0;
boolean status_lights = false;

void setup()
{
    Serial.begin(9600);
    rightBack.setSpeed(motorSpeed); // Set the motors to the motor speed
    rightFront.setSpeed(motorSpeed);
    leftFront.setSpeed(motorSpeed + motorOffset);
    leftBack.setSpeed(motorSpeed + motorOffset);
    rightBack.run(RELEASE); // Ensure all motors are stopped
    rightFront.run(RELEASE);
    leftFront.run(RELEASE);
    leftBack.run(RELEASE);
    servoLook.attach(10);  // Assign the servo pin
    pinMode(trig, OUTPUT); // Assign ultrasonic sensor pin modes
    pinMode(echo, INPUT);
    count = 1;
}

void loop()
{
    delay(1000);
    while (Serial.available() > 0)
    {
        value = Serial.read();
        readvoice += value;
        Serial.println(readvoice);
        value = readvoice[0];
    }

    if (value == 'F' || value == 'f')
    {
        moveForward();
    }
    else if (value == 'B' || value == 'b')
    {
        moveBackward();
    }
    else if (value == 'L' || value == 'l')
    {
        turnRight(4000);
        moveForward();
    }
    else if (value == 'R' || value == 'r')
    {
        turnLeft(4000);
        moveForward();
    }
    else if (value == 'S' || value == 's')
    {
        stopMove();
    }
    else if (value == 'C' || value == 'c')
    {
        clapcontrol();
    }
    else if (value == 'O' || value == 'o')
    {
        obstacle();
    }
    readvoice = "";
    value = "x";
    delay(200);
    int distance = getDistance();
    if (distance <= stopDist)
    {
        stopMove();
        Serial.println("Stopped due to Obstacle");
        delay(200);
    }
}
void clapcontrol()
{
    while (count == 1)
    {
        if (Serial.available() > 0)
        {
            return;
        }
        int status_sensor = digitalRead(Sensor);
        if (status_sensor == 0)
        {
            if (clap == 0)
            {
                detection_range_start = detection_range = millis();
                clap++;
            }
            else if (clap > 0 && millis() - detection_range >= 50)
            {
                detection_range = millis();
                clap++;
            }
        }
        if (millis() - detection_range_start >= 400)
        {
            if (clap == 1)
            {
                moveForward();
                delay(2000);
                stopMove();
                delay(100);
            }
            if (clap == 2)
            {
                turnRight(4000);
                delay(100);
            }
            if (clap == 3)
            {
                turnLeft(4000);
                delay(100);
            }
            clap = 0;
        }
    }
}
void obstacle()
{
    servoLook.write(100); // Set the servo to look straight ahead
    delay(750);
    int distance = getDistance(); // Check that there are no objects ahead
    if (distance >= stopDist)     // If there are no objects within the stopping distance, move forward
    {
        moveForward();
    }
    while (distance >= stopDist) // Keep checking the object distance until it is within the minimum stopping distance
    {
        if (Serial.available() > 0)
        {
            return;
        }
        distance = getDistance();
        delay(250);
    }
    stopMove(); // Stop the motors
    if (Serial.available() > 0)
    {
        return;
    }
    int turnDir = checkDirection(); // Check the left and right object distances and get the turning instruction
    Serial.print(turnDir);
    switch (turnDir) // Turn left, turn around or turn right depending on the instruction
    {
    case 0: // Turn left
        stopMove();
        delay(200);
        turnLeft(5000);
        stopMove();
        break;
    case 1: // Turn around
        stopMove();
        delay(200);
        turnRight(8000);
        break;
    case 2: // Turn right
        stopMove();
        delay(200);
        turnRight(4000);
        break;
    }
}
void moveForward() // Set all motors to run forward
{
    rightBack.run(FORWARD);
    rightFront.run(FORWARD);
    leftFront.run(FORWARD);
    leftBack.run(FORWARD);
}

void moveBackward() // Set all motors to run backwards
{
    rightBack.run(BACKWARD);
    rightFront.run(BACKWARD);
    leftFront.run(BACKWARD);
    leftBack.run(BACKWARD);
    delay(500);
}

void stopMove() // Set all motors to stop
{
    rightBack.run(RELEASE);
    rightFront.run(RELEASE);
    leftFront.run(RELEASE);
    leftBack.run(RELEASE);
}

void turnLeft(int duration) // Set motors to turn left for the specified duration then stop
{
    rightBack.setSpeed(motorSpeed + turnSpeed); // Set the motors to the motor speed
    rightFront.setSpeed(motorSpeed + turnSpeed);
    leftFront.setSpeed(motorSpeed + motorOffset + turnSpeed);
    leftBack.setSpeed(motorSpeed + motorOffset + turnSpeed);
    rightBack.run(FORWARD);
    rightFront.run(FORWARD);
    leftFront.run(BACKWARD);
    leftBack.run(BACKWARD);
    delay(duration);
    rightBack.setSpeed(motorSpeed); // Set the motors to the motor speed
    rightFront.setSpeed(motorSpeed);
    leftFront.setSpeed(motorSpeed + motorOffset);
    leftBack.setSpeed(motorSpeed + motorOffset);
    rightBack.run(RELEASE);
    rightFront.run(RELEASE);
    leftFront.run(RELEASE);
    leftBack.run(RELEASE);
}

void turnRight(int duration) // Set motors to turn right for the specified duration then stop
{
    rightBack.setSpeed(motorSpeed + turnSpeed); // Set the motors to the motor speed
    rightFront.setSpeed(motorSpeed + turnSpeed);
    leftFront.setSpeed(motorSpeed + motorOffset + turnSpeed);
    leftBack.setSpeed(motorSpeed + motorOffset + turnSpeed);
    rightBack.run(BACKWARD);
    rightFront.run(BACKWARD);
    leftFront.run(FORWARD);
    leftBack.run(FORWARD);
    delay(duration);
    rightBack.setSpeed(motorSpeed); // Set the motors to the motor speed
    rightFront.setSpeed(motorSpeed);
    leftFront.setSpeed(motorSpeed + motorOffset);
    leftBack.setSpeed(motorSpeed + motorOffset);
    rightBack.run(RELEASE);
    rightFront.run(RELEASE);
    leftFront.run(RELEASE);
    leftBack.run(RELEASE);
}

int getDistance() // Measure the distance to an object
{
    unsigned long pulseTime;  // Create a variable to store the pulse travel time
    int distance;             // Create a variable to store the calculated distance
    digitalWrite(trig, HIGH); // Generate a 10 microsecond pulse
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    pulseTime = pulseIn(echo, HIGH, timeOut);      // Measure the time for the pulse to return
    distance = (float)pulseTime * 340 / 2 / 10000; // Calculate the object distance based on the pulse time
    Serial.println(distance);
    return distance;
}

int checkDirection() // Check the left and right directions and decide which way to turn
{
    int distances[2] = {0, 0}; // Left and right distances
    int turnDir = 1;           // Direction to turn, 0 left, 1 reverse, 2 right
    servoLook.write(193);      // Turn servo to look left
    delay(500);
    distances[0] = getDistance(); // Get the left object distance
    servoLook.write(13);          // Turn servo to look right
    delay(500);
    distances[1] = getDistance(); // Get the right object distance
    servoLook.write(100);
    delay(300);
    if (distances[0] >= 100 && distances[1] >= 100) // If both directions are clear, turn left
        turnDir = 0;
    else if (distances[0] <= stopDist && distances[1] <= stopDist) // If both directions are blocked, turn around
        turnDir = 1;
    else if (distances[0] >= distances[1]) // If left has more space, turn left
        turnDir = 0;
    else if (distances[0] < distances[1]) // If right has more space, turn right
        turnDir = 2;
    return turnDir;
}
