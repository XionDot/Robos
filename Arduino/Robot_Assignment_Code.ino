#define PBL 4      // Left pushbutton pin number
#define PBR 2      // Right pushbutton pin number
#include <Servo.h> // Importing the library
#define L_STOP 85  // Left Servo Stopping Value
#define R_STOP 78  // Right Servo Stopping Value
#define L_PIN 6
#define R_PIN 5
#define IR_TRANSMIT 3 //RX PIN
#define IR_RECIEVER 2 //TX PIN
#include <EEPROM.h>

// 50hz light flash
// Left & Right offset values (values for it to move straight) 19 20
int counter = 0; // Global Counter to be used in while & for loops and if statments

const int ledGreen = 7;
const int ledYellow = 12;
const int ledRed = 13;

int ldr_list[3]; // Array of all ldr sensor reading averages

//Array for storing the Light Dependent Resistors Sensor's Overall Threshold | Light Limit | Dark Limit Respectively.
int sensorLDRThres[3];   // Threshold Array
int lightSensorLimit[3]; // Light Limit Array
int darkSensorLimit[3];  // Dark Limit Array

const int ldr_darkness_limit = 650; // limit for when ldr stops detecting darkness

const int one_degree_turn_time = 18; // time for robot to turn 1 degree.
const int circle_turn_time = 12590;  // actual value = 12.59s
const int meter_time = 14440;        // actual value = 14.44s

int angle = 0;

// the stop point for the left servo
const byte LEFT_STOP = 85;

// the stop point for the right servo
const byte RIGHT_STOP = 78;

// the values to add to / subtract from the stop point to make the robot drive straight
const byte LEFT_OFFSET = 19;
const byte RIGHT_OFFSET = 20;

const byte WHEEL_DIAMETER_IN_MM = 43;

//the threshold between light and dark for each of the LDRs
const unsigned int LEFT_THRESHOLD = sensorLDRThres[0];
const unsigned int MIDDLE_THRESHOLD = sensorLDRThres[1];
const unsigned int RIGHT_THRESHOLD = sensorLDRThres[2];



// Calling left & right Servos
Servo leftServo;
Servo rightServo;

void storeBValue(int eepromAddress, byte value, char *description)
{
  EEPROM.update(eepromAddress, value);
  Serial.print("Storing value ");
  Serial.print(value);
  Serial.print(" to address ");
  Serial.print(eepromAddress);
  Serial.print(" to save the ");
  Serial.println(description);
}
// storeValue sets a single byte in the EEPROM.  Do not edit this function
void storeUIValue(int eepromAddress, unsigned int value, char *description)
{
  EEPROM.put(eepromAddress, value);
  Serial.print("Storing value ");
  Serial.print(value);
  Serial.print(" to address ");
  Serial.print(eepromAddress);
  Serial.print(" to save the ");
  Serial.println(description);
}

byte readBValue(int eepromAddress)
{
  return EEPROM[eepromAddress];
}

unsigned int readUIValue(int eepromAddress)
{
  unsigned int uiVal;
  EEPROM.get(eepromAddress, uiVal);

  return uiVal;
}

void setSpeed(int s_Speed) // Funtion for setting the rotational speed of the motors
{
    leftServo.write(L_STOP + s_Speed);
    rightServo.write(R_STOP - s_Speed);
}

void moveStop() // FUNCTION TO STOP MOTORS
{
    leftServo.write(L_STOP);
    rightServo.write(R_STOP);
}

void move_Forward(float dist_M) // FUNCTION TO MOVE A GIVEN DISTANCE FORWARD
{
    leftServo.write(L_STOP + 20);
    rightServo.write(R_STOP - 20);
    int dist_Time = (dist_M * meter_time);
}

void move_Backwards(float dist_M) // FUNCTION TO MOVE A GIVEN DISTANCE BACKWARDS
{
    leftServo.write(L_STOP - 20);
    rightServo.write(R_STOP + 20);
    float dist_Time = (dist_M * meter_time); // converts the distance into the time for the movement.
    delay(dist_Time);                        //Uses the time from the conversion for the delay.
}

void turn_Right()
{
    leftServo.write(L_STOP + 20);
    rightServo.write(R_STOP + 20);
}

void turn_Left()
{
    leftServo.write(L_STOP - 20);
    rightServo.write(R_STOP - 20);
}

void turnServos_Clock(int angle) // FUNCTION TO TURN CLOCK-WISE AT GIVEN ANGLE.
{
    leftServo.write(L_STOP + 20);
    rightServo.write(R_STOP + 20);
    int angle_time = (angle * one_degree_turn_time); //converts the angle given into the appropriate time
    delay(angle_time);                               //Uses the time from the conversion for the delay.
}

void turnServos_AntiClock(int angle) // FUNCTION TO TURN ANTICLOCK-WISE AT GIVEN ANGLE.
{
    leftServo.write(L_STOP - 20);
    rightServo.write(R_STOP - 20);
    int angle_time = (angle * one_degree_turn_time); //converts the angle given into the appropriate time
    delay(angle_time);                               //Uses the time from the conversion for the delay
}

// Led = pin you want to modify, LedValue is the value that you wish to change
// ledGreen = 7 | ledYellow = 12 | ledRed = 13
void setLED(int led, int ledValue)
{
    if (ledValue != 0) //  If ledValue is 0 then the chosen LED will turn ON.
    {
        digitalWrite(led, HIGH);
    }
    else //  If ledValue is 0 then the chosen LED will turn OFF.
    {
        digitalWrite(led, LOW);
    }
}

//highOrlow = binary value to turn all leds on and off (1|ON , 0|OFF)
void ledFunc(int highOrlow)
{
    if (highOrlow != 0) // If ledValue is 1 then all LEDs turn ON
    {
        highOrlow = HIGH;
    }
    else // If ledValue is 0 then all LEDs turn OFF
    {
        highOrlow = LOW;
    }
    //Led pins
    int led_array[3] = {ledGreen, ledYellow, ledRed}; // Array storing all of the LED variables.
    for (int i = 0; i < 3; i++)                       // Increments through the array.
    {
        digitalWrite(led_array[i], highOrlow); // tells the robot which LED will turn on.
    }
}

void ldrSensorReadings() // FUNCTION USED TO OBTAIN LDR SENSOR READINGS.
{
    // Defining all needed variables.
    int left_LDR = 0;
    int center_LDR = 0;
    int right_LDR = 0;
    int i;
    for (int j = 0; j < 3; j++) // Traverses through the list indexs
    {
        // For each for the ldr the will be 3 readings added up to achieved a large number.
        left_LDR += analogRead(A2);
        center_LDR += analogRead(A1);
        right_LDR += analogRead(A0);
        delay(5); // Delay used to prevent errors.
    }
    // Calculates the average of all the total readings
    int left_ldr_average = left_LDR / 3;
    int center_ldr_average = center_LDR / 3;
    int right_ldr_average = right_LDR / 3;
    // Store all the averages in an Array.
    ldr_list[0] = left_ldr_average;
    ldr_list[1] = center_ldr_average;
    ldr_list[2] = right_ldr_average;
}

void calibrationLightLimit() // FUNCTION USED TO CALIBRATE ROBOT.
{
    //Bool variables defined to be used in statement comparison.
    bool lightCalibration = false;
    bool darkCalibration = false;

    setLED(13, 1);                                                    //Turns on red led
    while ((lightCalibration == false) || (darkCalibration == false)) // While either calibration expressions are false, begin the loop.
    {
        if (digitalRead(PBL) == LOW) // If the left button is pressed
        {
            ledFunc(1);                 // turn all LEDs ON
            delay(2000);                // for 2 seconds.
            ldrSensorReadings();        // call the LDR Sensor funtion.
            for (int i = 0; i < 3; i++) // Traverses through the list indexs
            {
                lightSensorLimit[i] += ldr_list[i]; // For each index storage the corresponding value in the lightLimit array.
            }
            ledFunc(0);  // turn all LEDs OFF
            delay(1000); // for 1 second

            lightCalibration = true; // the light is now calibrated
            Serial.println("Light Sensor Calibrated");
            setLED(7, 1);
            for (int i = 0; i < 3; i++)
            {
                Serial.println(lightSensorLimit[i]); // For testing - print out all the sensor limits stored.
                Serial.println("These are limits");
            }
        }
        setLED(13, 1);
        while ((darkCalibration == false) && (lightCalibration == true)) // While the darkCalibration expressions is false and the lightCalibration is false, begin the loop.
        {
            if (digitalRead(PBL) == LOW) // If the left button is pressed
            {
                ledFunc(0);                 // turn all LEDs OFF.
                delay(2000);                // for 2 seconds.
                ldrSensorReadings();        // call the LDR Sensor funtion.
                for (int i = 0; i < 3; i++) // Traverses through the list indexs
                {
                    darkSensorLimit[i] += ldr_list[i]; // For each index storage the corresponding value in the dark limit array.
                }
                delay(200);
                darkCalibration = true; // Sets the dark Calibration to true because it has been calibrated.
                ledFunc(1);
                Serial.println("Dark Sensor Calibrated");
                for (int i = 0; i < 3; i++) // For testing run through the array and print all the readings.
                {
                    Serial.println(darkSensorLimit[i]);
                    Serial.println("These are limits");
                }
            }
        }
    }
    Serial.println("| Threshold Limit Start |");
    sensorThresh(); // Calls the sensor Threshold function to calculate and store all of the total overall Thresholds.
    Serial.println("| Threshold Limit End |");
} // ^^ Prints out all the overall Thresholds for each of the LDRs. ^^

void sensorThresh()
{
    sensorLDRThres[3] = {};     // Declaring empty array to store all the thresholds
    for (int i = 0; i < 3; i++) // loop to run through both arrays called convert and store in above array.
    {
        sensorLDRThres[i] = (darkSensorLimit[i] + lightSensorLimit[i]) / 2; // Adds the 2 corresponding values of each array position and calculates averages and stores them.
        Serial.println(sensorLDRThres[i]);                                  // For testing prints out the whole array.
    }
}

void letsDance() // Stage 2 Moving forwards and backwards for 50cm.
{
    move_Forward(0.5);   // Moves robot forward for 50cm.
    delay(500);          // When finished waits 500ms.
    move_Backwards(0.5); // Moves robot backwards for 50cm.
    moveStop();          // When finished stops robot.
    delay(700);
    turnServos_Clock(90); // Turns robot 90 degrees clock-wise.
    moveStop();
    turnServos_AntiClock(90); // Turns robot 90 degrees anitclock-wise.
}

void danceSeq()
{
    move_Forward(0.1);    // Moves robot forward for 10 cm.
    turnServos_Clock(12); // Turns robot 12 degrees clock-wise.
    setLED(12, 1);        // Light show
    setLED(13, 1);        // Light show
    setLED(12, 1);        // Light show
    setLED(13, 0);        // Light show
    ledFunc(1);           // Light show
    move_Backwards(0.2);  // Moves robot backwards for 20cm.
    delay(200);
    turnServos_Clock(180); // Turns robot 180 degrees clock-wise.
    moveStop();
    delay(300);
    move_Forward(0.8); // Moves robot forwards for 80cm.
    ledFunc(0);
    setLED(7, 1);             // Light show.
    turnServos_AntiClock(90); // Turns robot 90 degrees anticlock-wise.
    move_Backwards(0.15);     // Moves robot backwards for 15cm..
    setLED(13, 1);            // Light show.
    setLED(12, 1);            // Light show.
    setLED(13, 0);            // Light show.
    ledFunc(0);               // Light show.
}

void junction_Count()
{

    ldrSensorReadings(); // Calling The Sensor reading function to get the current readings of LDRs.
    if (ldr_list[0] < sensorLDRThres[0] && ldr_list[1] < sensorLDRThres[1] && ldr_list[2] < sensorLDRThres[2])
    { // ^^ If all LDRs are within the threshold then a junction is found.
        setLED(7, 1);
        moveStop();
        counter++; // Increments the counter by +1 each loop.
        ledFunc(0);
        delay(200);
        Serial.println("Adding to Counter");
        Serial.println(counter);
        if (counter >= 2) // Checks if counter is equal to 3 (0,1,2) or higher.
        {
            moveStop();            // Call function to stop the Servos.
            turnServos_Clock(180); // Turn robot 180 degrees clock-wise.
            delay(800);
        }
    }
}

void backandForward()
{
    ldrSensorReadings(); // Calling The Sensor reading function to get the current readings of LDRs.
    if (ldr_list[0] < sensorLDRThres[0] && ldr_list[1] < sensorLDRThres[1] && ldr_list[2] < sensorLDRThres[2])
    { // ^^ If all LDRs are within the threshold then a junction is found.
        junction_Count();
        move_Backwards(20);
        delay(800);
        Serial.println("Junction Found");
    }
    if (ldr_list[1] < sensorLDRThres[1]) // Checks if the mid ldr sensor is within Threshold || If so set course forward.
    {
        setLED(12, 1); // Set Forward Path for Robo is darkness detected from center ldr
        move_Backwards(20);
        Serial.println("Moving Forward");
        //moveStop();
    }
    if (ldr_list[2] < sensorLDRThres[2]) // Checks if the right ldr sensor is within Threshold || If so adjust the robot to turn right.
    {                                    // Angles Robo Right if darkness is detected from right ldr
        turn_Left();
        Serial.println("Moving Right");
        //moveStop();
    }
    if (ldr_list[0] < sensorLDRThres[0]) // Checks if the left ldr sensor is within Threshold || If so adjust the robot to turn left.
    {                                    // Angles Robo Left if darkness is detected from left ldr
        turn_Right();
        Serial.println("Moving Left");
        //moveStop();
    }

    delay(200);
}

void obstacleDetection()
{
    tone(3, 38000);                      // tone function called to start sending IR signal at 38 kHz.
    if (digitalRead(IR_RECIEVER) == LOW) // Checks if IR sensor is recieving signals.
    {
        noTone(3); // noTone function called to stop sending IR signals.
        ledFunc(1);
        Serial.println("Obstacle Detected");
        delay(1000);
        moveStop();
        turnServos_Clock(180); // Turn robot 180 degrees clock-wise.
        ldr_Motion();          // function called to resume movement along path.
    }
    else
    {
        ldr_Motion();
    }
}

void ldr_Motion() // FUNCTION USED FOR THE LDR MOVEMENT STAGE
{
    ldrSensorReadings(); // Calling The Sensor reading function to get the current readings of LDRs.
    if (ldr_list[0] < sensorLDRThres[0] && ldr_list[1] < sensorLDRThres[1] && ldr_list[2] < sensorLDRThres[2])
    {                     // ^^ If all LDRs are within the threshold then a junction is found.
        junction_Count(); // If junction is found jump to the junction_Count function and start count.
        setSpeed(20);
        delay(800);
        Serial.println("Junction Found");
    }
    if (ldr_list[1] < sensorLDRThres[1]) // Checks if the mid ldr sensor is within Threshold || If so set course forward.
    {
        setLED(12, 1);
        setSpeed(20); // Set Forward Path for Robo is darkness detected from center ldr
        Serial.println("Moving Forward");
        //moveStop();
    }
    if (ldr_list[2] < sensorLDRThres[2]) // Checks if the right ldr sensor is within Threshold || If so adjust the robot to turn right.
    {                                    // Angles Robo Right if darkness is detected from right ldr
        turn_Right();
        Serial.println("Moving Right");
        //moveStop();
    }
    if (ldr_list[0] < sensorLDRThres[0]) // Checks if the left ldr sensor is within Threshold || If so adjust the robot to turn left.
    {                                    // Angles Robo Left if darkness is detected from left ldr
        turn_Left();
        Serial.println("Moving Left");
        //moveStop();
    }

    delay(200);
}

void setup()
{
    Serial.begin(9600);

    storeBValue(0, LEFT_STOP, "left servo stopping point");
    storeBValue(1, RIGHT_STOP, "right servo stopping point");
    storeBValue(2, LEFT_OFFSET, "left servo difference from LEFT_STOP to drive straight (when the right value is also set on the right servo");
  storeBValue(3, RIGHT_OFFSET, "right servo difference from RIGHT_STOP to drive straight (when the left value is also set on the left servo");

  storeBValue(4, WHEEL_DIAMETER_IN_MM, "wheel diameter");
  storeUIValue(10, LEFT_THRESHOLD, "threshold to separate light and dark values for the left LDR (A2)");
  storeUIValue(13, MIDDLE_THRESHOLD, "threshold to separate light and dark values for the middle LDR (A1)");
  storeUIValue(16, RIGHT_THRESHOLD, "threshold to separate light and dark values for the right LDR (A0)");

#ifdef CONFIRM
  // now print out the values as a test
  Serial.println(readBValue(0));
  Serial.println(readBValue(1));
  Serial.println(readBValue(2));
  Serial.println(readBValue(3));
  Serial.println(readBValue(4));

  Serial.println(readUIValue(10));
  Serial.println(readUIValue(13));
  Serial.println(readUIValue(16));
#endif

    pinMode(7, OUTPUT);           // Green LED pin output setup.
    pinMode(12, OUTPUT);          // Yellow LED pin output setup.
    pinMode(13, OUTPUT);          // Red LED pin output setup.
    pinMode(PBL, INPUT);          // Left button INPUT.
    pinMode(PBR, INPUT);          // Right button INPUT.
    pinMode(0, INPUT);            // LEFT LDR INPUT.
    pinMode(1, INPUT);            // CENTER LDR INPUT.
    pinMode(2, INPUT);            // Right LDR INPUT.
    pinMode(IR_RECIEVER, INPUT);  // INFRARED Sensor INPUT.
    pinMode(IR_TRANSMIT, OUTPUT); // INFRARED Transmitter OUTPUT.

    rightServo.attach(R_PIN);
    leftServo.attach(L_PIN);
    Serial.println("Servo Attached");
    moveStop(); // Call function to stop the Servos.
}

void loop()
{
    if (digitalRead(PBL) == LOW) // If Left button pressed first it will call the calibration function.
    {
        delay(20);
        calibrationLightLimit();
    }
    if (digitalRead(PBL) == LOW && digitalRead(PBR) == LOW) // Wait for both buttons to be pressed.
    {
        delay(20);
        while (digitalRead(PBL) == HIGH && digitalRead(PBR) == HIGH) // Wait for both buttons to be released.
        {                                                            // IF both left &right buttons are pressed the first stage will be run
            letsDance();
            delay(3000);
            danceSeq();
        }
    }
    if (digitalRead(PBR) == LOW) // Wait for Right button to be pressed.
    {
        delay(20);
        while (digitalRead(PBR) == HIGH) // Wait for Right button to be released.
        {                                // If Right button pressed stage 2.3, 3 & 4 will run
            obstacleDetection();
        }
    }
}