/* Simple skeleton test program for in-robot API */

#include "allcode_api.h"    // MUST include this to define robot functions

int main()
{
    FA_RobotInit();         // MUST call this before any other robot functions
    
    FA_LCDBacklight(50);    // Switch on backlight (half brightness)
    FA_LCDPrint("Hello", 5, 20, 25, FONT_NORMAL, LCD_OPAQUE);     // Say hi!
    FA_DelayMillis(1000);   // Pause 1 sec

    while(1)            // Execute this loop as long as robot is running
    {                   // (this is equivalent to Arduino loop() function_

        // Uncomment this code to activate motor functions

        FA_SetMotors(20, 20);   // Drive forward
      
        if(FA_ReadIR(1) > 600)  // Check front left IR
        {
            FA_Backwards(50);   // Dodge right if obstacle
            FA_Right(30);      
        }

        if(FA_ReadIR(2) > 600)  // Check front centre IR
        {
            FA_Backwards(50);   // Skip back if obstacle
        }

        if(FA_ReadIR(3) > 600)  // Check front right IR
        {
            FA_Backwards(50);   // Dodge left if obstacle
            FA_Left(30);      
        }



        FA_LCDClear();  // Clear display
        // Display left and right wheel encoder values
        FA_LCDNumber(FA_ReadEncoder(CHANNEL_LEFT), 0, 0, FONT_NORMAL, LCD_OPAQUE);
        FA_LCDNumber(FA_ReadEncoder(CHANNEL_RIGHT), 40, 0, FONT_NORMAL, LCD_OPAQUE);

        // Display battery reading
        FA_LCDNumber(FA_ReadBattery(), 0, 8, FONT_NORMAL, LCD_OPAQUE);

        // Display left and right line sensors
        FA_LCDNumber(FA_ReadLine(CHANNEL_LEFT), 0, 16, FONT_NORMAL, LCD_OPAQUE);
        FA_LCDNumber(FA_ReadLine(CHANNEL_RIGHT), 80, 16, FONT_NORMAL, LCD_OPAQUE);
        
        if(FA_ReadSwitch(0) == 1)   // If left button pressed
            FA_PlayNote(1200,200);  // play a low note
        if(FA_ReadSwitch(1) == 1)   // If right button pressed
            FA_PlayNote(2200,200);  // play a high note
        
        FA_DelayMillis(100);    // Pause 0.1 sec
        
        int i;
        for (i = 0; i < 8; i++) {       // Loop over all 8 IR sensors
            if (FA_ReadIR(i) > 600) {   // If obstacle, light corresponding LED
                FA_LEDOn(i);
            }
            else {
                FA_LEDOff(i);           // else switch LED off
            }
        }
    }
    
    return 0; // Actually, we should never get here...
}

