#pragma once
#include <Arduino.h>

// Parse a token that might be "kp=...", "ki=...", "kd=...", or single-char commands.
void parseToken(const String &token)
{
    if (token.startsWith("kp="))
    {
        String val = token.substring(3);
        float newKp = val.toFloat();
        if (newKp != 0.0 || val == "0" || val == "0.0")
        {
            kp = newKp;
            Serial.print("Updated kp to: ");
            Serial.println(kp);
        }
    }
    else if (token.startsWith("ki="))
    {
        String val = token.substring(3);
        float newKi = val.toFloat();
        if (newKi != 0.0 || val == "0" || val == "0.0")
        {
            ki = newKi;
            Serial.print("Updated ki to: ");
            Serial.println(ki);
        }
    }
    else if (token.startsWith("kd="))
    {
        String val = token.substring(3);
        float newKd = val.toFloat();
        if (newKd != 0.0 || val == "0" || val == "0.0")
        {
            kd = newKd;
            Serial.print("Updated kd to: ");
            Serial.println(kd);
        }
    }
    else if (token.startsWith("a="))
    {
        String val = token.substring(3);
        float newAlpha = val.toFloat();
        if (newAlpha != 0.0 || val == "0" || val == "0.0")
        {
            float COMP_ALPHA = newAlpha; // does nothing now but left in for structure
            Serial.print("Updated comp alpha to: ");
            Serial.println(newAlpha);
        }
    }
    // Here we allow m= to change maxServoChangePerUpdate (a float)
    else if (token.startsWith("m="))
    {
        String val = token.substring(2);
        float newM = val.toFloat();
        // Accept zero and non-zero floats
        if (newM != 0.0 || val == "0" || val == "0.0")
        {
            maxServoChangePerUpdate = newM;
            Serial.print("Updated maxServoChangePerUpdate to: ");
            Serial.println(maxServoChangePerUpdate);
        }
    }
    else
    {
        parseSingleCharCommands(token);
    }
}

// Function to process incoming serial commands
void processSerialCommands()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0)
            return;

        int startIndex = 0;
        while (true)
        {
            int spaceIndex = input.indexOf(' ', startIndex);
            String token;
            if (spaceIndex == -1)
            {
                token = input.substring(startIndex);
                token.trim();
                if (token.length() > 0)
                {
                    parseToken(token);
                }
                break;
            }
            else
            {
                token = input.substring(startIndex, spaceIndex);
                token.trim();
                if (token.length() > 0)
                {
                    parseToken(token);
                }
                startIndex = spaceIndex + 1;
            }
        }
    }
}