#include "Config.h"
#include <Servo.h>

Servo left_esc, right_esc;

#define LEFT_ESC_PIN    3
#define RIGHT_ESC_PIN   9

int left_esc_value = 700;
int right_esc_value = 700;

void setup()
{
  Serial.begin(115200);

  left_esc.attach(LEFT_ESC_PIN);
  right_esc.attach(RIGHT_ESC_PIN);

  // Directions
  Serial.println("To set values to the ESCs use \"l<value>r<value>\".");
  Serial.println("Do NOT use spaces, enter the command exactly as shown above.");
  Serial.println(" ");
  Serial.println("To set a single ESC use \"l<value>\" or \"r<value>\" respectively.");
  Serial.println(" ");
  Serial.println("Left ESC currently set to " + String(left_esc_value) + "us.");
  Serial.println("Right ESC currently set to " + String(right_esc_value) + "us.");
  Serial.println(" ");
}

void loop()
{
  // Write the values to the ESCs
  left_esc.writeMicroseconds(left_esc_value);
  right_esc.writeMicroseconds(right_esc_value);

  // Accept commands
  if(Serial.available() > 0)
  {
    // Define initial command processing state
    Command_State state = WAITING;

    // Loop through entire available data in serial buffer
    while(state != DONE)
    {
      // Parse command
      char command = Serial.read();

      // Process command
      switch(command)
      {
        case 'l': // Left ESC
          left_esc_value = Serial.parseInt();
          state = PROCESSING;
          break;
        case 'r': // Right ESC
          right_esc_value = Serial.parseInt();
          state = PROCESSING;
          break;
        default:
          state = DONE;
      }
    }

    // Display settings
    Serial.println("-> Left ESC set to " + String(left_esc_value) + "us. Right ESC set to " + String(right_esc_value) + "us.");
  }
}


