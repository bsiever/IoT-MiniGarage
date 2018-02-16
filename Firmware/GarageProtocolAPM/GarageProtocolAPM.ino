#include <AccelStepper.h>


// Arduino Pro Micro is an Arduino Leonardo Compatible (use that in settings)

//#define DEBUG 1

#ifdef DEBUG
#define debug_print(a)  Serial.print(a)
#define debug_println(a)  Serial.println(a)
#else
#define debug_print(a)  ;
#define debug_println(a)  ;

#endif

  
// Define Pins used 
const int rotationPin = 15;
const int downPin = 14;
const int upPin = 16;
const int laserPin = 10;
const int buttonPin = 8;
const int lightPin = 9;

const int speed = 450;  // Door speed // 350 orig (usb power)
const int doorFaultTime = 800; // Max time (in ms) between regsitering rotation while door is moving.

#define gc Serial1   // Garage Controller

// Forward declarations
static void rx();
static void sendMessage(byte type, byte *data, byte dataLength);

// Buffer for incoming messages
byte buffer[255];  // Should never exceed ~3 bytes on Photon; 255 on APM
byte bufferIndex = 0;
bool inMessage = false;
const byte startByte = 0x55;


byte statusByte = 0;

unsigned long lastRotationCheck = 0;
unsigned long lastLaserClear = 0;
unsigned long closedContactTime = 0;
unsigned long openedContactTime = 0;
unsigned long lastStatusByte = 0;
const int maxRotationErrorTime = 800;
const int maxContactErrorTime = 600;
unsigned long lastStatus = 0;

boolean lastRotationValue = false;

// Config the stepper
AccelStepper stepper(AccelStepper::FULL4WIRE, A0,A1,A2,A3); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5 


static enum {
    MOTOR_OPENING,
    MOTOR_CLOSING,
    MOTOR_STOPPED
} motorState = MOTOR_STOPPED;

// Status Byte bit masks
enum {
    SB_BUTTON_MASK = 0x01,
    SB_DOOR_OPENED_MASK = 0x02,
    SB_DOOR_CLOSED_MASK = 0x04,
    SB_FAULT_MASK = 0x08
};

// Enumeration of meaning of bytes within messages
enum {
    COMM_SIZE_BYTE=0,
    COMM_TYPE_BYTE=1,
    COMM_DATA_START=2
};

enum {
    MSG_STATUS = 1,
    MSG_MOTOR = 2,
    MSG_LIGHT = 3,
    MSG_ERROR = 4
};

enum {
    MOTOR_STOP = 0,
    MOTOR_OPEN = 1,
    MOTOR_CLOSE = 2
};


/*
  rx() : Retrieve any available bytes and process them
         If a complete, valid message is received, dispatch it as needed.
 */ 
static void rx() {
  while(gc.available()>0) {
    byte b = gc.read();
    if(!inMessage && b == startByte) {
      debug_println("Start of message");
      inMessage = true;
      bufferIndex = 0;
    } else if(inMessage) {
      buffer[bufferIndex] = b;
      bufferIndex++;
      // If we have a message size and all the bytes...
      if(bufferIndex>COMM_SIZE_BYTE && bufferIndex == buffer[COMM_SIZE_BYTE]) {
        debug_println("End of message");

        // Message complete.  Check the checksum and if good, Process the message
        byte total = 0;
        byte newStatus = 0;

        // Add all bytes from size to last data (inclusive)
        // Only ignores start (0x55) and length)
        for(int i=1;i<bufferIndex-1;i++) {
          total += buffer[i];
        }
        if(total == buffer[bufferIndex-1]) {
          debug_println("Good Checksum");

          // Good! Process it
          switch(buffer[COMM_TYPE_BYTE]) {  // Switch on type
            case MSG_STATUS: // Status byte (to Photon; Never to APM)
   
            break;

            case MSG_MOTOR: // Motor control (to APM; Never to Photon)
               debug_print("Motor Control: ");
               switch(buffer[COMM_DATA_START]) {
                case MOTOR_STOP:  // Stop
                  debug_println("stop");
                  stepper.setSpeed(0);  
                  motorState = MOTOR_STOPPED;  
                break;
                case MOTOR_OPEN:  // Open
                  debug_println("open");
                  stepper.setSpeed(-speed);                    
                  motorState = MOTOR_OPENING;  
                break;
                case MOTOR_CLOSE: // Close
                  debug_println("close");
                  stepper.setSpeed(speed);  
                  motorState = MOTOR_CLOSING;  
                break;
               }
            break;

            case MSG_LIGHT: // Light (to APM; Never to Photon)
               debug_print("Light Control: ");
               debug_println(buffer[COMM_DATA_START]);
               analogWrite(lightPin, buffer[COMM_DATA_START]);
            break;

            case MSG_ERROR: // Error (to APM; Never to Photon)
               debug_print("Error: ");
               // Add null and print
               buffer[bufferIndex-1]=0;
               Serial.println((char*)(buffer+COMM_DATA_START));
            break;

            default:
               debug_println("Undefined Message type");
          }
        }

        // Either way, reset
        inMessage = false;
        bufferIndex = 0;
      }
    }
  }
}

/*
  Send the designaged message type & data.  Builds length & checksum.
*/
static void sendMessage(byte type, byte *data, byte dataLength) {
  byte checkSum = 0;
  byte toSend;
  gc.write(startByte);      // Start byte
  gc.write(dataLength+3);  // Length of data + length byte + type + checksum
  gc.write(type);      // Write the type
  checkSum += type;
  for(int i=0;i<dataLength;i++) {
    gc.write(data[i]);
    checkSum += data[i];
  }
  gc.write(checkSum);
}



void setup() {
   gc.begin(57600);
   Serial.begin(9600);

   // Configure all I/O
   stepper.setMaxSpeed(4500);  // BSIEVER: FIX!
   stepper.setSpeed(0);  
   pinMode(rotationPin, INPUT_PULLUP);
   pinMode(downPin, INPUT_PULLUP);
   pinMode(upPin, INPUT_PULLUP);
   pinMode(buttonPin, INPUT_PULLUP);
   pinMode(laserPin, INPUT);   
   pinMode(lightPin, OUTPUT);
   digitalWrite(lightPin, LOW);   
}


void showInputHeader() {
   Serial.println("Up\tDown\tButton\tLaser\tRotation");
}

void showInputs() {
          Serial.print(digitalRead(upPin));
          Serial.print("\t");
          Serial.print(digitalRead(downPin));
          Serial.print("\t");
          Serial.print(digitalRead(buttonPin));
          Serial.print("\t");
          Serial.print(digitalRead(laserPin));
          Serial.print("\t");
          Serial.println(digitalRead(rotationPin));  
}

void manualControl() {
  if(Serial.available()) {
    switch(Serial.read()) {
      default:
      case '?':
        Serial.println("Single letter commands: ");
        Serial.println("\t s -> Show status of inputs for 5s");
        Serial.println("\t o -> Completely open the door");
        Serial.println("\t c -> Completely close the door");
        Serial.println("\t l -> Light the LED for 2s");
      break;

      case '\n':
      case '\r':
        // Ignore End of line characters
      break;

      case 'o':
        Serial.println("Opening the door");
         stepper.setSpeed(-speed);                    
         while(digitalRead(upPin)!=LOW) {
            stepper.runSpeed();
            showInputs();
         }
         Serial.println("Done");
      break;

      case 'c':
        Serial.println("Closing the door");
         stepper.setSpeed(speed);                    
         while(digitalRead(downPin)!=LOW) {
            stepper.runSpeed();
            showInputs();
         }
         Serial.println("Done");
      break;

      case 's': {
        Serial.println("Showing all inputs for 5s");        
        unsigned long now = millis();
          int i=0;
          Serial.println("Status: " );
        while(millis()-now < 5000) {
          if(i++ % 100 == 0) 
            showInputHeader();
          showInputs();
        }
      }
      break;

      case 'l':
        Serial.println("Light for 2s");
        digitalWrite(lightPin, HIGH);
        delay(2000);
        digitalWrite(lightPin, LOW);
      break;
    }
  }
}


void loop() {
  static bool laserFaultReported = false;
  static bool obstructionFaultReported = false;
  unsigned long now = millis();

  // Process any manual commands
  manualControl();

  // Process incoming messages
  rx();
  
  // Check for status changes on buttons/switches
  byte newStatusByte = 0;

  // If the button is pressed
  if(digitalRead(buttonPin)==LOW) {
    newStatusByte |= SB_BUTTON_MASK;
  }

  // If the closed switch is hit
  if(digitalRead(downPin)==LOW) {
    newStatusByte |= SB_DOOR_CLOSED_MASK;
  } else {
    closedContactTime = now;
  }

  // If the opened switch is hit
  if(digitalRead(upPin)==LOW) {
    newStatusByte |= SB_DOOR_OPENED_MASK;
  } else {
    openedContactTime = now;
  }

  // Check for laser faults
  if(motorState == MOTOR_CLOSING && digitalRead(laserPin) == LOW) {
    newStatusByte |= SB_FAULT_MASK;
    if(!laserFaultReported) {
      Serial.println("Fault due to laser obstruction");
      laserFaultReported = true;
    }
  } else {
    lastLaserClear = now;
    laserFaultReported = false;
  }

  // Check for rotation faults
  if(motorState!=MOTOR_STOPPED) {
    bool rotation = (digitalRead(rotationPin) == LOW);

    if(rotation != lastRotationValue) {
      lastRotationCheck = now;      
    }
    lastRotationValue = rotation;

    if(now-lastRotationCheck > doorFaultTime) {
      newStatusByte |= SB_FAULT_MASK;
      if(!obstructionFaultReported) {
        Serial.println("Fault due to motor obstruction");
        obstructionFaultReported = true;
      }
    }
  } else {
    lastRotationCheck = now;
    obstructionFaultReported = false;
  }



  // Send updated status byte
  if(newStatusByte != lastStatusByte || now-lastStatus >= 1000) {
    debug_print("New Status byte: ");
    debug_println(newStatusByte);
    sendMessage(MSG_STATUS, &newStatusByte, 1);
    lastStatus = now;
  }

  
  if(motorState == MOTOR_OPENING && now-openedContactTime > maxContactErrorTime) {
    Serial.println("EMERGENCY STOP! Error --- Opening even though contact hit");
    stepper.setSpeed(0);
    motorState = MOTOR_STOPPED;
  }
  if(motorState == MOTOR_CLOSING && now-closedContactTime > maxContactErrorTime) {
    Serial.println("EMERGENCY STOP! Error --- Closing even though contact hit");
    stepper.setSpeed(0);
    motorState = MOTOR_STOPPED;
  }  
  // Rotation fault!
  if(motorState != MOTOR_STOPPED && now-lastRotationCheck > maxRotationErrorTime) {
    Serial.println("EMERGENCY STOP! Error --- Fault due to motor obstruction");
    stepper.setSpeed(0);
    motorState = MOTOR_STOPPED;
  }
  // Obstruction!
  if(motorState == MOTOR_CLOSING && now-lastLaserClear > maxContactErrorTime) {
    Serial.println("EMERGENCY STOP! Error --- Fault due to beam obstruction");
    stepper.setSpeed(0);
    motorState = MOTOR_STOPPED;
  }

  lastStatusByte = newStatusByte;

  // Stepper updates (if needed)  
  if(motorState != MOTOR_STOPPED)
    stepper.runSpeed();
  else 
    stepper.disableOutputs();
}
 