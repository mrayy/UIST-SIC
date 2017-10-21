

#include "Common.h"
#include "MPU6050Control.h"


#include "BraccioControl.h"


MPUManager mpus;
BraccioManager braccio;
BendingSensor bending;
DetectionManager detector;
float euler[3];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    // initialize serial communication
    Serial.begin(38400);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    mpus.Initialize();
    bending.Initialize(A1);
    braccio.Initialize();

    
    // wait for ready
    Serial.println(F("\nSend any character to begin: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    Serial.println(F("Started!"));
    // configure LED for output
    pinMode(13, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void PrintData(float* e)
{

    
    //Braccio.ServoMovement(5,           0,  euler[2]*0.5+45, euler[2]*0.3+45,euler[2]*0.3+45, euler[1]+90,  73); 
    
      Serial.print("tilt:");Serial.print(euler[0]);
      Serial.print(", Pan:");Serial.print(euler[1]);
      Serial.print(", Roll:");Serial.print(euler[2]);
      Serial.println();
}

bool _printData=false;

void CheckInput()
{

    // send data only when you receive data:
    if (Serial.available() > 0) {
            // read the incoming byte:
            char c = Serial.read();
            switch(c)
            {
              case ' ':
              _printData=!_printData;
              break;
            case 'c':
              mpus.Calibrate();
              break;
            case 'e':
              braccio.SetEngaged(!braccio.IsEngaged());
              break;
            case 'b':
              if(bending.IsCalibrating())
                bending.EndCalibrate();
              else
                bending.StartCalibrate();
              break;
            }
    }
}
void loop() {

    CheckInput();
    mpus.Update();
    bending.Update();
    mpus.GetWristAngles(euler);

    bool triggered=detector.Update(euler[0],!braccio.IsEngaged());
    if(triggered)
    {
       braccio.SetEngaged(!braccio.IsEngaged());
    }

   /* mpus.GetWristAngles(euler);
    if(_printData)
      PrintData(euler);

    mpus.GetHandAngles(euler);
    if(_printData)
      PrintData(euler);
*/
  float b=bending.Value();
    if(_printData)
      PrintData(euler);

   braccio.Update(euler,b);

   Serial.println(b);
      
    if(_printData)
      Serial.println();

    //_printData=false;
}
