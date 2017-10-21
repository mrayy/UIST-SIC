
#include <Braccio.h>
#include <Servo.h>


// Braccio  related
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

#define PitchWeights0 -0.5
#define PitchWeights1 -0.25
#define PitchWeights2 -0.25

#define PitchOffsets0 45
#define PitchOffsets1 45
#define PitchOffsets2 0


class BraccioManager
{
  bool _isEngaged;

public:
  bool Initialize()
  {
    Braccio.begin();
    _isEngaged=false;

    //reset the arm to the initial position
     _SetInitial();

    return true;
  }
  void _SetInitial()
  {
    Braccio.ServoMovement(20, 90, 160, 150, 100, 0,  10);
  }

  void Update(float angles[3],float bending)
  {
    if(!_isEngaged)
      return;

    float M2,M3,M4,M5, M6;
    M2=PitchWeights0*angles[0]+PitchOffsets0;
    M3=PitchWeights1*angles[0]+PitchOffsets1;
    M4=PitchWeights2*angles[0]+PitchOffsets2;
    M5=angles[2];
    M6=constrain(bending*100,10,73);
    
    Braccio.ServoMovement(5, 90, M2, M3, M4, M5,  M6);

  #ifdef DEBUG_OUTPUT
    Serial.print(angles[0]);Serial.print(":  ");
    Serial.print(M2);Serial.print(", ");
    Serial.print(M3);Serial.print(", ");
    Serial.print(M4);Serial.print(", ");
    Serial.print(M5);Serial.print(", ");
    Serial.print(M6);Serial.println();
  #endif
  }

  void SetEngaged(bool e)
  {
    _isEngaged=e;

    if(!_isEngaged)
    {
      //Move to Initial Position
       _SetInitial();
    }else{
      
      Braccio.ServoMovement(20, 90, 45, 45, 45, 0,  10);
    }
  }

  bool IsEngaged(){
    return _isEngaged;
  }
  

};

