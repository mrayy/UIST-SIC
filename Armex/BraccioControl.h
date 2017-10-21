
#include <Braccio.h>
#include <Servo.h>


// Braccio  related
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;


class BraccioManager
{
  bool _isEngaged;

  float _PitchWeights[3];
  float _PitchOffsets[3];
public:
  bool Initialize()
  {
    Braccio.begin();
    _isEngaged=false;

    //reset the arm to the initial position
     _SetInitial();

    _PitchWeights[0]=-0.5;
    _PitchWeights[1]=-0.25;
    _PitchWeights[2]=-0.25;

    _PitchOffsets[0]=45;
    _PitchOffsets[1]=45;
    _PitchOffsets[2]=0;
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
    M2=_PitchWeights[0]*angles[0]+_PitchOffsets[0];
    M3=_PitchWeights[1]*angles[0]+_PitchOffsets[1];
    M4=_PitchWeights[2]*angles[0]+_PitchOffsets[2];
    M5=angles[2];
    M6=constrain(bending*100,10,73);
    
    Braccio.ServoMovement(5, 90, M2, M3, M4, M5,  M6);

  #if true
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

