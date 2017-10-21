
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"


#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



#include "MPUCalibrate.h"
#define LED_PIN2 LED_BUILTIN


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


  Quaternion q;
  VectorFloat gravity;
  
class MPUStatus
{
public:
  MPU6050 mpu;
  int intID;
  uint8_t fifoBuffer[64];
  float ypr[3];
  bool dmpReady;
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  volatile bool mpuInterrupt;

  //Average<float> smoothed[3];

  void (* _callbackfunc)();
  MPUStatus(int ID):mpu(ID)//,smoothed{Average<float>(SAMPLES_COUNT),Average<float>(SAMPLES_COUNT),Average<float>(SAMPLES_COUNT)}
  {
  }
  bool Initalize(int intID,char* name,void (* func)())
  {
    this->intID=intID;
    _callbackfunc=func;
    Serial.print("Begin Initializing: ");
    Serial.println(name);
    
    mpu.initialize();
    
    pinMode(intID, INPUT);
    // load and configure the DMP
   // Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu.dmpInitialize();
    Serial.print(mpu.testConnection() ? F("connection successful:") : F("connection failed:"));
    
    
    // supply your own gyro offsets here, scaled for min sensitivity
    //mpu.setXGyroOffset(220);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    dmpReady=false;
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
       // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);


        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
      attachInterrupt(digitalPinToInterrupt(intID), func, RISING);
      mpuIntStatus = mpu.getIntStatus();
      
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    mpuIntStatus = mpu.getIntStatus();
    if(!dmpReady)
      return false;

      Serial.println("Initializing Successful");
    return true;
  }

  
  bool GetAngles(float* euler)
  {
   //   mpu.dmpGetEuler(euler, &q);
     euler[0]=ypr[2]* RAD_DEG;
     euler[1]=ypr[0]* RAD_DEG;
     euler[2]=ypr[1]* RAD_DEG;
     
      return true;
  }

  float GetTilt()
  {
    return ypr[2]* RAD_DEG;
  }

  bool Update()
  {
    
    if (!dmpReady) return;
    // wait until new packets arrive
    while (!mpuInterrupt && fifoCount < packetSize) {
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return false;

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
    }

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //  for(int i=0;i<3;++i){
      //  smoothed[i].push(ypr[i]);
    //    ypr[i]=smoothed[i].mean();
    //  }
    return true;
  }

  void Calibrate()
  {
    
    mpu.initialize();
       if(autocalibrate('g','X',mpu)){
          Serial.println("Gyro X axis calibrated");
       }else{
          Serial.println("Gyro X axis calibration failed");}
          
      if(autocalibrate('g','y',mpu)){
               Serial.println("Gyro Y axis calibrated");
      }else{
          Serial.println("Gyro Y axis calibration failed");}
          
      if(autocalibrate('g','z',mpu)){
               Serial.println("Gyro Z axis calibrated");
      }else{
          Serial.println("Gyro Z axis calibration failed");}

       Serial.println("Calibrating Accelerometer");
       
      if(autocalibrate('a','x',mpu)){
               Serial.println("Accel X axis calibrated");
      }else{
          Serial.println("Accel X axis calibration failed");}
          
      if(autocalibrate('a','y',mpu)){
               Serial.println("Accel Y axis calibrated");
      }else{
          Serial.println("Accel Y axis calibration failed");}
          
     if(autocalibrate('a','z',mpu)){
               Serial.println("Accel Z axis calibrated");
      }else{
          Serial.println("Accel Z axis calibration failed");}

         
// load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    int devStatus = mpu.dmpInitialize();

      // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(intID, _callbackfunc, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  }
};

   MPUStatus _mpu[2]={MPUStatus(0x69),MPUStatus(0x68)};

void dmpDataReady0() {
    _mpu[0].mpuInterrupt = true;
}
void dmpDataReady1() {
    _mpu[1].mpuInterrupt = true;
}

class BendingSensor
{
  int16_t _min,_max;
 // Average<int16_t> bendingSmoothed;
  float value;
  bool _calibrating;
 public:
 BendingSensor():/*bendingSmoothed(BENDING_SAMPLES_COUNT)*/value(0),_calibrating(false),_min(0),_max(1023)
 {}
  void Initialize()
  {
    pinMode(FLEX_PIN, INPUT);
    
  }
  void Update()
  {
    value = analogRead(FLEX_PIN);
    if(_calibrating){
      if(value<_min)
        _min=value;
      if(value>_max)
        _max=value;
    }else{
      //bendingSmoothed.push(val);
      /*Serial.print(val);
      Serial.print(",  ");
      Serial.println(bendingSmoothed.mean());*/
    }
  }

  float Value(){
    
    return (float)(value-_min)/(float)(_max-_min);//bendingSmoothed.mean()
  }
  float RawValue(){
    
    return value;//bendingSmoothed.mean();
  }


  void StartCalibrate()
  {
    _calibrating=true;
    _max=0;
    _min=2000;
  }

  void EndCalibrate(){
    _calibrating=false;
    
    Serial.print(_min);
      Serial.print(",  ");
      Serial.println(_max);
  }

  bool IsCalibrating()
  {
    return _calibrating;
  }
};


class MPUManager
{
  bool _status;
  float _e0[3];
  float _e1[3];
  float ypr[3];
//internal

  int currentMillis;
  bool _calibrate;
  bool waitstatus;  
  int lastPercentage;
  bool blinkState;
public:
  MPUManager()
  {
    _calibrate=false;
    waitstatus=false;
  }
  bool Initialize()
  {
    _status=false;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    
    Serial.println(F("Initializing MPUs devices..."));
    if(!_mpu[0].Initalize(2,"Wrist",dmpDataReady0))
      return false;
    if(!_mpu[1].Initalize(3,"Hand",dmpDataReady1))
      return false;

    _status=true;
    return true;
  }

  bool Status()
  {
    return _status;
  }


  void Update(){
    // if programming failed, don't try to do anything
    if (!Status()) return;
    
   int dt=(millis()-currentMillis);
    if(_calibrate)
    {
      if(dt-StabilityTime<0)
      {
       if(waitstatus==false){
          Serial.println("WAITING FOR STABILITY");
          waitstatus=true;
          Serial.print("Time to calibrate(sec):");
          Serial.println(StabilityTime/1000);
          lastPercentage=0;
        }else
        {
          int perc=(100*dt)/StabilityTime;
          if((perc-lastPercentage)>=10)
          {
            Serial.print(perc);
            Serial.println("%");
            lastPercentage=perc;
          }
            
            blinkState = !blinkState;
            digitalWrite(LED_PIN2, blinkState);
        }
        
     }else
     {
          Serial.println("Done");
          _calibrate=false;
     }
    }else{
      _mpu[0].Update();
      _mpu[1].Update();

    }

  }

  void Calibrate()
  {
    
    if (!Status()) return;
    Serial.println("Calibrating First MPU");
    _mpu[0].Calibrate();
    Serial.println("Calibrating Second MPU");
    _mpu[1].Calibrate();

    currentMillis = millis();
    _calibrate=true;
    waitstatus=false;
  }
  bool GetHandAngles(float* euler)
  {
      _mpu[0].GetAngles(euler);
      return true;
  }
  bool GetWristAngles(float* euler)
  {
      _mpu[1].GetAngles(euler);
      return true;
  }
  bool GetAngles(float* euler)
  {
      _mpu[0].GetAngles(_e0);
      _mpu[1].GetAngles(_e1);

      euler[0]=_e1[0]-_e0[0];
      euler[1]=_e1[1]-_e0[1];
      euler[2]=_e1[2]-_e0[2];
      return true;
  }

};

class DetectionManager
{
  float currentTilt;
  int currentMillis;

  int state;
  public:
  void Reset(float angle)
  {
    currentMillis=millis();
    currentTilt=angle;
    state=0;
  }
  bool Update(float angle,bool detectOpen)
  {
    bool result=false;
    int dt=(millis()-currentMillis);
    switch(state)
    {
      case 0://actuation state
      if(dt>DETECTION_TIME)
      {
        Serial.print("Check:  ");
        Serial.println(angle);
        if(angle>DETECTION_THRESHOLD && detectOpen || 
           angle<-DETECTION_THRESHOLD && !detectOpen)
        {
          state=1;
        Serial.println("State 1 ");
        }
        currentMillis=millis();
        currentTilt=angle;
      }break;
      case 1://rest state
      if(dt>DETECTION_TIME)
      {
        if(angle<DETECTION_THRESHOLD/2)
        {
          state=2;
          result=true;
        Serial.println("State 2 ");
        }
        currentMillis=millis();
        currentTilt=angle;
      }else 
        state=0;
      
      break;
      case 2: //timeout state
      if(dt>DETECTION_TIMEOUT)
      {
        state=0;
        currentMillis=millis();
        currentTilt=angle;
      }break;
    }
    return result;
  }
};


