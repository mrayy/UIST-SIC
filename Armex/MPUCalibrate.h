
#include "Average.h"


#define ModeRange 100
#define StabilityTime 25000 //25 seconds, could be 40secs, only for dmp
#define AllowedRange 150    //this is value is in range round zero that is acceptable and no calibration of that axis is done.
#define GyroStepSize  1
#define AccStepSize  0.2 // this value can be lower to 0.1 to get calibrated data closer to zero but then calibration will take more time
#define MaxIncVal 10

Average<int16_t> Mode(ModeRange);

int16_t getValue(char , char,MPU6050);
void SetXAccelOffset(MPU6050,int16_t);
void SetYAccelOffset(MPU6050,int16_t);
void SetZAccelOffset(MPU6050,int16_t);
void SetXGyroOffset(MPU6050,int16_t);
void SetYGyroOffset(MPU6050,int16_t);
void SetZGyroOffset(MPU6050,int16_t);

//This function attempts to auto calibrate the Accelerometer and Gyroscope and returns true if operation is successful
//This function takes in 3 arguments 1st is sensor ('a'/'A'=Accelerometer and 'g'/'G'=Gyroscope), then 2nd argument is the axis to claibrate ('X'/'x','Y'/'y','Z'/'z') 
//and the last argument this function takes is the mpu6050 object that is created.
bool autocalibrate(char sensor,char axis,MPU6050 mpu){
        float n=1;
        int16_t initalvalue;
        int16_t calibrateValue;
        int16_t firstoutput, secondoutput;
        void (*FuncPointerToSetOffset)(MPU6050, int16_t);
        float stepsize=0.5;
    if(sensor != 'a' && sensor !='A' && sensor != 'g' && sensor !='G'){ //if user enters anything except a A g or G return false
      return false;
    }else if(axis !='x' && axis !='X' && axis !='y' && axis !='Y' && axis !='z' && axis !='Z'){ //if user enters anything except x X y Y z or Z return false
          return false;}
          
        switch(sensor){ //sensor can be accelerometer or gyroscope.
            case 'a'://acceleormeter is a or A
            case 'A':
            stepsize=AccStepSize;
              switch(axis)
              {case 'x': //axis x
              case 'X': //axis x
                  
                  FuncPointerToSetOffset =  SetXAccelOffset;
              break;
                case 'y':
                case 'Y':
               
                    FuncPointerToSetOffset = SetYAccelOffset;
                break;
                
                case'z':
                case'Z':
               
                    FuncPointerToSetOffset =SetZAccelOffset;
                break;
                }
            break;
            case 'g': //gyrometer is g or G
            case 'G': 
            stepsize=GyroStepSize;
               switch(axis)
              {case 'x': //axis x
              case 'X': //axis x
              
                   FuncPointerToSetOffset = SetXGyroOffset;
              break;
                case 'y':
                case 'Y':
                    FuncPointerToSetOffset = SetYGyroOffset;
                break;
                
                case'z':
                case'Z':
                     FuncPointerToSetOffset = SetZGyroOffset;
                break;
                }
                
            break;
            }
      calibrateValue=getValue(sensor,axis, mpu); // sensor (gyro/accl), axis(x y z) and mpu object
     
      if (abs(calibrateValue) <=AllowedRange ) //when you dont need to calibrate after all. when data is near to zero in allowable range (default 150)
       return true;
          
  
      initalvalue=calibrateValue*(-1); //inversion is important here.
      calibrateValue=initalvalue/n;
     FuncPointerToSetOffset(mpu,calibrateValue); //function pointer with function arguments
      firstoutput=getValue(sensor,axis, mpu); // sensor (gyro/accl), axis(x y z) and mpu object
      
   while(n<10){ //here 10 is a random select and this 10 will make sure the loop does not run for more than 50 loops (if step size is 0.2)
     n+=stepsize;
      calibrateValue=initalvalue/n;
      FuncPointerToSetOffset(mpu, calibrateValue);
      secondoutput=getValue(sensor,axis, mpu); // sensor (gyro/accl), axis(x y z) and mpu object
 //here we check wether the value due to previous offset  is closer to zero or the value due to current offset is closer to zero
       //additionally the sign of value is also checked i.e the moment it changes we stop (fro example value goes from +ve to -ve)
      if( abs(firstoutput) < abs(secondoutput) && (firstoutput/abs(firstoutput) * secondoutput/abs(secondoutput))!= -1 ){
        n=n-stepsize;
        FuncPointerToSetOffset(mpu ,initalvalue/n); //we set the previous value because the current value is more far away from zero mark
        break;
        }else{
          firstoutput=secondoutput; //if the current value is lesser than previous value than we check next value.
          }
        }
         
return true;
} //end of function



//the reason for creation of these function was that function pointer was used to point to these function.
//function pointer can not point to a static member function of an object i think. so these function had to be used.
//additionally we can use these functions to add subtract any other constants for offset calibration such as +16384 for Az can be inserted here.

void SetXAccelOffset(MPU6050 mpu ,int16_t value){
  
  mpu.setXAccelOffset(value);
  }

  void SetYAccelOffset(MPU6050 mpu ,int16_t value){
  
  mpu.setYAccelOffset(value);
  }

  void SetZAccelOffset(MPU6050 mpu ,int16_t value){
  
  mpu.setZAccelOffset(value);
  }

  void SetXGyroOffset(MPU6050 mpu ,int16_t value){
    
   mpu.setXGyroOffset(value);
   }

   
  void SetYGyroOffset(MPU6050 mpu ,int16_t value){
    
   mpu.setYGyroOffset(value);
   }

   
  void SetZGyroOffset(MPU6050 mpu ,int16_t value){
    
   mpu.setZGyroOffset(value);
   }

   
//The purpose of this function is to get a value out of a very variable data stream. This function uses
//mode technique to get a very accurate value. the ModeRange is defined as the maximum number of values on which modes is to be applied
//here the user have to pass a or g to indicate sensor, axis required i.e. x y z and finally mpu object
//will retun 0 if arguments are incorrect
int16_t getValue(char sensor, char axis,MPU6050 mpu ){ 

   int ii;
   int16_t dummy1,dummy2,dummy3,dummy4,dummy5; //neglecting values
   int16_t Final;

    if(sensor != 'a' && sensor !='A' && sensor != 'g' && sensor !='G'){ //if user enters anything except a A g or G return false
      return 0;
    }else if(axis !='x' && axis !='X' && axis !='y' && axis !='Y' && axis !='z' && axis !='Z'){ //if user enters anything except x X y Y z or Z return false
          return 0;}

        Serial.println("Begin Sampling");
   for(ii=0; ii<=ModeRange; ii++){
          switch(sensor){ //sensor can be accelerometer or gyroscope.
            case 'a'://acceleormeter is a or A
            case 'A':
              switch(axis){
              
              case 'x': //axis x
              case 'X': //axis x
                   mpu.getMotion6(&Final,&dummy1,&dummy2, &dummy3, &dummy4, &dummy5);
              break;
                case 'y':
                case 'Y':
                    mpu.getMotion6(&dummy1,&Final, &dummy2, &dummy3, &dummy4, &dummy5);
                break;
                
                case'z':
                case'Z':
                     mpu.getMotion6(&dummy1,&dummy2, &Final,&dummy3, &dummy4, &dummy5);
                break;
                }
            break;
            case 'g': //gyrometer is g or G
            case 'G': 
               switch(axis){
              
              case 'x': 
              case 'X': 
                   mpu.getMotion6(&dummy3, &dummy4, &dummy5,&Final,&dummy1,&dummy2);
              break;
                case 'y':
                case 'Y':
                    mpu.getMotion6(&dummy3, &dummy4, &dummy5,&dummy1,&Final, &dummy2);
                break;
                
                case'z':
                case'Z':
                     mpu.getMotion6(&dummy3, &dummy4, &dummy5, &dummy1,&dummy2, &Final);
                break;
                }
                
            break;
            }
        //delay(1);
        delayMicroseconds(500); //the primary reason to use micorseconds here is to reduce time taken for calibration
        Mode.push(Final);
        
        }
  Final=Mode.mode();
        Serial.println("Done Sampling");

  return Final;
    }
