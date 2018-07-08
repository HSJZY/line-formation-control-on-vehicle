#ifndef GLOBALSETTINGS_H
#define GLOBALSETTINGS_H

//global pin settings
#define LeftMotorIn1 2
#define LeftMotorIn2 3
#define RightMotorIn3 1
#define RightMotorIn4 0

#define LeftEncoderPin 4
#define RightEncoderPin 5

#define ServoSignalPin 11

//unicycle model settings
#define WheelRadius 0.034
#define WheelBaseLength 0.13;// !!!these two number need to be justified after measurment


#define robot_id 1

//运动方向定义
#define LeftSide 0
#define RightSide 1
#define ForwardSide 2
#define Backward_side 3
#define Rotate_side 4


//constant value
#define PI 3.1415926
#define StartDutyCycle 0.2

#endif // GLOBALSETTINGS_H
