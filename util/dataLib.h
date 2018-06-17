#ifndef DATALIB_H
#define DATALIB_H

extern int Check_Sign;

extern double Visual_Traslation_Matrix[16];
extern double Visual_Eular[3];
extern double Visual_Position[3];

extern float Water_Temperature;
extern float Water_Deep;

extern float IMU_L_Acc[3];//linear acceleration
extern float IMU_L_Vel[3];//linear velocity
extern float IMU_L_Dis[3];//linear displacement
extern float IMU_A_Acc[3];
extern float IMU_A_Vel[3];
extern float IMU_A_Ang[3];

extern float IMU_L_Acc_Offset[3];
extern float IMU_A_Vel_Offset[3];
extern float IMU_A_Ang_Offset[3];
extern float IMU_DATA_PACKAGE[500][9];

extern float IMU_L_Acc_c[3];//linear acceleration after correction
extern float IMU_L_Vel_c[3];//linear velocity after correction
extern float IMU_L_Dis_c[3];//linear displacement after correction
extern float IMU_A_Acc_c[3];
extern float IMU_A_Vel_c[3];
extern float IMU_A_Ang_c[3];


extern double  Data_acc[3];
extern double  Data_vel[3];
extern double Data_dis[3];
extern double Data_eul[3];

extern int Motor_PWM[6];//
extern int Motor_PWM_Control[6];
extern int Motor_PWM_ADD;

//PICTURE PARAMETERES
extern int Code_ID;
extern int Taget_ID;

//TAGET PATAMETERS
extern double Taget_1[6];


//AUV PARAMETERS
extern double AUV_L1;
extern double AUV_L2;
extern double AUV_L3;
extern double AUV_L4;
extern double AUV_fG;
extern double AUV_fb;


void float_to_char(float f,unsigned char *s);

void int_to_char(int t,unsigned char *s);

void char_to_float(unsigned char *s, float* f);

void char_to_int(unsigned char *s,int *t);



#endif // DATALIB_H
