#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H



class rotaryEncoder
{
public:
    rotaryEncoder();
    rotaryEncoder(int pin);

    //@param: side=0 leftEncoder, side=1 RightEncoder
    //void setEncoder(int side);

    void setPin(int pin);
    void setValue(int value);
    int getValue();

    int getInterval();
    void setInterval(int interval);

    float getAngularVelocity();

    //@param:get the element for two sides,left(0),right(1)
    float getStaticAngularVelocity(int side);
    int getStaticValue(int side);
    void setStaticValue(int value,int side);
    int getStaticInterval(int side);
    void setStaticinterval(int value,int side);

    rotaryEncoder getLeftEncoder();
    rotaryEncoder getRightEncoder();

    void startCounting();
private:
    //static rotaryEncoder *m_rotaryEncoder;
    static rotaryEncoder *leftEncoder;
    static rotaryEncoder *rightEncoder;

     //static void updateEncoders(void);
     static void updateRightEncoders();
     static void updateLeftEncoders();
     void checkSpeedZero();

     //check if this motor is moving forward, used for counting of the rotary encoder
     //@param side
     int isMovingForward(int side);
private:
    int m_pin;
    long int m_value;
    long int lastVal;

    float m_angularVelocity;
    float m_angularVelocity_1;
    float lastAngVel;

    int tInterval;
    long tp1;//time position 1
    long tp2;//time position 2
    int rv1;// record value 1 for angular velocity calculation
    int rv2;// record value 2 for current time


};

#endif // ROTARYENCODER_H
