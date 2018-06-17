#ifndef SERVO_H
#define SERVO_H


class servo
{
public:
    servo();
    servo(int pin);
    //~servo();
    void setPin(int pin);
    float getAbsAngle();

    void init();
    void drive2angle(float angle);
private:
    int m_pin;
    float m_absAngle;
};

#endif // SERVO_H
