#ifndef CarMazeofOz2023_h
#define CarMazeofOz2023_h
#include <Arduino.h>

class carMazeOfOz {
  public:
    carMazeOfOz();
    void setPin();
    void setInterrupt();
    void setMotorLeft(byte speed, bool direction);
    void setMotorRight(byte speed, bool direction);
    void setSpeedLeft(volatile float speed);
    void setSpeedRight(volatile float speed);
    float getSpeedLeft();
    float getSpeedRight();
    float getDistanceHead();
    float getDistanceLeft();
    float getDistanceRight();
    void configureSpeed(volatile float &speedValueLeft, volatile float &speedValueRight);
  private:
    volatile float speedValueLeft, speedValueRight;
};

#endif
