#include "Imu.h"

#define IMU_CONNECT_MAX_ATTEMPS 5

IMU imu;
void setup()
{
    SERIAL_PORT.begin(115200);
    imu.init(IMU_CONNECT_MAX_ATTEMPS);
}
 
void loop()
{
    imu.update();
    SERIAL_PORT.println(imu.str_repr());
}