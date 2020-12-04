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

    delay(1000);  // delay for debugging

    imu.log_data();
}
