#ifndef IMU_H
#define IMU_H

#include <ICM_20948.h>

#define STR_3_AXIS(MEAS) "[ " + String(MEAS.x, 2) + "," + String(MEAS.y, 2) + "," + String(MEAS.z,2) + " ]"
#define SERIAL_PORT Serial

// #define USE_SPI

// SPI Physical Hookup
#define SPI_PORT SPI  // MOSI
#define SPI_FREQ 10e6 // Overide default SPI frequency
#define CS_PIN 6  // Chip select pin


// I2C Physical Hookup
#define WIRE_PORT Wire  // Desired Wire port.
#define AD0_VAL 1  // The value of the last bit of the I2C address.
                   // On the SparkFun 9DoF IMU breakout the default is 1, and when
                   // the ADR jumper is closed the value becomes 0


typedef struct t_accel
{
    float x;
    float y;
    float z;
} Accelerometer;


typedef struct t_gyro
{
    float x;
    float y;
    float z;
} Gyroscope;


typedef struct t_magn
{
    float x;
    float y;
    float z;
} Magnetometer;

class IMU
{
    private:
        bool is_connected;
        unsigned long last_alive_time;
        Accelerometer acc;
        Gyroscope gyr;
        Magnetometer mag; 
        float temp;  // temperature
#ifdef USE_SPI
        ICM_20948_SPI icm20948;
#else    
        ICM_20948_I2C icm20948;
#endif

    public:
        IMU() : is_connected(false), last_alive_time(0){}  // Constructor
        void init(int max_attempts);  //initialise device
        bool isConnected(void);  // check if device is connected
        String str_repr(void);  // return string representation of device
        void update(void);  // update device state/measurements
};

#endif
