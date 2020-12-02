#include <stdio.h>
#include "Imu.h"


void IMU::init(int max_attempts)
{
    bool initialised = false;
    int n_attempts = 0;

#ifdef USE_SPI
    SPI_PORT.begin()
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
#endif

    SERIAL_PORT.println("Attempting to initialise icm20948...");
    while (!initialised && n_attempts < max_attempts)  // Attempt to connect
    {
#ifdef USE_SPI
        icm20948.begin(CS_PIN, SPI_PORT, SPI_FREQ);
#else
        icm20948.begin(WIRE_PORT, AD0_VAL);
#endif
        is_connected = true;

        SERIAL_PORT.print(F("Initialisation of icm20948 returned: "));
        SERIAL_PORT.println(icm20948.statusString());
        if (icm20948.status != ICM_20948_Stat_Ok)
        {
            SERIAL_PORT.println("Trying again...");
            delay(500);
            n_attempts += 1;
        }else{
            initialised = true;
        }
    } 

    // Exit if fail to connect
    if (!initialised)
    {
        SERIAL_PORT.println("Fatal: Failed to initialised icm20948. Exiting...");
        Serial.flush();  // Waits for the transmission of outgoing serial data to complete
        exit(0);
    }

    SERIAL_PORT.println("icm20948 connected!");
}


bool IMU::isConnected(void)
{
    return is_connected; 
}


String IMU::str_repr(void)
{
    String tmp = "";
    const char* format;

    tmp += "Scaled. Acc (mg) ";
    tmp += STR_3_AXIS(acc);
    tmp += " Gyr (DPS) ";
    tmp += STR_3_AXIS(gyr);
    tmp += " Mag (uT) ";
    tmp += STR_3_AXIS(mag);
    tmp += " Tmp (C) [ " + String(temp, 2) + " ]";

    return tmp;
}


void IMU::update()
{
    if (icm20948.dataReady())
    {
        icm20948.getAGMT();  // Update the measurements

        // Get accelerometer measurements
        acc.x = icm20948.accX();
        acc.y = icm20948.accY();
        acc.z = icm20948.accZ();

        // Get gyroscope measurements
        gyr.x = icm20948.gyrX();
        gyr.y = icm20948.gyrY();
        gyr.z = icm20948.gyrZ();

        // Get magnetometer measurements
        mag.x = icm20948.magX();
        mag.y = icm20948.magY();
        mag.z = icm20948.magZ();

        // Get temperature measurements
        temp = icm20948.temp();

        last_alive_time = millis();  // Update measurement time

    }
}
