#include <stdio.h>
#include "Imu.h"


static void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    SERIAL_PORT.print("-");
  }else{
    SERIAL_PORT.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      SERIAL_PORT.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    SERIAL_PORT.print(-val, decimals);
  }else{
    SERIAL_PORT.print(val, decimals);
  }
}


void IMU::init(int max_attempts)
{
    bool initialised = false;
    int n_attempts = 0;

#ifdef USE_SPI
    SPI_PORT.begin()
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);  // 400 kHZ I2C
#endif


    /* ---------- Connect to sensor ----------*/
    SERIAL_PORT.println("Attempting to initialise icm20948...");
    while (!initialised && n_attempts < max_attempts)  
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
        SERIAL_PORT.flush();  // Waits for the transmission of outgoing serial data to complete
        exit(-1);
    }

    SERIAL_PORT.println("icm20948 connected!");

    // Here we are doing a SW reset to make sure the device starts in a known state
    icm20948.swReset( );
    if( icm20948.status != ICM_20948_Stat_Ok){
      SERIAL_PORT.print(F("Software Reset returned: "));
      SERIAL_PORT.println(icm20948.statusString());
    }
    delay(250);
    
    // Now wake the sensor up
    icm20948.sleep( false );
    icm20948.lowPower( false );

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    icm20948.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),        
                           ICM_20948_Sample_Mode_Continuous ); 
    if( icm20948.status != ICM_20948_Stat_Ok){
      SERIAL_PORT.print(F("setSampleMode returned: "));
      SERIAL_PORT.println(icm20948.statusString());
    }

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    
    myFSS.a = gpm2;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                            // gpm2
                            // gpm4
                            // gpm8
                            // gpm16
                            
    myFSS.g = dps250;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                            // dps250
                            // dps500
                            // dps1000
                            // dps2000
                            
    icm20948.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );  
    if( icm20948.status != ICM_20948_Stat_Ok){
      SERIAL_PORT.print(F("setFullScale returned: "));
      SERIAL_PORT.println(icm20948.statusString());
    }


    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                            // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                            // acc_d111bw4_n136bw
                                            // acc_d50bw4_n68bw8
                                            // acc_d23bw9_n34bw4
                                            // acc_d11bw5_n17bw
                                            // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                            // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                            // gyr_d196bw6_n229bw8
                                            // gyr_d151bw8_n187bw6
                                            // gyr_d119bw5_n154bw3
                                            // gyr_d51bw2_n73bw3
                                            // gyr_d23bw9_n35bw9
                                            // gyr_d11bw6_n17bw8
                                            // gyr_d5bw7_n8bw9
                                            // gyr_d361bw4_n376bw5
                                            
    icm20948.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
    if( icm20948.status != ICM_20948_Stat_Ok){
      SERIAL_PORT.print(F("setDLPcfg returned: "));
      SERIAL_PORT.println(icm20948.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat = icm20948.enableDLPF( ICM_20948_Internal_Acc, false );
    ICM_20948_Status_e gyrDLPEnableStat = icm20948.enableDLPF( ICM_20948_Internal_Gyr, false );
    SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); 
    SERIAL_PORT.println(icm20948.statusString(accDLPEnableStat));
    SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); 
    SERIAL_PORT.println(icm20948.statusString(gyrDLPEnableStat));

    SERIAL_PORT.println();
    SERIAL_PORT.println(F("Configuration complete!")); 
}


bool IMU::isConnected(void)
{
    return is_connected; 
}


String IMU::str_repr(void)
{
    //TODO: make string representation of device
    String tmp = "";

    return tmp;
}


void IMU::log_data(void)
{
    SERIAL_PORT.print("Scaled. Acc (mg) [ ");
    printFormattedFloat( acc.t, 5, 0 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( acc.x, 5, 2 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( acc.y, 5, 2 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( acc.z, 5, 2 );
    SERIAL_PORT.print(" ], Gyr (DPS) [ ");
    printFormattedFloat( gyr.t, 5, 0 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( gyr.x, 5, 2 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( gyr.y, 5, 2 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( gyr.z, 5, 2 );
    SERIAL_PORT.print(" ], Mag (uT) [ ");
    printFormattedFloat( mag.t, 5, 0 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( mag.x, 5, 2 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( mag.y, 5, 2 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( mag.z, 5, 2 );
    SERIAL_PORT.print(" ], Tmp (C) [ ");
    printFormattedFloat( temp.t, 5, 0 );
    SERIAL_PORT.print(", ");
    printFormattedFloat( temp.temp, 5, 2 );
    SERIAL_PORT.print(" ]");
    SERIAL_PORT.println();
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
        acc.t =  millis();

        // Get gyroscope measurements
        gyr.x = icm20948.gyrX();
        gyr.y = icm20948.gyrY();
        gyr.z = icm20948.gyrZ();
        gyr.t = millis();

        // Get magnetometer measurements
        mag.x = icm20948.magX();
        mag.y = icm20948.magY();
        mag.z = icm20948.magZ();
        mag.t = millis();

        // Get temperature measurements
        temp.temp = icm20948.temp();
        temp.t = millis();
    }
}
