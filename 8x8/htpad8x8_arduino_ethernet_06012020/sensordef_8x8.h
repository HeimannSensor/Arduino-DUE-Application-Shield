// sensordef.h

#ifndef  _SENSORDEF_H
#define  _SENSORDEF_H



// DEVICE ADRESS
#define  SENSOR_ADDRESS          0x1A
#define  EEPROM_ADDRESS          0x1B

// I2C CLOCK
#define CLOCK_SENSOR            1200000

// TIMER (for sample rate)
#define NORM_BW                  68.598
#define SAFETY_FAC               0.90
#define E_BW                     0x0039

// TIMER (for sample rate)
#define NORM_BW                  49.72
#define SAFETY_FAC               0.90
#define E_BW                     0x0039


// ---EEPROM---

// EEPROM COMMANDS
#define STANDBY                 0x00
#define ACTIVE                  0x01
#define NORMAL_ERASE            0x02
#define NORMAL_WRITE            0x03
#define BLOCK_ERASE             0x04
#define BLOCK_WRITE             0x05
#define NORMAL_READ             0x06
#define SET_ADDRESS             0x09
#define SET_DATA                0x0A
#define GET_DATA                0x0B



// ID
#define  E_ID1                   0x003A
#define  E_ID2                   0x003B
// calibration settings
#define E_MBIT_CALIB            0x001A
#define E_BIAS_CALIB            0x001B
#define E_CLK_CALIB             0x001C
#define E_BPA_CALIB             0x001D
#define E_PU_CALIB              0x001E
// user settings
#define E_MBIT_USER             0x0020
#define E_BIAS_USER             0x0021
#define E_CLK_USER              0x0022
#define E_BPA_USER              0x0023
#define E_PU_USER               0x0024
// PTAT Gradient
#define E_PTATGR_1              0x0034
#define E_PTATGR_2              0x0035
// PTAT OFFSET
#define E_PTATOFF_1             0x0036
#define E_PTATOFF_2             0x0037
// PixCmin/PixCmax
#define E_PIXCMIN_1             0x0000
#define E_PIXCMIN_2             0x0001
#define E_PIXCMAX_1             0x0002
#define E_PIXCMAX_2             0x0003
// GlobalGain
#define E_GLOBALGAIN   		    0x0009
// start address for arrays
#define E_THGRAD                  0x0040
#define E_THOFFSET                0x0080
#define E_PIJ                     0x00C0

// other
#define E_GRADSCALE             0x0008
#define E_TABLENUMBER           0x000C
#define E_EPSILON               0x000D
//#define E_ARRAYTYPE             0x00??
#define E_IP                    0x0005
#define E_IP1                   0x0006
#define E_SUBNET                0x003E
#define E_SUBNET1               0x003F



// ---Sensor---
// write only
#define CONFIGURATION_REGISTER  0x01
#define TRIM_REGISTER1          0x03
#define TRIM_REGISTER2          0x04
#define TRIM_REGISTER3          0x05
#define TRIM_REGISTER4          0x06
#define TRIM_REGISTER5          0x07
#define TRIM_REGISTER6          0x08
#define TRIM_REGISTER7          0x09
// read only
#define STATUS_REGISTER         0x02
#define ARRAY_DATA              0x0A




// --- LOOKUP TABLE---


#define ReadToFromTable
#ifdef ReadToFromTable
  //#define HTPA32x32dL5_0HiGe
  //#define HTPA32x32dL5_0HiGeF7_7
  //#define HTPA32x32dL2_1HiSi
  #define HTPA32x32dL2_1HiSiF5_0
  //#define HTPA32x32dL2_1HiSiF5_0_withSiFilter 
  //#define HTPA32x32dL3_6HiSi  //same as L2.85 and L7.0, will be measured and adapted in the future
  //#define HTPA32x32dL2_1HiSiDLC
  //#define HTPA32x32dL2_1Si_withSiFilter
  //#define HTPA32x32dR1L7_0HiSi_Gain3k3

    #ifdef HTPA32x32dL5_0HiGe
    #define TABLENUMBER   79
    #define PCSCALEVAL    100000000   //PixelConst scale value for table
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   512
    #endif
  
  #ifdef HTPA32x32dL5_0HiGeF7_7
    #define TABLENUMBER   92
    #define PCSCALEVAL    100000000     //PixelConst scale value for table
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   256
    #endif

    #ifdef HTPA32x32dL2_1HiSi
    #define TABLENUMBER   80
    #define PCSCALEVAL    100000000   //PixelConst scale value for table
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   256
    #endif
  
  #ifdef HTPA32x32dL2_1HiSiF5_0
    #define TABLENUMBER   96
    #define PCSCALEVAL    100000000   //PixelConst scale value for table... lower 'L' for (long)
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471 //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   256
    #define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
    #ifdef EQUIADTABLE
      #undef FLOATTABLE
    #endif
    #endif

    #ifdef HTPA32x32dL2_1HiSiF5_0_withSiFilter
    #define TABLENUMBER   97
    #define PCSCALEVAL    100000000   //PixelConst scale value for table... lower 'L' for (long)
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471 //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   256
    #define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
    #ifdef EQUIADTABLE
      #undef FLOATTABLE
    #endif
    #endif

    #ifdef HTPA32x32dL3_6HiSi
    #define TABLENUMBER   81
    #define PCSCALEVAL    100000000   //PixelConst scale value for table
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   256
    #endif
  
    #ifdef HTPA32x32dL2_1HiSiDLC
    #define TABLENUMBER   83
    #define PCSCALEVAL    100000000   //PixelConst scale value for table
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   512
    #endif

    #ifdef HTPA32x32dL2_1Si_withSiFilter
    #define TABLENUMBER   88
    #define PCSCALEVAL    100000000   //PixelConst scale value for table
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  471
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  8   //dig
    #define ADEXPBITS   3   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   64
    #endif  
  
    #ifdef HTPA32x32dR1L7_0HiSi_Gain3k3
    #define TABLENUMBER   118
    #define PCSCALEVAL    100000000   //PixelConst scale value for table... lower 'L' for (long)
    #define NROFTAELEMENTS  7
    #define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
    #define TAEQUIDISTANCE  100   //dK
    #define ADEQUIDISTANCE  64    //dig
    #define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
    #define TABLEOFFSET   640
    #define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
    #ifdef EQUIADTABLE
      #undef FLOATTABLE
    #endif
    #define MBITTRIMDefault 0x2C
    #define SensRv 1
  #endif  
#endif





#endif  // _SENSORDEF_H
