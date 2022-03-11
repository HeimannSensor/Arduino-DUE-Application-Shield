// sensordef.h

#ifndef  _SENSORDEF_H
#define  _SENSORDEF_H

// I2C CLOCK
#define CLOCK_SENSOR            1200000

// TIMER (for sample rate)
#define NORM_BW                  68.598
#define SAFETY_FAC               0.90
#define E_BW                     0x0039

// DEVICE ADRESS
#define  SENSOR_ADDRESS          0x1A
#define  EEPROM_ADDRESS          0x1B


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
// VDD
#define E_VDDTH1	              0x0025
#define E_VDDTH2                0x0026
#define E_VDDSCGRAD             0x003E
#define E_VDDSCOFF              0x003F
// PTAT TH
#define E_PTATTH1               0x002C
#define E_PTATTH2               0x002D
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
// GlobalOff/GlobalGain
//#define E_GLOBALOFF             0x00??
#define E_GLOBALGAIN   		    0x0009
// start address for arrays
#define E_VDDCOMPGRAD             0x0040
#define E_VDDCOMPOFF              0x00A0
#define E_THGRAD                  0x0100
#define E_THOFFSET                0x0200
#define E_PIJ                     0x0300
#define E_DEADPIXADR              0x000F
#define E_DEADPIXMASK             0x000E

// other
#define E_NROFDEFPIX            0x000E
#define E_GRADSCALE             0x0008
#define E_TABLENUMBER           0x000C
#define E_EPSILON               0x000D
//#define E_ARRAYTYPE             0x00??
#define E_IP                    0x0012
#define E_IP1                   0x0013
#define E_SUBNET                0x0010
#define E_SUBNET1               0x0011



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
#define TOP_HALF                0x0A
#define BOTTOM_HALF             0x0B




// --- LOOKUP TABLE---


#define ReadToFromTable
#ifdef ReadToFromTable
//#define HTPA32x32dR1L1_6HiGe_Gain3k3
//#define HTPA32x32dR1L2_1SiF5_0_N2
#define HTPA32x32dL2_1HiSiF5_0_Gain3k3      //is used for SensorRev. 1 
//#define HTPA32x32dR1L1k8_0k7HiGe
//#define HTPA32x32dR1L2_1HiSiF5_0_Precise
//#define HTPA32x32dR1L2_1HiSiF5_0_Gain3k3_Extended
//#define HTPA32x32dR1L2_85Hi_Gain3k3
//#define HTPA32x32dR1L3_6HiSi_Rev1_Gain3k3
//#define HTPA32x32dR1L5_0HiGeF7_7_Gain3k3              //
//#define HTPA32x32dR1L5_0HiGeF7_7_Gain3k3_TaExtended
//#define HTPA32x32dR1L7_0HiSi_Gain3k3


#ifdef HTPA32x32dR1L5_0HiGeF7_7_Gain3k3
#define TABLENUMBER   113
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  7
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1

#endif

#ifdef HTPA32x32dR1L5_0HiGeF7_7_Gain3k3_TaExtended
#define TABLENUMBER   113
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  12
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
#endif

#ifdef HTPA32x32dR1L1_6HiGe_Gain3k3
#define TABLENUMBER   119
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  7
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
#endif

#ifdef HTPA32x32dR1L2_1SiF5_0_N2
#define TABLENUMBER   130
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  7
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   192
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
#endif

#ifdef HTPA32x32dL2_1HiSiF5_0_Gain3k3
#define TABLENUMBER   114
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  7
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C  //use REF_CAL=2 here. Table does not match, so GlobalGain ist set to 50 % to compensate this.
#define SensRv 1        //Sensor Revision is set to 1 (Redesign)
#endif

#ifdef HTPA32x32dR1L2_1HiSiF5_0_Gain3k3_Extended
#define TABLENUMBER   114
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  12
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1792
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
#endif

#ifdef HTPA32x32dR1L2_1HiSiF5_0_Precise
#define TABLENUMBER   116
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  22
#define NROFADELEMENTS  1000  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  50    //dK
#define ADEQUIDISTANCE  32    //dig
#define ADEXPBITS   5   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
#endif

#ifdef HTPA32x32dR1L2_85Hi_Gain3k3
#define TABLENUMBER   127
#define PCSCALEVAL    100000000 //327000000000    //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  7
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
#endif

#ifdef HTPA32x32dR1L3_6HiSi_Rev1_Gain3k3
#define TABLENUMBER   117
#define PCSCALEVAL    100000000   //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  7
#define NROFADELEMENTS  1595  //130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
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

#ifdef HTPA32x32dR1L1k8_0k7HiGe
#define TABLENUMBER   115
#define PCSCALEVAL    100000000   //PixelConst scale value for table... lower 'L' for (long)
#define NROFTAELEMENTS  10
#define NROFADELEMENTS  471   //possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
#define TAEQUIDISTANCE  100   //dK
#define ADEQUIDISTANCE  64    //dig
#define ADEXPBITS   6   //2^ADEXPBITS=ADEQUIDISTANCE
#define TABLEOFFSET   1024
#define EQUIADTABLE   //if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
#ifdef EQUIADTABLE
#undef FLOATTABLE
#endif
#define MBITTRIMDefault 0x2C
#define SensRv 1
#endif
#endif






#endif  // _SENSORDEF_H
