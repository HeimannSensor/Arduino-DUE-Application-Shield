#include <Wire.h>
#include "sensordef_8x8.h"
#include "lookuptable.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <DueTimer.h>

// SENSOR CHARACTERISTICS
struct characteristics {
  uint8_t number_row;    // number of raws
  uint8_t number_col;    // number of column
  uint8_t number_blocks; // number of blocks (top + down)
  uint8_t number_pixel;  // number of pixel
};
characteristics sensor = {8, 8, 1, 64};

// EEPROM DATA
uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, gradscale, epsilon;
uint8_t mbit_user, bias_user, clk_user, bpa_user, pu_user;
uint16_t tablenumber, globalgain;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;
int16_t thgrad[8][8];
int16_t thoffset[8][8];
uint16_t pij[8][8];
uint32_t pixcij_uint32[8][8];
uint32_t id;

// SENSOR DAta
uint8_t data_array[130];
uint8_t electrical_offset[130];
uint16_t eloffset[8][8];
uint16_t ptat, vdd;
uint16_t data_pixel[8][8];
uint8_t statusreg;


// CALCULATED VALUES
uint16_t ambient_temperature;
int32_t vij_comp_int32[8][8];
int32_t vij_comp_s_int32[8][8];
int32_t vij_pixc_int32[8][8];
uint32_t temp_pix_uint32[8][8];

// OTHER
uint32_t gradscale_div;
char var = 'm';
uint16_t timer_duration;




/********************************************************************
   Function:        void setup()

   Description:     setup before main loop

   Dependencies:
 *******************************************************************/
void setup() {

  // begin serial communication
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // begin I2C communication ( 1000kHz - sensor)
  Serial.print("search device");
  uint8_t error;
  while (error != 0) {
    Wire1.end();
    delay(2000);
    Wire1.begin();
    Wire1.beginTransmission(SENSOR_ADDRESS);
    error = Wire1.endTransmission();
    Serial.print(".");
  }


  Wire1.setClock(CLOCK_SENSOR);
  // HINT: To increase the frame rate, here the I2C clock is higher than 1MHz from datasheet. If this causes any problems, set to the datasheet value.


  Serial.print("\nread eeprom");
  read_complete_eeprom();

  Serial.print("\nwake up sensor");
  // to wake up sensor set configuration register to 0x01
  // |  7  |  6  |  5  |  4  |   3   |   2   |   1   |    0   |
  // |          RFU          | Start |  VDD  | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |   0   |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);

  Serial.print("\ninitialization");
  write_calibration_settings_to_sensor();

  Serial.print("\nstart sensor");
  // to start sensor set configuration register to 0x09
  // |  7  |  6  |  5  |  4  |   3   |   2   |   1   |    0   |
  // |          RFU          | Start |  VDD  | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |   0   |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);

  // other calculations before main loop
  gradscale_div = pow(2, gradscale);
  calculate_pixcij();  
  timer_duration = calc_timer_duration(bw, clk_calib, mbit_calib);



  // ERROR TABLENUMBER
  if (tablenumber != TABLENUMBER) {
    Serial.print("\n\nHINT:\tConnected Sensor does not match the selected look up table.");
    Serial.print("\n\tThe calculated temperatures could be wrong!");
    Serial.print("\n\tChange device in sensordef_16x16.h to sensor with tablenumber ");
    Serial.print(tablenumber);
  }
  // ERROR BUFFER LENGTH
  if (BUFFER_LENGTH < 258){
    Serial.print("\n\nHINT:\tBUFFER_LENGTH in Wire.h library is not 258 or higher.");
    Serial.print("\n\tThe calculated temperatures could be wrong!");
    Serial.print("\n\tChange BUFFER_LENGTH in wire.h to 258 or higher");
  }
  
}







/********************************************************************
   Function:        void loop()

   Description:     main loop of an arduino programm

   Dependencies:
 *******************************************************************/
void loop() {



  switch (var) {

    case 'm':
      // ---MENU---
      Serial.print("\n\nmenu\n\n");
      Serial.print("a... read eeprom (HEX)\n");
      Serial.print("b... read eeprom (value)\n");
      Serial.print("c... read pixel temps\n");
      Serial.print("d... print all calc steps\n");
      break;

    case 'a':
      print_eeprom_hex();
      break;


    case 'b':
      print_eeprom_value();
      break;

    case 'c':
      // --- PIXEL TEMPS WITHOUT CALC STEPS
      read_pixel_data();
      sort_data();
      calculate_pixel_temp();
      print_pixel_temps();
      break;


    case 'd':
      // --- PIXEL TEMPS WITH CALC STEPS
      read_pixel_data();
      sort_data();
      calculate_pixel_temp();
      print_calc_steps();
      break;


  }

  var = Serial.read();


}





/********************************************************************
   Function:      calc_timer_duration(float bw, uint8_t clk, uint8_t mbit)

   Description:   calculate the duration of the timer which reads the sensor blocks

   Dependencies:  band width (bw)
                  clock (clk)
                  adc resolution (mbit)
 *******************************************************************/
word calc_timer_duration(float bw, uint8_t clk, uint8_t mbit) {
  float Fclk_float = 12000000 / 63 * clk + 1000000;    // calc clk in Hz
  float a, b, c;
  uint16_t calculated_timer_duration;
  a = 1 / NORM_BW;
  b = 32 * (pow(2, mbit & 0b00001111) + 4) / Fclk_float;
  c = b / a;
  c = c / bw;
  c = SAFETY_FAC * c;

  calculated_timer_duration = c * 1000000; // c in s | timer_duration in µs

  return calculated_timer_duration;
}




/********************************************************************
   Function:        void read_pixel_data()

   Description:     read all blocks and el. Offset

   Dependencies:
 *******************************************************************/
void read_pixel_data() {


  // change block in configuration register (to data)
  // |  7  |  6  |  5  |  4  |   3   |   2   |   1   |    0   |
  // |          RFU          | Start |  VDD  | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |   0   |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }

  read_sensor_register( ARRAY_DATA, (uint8_t*)&data_array, 130);


  // change block in configuration register (to el.offset)
  // |  7  |  6  |  5  |  4  |   3   |   2   |   1   |    0   |
  // |          RFU          | Start |  VDD  | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |   1   |   1   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x0B + 0x04);
  
  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }

  read_sensor_register( ARRAY_DATA, (uint8_t*)&electrical_offset, 130);

}





/********************************************************************
   Function:        calculate_pixel_temp()

   Description:     compensate thermal and electrical offset and multiply sensitivity coeff
                    look for the correct temp in lookup table

   Dependencies:
 *******************************************************************/
void calculate_pixel_temp() {

  int64_t vij_pixc_and_pcscaleval;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;

  // find column of lookup table
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (ambient_temperature > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = ambient_temperature - XTATemps[table_col];
  ydist = (int32_t)ADEQUIDISTANCE;




  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {

      // --- THERMAL OFFSET ---
      // compensate thermal drifts (see datasheet, chapter: 10.2 Thermal Offset)
      vij_comp_int32[m][n] = (data_pixel[m][n] - (thgrad[m][n] * ambient_temperature) / gradscale_div - thoffset[m][n]);


      // --- ELECTRICAL OFFSET
      // compensate electrical offset (see datasheet, chapter: 10.3 Electrical Offset)
      vij_comp_s_int32[m][n] = vij_comp_int32[m][n] - eloffset[m][n];


      // --- SENSITIVITY ---
      // multiply sensitivity coeff for each pixel (see datasheet, chapter: 10.4 Object Temperature)
      vij_pixc_and_pcscaleval = (int64_t)vij_comp_s_int32[m][n] * (int64_t)PCSCALEVAL;
      vij_pixc_int32[m][n] =  (int32_t)(vij_pixc_and_pcscaleval / (int64_t)pixcij_uint32[m][n]);

      // --- LOOKUPtaBLE ---
      // find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter: 10.5 Look-up table)
      table_row = vij_pixc_int32[m][n] + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      temp_pix_uint32[m][n] = (uint32_t)((vy - vx) * ((int32_t)(vij_pixc_int32[m][n] + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

    }
  }

}



/********************************************************************
   Function:        void sort_data()

   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd

   Dependencies:
 *******************************************************************/
void sort_data() {

  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {

      // --- PIXEL DAta ---
      data_pixel[m][n] = data_array[2 * (n + m * 8) + 2] << 8 | data_array[2 * (n + m * 8) + 3];

      // --- ELECTRICAL OFFSET ---
      eloffset[m][n] = electrical_offset[2 * (n + m * 8) + 2] << 8 | electrical_offset[2 * (n + m * 8) + 3];

    }
  }

  ptat = data_array[0] << 8  | data_array[1];
  vdd = electrical_offset[0] << 8  | electrical_offset[1];

  // calculate ta (datasheet, chapter: 10.1 Ambient Temperature )
  ambient_temperature = ptat * ptatgr_float + ptatoff_float;

}





/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)

   Description:     read sensor register

   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n) {

  Wire1.requestFrom(SENSOR_ADDRESS, n, addr, 1, 1);
  while (Wire1.available()) {
    *dest++  = Wire1.read();
  }
  Wire1.endTransmission();
}




/********************************************************************
   Function:        void write_SENDOR_byte(int deviceaddress, unsigned int eeaddress )

   Description:     write sensor register

   Dependencies:    device address (deviceaddress)
                    register address (registeraddress)
                    input byte (input)
 *******************************************************************/
byte write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input) {

  Wire1.beginTransmission(deviceaddress);
  Wire1.write(registeraddress);
  Wire1.write(input);
  Wire1.endTransmission();

}


/********************************************************************
   Function:        void write_calibration_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_calibration_settings_to_sensor() {

  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_calib);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_calib);
  delay(5);
}



/********************************************************************
   Function:        void write_user_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_user_settings_to_sensor() {

  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_user);
  delay(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_user);
  delay(5);
}



/********************************************************************
   Function:        void calculate_pixcij()

   Description:     calculate sensitivity coefficients for each pixel

   Dependencies:    minimum sensitivity coefficient (pixcmin),
                    maximum sensitivity coefficient (pixcmax),
                    sensitivity coefficient (pij[32][32]),
                    emissivity factor (epsilon),
                    factor for fine tuning of the sensitivity (globalgain)
 *******************************************************************/
void calculate_pixcij() {


  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {

      // calc sensitivity coefficients (see datasheet, chapter: 11.5 Object Temperature)
      pixcij_uint32[m][n] = (uint32_t)pixcmax - (uint32_t)pixcmin;
      pixcij_uint32[m][n] = pixcij_uint32[m][n] / 65535;
      pixcij_uint32[m][n] = pixcij_uint32[m][n] * pij[m][n];
      pixcij_uint32[m][n] = pixcij_uint32[m][n] + pixcmin;
      pixcij_uint32[m][n] = pixcij_uint32[m][n] * 1.0 * epsilon / 100;
      pixcij_uint32[m][n] = pixcij_uint32[m][n] * 1.0 * globalgain / 10000;

    }
  }

}









/********************************************************************
   Function:        read_complete_eeprom();

   Description:     read complete eeprom and save in global variables

   Dependencies:
 *******************************************************************/
void read_complete_eeprom() {
  uint16_t b[2];
  bw = eeprom_read_routine(E_BW) / 100;
  id = eeprom_read_routine(E_ID2) << 16 | eeprom_read_routine(E_ID1);
  mbit_calib = (uint8_t)eeprom_read_routine(E_MBIT_CALIB);
  bias_calib = (uint8_t)eeprom_read_routine(E_BIAS_CALIB);
  clk_calib = (uint8_t)eeprom_read_routine(E_CLK_CALIB);
  bpa_calib = (uint8_t)eeprom_read_routine(E_BPA_CALIB);
  pu_calib = (uint8_t)eeprom_read_routine(E_PU_CALIB);
  mbit_user = (uint8_t)eeprom_read_routine(E_MBIT_USER);
  bias_user = (uint8_t)eeprom_read_routine(E_BIAS_USER);
  clk_user = (uint8_t)eeprom_read_routine(E_CLK_USER);
  bpa_user = (uint8_t)eeprom_read_routine(E_BPA_USER);
  pu_user = (uint8_t)eeprom_read_routine(E_PU_USER);
  tablenumber = (uint8_t)eeprom_read_routine(E_TABLENUMBER);
  gradscale = (uint8_t)eeprom_read_routine(E_GRADSCALE);
  epsilon = (uint8_t)eeprom_read_routine(E_EPSILON);
  globalgain = eeprom_read_routine(E_GLOBALGAIN);
  b[0] =  eeprom_read_routine(E_PTATGR_1);
  b[1] =  eeprom_read_routine(E_PTATGR_2);
  ptatgr_float = *(float*)b;
  b[0] =  eeprom_read_routine(E_PTATOFF_1);
  b[1] =  eeprom_read_routine(E_PTATOFF_2);
  ptatoff_float = *(float*)b;
  b[0] =  eeprom_read_routine(E_PIXCMIN_1);
  b[1] =  eeprom_read_routine(E_PIXCMIN_2);
  pixcmin = *(float*)b;
  b[0] =  eeprom_read_routine(E_PIXCMAX_1);
  b[1] =  eeprom_read_routine(E_PIXCMAX_2);
  pixcmax = *(float*)b;


  // --- Thgrad_ij ---
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      thgrad[m][n] = eeprom_read_routine(E_THGRAD + n + m * 8);
    }
  }

  // --- Thoffset_ij ---
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      thoffset[m][n] = eeprom_read_routine(E_THOFFSET + n + m * 8);
    }
  }

  // --- P_ij ---
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      pij[m][n] = eeprom_read_routine(E_PIJ + n + m * 8);
    }
  }



}









/********************************************************************
   Function:        read_eeprom_routine(uint16_t addr)

   Description:     read eeprom register (see datasheet, chp.: 10.10 I2C Example Sequence – EEPROM Sequential Read )

   Dependencies:    addr... eeprom register adress
 *******************************************************************/
word eeprom_read_routine(uint8_t addr) {

  // EEPROM_ACTIVE
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write(ACTIVE);
  Wire1.endTransmission();

  // SET_ADDR
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(SET_ADDRESS));
  Wire1.write((int)(addr));
  Wire1.endTransmission();


  // NORMAL_READ
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(NORMAL_READ));
  Wire1.endTransmission();

  // GET_DAta
  uint8_t eeprom_data_buffer[2] = {0};
  receive_eeprom_data(GET_DATA, (uint8_t*)eeprom_data_buffer, 2);

  uint16_t data_word = eeprom_data_buffer[1] << 8 | eeprom_data_buffer[0];
  return data_word;

}



/********************************************************************
   Function:        void read_EEPROM_byte( uint16_t addr, uint8_t *dest, uint16_t n)

   Description:     receive (16 bit) eeprom data

   Dependencies:    cmd... receive command (cmd)
                    dest... data buffer(dest)
                    n... number of bytes (2)
 *******************************************************************/
void receive_eeprom_data( uint16_t cmd, uint8_t *dest, uint16_t n) {

  Wire1.requestFrom(EEPROM_ADDRESS, n, cmd, 1, 1);
  while (Wire1.available()) {
    *dest++  = Wire1.read();
  }
  Wire1.endTransmission();
}



/********************************************************************
   Function:        write_eeprom_routine(uint16_t addr, uint16_t value)

   Description:     read eeprom register (see datasheet, chp.: 10.07 I2C Example Sequence – EEPROM Erase/Write )

   Dependencies:    addr... eeprom register adress
                    value... value to eeprom
 *******************************************************************/
word write_eeprom_routine(uint8_t addr, uint16_t value) {

  // EEPROM_ACTIVE
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write(ACTIVE);
  Wire1.endTransmission();
  delay(6);

  // SET_ADDR
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(SET_ADDRESS));
  Wire1.write((int)(addr));
  Wire1.endTransmission();

  // NORMAL_ERASE
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(NORMAL_ERASE));
  Wire1.endTransmission();
  delay(6);

  // SET_DATA
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(SET_DATA));
  Wire1.write((int)(value & 0xFF)); // LSB
  Wire1.write((int)(value >> 8));  // MSB

  Wire1.endTransmission();

  // NORMAL_WRITE
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(NORMAL_WRITE));
  Wire1.endTransmission();
  delay(6);


}



































/********************************************************************

   END OF READ-OUT PROGRAMM

   the following functions are used to chat with the Serial Monitor:

 *******************************************************************/


























/********************************************************************
   Function:        print_calc_steps()

   Description:

   Dependencies:
 *******************************************************************/
void print_calc_steps() {

  Serial.print("\n\n\n---PRINT ALL STEPS---");

  Serial.print("\n\n\n1) read row pixel data (V_ij):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n2) read electrical offset (elOffset_ij):\n\n");
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(eloffset[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
  
  Serial.print("\n\n\n3) calculate ambient temperature (Ta):\n\n");
  Serial.print("Ta = ");
  Serial.print(ptat);
  Serial.print(" * ");
  Serial.print(ptatgr_float);
  Serial.print(" + ");
  Serial.print(ptatoff_float);
  Serial.print(" = ");
  Serial.print(ambient_temperature);
  Serial.print(" (Value is given in dK)");


  Serial.print("\n\n\n4) compensate thermal offset (V_ij_comp):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(vij_comp_int32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n5) compensate electrical offset (V_ij_comp_s):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(vij_comp_s_int32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n6) calculate sensitivity coefficients (pixc_ij):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(pixcij_uint32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
    
  Serial.print("\n\n\n7) multiply scaling coeff and sensitivity coeff to compensated pixel voltages (V_ij_pixc):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(vij_pixc_int32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n8) calcluate final pixel temperature (in dK) with lookup table and bilinear interpolation:\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(temp_pix_uint32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }


  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");
}


/********************************************************************
   Function:        print_pixel_temps()

   Description:

   Dependencies:
 *******************************************************************/
void print_pixel_temps() {


  Serial.print("\n\n\n---PRINT PIXEL TEMPERATURE---\n");

  Serial.print("\n\npixel temperature (dK)\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(temp_pix_uint32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");
}



/********************************************************************
   Function:        print_eeprom_value()

   Description:

   Dependencies:
 *******************************************************************/
void print_eeprom_value() {

  
  Serial.print("\n\n\n---PRINT EEPROM (VALUE)---\n");
  Serial.print("\n\nEEPROM 8x8\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    Serial.print("- ");
  }
  // HEADER
  // 1st line
  Serial.print("\n");
  Serial.print("HEADER\t0x00");
  Serial.print("\t|\t");
  Serial.print(pixcmin);
  Serial.print("\t");
  Serial.print(pixcmax);
  Serial.print("\t\t\t\t\t");
  Serial.print(gradscale);
  Serial.print("\t");
  Serial.print(globalgain);
  Serial.print("\t\t\t");
  Serial.print(tablenumber);
  Serial.print("\t");
  Serial.print(epsilon);
  // 2nd line
  Serial.print("\n");
  Serial.print("HEADER\t0x10");
  Serial.print("\t|\t\t\t\t\t\t\t\t\t\t\t");
  Serial.print(mbit_calib);
  Serial.print("\t");
  Serial.print(bias_calib);
  Serial.print("\t");
  Serial.print(clk_calib);
  Serial.print("\t");
  Serial.print(bpa_calib);
  Serial.print("\t");
  Serial.print(pu_calib);
  // 3rd line
  Serial.print("\n");
  Serial.print("HEADER\t0x20");
  Serial.print("\t|\t");
  Serial.print(mbit_user);
  Serial.print("\t");
  Serial.print(bias_user);
  Serial.print("\t");
  Serial.print(clk_user);
  Serial.print("\t");
  Serial.print(bpa_user);
  Serial.print("\t");
  Serial.print(pu_user);
  // 4th line
  Serial.print("\n");
  Serial.print("HEADER\t0x30");
  Serial.print("\t|\t\t\t\t\t    ");
  Serial.print(ptatgr_float);
  Serial.print("\t\t   ");
  Serial.print(ptatoff_float);
  Serial.print("\t\t   ");
  Serial.print(id);

  // OTHER
  for (int i = 0x40; i <= 0xFF; i++) {

    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < 0x80) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0xC0) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0xFF) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }
    else {
      Serial.print("\t");
    }
    
    if (i >= 0x40 && i < 0x0C0) {
      Serial.print((int16_t)eeprom_read_routine(i));
    }
    else {
      Serial.print(eeprom_read_routine(i));
    }
    
  }

  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");
}





/********************************************************************
   Function:        print_eeprom_hex()

   Description:

   Dependencies:
 *******************************************************************/
void print_eeprom_hex() {

  Serial.print("\n\n\n---PRINT EEPROM (HEX)---\n");
  Serial.print("\n\nEEPROM 8x8\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    Serial.print("- ");
  }

  for (int i = 0; i <= 0xFF; i++) {


    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < 0x40) {
        Serial.print("HEADER\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x80) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0xC0) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0xFF) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }
    else {
      Serial.print("\t");
    }

    Serial.print(eeprom_read_routine(i), HEX);

  }
  
  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");
}
