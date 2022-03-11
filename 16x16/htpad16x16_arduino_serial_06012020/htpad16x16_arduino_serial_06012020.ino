#include <Wire.h>
#include "sensordef_16x16.h"
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
characteristics sensor = {16, 16, 4, 256};

// EEPROM DATA
uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon;
uint8_t mbit_user, bias_user, clk_user, bpa_user, pu_user, deadpixadr, deadpixmask;
uint16_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, globalgain;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;
int16_t thgrad[16][16];
int16_t thoffset[16][16];
int16_t vddcompgrad[8][16];
int16_t vddcompoff[8][16];
uint16_t pij[16][16];
uint32_t pixcij_uint32[16][16];
uint32_t id;

// SENSOR DATA
uint8_t data_top_block0[130], data_top_block1[130];
uint8_t data_bottom_block0[130], data_bottom_block1[130];
uint8_t electrical_offset_top[130], electrical_offset_bottom[130];
uint16_t eloffset[8][16];
uint16_t ptat_top_block0, ptat_top_block1, ptat_top_block2, ptat_top_block3;
uint16_t ptat_bottom_block0, ptat_bottom_block1, ptat_bottom_block2, ptat_bottom_block3;
uint16_t vdd_top_block0, vdd_top_block1, vdd_bottom_block0, vdd_bottom_block1;
uint16_t data_pixel[16][16];
uint8_t statusreg;

// CALCULATED VALUES
uint16_t ptat_av_uint16;
uint16_t vdd_av_uint16;
uint16_t ambient_temperature;
int32_t vij_pixc_int32[16][16];
uint32_t temp_pix_uint32[16][16];
int32_t vij_comp_int32[16][16];
int32_t vij_comp_s_int32[16][16];
int32_t vij_vddcomp_int32[16][16];

// OTHER
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;
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
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);

  Serial.print("\ninitialization");
  write_calibration_settings_to_sensor();

  Serial.print("\nstart sensor");
  // to start sensor set configuration register to 0x09
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);


  // other calculations before main loop
  gradscale_div = pow(2, gradscale);
  vddscgrad_div = pow(2, vddscgrad);
  vddscoff_div = pow(2, vddscoff);
  calculate_pixcij();
  timer_duration = calc_timer_duration(bw, clk_calib, mbit_calib);

  // ERROR TABLENUMBER
  if (tablenumber != TABLENUMBER) {
    Serial.print("\n\nHINT:\tConnected sensor does not match the selected look up table.");
    Serial.print("\n\tThe calculated temperatures could be wrong!");
    Serial.print("\n\tChange device in sensordef_16x16.h to sensor with tablenumber ");
    Serial.print(tablenumber);
  }

  // ERROR BUFFER LENGTH
  if (BUFFER_LENGTH < 258) {
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
      pixel_masking();
      print_pixel_temps();
      break;


    case 'd':
      // --- PIXEL TEMPS WITH CALC STEPS
      read_pixel_data();
      sort_data();
      calculate_pixel_temp();
      print_calc_steps();
      pixel_masking();
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

   Description:     read 2 complete pictures (first with ptat, second with vdd) and electrical Offset

   Dependencies:
 *******************************************************************/
void read_pixel_data() {

  // --- BLOCK 0 with PTAT ---

  // change block in configuration register (to block0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 130);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block0, 130);



  // --- BLOCK 1 with PTAT ---

  // change block in configuration register (to block0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  1  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x19 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block1, 130);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block1, 130);



  // SAVE PTAT
  ptat_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
  ptat_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
  ptat_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
  ptat_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];





  // --- BLOCK 0 with VDD ---

  // change block in configuration register (to block0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09 + 0x04);

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 130);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block0, 130);



  // --- BLOCK 1 with VDD ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  1  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x19 + 0x04);

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block1, 130);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block1, 130);



  // SAVE VDD
  vdd_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
  vdd_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
  vdd_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
  vdd_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];



  // --- EL.OFFSET ---

  // change block in configuration register (to block0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x0B );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&electrical_offset_top, 130);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&electrical_offset_bottom, 130);


}




/********************************************************************
   Function:        void pixel_masking()

   Description:     repair dead pixel by using the average of the neighbors

   Dependencies:    number of defect pixel (nrofdefpix),
                    dead pixel address (deadpixadr),
                    dead pixel mask (deadpixmask),
                    pixel temperatures (temp_pix_uint32[32][32])
 *******************************************************************/
void pixel_masking() {


  uint8_t number_neighbours;
  uint32_t temp_defpix;


  if (nrofdefpix > 0) {
    number_neighbours = 0;
    temp_defpix = 0;

    // top half

    if (deadpixadr < 128) {

      if ( (deadpixmask & 1 )  == 1) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) - 1][(deadpixadr % 16)];
      }


      if ( (deadpixmask & 2 )  == 2 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) - 1][(deadpixadr % 16) + 1];
      }

      if ( (deadpixmask & 4 )  == 4 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4)][(deadpixadr % 16) + 1];
      }

      if ( (deadpixmask & 8 )  == 8 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) + 1][(deadpixadr % 16) + 1];
      }

      if ( (deadpixmask & 16 )  == 16 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) + 1][(deadpixadr % 16)];
      }

      if ( (deadpixmask & 32 )  == 32 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) + 1][(deadpixadr % 16) - 1];
      }

      if ( (deadpixmask & 64 )  == 64 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4)][(deadpixadr % 16) - 1];
      }

      if ( (deadpixmask & 128 )  == 128 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) - 1][(deadpixadr % 16) - 1];
      }

    }

    // bottom half
    else {

      if ( (deadpixmask & 1 << 0 )  == 1 << 0) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) + 1][(deadpixadr % 16)];
      }

      if ( (deadpixmask & 2 )  == 2 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) + 1][(deadpixadr % 16) + 1];
      }

      if ( (deadpixmask & 4 )  == 4 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4)][(deadpixadr % 16) + 1];
      }

      if ( (deadpixmask & 8 )  == 8 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) - 1][(deadpixadr % 16) + 1];
      }

      if ( (deadpixmask & 16 )  == 16 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) - 1][(deadpixadr % 16)];
      }

      if ( (deadpixmask & 32 )  == 32 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) - 1][(deadpixadr % 16) - 1];
      }

      if ( (deadpixmask & 64 )  == 64 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4)][(deadpixadr % 16) - 1];
      }

      if ( (deadpixmask & 128 )  == 128 ) {
        number_neighbours++;
        temp_defpix = temp_defpix + temp_pix_uint32[(deadpixadr >> 4) + 1][(deadpixadr % 16) - 1];
      }
    }

    temp_defpix = temp_defpix / number_neighbours;
    temp_pix_uint32[deadpixadr >> 4][deadpixadr % 16] = temp_defpix;

  }


}


/********************************************************************
   Function:        calculate_pixel_temp()

   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table

   Dependencies:
 *******************************************************************/
void calculate_pixel_temp() {

  int64_t vij_pixc_and_pcscaleval;
  int64_t vdd_calc_steps;
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
      // compensate thermal drifts (see datasheet, chapter: 11.2 Thermal Offset)
      vij_comp_int32[m][n] = data_pixel[m][n] - (thgrad[m][n] * ptat_av_uint16) / gradscale_div - thoffset[m][n];


      // --- ELECTRICAL OFFSET
      // compensate electrical offset (see datasheet, chapter: 11.3 Electrical Offset)
      // top half
      if (m < sensor.number_row / 2) {
        vij_comp_s_int32[m][n] = vij_comp_int32[m][n] - eloffset[m % 4][n];
      }
      // bottom half
      else {
        vij_comp_s_int32[m][n] = vij_comp_int32[m][n] - eloffset[m % 4 + 4][n];
      }


      // --- VDD ---
      // select VddCompGrad and VddCompOff for pixel m,n:
      // top half
      if (m < sensor.number_row / 2) {
        vddcompgrad_n = vddcompgrad[m % 4][n];
        vddcompoff_n = vddcompoff[m % 4][n];
      }
      // bottom half
      else {
        vddcompgrad_n = vddcompgrad[m % 4 + 4][n];
        vddcompoff_n = vddcompoff[m % 4 + 4][n];
      }
      // compensate vdd (see datasheet, chapter: 11.4 Vdd Compensation)
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      vij_vddcomp_int32[m][n] = vij_comp_s_int32[m][n] - vdd_calc_steps;


      // --- SENSITIVITY ---
      // multiply sensitivity coeff for each pixel (see datasheet, chapter: 11.5 Object Temperature)
      vij_pixc_and_pcscaleval = (int64_t)vij_vddcomp_int32[m][n] * (int64_t)PCSCALEVAL;
      vij_pixc_int32[m][n] =  (int32_t)(vij_pixc_and_pcscaleval / (int64_t)pixcij_uint32[m][n]);


      // --- LOOKUPTABLE ---
      // find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter: 11.7 Look-up table)
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

  uint32_t sum;


  for (int n = 0; n < sensor.number_col; n++) {


    // --- PIXEL DATA TOP HALF ---
    // block 0
    data_pixel[0][n] = data_top_block0[2 * n + 2] << 8 | data_top_block0[2 * n + 3];
    data_pixel[1][n] = data_top_block0[2 * (n + 16) + 2] << 8 | data_top_block0[2 * (n + 16) + 3];
    data_pixel[2][n] = data_top_block0[2 * (n + 32) + 2] << 8 | data_top_block0[2 * (n + 32) + 3];
    data_pixel[3][n] = data_top_block0[2 * (n + 48) + 2] << 8 | data_top_block0[2 * (n + 48) + 3];

    // block 1
    data_pixel[4][n] = data_top_block1[2 * n + 2] << 8 | data_top_block1[2 * n + 3];
    data_pixel[5][n] = data_top_block1[2 * (n + 16) + 2] << 8 | data_top_block1[2 * (n + 16) + 3];
    data_pixel[6][n] = data_top_block1[2 * (n + 32) + 2] << 8 | data_top_block1[2 * (n + 32) + 3];
    data_pixel[7][n] = data_top_block1[2 * (n + 48) + 2] << 8 | data_top_block1[2 * (n + 48) + 3];


    // --- PIXEL DATA BOTTOM HALF ---
    // block 1
    data_pixel[8][n] = data_bottom_block1[2 * (n + 48) + 2] << 8 | data_bottom_block1[2 * (n + 48) + 3];
    data_pixel[9][n] = data_bottom_block1[2 * (n + 32) + 2] << 8 | data_bottom_block1[2 * (n + 32) + 3];
    data_pixel[10][n] = data_bottom_block1[2 * (n + 16) + 2] << 8 | data_bottom_block1[2 * (n + 16) + 3];
    data_pixel[11][n] = data_bottom_block1[2 * n + 2] << 8 | data_bottom_block1[2 * n + 3];

    // block 0
    data_pixel[12][n] = data_bottom_block0[2 * (n + 48) + 2] << 8 | data_bottom_block0[2 * (n + 48) + 3];
    data_pixel[13][n] = data_bottom_block0[2 * (n + 32) + 2] << 8 | data_bottom_block0[2 * (n + 32) + 3];
    data_pixel[14][n] = data_bottom_block0[2 * (n + 16) + 2] << 8 | data_bottom_block0[2 * (n + 16) + 3];
    data_pixel[15][n] = data_bottom_block0[2 * n + 2] << 8 | data_bottom_block0[2 * n + 3];


    // --- ELECTRICAL OFFSET ---
    // top half
    eloffset[0][n] = electrical_offset_top[2 * n + 2] << 8 | electrical_offset_top[2 * n + 3];
    eloffset[1][n] = electrical_offset_top[2 * (n + 16) + 2] << 8 | electrical_offset_top[2 * (n + 16) + 3];
    eloffset[2][n] = electrical_offset_top[2 * (n + 32) + 2] << 8 | electrical_offset_top[2 * (n + 32) + 3];
    eloffset[3][n] = electrical_offset_top[2 * (n + 48) + 2] << 8 | electrical_offset_top[2 * (n + 48) + 3];
    // bottom half
    eloffset[4][n] = electrical_offset_bottom[2 * (n + 48) + 2] << 8 | electrical_offset_bottom[2 * (n + 48) + 3];
    eloffset[5][n] = electrical_offset_bottom[2 * (n + 32) + 2] << 8 | electrical_offset_bottom[2 * (n + 32) + 3];
    eloffset[6][n] = electrical_offset_bottom[2 * (n + 16) + 2] << 8 | electrical_offset_bottom[2 * (n + 16) + 3];
    eloffset[7][n] = electrical_offset_bottom[2 * n + 2] << 8 | electrical_offset_bottom[2 * n + 3];


  }




  // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
  sum = ptat_top_block0 + ptat_top_block1 + ptat_bottom_block0 + ptat_bottom_block1;
  ptat_av_uint16 = sum / 4;

  // calculate ambient_temperature (datasheet, chapter: 11.1 Ambient Temperature )
  ambient_temperature = ptat_av_uint16 * ptatgr_float + ptatoff_float;



  // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
  sum = vdd_top_block0 + vdd_top_block1 + vdd_bottom_block0 + vdd_bottom_block1;
  vdd_av_uint16 = sum / 4;

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
  nrofdefpix = (uint8_t)((eeprom_read_routine(E_NROFDEFPIX) & 0x00FF));
  deadpixadr = (uint8_t)eeprom_read_routine(E_DEADPIXADR);
  deadpixmask = (uint8_t)((eeprom_read_routine(E_DEADPIXMASK) & 0xFF00) >> 8);
  gradscale = (uint8_t)eeprom_read_routine(E_GRADSCALE);
  vddscgrad = (uint8_t)eeprom_read_routine(E_VDDSCGRAD);
  vddscoff = (uint8_t)eeprom_read_routine(E_VDDSCOFF);
  epsilon = (uint8_t)eeprom_read_routine(E_EPSILON);
  vddth1 = eeprom_read_routine(E_VDDTH1);
  vddth2 = eeprom_read_routine(E_VDDTH2);
  ptatth1 = eeprom_read_routine(E_PTATTH1);
  ptatth2 = eeprom_read_routine(E_PTATTH2);
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

  if (nrofdefpix) {
    deadpixadr = eeprom_read_routine(E_DEADPIXADR);
    if (deadpixadr > 128) {    // adaptedAdr:
      deadpixadr = 256 + 128 - deadpixadr + 2 * (deadpixadr % 16 ) - 16;
    }
  }
  else {
    deadpixadr = 0;
  }


  // --- Thgrad_ij ---
  // top half
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < 16; n++) {
      thgrad[m][n] = eeprom_read_routine(0x100 + n + m * 16);
    }
  }

  // bottom half
  for (int n = 0; n < 16; n++) {
    thgrad[15][n] = eeprom_read_routine(E_THGRAD + 128 + n + 0 * 16);
    thgrad[14][n] = eeprom_read_routine(E_THGRAD + 128 + n + 1 * 16);
    thgrad[13][n] = eeprom_read_routine(E_THGRAD + 128 + n + 2 * 16);
    thgrad[12][n] = eeprom_read_routine(E_THGRAD + 128 + n + 3 * 16);
    thgrad[11][n] = eeprom_read_routine(E_THGRAD + 128 + n + 4 * 16);
    thgrad[10][n] = eeprom_read_routine(E_THGRAD + 128 + n + 5 * 16);
    thgrad[9][n] = eeprom_read_routine(E_THGRAD + 128 + n + 6 * 16);
    thgrad[8][n] = eeprom_read_routine(E_THGRAD + 128 + n + 7 * 16);

  }




  // --- Thoffset_ij ---
  // top half
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < 16; n++) {
      thoffset[m][n] = eeprom_read_routine(E_THOFFSET + n + m * 16);
    }
  }
  // bottom half
  for (int n = 0; n < 16; n++) {
    thoffset[15][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 0 * 16);
    thoffset[14][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 1 * 16);
    thoffset[13][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 2 * 16);
    thoffset[12][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 3 * 16);
    thoffset[11][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 4 * 16);
    thoffset[10][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 5 * 16);
    thoffset[9][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 6 * 16);
    thoffset[8][n] = eeprom_read_routine(E_THOFFSET + 128 + n + 7 * 16);
  }


  // --- P_ij ---
  // top half
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < 16; n++) {
      pij[m][n] = eeprom_read_routine(E_THOFFSET + n + m * 16);
    }
  }
  // bottom half
  for (int n = 0; n < 16; n++) {
    pij[15][n] = eeprom_read_routine(E_PIJ + 128 + n + 0 * 16);
    pij[14][n] = eeprom_read_routine(E_PIJ + 128 + n + 1 * 16);
    pij[13][n] = eeprom_read_routine(E_PIJ + 128 + n + 2 * 16);
    pij[12][n] = eeprom_read_routine(E_PIJ + 128 + n + 3 * 16);
    pij[11][n] = eeprom_read_routine(E_PIJ + 128 + n + 4 * 16);
    pij[10][n] = eeprom_read_routine(E_PIJ + 128 + n + 5 * 16);
    pij[9][n] = eeprom_read_routine(E_PIJ + 128 + n + 6 * 16);
    pij[8][n] = eeprom_read_routine(E_PIJ + 128 + n + 7 * 16);
  }


  // ---VDDCOMPGRAD---
  /* !!! length: 12 bit !!!

      HINT: read 4 values from 3 eeprom blocks

      example:
      i = 0;    // Block index
      k = 0;    // VDDCOMPGRAD index

      |     BLOCK_i       |     BLOCK_i+1     |     BLOCK_i+2     |       <- 16bit
      |   MSB   |   LSB   |   MSB   |   LSB   |   MSB   |   LSB   |       <-  8bit
      |    |    |    |    |    |    |    |    |    |    |    |    |       <-  4bit

      |    | 1st| 2nd| 3rd|    |    |    |    |    |    |    |    |       <-  VALUE_k
      | 3rd|    |    |    |    |    | 1st| 2nd|    |    |    |    |       <-  VALUE_k+1
      |    |    |    |    | 2nd| 3rd|    |    |    |    |    | 1st|       <-  VALUE_k+2
      |    |    |    |    |    |    |    |    | 1st| 2nd| 3rd|    |       <-  VALUE_k+3


      i = i+3;
      k = k+4;

  */



  uint16_t block_i, block_i1, block_i2;
  uint16_t value_k, value_k1, value_k2, value_k3;
  uint16_t value_array[8][16];

  int i = 0;
  int k = 0;

  while (k < 128) {

    // read 3 blocks
    block_i = eeprom_read_routine(E_VDDCOMPGRAD + i );
    block_i1 = eeprom_read_routine(E_VDDCOMPGRAD + i + 1 );
    block_i2 = eeprom_read_routine(E_VDDCOMPGRAD + i + 2 );



    // devide the 3 blocks in 4 values
    value_k = (block_i & 0x0FFF);
    value_k1 = (block_i1 & 0x00FF) << 4 | (block_i & 0xF000) >> 12;
    value_k2 = (block_i2 & 0x000F) << 8 | (block_i1 & 0xFF00) >> 8;
    value_k3 =  (block_i2 & 0xFFF0) >> 4;

    // save in array
    value_array[k / 16][k % 16] = value_k - 0x800;
    value_array[(k + 1) / 16][(k + 1) % 16] = value_k1 - 0x800;
    value_array[(k + 2) / 16][(k + 2) % 16] = value_k2 - 0x800;
    value_array[(k + 3) / 16][(k + 3) % 16] = value_k3 - 0x800;

    // increase i/k
    i = i + 3;
    k = k + 4;

  }


  // save in global variable
  for (int n = 0; n < 16; n++) {

    vddcompgrad[0][n] = value_array[0][n];
    vddcompgrad[1][n] = value_array[1][n];
    vddcompgrad[2][n] = value_array[2][n];
    vddcompgrad[3][n] = value_array[3][n];

    // bottom half
    vddcompgrad[4][n] = value_array[7][n];
    vddcompgrad[5][n] = value_array[6][n];
    vddcompgrad[6][n] = value_array[5][n];
    vddcompgrad[7][n] = value_array[4][n];
  }



  // --- VDDCOMPOFF ---
  // !!! 12 bit values (same as vddcompgrad) !!!


  // reset i/k
  i = 0;
  k = 0;
  while (k < 128) {


    // read 3 blocks
    block_i = eeprom_read_routine(E_VDDCOMPOFF + i );
    block_i1 = eeprom_read_routine(E_VDDCOMPOFF + i + 1 );
    block_i2 = eeprom_read_routine(E_VDDCOMPOFF + i + 2 );

    // devide the 3 blocks in 4 values
    value_k = (block_i & 0x0FFF);
    value_k1 = (block_i1 & 0x00FF) << 4 | (block_i & 0xF000) >> 12;
    value_k2 = (block_i2 & 0x000F) << 8 | (block_i1 & 0xFF00) >> 8;
    value_k3 =  (block_i2 & 0xFFF0) >> 4;

    // save in array
    value_array[k / 16][k % 16] = value_k - 0x800;
    value_array[(k + 1) / 16][(k + 1) % 16] = value_k1 - 0x800;
    value_array[(k + 2) / 16][(k + 2) % 16] = value_k2 - 0x800;
    value_array[(k + 3) / 16][(k + 3) % 16] = value_k3 - 0x800;

    // increase i/k
    i = i + 3;
    k = k + 4;

  }



  for (int n = 0; n < 16; n++) {
    // top half
    vddcompoff[0][n] = value_array[0][n];
    vddcompoff[1][n] = value_array[1][n];
    vddcompoff[2][n] = value_array[2][n];
    vddcompoff[3][n] = value_array[3][n];

    // bottom half
    vddcompoff[4][n] = value_array[7][n];
    vddcompoff[5][n] = value_array[6][n];
    vddcompoff[6][n] = value_array[5][n];
    vddcompoff[7][n] = value_array[4][n];
  }



}






/********************************************************************
   Function:        eeprom_read_routine(uint16_t addr)

   Description:     read eeprom register (see datasheet, chp.: 10.10 I2C Example Sequence – EEPROM Sequential Read )

   Dependencies:    addr... eeprom register adress
 *******************************************************************/
word eeprom_read_routine(uint16_t addr) {

  // SET_ADDR
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(SET_ADDRESS));
  Wire1.write((int)(addr >> 8));  // MSB
  Wire1.write((int)(addr & 0xFF)); // LSB
  Wire1.endTransmission();


  // NORMAL_READ
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(NORMAL_READ));
  Wire1.endTransmission();



  // GET_DATA
  uint8_t eeprom_data_buffer[2] = {0};
  receive_eeprom_data(GET_DATA, (uint8_t*)eeprom_data_buffer, 2);


  // EEPROM_ACTIVE
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write(ACTIVE);
  Wire1.endTransmission();



  uint16_t data_word = eeprom_data_buffer[1] << 8 | eeprom_data_buffer[0];
  return data_word;

}

/********************************************************************
   Function:        void read_EEPROM_byte( uint16_t addr, uint8_t *dest, uint16_t n)

   Description:     receive (16 bit) eeprom data

   Dependencies:    cmd... receive command (0x0B)
                    dest... data buffer
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
word write_eeprom_routine(uint16_t addr, uint16_t value) {

  // SET_ADDR
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write((int)(SET_ADDRESS));
  Wire1.write((int)(addr >> 8));  // MSB
  Wire1.write((int)(addr & 0xFF)); // LSB
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

  // EEPROM_ACTIVE
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write(ACTIVE);
  Wire1.endTransmission();
  delay(6);

}


































/********************************************************************

   END OF READ-OUT PROGRAMM

   the following functions are used to chat with the serial monitor

 *******************************************************************/


























/********************************************************************
   Function:        print_pixel_temps()

   Description:     print temperature on serial monitor

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
   Function:        print_calc_steps()

   Description:     print every needed step for temperature calculation + pixel masking

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
  Serial.print("PTAT_av = 1/4*(");
  Serial.print(ptat_top_block0);
  Serial.print(" + ");
  Serial.print(ptat_top_block1);
  Serial.print(" + ");
  Serial.print(ptat_bottom_block0);
  Serial.print(" + ");
  Serial.print(ptat_bottom_block1);
  Serial.print(") = ");
  Serial.print(ptat_av_uint16);
  Serial.print("\n\nTa = ");
  Serial.print(ptat_av_uint16);
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


  Serial.print("\n\n\n6) vdd compensation (V_ij_vddcomp):\n\n");
  Serial.print("VDD_av = 1/4*(");
  Serial.print(vdd_top_block0);
  Serial.print(" + ");
  Serial.print(vdd_top_block1);
  Serial.print(" + ");
  Serial.print(vdd_bottom_block0);
  Serial.print(" + ");
  Serial.print(vdd_bottom_block1);
  Serial.print(") = ");
  Serial.print(vdd_av_uint16);
  Serial.print("\n\n");



  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(vij_vddcomp_int32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }


  Serial.print("\n\n\n7) calculate sensitivity coefficients (pixc_ij):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(pixcij_uint32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n8) multiply scaling coeff and sensitivity coeff to compensated pixel voltages (V_ij_pixc):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(vij_pixc_int32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n9) calcluate final pixel temperature (in dK) with lookup table and bilinear interpolation:\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(temp_pix_uint32[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  Serial.print("\n\n\n10) pixel masking (if there is a defect pixel):\n\n");

  if (nrofdefpix > 0) {
    for (int m = 0; m < sensor.number_row; m++) {
      for (int n = 0; n < sensor.number_col; n++) {
        Serial.print(temp_pix_uint32[m][n]);
        Serial.print("\t");
      }
      Serial.print("\n");
    }
  }
  else {
    Serial.print("no defect pixel");
  }





  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");
}



/********************************************************************
   Function:        print_eeprom_hex()

   Description:     print eeprom contint as hex values

   Dependencies:
 *******************************************************************/
void print_eeprom_hex() {

  Serial.print("\n\n\n---PRINT EEPROM (HEX)---\n");
  Serial.print("\n\nEEPROM 16x16\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    Serial.print("- ");
  }

  for (int i = 0; i <= 0x3FF; i++) {


    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < 0x40) {
        Serial.print("HEADER\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0xA0) {
        Serial.print("VDDGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x100) {
        Serial.print("VDDOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x200) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x300) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x400) {
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


/********************************************************************
   Function:        print_eeprom_value()

   Description:     print all needed values in their saved form

   Dependencies:
 *******************************************************************/
void print_eeprom_value() {


  Serial.print("\n\n\n---PRINT EEPROM (VALUE)---\n");
  Serial.print("\n\nEEPROM 16x16\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

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
  Serial.print("\t");
  Serial.print(nrofdefpix + deadpixmask);
  Serial.print("\t");
  Serial.print(deadpixadr);
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
  Serial.print("\t");
  Serial.print(vddth1);
  Serial.print("\t");
  Serial.print(vddth2);
  Serial.print("\t\t\t\t\t\t");
  Serial.print(ptatth1);
  Serial.print("\t");
  Serial.print(ptatth2);
  // 4th line
  Serial.print("\n");
  Serial.print("HEADER\t0x30");
  Serial.print("\t|\t\t\t\t\t    ");
  Serial.print(ptatgr_float);
  Serial.print("\t\t   ");
  Serial.print(ptatoff_float);
  Serial.print("\t\t   ");
  Serial.print(id);
  Serial.print("\t\t\t");
  Serial.print(vddscgrad);
  Serial.print("\t");
  Serial.print(vddscoff);
  // 5th line
  Serial.print("\n");
  Serial.print("\t0x40\t|\n");
  Serial.print("\t...\t|\t\t VddCompGrad and VddCompOff below (12 bit values stored in 16bit register)\n");
  Serial.print("\t0xF0\t|");


  // OTHER
  for (int i = 0x100; i <= 0x3FF; i++) {

    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < 0x100) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x200) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x300) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x400) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }
    else {
      Serial.print("\t");
    }


    if (i >= 0x100 && i < 0x300) {
      Serial.print((int16_t)eeprom_read_routine(i));
    }
    else {
      Serial.print(eeprom_read_routine(i));
    }


  }

  Serial.print("\n\nVddCompGrad (12bit signed int):\n");
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(vddcompgrad[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }


  Serial.print("\nVddCompOff (12bit signed int):\n");
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(vddcompoff[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }



  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");

}
