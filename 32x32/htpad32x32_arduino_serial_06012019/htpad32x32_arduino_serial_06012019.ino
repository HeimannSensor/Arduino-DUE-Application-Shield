#include <Wire.h>  // for I2C
#include "sensordef_32x32.h"
#include "lookuptable.h"




// SENSOR CHARACTERISTICS
struct characteristics {
  uint8_t number_row;    // number of raws
  uint8_t number_col;    // number of column
  uint8_t number_blocks; // number of blocks (top + down)
  uint8_t number_pixel;  // number of pixel
};
characteristics sensor = {32, 32, 8, 1024};



// EEPROM DATA
uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, arraytype;
int8_t globaloff;
uint8_t mbit_user, bias_user, clk_user, bpa_user, pu_user;
uint16_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
int16_t thgrad[32][32];
int16_t thoffset[32][32];
int16_t vddcompgrad[8][32];
int16_t vddcompoff[8][32];
uint16_t pij[32][32];
uint16_t deadpixadr[24];
uint8_t deadpixmask[12];
int32_t pixcij_int32[32][32];
uint32_t id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;

// SENSOR DATA
uint8_t data_top_block0[258], data_top_block1[258], data_top_block2[258], data_top_block3[258];
uint8_t data_bottom_block0[258], data_bottom_block1[258], data_bottom_block2[258], data_bottom_block3[258];
uint8_t electrical_offset_top[258], electrical_offset_bottom[258];
uint16_t eloffset[8][32];
uint16_t ptat_top_block0, ptat_top_block1, ptat_top_block2, ptat_top_block3;
uint16_t ptat_bottom_block0, ptat_bottom_block1, ptat_bottom_block2, ptat_bottom_block3;
uint16_t vdd_top_block0, vdd_top_block1, vdd_top_block2, vdd_top_block3;
uint16_t vdd_bottom_block0, vdd_bottom_block1, vdd_bottom_block2, vdd_bottom_block3;
uint16_t data_pixel[32][32];
uint8_t statusreg;

// CALCULATED VALUES
uint16_t ptat_av_uint16;
uint16_t vdd_av_uint16;
uint16_t ambient_temperature;
int32_t vij_pixc_int32[32][32];
uint32_t temp_pix_uint32[32][32];
int32_t vij_comp_int32[32][32];
int32_t vij_comp_s_int32[32][32];
int32_t vij_vddcomp_int32[32][32];

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


  Serial.print("\nSETUP\n\n");
  // begin I2C communication (400kHz - eeprom / 1000kHz - sensor)
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

  Wire1.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)


  Serial.print("\nread eeprom");
  read_eeprom();


  Wire1.setClock(CLOCK_SENSOR);
  // HINT: To increase the frame rate, here the I2C clock is higher than 1MHz from datasheet. If this causes any problems, set to the datasheet value.

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
    Serial.print("\n\tChange device in sensordef_32x32.h to sensor with tablenumber ");
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

   Description:

   Dependencies:
 *******************************************************************/
void loop() {

  switch (var) {

    case 'm':
      // ---MENU---
      Serial.print("\n\nMENU\n\n");
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
      //pixel_masking() included in print_calc_steps
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
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block0, 258);



  // --- BLOCK 1 with PTAT ---

  // change block in configuration register (to block1)
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
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block1, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block1, 258);



  // --- BLOCK 2 with PTAT ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x29 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block2, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block2, 258);



  // --- BLOCK 3 with PTAT ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  1  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x39 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block3, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block3, 258);



  // SAVE PTAT
  ptat_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
  ptat_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
  ptat_top_block2 = data_top_block2[0] << 8  | data_top_block2[1];
  ptat_top_block3 = data_top_block3[0] << 8  | data_top_block3[1];
  ptat_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
  ptat_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];
  ptat_bottom_block2 = data_bottom_block2[0] << 8  | data_bottom_block2[1];
  ptat_bottom_block3 = data_bottom_block3[0] << 8  | data_bottom_block3[1];





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
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block0, 258);



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
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block1, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block1, 258);



  // --- BLOCK 2 with VDD ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  0  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x29 + 0x04);

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block2, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block2, 258);



  // --- BLOCK 3 with VDD ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  1  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x39 + 0x04);

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block3, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block3, 258);



  // SAVE VDD
  vdd_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
  vdd_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
  vdd_top_block2 = data_top_block2[0] << 8  | data_top_block2[1];
  vdd_top_block3 = data_top_block3[0] << 8  | data_top_block3[1];
  vdd_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
  vdd_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];
  vdd_bottom_block2 = data_bottom_block2[0] << 8  | data_bottom_block2[1];
  vdd_bottom_block3 = data_bottom_block3[0] << 8  | data_bottom_block3[1];



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
  read_sensor_register( TOP_HALF, (uint8_t*)&electrical_offset_top, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&electrical_offset_bottom, 258);


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


  uint8_t number_neighbours[24];
  uint32_t temp_defpix[24];


  for (int i = 0; i < nrofdefpix; i++) {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half

    if (deadpixadr[i] < 512) {

      if ( (deadpixmask[i] & 1 )  == 1) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32)];
      }


      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) - 1];
      }

    }

    // bottom half
    else {

      if ( (deadpixmask[i] & 1 << 0 )  == 1 << 0) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    temp_pix_uint32[deadpixadr[i] >> 5][deadpixadr[i] % 32] = temp_defpix[i];

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
      vij_comp_int32[m][n] = (data_pixel[m][n] - (thgrad[m][n] * ptat_av_uint16) / gradscale_div - thoffset[m][n]);


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
      vij_pixc_int32[m][n] =  (int32_t)(vij_pixc_and_pcscaleval / (int64_t)pixcij_int32[m][n]);


      // --- LOOKUPTABLE ---
      // find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter: 11.7 Look-up table)
      table_row = vij_pixc_int32[m][n] + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      temp_pix_uint32[m][n] = (uint32_t)((vy - vx) * ((int32_t)(vij_pixc_int32[m][n] + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      // --- GLOBAL OFFSET ---
      temp_pix_uint32[m][n] = temp_pix_uint32[m][n] + globaloff;

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
    data_pixel[1][n] = data_top_block0[2 * (n + 32) + 2] << 8 | data_top_block0[2 * (n + 32) + 3];
    data_pixel[2][n] = data_top_block0[2 * (n + 64) + 2] << 8 | data_top_block0[2 * (n + 64) + 3];
    data_pixel[3][n] = data_top_block0[2 * (n + 96) + 2] << 8 | data_top_block0[2 * (n + 96) + 3];

    // block 1
    data_pixel[4][n] = data_top_block1[2 * n + 2] << 8 | data_top_block1[2 * n + 3];
    data_pixel[5][n] = data_top_block1[2 * (n + 32) + 2] << 8 | data_top_block1[2 * (n + 32) + 3];
    data_pixel[6][n] = data_top_block1[2 * (n + 64) + 2] << 8 | data_top_block1[2 * (n + 64) + 3];
    data_pixel[7][n] = data_top_block1[2 * (n + 96) + 2] << 8 | data_top_block1[2 * (n + 96) + 3];

    // block 2
    data_pixel[8][n] = data_top_block2[2 * n + 2] << 8 | data_top_block2[2 * n + 3];
    data_pixel[9][n] = data_top_block2[2 * (n + 32) + 2] << 8 | data_top_block2[2 * (n + 32) + 3];
    data_pixel[10][n] = data_top_block2[2 * (n + 64) + 2] << 8 | data_top_block2[2 * (n + 64) + 3];
    data_pixel[11][n] = data_top_block2[2 * (n + 96) + 2] << 8 | data_top_block2[2 * (n + 96) + 3];

    // block 3
    data_pixel[12][n] = data_top_block3[2 * n + 2] << 8 | data_top_block3[2 * n + 3];
    data_pixel[13][n] = data_top_block3[2 * (n + 32) + 2] << 8 | data_top_block3[2 * (n + 32) + 3];
    data_pixel[14][n] = data_top_block3[2 * (n + 64) + 2] << 8 | data_top_block3[2 * (n + 64) + 3];
    data_pixel[15][n] = data_top_block3[2 * (n + 96) + 2] << 8 | data_top_block3[2 * (n + 96) + 3];

    // --- PIXEL DATA BOTTOM HALF ---
    // block 3
    data_pixel[16][n] = data_bottom_block3[192 + 2 * n + 2] << 8 | data_bottom_block3[192 + 2 * n + 3];
    data_pixel[17][n] = data_bottom_block3[128 + 2 * n + 2] << 8 | data_bottom_block3[128 + 2 * n + 3];
    data_pixel[18][n] = data_bottom_block3[64 + 2 * n + 2] << 8 | data_bottom_block3[64 + 2 * n + 3];
    data_pixel[19][n] = data_bottom_block3[0 + 2 * n + 2] << 8 | data_bottom_block3[0 + 2 * n + 3];

    // block 2
    data_pixel[20][n] = data_bottom_block2[192 + 2 * n + 2] << 8 | data_bottom_block2[192 + 2 * n + 3];
    data_pixel[21][n] = data_bottom_block2[128 + 2 * n + 2] << 8 | data_bottom_block2[128 + 2 * n + 3];
    data_pixel[22][n] = data_bottom_block2[64 + 2 * n + 2] << 8 | data_bottom_block2[64 + 2 * n + 3];
    data_pixel[23][n] = data_bottom_block2[0 + 2 * n + 2] << 8 | data_bottom_block2[0 + 2 * n + 3];

    // block 1
    data_pixel[24][n] = data_bottom_block1[192 + 2 * n + 2] << 8 | data_bottom_block1[192 + 2 * n + 3];
    data_pixel[25][n] = data_bottom_block1[128 + 2 * n + 2] << 8 | data_bottom_block1[128 + 2 * n + 3];
    data_pixel[26][n] = data_bottom_block1[64 + 2 * n + 2] << 8 | data_bottom_block1[64 + 2 * n + 3];
    data_pixel[27][n] = data_bottom_block1[0 + 2 * n + 2] << 8 | data_bottom_block1[0 + 2 * n + 3];

    // block 0
    data_pixel[28][n] = data_bottom_block0[192 + 2 * n + 2] << 8 | data_bottom_block0[192 + 2 * n + 3];
    data_pixel[29][n] = data_bottom_block0[128 + 2 * n + 2] << 8 | data_bottom_block0[128 + 2 * n + 3];
    data_pixel[30][n] = data_bottom_block0[64 + 2 * n + 2] << 8 | data_bottom_block0[64 + 2 * n + 3];
    data_pixel[31][n] = data_bottom_block0[0 + 2 * n + 2] << 8 | data_bottom_block0[0 + 2 * n + 3];


    // --- ELECTRICAL OFFSET ---
    // top half
    eloffset[0][n] = electrical_offset_top[2 * n + 2] << 8 | electrical_offset_top[2 * n + 3];
    eloffset[1][n] = electrical_offset_top[2 * (n + 32) + 2] << 8 | electrical_offset_top[2 * (n + 32) + 3];
    eloffset[2][n] = electrical_offset_top[2 * (n + 64) + 2] << 8 | electrical_offset_top[2 * (n + 64) + 3];
    eloffset[3][n] = electrical_offset_top[2 * (n + 96) + 2] << 8 | electrical_offset_top[2 * (n + 96) + 3];
    // bottom half
    eloffset[4][n] = electrical_offset_bottom[2 * (n + 96) + 2] << 8 | electrical_offset_bottom[2 * (n + 96) + 3];
    eloffset[5][n] = electrical_offset_bottom[2 * (n + 64) + 2] << 8 | electrical_offset_bottom[2 * (n + 64) + 3];
    eloffset[6][n] = electrical_offset_bottom[2 * (n + 32) + 2] << 8 | electrical_offset_bottom[2 * (n + 32) + 3];
    eloffset[7][n] = electrical_offset_bottom[2 * n + 2] << 8 | electrical_offset_bottom[2 * n + 3];


  }




  // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
  sum = ptat_top_block0 + ptat_top_block1 + ptat_top_block2 + ptat_top_block3 + ptat_bottom_block0 + ptat_bottom_block1 + ptat_bottom_block2 + ptat_bottom_block3;
  ptat_av_uint16 = sum / 8;


  // calculate ambient_temperature (datasheet, chapter: 11.1 Ambient Temperature )
  ambient_temperature = ptat_av_uint16 * ptatgr_float + ptatoff_float;


  // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
  sum = vdd_top_block0 + vdd_top_block1 + vdd_top_block2 + vdd_top_block3 + vdd_bottom_block0 + vdd_bottom_block1 + vdd_bottom_block2 + vdd_bottom_block3;
  vdd_av_uint16 = sum / 8;


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

  for (int m = 0; m < 32; m++) {
    for (int n = 0; n < 32; n++) {

      // calc sensitivity coefficients (see datasheet, chapter: 11.5 Object Temperature)
      pixcij_int32[m][n] = (int32_t)pixcmax - (int32_t)pixcmin;
      pixcij_int32[m][n] = pixcij_int32[m][n] / 65535;
      pixcij_int32[m][n] = pixcij_int32[m][n] * pij[m][n];
      pixcij_int32[m][n] = pixcij_int32[m][n] + pixcmin;
      pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0  * epsilon / 100;
      pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0  * globalgain / 10000;

    }
  }

}








/********************************************************************
   Function:        void read_EEPROM_byte(int deviceaddress, unsigned int eeaddress )

   Description:     read eeprom register

   Dependencies:    epprom address (deviceaddress)
                    eeprom register (eeaddress)
 *******************************************************************/
byte read_EEPROM_byte(int deviceaddress, unsigned int eeaddress ) {
  byte rdata = 0xFF;

  Wire1.beginTransmission(deviceaddress);
  Wire1.write((int)(eeaddress >> 8));   // MSB
  Wire1.write((int)(eeaddress & 0xFF)); // LSB
  Wire1.endTransmission();

  Wire1.requestFrom(deviceaddress, 1);

  if (Wire1.available()) rdata = Wire1.read();

  return rdata;
}



/********************************************************************
   Function:        void read_eeprom()

   Description:     read all values from eeprom

   Dependencies:
 *******************************************************************/
void read_eeprom() {
  byte b[3];
  bw = (read_EEPROM_byte(EEPROM_ADDRESS, E_BW2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_BW1)) / 100;
  id = read_EEPROM_byte(EEPROM_ADDRESS, E_ID4) << 24 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID3) << 16 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID1);
  mbit_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_MBIT_CALIB);
  bias_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_BIAS_CALIB);
  clk_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_CLK_CALIB);
  bpa_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_BPA_CALIB);
  pu_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_CALIB);
  mbit_user = read_EEPROM_byte(EEPROM_ADDRESS, E_MBIT_USER);
  bias_user = read_EEPROM_byte(EEPROM_ADDRESS, E_BIAS_USER);
  clk_user = read_EEPROM_byte(EEPROM_ADDRESS, E_CLK_USER);
  bpa_user = read_EEPROM_byte(EEPROM_ADDRESS, E_BPA_USER);
  pu_user = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_USER);
  vddth1 = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH1_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH1_1);
  vddth2 = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH2_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH2_1);
  vddscgrad = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDSCGRAD);
  vddscoff = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDSCOFF);
  ptatth1 = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH1_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH1_1);
  ptatth2 = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH2_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH2_1);
  nrofdefpix = read_EEPROM_byte(EEPROM_ADDRESS, E_NROFDEFPIX);
  gradscale = read_EEPROM_byte(EEPROM_ADDRESS, E_GRADSCALE);
  tablenumber = read_EEPROM_byte(EEPROM_ADDRESS, E_TABLENUMBER2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_TABLENUMBER1);
  arraytype = read_EEPROM_byte(EEPROM_ADDRESS, E_ARRAYTYPE);
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_4);
  ptatgr_float = *(float*)b;
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_4);
  ptatoff_float = *(float*)b;
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_4);
  pixcmin = *(float*)b;
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_4);
  pixcmax = *(float*)b;
  epsilon = read_EEPROM_byte(EEPROM_ADDRESS, E_EPSILON);
  globaloff = read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALOFF);
  globalgain = read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALGAIN_1);
  // --- DeadPixAdr ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixadr[i] = read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXADR + 2 * i + 1 ) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXADR + 2 * i);
    if (deadpixadr[i] > 512) {    // adaptedAdr:
      deadpixadr[i] = 1024 + 512 - deadpixadr[i] + 2 * (deadpixadr[i] % 32 ) - 32;
    }
  }
  // --- DeadPixMask ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixmask[i] = read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXMASK + i);
  }

  // --- Thgrad_ij ---
  int m = 0;
  int n = 0;
  uint16_t addr_i = 0x0740; // start address
  // top half
  for (int i = 0; i < 512; i++) {
    addr_i = 0x0740 + 2 * i;
    thgrad[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 2 * i);
    n++;
    if (n == 32) {
      n = 0;
      m++;
    }
  }
  // bottom half
  for (int i = 0; i < sensor.number_col; i++) {

    thgrad[31][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * i);
    thgrad[30][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 1 * 64 + 2 * i);
    thgrad[29][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * 64 + 2 * i);
    thgrad[28][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 3 * 64 + 2 * i);

    thgrad[27][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * i);
    thgrad[26][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 1 * 64 + 2 * i);
    thgrad[25][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * 64 + 2 * i);
    thgrad[24][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 3 * 64 + 2 * i);

    thgrad[23][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * i);
    thgrad[22][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 1 * 64 + 2 * i);
    thgrad[21][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * 64 + 2 * i);
    thgrad[20][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 3 * 64 + 2 * i);

    thgrad[19][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * i);
    thgrad[18][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 1 * 64 + 2 * i);
    thgrad[17][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * 64 + 2 * i);
    thgrad[16][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 3 * 64 + 2 * i);

  }

  // --- ThOffset_ij ---
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < 512; i++) {
    addr_i = 0x0F40 + 2 * i;
    thoffset[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 2 * i);
    n++;
    if (n == 32) {
      n = 0;
      m++;
    }
  }

  // bottom half
  for (int i = 0; i < sensor.number_col; i++) {
    thoffset[31][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * i);
    thoffset[30][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 1 * 64 + 2 * i);
    thoffset[29][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * 64 + 2 * i);
    thoffset[28][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 3 * 64 + 2 * i);

    thoffset[27][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * i);
    thoffset[26][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 1 * 64 + 2 * i);
    thoffset[25][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * 64 + 2 * i);
    thoffset[24][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 3 * 64 + 2 * i);

    thoffset[23][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * i);
    thoffset[22][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 1 * 64 + 2 * i);
    thoffset[21][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * 64 + 2 * i);
    thoffset[20][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 3 * 64 + 2 * i);

    thoffset[19][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * i);
    thoffset[18][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 1 * 64 + 2 * i);
    thoffset[17][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * 64 + 2 * i);
    thoffset[16][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 3 * 64 + 2 * i);
  }


  //---VddCompGrad---

  // top half
  for (int i = 0; i < sensor.number_col; i++) {
    // top half
    vddcompgrad[0][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * i);
    vddcompgrad[1][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 1 * 64 + 2 * i);
    vddcompgrad[2][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * 64 + 2 * i);
    vddcompgrad[3][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 3 * 64 + 2 * i);
    // bottom half (backwards)
    vddcompgrad[7][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 4 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 4 * 64 + 2 * i);
    vddcompgrad[6][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 5 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 5 * 64 + 2 * i);
    vddcompgrad[5][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 6 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 6 * 64 + 2 * i);
    vddcompgrad[4][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 7 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 7 * 64 + 2 * i);

  }

  //---VddCompOff---

  // top half
  for (int i = 0; i < sensor.number_col; i++) {
    // top half
    vddcompoff[0][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * i);
    vddcompoff[1][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 1 * 64 + 2 * i);
    vddcompoff[2][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * 64 + 2 * i);
    vddcompoff[3][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 3 * 64 + 2 * i);
    // bottom half
    vddcompoff[7][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 4 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 4 * 64 + 2 * i);
    vddcompoff[6][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 5 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 5 * 64 + 2 * i);
    vddcompoff[5][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 6 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 6 * 64 + 2 * i);
    vddcompoff[4][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 7 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 7 * 64 + 2 * i);

  }


  // --- P_ij ---
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < 512; i++) {
    addr_i = 0x0F40 + 2 * i;
    pij[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 2 * i);
    n++;
    if (n == 32) {
      n = 0;
      m++;
    }
  }

  // bottom half
  for (int i = 0; i < sensor.number_col; i++) {
    pij[31][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * i);
    pij[30][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 1 * 64 + 2 * i);
    pij[29][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * 64 + 2 * i);
    pij[28][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 3 * 64 + 2 * i);

    pij[27][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * i);
    pij[26][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 1 * 64 + 2 * i);
    pij[25][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * 64 + 2 * i);
    pij[24][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 3 * 64 + 2 * i);

    pij[23][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * i);
    pij[22][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 1 * 64 + 2 * i);
    pij[21][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * 64 + 2 * i);
    pij[20][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 3 * 64 + 2 * i);

    pij[19][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * i);
    pij[18][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 1 * 64 + 2 * i);
    pij[17][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * 64 + 2 * i);
    pij[16][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 3 * 64 + 2 * i);

  }


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
byte write_EEPROM_byte(int deviceaddress, unsigned int eeaddress, uint8_t input) {

  Wire1.beginTransmission(deviceaddress);
  Wire1.write((int)(eeaddress >> 8));   // MSB
  Wire1.write((int)(eeaddress & 0xFF)); // LSB
  Wire1.write(input); // LSB
  Wire1.endTransmission();

}

/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)

   Description:     read sensor register

   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
byte write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input) {

  Wire1.beginTransmission(deviceaddress);
  Wire1.write(registeraddress);
  Wire1.write(input);
  Wire1.endTransmission();

}




























/********************************************************************

   END OF READ-OUT PROGRAMM

   the following functions are used to chat with serial monitor

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
  Serial.print("PTAT_av = 1/8*(");
  Serial.print(ptat_top_block0);
  Serial.print(" + ");
  Serial.print(ptat_top_block1);
  Serial.print(" + ");
  Serial.print(ptat_top_block2);
  Serial.print(" + ");
  Serial.print(ptat_top_block3);
  Serial.print(" + ");
  Serial.print(ptat_bottom_block0);
  Serial.print(" + ");
  Serial.print(ptat_bottom_block1);
  Serial.print(" + ");
  Serial.print(ptat_bottom_block2);
  Serial.print(" + ");
  Serial.print(ptat_bottom_block3);
  Serial.print(") = ");
  Serial.print(ptat_av_uint16);
  Serial.print("\n\nTa = ");
  Serial.print(ptat_av_uint16);
  Serial.print(" * ");
  Serial.print(ptatgr_float, 5);
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
  Serial.print("VDD_av = 1/8*(");
  Serial.print(vdd_top_block0);
  Serial.print(" + ");
  Serial.print(vdd_top_block1);
  Serial.print(" + ");
  Serial.print(vdd_top_block2);
  Serial.print(" + ");
  Serial.print(vdd_top_block3);
  Serial.print(" + ");
  Serial.print(vdd_bottom_block0);
  Serial.print(" + ");
  Serial.print(vdd_bottom_block1);
  Serial.print(" + ");
  Serial.print(vdd_bottom_block2);
  Serial.print(" + ");
  Serial.print(vdd_bottom_block3);
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
      Serial.print(pixcij_int32[m][n]);
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


  pixel_masking();

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
  Serial.print("\n\nEEPROM 32x32\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    Serial.print("- ");
  }

  for (int i = 0; i <= 0x13FF; i++) {


    if (i % 16 == 0) {
      Serial.print("\n");

      if (i < 0x0080) {
        Serial.print("HEADER\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x00D0) {
        Serial.print("DEADPIX\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x0340) {
        Serial.print("FREE\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x0540) {
        Serial.print("VDDGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x0740) {
        Serial.print("VDDOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x0F40) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x1740) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x1F40) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }
    else {
      Serial.print("\t");
    }

    Serial.print(read_EEPROM_byte(EEPROM_ADDRESS, i), HEX);

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
  Serial.print("\nHINT: Here values longer than 8 bit are printed in their first block.\n");
  Serial.print("\n\nEEPROM 32x32\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

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
  Serial.print("\t\t\t");
  Serial.print(pixcmax);
  Serial.print("\t\t\t");
  Serial.print(gradscale);
  Serial.print("\t\t\t");
  Serial.print(tablenumber);
  Serial.print("\t\t");
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
  Serial.print("\t|\t\t\t");
  Serial.print(arraytype);
  Serial.print("\t\t\t\t");
  Serial.print(vddth1);
  Serial.print("\t\t");
  Serial.print(vddth2);
  // 4th line
  Serial.print("\n");
  Serial.print("HEADER\t0x30");
  Serial.print("\t|\t\t\t\t\t");
  Serial.print(ptatgr_float, 5);
  Serial.print("\t\t\t\t");
  Serial.print(ptatoff_float);
  Serial.print("\t\t\t\t");
  Serial.print(ptatth1);
  Serial.print("\t\t");
  Serial.print(ptatth2);
  // 5th line
  Serial.print("\n");
  Serial.print("HEADER\t0x40");
  Serial.print("\t|\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t");
  Serial.print(vddscgrad);
  Serial.print("\t");
  Serial.print(vddscoff);
  // 6th line
  Serial.print("\n");
  Serial.print("HEADER\t0x50");
  Serial.print("\t|\t\t\t\t\t");
  Serial.print(globaloff);
  Serial.print("\t");
  Serial.print(globalgain);
  // 7th line
  Serial.print("\n");
  Serial.print("HEADER\t0x60");
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
  // 8th line
  Serial.print("\n");
  Serial.print("HEADER\t0x70");
  Serial.print("\t|\t\t\t\t\t");
  Serial.print(id);
  Serial.print("\t\t\t\t\t\t\t\t\t\t\t");
  Serial.print(nrofdefpix);



  // OTHER (16bit)
  for (int i = 0x0080; i <= 0x00AF; i = i + 2) {

    if (i % 16 == 0) {
      Serial.print("\n");
      Serial.print("DEADPIX\t0x");
      Serial.print(i, HEX);
      Serial.print("\t|\t");
    }
    else {
      Serial.print("\t\t");
    }
    Serial.print(read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i));
  }


  // OTHER (8bit)
  for (int i = 0x00B0; i <= 0x033F; i++) {

    if (i % 16 == 0) {
      Serial.print("\n");
      if (i < 0x00D0) {
        Serial.print("DEADPIX\t0x");
      }
      else {
        Serial.print("FREE\t0x");
      }
      Serial.print(i, HEX);
      Serial.print("\t|\t");
    }
    else {
      Serial.print("\t");
    }
    Serial.print(read_EEPROM_byte(EEPROM_ADDRESS, i));
  }


  // OTHER (16bit)
  for (int i = 0x0340; i <= 0x1F3F; i = i + 2) {

    if (i % 16 == 0) {
      Serial.print("\n");


      if (i < 0x0540) {
        Serial.print("VDDGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x0740) {
        Serial.print("VDDOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x0F40) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x1740) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x1F40) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }

    else {
      Serial.print("\t\t");
    }



    if (i >= 0x0340 && i < 0x1740) {
      Serial.print((int16_t)(read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i)));
    }
    else {
      Serial.print(read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i));
    }


  }

  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");

}
