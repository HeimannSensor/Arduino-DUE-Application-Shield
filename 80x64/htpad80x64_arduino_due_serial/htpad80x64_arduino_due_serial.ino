#include <SPI.h>
#include "sensordef_80x64.h"
#include "lookuptable.h"


// SENSOR CHARACTERISTICS
struct characteristics {
  uint8_t number_row;    // number of raws
  uint8_t number_col;    // number of column
  uint8_t number_blocks; // number of blocks (top + down)
  uint16_t number_pixel;  // number of pixel
};
characteristics sensor = {64, 80, 8, 5120};


// EEPROM DATA
uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, arraytype;
int8_t globaloff;
uint8_t mbit_user, bias_user, clk_user, bpa_user, pu_user;
uint16_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
int8_t thgrad[64][80];
int16_t thoffset[64][80];
int16_t vddcompgrad[16][80];
int16_t vddcompoff[16][80];
uint16_t pij[64][80];
uint16_t deadpixadr[48];
uint8_t deadpixmask[24];
uint32_t id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;

// SENSOR DATA
uint8_t data_top_block0[1282], data_top_block1[1282], data_top_block2[1282], data_top_block3[1282];
uint8_t data_bottom_block0[1282], data_bottom_block1[1282], data_bottom_block2[1282], data_bottom_block3[1282];
uint8_t electrical_offset_top[1282], electrical_offset_bottom[1282];
uint16_t eloffset[16][80];
uint16_t ptat_top_block0, ptat_top_block1, ptat_top_block2, ptat_top_block3;
uint16_t ptat_bottom_block0, ptat_bottom_block1, ptat_bottom_block2, ptat_bottom_block3;
uint16_t vdd_top_block0, vdd_top_block1, vdd_top_block2, vdd_top_block3;
uint16_t vdd_bottom_block0, vdd_bottom_block1, vdd_bottom_block2, vdd_bottom_block3;
int32_t data_pixel[64][80];
uint8_t statusreg;

// CALCULATED VALUES
uint16_t ptat_av_uint16;
uint16_t vdd_av_uint16;
uint16_t ambient_temperature;
int32_t vij_comp_int32[64][80];


// OTHER
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;
char var = 'm';
uint16_t timer_duration;


void setup() {

  Serial.begin(115200);
  Serial.print("\nSETUP\n\n");

  // start the SPI library:
  SPI.begin();
  //SPI.setClockDivider(21);




  // SET INPUTS/OUTPUS (for DUE set pin 11 - 13 to INPUT)
  pinMode(13, INPUT);  // CLK
  pinMode(12, INPUT);  // MISO
  pinMode(11, INPUT);  // MOSI
  pinMode(10, OUTPUT); // chip select


  // search sensor via ID
  Serial.print("search device");
  while (id == 0 || id == 0xFFFFFFFF) {
    id = read_EEPROM_byte(E_ID4) << 24 | read_EEPROM_byte(E_ID3) << 16 | read_EEPROM_byte(E_ID2) << 8 | read_EEPROM_byte(E_ID1);
    Serial.print(".");
    delay(2000);
  }

  Serial.print("\nread eeprom");
  read_eeprom();

  Serial.print("\nwake up sensor");

  // to wake up sensor set configuration register to 0x01
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x01);

  Serial.print("\ninitialization");
  write_calibration_settings_to_sensor();

  Serial.print("\nstart sensor");
  // to start sensor set configuration register to 0x09
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x09);

  // other calculations before main loop
  gradscale_div = pow(2, gradscale);
  vddscgrad_div = pow(2, vddscgrad);
  vddscoff_div = pow(2, vddscoff);


  timer_duration = calc_timer_duration(bw, clk_calib, mbit_calib);
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
      //pixel_masking included in print_calc_steps
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

  // change block in configuration register (to block 0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x09 );


  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block0, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block0, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor



  // --- BLOCK 1 with PTAT ---

  // change block in configuration register (to block 1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  1  |   1   |    0     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x19 );



  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block1, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block1, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor



  // --- BLOCK 2 with PTAT ---

  // change block in configuration register (to block 2)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x29 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block2, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block2, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor



  // --- BLOCK 3 with PTAT ---

  // change block in configuration register (to block 3)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  1  |   1   |    0     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x39 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block3, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block3, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor



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

  // change block in configuration register (to block 0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    1     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x09 + 0x04 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block0, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block0, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor



  // --- BLOCK 1 with VDD ---

  // change block in configuration register (to block 1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  1  |   1   |    1     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x19 + 0x04 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block1, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block1, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor



  // --- BLOCK 2 with VDD ---

  // change block in configuration register (to block 2)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  0  |   1   |    1     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x29 + 0x04 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block2, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block2, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor

  // --- BLOCK 3 with VDD ---

  // change block in configuration register (to block 3)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  1  |   1   |    1     |   0   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x39 + 0x04 );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(data_top_block3, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(data_bottom_block3, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor


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

  // change block in configuration register
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
  write_sensor_byte(CONFIGURATION_REGISTER, 0x0B );

  // wait for end of conversion bit (~27ms)
  delayMicroseconds(timer_duration); // poll when 90% done
  statusreg = read_sensor_register( STATUS_REGISTER);
  while (bitRead(statusreg, 0) == 0) {
    statusreg = read_sensor_register( STATUS_REGISTER);
  }
  // get data of top half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(TOP_HALF);  // read command
  SPI.transfer(electrical_offset_top, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor
  // get data of bottom half:
  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(BOTTOM_HALF);  // read command
  SPI.transfer(electrical_offset_bottom, 1282); // receive data
  digitalWrite(10, LOW); // set HIGH, back to sensor


}




/********************************************************************
   Function:        void sort_data()

   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd

   Dependencies:
 *******************************************************************/
void sort_data() {

  uint32_t sum;
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < sensor.number_col; n++) {

      // --- PIXEL DATA TOP HALF ---

      // block 0
      data_pixel[m][n] = data_top_block0[2 * n + 2 + m * 160] << 8 | data_top_block0[2 * n + 3 + m * 160];
      // block 1
      data_pixel[m + 8][n] = data_top_block1[2 * n + 2 + m * 160] << 8 | data_top_block1[2 * n + 3 + m * 160];
      // block 2
      data_pixel[m + 16][n] = data_top_block2[2 * n + 2 + m * 160] << 8 | data_top_block2[2 * n + 3 + m * 160];
      // block 3
      data_pixel[m + 24][n] = data_top_block3[2 * n + 2 + m * 160] << 8 | data_top_block3[2 * n + 3 + m * 160];


      // --- PIXEL DATA BOTTOM HALF ---

      // block 0
      data_pixel[63 - m][n] = data_bottom_block0[2 * n + 2 + m * 160] << 8 | data_bottom_block0[2 * n + 3 + m * 160];
      // block 1
      data_pixel[63 - m - 8][n] = data_bottom_block1[2 * n + 2 + m * 160] << 8 | data_bottom_block1[2 * n + 3 + m * 160];
      // block 2
      data_pixel[63 - m - 16][n] = data_bottom_block2[2 * n + 2 + m * 160] << 8 | data_bottom_block2[2 * n + 3 + m * 160];
      // block 3
      data_pixel[63 - m - 24][n] = data_bottom_block3[2 * n + 2 + m * 160] << 8 | data_bottom_block3[2 * n + 3 + m * 160];

      // --- ELECTRICAL OFFSET ---
      // top half
      eloffset[m][n] = electrical_offset_top[2 * n + 2 + m * 160] << 8 | electrical_offset_top[2 * n + 3 + m * 160];
      // bottom half
      eloffset[15 - m][n] = electrical_offset_bottom[2 * n + 2 + m * 160] << 8 | electrical_offset_bottom[2 * n + 3 + m * 160];
    }
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
   Function:        calculate_pixel_temp()

   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table

   Dependencies:
 *******************************************************************/
void calculate_pixel_temp() {

  int64_t vij_pixc_and_pcscaleval;
  int64_t pixcij;
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
      //data_pixel[m][n] = (data_pixel[m][n] - (thgrad[m][n] * ptat_av_uint16) / gradscale_div - thoffset[m][n]);
      data_pixel[m][n] = ((int32_t)data_pixel[m][n] - (int32_t)( (int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div - (int32_t)thoffset[m][n]);



      // --- ELECTRICAL OFFSET
      // compensate electrical offset (see datasheet, chapter: 11.3 Electrical Offset)
      // top half
      if (m < sensor.number_row / 2) {
        data_pixel[m][n] = data_pixel[m][n] - eloffset[m % 8][n];
      }
      // bottom half
      else {
        data_pixel[m][n] = data_pixel[m][n] - eloffset[m % 8 + 8][n];
      }



      // --- VDD ---
      // select VddCompGrad and VddCompOff for pixel m,n:
      // top half
      if (m < sensor.number_row / 2) {
        vddcompgrad_n = vddcompgrad[m % 8][n];
        vddcompoff_n = vddcompoff[m % 8][n];
      }
      // bottom half
      else {
        vddcompgrad_n = vddcompgrad[m % 8 + 8][n];
        vddcompoff_n = vddcompoff[m % 8 + 8][n];
      }


      // compensate vdd (see datasheet, chapter: 11.4 Vdd Compensation)
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      data_pixel[m][n] = data_pixel[m][n] - vdd_calc_steps;

      // --- SENSITIVITY ---
      // multiply sensitivity coeff for each pixel (see datasheet, chapter: 11.5 Object Temperature)
      vij_pixc_and_pcscaleval = data_pixel[m][n] * (int64_t)PCSCALEVAL;
      pixcij = (int32_t)pixcmax - (int32_t)pixcmin;
      pixcij = pixcij * pij[m][n];
      pixcij = pixcij / 65535;
      pixcij = pixcij + pixcmin;
      pixcij = pixcij * epsilon / 100.0;
      pixcij = pixcij * globalgain / 10000.0;
      pixcij = pixcij;

      data_pixel[m][n] =  (vij_pixc_and_pcscaleval / pixcij);


      // --- LOOKUPTABLE ---
      // find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter: 11.7 Look-up table)
      table_row = data_pixel[m][n] + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      data_pixel[m][n] = (uint32_t)((vy - vx) * ((int32_t)(data_pixel[m][n] + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      // --- GLOBAL OFFSET ---
      data_pixel[m][n] = data_pixel[m][n] + globaloff;
    }
  }


}



/********************************************************************
   Function:        void pixel_masking()

   Description:     repair dead pixel by using the average of the neighbors

   Dependencies:    number of defect pixel (nrofdefpix),
                    dead pixel address (deadpixadr),
                    dead pixel mask (deadpixmask),
                    pixel temperatures (data_pixel[32][32])
 *******************************************************************/
void pixel_masking() {


  uint8_t number_neighbours[24];
  uint32_t temp_defpix[24];



  for (int i = 0; i < nrofdefpix; i++) {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half
    if (deadpixadr[i] < 2560) {

      if ( (deadpixmask[i] & 1 )  == 1) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) - 1][(deadpixadr[i] % 80)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) - 1][(deadpixadr[i] % 80) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80)][(deadpixadr[i] % 80) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) + 1][(deadpixadr[i] % 80) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) + 1][(deadpixadr[i] % 80)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) + 1][(deadpixadr[i] % 80) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80)][(deadpixadr[i] % 80) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) - 1][(deadpixadr[i] % 80) - 1];
      }

    }

    // bottom half
    else {

      if ( (deadpixmask[i] & 1 )  == 1 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) + 1][(deadpixadr[i] % 80)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) + 1][(deadpixadr[i] % 80) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80)][(deadpixadr[i] % 80) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) - 1][(deadpixadr[i] % 80) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) - 1][(deadpixadr[i] % 80)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) - 1][(deadpixadr[i] % 80) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80)][(deadpixadr[i] % 80) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(deadpixadr[i] / 80) + 1][(deadpixadr[i] % 80) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    data_pixel[deadpixadr[i] / 80][deadpixadr[i] % 80] = temp_defpix[i];

  }


}

/********************************************************************
   Function:        void read_eeprom()

   Description:     read all values from eeprom

   Dependencies:
 *******************************************************************/
void read_eeprom() {
  int m = 0;
  int n = 0;
  byte b[3];
  bw = (read_EEPROM_byte(E_BW2) << 8 | read_EEPROM_byte(E_BW1)) / 100;
  id = read_EEPROM_byte(E_ID4) << 24 | read_EEPROM_byte(E_ID3) << 16 | read_EEPROM_byte(E_ID2) << 8 | read_EEPROM_byte(E_ID1);
  mbit_calib = read_EEPROM_byte(E_MBIT_CALIB);
  bias_calib = read_EEPROM_byte(E_BIAS_CALIB);
  clk_calib = read_EEPROM_byte(E_CLK_CALIB);
  bpa_calib = read_EEPROM_byte(E_BPA_CALIB);
  pu_calib = read_EEPROM_byte(E_PU_CALIB);
  mbit_user = read_EEPROM_byte(E_MBIT_USER);
  bias_user = read_EEPROM_byte(E_BIAS_USER);
  clk_user = read_EEPROM_byte(E_CLK_USER);
  bpa_user = read_EEPROM_byte(E_BPA_USER);
  pu_user = read_EEPROM_byte(E_PU_USER);
  vddth1 = read_EEPROM_byte(E_VDDTH1_2) << 8 | read_EEPROM_byte(E_VDDTH1_1);
  vddth2 = read_EEPROM_byte(E_VDDTH2_2) << 8 | read_EEPROM_byte(E_VDDTH2_1);
  vddscgrad = read_EEPROM_byte(E_VDDSCGRAD);
  vddscoff = read_EEPROM_byte(E_VDDSCOFF);
  ptatth1 = read_EEPROM_byte(E_PTATTH1_2) << 8 | read_EEPROM_byte(E_PTATTH1_1);
  ptatth2 = read_EEPROM_byte(E_PTATTH2_2) << 8 | read_EEPROM_byte(E_PTATTH2_1);
  nrofdefpix = read_EEPROM_byte(E_NROFDEFPIX);
  gradscale = read_EEPROM_byte(E_GRADSCALE);
  tablenumber = read_EEPROM_byte(E_TABLENUMBER2) << 8 | read_EEPROM_byte(E_TABLENUMBER1);
  arraytype = read_EEPROM_byte(E_ARRAYTYPE);
  b[0] = read_EEPROM_byte(E_PTATGR_1);
  b[1] = read_EEPROM_byte(E_PTATGR_2);
  b[2] = read_EEPROM_byte(E_PTATGR_3);
  b[3] = read_EEPROM_byte(E_PTATGR_4);
  ptatgr_float = *(float*)b;
  b[0] = read_EEPROM_byte(E_PTATOFF_1);
  b[1] = read_EEPROM_byte(E_PTATOFF_2);
  b[2] = read_EEPROM_byte(E_PTATOFF_3);
  b[3] = read_EEPROM_byte(E_PTATOFF_4);
  ptatoff_float = *(float*)b;
  b[0] = read_EEPROM_byte(E_PIXCMIN_1);
  b[1] = read_EEPROM_byte(E_PIXCMIN_2);
  b[2] = read_EEPROM_byte(E_PIXCMIN_3);
  b[3] = read_EEPROM_byte(E_PIXCMIN_4);
  pixcmin = *(float*)b;
  b[0] = read_EEPROM_byte(E_PIXCMAX_1);
  b[1] = read_EEPROM_byte(E_PIXCMAX_2);
  b[2] = read_EEPROM_byte(E_PIXCMAX_3);
  b[3] = read_EEPROM_byte(E_PIXCMAX_4);
  pixcmax = *(float*)b;
  epsilon = read_EEPROM_byte(E_EPSILON);
  globaloff = read_EEPROM_byte(E_GLOBALOFF);
  globalgain = read_EEPROM_byte(E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(E_GLOBALGAIN_1);


  // --- DeadPixAdr ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixadr[i] = read_EEPROM_byte(E_DEADPIXADR + 2 * i + 1 ) << 8 | read_EEPROM_byte(E_DEADPIXADR + 2 * i);
    if (deadpixadr[i] > 2560) {    // adaptedAdr:
      deadpixadr[i] = 5120 + 2560 - deadpixadr[i] + 2 * (deadpixadr[i] % 80 ) - 80;
    }
  }


  // --- DeadPixMask ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixmask[i] = read_EEPROM_byte(E_DEADPIXMASK + i);
  }


  // --- Thgrad_ij, ThOffset_ij and P_ij ---
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < 2560; i++) {
    thgrad[m][n] = (int8_t)(read_EEPROM_byte(E_THGRAD + i));
    thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
    pij[m][n] = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
    n++;
    if (n ==  80) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = 63;
  n = 0;
  for (int i = 2560; i < 5120; i++) {
    thgrad[m][n] = read_EEPROM_byte(E_THGRAD + i);
    thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
    pij[m][n] = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
    n++;

    if (n ==  80) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

  //---VddCompGrad and VddCompOff---
  // top half
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < 640; i++) {
    vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  80) {
      n = 0;
      m++;  // !!!! forwards !!!!
    }
  }
  // bottom half
  m = 15;
  n = 0;
  for (int i = 640; i < 1280; i++) {
    vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
    vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
    n++;
    if (n ==  80) {
      n = 0;
      m--;      // !!!! backwards !!!!
    }
  }

}


/********************************************************************
   Function:        void write_calibration_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_calibration_settings_to_sensor() {

  write_sensor_byte(TRIM_REGISTER1, mbit_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER2, bias_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER3, bias_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER4, clk_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER5, bpa_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER6, bpa_calib);
  delay(5);
  write_sensor_byte(TRIM_REGISTER7, pu_calib);
}






/********************************************************************
   Function:        void read_EEPROM_byte(unsigned int eeaddress )

   Description:     read eeprom register

   Dependencies:    register address (address)
 *******************************************************************/
byte read_EEPROM_byte(unsigned int address ) {
  byte rdata = 0xFF;
  digitalWrite(10, LOW);  // set LOW to communicate with eeprom
  SPI.transfer(0b00000011); // READ command eeprom
  SPI.transfer16(address);  // register address eeprom
  rdata = SPI.transfer(0xFF); // end of message
  digitalWrite(10, HIGH); // set HIGH, back to sensor

  return rdata;
}




/********************************************************************
   Function:        void write_sensor_register( uint16_t addr)

   Description:     write to sensor register

   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
byte write_sensor_byte( uint8_t addr, uint8_t input) {

  digitalWrite(10, HIGH);  // set HIGH to communicate with Sensor
  SPI.transfer(addr);  // register address eeprom
  SPI.transfer(input);
  SPI.transfer(0xFF);// end of message
  digitalWrite(10, LOW); // set LOW, back to eeprom
}




/********************************************************************
   Function:        void read_sensor_register( uint16_t addr)

   Description:     read sensor register

   Dependencies:    register address (addr),
 *******************************************************************/
byte read_sensor_register( uint8_t addr) {
  byte rdata;

  digitalWrite(10, HIGH);  // set HIGH to communicate with sensor
  SPI.transfer(addr);  // command
  rdata = SPI.transfer(0xFF);// end of message
  digitalWrite(10, LOW); // set Low, back to eeprom

  return rdata;

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
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
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

  for (int i = 0; i < 0x8000; i++) {


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
      else if (i < 0x0800) {
        Serial.print("FREE\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x1200) {
        Serial.print("VDDGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x1C00) {
        Serial.print("VDDOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x3000) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x5800) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x8000) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }
    else {
      Serial.print("\t");
    }

    Serial.print("0x");
    if (read_EEPROM_byte(i) < 0x10) {
      Serial.print("0");
    }
    Serial.print(read_EEPROM_byte(i), HEX);

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
  Serial.print(ptatgr_float);
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
    Serial.print(read_EEPROM_byte( i + 1) << 8 | read_EEPROM_byte( i));
  }


  // OTHER (8bit)
  for (int i = 0x00B0; i <= 0x07FF; i++) {

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
    Serial.print(read_EEPROM_byte( i));
  }


  // OTHER (16bit)
  for (int i = 0x0800; i < 0x1C00; i = i + 2) {

    if (i % 16 == 0) {
      Serial.print("\n");


      if (i < 0x1200) {
        Serial.print("VDDGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x1C00) {
        Serial.print("VDDOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x3000) {
        Serial.print("THGRAD\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x5800) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x8000) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }

    else {
      Serial.print("\t\t");
    }

    Serial.print((int16_t)(read_EEPROM_byte( i + 1) << 8 | read_EEPROM_byte( i)));
  }




  // OTHER (8bit)
  for (int i = 0x1C00; i < 0x3000; i++) {

    if (i % 16 == 0) {
      Serial.print("\n");


      Serial.print("THGRAD\t0x");
      Serial.print(i, HEX);
      Serial.print("\t|\t");

    }
    else {
      Serial.print("\t");
    }
    Serial.print((int8_t)(read_EEPROM_byte( i)));
  }


  // OTHER (16bit)
  for (int i = 0x3000; i < 0x8000; i = i + 2) {

    if (i % 16 == 0) {
      Serial.print("\n");


      if (i < 0x5800) {
        Serial.print("THOFF\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
      else if (i < 0x8000) {
        Serial.print("Pij\t0x");
        Serial.print(i, HEX);
        Serial.print("\t|\t");
      }
    }

    else {
      Serial.print("\t\t");
    }
    if (i >= 0x5800 ) {
      Serial.print(read_EEPROM_byte( i + 1) << 8 | read_EEPROM_byte( i));
    }
    else {
      Serial.print((int16_t)(read_EEPROM_byte( i + 1) << 8 | read_EEPROM_byte( i)));
    }

  }


  Serial.print("\n\n\n\ndone (m... back to menu)\n\n\n");

}





/********************************************************************
   Function:        print_calc_steps()

   Description:     print every needed step for temperature calculation + pixel masking

   Dependencies:
 *******************************************************************/
void print_calc_steps() {

  int64_t vij_pixc_and_pcscaleval;
  int32_t pixcij;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;


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
  for (int m = 0; m < 16; m++) {
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

      data_pixel[m][n] = ((int32_t)data_pixel[m][n] - (int32_t)( (int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div - (int32_t)thoffset[m][n]);

    }
  }


  Serial.print("\n\n\n4) compensate thermal offset (V_ij_comp):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }


  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {



      // --- ELECTRICAL OFFSET
      // compensate electrical offset (see datasheet, chapter: 11.3 Electrical Offset)
      // top half
      if (m < sensor.number_row / 2) {
        data_pixel[m][n] = data_pixel[m][n] - eloffset[m % 8][n];
      }
      // bottom half
      else {
        data_pixel[m][n] = data_pixel[m][n] - eloffset[m % 8 + 8][n];
      }

    }
  }

  Serial.print("\n\n\n5) compensate electrical offset (V_ij_comp_s):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }



  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {

      // --- VDD ---
      // select VddCompGrad and VddCompOff for pixel m,n:
      // top half
      if (m < sensor.number_row / 2) {
        vddcompgrad_n = vddcompgrad[m % 8][n];
        vddcompoff_n = vddcompoff[m % 8][n];
      }
      // bottom half
      else {
        vddcompgrad_n = vddcompgrad[m % 8 + 8][n];
        vddcompoff_n = vddcompoff[m % 8 + 8][n];
      }

      // compensate vdd (see datasheet, chapter: 11.4 Vdd Compensation)
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      data_pixel[m][n] = data_pixel[m][n] - vdd_calc_steps;

    }
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
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }


  Serial.print("\n\n\n7) calculate sensitivity coefficients (pixc_ij):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {

      // --- SENSITIVITY ---
      // multiply sensitivity coeff for each pixel (see datasheet, chapter: 11.5 Object Temperature)
      vij_pixc_and_pcscaleval = (int64_t)data_pixel[m][n] * (int64_t)PCSCALEVAL;
      pixcij = (int32_t)pixcmax - (int32_t)pixcmin;
      pixcij = pixcij / 65535;
      pixcij = pixcij * pij[m][n];
      pixcij = pixcij + pixcmin;
      pixcij = pixcij * 1.0 / 100 * epsilon;
      pixcij = pixcij * 1.0 / 10000 * globalgain;
      data_pixel[m][n] =  (int32_t)(vij_pixc_and_pcscaleval / (int64_t)pixcij);

      Serial.print(pixcij);
      if (n == 79) {
        Serial.print("\n");
      }
      else {
        Serial.print("\t");
      }

    }
  }
  Serial.print("\n\n\n8) multiply scaling coeff and sensitivity coeff to compensated pixel voltages (V_ij_pixc):\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }


  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {

      // --- LOOKUPTABLE ---
      // find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter: 11.7 Look-up table)
      table_row = data_pixel[m][n] + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      data_pixel[m][n] = (uint32_t)((vy - vx) * ((int32_t)(data_pixel[m][n] + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

    }
  }

  Serial.print("\n\n\n9) calcluate final pixel temperature (in dK) with lookup table and bilinear interpolation:\n\n");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      Serial.print(data_pixel[m][n]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  pixel_masking();

  Serial.print("\n\n\n10) pixel masking (if there is a defect pixel):\n\n");

  if (nrofdefpix > 0) {
    for (int m = 0; m < sensor.number_row; m++) {
      for (int n = 0; n < sensor.number_col; n++) {
        Serial.print(data_pixel[m][n]);
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
