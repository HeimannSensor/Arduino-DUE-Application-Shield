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


// ETHERNET-COMMUNICATION
uint8_t mac[] = {0x2C, 0xF7, 0xF1, 0x08, 0x19, 0x6C};
unsigned int localPort = 30444;      // local port to listen on
EthernetUDP Udp;
uint8_t ip_partner[4];
uint8_t device_bind;


// PROGRAMM CONTROL
uint16_t timer_duration; // [µs]
uint8_t switch_ptat_vdd = 0;
uint8_t adr_offset = 0x00;
uint8_t send_data = 0;
uint8_t statusreg;
uint8_t picnum = 0;
uint8_t state = 0;
uint8_t read_block_num = 0;
uint8_t read_eloffset_next_pic = 0;
uint8_t gui_mode = 0;
uint8_t wait_pic = 0;



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
uint16_t data_pixel[16][16];

// CALCULATED VALUES
uint16_t ptat_av_uint16;
uint16_t vdd_av_uint16;
uint16_t ambient_temperature;
int32_t vij_pixc_int32[16][16];
uint32_t temp_pix_uint32[16][16];

// OTHER
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;

uint32_t timea, timeb;





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


  // read ip from eeprom
  IPAddress ip((eeprom_read_routine(E_IP) & 0xFF00) >> 8,
               eeprom_read_routine(E_IP) & 0x00FF,
               (eeprom_read_routine(E_IP1) & 0xFF00) >> 8,
               eeprom_read_routine(E_IP1) & 0x00FF);
  IPAddress subnet((eeprom_read_routine(E_SUBNET) & 0xFF00) >> 8,
                   eeprom_read_routine(E_SUBNET) & 0x00FF,
                   (eeprom_read_routine(E_SUBNET1) & 0xFF00) >> 8,
                   eeprom_read_routine(E_SUBNET1) & 0x00FF);
  IPAddress myDns(ip[0], ip[1], ip[2], 1);
  IPAddress gateway(ip[0], ip[1], ip[2], 1);



  // look for dhcp. If there's no dhcp -> use eeprom ip
  Serial.print("\nask dhcp for ip: ");
  Ethernet.begin(mac);
  if (Ethernet.localIP()[0] > 0) {
    Serial.print("ok -> ip: ");
    Serial.print(Ethernet.localIP()[0]);
    Serial.print(".");
    Serial.print(Ethernet.localIP()[1]);
    Serial.print(".");
    Serial.print(Ethernet.localIP()[2]);
    Serial.print(".");
    Serial.print(Ethernet.localIP()[3]);
  }
  else {
    Ethernet.begin(mac, ip, myDns, gateway, subnet);
    Serial.print("fail -> read default ip from eeprom: ");
    Serial.print(ip[0]);
    Serial.print(".");
    Serial.print(ip[1]);
    Serial.print(".");
    Serial.print(ip[2]);
    Serial.print(".");
    Serial.print(ip[3]);
  }

  Udp.begin(localPort);






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


  // config Timer on Adruino Due (need lib DueTimer.h)
  Timer3.attachInterrupt(readblockinterrupt);
  Timer3.start(timer_duration); // Calls every 31ms
  Serial.print("\nsetup done -> GUI");

  // ERROR TABLENUMBER
  if (tablenumber != TABLENUMBER) {
    Serial.print("\n\nHINT:\tConnected Sensor does not match the selected look up table.");
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
   Function:        void readblockinterrupt()

   Description:     read one sensor block and change configuration register to next block
                    (also read electrical offset when read_eloffset_next_pic is set)

   Dependencies:    current number to read (read_block_num)
                    marker to read electrical offset after last block (read_eloffset_next_pic)
                    number of complete pictures (picnum)
 *******************************************************************/
void readblockinterrupt() {

  Timer3.stop();
  // wait for end of conversion bit (~27ms)
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }


  if (read_block_num == 0) {
    // read block 0 of top half and block 0 of bottom half
    read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 130);
    read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block0, 130);
    // change block in configuration register (to block 1)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  1  |   1   |    x     |   0   |    1   |
    // x is 1 if adr_offset is active
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (0x19 + adr_offset));
  }

  if (read_block_num == 1) {
    // read block 1 of top half and block 1 of bottom half
    read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block1, 130);
    read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block1, 130);
    if (read_eloffset_next_pic == 1) {
      // change block in configuration register (to el.offset)
      // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
      // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
      write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (0x0B));
    }
  }



  if (read_block_num == 2) {
    // read block 0 of top half and block 0 of bottom half
    read_sensor_register( TOP_HALF, (uint8_t*)&electrical_offset_top, 130);
    read_sensor_register( BOTTOM_HALF, (uint8_t*)&electrical_offset_bottom, 130);
  }




  read_block_num++;

  if ( (read_eloffset_next_pic == 0 && read_block_num == 2) || (read_eloffset_next_pic == 1 && read_block_num == 3)) {
    state = 1;
    read_block_num = 0;
    picnum++;

    // read vdd at next picture
    if (switch_ptat_vdd == 0) {
      switch_ptat_vdd = 1;
      adr_offset = 0x04;
    }
    else {
      // read ptat at next picture
      switch_ptat_vdd = 0;
      adr_offset = 0x00;
    }

    // read electrical offset at next picture
    if (picnum == 10) {
      read_eloffset_next_pic = 1;
      picnum = 0;
    }
    else {
      read_eloffset_next_pic = 0;
    }

    // change block in configuration register (to block 0)
    // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   1   |    x     |   0   |    1   |
    // x is 1 if adr_offset is active
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (0x09 + adr_offset));



  }

  Timer3.start(timer_duration);


}














/********************************************************************
   Function:        void loop()

   Description:     main loop of an arduino program

   Dependencies:
 *******************************************************************/
void loop() {


  /*
     state is the state in this "state machine"
        0... reading new data and pic is not complete
        1... reading done (set in ISR readblockinterrupt)
        2... raw data blocks sorted
        3... pixel temps calculated
  */

  switch (state) {

    // ---IDLE---
    case 0:
      // do nothing
      break;

    // ---SORT---
    case 1:
      sort_data();
      state++;
      break;

    // ---CALC---
    case 2:
      // only when sending udp packets with temperatures
      if (send_data == 1) {
        calculate_pixel_temp();
        pixel_masking();
      }
      state++;
      break;

    // ---SEND---
    case 3:
      send_udp_packets();
      state = 0;
      break;

  }

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

  int32_t vij_comp_int32[sensor.number_row][sensor.number_col];
  int32_t vij_comp_s_int32[sensor.number_row][sensor.number_col];
  int32_t vij_vddcomp_int32[sensor.number_row][sensor.number_col];
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
    //Serial.print("\n");
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




  if (switch_ptat_vdd) {
    // if switch_ptat_vdd = 1 -> vdd is set current pic -> ptat was measured in last pic

    // new ptat values (1st and 2nd byte at every data block)

    // top
    ptat_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
    ptat_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
    // bottom
    ptat_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
    ptat_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];


    // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
    sum = ptat_top_block0 + ptat_top_block1 + ptat_bottom_block0 + ptat_bottom_block1;
    ptat_av_uint16 = sum / 4;

    // calculate ambient_temperature (datasheet, chapter: 11.1 Ambient Temperature )
    ambient_temperature = ptat_av_uint16 * ptatgr_float + ptatoff_float;

  }
  else {
    // if switch_ptat_vdd = 0 -> ptat is set current pic -> vdd was measured in last pic

    // new vdd values (1st and 2nd byte at every data block)
    uint16_t vdd_top_block0, vdd_top_block1, vdd_bottom_block0, vdd_bottom_block1;

    // top
    vdd_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
    vdd_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
    // bottom
    vdd_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
    vdd_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];

    // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
    sum = vdd_top_block0 + vdd_top_block1 + vdd_bottom_block0 + vdd_bottom_block1;
    vdd_av_uint16 = sum / 4;

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

   the following function is used to chat with the GUI:
      "Heimann Sensor ArraySoft v2"

 *******************************************************************/


























/********************************************************************
   Function:        void send_udp_packets()

   Description:

   Dependencies:
 *******************************************************************/
void send_udp_packets() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  char packetBuffer[1000] = {""};
  // strings with unknown parts:
  char packetChangeIP[] = {"HTPA device IP change request to "};
  char packetChangeEPSILON[] = {"Set Emission to "};
  uint8_t change_ip, change_epsilon, change_id;


  if (packetSize) {
    IPAddress remote = Udp.remoteIP();

    /*  debug
      Serial.print("Received packet of size ");
      Serial.println(packetSize);

      Serial.print("From ");


      for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
      }

      Serial.print(", port ");
      Serial.println(Udp.remotePort());
    */

    if ( (ip_partner[0] == Udp.remoteIP()[0] &&
          ip_partner[1] == Udp.remoteIP()[1] &&
          ip_partner[2] == Udp.remoteIP()[2] &&
          ip_partner[3] == Udp.remoteIP()[3]) || device_bind == 0) {

      // read the packet into packetBufffer
      Udp.read(packetBuffer, 1000);
    }

    else {
      return;
    }


    // ----------------------------------------------------
    // HTPA RESPONSED
    if (strcmp(packetBuffer, "Calling HTPA series devices") == 0) {
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("HTPA series responsed! I am Arraytype 01");
      //Udp.print(arraytype);
      Udp.write(" MODTYPE 005\r\nADC: ");
      Udp.print( (mbit_calib & 15) + 4);    // calc ADC resolution
      Udp.write("\r\n");
      Udp.write("HTPA16x16d v.0.01 Heimann Sensor GmbH; written by D. Pauer 2019-11-21\r\n");
      Udp.write("I am running on ");
      float clk_float = 12000000 / 63 * clk_calib + 1000000;    // calc clk in MHz
      Udp.print(clk_float / 1000, 1); // print clk in kHz
      Udp.write(" kHz\r\n");
      Udp.write("MAC-ID: ");
      for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) {
          Udp.write("0");
        }
        Udp.print(mac[i], HEX);
        if (i < 5) {
          Udp.write(".");
        }
      }
      Udp.write(" IP: ");
      for (int i = 0; i < 4; i++) {

        if (Ethernet.localIP()[i] < 10) {
          Udp.write("00");
        }
        else if (Ethernet.localIP()[i] < 100) {
          Udp.write("0");
        }
        Udp.print(Ethernet.localIP()[i]);
        if (i < 3) {
          Udp.write(".");
        }
      }
      Udp.write(" DevID: ");
      for (int i = 1; i < 9; i++) {
        if (id < pow(10, i)) {
          Udp.write("0");
        }
      }
      Udp.print(id);
      Udp.endPacket();

    }


    // ----------------------------------------------------
    // CHANGE IP AND SUBNET
    if (packetSize > 33) {
      change_ip = 1;
      // compare the first position of string
      for (int i = 0; i < 33; i++) {
        if (packetBuffer[i] != packetChangeIP[i]) {
          change_ip = 0;

        }
      }

      if (change_ip) {
        send_data = 0;
        Timer3.stop();

        byte new_ip[4];
        byte new_subnet[4];

        new_ip[0] = (int)(packetBuffer[33] - '0') * 100 + (int)(packetBuffer[34] - '0') * 10 + (int)(packetBuffer[35] - '0');
        new_ip[1] = (int)(packetBuffer[37] - '0') * 100 + (int)(packetBuffer[38] - '0') * 10 + (int)(packetBuffer[39] - '0');
        new_ip[2] = (int)(packetBuffer[41] - '0') * 100 + (int)(packetBuffer[42] - '0') * 10 + (int)(packetBuffer[43] - '0');
        new_ip[3] = (int)(packetBuffer[45] - '0') * 100 + (int)(packetBuffer[46] - '0') * 10 + (int)(packetBuffer[47] - '0');
        new_subnet[0] = (int)(packetBuffer[49] - '0') * 100 + (int)(packetBuffer[50] - '0') * 10 + (int)(packetBuffer[51] - '0');
        new_subnet[1] = (int)(packetBuffer[53] - '0') * 100 + (int)(packetBuffer[54] - '0') * 10 + (int)(packetBuffer[55] - '0');
        new_subnet[2] = (int)(packetBuffer[57] - '0') * 100 + (int)(packetBuffer[58] - '0') * 10 + (int)(packetBuffer[59] - '0');
        new_subnet[3] = (int)(packetBuffer[61] - '0') * 100 + (int)(packetBuffer[62] - '0') * 10 + (int)(packetBuffer[63] - '0');


        Serial.print("\n");
        Serial.print(new_ip[0]);
        Serial.print("\t");
        Serial.print(new_ip[1]);
        Serial.print("\t");
        Serial.print(new_ip[2]);
        Serial.print("\t");
        Serial.print(new_ip[3]);
        Serial.print("\n");


        // write new ip to eeprom
        write_eeprom_routine(E_IP, new_ip[0] << 8 | new_ip[1]);
        delay(10);
        write_eeprom_routine(E_IP1, new_ip[2] << 8 | new_ip[3]);
        delay(10);

        // write new subnet to eeprom
        write_eeprom_routine(E_SUBNET,  new_subnet[0] << 8 | new_subnet[1]);
        delay(10);
        write_eeprom_routine(E_SUBNET1,  new_subnet[2] << 8 | new_subnet[3]);
        delay(10);



        Udp.beginPacket(ip_partner, Udp.remotePort());
        Udp.write("Device changed IP to ");
        for (int i = 0; i < 4; i++) {
          if (new_ip[i] < 10) {
            Udp.write("00");
          } else if (new_ip[i] < 100) {
            Udp.write("0");
          }
          Udp.print(new_ip[i]);
          Udp.write(".");
        }

        Udp.write(" and Subnet to ");

        for (int i = 0; i < 4; i++) {
          if (new_subnet[i] < 10) {
            Udp.write("00");
          } else if (new_subnet[i] < 100) {
            Udp.write("0");
          }
          Udp.print(new_subnet[i]);
          Udp.write(".");
        }
        Udp.write("\r\n");
        Udp.endPacket();


        IPAddress new_ip1(new_ip[0], new_ip[1], new_ip[2], new_ip[3]);
        IPAddress new_subnet1(new_subnet[0], new_subnet[1], new_subnet[2], new_subnet[3]);
        IPAddress new_myDns1(new_ip[0], new_ip[1], new_ip[2], 1);
        IPAddress new_gateway1(new_ip[0], new_ip[1], new_ip[2], 1);

        delay(100);


        Udp.stop();
        Ethernet.begin(mac, new_ip1, new_myDns1, new_gateway1, new_subnet1);
        delay(2000);
        Udp.begin(localPort);
        delay(2000);


        Timer3.start(timer_duration);
      }
    }






    // ----------------------------------------------------
    // SEND IP AND MAC (HW FILTER)
    if (strcmp(packetBuffer, "Bind HTPA series device") == 0) {
      if (device_bind == 0) {
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("HW Filter is ");
        for (int i = 0; i < 4; i++) {

          if (Ethernet.localIP()[i] < 10) {
            Udp.write("00");
          }
          else if (Ethernet.localIP()[i] < 100) {
            Udp.write("0");
          }
          Udp.print(Ethernet.localIP()[i]);
          if (i < 3) {
            Udp.write(".");
          }
        }
        Udp.write(" MAC ");
        for (int i = 0; i < 6; i++) {
          if (mac[i] < 0x10) {
            Udp.write("0");
          }
          Udp.print(mac[i], HEX);
          if (i < 5) {
            Udp.write(".");
          }
        }
        Udp.write("\n\r");
        Udp.endPacket();

        device_bind = 1;
        for (int i = 0; i < 4; i++) {
          ip_partner[i] = Udp.remoteIP()[i];
        }

      }
      else {
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("Device already bound\n\r");
        Udp.endPacket();
      }
    }


    // ----------------------------------------------------
    // CHANGE EPSILON
    if (packetSize > 16) {
      change_epsilon = 1;
      // compare the first position of string
      for (int i = 0; i < 16; i++) {
        if (packetBuffer[i] != packetChangeEPSILON[i]) {
          change_epsilon = 0;
        }
      }
      if (change_epsilon) {
        send_data = 0;
        Timer3.stop();
        epsilon = (int)(packetBuffer[16] - '0') * 100 + (int)(packetBuffer[17] - '0') * 10 + (int)(packetBuffer[18] - '0');


        // write new epsilon to eeprom
        write_eeprom_routine(E_EPSILON, epsilon);

        // calculate pixcij with new epsilon
        calculate_pixcij();


        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("Emission changed to ");
        Udp.print(epsilon);
        Udp.write("%\r\n\r\n");
        Udp.endPacket();

        delay(1000);
        Timer3.start(timer_duration);
      }
    }


    // ----------------------------------------------------
    //USER SETTING
    if (strcmp(packetBuffer, "G") == 0) {
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("HTPA32x32d 2019/11/31 v.0.01 Heimann Sensor GmbH; written by D. Pauer\n\r");
      Udp.write("BIAS: ");
      if (bias_user < 0x10) {
        Udp.write("0");
      }
      Udp.print(bias_user, HEX);
      Udp.write("Clock: ");
      if (clk_calib < 0x10) {
        Udp.write("0");
      }
      Udp.print(clk_user, HEX);
      Udp.write("MBIT: ");
      if (mbit_user < 0x10) {
        Udp.write("0");
      }
      Udp.print(mbit_user, HEX);
      Udp.write("BPA: ");
      if (bpa_user < 0x10) {
        Udp.write("0");
      }
      Udp.print(bpa_user, HEX);
      Udp.write("PU: ");
      if (pu_user < 0x10) {
        Udp.write("0");
      }
      Udp.print(pu_user, HEX);
      Udp.write("GlobalOffset: 0");
      //Udp.print(globaloff, HEX);
      Udp.write("GlobalGain: ");
      Udp.print(globalgain, HEX);

      Udp.endPacket();

    }


    // ----------------------------------------------------
    // DECREAS/INCREASE CLK
    if (strcmp(packetBuffer, "a") == 0 || strcmp(packetBuffer, "A") == 0 ) {

      Timer3.stop();

      if (strcmp(packetBuffer, "a") == 0  && clk_user > 0) {
        clk_user = clk_user - 1;
      }
      if (strcmp(packetBuffer, "A") == 0 && clk_user < 63) {
        clk_user = clk_user + 1;
      }


      write_eeprom_routine(E_CLK_USER, clk_user);
      delay(5);

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("MHZClk is ");
      float clk_float = 12000000 / 63 * clk_user + 1000000;    // calc clk in MHz

      Udp.print(clk_float / 1000, 1);
      Udp.write(" kHz\r\n");
      Udp.endPacket();

      send_data = 0;

      Timer3.start(timer_duration);
    }

    // ----------------------------------------------------
    // DECREAS/INCREASE BIAS
    if (strcmp(packetBuffer, "i") == 0 || strcmp(packetBuffer, "I") == 0 ) {

      Timer3.stop();

      if (strcmp(packetBuffer, "i") == 0 && bias_user > 0) {
        bias_user = bias_user - 1;
      }
      if (strcmp(packetBuffer, "I") == 0 && bias_user < 31) {
        bias_user = bias_user + 1;
      }


      write_eeprom_routine(E_BIAS_USER, bias_user);
      delay(5);

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("BIAS-Trim: ");
      Udp.print(bias_user, HEX);
      Udp.write("\r\n");
      Udp.endPacket();

      send_data = 0;
      Timer3.start(timer_duration);
    }


    // ----------------------------------------------------
    // DECREAS/INCREASE BPA
    if (strcmp(packetBuffer, "j") == 0 || strcmp(packetBuffer, "J") == 0 ) {

      Timer3.stop();

      // write new subnet to eeprom
      if (strcmp(packetBuffer, "j") == 0 && bpa_user > 0) {
        bpa_user = bpa_user - 1;
      }
      if (strcmp(packetBuffer, "J") == 0 && bpa_user < 31) {
        bpa_user = bpa_user + 1;
      }

      write_eeprom_routine(E_BPA_USER, bpa_user);
      delay(5);

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("BPA-Trim: ");
      Udp.print(bpa_user, HEX);
      Udp.write("\r\n");
      Udp.endPacket();

      send_data = 0;
      Timer3.start(timer_duration);
    }


    // ----------------------------------------------------
    // DECREAS/INCREASE MBIT
    if (strcmp(packetBuffer, "r") == 0 ||
        strcmp(packetBuffer, "R") == 0 ||
        strcmp(packetBuffer, "o") == 0 ||
        strcmp(packetBuffer, "O") == 0 ) {

      Timer3.stop();

      uint8_t adc_res, adc_ref;
      adc_res = mbit_user & 15;
      adc_ref = (mbit_user & 48) >> 4;


      if (strcmp(packetBuffer, "r") == 0 && adc_res > 4) {
        adc_res = adc_res - 1;
      }
      if (strcmp(packetBuffer, "R") == 0 && adc_res < 12) {
        adc_res = adc_res + 1;
      }
      if (strcmp(packetBuffer, "o") == 0 && adc_ref > 0) {
        adc_ref = adc_ref - 1;
      }
      if (strcmp(packetBuffer, "O") == 0 && adc_ref < 3) {
        adc_ref = adc_ref + 1;
      }

      mbit_user = adc_ref << 4 | adc_res;
      Serial.print("\n");
      Serial.print(adc_res);
      Serial.print("\n");
      Serial.print(adc_ref);
      delay(5);

      if (strcmp(packetBuffer, "r") == 0 ||
          strcmp(packetBuffer, "R") == 0 ) {
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("Resolution: ");
        if (adc_res + 4 < 10) {
          Udp.write("0");
        }
        Udp.print(adc_res + 4);
        Udp.write(" bit\r\n");
        Udp.endPacket();
      }

      if (strcmp(packetBuffer, "o") == 0 ||
          strcmp(packetBuffer, "O") == 0 ) {
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("ADC-Ref: ");
        if (adc_ref < 10) {
          Udp.write("0");
        }
        Udp.print(adc_ref);
        Udp.write(" bit\r\n");
        Udp.endPacket();
      }

      write_eeprom_routine(E_MBIT_USER, mbit_user);

      send_data = 0;
      Timer3.start(timer_duration);
    }

    // ----------------------------------------------------
    // DECREAS/INCREASE PU
    if (strcmp(packetBuffer, "p") == 0) {

      Timer3.stop();

      if (pu_user == 17) {
        pu_user = 34;
      }
      else if (pu_user == 34) {
        pu_user = 68;
      }
      else if (pu_user == 68) {
        pu_user = 136;
      }
      else if (pu_user == 136) {
        pu_user = 17;
      }
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("PU-Trim: ");
      Udp.print(pu_user, HEX);
      Udp.write("\r\n");
      Udp.endPacket();

      write_eeprom_routine(E_PU_USER, pu_user);

      send_data = 0;
      Timer3.start(timer_duration);
    }

    // ----------------------------------------------------
    //HW RELEASED
    if (strcmp(packetBuffer, "x Release HTPA series device") == 0) {
      send_data = 0;
      device_bind = 0;
      for (int i = 0; i < 4; i++) {
        ip_partner[i] = 0;
      }
      Udp.beginPacket(ip_partner, Udp.remotePort());
      Udp.write("HW-Filter released\r\n");
      Udp.endPacket();
    }



    // ----------------------------------------------------
    // SEND DATA (TEMPS)
    if (strcmp(packetBuffer, "K") == 0) {
      if (gui_mode == 1) {
        Timer3.stop();
        // write user calobration to sensor
        write_calibration_settings_to_sensor();
        delay(100);
        timer_duration = calc_timer_duration(bw, clk_calib, mbit_calib);
        Timer3.start(timer_duration);
        gui_mode = 0;
        wait_pic = 0;
      }
      send_data = 1;
    }


    // ----------------------------------------------------
    // SEND DATA (VOLTAGE)
    if (strcmp(packetBuffer, "t") == 0) {
      Timer3.stop();
      // write user settings to sensor
      write_user_settings_to_sensor();
      delay(100);
      timer_duration = calc_timer_duration(bw, clk_user, mbit_user);
      Timer3.start(timer_duration);
      gui_mode = 1;
      send_data = 2;
      wait_pic = 0;
    }

    // ----------------------------------------------------
    // STOP SENDING
    if (strcmp(packetBuffer, "x") == 0 || strcmp(packetBuffer, "X") == 0) {
      send_data = 0;
      device_bind = 0;
      wait_pic = 0;
    }


  }





  // send data
  if (send_data > 0) {
    // convert data in 2 udp packets


    uint8_t packet1[790];
    int p = 0;
    for (int m = 0; m < sensor.number_row; m++) {
      for (int n = 0; n < sensor.number_col; n++) {

        if (send_data == 1) {
          packet1[p] = ((uint16_t) temp_pix_uint32[m][n])  & 0x00ff; // low byte
          p++;
          packet1[p] = (((uint16_t) temp_pix_uint32[m][n])  & 0xff00) >> 8; // high byte
          p++;
        }

        if (send_data == 2) {
          packet1[p] = ((uint16_t) data_pixel[m][n])  & 0x00ff; // low byte
          p++;
          packet1[p] = (((uint16_t) data_pixel[m][n])  & 0xff00) >> 8; // high byte
          p++;
        }

      }
    }

    for (int m = 0; m < 8; m++) {
      for (int n = 0; n < sensor.number_col; n++) {
        packet1[p] = eloffset[m][n] & 0x00ff; // low byte
        p++;
        packet1[p] = (eloffset[m][n] & 0xff00) >> 8; // high byte
        p++;
      }

    }

    packet1[768] = ((uint16_t) vdd_av_uint16) & 0x00ff; // low byte
    packet1[769] = (((uint16_t) vdd_av_uint16) & 0xff00) >> 8; // high byte
    packet1[770] = ((uint16_t) ambient_temperature) & 0x00ff; // low byte
    packet1[771] = (((uint16_t) ambient_temperature) & 0xff00) >> 8; // high byte
    packet1[772] = ptat_top_block0 & 0x00ff; // low byte
    packet1[773] = (ptat_top_block0 & 0xff00) >> 8; // high byte
    packet1[774] = ptat_top_block1 & 0x00ff; // low byte
    packet1[775] = (ptat_top_block1 & 0xff00) >> 8; // high byte
    packet1[776] = ptat_bottom_block0 & 0x00ff; // low byte
    packet1[777] = (ptat_bottom_block0 & 0xff00) >> 8; // high byte
    packet1[778] = ptat_bottom_block1 & 0x00ff; // low byte
    packet1[779] = (ptat_bottom_block1 & 0xff00) >> 8; // high byte



    if (wait_pic > 5) {
      // packet 1
      //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.beginPacket(ip_partner, Udp.remotePort());
      Udp.write(packet1, 780);
      Udp.endPacket();


    }
    else
    {
      wait_pic++;
    }
  }


}
