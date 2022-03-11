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


// ETHERNET-COMMUNICATION
uint8_t mac[] = {0x2C, 0xF7, 0xF1, 0x08, 0x19, 0x6C}; // change to your device
unsigned int localPort = 30444;      // local port to listen on
EthernetUDP Udp;
uint8_t ip_partner[4];
uint8_t device_bind;

// PROGRAMM CONTROL
uint16_t timer_duration = 28000; // [µs]
uint8_t switch_ptat_vdd = 0;
uint8_t adr_offset = 0x00;
uint8_t send_data = 0;
uint8_t statusreg;
uint8_t picnum = 0;
uint8_t state = 0;
uint8_t read_block_num = 0;
uint8_t read_eloffset_next_pic = 1;
uint8_t gui_mode = 0;
uint8_t wait_pic = 0;

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

// CALCULATED VALUES
uint16_t ambient_temperature;
int32_t vij_pixc_int32[8][8];
uint32_t temp_pix_uint32[8][8];

// OTHER
uint32_t gradscale_div;





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

  // config Timer on Adruino Due (need lib DueTimer.h)
  Timer3.attachInterrupt(readblockinterrupt);
  Timer3.start(timer_duration); // Calls every 31ms
  Serial.print("\nsetup done -> GUI");

  // ERROR TABLENUMBER
  if (tablenumber != TABLENUMBER) {
    Serial.print("\n\nHINT:\tConnected sensor does not match the selected look up table.");
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
                    (also read electrical offset all read_eloffset_next_pic is set)

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

    read_sensor_register( ARRAY_DATA, (uint8_t*)&data_array, 130);
    if (read_eloffset_next_pic == 1) {
      // change block in configuration register (to el.offset)
      // |  7  |  6  |  5  |  4  |   3   |   2   |   1   |    0   |
      // |          RFU          | Start |  VDD  | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |   1   |   1   |    1   |
      write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x0B + 0x04);
    }
    else {
      // change block in configuration register (to data)
      // |  7  |  6  |  5  |  4  |   3   |   2   |   1   |    0   |
      // |          RFU          | Start |  VDD  | BLIND | WAKEUP |
      // |  0  |  0  |  0  |  0  |   1   |   0   |   1   |    1   |
      write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);
    }
  }

  if (read_block_num == 1) {
    read_sensor_register( ARRAY_DATA, (uint8_t*)&electrical_offset, 130);
    // change block in configuration register (to data)
    // |  7  |  6  |  5  |  4  |   3   |   2   |   1   |    0   |
    // |          RFU          | Start |  VDD  | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   1   |   0   |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09 );
  }

  read_block_num++;

  if ( (read_eloffset_next_pic == 0 && read_block_num == 1) || (read_eloffset_next_pic == 1 && read_block_num == 2)) {
    state = 1;
    read_block_num = 0;
    picnum++;

    // read electrical offset next time
    if (picnum == 10) {
      read_eloffset_next_pic = 1;
      picnum = 0;
    }
    else {
      read_eloffset_next_pic = 0;
    }
  }

  Timer3.start(timer_duration);

}





/********************************************************************
   Function:        void loop()

   Description:     main loop of an arduino programm

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
      calculate_pixel_temp();
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
   Function:        calculate_pixel_temp()

   Description:     compensate thermal and electrical offset and multiply sensitivity coeff
                    look for the correct temp in lookup table

   Dependencies:
 *******************************************************************/
void calculate_pixel_temp() {

  int32_t vij_comp_int32[sensor.number_row][sensor.number_col];
  int32_t vij_comp_s_int32[sensor.number_row][sensor.number_col];
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
      Udp.write("HTPA series responsed! I am Arraytype 00");
      //Udp.print(arraytype);
      Udp.write(" MODTYPE 005\r\nADC: ");
      Udp.print( (mbit_calib & 15) + 4);    // calc ADC resolution
      Udp.write("\r\n");
      Udp.write("HTPA8x8d v.0.01 Heimann Sensor GmbH; written by D. Pauer 2019-11-26\r\n");
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
    //SEND EEPROM CONTENT
    if (strcmp(packetBuffer, "h") == 0) {
      send_data = 0;
      Timer3.stop();
      uint8_t eeprom_content[512];
      for (int i = 0; i < 256; i++) {
        eeprom_content[2 * i]   =  (eeprom_read_routine(i) & 0xFF00) >> 8;
        eeprom_content[2 * i + 1] =  eeprom_read_routine(i) & 0x00FF;
      }
      Udp.beginPacket(ip_partner, Udp.remotePort());
      Udp.write(eeprom_content, 512);
      Udp.endPacket();

      Timer3.start(timer_duration);
      gui_mode = 0;
      wait_pic = 0;


    }

    // ----------------------------------------------------
    // SEND DATA (TEMPS)
    if (strcmp(packetBuffer, "K") == 0) {
      if (gui_mode == 1) {
        Timer3.stop();
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


    uint8_t packet1[262];
    int p = 0;
    for (int m = 0; m < 8; m++) {
      for (int n = 0; n < 8; n++) {

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
      for (int n = 0; n < 8; n++) {
        packet1[p] = eloffset[m][n] & 0x00ff; // low byte
        p++;
        packet1[p] = (eloffset[m][n] & 0xff00) >> 8; // high byte
        p++;
      }

    }

    packet1[p] = ((uint16_t) vdd) & 0x00ff; // low byte
    packet1[p + 1] = (((uint16_t) vdd) & 0xff00) >> 8; // high byte
    packet1[p + 2] = ((uint16_t) ambient_temperature) & 0x00ff; // low byte
    packet1[p + 3] = (((uint16_t) ambient_temperature) & 0xff00) >> 8; // high byte
    packet1[p + 4] = ptat & 0x00ff; // low byte
    packet1[p + 5] = (ptat & 0xff00) >> 8; // high byte


    if (wait_pic > 5) {
      // packet 1
      Udp.beginPacket(ip_partner, Udp.remotePort());
      Udp.write(packet1, 262);
      Udp.endPacket();


    }
    else
    {
      wait_pic++;
    }
  }


}
