Hint: You will find the installation guide in the user_manual_ApplicationShieldArduino.pdf

# Arduino-DUE-Application-Shield

For thermal imaging and easy application of our arrays we designed an application board to read thermopile sensors with an Arduino board.

The source code is completely open and includes all required steps from reading the EEPROM and sensor values to calculation of the final temperature image. Therefore, the C++ sample code can be viewed and modified via the Arduino IDE. The PCB is designed as an Arduino extension and supports the board Arduino Due.

You can choose between two modes: Ethernet mode and Serial mode.

# Serial Mode

Thismode prints all results in the serial monitor of the Arduino IDE. Here the EEPROM content and sensor voltages can be visualized. The serial mode only reads new data on request, to show which calculation steps are required and in which order.
Benefits:
- shows EEPROM content in hexadecimal or associated data type (float, short, long, …)
- prints results after each calculation steps

![grafik](https://user-images.githubusercontent.com/59830049/157813718-4692e8d5-bdb7-471b-bb9a-92871382cc07.png)

# Ethernet Mode

In Ethernet mode you can connect an I2C sensor with the Heimann Sensor GUI to stream continuously. The sample code establishes via DHCP a connection or you can use your local network card. An Arduino Ethernet Shield and an Arduino Due are required. In the GUI you can stream the sensor images in temperature or voltage mode. Also, you can change user settings, like clock, ADC resolution and emissivity factor.
Benefits:
- false color visualization of images
- stream continuously
- switch between temperature and voltagemode
- record/replay
- change user settings

![grafik](https://user-images.githubusercontent.com/59830049/157813800-e90de335-d421-497f-b725-b8bd09323238.png)
