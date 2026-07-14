## Electronics

The system supervisor (SYS_SV), which is device that overseas all operations is **STM32G474**. This MCU is chosen as it has high clock system and FDCAN clock speed (170MHz) that is required to poll the motor data in high speed as to capture the dynamic changes of the leg. The communication protocol between SYS_SV and Moteus controller is CANFD with arbitration rate and data rate of 1Mbps and 5Mbps. Position commands and queries are exchanged between these two nodes. 
The leg will receive commands through a wireless Bluetooth handheld remote with ESP32 as a data proxy to the system supervisor. ESP32 will transfer UART data to SYS_SV whenever it receives a packet from PS5 controller, which the SYS_SV will handle it in a user-defined ISR function. An MPX5010 pressure sensor is read by SYS_SV via ADC GPIO and pneumatic valve is controlled by a signal pin.


The system operates using three voltage rails: 24 V, 5 V, and 3.3 V. A 240 VAC-to-24 VDC, 16.7 A wall power supply serves as the primary power source, supplying the moteus controller, pneumatic solenoid valve, and a DC-DC buck converter. An LM2596 buck converter steps the 24 V supply down to 5 V, which powers the SYS_SV rail of the STM32G474RE microcontroller, an ESP32, the MCP2562FD CAN transceiver, the TXS0108 logic level shifter, the MPX5010 air pressure sensor, and the external 5 V pull-up circuitry.

### Circuit Topology
![Circuit Topology](https://github.com/milotruck-shin/FYP/blob/main/electronics/circuit%20topology.png)

for more detailed schematics, refer to **/electronics**
## Mechanical

The designed actuator consists of a 1:8 cycloidal gear drive, brushless motor, and a field-oriented controller mounted at the back of the actuator. Due to time constraints, off-the-shelves motors and motor controller will be used instead of custom making these 2 parts. The brushless motor to be used is Eaglepower 8303 90kV brushless drone motors which have 40 poles and could sustain a continuous power of 900W and continuous current of 20A. At idle, the motor draws up to 0.6A at 24V. The motor controller to be used are MJbots’s moteus r4.11 field-oriented-control (FOC) controller. 
