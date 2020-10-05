# Bpod Stepper Motor Module
A stepper motor module for the Bpod State Machine r2.

![](images/board.png)


## State Machine Command Interface
* **'A' / Ascii 65: set acceleration** (steps / s<sup>2</sup>)  
    Must be followed by one Int16:
  * Byte 1: acceleration (least significant byte),
  * Byte 2: acceleration (most significant byte).
* **'V' / Ascii 86: set maximum velocity** (steps / s)  
    Must be followed by one Int16:
  * Byte 1: velocity (least significant byte),
  * Byte 2: velocity (most significant byte).
* **'S' / Ascii 83: move by a number of steps** (steps)  
  Must be followed by one Int16:
  * Byte 1: number of steps (least significant byte),
  * Byte 2: number of steps (most significant byte).
  
  Positive numbers will result in clockwise, negative numbers in counter-clockwise rotation.
* **'D' / Ascii 68: move by a defined angle** (degrees)  
  Must be followed by one Int16:
  * Byte 1: angle (least significant byte),
  * Byte 2: angle (most significant byte).
  
  Positive numbers will result in clockwise, negative numbers in counter-clockwise rotation.
* **'L' / Ascii 76: search limit switch**  
  Must be followed by two bytes:
  * Byte 1: specifies the limit switch to monitor (1 or 2), 
  * Byte 2: specifies the movement direction (0 = CCW, 1 = CW)
  
  This will advance the motor at constant, low speed until one of the limit switches has been reached.
* **Byte 255: return module info** (reserved)


## Bill of Materials
| Item     | Vendor   | Qty | Part Number                                                                                     | Description                 |
| :------- | :------  | :-: | :---------------------------------------------------------------------------------------------- | :-------------------------  |
| IC1      | Digi-Key |  1  | [LM3480IM3-5.0/NOPBCT-ND](https://www.digikey.com/products/en?keywords=LM3480IM3-5.0/NOPBCT-ND) | 5V regulator                |
| IC2      | Digi-Key |  1  | [ADM3077EARZ-ND](https://www.digikey.com/products/en?keywords=ADM3077EARZ-ND)                   | RS-485 IC                   |
| C1, C5   | Digi-Key |  2  | [PCE3203TR-ND](https://www.digikey.com/products/en?keywords=PCE3203TR-ND)                       | Aluminium capacitor, 100 µF |
| C2 - C4  | Digi-Key |  3  | [311-1179-1-ND](https://www.digikey.com/products/en?keywords=311-1179-1-ND)                     | Ceramic capacitor, 0.1 µf   |
| D1       | Digi-Key |  1  | [MBR0520LCT-ND](https://www.digikey.com/products/en?keywords=MBR0520LCT-ND)                     | Schottky diode, 20V 500mA   |
| D2       | Digi-Key |  1  | [1KSMB75CACT-ND](https://www.digikey.com/products/en?keywords=1KSMB75CACT-ND)                   | TVS diode                   |
| D3 - D11 | Digi-Key |  9  | [S310FACT-ND](https://www.digikey.com/products/en?keywords=S310FACT-ND)                         | Schottky diode, 100V 3A     |
|          | Digi-Key |  1  | [A31442-ND](https://www.digikey.com/products/en?keywords=A31442-ND)                             | Ethernet jack               |
|          | Digi-Key |  1  | [839-1512-ND](https://www.digikey.com/products/en?keywords=839-1512-ND)                         | DC barrel jack              |
|          | Digi-Key |  1  | [S9001-ND](https://www.digikey.com/products/en?keywords=S9001-ND)                               | Jumper                      |
|          | Digi-Key |  1  | [S1011EC-02-ND](https://www.digikey.com/products/en?keywords=S1011EC-02-ND)                     | Male header, 1x2            |
|          | Digi-Key |  2  | [PPPC081LFBN-RC](https://www.digikey.com/products/en?keywords=PPPC081LFBN-RC)                   | Female header, 1x8          |
|          | Digi-Key |  2  | [PPPC241LFBN-RC](https://www.digikey.com/products/en?keywords=PPPC241LFBN-RC)                   | Female header, 1x24         |
|          | Digi-Key |  1  | [1568-1465-ND](https://www.digikey.com/products/en?keywords=1568-1465-ND)                       | Teensy 3.6                  |
|          | Digi-Key |  1  | [2100-20150007-002-ND](https://www.digikey.com/products/en?keywords=2100-20150007-002-ND)       | Stepper motor driver        |
|          | Digi-Key |  1  | [2100-201835-ND](https://www.digikey.com/products/en?keywords=2100-201835-ND)                   | Heat sink                   |
|          | Digi-Key |  2  | [A98334-ND](https://www.digikey.com/products/en?keywords=A98334-ND)                             | Terminal block, 1x3         |
|          | Digi-Key |  1  | [A98081-ND](https://www.digikey.com/products/en?keywords=A98081-ND)                             | Terminal block, 1x4         |


## Credits ##
* Concept & firmware by Florian Rau
* PCB layout by Christopher Schultz and Florian Rau
* PCB layout partially based on:
  * [Bpod Teensy Shield](https://github.com/sanworks/Bpod-CAD/tree/master/PCB/Modules/Gen2/Bpod%20Teensy%20Shield) by Sanworks ([GPL v3](https://www.gnu.org/licenses/gpl-3.0.en.html))
  * [SilentStepStick Protector](https://github.com/watterott/SilentStepStick) by Watterott ([CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/))
* Firmare uses the following libraries:
  * [ArCOM](https://github.com/sanworks/ArCOM) by Sanworks ([GPL v3](https://www.gnu.org/licenses/gpl-3.0.en.html))
  * [SmoothStepper](https://github.com/bimac/SmoothStepper) by Florian Rau ([GPL v3](https://www.gnu.org/licenses/gpl-3.0.en.html))
