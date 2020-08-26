# Bpod_Stepper
A stepper motor module for the Bpod State Machine r2.

## Credits ##
* Concept & Firmware by Florian Rau
* PCB layout by Christopher Schultz and Florian Rau
* PCB layout partially based on:
  * [Bpod Teensy Shield](https://github.com/sanworks/Bpod-CAD/tree/master/PCB/Modules/Gen2/Bpod%20Teensy%20Shield) by Josh Sanders
  * [SilentStepStick Protector](https://github.com/watterott/SilentStepStick) by Watterott

## Bill of Materials
| Item     | Vendor   | Qty | Part Number                                                                                     | Description                 |
| :------- | :------  | :-: | :---------------------------------------------------------------------------------------------- | :-------------------------  |
| IC1      | Digi-Key |  1  | [LM3480IM3-5.0/NOPBCT-ND](https://www.digikey.com/products/en?keywords=LM3480IM3-5.0/NOPBCT-ND) | 5V regulator                |
| IC2      | Digi-Key |  1  | [ADM3077EARZ-ND](https://www.digikey.com/products/en?keywords=ADM3077EARZ-ND)                   | RS-485 IC                   |
| C1       | Digi-Key |  1  | [PCE3808CT-ND](https://www.digikey.com/products/en?keywords=PCE3808CT-ND)                       | Aluminium capacitor, 10 µF  |
| C2 - C4  | Digi-Key |  3  | [311-1179-1-ND](https://www.digikey.com/products/en?keywords=311-1179-1-ND)                     | Ceramic capacitor, 0.1 µf   |
| C5       | Digi-Key |  1  | [1189-1300-ND](https://www.digikey.com/products/en?keywords=1189-1300-ND)                       | Aluminium capacitor, 100 µF |
| D1       | Digi-Key |  1  | [MBR0520LCT-ND](https://www.digikey.com/products/en?keywords=MBR0520LCT-ND)                     | Schottky diode, 20V 500mA   |
| D2       | Digi-Key |  1  | [1KSMB75CACT-ND](https://www.digikey.com/products/en?keywords=1KSMB75CACT-ND)                   | TVS diode                   |
| D3 - D11 | Digi-Key |  9  | [S310FACT-ND](https://www.digikey.com/products/en?keywords=S310FACT-ND)                         | Schottky diode, 100V 3A     |
|          | Digi-Key |  1  | [A31442-ND](https://www.digikey.com/products/en?keywords=A31442-ND)                             | Ethernet jack               |
|          | Digi-Key |  1  | [732-5930-ND](https://www.digikey.com/products/en?keywords=732-5930-ND)                         | DC barrel jack              |
|          | Digi-Key |  1  | [S9001-ND](https://www.digikey.com/products/en?keywords=S9001-ND)                               | Jumper                      |
|          | Digi-Key |  1  | [S1011EC-02-ND](https://www.digikey.com/products/en?keywords=S1011EC-02-ND)                     | Male header, 1x2            |
|          | Digi-Key |  2  | [PPPC081LFBN-RC](https://www.digikey.com/products/en?keywords=PPPC081LFBN-RC)                   | Female header, 1x8          |
|          | Digi-Key |  2  | [PPPC241LFBN-RC](https://www.digikey.com/products/en?keywords=PPPC241LFBN-RC)                   | Female header, 1x24         |
|          | Digi-Key |  1  | [1568-1465-ND](https://www.digikey.com/products/en?keywords=1568-1465-ND)                       | Teensy 3.6                  |
|          | Digi-Key |  1  | [2100-20150007-002-ND](https://www.digikey.com/products/en?keywords=2100-20150007-002-ND)       | Stepper motor driver        |
|          | Digi-Key |  1  | [2100-201835-ND](https://www.digikey.com/products/en?keywords=2100-201835-ND)                   | Heat sink                   |
