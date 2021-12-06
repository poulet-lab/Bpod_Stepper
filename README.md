# Bpod Stepper Module

Combining smooth acceleration profiles with a _SilentStepStick_ driver, the _Bpod Stepper Module_ allows for virtually noiseless operation of a stepper motor - either as a module for _Bpod state machine r2_ or as a stand-alone USB device.

## Serial Command Interface

### Moving the Motor
The following serial commands control the movement of the motor. All movements can be interrupted by activation of an end-switch or issuance of a stop command. The Stepper Module will try to keep track of the position at all times (it should be able to do so unless there is a loss of steps). This way, you can target absolute positions.

* #### Move forwards
  Start continuous movement forwards.

      PUT 1 uInt8: 68 ('F')

* #### Move backwards
  Start continuous movement backwards.

      PUT 1 uInt8: 68 ('B')

* #### Move to a relative position
  Move a defined number of steps relative to the current position. Positive numbers will result in clockwise, negative numbers in counter-clockwise rotation.

      PUT 1 uInt8: 83 ('S')
      PUT 1 Int16: relative position [steps]

* #### Move to an absolute position

      PUT 1 uInt8: 80 ('P')
      PUT 1 Int16: absolut position [steps]


* #### Move to a predefined target

      PUT 1 uInt8: 1 … 9 [target ID]

* #### Get current position
  This will return the current position of the motor. The command can also be used to monitor ongoing movements.

      PUT 2 uInt8: 71, 80 ('GP')
      GET 1 Int16: absolute position [steps]

* #### Reset absolute position
  This command will reset the current position to zero (without moving the motor).

      PUT 1 uInt8: 90 ('Z')

* #### Bind forward rotation to IO port

      PUT 1 uInt8: 77 ('M')
      PUT 1 uInt8: 1 … 6 [IO port]
      PUT 1 uInt8: 70 ('F')

* #### Bind backward rotationto IO port

      PUT 1 uInt8: 77 ('M')
      PUT 1 uInt8: 1 … 6 [IO port]
      PUT 1 uInt8: 66 ('B')

### Stopping the motor
In addition to using end-switches, the motor can also be stopped by means of serial commands.

* #### Soft stop
  Decelerate the motor to a complete standstill.

      PUT 1 uInt8: 120 ('x')

* #### Emergency stop
  Stop the motor abruptly. Depending on motor speed this will lead to a loss of steps.

      PUT 1 uInt8: 88 ('X')

* #### Bind soft stop to IO port

      PUT 1 uInt8: 77 ('M')
      PUT 1 uInt8: 1 … 6 [IO port]
      PUT 1 uInt8: 120 ('x')

* #### Bind emergency stop to IO port
  Configures an IO port for use with a limit switch.

      PUT 1 uInt8: 77 ('M')
      PUT 1 uInt8: 1 … 6 [IO port]
      PUT 1 uInt8: 88 ('X')


### Predefined targets
The stepper module can store up to 9 target definitions.
Movement to one of these targets can be triggered by a single byte serial command (see [*Moving the Motor*](#move-to-a-predefined-target)).
Alternatively, you can bind a trigger to one of the IO ports.


* #### Define a target

      PUT 1 uInt8: 84 ('T')
      PUT 1 uInt8: 1 … 9 [target ID]
      PUT 1 Int32: target position [steps]

* #### Get the definition of a target

      PUT 1 uInt8: 71 ('G')
      PUT 1 uInt8: 1 … 9 [target ID]
      GET 1 Int32: target position [steps]

* #### Bind target trigger to IO port

      PUT 1 uInt8: 77 ('M')
      PUT 1 uInt8: 1 … 6 [IO port]
      PUT 1 uInt8: 1 … 9 [target ID]


### Configuration of movement parameters
All of the motors movements are defined by an acceleration phase, a peak velocity and a deceleration phase (the sole exception being the [emergency stop](#emergency-stop)).

* #### Set Acceleration
  This parameter is valid for, both, the acceleration and the deceleration phase.

      PUT 1 uInt8:  65 ('A')
      PUT 1 uInt16: acceleration [steps / s^2]

* #### Get Acceleration

      PUT 2 uInt8:  71, 65 ('GA')
      GET 1 uInt16: acceleration [steps / s^2]

* #### Set peak velocity

      PUT 1 uInt8:  86 ('V')
      PUT 1 uInt16: peak velocity [steps / s]

* #### Get peak velocity

      PUT 2 uInt8:  71, 86 ('GV')
      GET 1 uInt16: peak velocity [steps / s]


### Configuration of motor parameters

* #### Set RMS current
  Set the driver's RMS current output to the motor (up to 850mA for TMC2130, up to 2000mA for TMC5160).

      PUT 1 uInt8:  73 ('I')
      PUT 1 uInt16: RMS current [mA]

* #### Get RMS current

      PUT 2 uInt8:  71, 73 ('GI')
      GET 1 uInt16: RMS current [mA]

* #### Set chopper mode
  You can select between two different chopper modes: a voltage chopper and a PWM chopper. The voltage chopper offers extremely quiet operation at standstill and low to medium speeds. Depending on the use case, the PWM chopper can perform better at higher speeds. Refer to [the Trinamic website](https://www.trinamic.com/technology/motor-control-technology/chopper-modes/) for more details.

      PUT 1 uInt8:  67 ('C')
      PUT 1 uInt8:  chopper mode [0 = PWM chopper, 1 = voltage chopper]

* #### Get chopper mode

      PUT 2 uInt8:  71, 67 ('GC')
      GET 1 uInt8:  chopper mode [0 = PWM chopper, 1 = voltage chopper]


### Configuration of IO ports

* #### Set input configuration of IO port
  When used as inputs the IO ports can be configured with different input modes: floating, pull-up or pull-down.

      PUT 1 uInt8: 82 ('R')
      PUT 1 uInt8: 1 … 6 [number of IO port]
      PUT 1 uInt8: input configuration [0 = floating; 1 = pull-up; 2 = pull-down]

* #### Get input configuration of IO port

      PUT 2 uInt8: 71, 82 ('GR')
      PUT 1 uInt8: 1 … 6 [number of IO port]
      GET 1 uInt8: input configuration [0 = floating; 1 = pull-up; 2 = pull-down]


### EEPROM storage
The Stepper Module can store its configuration to non-volatile memory.
This way you can define default values for your specific setup.
Stored values will automatically be loaded during start-up of the module.
This enables the use of the Stepper Module as a headless unit (i.e., without connection to Bpod or USB host).

* #### Store settings to EEPROM
  To store the current configuration to EEPROM:

      PUT 1 uInt8: 69 ('E')

  The following values will be stored:
  * maximum velocity,
  * acceleration,
  * RMS current,
  * chopper mode,
  * predefined targets,
  * input resistance of IO ports, and
  * function of IO ports.


### System Information

* #### Get Hardware version
  Identify the Stepper Module's hardware revision

      PUT 2 uInt8: 71, 72 ('GH')
      GET 1 uInt8: hardware revision [multiplied by 10]

* #### Get driver version
  Identify the installed stepper driver.

      PUT 2 uInt8: 71, 84 ('GT')
      GET 1 uInt8: driver version [0 = unknown, 17 = TMC2130, 48 = TMC5360]

* #### Get firmware version / USB handshake

      PUT 1 uInt8:  212
      GET 1 uInt32: firmware version

* #### Get Module info (reserved)

      PUT 1 uInt8:  255


## Bill of Materials
| Item     | Vendor    | Qty | Part Number                                                                                      | Description                 |
| :------- | :-------- | :-: | :----------------------------------------------------------------------------------------------- | :-------------------------- |
| C1, C2   | Digi-Key  |  2  | [1189-3780-1-ND](https://www.digikey.com/en/products?keywords=1189-3780-1-ND)                    | Aluminium capacitor, 100 µF |
| C3, C4   | Digi-Key  |  2  | [PCE4362CT-ND](https://www.digikey.com/en/products?keywords=PCE4362CT-ND)                        | Aluminium capacitor, 2.2 µF |
| C5 - C8  | Digi-Key  |  4  | [311-1179-1-ND](https://www.digikey.com/en/products?keywords=311-1179-1-ND)                      | Ceramic capacitor, 0.1 µf   |
| D1       | Digi-Key  |  1  | [1KSMB75CACT-ND](https://www.digikey.com/en/products?keywords=1KSMB75CACT-ND)                    | TVS diode                   |
| D2 - D3  | Digi-Key  |  2  | [SBR80520LT1G](https://www.digikey.com/en/products?keywords=SBR80520LT1G)                        | Schottky diode, 20V 500mA   |
| D4 - D12 | Digi-Key  |  9  | [SS310LWHRVGCT-ND](https://www.digikey.com/en/products?keywords=SS310LWHRVGCT-ND)                | Schottky diode, 100V 3A     |
| IC1      | Digi-Key  |  1  | [LM3480IM3-5.0/NOPBCT-ND](https://octopart.com/lm3480im3-5.0%2Fnopb-texas+instruments-24813903)  | 5V regulator                |
| IC2      | Digi-Key  |  1  | [ADM3077EARZ-ND](https://www.digikey.com/en/products?keywords=ADM3077EARZ-ND)                    | RS-485 IC                   |
| IC3      | Digi-Key  |  1  | [MCP1792T-5002H/CB](https://www.digikey.com/en/products?keywords=MCP1792T-5002H/CB)              | 5V regulator                |
| IC4      | Digi-Key  |  1  | [MCP1793T-3302H/DC](https://www.digikey.com/en/products?keywords=MCP1793T-3302H/DC)              | 3.3V regulator              |
|          | Digi-Key  |  1  | [A31442-ND](https://www.digikey.com/en/products?keywords=A31442-ND)                              | Ethernet jack               |
|          | Digi-Key  |  1  | [839-1512-ND](https://www.digikey.com/en/products?keywords=839-1512-ND)                          | DC barrel jack              |
|          | Digi-Key  |  1  | [PPTC021LFBN-RC](https://www.digikey.com/en/products?keywords=PPTC021LFBN-RC)                    | Female header, 1x2          |
|          | Digi-Key  |  2  | [PPPC081LFBN-RC](https://www.digikey.com/en/products?keywords=PPPC081LFBN-RC)                    | Female header, 1x8          |
|          | Digi-Key  |  2  | [PPPC241LFBN-RC](https://www.digikey.com/en/products?keywords=PPPC241LFBN-RC)                    | Female header, 1x24         |
|          | Digi-Key  |  1  | [1568-1464-ND](https://www.digikey.com/en/products?keywords=1568-1464-ND)                        | Teensy 3.5                  |
|          | Digi-Key  |  1  | [WM21887-ND](https://www.digikey.com/en/products?keywords=WM21887-ND)                            | Terminal Block Header, 1x4  |
|          | Digi-Key  |  1  | [WM7780-ND](https://www.digikey.com/en/products?keywords=WM7780-ND)                              | Terminal Block Header, 1x12 |
|          | Digi-Key  |  1  | [WM7791-ND](https://www.digikey.com/en/products?keywords=WM7791-ND)                              | Terminal Block Plug, 1x4    |
|          | Digi-Key  |  1  | [WM7742-ND](https://www.digikey.com/en/products?keywords=WM7742-ND)                              | Terminal Block Plug, 1x12   |
|          | Watterott |  1  | [201899-001](https://shop.watterott.com/SilentStepStick-TMC5160-Stepper-motor-driver-10-35V-V15) | Stepper motor driver        |



## Credits ##
<img align="right" src="images/module.png" width="350px">

* Concept & firmware by Florian Rau
* PCB layout by Christopher Schultz and Florian Rau
* PCB layout partially based on:
  * [Bpod Teensy Shield](https://github.com/sanworks/Bpod-CAD/tree/master/PCB/Modules/Gen2/Bpod%20Teensy%20Shield) by Sanworks ([GPL v3](https://www.gnu.org/licenses/gpl-3.0.en.html))
  * [SilentStepStick Protector](https://github.com/watterott/SilentStepStick) by Watterott ([CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/))
* Firmare uses the following libraries:
  * [ArCOM](https://github.com/sanworks/ArCOM) by Sanworks ([GPL v3](https://www.gnu.org/licenses/gpl-3.0.en.html))
  * [SmoothStepper](https://github.com/bimac/SmoothStepper) by Florian Rau ([GPL v3](https://www.gnu.org/licenses/gpl-3.0.en.html))
  * [TMCStepper](https://github.com/teemuatlut/TMCStepper) ([MIT](https://opensource.org/licenses/MIT))
