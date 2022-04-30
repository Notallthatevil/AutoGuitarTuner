# AutoGuitarTuner

## Motivation
The idea behind this project was to create a simple automatic guitar tuner. Theoretically it could be configured to tune any instrument for which the motor could be attached to. 

## Setup
To use this code for your own use follow the schematics and setup instructions below

### High Level Overview
This image shows the quick glance, high level overview of the project. This project consists of 4 components. The STM32F072 Microcontroller, a sound detector (I used SEN-14262), a board to control the motor, and the motor itself. 
![alt text](https://github.com/Notallthatevil/AutoGuitarTuner/blob/main/HighLevelDiagram.png)

### Custom Motor Interface Schematic
Here is the schematic I used to control the motor and a list of parts.
![alt text](https://github.com/Notallthatevil/AutoGuitarTuner/blob/main/CustomMotorInterfaceSchematic.png)

| Part | Quantity | Mouser/Digikey Part # | Manufacturer Part # | Manufacturer | Short Description |
|------|----------|-----------------------|---------------------|--------------|-------------------|
|C1|1|140-REA1R0M2CBK0511P|REA010M2CBK-0511P|Lelon|polarized cap - 1uF|
|C2, C3, C4|3|399-1249-1-ND|C1206C104K5RAC7800|Kemet|ceramic cap - 100nF|
|R1, R3, R4, R5|4|311-10KERCT-ND|RC1206JR-0710KL|Yageo|resistor - 10k|
|R2|1|311-.04LWCT-ND|RL1206FR-070R04L|Yageo|current sense resistor - 0.04|
|U5, U6, U7, U8|4|RS2AAR3GCT-ND|RS2AA|R3G|Taiwan Semiconductor Corporation|diode|
|U1|1|511-L298|L298N|STMicroelectronics|L298N|
|U2|1|926-LM75AIMX/NOPB|LM75AIMX/NOPB|Texas Instruments|LM75A|
|J1, J2, J3, J5|1(21 pins)|649-68001-121HLF|68001-121HLF|Amphenol FCI|generic pin header|
|J4|1|992-CON-SOCJ-2155|CON-SOCJ-2155|Gravitech|2.1mm x 5.5mm DC power jack|


### Instructions
Using the two images above the full system can be setup. From the sound detector we connect a 5V source and ground. Then we connect the audio output to a valid ADC pin on the STM32. I chose pin PC0. The motor controller needs a 6V external source from a barrel connector and additional 5V and 3V3 sources from the STM32. The 6 pins on the motor then connect to the labeled wires on the schematic. I then connected another 5V to the ENABLE pin to enable the motor. PC6 and PC7 were setup as output pins and attached to Input1 and Input2 respectively. These two pins control whether the motor is spinning clockwise or counter-clockwise. 
The motor is then attached to a 4mm-to-4mm coupler. The other end of the couple is attached to shaft of the guitars tuning key. 

Once all configured and running, attach the motor and coupling to the note to be tuned and pluck away until the motor stops spinning. The red and blue LEDs on the STM32 will also be on if the note needs to be tuned up or down. 
