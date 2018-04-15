EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ESP8266
LIBS:n39-kicad
LIBS:Board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Cloud Temperature Sensor"
Date "2018-04-14"
Rev ""
Comp "penguineer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP-12E U?
U 1 1 5AD1E154
P 8100 2750
F 0 "U?" H 8100 2650 50  0000 C CNN
F 1 "ESP-12E" H 8100 2850 50  0000 C CNN
F 2 "" H 8100 2750 50  0001 C CNN
F 3 "" H 8100 2750 50  0001 C CNN
	1    8100 2750
	1    0    0    -1  
$EndComp
$Comp
L MCP9808 U?
U 1 1 5AD1E1BA
P 9100 5100
F 0 "U?" H 8850 5400 60  0000 C CNN
F 1 "MCP9808" H 9400 5400 60  0000 C CNN
F 2 "Housings_SSOP:MSOP-8_3x3mm_Pitch0.65mm" H 9150 4350 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/MC9808.pdf" H 9300 4500 60  0001 C CNN
F 4 "MCP 9808-EMS" H 9050 5700 60  0001 C CNN "REICHELT"
F 5 "http://www.reichelt.de/index.html?ARTICLE=137341" H 9150 5850 60  0001 C CNN "REICHELT URI"
	1    9100 5100
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD1E3D6
P 6750 3150
F 0 "#PWR?" H 6750 3000 50  0001 C CNN
F 1 "VCC" H 6750 3300 50  0000 C CNN
F 2 "" H 6750 3150 50  0000 C CNN
F 3 "" H 6750 3150 50  0000 C CNN
	1    6750 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD1E3E7
P 9250 3150
F 0 "#PWR?" H 9250 2900 50  0001 C CNN
F 1 "GND" H 9250 3000 50  0000 C CNN
F 2 "" H 9250 3150 50  0000 C CNN
F 3 "" H 9250 3150 50  0000 C CNN
	1    9250 3150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AD1E52B
P 10100 5100
F 0 "C?" H 10125 5200 50  0000 L CNN
F 1 "100n" H 10125 5000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10138 4950 50  0001 C CNN
F 3 "" H 10100 5100 50  0000 C CNN
	1    10100 5100
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD1E585
P 10100 4500
F 0 "#PWR?" H 10100 4350 50  0001 C CNN
F 1 "VCC" H 10100 4650 50  0000 C CNN
F 2 "" H 10100 4500 50  0000 C CNN
F 3 "" H 10100 4500 50  0000 C CNN
	1    10100 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD1E599
P 10100 5750
F 0 "#PWR?" H 10100 5500 50  0001 C CNN
F 1 "GND" H 10100 5600 50  0000 C CNN
F 2 "" H 10100 5750 50  0000 C CNN
F 3 "" H 10100 5750 50  0000 C CNN
	1    10100 5750
	1    0    0    -1  
$EndComp
Text GLabel 8400 4950 0    39   BiDi ~ 0
SDA
Text GLabel 8400 5050 0    39   BiDi ~ 0
SCL
Text GLabel 8400 5250 0    39   Output ~ 0
Alert
$Comp
L CONN_01X05 P?
U 1 1 5AD1E853
P 9450 1650
F 0 "P?" H 9450 1950 50  0000 C CNN
F 1 "CONN_01X05" V 9550 1650 50  0000 C CNN
F 2 "" H 9450 1650 50  0000 C CNN
F 3 "" H 9450 1650 50  0000 C CNN
	1    9450 1650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD1E8CC
P 9350 2000
F 0 "#PWR?" H 9350 1750 50  0001 C CNN
F 1 "GND" H 9350 1850 50  0000 C CNN
F 2 "" H 9350 2000 50  0000 C CNN
F 3 "" H 9350 2000 50  0000 C CNN
	1    9350 2000
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD1E910
P 9900 1850
F 0 "#PWR?" H 9900 1700 50  0001 C CNN
F 1 "VCC" H 9900 2000 50  0000 C CNN
F 2 "" H 9900 1850 50  0000 C CNN
F 3 "" H 9900 1850 50  0000 C CNN
	1    9900 1850
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR?
U 1 1 5AD1E925
P 10100 1850
F 0 "#PWR?" H 10100 1700 50  0001 C CNN
F 1 "VDD" H 10100 2000 50  0000 C CNN
F 2 "" H 10100 1850 50  0000 C CNN
F 3 "" H 10100 1850 50  0000 C CNN
	1    10100 1850
	1    0    0    -1  
$EndComp
Text GLabel 9250 2750 2    39   BiDi ~ 0
SDA
Text GLabel 9250 2650 2    39   BiDi ~ 0
SCL
Text GLabel 9250 3050 2    39   Input ~ 0
Alert
$Comp
L VCC #PWR?
U 1 1 5AD21696
P 4500 6150
F 0 "#PWR?" H 4500 6000 50  0001 C CNN
F 1 "VCC" H 4500 6300 50  0000 C CNN
F 2 "" H 4500 6150 50  0000 C CNN
F 3 "" H 4500 6150 50  0000 C CNN
	1    4500 6150
	1    0    0    -1  
$EndComp
$Comp
L VQE24 D?
U 1 1 5AD34FD0
P 5650 5400
F 0 "D?" H 6300 4750 60  0000 C CNN
F 1 "VQE24" H 5650 4950 60  0000 C CNN
F 2 "" H 5500 5250 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/VQE_7segment.pdf" H 5850 4450 60  0001 C CNN
	1    5650 5400
	1    0    0    -1  
$EndComp
$Comp
L PCA9551 U?
U 1 1 5AD35842
P 2100 5450
F 0 "U?" H 2400 5000 60  0000 C CNN
F 1 "PCA9551" H 2100 5100 60  0000 C CNN
F 2 "SMD_Packages:SO-16-N" H 2300 4550 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/PCA9551_NXP.pdf" H 2800 4750 60  0001 C CNN
F 4 "PCA 9551 D" H 2400 4650 60  0001 C CNN "REICHELT"
	1    2100 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD35EBC
P 2400 4750
F 0 "#PWR?" H 2400 4500 50  0001 C CNN
F 1 "GND" H 2400 4600 50  0000 C CNN
F 2 "" H 2400 4750 50  0000 C CNN
F 3 "" H 2400 4750 50  0000 C CNN
	1    2400 4750
	1    0    0    -1  
$EndComp
Text GLabel 1400 5500 0    39   BiDi ~ 0
SDA
Text GLabel 1400 5400 0    39   BiDi ~ 0
SCL
$Comp
L GND #PWR?
U 1 1 5AD36838
P 950 5850
F 0 "#PWR?" H 950 5600 50  0001 C CNN
F 1 "GND" H 950 5700 50  0000 C CNN
F 2 "" H 950 5850 50  0000 C CNN
F 3 "" H 950 5850 50  0000 C CNN
	1    950  5850
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD36C07
P 950 4950
F 0 "#PWR?" H 950 4800 50  0001 C CNN
F 1 "VCC" H 950 5100 50  0000 C CNN
F 2 "" H 950 4950 50  0000 C CNN
F 3 "" H 950 4950 50  0000 C CNN
	1    950  4950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AD36C8E
P 950 5400
F 0 "C?" H 975 5500 50  0000 L CNN
F 1 "100n" H 975 5300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 988 5250 50  0001 C CNN
F 3 "" H 950 5400 50  0000 C CNN
	1    950  5400
	1    0    0    -1  
$EndComp
Text GLabel 3350 5050 2    39   Output ~ 0
a1
Text GLabel 3350 5150 2    39   Output ~ 0
b1
Text GLabel 3350 5250 2    39   Output ~ 0
c1
Text GLabel 3350 5450 2    39   Output ~ 0
e1
Text GLabel 3350 5550 2    39   Output ~ 0
f1
Text GLabel 3350 5650 2    39   Output ~ 0
g1
Text GLabel 3350 5350 2    39   Output ~ 0
d1
Text GLabel 3350 5750 2    39   Output ~ 0
h1
$Comp
L R R?
U 1 1 5AD385D2
P 3050 5050
F 0 "R?" V 3000 4900 50  0000 C CNN
F 1 "39R" V 3050 5050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5050 50  0001 C CNN
F 3 "" H 3050 5050 50  0000 C CNN
	1    3050 5050
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3864D
P 3050 5150
F 0 "R?" V 3000 5000 50  0000 C CNN
F 1 "39R" V 3050 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5150 50  0001 C CNN
F 3 "" H 3050 5150 50  0000 C CNN
	1    3050 5150
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3867E
P 3050 5250
F 0 "R?" V 3000 5100 50  0000 C CNN
F 1 "39R" V 3050 5250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5250 50  0001 C CNN
F 3 "" H 3050 5250 50  0000 C CNN
	1    3050 5250
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD386B2
P 3050 5350
F 0 "R?" V 3000 5200 50  0000 C CNN
F 1 "39R" V 3050 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5350 50  0001 C CNN
F 3 "" H 3050 5350 50  0000 C CNN
	1    3050 5350
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3871E
P 3050 5450
F 0 "R?" V 3000 5300 50  0000 C CNN
F 1 "39R" V 3050 5450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5450 50  0001 C CNN
F 3 "" H 3050 5450 50  0000 C CNN
	1    3050 5450
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD38758
P 3050 5550
F 0 "R?" V 3000 5400 50  0000 C CNN
F 1 "39R" V 3050 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5550 50  0001 C CNN
F 3 "" H 3050 5550 50  0000 C CNN
	1    3050 5550
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD38795
P 3050 5650
F 0 "R?" V 3000 5500 50  0000 C CNN
F 1 "39R" V 3050 5650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5650 50  0001 C CNN
F 3 "" H 3050 5650 50  0000 C CNN
	1    3050 5650
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD387DB
P 3050 5750
F 0 "R?" V 3000 5600 50  0000 C CNN
F 1 "39R" V 3050 5750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 5750 50  0001 C CNN
F 3 "" H 3050 5750 50  0000 C CNN
	1    3050 5750
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD38C91
P 1250 5150
F 0 "R?" V 1330 5150 50  0000 C CNN
F 1 "10k" V 1250 5150 50  0000 C CNN
F 2 "" V 1180 5150 50  0000 C CNN
F 3 "" H 1250 5150 50  0000 C CNN
	1    1250 5150
	0    1    1    0   
$EndComp
$Comp
L PCA9551 U?
U 1 1 5AD39642
P 2100 7000
F 0 "U?" H 2400 6550 60  0000 C CNN
F 1 "PCA9551" H 2100 6650 60  0000 C CNN
F 2 "SMD_Packages:SO-16-N" H 2300 6100 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/PCA9551_NXP.pdf" H 2800 6300 60  0001 C CNN
F 4 "PCA 9551 D" H 2400 6200 60  0001 C CNN "REICHELT"
	1    2100 7000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD39648
P 2400 6300
F 0 "#PWR?" H 2400 6050 50  0001 C CNN
F 1 "GND" H 2400 6150 50  0000 C CNN
F 2 "" H 2400 6300 50  0000 C CNN
F 3 "" H 2400 6300 50  0000 C CNN
	1    2400 6300
	1    0    0    -1  
$EndComp
Text GLabel 1400 7050 0    39   BiDi ~ 0
SDA
Text GLabel 1400 6950 0    39   BiDi ~ 0
SCL
$Comp
L GND #PWR?
U 1 1 5AD3965B
P 950 7400
F 0 "#PWR?" H 950 7150 50  0001 C CNN
F 1 "GND" H 950 7250 50  0000 C CNN
F 2 "" H 950 7400 50  0000 C CNN
F 3 "" H 950 7400 50  0000 C CNN
	1    950  7400
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD39665
P 950 6500
F 0 "#PWR?" H 950 6350 50  0001 C CNN
F 1 "VCC" H 950 6650 50  0000 C CNN
F 2 "" H 950 6500 50  0000 C CNN
F 3 "" H 950 6500 50  0000 C CNN
	1    950  6500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AD3966F
P 950 6950
F 0 "C?" H 975 7050 50  0000 L CNN
F 1 "100n" H 975 6850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 988 6800 50  0001 C CNN
F 3 "" H 950 6950 50  0000 C CNN
	1    950  6950
	1    0    0    -1  
$EndComp
Text GLabel 3350 6600 2    39   Output ~ 0
a2
Text GLabel 3350 6700 2    39   Output ~ 0
b2
Text GLabel 3350 6800 2    39   Output ~ 0
c2
Text GLabel 3350 7000 2    39   Output ~ 0
e2
Text GLabel 3350 7100 2    39   Output ~ 0
f2
Text GLabel 3350 7200 2    39   Output ~ 0
g2
Text GLabel 3350 6900 2    39   Output ~ 0
d2
Text GLabel 3350 7300 2    39   Output ~ 0
h2
$Comp
L R R?
U 1 1 5AD3968C
P 3050 6600
F 0 "R?" V 3000 6450 50  0000 C CNN
F 1 "39R" V 3050 6600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 6600 50  0001 C CNN
F 3 "" H 3050 6600 50  0000 C CNN
	1    3050 6600
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD39692
P 3050 6700
F 0 "R?" V 3000 6550 50  0000 C CNN
F 1 "39R" V 3050 6700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 6700 50  0001 C CNN
F 3 "" H 3050 6700 50  0000 C CNN
	1    3050 6700
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD39698
P 3050 6800
F 0 "R?" V 3000 6650 50  0000 C CNN
F 1 "39R" V 3050 6800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 6800 50  0001 C CNN
F 3 "" H 3050 6800 50  0000 C CNN
	1    3050 6800
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3969E
P 3050 6900
F 0 "R?" V 3000 6750 50  0000 C CNN
F 1 "39R" V 3050 6900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 6900 50  0001 C CNN
F 3 "" H 3050 6900 50  0000 C CNN
	1    3050 6900
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396A4
P 3050 7000
F 0 "R?" V 3000 6850 50  0000 C CNN
F 1 "39R" V 3050 7000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 7000 50  0001 C CNN
F 3 "" H 3050 7000 50  0000 C CNN
	1    3050 7000
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396AA
P 3050 7100
F 0 "R?" V 3000 6950 50  0000 C CNN
F 1 "39R" V 3050 7100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 7100 50  0001 C CNN
F 3 "" H 3050 7100 50  0000 C CNN
	1    3050 7100
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396B0
P 3050 7200
F 0 "R?" V 3000 7050 50  0000 C CNN
F 1 "39R" V 3050 7200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 7200 50  0001 C CNN
F 3 "" H 3050 7200 50  0000 C CNN
	1    3050 7200
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396B6
P 3050 7300
F 0 "R?" V 3000 7150 50  0000 C CNN
F 1 "39R" V 3050 7300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2980 7300 50  0001 C CNN
F 3 "" H 3050 7300 50  0000 C CNN
	1    3050 7300
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396BF
P 1250 6700
F 0 "R?" V 1330 6700 50  0000 C CNN
F 1 "10k" V 1250 6700 50  0000 C CNN
F 2 "" V 1180 6700 50  0000 C CNN
F 3 "" H 1250 6700 50  0000 C CNN
	1    1250 6700
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD397EB
P 2000 6200
F 0 "#PWR?" H 2000 6050 50  0001 C CNN
F 1 "VCC" H 2000 6350 50  0000 C CNN
F 2 "" H 2000 6200 50  0000 C CNN
F 3 "" H 2000 6200 50  0000 C CNN
	1    2000 6200
	1    0    0    -1  
$EndComp
Text GLabel 4500 5000 0    39   Input ~ 0
a1
Text GLabel 4500 5100 0    39   Input ~ 0
b1
Text GLabel 4500 5200 0    39   Input ~ 0
c1
Text GLabel 4500 5400 0    39   Input ~ 0
e1
Text GLabel 4500 5500 0    39   Input ~ 0
f1
Text GLabel 4500 5600 0    39   Input ~ 0
g1
Text GLabel 4500 5300 0    39   Input ~ 0
d1
Text GLabel 4500 5750 0    39   Input ~ 0
h1
Text GLabel 6800 5000 2    39   Input ~ 0
a2
Text GLabel 6800 5100 2    39   Input ~ 0
b2
Text GLabel 6800 5200 2    39   Input ~ 0
c2
Text GLabel 6800 5400 2    39   Input ~ 0
e2
Text GLabel 6800 5500 2    39   Input ~ 0
f2
Text GLabel 6800 5600 2    39   Input ~ 0
g2
Text GLabel 6800 5300 2    39   Input ~ 0
d2
Text GLabel 6800 5750 2    39   Input ~ 0
h2
Text GLabel 10600 3200 2    39   BiDi ~ 0
SCL
Text GLabel 10600 3600 2    39   BiDi ~ 0
SDA
$Comp
L R R?
U 1 1 5AD3CEF0
P 10400 3050
F 0 "R?" V 10480 3050 50  0000 C CNN
F 1 "10k" V 10400 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10330 3050 50  0001 C CNN
F 3 "" H 10400 3050 50  0000 C CNN
	1    10400 3050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5AD3CF7D
P 10250 3450
F 0 "R?" V 10330 3450 50  0000 C CNN
F 1 "10k" V 10250 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10180 3450 50  0001 C CNN
F 3 "" H 10250 3450 50  0000 C CNN
	1    10250 3450
	1    0    0    -1  
$EndComp
Text GLabel 10050 3200 0    39   BiDi ~ 0
SCL
Text GLabel 10050 3600 0    39   BiDi ~ 0
SDA
$Comp
L VCC #PWR?
U 1 1 5AD3D501
P 10250 2700
F 0 "#PWR?" H 10250 2550 50  0001 C CNN
F 1 "VCC" H 10250 2850 50  0000 C CNN
F 2 "" H 10250 2700 50  0000 C CNN
F 3 "" H 10250 2700 50  0000 C CNN
	1    10250 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 5550 9100 5650
Wire Wire Line
	9000 3150 9250 3150
Wire Wire Line
	7200 3150 6750 3150
Wire Wire Line
	10100 4500 10100 4600
Wire Wire Line
	10100 4600 10100 4950
Wire Wire Line
	10100 5250 10100 5650
Wire Wire Line
	10100 5650 10100 5750
Wire Wire Line
	8400 4950 8600 4950
Wire Wire Line
	8400 5050 8600 5050
Wire Wire Line
	8400 5250 8600 5250
Wire Wire Line
	9600 5050 9750 5050
Wire Wire Line
	9750 5050 9750 5150
Wire Wire Line
	9750 5150 9750 5250
Wire Wire Line
	9750 5250 9750 5650
Wire Wire Line
	9600 5150 9750 5150
Connection ~ 9750 5150
Wire Wire Line
	9600 5250 9750 5250
Connection ~ 9750 5250
Wire Wire Line
	9000 2450 9250 2450
Wire Wire Line
	9250 2450 9250 1850
Wire Wire Line
	9000 2550 9450 2550
Wire Wire Line
	9450 2550 9450 1850
Wire Wire Line
	9350 1850 9350 2000
Wire Wire Line
	9650 1850 9650 1900
Wire Wire Line
	9650 1900 9900 1900
Wire Wire Line
	9900 1900 9900 1850
Wire Wire Line
	9550 1850 9550 2050
Wire Wire Line
	9550 2050 10100 2050
Wire Wire Line
	10100 2050 10100 1850
Wire Wire Line
	9000 2650 9250 2650
Wire Wire Line
	9000 2750 9250 2750
Wire Wire Line
	9250 3050 9000 3050
Wire Wire Line
	2000 4750 2000 4650
Wire Wire Line
	2000 4650 2100 4650
Wire Wire Line
	2100 4650 2200 4650
Wire Wire Line
	2200 4650 2400 4650
Wire Wire Line
	2400 4650 2400 4750
Wire Wire Line
	2100 4750 2100 4650
Connection ~ 2100 4650
Wire Wire Line
	2200 4750 2200 4650
Connection ~ 2200 4650
Wire Wire Line
	1400 5400 1450 5400
Wire Wire Line
	1400 5500 1450 5500
Wire Wire Line
	1450 5750 950  5750
Wire Wire Line
	950  5550 950  5750
Wire Wire Line
	950  5750 950  5850
Wire Wire Line
	1400 5150 1450 5150
Wire Wire Line
	1450 5050 950  5050
Wire Wire Line
	950  4950 950  5050
Wire Wire Line
	950  5050 950  5150
Wire Wire Line
	950  5150 950  5250
Connection ~ 950  5050
Connection ~ 950  5750
Wire Wire Line
	2750 5050 2900 5050
Wire Wire Line
	2750 5150 2900 5150
Wire Wire Line
	2750 5250 2900 5250
Wire Wire Line
	2750 5350 2900 5350
Wire Wire Line
	2750 5450 2900 5450
Wire Wire Line
	2750 5550 2900 5550
Wire Wire Line
	2750 5650 2900 5650
Wire Wire Line
	2750 5750 2900 5750
Wire Wire Line
	3200 5350 3350 5350
Wire Wire Line
	3200 5450 3350 5450
Wire Wire Line
	3200 5550 3350 5550
Wire Wire Line
	3200 5650 3350 5650
Wire Wire Line
	3200 5750 3350 5750
Wire Wire Line
	3200 5050 3350 5050
Wire Wire Line
	3200 5150 3350 5150
Wire Wire Line
	3200 5250 3350 5250
Wire Wire Line
	1100 5150 950  5150
Connection ~ 950  5150
Wire Wire Line
	2100 6200 2200 6200
Wire Wire Line
	2200 6200 2400 6200
Wire Wire Line
	2400 6200 2400 6300
Wire Wire Line
	2100 6300 2100 6200
Wire Wire Line
	2200 6300 2200 6200
Connection ~ 2200 6200
Wire Wire Line
	1400 6950 1450 6950
Wire Wire Line
	1400 7050 1450 7050
Wire Wire Line
	1450 7300 950  7300
Wire Wire Line
	950  7100 950  7300
Wire Wire Line
	950  7300 950  7400
Wire Wire Line
	1400 6700 1450 6700
Wire Wire Line
	1450 6600 950  6600
Wire Wire Line
	950  6500 950  6600
Wire Wire Line
	950  6600 950  6700
Wire Wire Line
	950  6700 950  6800
Connection ~ 950  6600
Connection ~ 950  7300
Wire Wire Line
	2750 6600 2900 6600
Wire Wire Line
	2750 6700 2900 6700
Wire Wire Line
	2750 6800 2900 6800
Wire Wire Line
	2750 6900 2900 6900
Wire Wire Line
	2750 7000 2900 7000
Wire Wire Line
	2750 7100 2900 7100
Wire Wire Line
	2750 7200 2900 7200
Wire Wire Line
	2750 7300 2900 7300
Wire Wire Line
	3200 6900 3350 6900
Wire Wire Line
	3200 7000 3350 7000
Wire Wire Line
	3200 7100 3350 7100
Wire Wire Line
	3200 7200 3350 7200
Wire Wire Line
	3200 7300 3350 7300
Wire Wire Line
	3200 6600 3350 6600
Wire Wire Line
	3200 6700 3350 6700
Wire Wire Line
	3200 6800 3350 6800
Wire Wire Line
	1100 6700 950  6700
Connection ~ 950  6700
Wire Wire Line
	2000 6200 2000 6300
Wire Wire Line
	4650 5300 4500 5300
Wire Wire Line
	4650 5400 4500 5400
Wire Wire Line
	4650 5500 4500 5500
Wire Wire Line
	4650 5600 4500 5600
Wire Wire Line
	4650 5750 4500 5750
Wire Wire Line
	4650 5000 4500 5000
Wire Wire Line
	4650 5100 4500 5100
Wire Wire Line
	4650 5200 4500 5200
Wire Wire Line
	6650 5300 6800 5300
Wire Wire Line
	6650 5400 6800 5400
Wire Wire Line
	6650 5500 6800 5500
Wire Wire Line
	6650 5600 6800 5600
Wire Wire Line
	6650 5750 6800 5750
Wire Wire Line
	6650 5000 6800 5000
Wire Wire Line
	6650 5100 6800 5100
Wire Wire Line
	6650 5200 6800 5200
Wire Wire Line
	4500 6150 4500 6250
Wire Wire Line
	4500 6250 5200 6250
Wire Wire Line
	5200 6250 6100 6250
Wire Wire Line
	6100 6250 6100 6150
Wire Wire Line
	5200 6150 5200 6250
Connection ~ 5200 6250
Wire Wire Line
	10050 3200 10400 3200
Wire Wire Line
	10400 3200 10600 3200
Connection ~ 10400 3200
Wire Wire Line
	10050 3600 10250 3600
Wire Wire Line
	10250 3600 10600 3600
Connection ~ 10250 3600
Wire Wire Line
	10250 2700 10250 2800
Wire Wire Line
	10250 2800 10250 3300
Wire Wire Line
	10250 2800 10400 2800
Wire Wire Line
	10400 2800 10400 2900
Connection ~ 10250 2800
Wire Wire Line
	9100 4650 9100 4600
Wire Wire Line
	9100 4600 10100 4600
Connection ~ 10100 4600
Wire Wire Line
	9100 5650 9750 5650
Wire Wire Line
	9750 5650 10100 5650
Connection ~ 10100 5650
Connection ~ 9750 5650
$EndSCHEMATC
