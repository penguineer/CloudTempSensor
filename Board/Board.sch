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
LIBS:switches
LIBS:Board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Cloud Temperature Sensor"
Date "2018-04-21"
Rev ""
Comp "penguineer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP-12E U4
U 1 1 5AD1E154
P 6800 2900
F 0 "U4" H 6800 2800 50  0000 C CNN
F 1 "ESP-12E" H 6800 3000 50  0000 C CNN
F 2 "ESP8266:ESP-12E" H 6800 2900 50  0001 C CNN
F 3 "" H 6800 2900 50  0001 C CNN
	1    6800 2900
	1    0    0    -1  
$EndComp
$Comp
L MCP9808 U3
U 1 1 5AD1E1BA
P 5150 6600
F 0 "U3" H 4900 6900 60  0000 C CNN
F 1 "MCP9808" H 5450 6900 60  0000 C CNN
F 2 "Housings_SSOP:MSOP-8_3x3mm_Pitch0.65mm" H 5200 5850 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/MC9808.pdf" H 5350 6000 60  0001 C CNN
F 4 "MCP 9808-EMS" H 5100 7200 60  0001 C CNN "REICHELT"
F 5 "http://www.reichelt.de/index.html?ARTICLE=137341" H 5200 7350 60  0001 C CNN "REICHELT URI"
	1    5150 6600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR17
U 1 1 5AD1E3D6
P 4950 2400
F 0 "#PWR17" H 4950 2250 50  0001 C CNN
F 1 "VCC" H 4950 2550 50  0000 C CNN
F 2 "" H 4950 2400 50  0000 C CNN
F 3 "" H 4950 2400 50  0000 C CNN
	1    4950 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR26
U 1 1 5AD1E3E7
P 8150 3400
F 0 "#PWR26" H 8150 3150 50  0001 C CNN
F 1 "GND" H 8150 3250 50  0000 C CNN
F 2 "" H 8150 3400 50  0000 C CNN
F 3 "" H 8150 3400 50  0000 C CNN
	1    8150 3400
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 5AD1E52B
P 6150 6600
F 0 "C6" H 6175 6700 50  0000 L CNN
F 1 "100n" H 6175 6500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6188 6450 50  0001 C CNN
F 3 "" H 6150 6600 50  0000 C CNN
	1    6150 6600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR21
U 1 1 5AD1E585
P 6150 6000
F 0 "#PWR21" H 6150 5850 50  0001 C CNN
F 1 "VCC" H 6150 6150 50  0000 C CNN
F 2 "" H 6150 6000 50  0000 C CNN
F 3 "" H 6150 6000 50  0000 C CNN
	1    6150 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR22
U 1 1 5AD1E599
P 6150 7250
F 0 "#PWR22" H 6150 7000 50  0001 C CNN
F 1 "GND" H 6150 7100 50  0000 C CNN
F 2 "" H 6150 7250 50  0000 C CNN
F 3 "" H 6150 7250 50  0000 C CNN
	1    6150 7250
	1    0    0    -1  
$EndComp
Text GLabel 4600 6450 0    39   BiDi ~ 0
SDA
Text GLabel 4600 6550 0    39   BiDi ~ 0
SCL
Text GLabel 4600 6750 0    39   Output ~ 0
Alert
$Comp
L CONN_01X05 P2
U 1 1 5AD1E853
P 9950 1100
F 0 "P2" H 9950 1400 50  0000 C CNN
F 1 "CONN_01X05" V 10050 1100 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x05" H 9950 1100 50  0001 C CNN
F 3 "" H 9950 1100 50  0000 C CNN
	1    9950 1100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR30
U 1 1 5AD1E8CC
P 9850 1350
F 0 "#PWR30" H 9850 1100 50  0001 C CNN
F 1 "GND" H 9850 1200 50  0000 C CNN
F 2 "" H 9850 1350 50  0000 C CNN
F 3 "" H 9850 1350 50  0000 C CNN
	1    9850 1350
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR34
U 1 1 5AD1E910
P 10600 1300
F 0 "#PWR34" H 10600 1150 50  0001 C CNN
F 1 "VCC" H 10600 1450 50  0000 C CNN
F 2 "" H 10600 1300 50  0000 C CNN
F 3 "" H 10600 1300 50  0000 C CNN
	1    10600 1300
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR33
U 1 1 5AD1E925
P 10400 1300
F 0 "#PWR33" H 10400 1150 50  0001 C CNN
F 1 "VDD" H 10400 1450 50  0000 C CNN
F 2 "" H 10400 1300 50  0000 C CNN
F 3 "" H 10400 1300 50  0000 C CNN
	1    10400 1300
	1    0    0    -1  
$EndComp
Text GLabel 8250 2900 2    39   BiDi ~ 0
SDA
Text GLabel 8250 2800 2    39   BiDi ~ 0
SCL
Text GLabel 5600 2500 1    39   Input ~ 0
Alert
$Comp
L VCC #PWR4
U 1 1 5AD21696
P 1150 7450
F 0 "#PWR4" H 1150 7300 50  0001 C CNN
F 1 "VCC" H 1150 7600 50  0000 C CNN
F 2 "" H 1150 7450 50  0000 C CNN
F 3 "" H 1150 7450 50  0000 C CNN
	1    1150 7450
	1    0    0    -1  
$EndComp
$Comp
L VQE24 D1
U 1 1 5AD34FD0
P 2300 6700
F 0 "D1" H 2950 6050 60  0000 C CNN
F 1 "VQE24" H 2300 6250 60  0000 C CNN
F 2 "n39-kicad:VQE24" H 2150 6550 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/VQE_7segment.pdf" H 2500 5750 60  0001 C CNN
	1    2300 6700
	1    0    0    -1  
$EndComp
Text GLabel 3500 5100 2    39   Output ~ 0
a1
Text GLabel 3500 4900 2    39   Output ~ 0
b1
Text GLabel 3500 4800 2    39   Output ~ 0
c1
Text GLabel 3500 4700 2    39   Output ~ 0
e1
Text GLabel 3500 5000 2    39   Output ~ 0
f1
Text GLabel 3500 5200 2    39   Output ~ 0
g1
Text GLabel 3500 4600 2    39   Output ~ 0
d1
Text GLabel 3500 5300 2    39   Output ~ 0
h1
$Comp
L R R10
U 1 1 5AD385D2
P 3200 4600
F 0 "R10" V 3150 4400 50  0000 C CNN
F 1 "39R" V 3200 4600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4600 50  0001 C CNN
F 3 "" H 3200 4600 50  0000 C CNN
	1    3200 4600
	0    1    1    0   
$EndComp
$Comp
L R R11
U 1 1 5AD3864D
P 3200 4700
F 0 "R11" V 3150 4500 50  0000 C CNN
F 1 "39R" V 3200 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4700 50  0001 C CNN
F 3 "" H 3200 4700 50  0000 C CNN
	1    3200 4700
	0    1    1    0   
$EndComp
$Comp
L R R12
U 1 1 5AD3867E
P 3200 4800
F 0 "R12" V 3150 4600 50  0000 C CNN
F 1 "39R" V 3200 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4800 50  0001 C CNN
F 3 "" H 3200 4800 50  0000 C CNN
	1    3200 4800
	0    1    1    0   
$EndComp
$Comp
L R R13
U 1 1 5AD386B2
P 3200 4900
F 0 "R13" V 3150 4700 50  0000 C CNN
F 1 "39R" V 3200 4900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4900 50  0001 C CNN
F 3 "" H 3200 4900 50  0000 C CNN
	1    3200 4900
	0    1    1    0   
$EndComp
$Comp
L R R14
U 1 1 5AD3871E
P 3200 5000
F 0 "R14" V 3150 4800 50  0000 C CNN
F 1 "39R" V 3200 5000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 5000 50  0001 C CNN
F 3 "" H 3200 5000 50  0000 C CNN
	1    3200 5000
	0    1    1    0   
$EndComp
$Comp
L R R15
U 1 1 5AD38758
P 3200 5100
F 0 "R15" V 3150 4900 50  0000 C CNN
F 1 "39R" V 3200 5100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 5100 50  0001 C CNN
F 3 "" H 3200 5100 50  0000 C CNN
	1    3200 5100
	0    1    1    0   
$EndComp
$Comp
L R R16
U 1 1 5AD38795
P 3200 5200
F 0 "R16" V 3150 5000 50  0000 C CNN
F 1 "39R" V 3200 5200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 5200 50  0001 C CNN
F 3 "" H 3200 5200 50  0000 C CNN
	1    3200 5200
	0    1    1    0   
$EndComp
$Comp
L R R17
U 1 1 5AD387DB
P 3200 5300
F 0 "R17" V 3150 5100 50  0000 C CNN
F 1 "39R" V 3200 5300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 5300 50  0001 C CNN
F 3 "" H 3200 5300 50  0000 C CNN
	1    3200 5300
	0    1    1    0   
$EndComp
Text GLabel 3500 3800 2    39   Output ~ 0
a2
Text GLabel 3500 4000 2    39   Output ~ 0
b2
Text GLabel 3500 4300 2    39   Output ~ 0
c2
Text GLabel 3500 4200 2    39   Output ~ 0
e2
Text GLabel 3500 3900 2    39   Output ~ 0
f2
Text GLabel 3500 3700 2    39   Output ~ 0
g2
Text GLabel 3500 4100 2    39   Output ~ 0
d2
Text GLabel 3500 4400 2    39   Output ~ 0
h2
$Comp
L R R2
U 1 1 5AD3968C
P 3200 3700
F 0 "R2" V 3150 3500 50  0000 C CNN
F 1 "39R" V 3200 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 3700 50  0001 C CNN
F 3 "" H 3200 3700 50  0000 C CNN
	1    3200 3700
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 5AD39692
P 3200 3800
F 0 "R3" V 3150 3600 50  0000 C CNN
F 1 "39R" V 3200 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 3800 50  0001 C CNN
F 3 "" H 3200 3800 50  0000 C CNN
	1    3200 3800
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 5AD39698
P 3200 3900
F 0 "R4" V 3150 3700 50  0000 C CNN
F 1 "39R" V 3200 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 3900 50  0001 C CNN
F 3 "" H 3200 3900 50  0000 C CNN
	1    3200 3900
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 5AD3969E
P 3200 4000
F 0 "R5" V 3150 3800 50  0000 C CNN
F 1 "39R" V 3200 4000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4000 50  0001 C CNN
F 3 "" H 3200 4000 50  0000 C CNN
	1    3200 4000
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 5AD396A4
P 3200 4100
F 0 "R6" V 3150 3900 50  0000 C CNN
F 1 "39R" V 3200 4100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4100 50  0001 C CNN
F 3 "" H 3200 4100 50  0000 C CNN
	1    3200 4100
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 5AD396AA
P 3200 4200
F 0 "R7" V 3150 4000 50  0000 C CNN
F 1 "39R" V 3200 4200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4200 50  0001 C CNN
F 3 "" H 3200 4200 50  0000 C CNN
	1    3200 4200
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 5AD396B0
P 3200 4300
F 0 "R8" V 3150 4100 50  0000 C CNN
F 1 "39R" V 3200 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4300 50  0001 C CNN
F 3 "" H 3200 4300 50  0000 C CNN
	1    3200 4300
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 5AD396B6
P 3200 4400
F 0 "R9" V 3150 4200 50  0000 C CNN
F 1 "39R" V 3200 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3130 4400 50  0001 C CNN
F 3 "" H 3200 4400 50  0000 C CNN
	1    3200 4400
	0    1    1    0   
$EndComp
Text GLabel 1150 6300 0    39   Input ~ 0
a1
Text GLabel 1150 6400 0    39   Input ~ 0
b1
Text GLabel 1150 6500 0    39   Input ~ 0
c1
Text GLabel 1150 6700 0    39   Input ~ 0
e1
Text GLabel 1150 6800 0    39   Input ~ 0
f1
Text GLabel 1150 6900 0    39   Input ~ 0
g1
Text GLabel 1150 6600 0    39   Input ~ 0
d1
Text GLabel 1150 7050 0    39   Input ~ 0
h1
Text GLabel 3450 6300 2    39   Input ~ 0
a2
Text GLabel 3450 6400 2    39   Input ~ 0
b2
Text GLabel 3450 6500 2    39   Input ~ 0
c2
Text GLabel 3450 6700 2    39   Input ~ 0
e2
Text GLabel 3450 6800 2    39   Input ~ 0
f2
Text GLabel 3450 6900 2    39   Input ~ 0
g2
Text GLabel 3450 6600 2    39   Input ~ 0
d2
Text GLabel 3450 7050 2    39   Input ~ 0
h2
Text GLabel 9200 1250 2    39   BiDi ~ 0
SCL
Text GLabel 9200 1600 2    39   BiDi ~ 0
SDA
$Comp
L R R32
U 1 1 5AD3CEF0
P 9100 1100
F 0 "R32" V 9180 1100 50  0000 C CNN
F 1 "10k" V 9100 1100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9030 1100 50  0001 C CNN
F 3 "" H 9100 1100 50  0000 C CNN
	1    9100 1100
	1    0    0    -1  
$EndComp
$Comp
L R R31
U 1 1 5AD3CF7D
P 8950 1450
F 0 "R31" V 9030 1450 50  0000 C CNN
F 1 "10k" V 8950 1450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8880 1450 50  0001 C CNN
F 3 "" H 8950 1450 50  0000 C CNN
	1    8950 1450
	1    0    0    -1  
$EndComp
Text GLabel 8800 1250 0    39   BiDi ~ 0
SCL
Text GLabel 8800 1600 0    39   BiDi ~ 0
SDA
$Comp
L VCC #PWR27
U 1 1 5AD3D501
P 8950 800
F 0 "#PWR27" H 8950 650 50  0001 C CNN
F 1 "VCC" H 8950 950 50  0000 C CNN
F 2 "" H 8950 800 50  0000 C CNN
F 3 "" H 8950 800 50  0000 C CNN
	1    8950 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 7050 5150 7150
Wire Wire Line
	4950 3300 5900 3300
Wire Wire Line
	6150 6000 6150 6450
Wire Wire Line
	6150 6750 6150 7250
Wire Wire Line
	4600 6450 4650 6450
Wire Wire Line
	4600 6550 4650 6550
Wire Wire Line
	4600 6750 4650 6750
Wire Wire Line
	5650 6550 5800 6550
Wire Wire Line
	5800 6550 5800 7150
Wire Wire Line
	5650 6650 5800 6650
Connection ~ 5800 6650
Wire Wire Line
	5650 6750 5800 6750
Connection ~ 5800 6750
Wire Wire Line
	7700 2600 7800 2600
Wire Wire Line
	9750 1600 9750 1300
Wire Wire Line
	7700 2700 7800 2700
Wire Wire Line
	9950 1600 9950 1300
Wire Wire Line
	9850 1300 9850 1350
Wire Wire Line
	10150 1300 10150 1350
Wire Wire Line
	10150 1350 10400 1350
Wire Wire Line
	10400 1350 10400 1300
Wire Wire Line
	10050 1300 10050 1500
Wire Wire Line
	10050 1500 10600 1500
Wire Wire Line
	10600 1500 10600 1300
Wire Wire Line
	7700 2800 7800 2800
Wire Wire Line
	7700 2900 7800 2900
Wire Wire Line
	2850 4600 3050 4600
Wire Wire Line
	2850 4700 3050 4700
Wire Wire Line
	2850 4800 3050 4800
Wire Wire Line
	2850 4900 3050 4900
Wire Wire Line
	2850 5000 3050 5000
Wire Wire Line
	2850 5100 3050 5100
Wire Wire Line
	2850 5200 3050 5200
Wire Wire Line
	2850 5300 3050 5300
Wire Wire Line
	3350 4900 3500 4900
Wire Wire Line
	3350 5000 3500 5000
Wire Wire Line
	3350 5100 3500 5100
Wire Wire Line
	3350 5200 3500 5200
Wire Wire Line
	3350 5300 3500 5300
Wire Wire Line
	3350 4600 3500 4600
Wire Wire Line
	3350 4700 3500 4700
Wire Wire Line
	3350 4800 3500 4800
Wire Wire Line
	2850 3700 3050 3700
Wire Wire Line
	2850 3800 3050 3800
Wire Wire Line
	2850 3900 3050 3900
Wire Wire Line
	2850 4000 3050 4000
Wire Wire Line
	2850 4100 3050 4100
Wire Wire Line
	2850 4200 3050 4200
Wire Wire Line
	2850 4300 3050 4300
Wire Wire Line
	2850 4400 3050 4400
Wire Wire Line
	3350 4000 3500 4000
Wire Wire Line
	3350 4100 3500 4100
Wire Wire Line
	3350 4200 3500 4200
Wire Wire Line
	3350 4300 3500 4300
Wire Wire Line
	3350 4400 3500 4400
Wire Wire Line
	3350 3700 3500 3700
Wire Wire Line
	3350 3800 3500 3800
Wire Wire Line
	3350 3900 3500 3900
Wire Wire Line
	1300 6600 1150 6600
Wire Wire Line
	1300 6700 1150 6700
Wire Wire Line
	1300 6800 1150 6800
Wire Wire Line
	1300 6900 1150 6900
Wire Wire Line
	1300 7050 1150 7050
Wire Wire Line
	1300 6300 1150 6300
Wire Wire Line
	1300 6400 1150 6400
Wire Wire Line
	1300 6500 1150 6500
Wire Wire Line
	3300 6600 3450 6600
Wire Wire Line
	3300 6700 3450 6700
Wire Wire Line
	3300 6800 3450 6800
Wire Wire Line
	3300 6900 3450 6900
Wire Wire Line
	3300 7050 3450 7050
Wire Wire Line
	3300 6300 3450 6300
Wire Wire Line
	3300 6400 3450 6400
Wire Wire Line
	3300 6500 3450 6500
Wire Wire Line
	1150 7450 1150 7550
Wire Wire Line
	1150 7550 2750 7550
Wire Wire Line
	2750 7550 2750 7450
Wire Wire Line
	1850 7450 1850 7550
Connection ~ 1850 7550
Wire Wire Line
	8800 1250 9200 1250
Connection ~ 9100 1250
Wire Wire Line
	8950 800  8950 1300
Wire Wire Line
	8950 900  9100 900 
Wire Wire Line
	9100 900  9100 950 
Connection ~ 8950 900 
Wire Wire Line
	5150 6150 5150 6100
Wire Wire Line
	5150 6100 6150 6100
Connection ~ 6150 6100
Wire Wire Line
	5150 7150 6150 7150
Connection ~ 6150 7150
Connection ~ 5800 7150
$Comp
L Screw_Terminal_1x02 J1
U 1 1 5AD3EFB6
P 1100 1100
F 0 "J1" H 1100 1350 50  0000 C TNN
F 1 "Screw_Terminal_1x02" V 950 1100 50  0000 C TNN
F 2 "n39-kicad:AKL_101-02" H 1100 875 50  0001 C CNN
F 3 "" H 1075 1100 50  0001 C CNN
	1    1100 1100
	1    0    0    -1  
$EndComp
$Comp
L USB_B P1
U 1 1 5AD3F1CE
P 1200 2100
F 0 "P1" H 1400 1900 50  0000 C CNN
F 1 "USB_B" H 1150 2300 50  0000 C CNN
F 2 "Connect:USB_Micro-B" V 1150 2000 50  0001 C CNN
F 3 "" V 1150 2000 50  0000 C CNN
	1    1200 2100
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR10
U 1 1 5AD3FEC4
P 1650 2300
F 0 "#PWR10" H 1650 2050 50  0001 C CNN
F 1 "GND" H 1650 2150 50  0000 C CNN
F 2 "" H 1650 2300 50  0000 C CNN
F 3 "" H 1650 2300 50  0000 C CNN
	1    1650 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2200 1650 2200
Wire Wire Line
	1650 2200 1650 2300
Wire Wire Line
	1500 1900 1650 1900
Wire Wire Line
	1650 1900 1650 1800
$Comp
L GND #PWR8
U 1 1 5AD40223
P 1650 1300
F 0 "#PWR8" H 1650 1050 50  0001 C CNN
F 1 "GND" H 1650 1150 50  0000 C CNN
F 2 "" H 1650 1300 50  0000 C CNN
F 3 "" H 1650 1300 50  0000 C CNN
	1    1650 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1000 1650 1000
Wire Wire Line
	1650 1000 1650 900 
Wire Wire Line
	1300 1200 1650 1200
Wire Wire Line
	1650 1200 1650 1300
$Comp
L VDD #PWR7
U 1 1 5AD40ECC
P 1650 900
F 0 "#PWR7" H 1650 750 50  0001 C CNN
F 1 "VDD" H 1650 1050 50  0000 C CNN
F 2 "" H 1650 900 50  0000 C CNN
F 3 "" H 1650 900 50  0000 C CNN
	1    1650 900 
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR9
U 1 1 5AD411EA
P 1650 1800
F 0 "#PWR9" H 1650 1650 50  0001 C CNN
F 1 "VDD" H 1650 1950 50  0000 C CNN
F 2 "" H 1650 1800 50  0000 C CNN
F 3 "" H 1650 1800 50  0000 C CNN
	1    1650 1800
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR14
U 1 1 5AD4147C
P 2550 900
F 0 "#PWR14" H 2550 750 50  0001 C CNN
F 1 "VDD" H 2550 1050 50  0000 C CNN
F 2 "" H 2550 900 50  0000 C CNN
F 3 "" H 2550 900 50  0000 C CNN
	1    2550 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR15
U 1 1 5AD414E4
P 2550 1900
F 0 "#PWR15" H 2550 1650 50  0001 C CNN
F 1 "GND" H 2550 1750 50  0000 C CNN
F 2 "" H 2550 1900 50  0000 C CNN
F 3 "" H 2550 1900 50  0000 C CNN
	1    2550 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 900  2550 1200
Wire Wire Line
	2550 1000 2750 1000
Wire Wire Line
	2550 1500 2550 1900
Wire Wire Line
	2550 1800 3850 1800
Wire Wire Line
	3050 1800 3050 1300
$Comp
L VCC #PWR16
U 1 1 5AD41DB7
P 3850 900
F 0 "#PWR16" H 3850 750 50  0001 C CNN
F 1 "VCC" H 3850 1050 50  0000 C CNN
F 2 "" H 3850 900 50  0000 C CNN
F 3 "" H 3850 900 50  0000 C CNN
	1    3850 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1000 3850 1000
Wire Wire Line
	3850 900  3850 1100
$Comp
L C C4
U 1 1 5AD41FA2
P 3450 1350
F 0 "C4" H 3475 1450 50  0000 L CNN
F 1 "22µ" H 3475 1250 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3488 1200 50  0001 C CNN
F 3 "" H 3450 1350 50  0000 C CNN
	1    3450 1350
	1    0    0    -1  
$EndComp
Connection ~ 3050 1800
$Comp
L LM1117-3.3 U2
U 1 1 5AD42652
P 3050 1000
F 0 "U2" H 3150 750 50  0000 C CNN
F 1 "LM1117-3.3" H 3050 1250 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 3050 1000 50  0001 C CNN
F 3 "" H 3050 1000 50  0000 C CNN
	1    3050 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1200 3450 1000
Connection ~ 3450 1000
Wire Wire Line
	3450 1800 3450 1500
$Comp
L R R18
U 1 1 5AD4429A
P 3850 1600
F 0 "R18" V 3930 1600 50  0000 C CNN
F 1 "39R" V 3850 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3780 1600 50  0001 C CNN
F 3 "" H 3850 1600 50  0000 C CNN
	1    3850 1600
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 5AD44319
P 3850 1250
F 0 "D2" H 3850 1350 50  0000 C CNN
F 1 "gn" H 3850 1150 50  0000 C CNN
F 2 "LEDs:LED_1206" H 3850 1250 50  0001 C CNN
F 3 "" H 3850 1250 50  0000 C CNN
	1    3850 1250
	0    -1   -1   0   
$EndComp
Connection ~ 3450 1800
Wire Wire Line
	3850 1450 3850 1400
Connection ~ 3850 1000
Wire Wire Line
	4950 2400 4950 3300
Connection ~ 4950 2800
Text GLabel 8600 2950 1    39   Input ~ 0
PROG
Wire Wire Line
	7700 3000 8800 3000
Text GLabel 5800 2500 1    39   Input ~ 0
RESET
$Comp
L R R19
U 1 1 5AD461C6
P 5200 2600
F 0 "R19" V 5250 2800 50  0000 C CNN
F 1 "10k" V 5200 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5130 2600 50  0001 C CNN
F 3 "" H 5200 2600 50  0000 C CNN
	1    5200 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 2600 5900 2600
Connection ~ 4950 2600
Wire Wire Line
	5800 2500 5800 2900
Connection ~ 5800 2600
$Comp
L C C3
U 1 1 5AD46D63
P 2550 1350
F 0 "C3" H 2575 1450 50  0000 L CNN
F 1 "10µ" H 2575 1250 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2588 1200 50  0001 C CNN
F 3 "" H 2550 1350 50  0000 C CNN
	1    2550 1350
	1    0    0    -1  
$EndComp
Connection ~ 2550 1000
Connection ~ 2550 1800
$Comp
L R R20
U 1 1 5AD47DE5
P 5200 2800
F 0 "R20" V 5250 3000 50  0000 C CNN
F 1 "10k" V 5200 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5130 2800 50  0001 C CNN
F 3 "" H 5200 2800 50  0000 C CNN
	1    5200 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 2800 5050 2800
Wire Wire Line
	5800 2900 5900 2900
Wire Wire Line
	5350 3000 5900 3000
Text GLabel 9950 1600 3    39   Output ~ 0
RX
Text GLabel 7800 2700 2    39   Input ~ 0
RX
Text GLabel 7800 2600 2    39   Output ~ 0
TX
Text GLabel 9750 1600 3    39   Input ~ 0
TX
Wire Wire Line
	7700 3100 8800 3100
$Comp
L R R27
U 1 1 5AD4AD28
P 8950 3000
F 0 "R27" V 8900 2800 50  0000 C CNN
F 1 "10k" V 8950 3000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8880 3000 50  0001 C CNN
F 3 "" H 8950 3000 50  0000 C CNN
	1    8950 3000
	0    1    1    0   
$EndComp
$Comp
L R R28
U 1 1 5AD4AF54
P 8950 3100
F 0 "R28" V 8900 2900 50  0000 C CNN
F 1 "10k" V 8950 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8880 3100 50  0001 C CNN
F 3 "" H 8950 3100 50  0000 C CNN
	1    8950 3100
	0    1    1    0   
$EndComp
$Comp
L R R26
U 1 1 5AD4AFDF
P 7950 3200
F 0 "R26" V 7900 3000 50  0000 C CNN
F 1 "10k" V 7950 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7880 3200 50  0001 C CNN
F 3 "" H 7950 3200 50  0000 C CNN
	1    7950 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 3200 8150 3200
Wire Wire Line
	8150 3200 8150 3400
Wire Wire Line
	8600 2950 8600 3000
Connection ~ 8600 3000
$Comp
L VCC #PWR28
U 1 1 5AD4B964
P 9200 2900
F 0 "#PWR28" H 9200 2750 50  0001 C CNN
F 1 "VCC" H 9200 3050 50  0000 C CNN
F 2 "" H 9200 2900 50  0000 C CNN
F 3 "" H 9200 2900 50  0000 C CNN
	1    9200 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 3100 9100 3100
Wire Wire Line
	9200 2900 9200 3100
Wire Wire Line
	9100 3000 9200 3000
Connection ~ 9200 3000
Text GLabel 5500 2500 1    39   Output ~ 0
STATUS
$Comp
L R R29
U 1 1 5AD4CF5B
P 7950 2900
F 0 "R29" V 7900 3100 50  0000 C CNN
F 1 "330R" V 7950 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7880 2900 50  0001 C CNN
F 3 "" H 7950 2900 50  0000 C CNN
	1    7950 2900
	0    1    1    0   
$EndComp
$Comp
L R R30
U 1 1 5AD4D088
P 7950 2800
F 0 "R30" V 7900 3000 50  0000 C CNN
F 1 "330R" V 7950 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7880 2800 50  0001 C CNN
F 3 "" H 7950 2800 50  0000 C CNN
	1    7950 2800
	0    1    1    0   
$EndComp
Text Notes 5500 6450 0    39   ~ 0
I2C address 0x18
$Comp
L MCP23016 U1
U 1 1 5AD3788D
P 2350 4500
F 0 "U1" H 2250 5525 50  0000 R CNN
F 1 "MCP23016" H 2250 5450 50  0000 R CNN
F 2 "Housings_SOIC:SOIC-28W_7.5x17.9mm_Pitch1.27mm" H 2500 3550 50  0001 L CNN
F 3 "" H 2600 5500 50  0001 C CNN
	1    2350 4500
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 5AD38C7F
P 2350 5700
F 0 "#PWR13" H 2350 5450 50  0001 C CNN
F 1 "GND" H 2350 5550 50  0000 C CNN
F 2 "" H 2350 5700 50  0000 C CNN
F 3 "" H 2350 5700 50  0000 C CNN
	1    2350 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5500 2250 5600
Wire Wire Line
	2250 5600 2450 5600
Wire Wire Line
	2450 5600 2450 5500
Wire Wire Line
	2350 5500 2350 5700
Connection ~ 2350 5600
$Comp
L GND #PWR11
U 1 1 5AD392B5
P 1700 5400
F 0 "#PWR11" H 1700 5150 50  0001 C CNN
F 1 "GND" H 1700 5250 50  0000 C CNN
F 2 "" H 1700 5400 50  0000 C CNN
F 3 "" H 1700 5400 50  0000 C CNN
	1    1700 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 5100 1700 5100
Wire Wire Line
	1700 5100 1700 5400
Wire Wire Line
	1850 5200 1700 5200
Connection ~ 1700 5200
Wire Wire Line
	1850 5300 1700 5300
Connection ~ 1700 5300
Text GLabel 1700 4800 0    39   BiDi ~ 0
SCL
Text GLabel 1700 4900 0    39   BiDi ~ 0
SDA
Wire Wire Line
	1700 4800 1850 4800
Wire Wire Line
	1700 4900 1850 4900
$Comp
L C C2
U 1 1 5AD3A16F
P 1400 3950
F 0 "C2" H 1425 4050 50  0000 L CNN
F 1 "33p" H 1425 3850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1438 3800 50  0001 C CNN
F 3 "" H 1400 3950 50  0000 C CNN
	1    1400 3950
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5AD3A3D3
P 1400 3500
F 0 "R1" V 1350 3350 50  0000 C CNN
F 1 "3k9" V 1400 3500 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1330 3500 50  0001 C CNN
F 3 "" H 1400 3500 50  0000 C CNN
	1    1400 3500
	-1   0    0    1   
$EndComp
Wire Wire Line
	1850 3700 1400 3700
Wire Wire Line
	1400 3650 1400 3800
Connection ~ 1400 3700
$Comp
L GND #PWR6
U 1 1 5AD3A94D
P 1400 4200
F 0 "#PWR6" H 1400 3950 50  0001 C CNN
F 1 "GND" H 1400 4050 50  0000 C CNN
F 2 "" H 1400 4200 50  0000 C CNN
F 3 "" H 1400 4200 50  0000 C CNN
	1    1400 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 4100 1400 4200
Text GLabel 1800 4200 0    39   Output ~ 0
IOINT
Wire Wire Line
	1850 4200 1800 4200
$Comp
L VCC #PWR5
U 1 1 5AD3B1E1
P 1400 3250
F 0 "#PWR5" H 1400 3100 50  0001 C CNN
F 1 "VCC" H 1400 3400 50  0000 C CNN
F 2 "" H 1400 3250 50  0000 C CNN
F 3 "" H 1400 3250 50  0000 C CNN
	1    1400 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3250 1400 3350
$Comp
L VCC #PWR12
U 1 1 5AD3B513
P 2350 3250
F 0 "#PWR12" H 2350 3100 50  0001 C CNN
F 1 "VCC" H 2350 3400 50  0000 C CNN
F 2 "" H 2350 3250 50  0000 C CNN
F 3 "" H 2350 3250 50  0000 C CNN
	1    2350 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3250 2350 3500
$Comp
L VCC #PWR2
U 1 1 5AD3BA06
P 1150 4700
F 0 "#PWR2" H 1150 4550 50  0001 C CNN
F 1 "VCC" H 1150 4850 50  0000 C CNN
F 2 "" H 1150 4700 50  0000 C CNN
F 3 "" H 1150 4700 50  0000 C CNN
	1    1150 4700
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5AD3BA9E
P 1150 4950
F 0 "C1" H 1175 5050 50  0000 L CNN
F 1 "100n" H 1175 4850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1188 4800 50  0001 C CNN
F 3 "" H 1150 4950 50  0000 C CNN
	1    1150 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR3
U 1 1 5AD3BB78
P 1150 5200
F 0 "#PWR3" H 1150 4950 50  0001 C CNN
F 1 "GND" H 1150 5050 50  0000 C CNN
F 2 "" H 1150 5200 50  0000 C CNN
F 3 "" H 1150 5200 50  0000 C CNN
	1    1150 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 4700 1150 4800
Wire Wire Line
	1150 5100 1150 5200
Text Notes 1350 5700 0    39   ~ 0
I2C address 0x20
Text GLabel 5700 2500 1    39   Input ~ 0
IOINT
Wire Wire Line
	5350 3200 5900 3200
$Comp
L R R23
U 1 1 5AD4F33A
P 5700 4450
F 0 "R23" V 5780 4450 50  0000 C CNN
F 1 "150R" V 5700 4450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5630 4450 50  0001 C CNN
F 3 "" H 5700 4450 50  0000 C CNN
	1    5700 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 4200 6200 4800
Wire Wire Line
	6200 4450 5850 4450
Wire Wire Line
	5200 4450 5200 4950
$Comp
L VCC #PWR23
U 1 1 5AD4F68E
P 6200 4200
F 0 "#PWR23" H 6200 4050 50  0001 C CNN
F 1 "VCC" H 6200 4350 50  0000 C CNN
F 2 "" H 6200 4200 50  0000 C CNN
F 3 "" H 6200 4200 50  0000 C CNN
	1    6200 4200
	1    0    0    -1  
$EndComp
Connection ~ 6200 4450
$Comp
L C C5
U 1 1 5AD4F92C
P 4950 4750
F 0 "C5" H 4975 4850 50  0000 L CNN
F 1 "100n" H 4975 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4988 4600 50  0001 C CNN
F 3 "" H 4950 4750 50  0000 C CNN
	1    4950 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 4900 4950 5200
Wire Wire Line
	4950 5100 5200 5100
$Comp
L GND #PWR18
U 1 1 5AD4FEAF
P 4950 5200
F 0 "#PWR18" H 4950 4950 50  0001 C CNN
F 1 "GND" H 4950 5050 50  0000 C CNN
F 2 "" H 4950 5200 50  0000 C CNN
F 3 "" H 4950 5200 50  0000 C CNN
	1    4950 5200
	1    0    0    -1  
$EndComp
Connection ~ 4950 5100
Text GLabel 6350 4950 2    39   Input ~ 0
STATUS
Wire Wire Line
	6350 4950 6200 4950
$Comp
L WS2812 D3
U 1 1 5AD50C56
P 5700 4950
F 0 "D3" H 5900 4600 60  0000 C CNN
F 1 "WS2812" H 5600 5200 60  0000 C CNN
F 2 "LEDs:LED_WS2812-PLCC6" H 5750 4200 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/WS2812.pdf" H 5950 4300 60  0001 C CNN
	1    5700 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR25
U 1 1 5AD51CC2
P 7500 1700
F 0 "#PWR25" H 7500 1450 50  0001 C CNN
F 1 "GND" H 7500 1550 50  0000 C CNN
F 2 "" H 7500 1700 50  0000 C CNN
F 3 "" H 7500 1700 50  0000 C CNN
	1    7500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 1800 3850 1750
Text GLabel 6100 1000 2    39   Output ~ 0
RESET
Wire Wire Line
	5600 1000 5700 1000
$Comp
L GND #PWR19
U 1 1 5AD69753
P 5050 1100
F 0 "#PWR19" H 5050 850 50  0001 C CNN
F 1 "GND" H 5050 950 50  0000 C CNN
F 2 "" H 5050 1100 50  0000 C CNN
F 3 "" H 5050 1100 50  0000 C CNN
	1    5050 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 1100 5050 1000
Wire Wire Line
	5050 1000 5200 1000
Text GLabel 6100 1400 2    39   Output ~ 0
PROG
Wire Wire Line
	5600 1400 5700 1400
$Comp
L GND #PWR20
U 1 1 5AD69C58
P 5050 1500
F 0 "#PWR20" H 5050 1250 50  0001 C CNN
F 1 "GND" H 5050 1350 50  0000 C CNN
F 2 "" H 5050 1500 50  0000 C CNN
F 3 "" H 5050 1500 50  0000 C CNN
	1    5050 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 1500 5050 1400
Wire Wire Line
	5050 1400 5200 1400
$Comp
L R R24
U 1 1 5AD6BCA7
P 5850 1000
F 0 "R24" V 5930 1000 50  0000 C CNN
F 1 "330R" V 5850 1000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5780 1000 50  0001 C CNN
F 3 "" H 5850 1000 50  0000 C CNN
	1    5850 1000
	0    1    1    0   
$EndComp
$Comp
L R R25
U 1 1 5AD6C15E
P 5850 1400
F 0 "R25" V 5930 1400 50  0000 C CNN
F 1 "330R" V 5850 1400 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5780 1400 50  0001 C CNN
F 3 "" H 5850 1400 50  0000 C CNN
	1    5850 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 1000 6100 1000
Wire Wire Line
	6000 1400 6100 1400
Wire Wire Line
	5700 2500 5700 3200
Connection ~ 5700 3200
$Comp
L R R22
U 1 1 5AD70D2C
P 5200 3200
F 0 "R22" V 5250 3400 50  0000 C CNN
F 1 "10k" V 5200 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5130 3200 50  0001 C CNN
F 3 "" H 5200 3200 50  0000 C CNN
	1    5200 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	5050 3200 4950 3200
Connection ~ 4950 3200
$Comp
L R R21
U 1 1 5AD719D3
P 5200 3000
F 0 "R21" V 5250 3200 50  0000 C CNN
F 1 "10k" V 5200 3000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5130 3000 50  0001 C CNN
F 3 "" H 5200 3000 50  0000 C CNN
	1    5200 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 3000 5050 3000
Connection ~ 4950 3000
Wire Wire Line
	5600 2500 5600 3000
Connection ~ 5600 3000
$Comp
L C C7
U 1 1 5AD7521C
P 7500 1400
F 0 "C7" H 7525 1500 50  0000 L CNN
F 1 "22µ" H 7525 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 7538 1250 50  0001 C CNN
F 3 "" H 7500 1400 50  0000 C CNN
	1    7500 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3100 5500 2500
Wire Wire Line
	5350 2800 5900 2800
Wire Wire Line
	5500 3100 5900 3100
Wire Wire Line
	7500 1050 7500 1250
Wire Wire Line
	7500 1550 7500 1700
$Comp
L VCC #PWR24
U 1 1 5AD7B5BE
P 7500 1050
F 0 "#PWR24" H 7500 900 50  0001 C CNN
F 1 "VCC" H 7500 1200 50  0000 C CNN
F 2 "" H 7500 1050 50  0000 C CNN
F 3 "" H 7500 1050 50  0000 C CNN
	1    7500 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2600 4950 2600
$Comp
L GND #PWR32
U 1 1 5AD82799
P 10050 5400
F 0 "#PWR32" H 10050 5150 50  0001 C CNN
F 1 "GND" H 10050 5250 50  0000 C CNN
F 2 "" H 10050 5400 50  0000 C CNN
F 3 "" H 10050 5400 50  0000 C CNN
	1    10050 5400
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR31
U 1 1 5AD8279F
P 10050 4700
F 0 "#PWR31" H 10050 4550 50  0001 C CNN
F 1 "VCC" H 10050 4850 50  0000 C CNN
F 2 "" H 10050 4700 50  0000 C CNN
F 3 "" H 10050 4700 50  0000 C CNN
	1    10050 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 4700 10050 4800
Wire Wire Line
	10050 4800 10200 4800
Wire Wire Line
	10050 5400 10050 5300
Wire Wire Line
	10050 5300 10200 5300
Text GLabel 10000 5000 0    39   Output ~ 0
MISO
Text GLabel 10000 5100 0    39   Input ~ 0
MOSI
Text GLabel 10000 5200 0    39   Input ~ 0
SCK
Wire Wire Line
	10200 4900 10000 4900
Wire Wire Line
	10200 5000 10000 5000
Wire Wire Line
	10200 5100 10000 5100
Text GLabel 6650 3900 3    39   Input ~ 0
MISO
Wire Wire Line
	6650 3900 6650 3800
Text GLabel 6950 3900 3    39   Output ~ 0
MOSI
Wire Wire Line
	6950 3800 6950 3900
Text GLabel 7050 3900 3    39   Output ~ 0
SCK
Wire Wire Line
	7050 3900 7050 3800
$Comp
L CONN_01X06 P3
U 1 1 5AD852A1
P 10400 5050
F 0 "P3" H 10400 5400 50  0000 C CNN
F 1 "CONN_01X06" V 10500 5050 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x06" H 10400 5050 50  0001 C CNN
F 3 "" H 10400 5050 50  0000 C CNN
	1    10400 5050
	1    0    0    -1  
$EndComp
Text GLabel 10000 4900 0    39   Input ~ 0
CS
Wire Wire Line
	10000 5200 10200 5200
Text GLabel 6550 3900 3    39   Output ~ 0
CS
Wire Wire Line
	6550 3800 6550 3900
Wire Wire Line
	7800 3200 7700 3200
$Comp
L SW_SPST SW1
U 1 1 5AD68F78
P 5400 1000
F 0 "SW1" H 5400 1125 50  0000 C CNN
F 1 "SW_SPST" H 5400 900 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVQPE1" H 5400 1000 50  0001 C CNN
F 3 "" H 5400 1000 50  0000 C CNN
	1    5400 1000
	1    0    0    -1  
$EndComp
$Comp
L SW_SPST SW2
U 1 1 5ADB847E
P 5400 1400
F 0 "SW2" H 5400 1525 50  0000 C CNN
F 1 "SW_SPST" H 5400 1300 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVQPE1" H 5400 1400 50  0001 C CNN
F 3 "" H 5400 1400 50  0000 C CNN
	1    5400 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 1600 9200 1600
Connection ~ 8950 1600
Wire Wire Line
	5200 4450 5550 4450
Wire Wire Line
	4950 4600 4950 4300
Wire Wire Line
	4950 4300 6200 4300
Connection ~ 6200 4300
Wire Wire Line
	7700 3300 8150 3300
Connection ~ 8150 3300
Wire Wire Line
	8100 2800 8250 2800
Wire Wire Line
	8100 2900 8250 2900
$Comp
L CONN_01X03 P4
U 1 1 5B82EAAC
P 10450 2400
F 0 "P4" H 10450 2600 50  0000 C CNN
F 1 "CONN_01X03" V 10550 2400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 10450 2400 50  0001 C CNN
F 3 "" H 10450 2400 50  0000 C CNN
	1    10450 2400
	1    0    0    -1  
$EndComp
Text GLabel 10100 2300 0    39   BiDi ~ 0
SCL
Text GLabel 10100 2500 0    39   BiDi ~ 0
SDA
$Comp
L GND #PWR29
U 1 1 5B82F09B
P 9750 2500
F 0 "#PWR29" H 9750 2250 50  0001 C CNN
F 1 "GND" H 9750 2350 50  0000 C CNN
F 2 "" H 9750 2500 50  0000 C CNN
F 3 "" H 9750 2500 50  0000 C CNN
	1    9750 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2300 10100 2300
Wire Wire Line
	10250 2500 10100 2500
Wire Wire Line
	10250 2400 9750 2400
Wire Wire Line
	9750 2400 9750 2500
$Comp
L R R33
U 1 1 5BA92E19
P 1100 2600
F 0 "R33" V 1000 2600 50  0000 C CNN
F 1 "0" V 1100 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1030 2600 50  0001 C CNN
F 3 "" H 1100 2600 50  0000 C CNN
	1    1100 2600
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR1
U 1 1 5BA92F35
P 1100 2800
F 0 "#PWR1" H 1100 2550 50  0001 C CNN
F 1 "GND" H 1100 2650 50  0000 C CNN
F 2 "" H 1100 2800 50  0000 C CNN
F 3 "" H 1100 2800 50  0000 C CNN
	1    1100 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 2400 1100 2450
Wire Wire Line
	1100 2750 1100 2800
$EndSCHEMATC
