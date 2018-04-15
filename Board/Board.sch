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
Date "2018-04-15"
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
P 6900 3350
F 0 "U?" H 6900 3250 50  0000 C CNN
F 1 "ESP-12E" H 6900 3450 50  0000 C CNN
F 2 "ESP8266:ESP-12E" H 6900 3350 50  0001 C CNN
F 3 "" H 6900 3350 50  0001 C CNN
	1    6900 3350
	1    0    0    -1  
$EndComp
$Comp
L MCP9808 U?
U 1 1 5AD1E1BA
P 5150 6600
F 0 "U?" H 4900 6900 60  0000 C CNN
F 1 "MCP9808" H 5450 6900 60  0000 C CNN
F 2 "Housings_SSOP:MSOP-8_3x3mm_Pitch0.65mm" H 5200 5850 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/MC9808.pdf" H 5350 6000 60  0001 C CNN
F 4 "MCP 9808-EMS" H 5100 7200 60  0001 C CNN "REICHELT"
F 5 "http://www.reichelt.de/index.html?ARTICLE=137341" H 5200 7350 60  0001 C CNN "REICHELT URI"
	1    5150 6600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD1E3D6
P 5400 2850
F 0 "#PWR?" H 5400 2700 50  0001 C CNN
F 1 "VCC" H 5400 3000 50  0000 C CNN
F 2 "" H 5400 2850 50  0000 C CNN
F 3 "" H 5400 2850 50  0000 C CNN
	1    5400 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD1E3E7
P 8150 3850
F 0 "#PWR?" H 8150 3600 50  0001 C CNN
F 1 "GND" H 8150 3700 50  0000 C CNN
F 2 "" H 8150 3850 50  0000 C CNN
F 3 "" H 8150 3850 50  0000 C CNN
	1    8150 3850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AD1E52B
P 6150 6600
F 0 "C?" H 6175 6700 50  0000 L CNN
F 1 "100n" H 6175 6500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6188 6450 50  0001 C CNN
F 3 "" H 6150 6600 50  0000 C CNN
	1    6150 6600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD1E585
P 6150 6000
F 0 "#PWR?" H 6150 5850 50  0001 C CNN
F 1 "VCC" H 6150 6150 50  0000 C CNN
F 2 "" H 6150 6000 50  0000 C CNN
F 3 "" H 6150 6000 50  0000 C CNN
	1    6150 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD1E599
P 6150 7250
F 0 "#PWR?" H 6150 7000 50  0001 C CNN
F 1 "GND" H 6150 7100 50  0000 C CNN
F 2 "" H 6150 7250 50  0000 C CNN
F 3 "" H 6150 7250 50  0000 C CNN
	1    6150 7250
	1    0    0    -1  
$EndComp
Text GLabel 4450 6450 0    39   BiDi ~ 0
SDA
Text GLabel 4450 6550 0    39   BiDi ~ 0
SCL
Text GLabel 4450 6750 0    39   Output ~ 0
Alert
$Comp
L CONN_01X05 P?
U 1 1 5AD1E853
P 10250 800
F 0 "P?" H 10250 1100 50  0000 C CNN
F 1 "CONN_01X05" V 10350 800 50  0000 C CNN
F 2 "" H 10250 800 50  0000 C CNN
F 3 "" H 10250 800 50  0000 C CNN
	1    10250 800 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD1E8CC
P 10150 1050
F 0 "#PWR?" H 10150 800 50  0001 C CNN
F 1 "GND" H 10150 900 50  0000 C CNN
F 2 "" H 10150 1050 50  0000 C CNN
F 3 "" H 10150 1050 50  0000 C CNN
	1    10150 1050
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5AD1E910
P 10700 1000
F 0 "#PWR?" H 10700 850 50  0001 C CNN
F 1 "VCC" H 10700 1150 50  0000 C CNN
F 2 "" H 10700 1000 50  0000 C CNN
F 3 "" H 10700 1000 50  0000 C CNN
	1    10700 1000
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR?
U 1 1 5AD1E925
P 10900 1000
F 0 "#PWR?" H 10900 850 50  0001 C CNN
F 1 "VDD" H 10900 1150 50  0000 C CNN
F 2 "" H 10900 1000 50  0000 C CNN
F 3 "" H 10900 1000 50  0000 C CNN
	1    10900 1000
	1    0    0    -1  
$EndComp
Text GLabel 7900 3350 2    39   BiDi ~ 0
SDA
Text GLabel 7900 3250 2    39   BiDi ~ 0
SCL
Text GLabel 5800 3450 0    39   Input ~ 0
Alert
$Comp
L VCC #PWR?
U 1 1 5AD21696
P 1150 7450
F 0 "#PWR?" H 1150 7300 50  0001 C CNN
F 1 "VCC" H 1150 7600 50  0000 C CNN
F 2 "" H 1150 7450 50  0000 C CNN
F 3 "" H 1150 7450 50  0000 C CNN
	1    1150 7450
	1    0    0    -1  
$EndComp
$Comp
L VQE24 D?
U 1 1 5AD34FD0
P 2300 6700
F 0 "D?" H 2950 6050 60  0000 C CNN
F 1 "VQE24" H 2300 6250 60  0000 C CNN
F 2 "" H 2150 6550 60  0001 C CNN
F 3 "https://github.com/netz39/kicad_parts/raw/master/datasheets/VQE_7segment.pdf" H 2500 5750 60  0001 C CNN
	1    2300 6700
	1    0    0    -1  
$EndComp
Text GLabel 3450 4600 2    39   Output ~ 0
a1
Text GLabel 3450 4700 2    39   Output ~ 0
b1
Text GLabel 3450 4800 2    39   Output ~ 0
c1
Text GLabel 3450 5000 2    39   Output ~ 0
e1
Text GLabel 3450 5100 2    39   Output ~ 0
f1
Text GLabel 3450 5200 2    39   Output ~ 0
g1
Text GLabel 3450 4900 2    39   Output ~ 0
d1
Text GLabel 3450 5300 2    39   Output ~ 0
h1
$Comp
L R R?
U 1 1 5AD385D2
P 3150 4600
F 0 "R?" V 3100 4450 50  0000 C CNN
F 1 "39R" V 3150 4600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4600 50  0001 C CNN
F 3 "" H 3150 4600 50  0000 C CNN
	1    3150 4600
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3864D
P 3150 4700
F 0 "R?" V 3100 4550 50  0000 C CNN
F 1 "39R" V 3150 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4700 50  0001 C CNN
F 3 "" H 3150 4700 50  0000 C CNN
	1    3150 4700
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3867E
P 3150 4800
F 0 "R?" V 3100 4650 50  0000 C CNN
F 1 "39R" V 3150 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4800 50  0001 C CNN
F 3 "" H 3150 4800 50  0000 C CNN
	1    3150 4800
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD386B2
P 3150 4900
F 0 "R?" V 3100 4750 50  0000 C CNN
F 1 "39R" V 3150 4900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4900 50  0001 C CNN
F 3 "" H 3150 4900 50  0000 C CNN
	1    3150 4900
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3871E
P 3150 5000
F 0 "R?" V 3100 4850 50  0000 C CNN
F 1 "39R" V 3150 5000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 5000 50  0001 C CNN
F 3 "" H 3150 5000 50  0000 C CNN
	1    3150 5000
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD38758
P 3150 5100
F 0 "R?" V 3100 4950 50  0000 C CNN
F 1 "39R" V 3150 5100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 5100 50  0001 C CNN
F 3 "" H 3150 5100 50  0000 C CNN
	1    3150 5100
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD38795
P 3150 5200
F 0 "R?" V 3100 5050 50  0000 C CNN
F 1 "39R" V 3150 5200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 5200 50  0001 C CNN
F 3 "" H 3150 5200 50  0000 C CNN
	1    3150 5200
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD387DB
P 3150 5300
F 0 "R?" V 3100 5150 50  0000 C CNN
F 1 "39R" V 3150 5300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 5300 50  0001 C CNN
F 3 "" H 3150 5300 50  0000 C CNN
	1    3150 5300
	0    1    1    0   
$EndComp
Text GLabel 3450 3700 2    39   Output ~ 0
a2
Text GLabel 3450 3800 2    39   Output ~ 0
b2
Text GLabel 3450 3900 2    39   Output ~ 0
c2
Text GLabel 3450 4100 2    39   Output ~ 0
e2
Text GLabel 3450 4200 2    39   Output ~ 0
f2
Text GLabel 3450 4300 2    39   Output ~ 0
g2
Text GLabel 3450 4000 2    39   Output ~ 0
d2
Text GLabel 3450 4400 2    39   Output ~ 0
h2
$Comp
L R R?
U 1 1 5AD3968C
P 3150 3700
F 0 "R?" V 3100 3550 50  0000 C CNN
F 1 "39R" V 3150 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 3700 50  0001 C CNN
F 3 "" H 3150 3700 50  0000 C CNN
	1    3150 3700
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD39692
P 3150 3800
F 0 "R?" V 3100 3650 50  0000 C CNN
F 1 "39R" V 3150 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 3800 50  0001 C CNN
F 3 "" H 3150 3800 50  0000 C CNN
	1    3150 3800
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD39698
P 3150 3900
F 0 "R?" V 3100 3750 50  0000 C CNN
F 1 "39R" V 3150 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 3900 50  0001 C CNN
F 3 "" H 3150 3900 50  0000 C CNN
	1    3150 3900
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD3969E
P 3150 4000
F 0 "R?" V 3100 3850 50  0000 C CNN
F 1 "39R" V 3150 4000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4000 50  0001 C CNN
F 3 "" H 3150 4000 50  0000 C CNN
	1    3150 4000
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396A4
P 3150 4100
F 0 "R?" V 3100 3950 50  0000 C CNN
F 1 "39R" V 3150 4100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4100 50  0001 C CNN
F 3 "" H 3150 4100 50  0000 C CNN
	1    3150 4100
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396AA
P 3150 4200
F 0 "R?" V 3100 4050 50  0000 C CNN
F 1 "39R" V 3150 4200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4200 50  0001 C CNN
F 3 "" H 3150 4200 50  0000 C CNN
	1    3150 4200
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396B0
P 3150 4300
F 0 "R?" V 3100 4150 50  0000 C CNN
F 1 "39R" V 3150 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4300 50  0001 C CNN
F 3 "" H 3150 4300 50  0000 C CNN
	1    3150 4300
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD396B6
P 3150 4400
F 0 "R?" V 3100 4250 50  0000 C CNN
F 1 "39R" V 3150 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 4400 50  0001 C CNN
F 3 "" H 3150 4400 50  0000 C CNN
	1    3150 4400
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
Text GLabel 10750 2450 2    39   BiDi ~ 0
SCL
Text GLabel 10750 2800 2    39   BiDi ~ 0
SDA
$Comp
L R R?
U 1 1 5AD3CEF0
P 10650 2300
F 0 "R?" V 10730 2300 50  0000 C CNN
F 1 "10k" V 10650 2300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10580 2300 50  0001 C CNN
F 3 "" H 10650 2300 50  0000 C CNN
	1    10650 2300
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5AD3CF7D
P 10500 2650
F 0 "R?" V 10580 2650 50  0000 C CNN
F 1 "10k" V 10500 2650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10430 2650 50  0001 C CNN
F 3 "" H 10500 2650 50  0000 C CNN
	1    10500 2650
	1    0    0    -1  
$EndComp
Text GLabel 10000 2450 0    39   BiDi ~ 0
SCL
Text GLabel 10000 2800 0    39   BiDi ~ 0
SDA
$Comp
L VCC #PWR?
U 1 1 5AD3D501
P 10500 2000
F 0 "#PWR?" H 10500 1850 50  0001 C CNN
F 1 "VCC" H 10500 2150 50  0000 C CNN
F 2 "" H 10500 2000 50  0000 C CNN
F 3 "" H 10500 2000 50  0000 C CNN
	1    10500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 7050 5150 7150
Wire Wire Line
	7800 3750 8150 3750
Wire Wire Line
	5400 3750 6000 3750
Wire Wire Line
	6150 6000 6150 6450
Wire Wire Line
	6150 6750 6150 7250
Wire Wire Line
	4450 6450 4650 6450
Wire Wire Line
	4450 6550 4650 6550
Wire Wire Line
	4450 6750 4650 6750
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
	7800 3050 7900 3050
Wire Wire Line
	10050 1300 10050 1000
Wire Wire Line
	7800 3150 7900 3150
Wire Wire Line
	10250 1300 10250 1000
Wire Wire Line
	10150 1000 10150 1050
Wire Wire Line
	10450 1000 10450 1050
Wire Wire Line
	10450 1050 10700 1050
Wire Wire Line
	10700 1050 10700 1000
Wire Wire Line
	10350 1000 10350 1200
Wire Wire Line
	10350 1200 10900 1200
Wire Wire Line
	10900 1200 10900 1000
Wire Wire Line
	7800 3250 7900 3250
Wire Wire Line
	7800 3350 7900 3350
Wire Wire Line
	2850 4600 3000 4600
Wire Wire Line
	2850 4700 3000 4700
Wire Wire Line
	2850 4800 3000 4800
Wire Wire Line
	2850 4900 3000 4900
Wire Wire Line
	2850 5000 3000 5000
Wire Wire Line
	2850 5100 3000 5100
Wire Wire Line
	2850 5200 3000 5200
Wire Wire Line
	2850 5300 3000 5300
Wire Wire Line
	3300 4900 3450 4900
Wire Wire Line
	3300 5000 3450 5000
Wire Wire Line
	3300 5100 3450 5100
Wire Wire Line
	3300 5200 3450 5200
Wire Wire Line
	3300 5300 3450 5300
Wire Wire Line
	3300 4600 3450 4600
Wire Wire Line
	3300 4700 3450 4700
Wire Wire Line
	3300 4800 3450 4800
Wire Wire Line
	2850 3700 3000 3700
Wire Wire Line
	2850 3800 3000 3800
Wire Wire Line
	2850 3900 3000 3900
Wire Wire Line
	2850 4000 3000 4000
Wire Wire Line
	2850 4100 3000 4100
Wire Wire Line
	2850 4200 3000 4200
Wire Wire Line
	2850 4300 3000 4300
Wire Wire Line
	2850 4400 3000 4400
Wire Wire Line
	3300 4000 3450 4000
Wire Wire Line
	3300 4100 3450 4100
Wire Wire Line
	3300 4200 3450 4200
Wire Wire Line
	3300 4300 3450 4300
Wire Wire Line
	3300 4400 3450 4400
Wire Wire Line
	3300 3700 3450 3700
Wire Wire Line
	3300 3800 3450 3800
Wire Wire Line
	3300 3900 3450 3900
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
	10350 2450 10750 2450
Connection ~ 10650 2450
Wire Wire Line
	10500 2000 10500 2500
Wire Wire Line
	10500 2100 10650 2100
Wire Wire Line
	10650 2100 10650 2150
Connection ~ 10500 2100
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
L Screw_Terminal_1x02 J?
U 1 1 5AD3EFB6
P 1100 1100
F 0 "J?" H 1100 1350 50  0000 C TNN
F 1 "Screw_Terminal_1x02" V 950 1100 50  0000 C TNN
F 2 "n39-kicad:AKL_101-02" H 1100 875 50  0001 C CNN
F 3 "" H 1075 1100 50  0001 C CNN
	1    1100 1100
	1    0    0    -1  
$EndComp
$Comp
L USB_B P?
U 1 1 5AD3F1CE
P 1200 2100
F 0 "P?" H 1400 1900 50  0000 C CNN
F 1 "USB_B" H 1150 2300 50  0000 C CNN
F 2 "Connect:USB_Micro-B" V 1150 2000 50  0001 C CNN
F 3 "" V 1150 2000 50  0000 C CNN
	1    1200 2100
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD3FEC4
P 1650 2300
F 0 "#PWR?" H 1650 2050 50  0001 C CNN
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
L GND #PWR?
U 1 1 5AD40223
P 1650 1300
F 0 "#PWR?" H 1650 1050 50  0001 C CNN
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
L VDD #PWR?
U 1 1 5AD40ECC
P 1650 900
F 0 "#PWR?" H 1650 750 50  0001 C CNN
F 1 "VDD" H 1650 1050 50  0000 C CNN
F 2 "" H 1650 900 50  0000 C CNN
F 3 "" H 1650 900 50  0000 C CNN
	1    1650 900 
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR?
U 1 1 5AD411EA
P 1650 1800
F 0 "#PWR?" H 1650 1650 50  0001 C CNN
F 1 "VDD" H 1650 1950 50  0000 C CNN
F 2 "" H 1650 1800 50  0000 C CNN
F 3 "" H 1650 1800 50  0000 C CNN
	1    1650 1800
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR?
U 1 1 5AD4147C
P 2500 900
F 0 "#PWR?" H 2500 750 50  0001 C CNN
F 1 "VDD" H 2500 1050 50  0000 C CNN
F 2 "" H 2500 900 50  0000 C CNN
F 3 "" H 2500 900 50  0000 C CNN
	1    2500 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD414E4
P 2500 1900
F 0 "#PWR?" H 2500 1650 50  0001 C CNN
F 1 "GND" H 2500 1750 50  0000 C CNN
F 2 "" H 2500 1900 50  0000 C CNN
F 3 "" H 2500 1900 50  0000 C CNN
	1    2500 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 900  2500 1250
Wire Wire Line
	2500 1000 3000 1000
Wire Wire Line
	2500 1550 2500 1900
Wire Wire Line
	2500 1800 4200 1800
Wire Wire Line
	3300 1800 3300 1300
$Comp
L C C?
U 1 1 5AD419E3
P 2800 1400
F 0 "C?" H 2825 1500 50  0000 L CNN
F 1 "0.1µ" H 2825 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2838 1250 50  0001 C CNN
F 3 "" H 2800 1400 50  0000 C CNN
	1    2800 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 1250 2800 1000
Connection ~ 2800 1000
Wire Wire Line
	2800 1550 2800 1800
Connection ~ 2800 1800
$Comp
L VCC #PWR?
U 1 1 5AD41DB7
P 4200 900
F 0 "#PWR?" H 4200 750 50  0001 C CNN
F 1 "VCC" H 4200 1050 50  0000 C CNN
F 2 "" H 4200 900 50  0000 C CNN
F 3 "" H 4200 900 50  0000 C CNN
	1    4200 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 1000 4200 1000
Wire Wire Line
	4200 900  4200 1100
$Comp
L C C?
U 1 1 5AD41FA2
P 3800 1400
F 0 "C?" H 3825 1500 50  0000 L CNN
F 1 "0.1µ" H 3825 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3838 1250 50  0001 C CNN
F 3 "" H 3800 1400 50  0000 C CNN
	1    3800 1400
	1    0    0    -1  
$EndComp
Connection ~ 3300 1800
$Comp
L LM1117-3.3 U?
U 1 1 5AD42652
P 3300 1000
F 0 "U?" H 3400 750 50  0000 C CNN
F 1 "LM1117-3.3" H 3300 1250 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 3300 1000 50  0001 C CNN
F 3 "" H 3300 1000 50  0000 C CNN
	1    3300 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1250 3800 1000
Connection ~ 3800 1000
Wire Wire Line
	3800 1800 3800 1550
$Comp
L R R?
U 1 1 5AD4429A
P 4200 1650
F 0 "R?" V 4280 1650 50  0000 C CNN
F 1 "R" V 4200 1650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4130 1650 50  0001 C CNN
F 3 "" H 4200 1650 50  0000 C CNN
	1    4200 1650
	1    0    0    -1  
$EndComp
$Comp
L LED D?
U 1 1 5AD44319
P 4200 1250
F 0 "D?" H 4200 1350 50  0000 C CNN
F 1 "gn" H 4200 1150 50  0000 C CNN
F 2 "LEDs:LED_0603" H 4200 1250 50  0001 C CNN
F 3 "" H 4200 1250 50  0000 C CNN
	1    4200 1250
	0    -1   -1   0   
$EndComp
Connection ~ 3800 1800
Wire Wire Line
	4200 1500 4200 1400
Connection ~ 4200 1000
Wire Wire Line
	5400 2850 5400 3750
Connection ~ 5400 3250
Text GLabel 8200 3400 1    39   Input ~ 0
PROG
Wire Wire Line
	7800 3450 8300 3450
Text GLabel 5900 3000 1    39   Input ~ 0
RESET
$Comp
L R R?
U 1 1 5AD461C6
P 5650 3050
F 0 "R?" V 5730 3050 50  0000 C CNN
F 1 "10k" V 5650 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5580 3050 50  0001 C CNN
F 3 "" H 5650 3050 50  0000 C CNN
	1    5650 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	5800 3050 6000 3050
Wire Wire Line
	5500 3050 5400 3050
Connection ~ 5400 3050
Wire Wire Line
	5900 3000 5900 3350
Connection ~ 5900 3050
$Comp
L C C?
U 1 1 5AD46D63
P 2500 1400
F 0 "C?" H 2525 1500 50  0000 L CNN
F 1 "100µ" H 2525 1300 50  0000 L CNN
F 2 "" H 2538 1250 50  0001 C CNN
F 3 "" H 2500 1400 50  0000 C CNN
	1    2500 1400
	1    0    0    -1  
$EndComp
Connection ~ 2500 1000
Connection ~ 2500 1800
$Comp
L R R?
U 1 1 5AD47DE5
P 5650 3250
F 0 "R?" V 5730 3250 50  0000 C CNN
F 1 "10k" V 5650 3250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5580 3250 50  0001 C CNN
F 3 "" H 5650 3250 50  0000 C CNN
	1    5650 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 3250 5500 3250
Wire Wire Line
	5800 3250 6000 3250
Wire Wire Line
	5900 3350 6000 3350
Wire Wire Line
	5800 3450 6000 3450
Text GLabel 10050 1300 3    39   Output ~ 0
RX
Text GLabel 7900 3150 2    39   Input ~ 0
RX
Text GLabel 7900 3050 2    39   Output ~ 0
TX
Text GLabel 10250 1300 3    39   Input ~ 0
TX
Wire Wire Line
	7800 3550 8300 3550
$Comp
L R R?
U 1 1 5AD4AD28
P 8450 3450
F 0 "R?" V 8400 3300 50  0000 C CNN
F 1 "10k" V 8450 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8380 3450 50  0001 C CNN
F 3 "" H 8450 3450 50  0000 C CNN
	1    8450 3450
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD4AF54
P 8450 3550
F 0 "R?" V 8400 3400 50  0000 C CNN
F 1 "10k" V 8450 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8380 3550 50  0001 C CNN
F 3 "" H 8450 3550 50  0000 C CNN
	1    8450 3550
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD4AFDF
P 7950 3650
F 0 "R?" V 7900 3500 50  0000 C CNN
F 1 "10k" V 7950 3650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7880 3650 50  0001 C CNN
F 3 "" H 7950 3650 50  0000 C CNN
	1    7950 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 3650 8150 3650
Wire Wire Line
	8150 3650 8150 3850
Wire Wire Line
	8200 3400 8200 3450
Connection ~ 8200 3450
$Comp
L VCC #PWR?
U 1 1 5AD4B964
P 8700 3350
F 0 "#PWR?" H 8700 3200 50  0001 C CNN
F 1 "VCC" H 8700 3500 50  0000 C CNN
F 2 "" H 8700 3350 50  0000 C CNN
F 3 "" H 8700 3350 50  0000 C CNN
	1    8700 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 3550 8600 3550
Wire Wire Line
	8700 3350 8700 3550
Wire Wire Line
	8600 3450 8700 3450
Connection ~ 8700 3450
Text GLabel 5800 3550 0    39   Output ~ 0
STATUS
Wire Wire Line
	6000 3550 5800 3550
$Comp
L R R?
U 1 1 5AD4CF5B
P 10200 2450
F 0 "R?" V 10280 2450 50  0000 C CNN
F 1 "330R" V 10200 2450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10130 2450 50  0001 C CNN
F 3 "" H 10200 2450 50  0000 C CNN
	1    10200 2450
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5AD4D088
P 10200 2800
F 0 "R?" V 10280 2800 50  0000 C CNN
F 1 "330R" V 10200 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10130 2800 50  0001 C CNN
F 3 "" H 10200 2800 50  0000 C CNN
	1    10200 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	10000 2450 10050 2450
Wire Wire Line
	10000 2800 10050 2800
Wire Wire Line
	10350 2800 10750 2800
Text Notes 5500 6450 0    39   ~ 0
I2C address 0x18
$Comp
L MCP23016 U?
U 1 1 5AD3788D
P 2350 4500
F 0 "U?" H 2250 5525 50  0000 R CNN
F 1 "MCP23016" H 2250 5450 50  0000 R CNN
F 2 "SMD_Packages:SOIC-28" H 2500 3550 50  0001 L CNN
F 3 "" H 2600 5500 50  0001 C CNN
	1    2350 4500
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD38C7F
P 2350 5700
F 0 "#PWR?" H 2350 5450 50  0001 C CNN
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
L GND #PWR?
U 1 1 5AD392B5
P 1700 5400
F 0 "#PWR?" H 1700 5150 50  0001 C CNN
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
L C C?
U 1 1 5AD3A16F
P 1400 3950
F 0 "C?" H 1425 4050 50  0000 L CNN
F 1 "33p" H 1425 3850 50  0000 L CNN
F 2 "" H 1438 3800 50  0001 C CNN
F 3 "" H 1400 3950 50  0000 C CNN
	1    1400 3950
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5AD3A3D3
P 1400 3500
F 0 "R?" V 1350 3350 50  0000 C CNN
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
L GND #PWR?
U 1 1 5AD3A94D
P 1400 4200
F 0 "#PWR?" H 1400 3950 50  0001 C CNN
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
L VCC #PWR?
U 1 1 5AD3B1E1
P 1400 3250
F 0 "#PWR?" H 1400 3100 50  0001 C CNN
F 1 "VCC" H 1400 3400 50  0000 C CNN
F 2 "" H 1400 3250 50  0000 C CNN
F 3 "" H 1400 3250 50  0000 C CNN
	1    1400 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3250 1400 3350
$Comp
L VCC #PWR?
U 1 1 5AD3B513
P 2350 3250
F 0 "#PWR?" H 2350 3100 50  0001 C CNN
F 1 "VCC" H 2350 3400 50  0000 C CNN
F 2 "" H 2350 3250 50  0000 C CNN
F 3 "" H 2350 3250 50  0000 C CNN
	1    2350 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3250 2350 3500
$Comp
L VCC #PWR?
U 1 1 5AD3BA06
P 1150 4700
F 0 "#PWR?" H 1150 4550 50  0001 C CNN
F 1 "VCC" H 1150 4850 50  0000 C CNN
F 2 "" H 1150 4700 50  0000 C CNN
F 3 "" H 1150 4700 50  0000 C CNN
	1    1150 4700
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AD3BA9E
P 1150 4950
F 0 "C?" H 1175 5050 50  0000 L CNN
F 1 "100µ" H 1175 4850 50  0000 L CNN
F 2 "" H 1188 4800 50  0001 C CNN
F 3 "" H 1150 4950 50  0000 C CNN
	1    1150 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD3BB78
P 1150 5200
F 0 "#PWR?" H 1150 4950 50  0001 C CNN
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
Text GLabel 5800 3650 0    39   Input ~ 0
IOINT
Wire Wire Line
	5800 3650 6000 3650
$EndSCHEMATC
