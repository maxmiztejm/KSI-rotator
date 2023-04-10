EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L TCST2300:TCST2103 U1
U 1 1 61AC5883
P 3350 2400
F 0 "U1" H 3008 2665 50  0000 C CNN
F 1 "TCST2103" H 3008 2574 50  0000 C CNN
F 2 "KSI_rotator_endstop:Vishay-TCST2300" H 3350 2800 50  0001 L CNN
F 3 "http://www.vishay.com/docs/81147/tcst2103.pdf" H 3008 2573 50  0001 C CNN
F 4 "IC" H 3350 3000 50  0001 L CNN "category"
F 5 "TCST2103-ND" H 3350 3100 50  0001 L CNN "digikey description"
F 6 "SENSR OPTO SLOT 3.1MM TRANS THRU" H 3350 3200 50  0001 L CNN "digikey part number"
F 7 "yes" H 3350 3300 50  0001 L CNN "lead free"
F 8 "f2a1018269320391" H 3350 3400 50  0001 L CNN "library id"
F 9 "Vishay" H 3350 3500 50  0001 L CNN "manufacturer"
F 10 "782-TCST2103" H 3350 3600 50  0001 L CNN "mouser part number"
F 11 "Slotted Module, 4-Lead Dual Row" H 3350 3700 50  0001 L CNN "package"
F 12 "yes" H 3350 3800 50  0001 L CNN "rohs"
F 13 "+85°C" H 3350 3900 50  0001 L CNN "temperature range high"
F 14 "-55°C" H 3350 4000 50  0001 L CNN "temperature range low"
	1    3350 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 61AC88F6
P 3350 2250
F 0 "R1" H 3291 2204 50  0000 R CNN
F 1 "100" H 3291 2295 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3350 2250 50  0001 C CNN
F 3 "~" H 3350 2250 50  0001 C CNN
	1    3350 2250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 61AC97E5
P 3350 2850
F 0 "#PWR0101" H 3350 2600 50  0001 C CNN
F 1 "GND" H 3355 2677 50  0000 C CNN
F 2 "" H 3350 2850 50  0001 C CNN
F 3 "" H 3350 2850 50  0001 C CNN
	1    3350 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2700 3250 2700
$Comp
L power:+5V #PWR0102
U 1 1 61ACAA1F
P 3350 2100
F 0 "#PWR0102" H 3350 1950 50  0001 C CNN
F 1 "+5V" H 3365 2273 50  0000 C CNN
F 2 "" H 3350 2100 50  0001 C CNN
F 3 "" H 3350 2100 50  0001 C CNN
	1    3350 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2400 3250 2400
Wire Wire Line
	3250 2800 3350 2800
Wire Wire Line
	3350 2800 3350 2850
Wire Wire Line
	3350 2700 3350 2800
Connection ~ 3350 2800
$Comp
L Device:R_Small R2
U 1 1 61ACD0CF
P 3650 2250
F 0 "R2" H 3591 2204 50  0000 R CNN
F 1 "X" H 3591 2295 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3650 2250 50  0001 C CNN
F 3 "~" H 3650 2250 50  0001 C CNN
	1    3650 2250
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 61ACDEFF
P 3650 2100
F 0 "#PWR0103" H 3650 1950 50  0001 C CNN
F 1 "+5V" H 3665 2273 50  0000 C CNN
F 2 "" H 3650 2100 50  0001 C CNN
F 3 "" H 3650 2100 50  0001 C CNN
	1    3650 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2350 3350 2400
Wire Wire Line
	3250 2500 3650 2500
Wire Wire Line
	3350 2100 3350 2150
Wire Wire Line
	3650 2350 3650 2500
Wire Wire Line
	3650 2100 3650 2150
Wire Wire Line
	3650 2500 3800 2500
Connection ~ 3650 2500
Text GLabel 3800 2500 2    50   Output ~ 0
OUT1
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 61AD37E3
P 4300 3050
F 0 "J1" H 4380 3092 50  0000 L CNN
F 1 "Conn_01x03" H 4380 3001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4300 3050 50  0001 C CNN
F 3 "~" H 4300 3050 50  0001 C CNN
	1    4300 3050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 61AD3E97
P 4000 2950
F 0 "#PWR0104" H 4000 2800 50  0001 C CNN
F 1 "+5V" V 4015 3078 50  0000 L CNN
F 2 "" H 4000 2950 50  0001 C CNN
F 3 "" H 4000 2950 50  0001 C CNN
	1    4000 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 2950 4100 2950
$Comp
L power:GND #PWR0105
U 1 1 61AD4C5C
P 4000 3150
F 0 "#PWR0105" H 4000 2900 50  0001 C CNN
F 1 "GND" V 4005 3022 50  0000 R CNN
F 2 "" H 4000 3150 50  0001 C CNN
F 3 "" H 4000 3150 50  0001 C CNN
	1    4000 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 3150 4100 3150
Wire Wire Line
	4100 3050 4000 3050
Text GLabel 4000 3050 0    50   Input ~ 0
OUT1
$Comp
L TCST2300:TCST2103 U2
U 1 1 6164A83D
P 3450 4000
F 0 "U2" H 3108 4265 50  0000 C CNN
F 1 "TCST2103" H 3108 4174 50  0000 C CNN
F 2 "KSI_rotator_endstop:Vishay-TCST2300" H 3450 4400 50  0001 L CNN
F 3 "http://www.vishay.com/docs/81147/tcst2103.pdf" H 3108 4173 50  0001 C CNN
F 4 "IC" H 3450 4600 50  0001 L CNN "category"
F 5 "TCST2103-ND" H 3450 4700 50  0001 L CNN "digikey description"
F 6 "SENSR OPTO SLOT 3.1MM TRANS THRU" H 3450 4800 50  0001 L CNN "digikey part number"
F 7 "yes" H 3450 4900 50  0001 L CNN "lead free"
F 8 "f2a1018269320391" H 3450 5000 50  0001 L CNN "library id"
F 9 "Vishay" H 3450 5100 50  0001 L CNN "manufacturer"
F 10 "782-TCST2103" H 3450 5200 50  0001 L CNN "mouser part number"
F 11 "Slotted Module, 4-Lead Dual Row" H 3450 5300 50  0001 L CNN "package"
F 12 "yes" H 3450 5400 50  0001 L CNN "rohs"
F 13 "+85°C" H 3450 5500 50  0001 L CNN "temperature range high"
F 14 "-55°C" H 3450 5600 50  0001 L CNN "temperature range low"
	1    3450 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 6164A843
P 3450 3850
F 0 "R3" H 3391 3804 50  0000 R CNN
F 1 "100" H 3391 3895 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3450 3850 50  0001 C CNN
F 3 "~" H 3450 3850 50  0001 C CNN
	1    3450 3850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 6164A849
P 3450 4450
F 0 "#PWR0106" H 3450 4200 50  0001 C CNN
F 1 "GND" H 3455 4277 50  0000 C CNN
F 2 "" H 3450 4450 50  0001 C CNN
F 3 "" H 3450 4450 50  0001 C CNN
	1    3450 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 4300 3350 4300
Wire Wire Line
	3450 4000 3350 4000
Wire Wire Line
	3350 4400 3450 4400
Wire Wire Line
	3450 4400 3450 4450
Wire Wire Line
	3450 4300 3450 4400
Connection ~ 3450 4400
$Comp
L Device:R_Small R4
U 1 1 6164A85B
P 3750 3850
F 0 "R4" H 3691 3804 50  0000 R CNN
F 1 "X" H 3691 3895 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3750 3850 50  0001 C CNN
F 3 "~" H 3750 3850 50  0001 C CNN
	1    3750 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3450 3950 3450 4000
Wire Wire Line
	3350 4100 3750 4100
Wire Wire Line
	3450 3700 3450 3750
Wire Wire Line
	3750 3950 3750 4100
Wire Wire Line
	3750 3700 3750 3750
Wire Wire Line
	3750 4100 3900 4100
Connection ~ 3750 4100
Text GLabel 3900 4100 2    50   Output ~ 0
OUT2
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 6164A86F
P 4400 4650
F 0 "J2" H 4480 4692 50  0000 L CNN
F 1 "Conn_01x03" H 4480 4601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4400 4650 50  0001 C CNN
F 3 "~" H 4400 4650 50  0001 C CNN
	1    4400 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4550 4200 4550
$Comp
L power:GND #PWR0110
U 1 1 6164A87C
P 4100 4750
F 0 "#PWR0110" H 4100 4500 50  0001 C CNN
F 1 "GND" V 4105 4622 50  0000 R CNN
F 2 "" H 4100 4750 50  0001 C CNN
F 3 "" H 4100 4750 50  0001 C CNN
	1    4100 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 4750 4200 4750
Wire Wire Line
	4200 4650 4100 4650
Text GLabel 4100 4650 0    50   Input ~ 0
OUT2
$Comp
L TCST2300:TCST2103 U3
U 1 1 6164E26B
P 5650 2300
F 0 "U3" H 5308 2565 50  0000 C CNN
F 1 "TCST2103" H 5308 2474 50  0000 C CNN
F 2 "KSI_rotator_endstop:Vishay-TCST2300" H 5650 2700 50  0001 L CNN
F 3 "http://www.vishay.com/docs/81147/tcst2103.pdf" H 5308 2473 50  0001 C CNN
F 4 "IC" H 5650 2900 50  0001 L CNN "category"
F 5 "TCST2103-ND" H 5650 3000 50  0001 L CNN "digikey description"
F 6 "SENSR OPTO SLOT 3.1MM TRANS THRU" H 5650 3100 50  0001 L CNN "digikey part number"
F 7 "yes" H 5650 3200 50  0001 L CNN "lead free"
F 8 "f2a1018269320391" H 5650 3300 50  0001 L CNN "library id"
F 9 "Vishay" H 5650 3400 50  0001 L CNN "manufacturer"
F 10 "782-TCST2103" H 5650 3500 50  0001 L CNN "mouser part number"
F 11 "Slotted Module, 4-Lead Dual Row" H 5650 3600 50  0001 L CNN "package"
F 12 "yes" H 5650 3700 50  0001 L CNN "rohs"
F 13 "+85°C" H 5650 3800 50  0001 L CNN "temperature range high"
F 14 "-55°C" H 5650 3900 50  0001 L CNN "temperature range low"
	1    5650 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 6164E271
P 5650 2150
F 0 "R5" H 5591 2104 50  0000 R CNN
F 1 "100" H 5591 2195 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 5650 2150 50  0001 C CNN
F 3 "~" H 5650 2150 50  0001 C CNN
	1    5650 2150
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 6164E277
P 5650 2750
F 0 "#PWR0111" H 5650 2500 50  0001 C CNN
F 1 "GND" H 5655 2577 50  0000 C CNN
F 2 "" H 5650 2750 50  0001 C CNN
F 3 "" H 5650 2750 50  0001 C CNN
	1    5650 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 2600 5550 2600
Wire Wire Line
	5650 2300 5550 2300
Wire Wire Line
	5550 2700 5650 2700
Wire Wire Line
	5650 2700 5650 2750
Wire Wire Line
	5650 2600 5650 2700
Connection ~ 5650 2700
$Comp
L Device:R_Small R7
U 1 1 6164E289
P 5950 2150
F 0 "R7" H 5891 2104 50  0000 R CNN
F 1 "X" H 5891 2195 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 5950 2150 50  0001 C CNN
F 3 "~" H 5950 2150 50  0001 C CNN
	1    5950 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	5650 2250 5650 2300
Wire Wire Line
	5550 2400 5950 2400
Wire Wire Line
	5650 2000 5650 2050
Wire Wire Line
	5950 2250 5950 2400
Wire Wire Line
	5950 2000 5950 2050
Wire Wire Line
	5950 2400 6100 2400
Connection ~ 5950 2400
Text GLabel 6100 2400 2    50   Output ~ 0
OUT4
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 6164E29D
P 6600 2950
F 0 "J3" H 6680 2992 50  0000 L CNN
F 1 "Conn_01x03" H 6680 2901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6600 2950 50  0001 C CNN
F 3 "~" H 6600 2950 50  0001 C CNN
	1    6600 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 2850 6400 2850
$Comp
L power:GND #PWR0115
U 1 1 6164E2AA
P 6300 3050
F 0 "#PWR0115" H 6300 2800 50  0001 C CNN
F 1 "GND" V 6305 2922 50  0000 R CNN
F 2 "" H 6300 3050 50  0001 C CNN
F 3 "" H 6300 3050 50  0001 C CNN
	1    6300 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 3050 6400 3050
Wire Wire Line
	6400 2950 6300 2950
Text GLabel 6300 2950 0    50   Input ~ 0
OUT4
$Comp
L TCST2300:TCST2103 U4
U 1 1 616518EC
P 5700 4050
F 0 "U4" H 5358 4315 50  0000 C CNN
F 1 "TCST2103" H 5358 4224 50  0000 C CNN
F 2 "KSI_rotator_endstop:Vishay-TCST2300" H 5700 4450 50  0001 L CNN
F 3 "http://www.vishay.com/docs/81147/tcst2103.pdf" H 5358 4223 50  0001 C CNN
F 4 "IC" H 5700 4650 50  0001 L CNN "category"
F 5 "TCST2103-ND" H 5700 4750 50  0001 L CNN "digikey description"
F 6 "SENSR OPTO SLOT 3.1MM TRANS THRU" H 5700 4850 50  0001 L CNN "digikey part number"
F 7 "yes" H 5700 4950 50  0001 L CNN "lead free"
F 8 "f2a1018269320391" H 5700 5050 50  0001 L CNN "library id"
F 9 "Vishay" H 5700 5150 50  0001 L CNN "manufacturer"
F 10 "782-TCST2103" H 5700 5250 50  0001 L CNN "mouser part number"
F 11 "Slotted Module, 4-Lead Dual Row" H 5700 5350 50  0001 L CNN "package"
F 12 "yes" H 5700 5450 50  0001 L CNN "rohs"
F 13 "+85°C" H 5700 5550 50  0001 L CNN "temperature range high"
F 14 "-55°C" H 5700 5650 50  0001 L CNN "temperature range low"
	1    5700 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R6
U 1 1 616518F2
P 5700 3900
F 0 "R6" H 5641 3854 50  0000 R CNN
F 1 "100" H 5641 3945 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 5700 3900 50  0001 C CNN
F 3 "~" H 5700 3900 50  0001 C CNN
	1    5700 3900
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 616518F8
P 5700 4500
F 0 "#PWR0116" H 5700 4250 50  0001 C CNN
F 1 "GND" H 5705 4327 50  0000 C CNN
F 2 "" H 5700 4500 50  0001 C CNN
F 3 "" H 5700 4500 50  0001 C CNN
	1    5700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4350 5600 4350
Wire Wire Line
	5700 4050 5600 4050
Wire Wire Line
	5600 4450 5700 4450
Wire Wire Line
	5700 4450 5700 4500
Wire Wire Line
	5700 4350 5700 4450
Connection ~ 5700 4450
$Comp
L Device:R_Small R8
U 1 1 6165190A
P 6000 3900
F 0 "R8" H 5941 3854 50  0000 R CNN
F 1 "X" H 5941 3945 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 6000 3900 50  0001 C CNN
F 3 "~" H 6000 3900 50  0001 C CNN
	1    6000 3900
	-1   0    0    1   
$EndComp
Wire Wire Line
	5700 4000 5700 4050
Wire Wire Line
	5600 4150 6000 4150
Wire Wire Line
	5700 3750 5700 3800
Wire Wire Line
	6000 4000 6000 4150
Wire Wire Line
	6000 3750 6000 3800
Wire Wire Line
	6000 4150 6150 4150
Connection ~ 6000 4150
Text GLabel 6150 4150 2    50   Output ~ 0
OUT3
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 6165191E
P 6650 4700
F 0 "J4" H 6730 4742 50  0000 L CNN
F 1 "Conn_01x03" H 6730 4651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6650 4700 50  0001 C CNN
F 3 "~" H 6650 4700 50  0001 C CNN
	1    6650 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 4600 6450 4600
$Comp
L power:GND #PWR0120
U 1 1 6165192B
P 6350 4800
F 0 "#PWR0120" H 6350 4550 50  0001 C CNN
F 1 "GND" V 6355 4672 50  0000 R CNN
F 2 "" H 6350 4800 50  0001 C CNN
F 3 "" H 6350 4800 50  0001 C CNN
	1    6350 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	6350 4800 6450 4800
Wire Wire Line
	6450 4700 6350 4700
Text GLabel 6350 4700 0    50   Input ~ 0
OUT3
$Comp
L power:VCC #PWR0107
U 1 1 6167A1EB
P 5700 3750
F 0 "#PWR0107" H 5700 3600 50  0001 C CNN
F 1 "VCC" H 5715 3923 50  0000 C CNN
F 2 "" H 5700 3750 50  0001 C CNN
F 3 "" H 5700 3750 50  0001 C CNN
	1    5700 3750
	1    0    0    -1  
$EndComp
$Comp
L power:VCCQ #PWR0108
U 1 1 6167A961
P 5650 2000
F 0 "#PWR0108" H 5650 1850 50  0001 C CNN
F 1 "VCCQ" H 5665 2173 50  0000 C CNN
F 2 "" H 5650 2000 50  0001 C CNN
F 3 "" H 5650 2000 50  0001 C CNN
	1    5650 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0109
U 1 1 6167B23F
P 3450 3700
F 0 "#PWR0109" H 3450 3550 50  0001 C CNN
F 1 "+3V3" H 3465 3873 50  0000 C CNN
F 2 "" H 3450 3700 50  0001 C CNN
F 3 "" H 3450 3700 50  0001 C CNN
	1    3450 3700
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0112
U 1 1 6167C647
P 3750 3700
F 0 "#PWR0112" H 3750 3550 50  0001 C CNN
F 1 "+3V3" H 3765 3873 50  0000 C CNN
F 2 "" H 3750 3700 50  0001 C CNN
F 3 "" H 3750 3700 50  0001 C CNN
	1    3750 3700
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0113
U 1 1 6167CE5D
P 4100 4550
F 0 "#PWR0113" H 4100 4400 50  0001 C CNN
F 1 "+3V3" V 4115 4678 50  0000 L CNN
F 2 "" H 4100 4550 50  0001 C CNN
F 3 "" H 4100 4550 50  0001 C CNN
	1    4100 4550
	0    -1   -1   0   
$EndComp
$Comp
L power:VCCQ #PWR0114
U 1 1 6167EC5D
P 5950 2000
F 0 "#PWR0114" H 5950 1850 50  0001 C CNN
F 1 "VCCQ" H 5965 2173 50  0000 C CNN
F 2 "" H 5950 2000 50  0001 C CNN
F 3 "" H 5950 2000 50  0001 C CNN
	1    5950 2000
	1    0    0    -1  
$EndComp
$Comp
L power:VCCQ #PWR0117
U 1 1 6167F021
P 6300 2850
F 0 "#PWR0117" H 6300 2700 50  0001 C CNN
F 1 "VCCQ" V 6315 2977 50  0000 L CNN
F 2 "" H 6300 2850 50  0001 C CNN
F 3 "" H 6300 2850 50  0001 C CNN
	1    6300 2850
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR0118
U 1 1 616809D2
P 6000 3750
F 0 "#PWR0118" H 6000 3600 50  0001 C CNN
F 1 "VCC" H 6015 3923 50  0000 C CNN
F 2 "" H 6000 3750 50  0001 C CNN
F 3 "" H 6000 3750 50  0001 C CNN
	1    6000 3750
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0119
U 1 1 61680CFB
P 6350 4600
F 0 "#PWR0119" H 6350 4450 50  0001 C CNN
F 1 "VCC" V 6365 4727 50  0000 L CNN
F 2 "" H 6350 4600 50  0001 C CNN
F 3 "" H 6350 4600 50  0001 C CNN
	1    6350 4600
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
