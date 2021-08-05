EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
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
L bmsLV-rescue:MCP2562-H-SN-Interface_CAN_LIN U4
U 1 1 5FB847F4
P 3650 2500
F 0 "U4" H 3650 3081 50  0000 C CNN
F 1 "MCP2562-H-SN" H 3650 2990 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3650 2000 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/25167A.pdf" H 3650 2500 50  0001 C CNN
	1    3650 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2300 2850 2300
Wire Wire Line
	3150 2400 2850 2400
Text GLabel 2850 2300 0    50   Input ~ 0
TX_Can
Text GLabel 2850 2400 0    50   Input ~ 0
RX_Can
Wire Wire Line
	3650 2100 3650 1800
$Comp
L bmsLV-rescue:+5V-power #PWR032
U 1 1 5FB87973
P 3650 1650
F 0 "#PWR032" H 3650 1500 50  0001 C CNN
F 1 "+5V" H 3665 1823 50  0000 C CNN
F 2 "" H 3650 1650 50  0001 C CNN
F 3 "" H 3650 1650 50  0001 C CNN
	1    3650 1650
	1    0    0    -1  
$EndComp
$Comp
L bmsLV-rescue:C_Small-Device C17
U 1 1 5FB886F3
P 3550 1800
F 0 "C17" V 3321 1800 50  0000 C CNN
F 1 "100n" V 3412 1800 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3550 1800 50  0001 C CNN
F 3 "~" H 3550 1800 50  0001 C CNN
	1    3550 1800
	0    1    1    0   
$EndComp
Connection ~ 3650 1800
Wire Wire Line
	3650 1800 3650 1650
$Comp
L bmsLV-rescue:GND-power #PWR031
U 1 1 5FB8975D
P 3350 1800
F 0 "#PWR031" H 3350 1550 50  0001 C CNN
F 1 "GND" V 3355 1672 50  0000 R CNN
F 2 "" H 3350 1800 50  0001 C CNN
F 3 "" H 3350 1800 50  0001 C CNN
	1    3350 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1800 3450 1800
Wire Wire Line
	3150 2600 2900 2600
$Comp
L bmsLV-rescue:+3.3V-power #PWR028
U 1 1 5FB8A58B
P 2300 2600
F 0 "#PWR028" H 2300 2450 50  0001 C CNN
F 1 "+3.3V" V 2315 2728 50  0000 L CNN
F 2 "" H 2300 2600 50  0001 C CNN
F 3 "" H 2300 2600 50  0001 C CNN
	1    2300 2600
	1    0    0    -1  
$EndComp
$Comp
L bmsLV-rescue:C_Small-Device C16
U 1 1 5FB8B717
P 2900 2700
F 0 "C16" V 2671 2700 50  0000 C CNN
F 1 "100n" V 2762 2700 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2900 2700 50  0001 C CNN
F 3 "~" H 2900 2700 50  0001 C CNN
	1    2900 2700
	1    0    0    -1  
$EndComp
Connection ~ 2900 2600
$Comp
L bmsLV-rescue:GND-power #PWR029
U 1 1 5FB8CC3F
P 2900 2900
F 0 "#PWR029" H 2900 2650 50  0001 C CNN
F 1 "GND" H 2905 2727 50  0000 C CNN
F 2 "" H 2900 2900 50  0001 C CNN
F 3 "" H 2900 2900 50  0001 C CNN
	1    2900 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 2800 2900 2900
Wire Wire Line
	3150 2700 3150 3000
$Comp
L bmsLV-rescue:GND-power #PWR030
U 1 1 5FB8DCB8
P 3150 3000
F 0 "#PWR030" H 3150 2750 50  0001 C CNN
F 1 "GND" H 3155 2827 50  0000 C CNN
F 2 "" H 3150 3000 50  0001 C CNN
F 3 "" H 3150 3000 50  0001 C CNN
	1    3150 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2600 2900 2600
Wire Wire Line
	3650 2900 3650 3000
$Comp
L bmsLV-rescue:GND-power #PWR033
U 1 1 5FB9013B
P 3650 3000
F 0 "#PWR033" H 3650 2750 50  0001 C CNN
F 1 "GND" H 3655 2827 50  0000 C CNN
F 2 "" H 3650 3000 50  0001 C CNN
F 3 "" H 3650 3000 50  0001 C CNN
	1    3650 3000
	1    0    0    -1  
$EndComp
$Comp
L bmsLV-rescue:R_Small-Device R9
U 1 1 5FB91F93
P 4400 2400
F 0 "R9" H 4459 2446 50  0000 L CNN
F 1 "120" H 4459 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4400 2400 50  0001 C CNN
F 3 "~" H 4400 2400 50  0001 C CNN
	1    4400 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 2250 4400 2300
$Comp
L bmsLV-rescue:D_TVS_x2_AAC-Device D8
U 1 1 5FB95EFA
P 4800 2500
F 0 "D8" V 4846 2579 50  0000 L CNN
F 1 "D_TVS_x2_AAC" V 4755 2579 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4650 2500 50  0001 C CNN
F 3 "~" H 4650 2500 50  0001 C CNN
	1    4800 2500
	0    -1   -1   0   
$EndComp
$Comp
L bmsLV-rescue:GND-power #PWR034
U 1 1 5FB9744E
P 5050 2500
F 0 "#PWR034" H 5050 2250 50  0001 C CNN
F 1 "GND" V 5055 2372 50  0000 R CNN
F 2 "" H 5050 2500 50  0001 C CNN
F 3 "" H 5050 2500 50  0001 C CNN
	1    5050 2500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4950 2500 5050 2500
Wire Wire Line
	4800 2150 4800 2100
Wire Wire Line
	4150 2100 4800 2100
Wire Wire Line
	4800 2850 4800 2900
Wire Wire Line
	4150 2900 4800 2900
Wire Wire Line
	4800 2100 5450 2100
Connection ~ 4800 2100
Wire Wire Line
	4800 2900 5450 2900
Connection ~ 4800 2900
Text GLabel 5450 2100 2    50   Input ~ 0
CANH
Text GLabel 5450 2900 2    50   Input ~ 0
CANL
$Comp
L bmsLV-rescue:Conn_01x02_Male-Connector J4
U 1 1 606A8AD6
P 4600 2700
F 0 "J4" H 4572 2582 50  0000 R CNN
F 1 "Conn_01x02_Male" H 4572 2673 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 4600 2700 50  0001 C CNN
F 3 "~" H 4600 2700 50  0001 C CNN
	1    4600 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 2100 4150 2250
Wire Wire Line
	4150 2600 4150 2750
Connection ~ 4150 2750
Wire Wire Line
	4150 2750 4150 2900
Wire Wire Line
	4400 2750 4400 2700
Wire Wire Line
	4150 2750 4400 2750
Wire Wire Line
	4400 2600 4400 2500
Wire Wire Line
	4400 2250 4150 2250
Connection ~ 4150 2250
Wire Wire Line
	4150 2250 4150 2400
$EndSCHEMATC
