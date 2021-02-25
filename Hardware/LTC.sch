EESchema Schematic File Version 4
LIBS:bmsLV-cache
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 4 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3600 9600 5450 9600
Wire Wire Line
	5450 9600 5450 3950
Wire Wire Line
	4100 9050 5400 9050
Wire Wire Line
	5400 3550 5400 9050
Wire Wire Line
	4250 8550 5350 8550
Wire Wire Line
	5350 8550 5350 3450
Connection ~ 3600 9600
Wire Wire Line
	3200 9600 3600 9600
Wire Wire Line
	3200 8550 3200 9600
$Comp
L power:GND #PWR035
U 1 1 601D8D8A
P 3050 8600
F 0 "#PWR035" H 3050 8350 50  0001 C CNN
F 1 "GND" H 3055 8427 50  0000 C CNN
F 2 "" H 3050 8600 50  0001 C CNN
F 3 "" H 3050 8600 50  0001 C CNN
	1    3050 8600
	1    0    0    -1  
$EndComp
Connection ~ 3050 8550
Wire Wire Line
	3050 8550 3050 8600
Connection ~ 3600 9500
Wire Wire Line
	3600 9500 3600 9600
Wire Wire Line
	4100 9050 4100 9400
Wire Wire Line
	4100 9050 4100 8850
Connection ~ 4100 9050
Wire Wire Line
	3050 8550 3200 8550
Wire Wire Line
	3600 8900 3600 8800
Wire Wire Line
	3950 9400 4100 9400
$Comp
L Device:R_Small R23
U 1 1 601C63CF
P 3850 9400
F 0 "R23" V 3654 9400 50  0000 C CNN
F 1 "1k" V 3745 9400 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 9400 50  0001 C CNN
F 3 "~" H 3850 9400 50  0001 C CNN
	1    3850 9400
	0    1    1    0   
$EndComp
Connection ~ 4100 9400
Wire Wire Line
	3300 8850 3300 9100
Wire Wire Line
	4100 8850 3300 8850
Wire Wire Line
	4100 9500 3950 9500
Wire Wire Line
	4100 9400 4100 9500
Wire Wire Line
	3600 9400 3750 9400
Wire Wire Line
	3600 9300 3600 9400
Connection ~ 3600 9300
$Comp
L Device:Q_PMOS_GSD Q8
U 1 1 601C63C1
P 3500 9100
F 0 "Q8" H 3705 9146 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 3705 9055 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 3700 9200 50  0001 C CNN
F 3 "~" H 3500 9100 50  0001 C CNN
	1    3500 9100
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener_Small D15
U 1 1 601C63BB
P 3850 9500
F 0 "D15" H 3850 9295 50  0000 C CNN
F 1 "9.1V" H 3850 9386 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 3850 9500 50  0001 C CNN
F 3 "~" V 3850 9500 50  0001 C CNN
	1    3850 9500
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 9500 3750 9500
Connection ~ 3600 9400
Wire Wire Line
	3600 9400 3600 9500
Wire Wire Line
	3600 9250 3600 9300
$Comp
L Device:R_Small R16
U 1 1 601C63B0
P 3600 8700
F 0 "R16" H 3659 8746 50  0000 L CNN
F 1 "30" H 3659 8655 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 3600 8700 50  0001 C CNN
F 3 "~" H 3600 8700 50  0001 C CNN
	1    3600 8700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR041
U 1 1 601C63AA
P 4250 8800
F 0 "#PWR041" H 4250 8550 50  0001 C CNN
F 1 "GND" H 4255 8627 50  0000 C CNN
F 2 "" H 4250 8800 50  0001 C CNN
F 3 "" H 4250 8800 50  0001 C CNN
	1    4250 8800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 8750 4250 8800
$Comp
L Device:R_Small R29
U 1 1 601C63A0
P 4050 8550
F 0 "R29" V 4246 8550 50  0000 C CNN
F 1 "100" V 4155 8550 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4050 8550 50  0001 C CNN
F 3 "~" H 4050 8550 50  0001 C CNN
	1    4050 8550
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C23
U 1 1 601C6399
P 4250 8650
F 0 "C23" H 4158 8604 50  0000 R CNN
F 1 "10nF" H 4158 8695 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4250 8650 50  0001 C CNN
F 3 "~" H 4250 8650 50  0001 C CNN
	1    4250 8650
	-1   0    0    1   
$EndComp
Wire Wire Line
	4100 8000 5300 8000
Wire Wire Line
	5300 8000 5300 3350
Wire Wire Line
	4250 7500 5250 7500
Wire Wire Line
	5250 3250 5250 7500
Wire Wire Line
	4100 6950 5200 6950
Wire Wire Line
	5200 3150 5200 6950
Wire Wire Line
	4250 6450 5150 6450
Wire Wire Line
	5150 3050 5150 6450
Wire Wire Line
	4100 5900 5100 5900
Wire Wire Line
	5100 2950 5100 5900
Wire Wire Line
	4250 5400 5050 5400
Wire Wire Line
	5050 5400 5050 2850
Wire Wire Line
	4100 4850 5000 4850
Wire Wire Line
	5000 4850 5000 2750
Wire Wire Line
	4250 4350 4950 4350
Wire Wire Line
	4950 4350 4950 2650
Wire Wire Line
	4100 3800 4900 3800
Wire Wire Line
	4900 3800 4900 2550
Wire Wire Line
	4850 3300 4850 2450
Wire Wire Line
	3050 8550 3050 8200
Wire Wire Line
	4100 8000 4100 8350
Wire Wire Line
	4100 8000 4100 7800
Connection ~ 4100 8000
Connection ~ 3200 7500
Wire Wire Line
	3200 8350 3200 7500
Wire Wire Line
	3050 7500 3200 7500
Wire Wire Line
	3600 7400 3600 7550
Connection ~ 3050 7500
Wire Wire Line
	3600 7850 3600 7750
Wire Wire Line
	3950 8350 4100 8350
$Comp
L Device:R_Small R22
U 1 1 6011B81C
P 3850 8350
F 0 "R22" V 3654 8350 50  0000 C CNN
F 1 "1k" V 3745 8350 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 8350 50  0001 C CNN
F 3 "~" H 3850 8350 50  0001 C CNN
	1    3850 8350
	0    1    1    0   
$EndComp
Connection ~ 4100 8350
Wire Wire Line
	3300 7800 3300 8050
Wire Wire Line
	4100 7800 3300 7800
Wire Wire Line
	4100 8450 3950 8450
Wire Wire Line
	4100 8350 4100 8450
Wire Wire Line
	3600 8350 3750 8350
Wire Wire Line
	3600 8250 3600 8350
Connection ~ 3600 8250
$Comp
L Device:Q_PMOS_GSD Q7
U 1 1 6011B80E
P 3500 8050
F 0 "Q7" H 3705 8096 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 3705 8005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 3700 8150 50  0001 C CNN
F 3 "~" H 3500 8050 50  0001 C CNN
	1    3500 8050
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener_Small D14
U 1 1 6011B808
P 3850 8450
F 0 "D14" H 3850 8245 50  0000 C CNN
F 1 "9.1V" H 3850 8336 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 3850 8450 50  0001 C CNN
F 3 "~" V 3850 8450 50  0001 C CNN
	1    3850 8450
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 8450 3750 8450
Connection ~ 3600 8350
Wire Wire Line
	3600 8350 3600 8450
Wire Wire Line
	3600 8350 3200 8350
Wire Wire Line
	3600 8200 3600 8250
$Comp
L power:GND #PWR040
U 1 1 6011B7F7
P 4250 7750
F 0 "#PWR040" H 4250 7500 50  0001 C CNN
F 1 "GND" H 4255 7577 50  0000 C CNN
F 2 "" H 4250 7750 50  0001 C CNN
F 3 "" H 4250 7750 50  0001 C CNN
	1    4250 7750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 7700 4250 7750
Wire Wire Line
	3050 7500 3050 7900
$Comp
L Device:R_Small R28
U 1 1 6011B7ED
P 4050 7500
F 0 "R28" V 4246 7500 50  0000 C CNN
F 1 "100" V 4155 7500 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4050 7500 50  0001 C CNN
F 3 "~" H 4050 7500 50  0001 C CNN
	1    4050 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C22
U 1 1 6011B7E6
P 4250 7600
F 0 "C22" H 4158 7554 50  0000 R CNN
F 1 "10nF" H 4158 7645 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4250 7600 50  0001 C CNN
F 3 "~" H 4250 7600 50  0001 C CNN
	1    4250 7600
	-1   0    0    1   
$EndComp
$Comp
L Device:Battery_Cell BT6
U 1 1 6011B7E0
P 3050 8100
F 0 "BT6" H 3168 8196 50  0000 L CNN
F 1 "Battery_Cell" H 3168 8105 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 3050 8160 50  0001 C CNN
F 3 "~" V 3050 8160 50  0001 C CNN
	1    3050 8100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 7500 3050 7150
Connection ~ 3600 7400
Wire Wire Line
	4100 6950 4100 7300
Wire Wire Line
	4100 6950 4100 6750
Connection ~ 4100 6950
Connection ~ 3200 6450
Wire Wire Line
	3200 7300 3200 6450
Wire Wire Line
	3050 6450 3200 6450
Wire Wire Line
	3600 6350 3600 6500
Connection ~ 3050 6450
Wire Wire Line
	3600 6800 3600 6700
Wire Wire Line
	3950 7300 4100 7300
$Comp
L Device:R_Small R21
U 1 1 60111131
P 3850 7300
F 0 "R21" V 3654 7300 50  0000 C CNN
F 1 "1k" V 3745 7300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 7300 50  0001 C CNN
F 3 "~" H 3850 7300 50  0001 C CNN
	1    3850 7300
	0    1    1    0   
$EndComp
Connection ~ 4100 7300
Wire Wire Line
	3300 6750 3300 7000
Wire Wire Line
	4100 6750 3300 6750
Wire Wire Line
	4100 7400 3950 7400
Wire Wire Line
	4100 7300 4100 7400
Wire Wire Line
	3600 7300 3750 7300
Wire Wire Line
	3600 7200 3600 7300
Connection ~ 3600 7200
$Comp
L Device:Q_PMOS_GSD Q6
U 1 1 60111123
P 3500 7000
F 0 "Q6" H 3705 7046 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 3705 6955 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 3700 7100 50  0001 C CNN
F 3 "~" H 3500 7000 50  0001 C CNN
	1    3500 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener_Small D13
U 1 1 6011111D
P 3850 7400
F 0 "D13" H 3850 7195 50  0000 C CNN
F 1 "9.1V" H 3850 7286 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 3850 7400 50  0001 C CNN
F 3 "~" V 3850 7400 50  0001 C CNN
	1    3850 7400
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 7400 3750 7400
Connection ~ 3600 7300
Wire Wire Line
	3600 7300 3600 7400
Wire Wire Line
	3600 7300 3200 7300
Wire Wire Line
	3600 7150 3600 7200
$Comp
L power:GND #PWR039
U 1 1 6011110C
P 4250 6700
F 0 "#PWR039" H 4250 6450 50  0001 C CNN
F 1 "GND" H 4255 6527 50  0000 C CNN
F 2 "" H 4250 6700 50  0001 C CNN
F 3 "" H 4250 6700 50  0001 C CNN
	1    4250 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 6650 4250 6700
Wire Wire Line
	3050 6450 3050 6850
$Comp
L Device:R_Small R27
U 1 1 60111102
P 4050 6450
F 0 "R27" V 4246 6450 50  0000 C CNN
F 1 "100" V 4155 6450 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4050 6450 50  0001 C CNN
F 3 "~" H 4050 6450 50  0001 C CNN
	1    4050 6450
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C21
U 1 1 601110FB
P 4250 6550
F 0 "C21" H 4158 6504 50  0000 R CNN
F 1 "10nF" H 4158 6595 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4250 6550 50  0001 C CNN
F 3 "~" H 4250 6550 50  0001 C CNN
	1    4250 6550
	-1   0    0    1   
$EndComp
$Comp
L Device:Battery_Cell BT5
U 1 1 601110F5
P 3050 7050
F 0 "BT5" H 3168 7146 50  0000 L CNN
F 1 "Battery_Cell" H 3168 7055 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 3050 7110 50  0001 C CNN
F 3 "~" V 3050 7110 50  0001 C CNN
	1    3050 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 6450 3050 6100
Connection ~ 3600 6350
Wire Wire Line
	4100 5900 4100 6250
Wire Wire Line
	4100 5900 4100 5700
Connection ~ 4100 5900
Connection ~ 3200 5400
Wire Wire Line
	3200 6250 3200 5400
Wire Wire Line
	3050 5400 3200 5400
Wire Wire Line
	3600 5300 3600 5450
Connection ~ 3050 5400
Wire Wire Line
	3600 5750 3600 5650
Wire Wire Line
	3950 6250 4100 6250
$Comp
L Device:R_Small R20
U 1 1 6010625F
P 3850 6250
F 0 "R20" V 3654 6250 50  0000 C CNN
F 1 "1k" V 3745 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 6250 50  0001 C CNN
F 3 "~" H 3850 6250 50  0001 C CNN
	1    3850 6250
	0    1    1    0   
$EndComp
Connection ~ 4100 6250
Wire Wire Line
	3300 5700 3300 5950
Wire Wire Line
	4100 5700 3300 5700
Wire Wire Line
	4100 6350 3950 6350
Wire Wire Line
	4100 6250 4100 6350
Wire Wire Line
	3600 6250 3750 6250
Wire Wire Line
	3600 6150 3600 6250
Connection ~ 3600 6150
$Comp
L Device:Q_PMOS_GSD Q5
U 1 1 60106251
P 3500 5950
F 0 "Q5" H 3705 5996 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 3705 5905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 3700 6050 50  0001 C CNN
F 3 "~" H 3500 5950 50  0001 C CNN
	1    3500 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener_Small D12
U 1 1 6010624B
P 3850 6350
F 0 "D12" H 3850 6145 50  0000 C CNN
F 1 "9.1V" H 3850 6236 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 3850 6350 50  0001 C CNN
F 3 "~" V 3850 6350 50  0001 C CNN
	1    3850 6350
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 6350 3750 6350
Connection ~ 3600 6250
Wire Wire Line
	3600 6250 3600 6350
Wire Wire Line
	3600 6250 3200 6250
Wire Wire Line
	3600 6100 3600 6150
$Comp
L power:GND #PWR038
U 1 1 6010623A
P 4250 5650
F 0 "#PWR038" H 4250 5400 50  0001 C CNN
F 1 "GND" H 4255 5477 50  0000 C CNN
F 2 "" H 4250 5650 50  0001 C CNN
F 3 "" H 4250 5650 50  0001 C CNN
	1    4250 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 5600 4250 5650
Wire Wire Line
	3050 5400 3050 5800
$Comp
L Device:R_Small R26
U 1 1 60106230
P 4050 5400
F 0 "R26" V 4246 5400 50  0000 C CNN
F 1 "100" V 4155 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4050 5400 50  0001 C CNN
F 3 "~" H 4050 5400 50  0001 C CNN
	1    4050 5400
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C20
U 1 1 60106229
P 4250 5500
F 0 "C20" H 4158 5454 50  0000 R CNN
F 1 "10nF" H 4158 5545 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4250 5500 50  0001 C CNN
F 3 "~" H 4250 5500 50  0001 C CNN
	1    4250 5500
	-1   0    0    1   
$EndComp
$Comp
L Device:Battery_Cell BT4
U 1 1 60106223
P 3050 6000
F 0 "BT4" H 3168 6096 50  0000 L CNN
F 1 "Battery_Cell" H 3168 6005 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 3050 6060 50  0001 C CNN
F 3 "~" V 3050 6060 50  0001 C CNN
	1    3050 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 5400 3050 5050
Connection ~ 3600 5300
Wire Wire Line
	4100 4850 4100 5200
Wire Wire Line
	4100 4850 4100 4650
Connection ~ 4100 4850
Connection ~ 3200 4350
Wire Wire Line
	3200 5200 3200 4350
Wire Wire Line
	3050 4350 3200 4350
Wire Wire Line
	3600 4250 3600 4400
Connection ~ 3050 4350
Wire Wire Line
	3600 4700 3600 4600
Wire Wire Line
	3950 5200 4100 5200
$Comp
L Device:R_Small R19
U 1 1 600FD851
P 3850 5200
F 0 "R19" V 3654 5200 50  0000 C CNN
F 1 "1k" V 3745 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 5200 50  0001 C CNN
F 3 "~" H 3850 5200 50  0001 C CNN
	1    3850 5200
	0    1    1    0   
$EndComp
Connection ~ 4100 5200
Wire Wire Line
	3300 4650 3300 4900
Wire Wire Line
	4100 4650 3300 4650
Wire Wire Line
	4100 5300 3950 5300
Wire Wire Line
	4100 5200 4100 5300
Wire Wire Line
	3600 5200 3750 5200
Wire Wire Line
	3600 5100 3600 5200
Connection ~ 3600 5100
$Comp
L Device:Q_PMOS_GSD Q4
U 1 1 600FD843
P 3500 4900
F 0 "Q4" H 3705 4946 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 3705 4855 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 3700 5000 50  0001 C CNN
F 3 "~" H 3500 4900 50  0001 C CNN
	1    3500 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener_Small D11
U 1 1 600FD83D
P 3850 5300
F 0 "D11" H 3850 5095 50  0000 C CNN
F 1 "9.1V" H 3850 5186 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 3850 5300 50  0001 C CNN
F 3 "~" V 3850 5300 50  0001 C CNN
	1    3850 5300
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 5300 3750 5300
Connection ~ 3600 5200
Wire Wire Line
	3600 5200 3600 5300
Wire Wire Line
	3600 5200 3200 5200
Wire Wire Line
	3600 5050 3600 5100
$Comp
L power:GND #PWR037
U 1 1 600FD82C
P 4250 4600
F 0 "#PWR037" H 4250 4350 50  0001 C CNN
F 1 "GND" H 4255 4427 50  0000 C CNN
F 2 "" H 4250 4600 50  0001 C CNN
F 3 "" H 4250 4600 50  0001 C CNN
	1    4250 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 4550 4250 4600
Wire Wire Line
	3050 4350 3050 4750
$Comp
L Device:R_Small R25
U 1 1 600FD822
P 4050 4350
F 0 "R25" V 4246 4350 50  0000 C CNN
F 1 "100" V 4155 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4050 4350 50  0001 C CNN
F 3 "~" H 4050 4350 50  0001 C CNN
	1    4050 4350
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C19
U 1 1 600FD81B
P 4250 4450
F 0 "C19" H 4158 4404 50  0000 R CNN
F 1 "10nF" H 4158 4495 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4250 4450 50  0001 C CNN
F 3 "~" H 4250 4450 50  0001 C CNN
	1    4250 4450
	-1   0    0    1   
$EndComp
$Comp
L Device:Battery_Cell BT3
U 1 1 600FD815
P 3050 4950
F 0 "BT3" H 3168 5046 50  0000 L CNN
F 1 "Battery_Cell" H 3168 4955 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 3050 5010 50  0001 C CNN
F 3 "~" V 3050 5010 50  0001 C CNN
	1    3050 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 4350 3050 4000
Connection ~ 3600 4250
Wire Wire Line
	4100 3800 4100 4150
Wire Wire Line
	4100 3800 4100 3600
Connection ~ 4100 3800
Wire Wire Line
	4100 2750 4100 3100
Wire Wire Line
	4100 2750 4100 2550
Connection ~ 4100 2750
Wire Wire Line
	4100 2750 4800 2750
Connection ~ 3200 3300
Wire Wire Line
	3200 4150 3200 3300
Wire Wire Line
	3050 3300 3200 3300
Connection ~ 3600 3200
Wire Wire Line
	3600 3200 3600 3350
Connection ~ 3050 3300
Wire Wire Line
	3050 2950 3050 3300
Wire Wire Line
	3600 3650 3600 3550
Wire Wire Line
	3950 4150 4100 4150
$Comp
L Device:R_Small R18
U 1 1 5FFC8479
P 3850 4150
F 0 "R18" V 3654 4150 50  0000 C CNN
F 1 "1k" V 3745 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 4150 50  0001 C CNN
F 3 "~" H 3850 4150 50  0001 C CNN
	1    3850 4150
	0    1    1    0   
$EndComp
Connection ~ 4100 4150
Wire Wire Line
	3300 3600 3300 3850
Wire Wire Line
	4100 3600 3300 3600
Wire Wire Line
	4100 4250 3950 4250
Wire Wire Line
	4100 4150 4100 4250
Wire Wire Line
	3600 4150 3750 4150
Wire Wire Line
	3600 4050 3600 4150
Connection ~ 3600 4050
$Comp
L Device:Q_PMOS_GSD Q3
U 1 1 5FFC8465
P 3500 3850
F 0 "Q3" H 3705 3896 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 3705 3805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 3700 3950 50  0001 C CNN
F 3 "~" H 3500 3850 50  0001 C CNN
	1    3500 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener_Small D10
U 1 1 5FFC845F
P 3850 4250
F 0 "D10" H 3850 4045 50  0000 C CNN
F 1 "9.1V" H 3850 4136 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 3850 4250 50  0001 C CNN
F 3 "~" V 3850 4250 50  0001 C CNN
	1    3850 4250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 4250 3750 4250
Connection ~ 3600 4150
Wire Wire Line
	3600 4150 3600 4250
Wire Wire Line
	3600 4150 3200 4150
Wire Wire Line
	3600 4000 3600 4050
$Comp
L power:GND #PWR036
U 1 1 5FFC24FD
P 4250 3550
F 0 "#PWR036" H 4250 3300 50  0001 C CNN
F 1 "GND" H 4255 3377 50  0000 C CNN
F 2 "" H 4250 3550 50  0001 C CNN
F 3 "" H 4250 3550 50  0001 C CNN
	1    4250 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3500 4250 3550
Wire Wire Line
	3050 3300 3050 3700
Wire Wire Line
	4250 3300 4850 3300
$Comp
L Device:R_Small R24
U 1 1 5FFBF3A5
P 4050 3300
F 0 "R24" V 4246 3300 50  0000 C CNN
F 1 "100" V 4155 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4050 3300 50  0001 C CNN
F 3 "~" H 4050 3300 50  0001 C CNN
	1    4050 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C18
U 1 1 5FFBF39B
P 4250 3400
F 0 "C18" H 4158 3354 50  0000 R CNN
F 1 "10nF" H 4158 3445 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4250 3400 50  0001 C CNN
F 3 "~" H 4250 3400 50  0001 C CNN
	1    4250 3400
	-1   0    0    1   
$EndComp
Wire Wire Line
	4450 2450 4450 2500
Wire Wire Line
	4100 3200 3950 3200
Wire Wire Line
	4100 3100 4100 3200
$Comp
L Device:D_Zener_Small D9
U 1 1 5FFB3CD8
P 3850 3200
F 0 "D9" H 3850 2995 50  0000 C CNN
F 1 "9.1V" H 3850 3086 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 3850 3200 50  0001 C CNN
F 3 "~" V 3850 3200 50  0001 C CNN
	1    3850 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 3200 3750 3200
Wire Wire Line
	3600 3100 3600 3200
Wire Wire Line
	3600 3000 3600 3100
$Comp
L Device:Battery_Cell BT2
U 1 1 5FFCD1E6
P 3050 3900
F 0 "BT2" H 3168 3996 50  0000 L CNN
F 1 "Battery_Cell" H 3168 3905 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 3050 3960 50  0001 C CNN
F 3 "~" V 3050 3960 50  0001 C CNN
	1    3050 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2600 3600 2500
Wire Wire Line
	3950 3100 4100 3100
$Comp
L Device:R_Small R17
U 1 1 5FFC350A
P 3850 3100
F 0 "R17" V 3654 3100 50  0000 C CNN
F 1 "1k" V 3745 3100 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 3100 50  0001 C CNN
F 3 "~" H 3850 3100 50  0001 C CNN
	1    3850 3100
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BT1
U 1 1 5FFBB0A9
P 3050 2850
F 0 "BT1" H 3168 2946 50  0000 L CNN
F 1 "Battery_Cell" H 3168 2855 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 3050 2910 50  0001 C CNN
F 3 "~" V 3050 2910 50  0001 C CNN
	1    3050 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 2250 3050 2650
Connection ~ 3200 2250
Wire Wire Line
	3200 2250 3050 2250
Connection ~ 3600 2250
Wire Wire Line
	3200 2250 3600 2250
Wire Wire Line
	3200 3100 3200 2250
Wire Wire Line
	3600 2250 3850 2250
Wire Wire Line
	3600 2300 3600 2250
Connection ~ 4100 3100
Wire Wire Line
	3300 2550 3300 2800
Wire Wire Line
	4100 2550 3300 2550
Wire Wire Line
	3600 3100 3750 3100
$Comp
L Device:Q_PMOS_GSD Q2
U 1 1 5FFB54E4
P 3500 2800
F 0 "Q2" H 3705 2846 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 3705 2755 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 3700 2900 50  0001 C CNN
F 3 "~" H 3500 2800 50  0001 C CNN
	1    3500 2800
	1    0    0    -1  
$EndComp
Connection ~ 3600 3100
Wire Wire Line
	3600 3100 3200 3100
Wire Wire Line
	4800 2350 4800 2750
$Comp
L Device:R_Small R30
U 1 1 5FF96984
P 4150 2250
F 0 "R30" V 4346 2250 50  0000 C CNN
F 1 "100" V 4255 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4150 2250 50  0001 C CNN
F 3 "~" H 4150 2250 50  0001 C CNN
	1    4150 2250
	0    -1   -1   0   
$EndComp
Connection ~ 4450 2250
Wire Wire Line
	4250 2250 4450 2250
$Comp
L power:GND #PWR042
U 1 1 5FF957ED
P 4450 2500
F 0 "#PWR042" H 4450 2250 50  0001 C CNN
F 1 "GND" H 4455 2327 50  0000 C CNN
F 2 "" H 4450 2500 50  0001 C CNN
F 3 "" H 4450 2500 50  0001 C CNN
	1    4450 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C24
U 1 1 5FF93E07
P 4450 2350
F 0 "C24" H 4358 2304 50  0000 R CNN
F 1 "10nF" H 4358 2395 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4450 2350 50  0001 C CNN
F 3 "~" H 4450 2350 50  0001 C CNN
	1    4450 2350
	-1   0    0    1   
$EndComp
Wire Wire Line
	7050 2250 7250 2250
Wire Wire Line
	7050 2350 7250 2350
Wire Wire Line
	7050 2650 7250 2650
Wire Wire Line
	7050 2750 7250 2750
Text GLabel 7250 2650 2    50   Input ~ 0
SCK
Text GLabel 7250 2750 2    50   Input ~ 0
CS
Wire Wire Line
	5850 3550 5400 3550
Wire Wire Line
	5350 3450 5850 3450
Wire Wire Line
	5300 3350 5850 3350
Wire Wire Line
	5250 3250 5850 3250
Wire Wire Line
	5200 3150 5850 3150
Wire Wire Line
	5150 3050 5850 3050
Wire Wire Line
	5100 2950 5850 2950
Wire Wire Line
	5050 2850 5850 2850
Wire Wire Line
	5000 2750 5850 2750
Wire Wire Line
	4950 2650 5850 2650
Wire Wire Line
	4900 2550 5850 2550
Wire Wire Line
	4850 2450 5850 2450
Wire Wire Line
	4800 2350 5850 2350
Wire Wire Line
	4450 2250 5850 2250
Wire Wire Line
	5450 3950 5750 3950
Wire Wire Line
	5750 3950 5750 4050
Wire Wire Line
	5750 4050 5850 4050
Connection ~ 5750 3950
Wire Wire Line
	5750 3950 5850 3950
Wire Wire Line
	7050 3950 7150 3950
Wire Wire Line
	7150 3950 7150 4050
Wire Wire Line
	7150 4300 6450 4300
Wire Wire Line
	5750 4300 5750 4050
Connection ~ 5750 4050
Wire Wire Line
	7050 4050 7150 4050
Connection ~ 7150 4050
Wire Wire Line
	7150 4050 7150 4300
Wire Wire Line
	6450 4300 6450 4350
Connection ~ 6450 4300
Wire Wire Line
	6450 4300 5750 4300
$Comp
L power:GND #PWR045
U 1 1 60166D55
P 6450 4350
F 0 "#PWR045" H 6450 4100 50  0001 C CNN
F 1 "GND" H 6455 4177 50  0000 C CNN
F 2 "" H 6450 4350 50  0001 C CNN
F 3 "" H 6450 4350 50  0001 C CNN
	1    6450 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3450 7150 3450
Wire Wire Line
	7050 3250 7500 3250
Wire Wire Line
	7500 3250 7500 3300
$Comp
L Device:C_Small C27
U 1 1 6019F5F8
P 7500 3400
F 0 "C27" H 7592 3446 50  0000 L CNN
F 1 "1u" H 7592 3355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7500 3400 50  0001 C CNN
F 3 "~" H 7500 3400 50  0001 C CNN
	1    7500 3400
	1    0    0    -1  
$EndComp
Connection ~ 7500 3650
Wire Wire Line
	7500 3650 7750 3650
Wire Wire Line
	7050 3350 7300 3350
Wire Wire Line
	7150 3450 7150 3650
$Comp
L Device:C_Small C26
U 1 1 601E809B
P 7300 3500
F 0 "C26" H 7392 3546 50  0000 L CNN
F 1 "1u" H 7392 3455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7300 3500 50  0001 C CNN
F 3 "~" H 7300 3500 50  0001 C CNN
	1    7300 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3650 7300 3650
Wire Wire Line
	7300 3350 7300 3400
Wire Wire Line
	7300 3600 7300 3650
Connection ~ 7300 3650
Wire Wire Line
	7300 3650 7500 3650
Wire Wire Line
	7500 3500 7500 3650
Wire Wire Line
	7750 3650 7750 3750
$Comp
L power:GND #PWR050
U 1 1 6022F4BC
P 7750 3750
F 0 "#PWR050" H 7750 3500 50  0001 C CNN
F 1 "GND" H 7755 3577 50  0000 C CNN
F 2 "" H 7750 3750 50  0001 C CNN
F 3 "" H 7750 3750 50  0001 C CNN
	1    7750 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3650 7750 3500
$Comp
L Device:C_Small C28
U 1 1 6023E9B3
P 7750 3400
F 0 "C28" H 7842 3446 50  0000 L CNN
F 1 "1u" H 7842 3355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7750 3400 50  0001 C CNN
F 3 "~" H 7750 3400 50  0001 C CNN
	1    7750 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3150 7750 3150
Wire Wire Line
	7750 3150 7750 3300
Connection ~ 7750 3150
Connection ~ 7750 3650
Wire Wire Line
	7050 3050 7450 3050
$Comp
L Device:Q_NPN_BCE Q9
U 1 1 602887F0
P 7650 2900
F 0 "Q9" H 7841 2946 50  0000 L CNN
F 1 "Q_NPN_BCE" H 7841 2855 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 7850 3000 50  0001 C CNN
F 3 "~" H 7650 2900 50  0001 C CNN
	1    7650 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3100 7750 3150
Wire Wire Line
	7450 3050 7450 2900
$Comp
L Device:C_Small C29
U 1 1 602D5887
P 8100 2600
F 0 "C29" V 8329 2600 50  0000 C CNN
F 1 "0.1u" V 8238 2600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8100 2600 50  0001 C CNN
F 3 "~" H 8100 2600 50  0001 C CNN
	1    8100 2600
	0    -1   -1   0   
$EndComp
Connection ~ 7750 2600
Wire Wire Line
	7750 2600 7750 2700
$Comp
L power:GND #PWR051
U 1 1 60321616
P 8350 2600
F 0 "#PWR051" H 8350 2350 50  0001 C CNN
F 1 "GND" V 8355 2472 50  0000 R CNN
F 2 "" H 8350 2600 50  0001 C CNN
F 3 "" H 8350 2600 50  0001 C CNN
	1    8350 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 2300 7750 2600
$Comp
L Device:R_Small R32
U 1 1 60331F67
P 7750 2200
F 0 "R32" H 7809 2246 50  0000 L CNN
F 1 "100" H 7809 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 7750 2200 50  0001 C CNN
F 3 "~" H 7750 2200 50  0001 C CNN
	1    7750 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R15
U 1 1 6035282B
P 3600 7650
F 0 "R15" H 3659 7696 50  0000 L CNN
F 1 "30" H 3659 7605 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 3600 7650 50  0001 C CNN
F 3 "~" H 3600 7650 50  0001 C CNN
	1    3600 7650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R14
U 1 1 60370C7C
P 3600 6600
F 0 "R14" H 3659 6646 50  0000 L CNN
F 1 "30" H 3659 6555 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 3600 6600 50  0001 C CNN
F 3 "~" H 3600 6600 50  0001 C CNN
	1    3600 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R13
U 1 1 6038EE9D
P 3600 5550
F 0 "R13" H 3659 5596 50  0000 L CNN
F 1 "30" H 3659 5505 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 3600 5550 50  0001 C CNN
F 3 "~" H 3600 5550 50  0001 C CNN
	1    3600 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R12
U 1 1 603AD261
P 3600 4500
F 0 "R12" H 3659 4546 50  0000 L CNN
F 1 "30" H 3659 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 3600 4500 50  0001 C CNN
F 3 "~" H 3600 4500 50  0001 C CNN
	1    3600 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R11
U 1 1 603CB5B0
P 3600 3450
F 0 "R11" H 3659 3496 50  0000 L CNN
F 1 "30" H 3659 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 3600 3450 50  0001 C CNN
F 3 "~" H 3600 3450 50  0001 C CNN
	1    3600 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R10
U 1 1 603E990D
P 3600 2400
F 0 "R10" H 3659 2446 50  0000 L CNN
F 1 "30" H 3659 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" H 3600 2400 50  0001 C CNN
F 3 "~" H 3600 2400 50  0001 C CNN
	1    3600 2400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J6
U 1 1 6043A25B
P 2850 2250
F 0 "J6" H 2878 2276 50  0000 L CNN
F 1 "Conn_01x01_Female" H 2878 2185 50  0000 L CNN
F 2 "CellPad:CellPad" H 2850 2250 50  0001 C CNN
F 3 "~" H 2850 2250 50  0001 C CNN
	1    2850 2250
	-1   0    0    1   
$EndComp
Connection ~ 3050 2250
$Comp
L Connector:Conn_01x01_Female J7
U 1 1 6044A31A
P 2850 3300
F 0 "J7" H 2742 3075 50  0000 C CNN
F 1 "Conn_01x01_Female" H 2742 3166 50  0000 C CNN
F 2 "CellPad:CellPad" H 2850 3300 50  0001 C CNN
F 3 "~" H 2850 3300 50  0001 C CNN
	1    2850 3300
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J8
U 1 1 6044B5BA
P 2850 4350
F 0 "J8" H 2742 4125 50  0000 C CNN
F 1 "Conn_01x01_Female" H 2742 4216 50  0000 C CNN
F 2 "CellPad:CellPad" H 2850 4350 50  0001 C CNN
F 3 "~" H 2850 4350 50  0001 C CNN
	1    2850 4350
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J9
U 1 1 6044C075
P 2850 5400
F 0 "J9" H 2742 5175 50  0000 C CNN
F 1 "Conn_01x01_Female" H 2742 5266 50  0000 C CNN
F 2 "CellPad:CellPad" H 2850 5400 50  0001 C CNN
F 3 "~" H 2850 5400 50  0001 C CNN
	1    2850 5400
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J10
U 1 1 6044D224
P 2850 6450
F 0 "J10" H 2742 6225 50  0000 C CNN
F 1 "Conn_01x01_Female" H 2742 6316 50  0000 C CNN
F 2 "CellPad:CellPad" H 2850 6450 50  0001 C CNN
F 3 "~" H 2850 6450 50  0001 C CNN
	1    2850 6450
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J11
U 1 1 6044DF34
P 2850 7500
F 0 "J11" H 2742 7275 50  0000 C CNN
F 1 "Conn_01x01_Female" H 2742 7366 50  0000 C CNN
F 2 "CellPad:CellPad" H 2850 7500 50  0001 C CNN
F 3 "~" H 2850 7500 50  0001 C CNN
	1    2850 7500
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J12
U 1 1 6044ED17
P 2850 8550
F 0 "J12" H 2742 8325 50  0000 C CNN
F 1 "Conn_01x01_Female" H 2742 8416 50  0000 C CNN
F 2 "CellPad:CellPad" H 2850 8550 50  0001 C CNN
F 3 "~" H 2850 8550 50  0001 C CNN
	1    2850 8550
	-1   0    0    1   
$EndComp
Text GLabel 7250 2250 2    50   Input ~ 0
MISO
Text GLabel 7250 2350 2    50   Input ~ 0
MOSI
Wire Wire Line
	5850 1950 5800 1950
Wire Wire Line
	5800 1950 5800 1650
Wire Wire Line
	5800 1650 6450 1650
Wire Wire Line
	7100 1650 7100 1950
Wire Wire Line
	7100 2050 7050 2050
Wire Wire Line
	7100 1950 7050 1950
Connection ~ 7100 1950
Wire Wire Line
	7100 1950 7100 2050
Connection ~ 6450 1650
Wire Wire Line
	6450 1650 7100 1650
$Comp
L power:GND #PWR044
U 1 1 604A2D56
P 6550 1500
F 0 "#PWR044" H 6550 1250 50  0001 C CNN
F 1 "GND" H 6555 1327 50  0000 C CNN
F 2 "" H 6550 1500 50  0001 C CNN
F 3 "" H 6550 1500 50  0001 C CNN
	1    6550 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2250 3850 1700
Wire Wire Line
	3850 1350 7750 1350
Connection ~ 3850 2250
Wire Wire Line
	3850 2250 4050 2250
Wire Wire Line
	7750 1350 7750 2100
Wire Wire Line
	3850 1700 4250 1700
Connection ~ 3850 1700
Wire Wire Line
	3850 1700 3850 1350
$Comp
L Device:R_Small R31
U 1 1 604D5C59
P 4350 1700
F 0 "R31" V 4154 1700 50  0000 C CNN
F 1 "100" V 4245 1700 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4350 1700 50  0001 C CNN
F 3 "~" H 4350 1700 50  0001 C CNN
	1    4350 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 1700 5650 2150
Wire Wire Line
	4450 1700 4750 1700
Wire Wire Line
	5850 2150 5650 2150
Wire Wire Line
	4750 1700 4750 1750
Connection ~ 4750 1700
Wire Wire Line
	4750 1700 5650 1700
$Comp
L Device:C_Small C25
U 1 1 6051CA4B
P 4750 1850
F 0 "C25" H 4842 1896 50  0000 L CNN
F 1 "1u" H 4842 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4750 1850 50  0001 C CNN
F 3 "~" H 4750 1850 50  0001 C CNN
	1    4750 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1950 4750 2000
$Comp
L power:GND #PWR043
U 1 1 6052F649
P 4750 2000
F 0 "#PWR043" H 4750 1750 50  0001 C CNN
F 1 "GND" H 4755 1827 50  0000 C CNN
F 2 "" H 4750 2000 50  0001 C CNN
F 3 "" H 4750 2000 50  0001 C CNN
	1    4750 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR046
U 1 1 605E56B3
P 7050 2450
F 0 "#PWR046" H 7050 2200 50  0001 C CNN
F 1 "GND" V 7055 2322 50  0000 R CNN
F 2 "" H 7050 2450 50  0001 C CNN
F 3 "" H 7050 2450 50  0001 C CNN
	1    7050 2450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7750 2600 8000 2600
Wire Wire Line
	8200 2600 8350 2600
$Comp
L LTC6810-1:LTC6810-1-LTC6810-1 LTC?
U 1 1 5FF8D74A
P 7350 4100
AR Path="/5FF8D74A" Ref="LTC?"  Part="1" 
AR Path="/5FF86554/5FF8D74A" Ref="LTC1"  Part="1" 
F 0 "LTC1" H 6450 6475 50  0000 C CNN
F 1 "LTC6810-1" H 6450 6384 50  0000 C CNN
F 2 "Package_SO:SSOP-44_5.3x12.8mm_P0.5mm" H 6200 5150 50  0001 C CNN
F 3 "" H 6200 5150 50  0001 C CNN
	1    7350 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 605F8879
P 7050 2550
F 0 "#PWR047" H 7050 2300 50  0001 C CNN
F 1 "GND" V 7055 2422 50  0000 R CNN
F 2 "" H 7050 2550 50  0001 C CNN
F 3 "" H 7050 2550 50  0001 C CNN
	1    7050 2550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR048
U 1 1 605F8ED2
P 7050 2850
F 0 "#PWR048" H 7050 2600 50  0001 C CNN
F 1 "GND" V 7055 2722 50  0000 R CNN
F 2 "" H 7050 2850 50  0001 C CNN
F 3 "" H 7050 2850 50  0001 C CNN
	1    7050 2850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR049
U 1 1 605F9ECA
P 7050 2950
F 0 "#PWR049" H 7050 2700 50  0001 C CNN
F 1 "GND" V 7055 2822 50  0000 R CNN
F 2 "" H 7050 2950 50  0001 C CNN
F 3 "" H 7050 2950 50  0001 C CNN
	1    7050 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6550 1500 6450 1500
Wire Wire Line
	6450 1500 6450 1650
Connection ~ 3200 8550
Wire Wire Line
	3600 8600 3600 8450
Connection ~ 3600 8450
$Comp
L Device:R_Small R42
U 1 1 6044A16F
P 3850 3350
F 0 "R42" V 3654 3350 50  0000 C CNN
F 1 "330" V 3745 3350 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 3350 50  0001 C CNN
F 3 "~" H 3850 3350 50  0001 C CNN
	1    3850 3350
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D18
U 1 1 6044B42B
P 3850 3550
F 0 "D18" H 3850 3785 50  0000 C CNN
F 1 "LED_Small" H 3850 3694 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 3850 3550 50  0001 C CNN
F 3 "~" V 3850 3550 50  0001 C CNN
	1    3850 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 3300 3950 3300
Wire Wire Line
	4150 3300 4250 3300
Connection ~ 4250 3300
Wire Wire Line
	3600 3350 3750 3350
Connection ~ 3600 3350
Wire Wire Line
	3600 3550 3750 3550
Connection ~ 3600 3550
Wire Wire Line
	3950 3350 3950 3550
Wire Wire Line
	3200 4350 3950 4350
Wire Wire Line
	4150 4350 4250 4350
Connection ~ 4250 4350
Wire Wire Line
	3200 5400 3950 5400
Wire Wire Line
	4150 5400 4250 5400
Connection ~ 4250 5400
Wire Wire Line
	3200 6450 3950 6450
Wire Wire Line
	4150 6450 4250 6450
Connection ~ 4250 6450
Wire Wire Line
	3200 7500 3950 7500
Wire Wire Line
	4150 7500 4250 7500
Connection ~ 4250 7500
Wire Wire Line
	3200 8550 3950 8550
Wire Wire Line
	4150 8550 4250 8550
Connection ~ 4250 8550
$Comp
L Device:R_Small R41
U 1 1 607A510A
P 3850 2300
F 0 "R41" V 3654 2300 50  0000 C CNN
F 1 "330" V 3745 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 2300 50  0001 C CNN
F 3 "~" H 3850 2300 50  0001 C CNN
	1    3850 2300
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D17
U 1 1 607A5110
P 3850 2500
F 0 "D17" H 3850 2735 50  0000 C CNN
F 1 "LED_Small" H 3850 2644 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 3850 2500 50  0001 C CNN
F 3 "~" V 3850 2500 50  0001 C CNN
	1    3850 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2300 3750 2300
Wire Wire Line
	3600 2500 3750 2500
Wire Wire Line
	3950 2300 3950 2500
$Comp
L Device:R_Small R43
U 1 1 607B74E7
P 3850 4400
F 0 "R43" V 3654 4400 50  0000 C CNN
F 1 "330" V 3745 4400 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 4400 50  0001 C CNN
F 3 "~" H 3850 4400 50  0001 C CNN
	1    3850 4400
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D19
U 1 1 607B74ED
P 3850 4600
F 0 "D19" H 3850 4835 50  0000 C CNN
F 1 "LED_Small" H 3850 4744 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 3850 4600 50  0001 C CNN
F 3 "~" V 3850 4600 50  0001 C CNN
	1    3850 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4400 3750 4400
Wire Wire Line
	3600 4600 3750 4600
Wire Wire Line
	3950 4400 3950 4600
$Comp
L Device:LED_Small D20
U 1 1 607CA194
P 3850 5650
F 0 "D20" H 3850 5885 50  0000 C CNN
F 1 "LED_Small" H 3850 5794 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 3850 5650 50  0001 C CNN
F 3 "~" V 3850 5650 50  0001 C CNN
	1    3850 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5450 3750 5450
Wire Wire Line
	3600 5650 3750 5650
Wire Wire Line
	3950 5450 3950 5650
$Comp
L Device:LED_Small D21
U 1 1 607DD4B1
P 3850 6700
F 0 "D21" H 3850 6935 50  0000 C CNN
F 1 "LED_Small" H 3850 6844 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 3850 6700 50  0001 C CNN
F 3 "~" V 3850 6700 50  0001 C CNN
	1    3850 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6500 3750 6500
Wire Wire Line
	3600 6700 3750 6700
Wire Wire Line
	3950 6500 3950 6700
$Comp
L Device:R_Small R46
U 1 1 607F0E00
P 3850 7550
F 0 "R46" V 3654 7550 50  0000 C CNN
F 1 "330" V 3745 7550 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 7550 50  0001 C CNN
F 3 "~" H 3850 7550 50  0001 C CNN
	1    3850 7550
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D22
U 1 1 607F0E06
P 3850 7750
F 0 "D22" H 3850 7985 50  0000 C CNN
F 1 "LED_Small" H 3850 7894 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 3850 7750 50  0001 C CNN
F 3 "~" V 3850 7750 50  0001 C CNN
	1    3850 7750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 7550 3750 7550
Wire Wire Line
	3600 7750 3750 7750
Wire Wire Line
	3950 7550 3950 7750
$Comp
L Device:R_Small R47
U 1 1 60804E2A
P 3850 8600
F 0 "R47" V 3654 8600 50  0000 C CNN
F 1 "330" V 3745 8600 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 8600 50  0001 C CNN
F 3 "~" H 3850 8600 50  0001 C CNN
	1    3850 8600
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D23
U 1 1 60804E30
P 3850 8800
F 0 "D23" H 3850 9035 50  0000 C CNN
F 1 "LED_Small" H 3850 8944 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 3850 8800 50  0001 C CNN
F 3 "~" V 3850 8800 50  0001 C CNN
	1    3850 8800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 8600 3750 8600
Wire Wire Line
	3600 8800 3750 8800
Wire Wire Line
	3950 8600 3950 8800
$Comp
L Device:R_Small R45
U 1 1 607DD4AB
P 3850 6500
F 0 "R45" V 3654 6500 50  0000 C CNN
F 1 "330" V 3745 6500 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 6500 50  0001 C CNN
F 3 "~" H 3850 6500 50  0001 C CNN
	1    3850 6500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R44
U 1 1 607CA18E
P 3850 5450
F 0 "R44" V 3654 5450 50  0000 C CNN
F 1 "330" V 3745 5450 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3850 5450 50  0001 C CNN
F 3 "~" H 3850 5450 50  0001 C CNN
	1    3850 5450
	0    1    1    0   
$EndComp
$EndSCHEMATC
