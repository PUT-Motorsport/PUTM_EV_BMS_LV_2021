EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 850  1900 1150 700 
U 5FAD7AC5
F0 "Power" 50
F1 "Power.sch" 50
$EndSheet
$Comp
L Device:C_Small C6
U 1 1 5FB27BDA
P 5650 2800
F 0 "C6" H 5742 2846 50  0000 L CNN
F 1 "100n" H 5742 2755 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5650 2800 50  0001 C CNN
F 3 "~" H 5650 2800 50  0001 C CNN
	1    5650 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5FB29E0C
P 6100 2350
F 0 "C8" H 6192 2396 50  0000 L CNN
F 1 "100n" H 6192 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 6100 2350 50  0001 C CNN
F 3 "~" H 6100 2350 50  0001 C CNN
	1    6100 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5FB2ADEE
P 5200 2350
F 0 "C4" H 5292 2396 50  0000 L CNN
F 1 "100n" H 5292 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5200 2350 50  0001 C CNN
F 3 "~" H 5200 2350 50  0001 C CNN
	1    5200 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5FB2A08E
P 5500 2350
F 0 "C5" H 5592 2396 50  0000 L CNN
F 1 "100n" H 5592 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5500 2350 50  0001 C CNN
F 3 "~" H 5500 2350 50  0001 C CNN
	1    5500 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5FB26842
P 5800 2350
F 0 "C7" H 5892 2396 50  0000 L CNN
F 1 "100n" H 5892 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5800 2350 50  0001 C CNN
F 3 "~" H 5800 2350 50  0001 C CNN
	1    5800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2250 5800 2250
Connection ~ 5200 2250
Wire Wire Line
	5200 2250 5100 2250
Connection ~ 5500 2250
Wire Wire Line
	5500 2250 5200 2250
Connection ~ 5800 2250
Wire Wire Line
	5800 2250 5500 2250
Connection ~ 4900 2250
Connection ~ 5000 2250
Wire Wire Line
	5000 2250 4900 2250
Wire Wire Line
	4700 2250 4800 2250
Connection ~ 4800 2250
Wire Wire Line
	4800 2250 4900 2250
$Comp
L Device:L_Small L1
U 1 1 5FB3384E
P 5100 2550
F 0 "L1" H 5148 2596 50  0000 L CNN
F 1 "10u" H 5148 2505 50  0000 L CNN
F 2 "Inductor_SMD:L_0805_2012Metric" H 5100 2550 50  0001 C CNN
F 3 "~" H 5100 2550 50  0001 C CNN
	1    5100 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 2450 5100 2250
Connection ~ 5100 2250
Wire Wire Line
	5100 2250 5000 2250
Wire Wire Line
	5100 2700 5100 2650
$Comp
L power:GND #PWR011
U 1 1 5FB3676C
P 5650 2900
F 0 "#PWR011" H 5650 2650 50  0001 C CNN
F 1 "GND" H 5655 2727 50  0000 C CNN
F 2 "" H 5650 2900 50  0001 C CNN
F 3 "" H 5650 2900 50  0001 C CNN
	1    5650 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2450 5500 2450
Connection ~ 5500 2450
Wire Wire Line
	5500 2450 5800 2450
Connection ~ 5800 2450
Wire Wire Line
	5800 2450 6100 2450
$Comp
L power:GND #PWR013
U 1 1 5FB3954C
P 6100 2500
F 0 "#PWR013" H 6100 2250 50  0001 C CNN
F 1 "GND" H 6105 2327 50  0000 C CNN
F 2 "" H 6100 2500 50  0001 C CNN
F 3 "" H 6100 2500 50  0001 C CNN
	1    6100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2450 6100 2500
Connection ~ 6100 2450
$Comp
L power:+3.3V #PWR012
U 1 1 5FB39954
P 6100 2200
F 0 "#PWR012" H 6100 2050 50  0001 C CNN
F 1 "+3.3V" H 6115 2373 50  0000 C CNN
F 2 "" H 6100 2200 50  0001 C CNN
F 3 "" H 6100 2200 50  0001 C CNN
	1    6100 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2200 6100 2250
Connection ~ 6100 2250
$Comp
L Device:Crystal Y1
U 1 1 5FB3B115
P 3750 3400
F 0 "Y1" V 3704 3531 50  0000 L CNN
F 1 "Crystal" V 3795 3531 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_HC49-SD" H 3750 3400 50  0001 C CNN
F 3 "~" H 3750 3400 50  0001 C CNN
	1    3750 3400
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5FB3D416
P 3450 3250
F 0 "C1" H 3542 3296 50  0000 L CNN
F 1 "22p" H 3542 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3450 3250 50  0001 C CNN
F 3 "~" H 3450 3250 50  0001 C CNN
	1    3450 3250
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5FB3DF71
P 3450 3550
F 0 "C2" H 3542 3596 50  0000 L CNN
F 1 "22p" H 3542 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3450 3550 50  0001 C CNN
F 3 "~" H 3450 3550 50  0001 C CNN
	1    3450 3550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3750 3250 3550 3250
Wire Wire Line
	3550 3550 3750 3550
Wire Wire Line
	3750 3550 4100 3550
Wire Wire Line
	4100 3550 4100 3450
Connection ~ 3750 3550
Wire Wire Line
	4100 3350 4100 3250
Wire Wire Line
	4100 3250 3750 3250
Connection ~ 3750 3250
Connection ~ 5100 2700
Wire Wire Line
	5100 2700 5650 2700
Wire Wire Line
	5100 2750 5100 2700
Wire Wire Line
	5000 2250 5000 2750
Wire Wire Line
	4900 2750 4900 2250
Wire Wire Line
	4800 2250 4800 2750
Wire Wire Line
	4700 2750 4700 2250
Wire Wire Line
	4100 3450 4200 3450
Wire Wire Line
	4200 3350 4100 3350
$Comp
L MCU_ST_STM32F1:STM32F103C8Tx U1
U 1 1 5FAFDC6D
P 4900 4250
F 0 "U1" H 4850 2661 50  0000 C CNN
F 1 "STM32F103C8Tx" H 4850 2570 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 4300 2850 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 4900 4250 50  0001 C CNN
	1    4900 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5FB4288C
P 3250 3550
F 0 "#PWR06" H 3250 3300 50  0001 C CNN
F 1 "GND" H 3255 3377 50  0000 C CNN
F 2 "" H 3250 3550 50  0001 C CNN
F 3 "" H 3250 3550 50  0001 C CNN
	1    3250 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5FB43535
P 3250 3250
F 0 "#PWR05" H 3250 3000 50  0001 C CNN
F 1 "GND" H 3255 3077 50  0000 C CNN
F 2 "" H 3250 3250 50  0001 C CNN
F 3 "" H 3250 3250 50  0001 C CNN
	1    3250 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3550 3350 3550
Wire Wire Line
	3250 3250 3350 3250
$Comp
L Device:R_Small R6
U 1 1 5FB45468
P 3950 3150
F 0 "R6" V 3754 3150 50  0000 C CNN
F 1 "100K" V 3845 3150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3950 3150 50  0001 C CNN
F 3 "~" H 3950 3150 50  0001 C CNN
	1    3950 3150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5FB46A7D
P 3750 3150
F 0 "#PWR09" H 3750 2900 50  0001 C CNN
F 1 "GND" H 3755 2977 50  0000 C CNN
F 2 "" H 3750 3150 50  0001 C CNN
F 3 "" H 3750 3150 50  0001 C CNN
	1    3750 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	3750 3150 3850 3150
Wire Wire Line
	4050 3150 4200 3150
$Comp
L Device:R_Small R5
U 1 1 5FB480CC
P 3600 2200
F 0 "R5" V 3404 2200 50  0000 C CNN
F 1 "100K" V 3495 2200 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3600 2200 50  0001 C CNN
F 3 "~" H 3600 2200 50  0001 C CNN
	1    3600 2200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR07
U 1 1 5FB496D7
P 3600 2100
F 0 "#PWR07" H 3600 1950 50  0001 C CNN
F 1 "+3.3V" H 3615 2273 50  0000 C CNN
F 2 "" H 3600 2100 50  0001 C CNN
F 3 "" H 3600 2100 50  0001 C CNN
	1    3600 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5FB4ACAA
P 3600 2450
F 0 "C3" H 3692 2496 50  0000 L CNN
F 1 "100n" H 3692 2405 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3600 2450 50  0001 C CNN
F 3 "~" H 3600 2450 50  0001 C CNN
	1    3600 2450
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 2300 3600 2350
$Comp
L power:GND #PWR08
U 1 1 5FB4BD46
P 3600 2650
F 0 "#PWR08" H 3600 2400 50  0001 C CNN
F 1 "GND" H 3605 2477 50  0000 C CNN
F 2 "" H 3600 2650 50  0001 C CNN
F 3 "" H 3600 2650 50  0001 C CNN
	1    3600 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2350 3850 2350
Connection ~ 3600 2350
Wire Wire Line
	4050 2950 4200 2950
Wire Wire Line
	4050 2350 4050 2950
$Comp
L Switch:SW_Push SW1
U 1 1 5FB503A5
P 3850 2550
F 0 "SW1" V 3804 2698 50  0000 L CNN
F 1 "SW_Push" V 3895 2698 50  0000 L CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 3850 2750 50  0001 C CNN
F 3 "~" H 3850 2750 50  0001 C CNN
	1    3850 2550
	0    1    1    0   
$EndComp
Connection ~ 3850 2350
Wire Wire Line
	3850 2350 4050 2350
Wire Wire Line
	3600 2550 3600 2600
Wire Wire Line
	3600 2600 3750 2600
Wire Wire Line
	3750 2600 3750 2750
Wire Wire Line
	3750 2750 3850 2750
Connection ~ 3600 2600
Wire Wire Line
	3600 2600 3600 2650
Wire Wire Line
	4700 5750 4700 5800
Wire Wire Line
	4700 5800 4800 5800
Wire Wire Line
	5000 5800 5000 5750
Wire Wire Line
	4900 5750 4900 5800
Connection ~ 4900 5800
Wire Wire Line
	4900 5800 5000 5800
Wire Wire Line
	4800 5750 4800 5800
Connection ~ 4800 5800
Wire Wire Line
	4800 5800 4900 5800
Wire Wire Line
	4800 5800 4800 5900
$Comp
L power:GND #PWR010
U 1 1 5FB5A063
P 4800 5900
F 0 "#PWR010" H 4800 5650 50  0001 C CNN
F 1 "GND" H 4805 5727 50  0000 C CNN
F 2 "" H 4800 5900 50  0001 C CNN
F 3 "" H 4800 5900 50  0001 C CNN
	1    4800 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3450 8100 3450
Wire Wire Line
	8250 3550 7850 3550
Wire Wire Line
	8250 3650 8100 3650
Wire Wire Line
	8250 3750 7850 3750
Wire Wire Line
	8250 3850 7850 3850
Wire Wire Line
	8250 3950 7850 3950
Text GLabel 8250 3450 2    50   Input ~ 0
SW_VDD
Text GLabel 8250 3550 2    50   Input ~ 0
SW_CLK
$Comp
L Connector:Conn_01x06_Male J1
U 1 1 5FB6999E
P 7650 3650
F 0 "J1" H 7758 4031 50  0000 C CNN
F 1 "Conn_01x06_Male" H 7758 3940 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 7650 3650 50  0001 C CNN
F 3 "~" H 7650 3650 50  0001 C CNN
	1    7650 3650
	1    0    0    -1  
$EndComp
Text GLabel 8250 3650 2    50   Input ~ 0
SW_GND
Text GLabel 8250 3750 2    50   Input ~ 0
SW_DIO
Text GLabel 8250 3850 2    50   Input ~ 0
SW_NRST
NoConn ~ 8250 3950
$Comp
L power:+3.3V #PWR015
U 1 1 5FB73F02
P 8100 3300
F 0 "#PWR015" H 8100 3150 50  0001 C CNN
F 1 "+3.3V" H 8115 3473 50  0000 C CNN
F 2 "" H 8100 3300 50  0001 C CNN
F 3 "" H 8100 3300 50  0001 C CNN
	1    8100 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 3300 8100 3450
Connection ~ 8100 3450
Wire Wire Line
	8100 3450 7850 3450
Wire Wire Line
	8100 3650 8100 4100
Connection ~ 8100 3650
Wire Wire Line
	8100 3650 7850 3650
$Comp
L power:GND #PWR016
U 1 1 5FB788E0
P 8100 4100
F 0 "#PWR016" H 8100 3850 50  0001 C CNN
F 1 "GND" H 8105 3927 50  0000 C CNN
F 2 "" H 8100 4100 50  0001 C CNN
F 3 "" H 8100 4100 50  0001 C CNN
	1    8100 4100
	1    0    0    -1  
$EndComp
Text GLabel 4000 2350 1    50   Input ~ 0
SW_NRST
Text GLabel 5500 5450 2    50   Input ~ 0
SW_CLK
Text GLabel 5500 5350 2    50   Input ~ 0
SW_DIO
$Comp
L Device:LED D2
U 1 1 5FB7AFBE
P 3750 4250
F 0 "D2" H 3743 4466 50  0000 C CNN
F 1 "LED" H 3743 4375 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 3750 4250 50  0001 C CNN
F 3 "~" H 3750 4250 50  0001 C CNN
	1    3750 4250
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5FB7C791
P 3400 4250
F 0 "R2" V 3204 4250 50  0000 C CNN
F 1 "120" V 3295 4250 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3400 4250 50  0001 C CNN
F 3 "~" H 3400 4250 50  0001 C CNN
	1    3400 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 4250 3900 4250
Wire Wire Line
	3600 4250 3500 4250
$Sheet
S 850  2850 1150 650 
U 5FB83FF2
F0 "Can" 50
F1 "Can.sch" 50
$EndSheet
Text GLabel 4200 4950 0    50   Input ~ 0
TX_Can
Text GLabel 4200 4850 0    50   Input ~ 0
RX_Can
$Sheet
S 850  3800 1150 650 
U 5FF86554
F0 "LTC6810" 50
F1 "LTC.sch" 50
$EndSheet
$Comp
L Device:LED D1
U 1 1 5FFA9478
P 3750 4150
F 0 "D1" H 3743 4366 50  0000 C CNN
F 1 "LED" H 3743 4275 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 3750 4150 50  0001 C CNN
F 3 "~" H 3750 4150 50  0001 C CNN
	1    3750 4150
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5FFA947E
P 3400 5050
F 0 "R3" V 3204 5050 50  0000 C CNN
F 1 "120" V 3295 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3400 5050 50  0001 C CNN
F 3 "~" H 3400 5050 50  0001 C CNN
	1    3400 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 4150 3900 4150
Wire Wire Line
	3600 4150 3500 4150
$Comp
L power:+3.3V #PWR01
U 1 1 5FFA9487
P 3050 4250
F 0 "#PWR01" H 3050 4100 50  0001 C CNN
F 1 "+3.3V" H 3065 4423 50  0000 C CNN
F 2 "" H 3050 4250 50  0001 C CNN
F 3 "" H 3050 4250 50  0001 C CNN
	1    3050 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5FFAB034
P 3750 5150
F 0 "D4" H 3743 5366 50  0000 C CNN
F 1 "LED" H 3743 5275 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 3750 5150 50  0001 C CNN
F 3 "~" H 3750 5150 50  0001 C CNN
	1    3750 5150
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5FFAB03A
P 3400 5150
F 0 "R4" V 3204 5150 50  0000 C CNN
F 1 "120" V 3295 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3400 5150 50  0001 C CNN
F 3 "~" H 3400 5150 50  0001 C CNN
	1    3400 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 5150 3900 5150
Wire Wire Line
	3600 5150 3500 5150
Text GLabel 4200 4350 0    50   Input ~ 0
SCK
Text GLabel 4200 4450 0    50   Input ~ 0
MISO
Text GLabel 4200 4550 0    50   Input ~ 0
MOSI
Text GLabel 5500 5550 2    50   Input ~ 0
CS
Text GLabel 5500 4350 2    50   Input ~ 0
Term_1
Text GLabel 5500 4450 2    50   Input ~ 0
Term_2
Text GLabel 5500 4550 2    50   Input ~ 0
Term_3
Text GLabel 5500 4650 2    50   Input ~ 0
Term_4
$Comp
L Device:R_Small R1
U 1 1 605BF241
P 3400 4150
F 0 "R1" V 3204 4150 50  0000 C CNN
F 1 "120" V 3295 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3400 4150 50  0001 C CNN
F 3 "~" H 3400 4150 50  0001 C CNN
	1    3400 4150
	0    1    1    0   
$EndComp
$Comp
L Device:LED D3
U 1 1 605BF247
P 3750 5050
F 0 "D3" H 3743 5266 50  0000 C CNN
F 1 "LED" H 3743 5175 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 3750 5050 50  0001 C CNN
F 3 "~" H 3750 5050 50  0001 C CNN
	1    3750 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 5050 3900 5050
Wire Wire Line
	3600 5050 3500 5050
Text GLabel 5500 4750 2    50   Input ~ 0
Term_5
Text GLabel 4200 4050 0    50   Input ~ 0
Term_6
$Sheet
S 850  4700 1150 650 
U 60619A6B
F0 "Thermistors" 50
F1 "Thermistors.sch" 50
$EndSheet
Wire Wire Line
	3300 4150 3150 4150
Wire Wire Line
	3300 4250 3050 4250
Text GLabel 6100 5250 2    50   Input ~ 0
USB_DP
Text GLabel 6100 5150 2    50   Input ~ 0
USB_DM
$Comp
L Connector:USB_B_Mini J2
U 1 1 5FFA0FD0
P 8450 1550
F 0 "J2" H 8220 1447 50  0000 R CNN
F 1 "USB_B_Mini" H 8220 1538 50  0000 R CNN
F 2 "Connector_USB:USB_Micro-B_Molex_47346-0001" H 8600 1500 50  0001 C CNN
F 3 "~" H 8600 1500 50  0001 C CNN
	1    8450 1550
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5FFA2879
P 8750 1100
F 0 "#PWR019" H 8750 850 50  0001 C CNN
F 1 "GND" H 8755 927 50  0000 C CNN
F 2 "" H 8750 1100 50  0001 C CNN
F 3 "" H 8750 1100 50  0001 C CNN
	1    8750 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 1100 8450 1150
$Comp
L power:+5V #PWR014
U 1 1 5FFAAF0D
P 7750 1750
F 0 "#PWR014" H 7750 1600 50  0001 C CNN
F 1 "+5V" H 7765 1923 50  0000 C CNN
F 2 "" H 7750 1750 50  0001 C CNN
F 3 "" H 7750 1750 50  0001 C CNN
	1    7750 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 1100 8550 1100
Wire Wire Line
	8550 1150 8550 1100
Connection ~ 8550 1100
Wire Wire Line
	8550 1100 8750 1100
Text GLabel 8150 1450 0    50   Input ~ 0
USB_DM
Text GLabel 8150 1550 0    50   Input ~ 0
USB_DP
Wire Wire Line
	7750 1750 8150 1750
$Comp
L power:+3.3V #PWR03
U 1 1 5FB83597
P 3150 4150
F 0 "#PWR03" H 3150 4000 50  0001 C CNN
F 1 "+3.3V" H 3165 4323 50  0000 C CNN
F 2 "" H 3150 4150 50  0001 C CNN
F 3 "" H 3150 4150 50  0001 C CNN
	1    3150 4150
	1    0    0    -1  
$EndComp
Text GLabel 5500 4250 2    50   Input ~ 0
En_Relay
$Comp
L power:+3.3V #PWR02
U 1 1 600216FB
P 3050 5150
F 0 "#PWR02" H 3050 5000 50  0001 C CNN
F 1 "+3.3V" H 3065 5323 50  0000 C CNN
F 2 "" H 3050 5150 50  0001 C CNN
F 3 "" H 3050 5150 50  0001 C CNN
	1    3050 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 5050 3150 5050
Wire Wire Line
	3300 5150 3050 5150
$Comp
L power:+3.3V #PWR04
U 1 1 60021703
P 3150 5050
F 0 "#PWR04" H 3150 4900 50  0001 C CNN
F 1 "+3.3V" H 3165 5223 50  0000 C CNN
F 2 "" H 3150 5050 50  0001 C CNN
F 3 "" H 3150 5050 50  0001 C CNN
	1    3150 5050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5FFFFAC2
P 7900 2450
AR Path="/5FAD7AC5/5FFFFAC2" Ref="J?"  Part="1" 
AR Path="/5FFFFAC2" Ref="J3"  Part="1" 
F 0 "J3" H 7792 2125 50  0000 C CNN
F 1 "Conn_01x02_Female" H 7792 2216 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 7900 2450 50  0001 C CNN
F 3 "~" H 7900 2450 50  0001 C CNN
	1    7900 2450
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FFFFAC8
P 8250 2450
AR Path="/5FAD7AC5/5FFFFAC8" Ref="#PWR?"  Part="1" 
AR Path="/5FFFFAC8" Ref="#PWR0101"  Part="1" 
F 0 "#PWR0101" H 8250 2200 50  0001 C CNN
F 1 "GND" H 8255 2277 50  0000 C CNN
F 2 "" H 8250 2450 50  0001 C CNN
F 3 "" H 8250 2450 50  0001 C CNN
	1    8250 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2450 8100 2450
Wire Wire Line
	8100 2350 8250 2350
Wire Wire Line
	8250 2350 8250 2300
$Comp
L power:VCC #PWR0102
U 1 1 60004D86
P 8250 2300
F 0 "#PWR0102" H 8250 2150 50  0001 C CNN
F 1 "VCC" H 8267 2473 50  0000 C CNN
F 2 "" H 8250 2300 50  0001 C CNN
F 3 "" H 8250 2300 50  0001 C CNN
	1    8250 2300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 60009E9A
P 7900 3000
AR Path="/5FB83FF2/60009E9A" Ref="J?"  Part="1" 
AR Path="/60009E9A" Ref="J5"  Part="1" 
F 0 "J5" H 7928 2976 50  0000 L CNN
F 1 "Conn_01x02_Female" H 7928 2885 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 7900 3000 50  0001 C CNN
F 3 "~" H 7900 3000 50  0001 C CNN
	1    7900 3000
	-1   0    0    1   
$EndComp
Text GLabel 8100 3000 2    50   Input ~ 0
CANH
Text GLabel 8100 2900 2    50   Input ~ 0
CANL
Text GLabel 7150 5800 0    50   Input ~ 0
En_Relay
$Comp
L power:+24V #PWR017
U 1 1 6061005B
P 8000 4900
F 0 "#PWR017" H 8000 4750 50  0001 C CNN
F 1 "+24V" H 8015 5073 50  0000 C CNN
F 2 "" H 8000 4900 50  0001 C CNN
F 3 "" H 8000 4900 50  0001 C CNN
	1    8000 4900
	1    0    0    -1  
$EndComp
Wire Notes Line
	6800 1000 9150 1000
Wire Notes Line
	9150 1000 9150 4600
Wire Notes Line
	9150 4600 6800 4600
Wire Notes Line
	6800 4600 6800 1000
Text Label 6800 1000 0    157  ~ 0
Connectors
$Comp
L RM85V7:RM85V7 K1
U 1 1 605FDFF5
P 8350 5200
F 0 "K1" H 8375 5475 50  0000 C CNN
F 1 "RM85V7" H 8375 5384 50  0000 C CNN
F 2 "RM85V7:RM85V7" V 8500 5200 50  0001 C CNN
F 3 "" V 8500 5200 50  0001 C CNN
	1    8350 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4900 8000 5150
Wire Wire Line
	8000 5350 8000 5600
$Comp
L power:GND #PWR018
U 1 1 6065B29C
P 8000 6100
F 0 "#PWR018" H 8000 5850 50  0001 C CNN
F 1 "GND" H 8005 5927 50  0000 C CNN
F 2 "" H 8000 6100 50  0001 C CNN
F 3 "" H 8000 6100 50  0001 C CNN
	1    8000 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 5800 7550 5800
Wire Wire Line
	7550 5800 7550 6050
Wire Wire Line
	7550 6050 7650 6050
$Comp
L Device:R_Small R8
U 1 1 6065E7D3
P 7750 6050
F 0 "R8" V 7554 6050 50  0000 C CNN
F 1 "10K" V 7645 6050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 7750 6050 50  0001 C CNN
F 3 "~" H 7750 6050 50  0001 C CNN
	1    7750 6050
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 6000 8000 6050
Wire Wire Line
	8000 6050 7850 6050
Connection ~ 8000 6050
Wire Wire Line
	8000 6050 8000 6100
Wire Wire Line
	7550 5800 7450 5800
Connection ~ 7550 5800
$Comp
L Device:R_Small R7
U 1 1 6066752D
P 7350 5800
F 0 "R7" V 7154 5800 50  0000 C CNN
F 1 "100" V 7245 5800 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 7350 5800 50  0001 C CNN
F 3 "~" H 7350 5800 50  0001 C CNN
	1    7350 5800
	0    1    1    0   
$EndComp
Wire Wire Line
	7250 5800 7150 5800
Wire Wire Line
	8750 5200 8850 5200
Wire Wire Line
	8750 5300 8850 5300
Text GLabel 8850 5300 2    50   Input ~ 0
V_Out
$Comp
L power:+24V #PWR020
U 1 1 6068138D
P 8850 5200
F 0 "#PWR020" H 8850 5050 50  0001 C CNN
F 1 "+24V" V 8865 5328 50  0000 L CNN
F 2 "" H 8850 5200 50  0001 C CNN
F 3 "" H 8850 5200 50  0001 C CNN
	1    8850 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5350 7900 5350
Wire Wire Line
	7900 5150 8000 5150
Connection ~ 8000 5350
Connection ~ 8000 5150
$Comp
L Device:D_Small D16
U 1 1 60308393
P 7900 5250
F 0 "D16" H 7900 5455 50  0000 C CNN
F 1 "D_Small" H 7900 5364 50  0000 C CNN
F 2 "Diode_SMD:D_SMB" V 7900 5250 50  0001 C CNN
F 3 "~" V 7900 5250 50  0001 C CNN
	1    7900 5250
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 60613319
P 7900 5800
F 0 "Q1" H 8105 5754 50  0000 L CNN
F 1 "BSS138" H 8105 5845 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8100 5725 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 7900 5800 50  0001 L CNN
	1    7900 5800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J13
U 1 1 60342BDE
P 7000 1450
F 0 "J13" H 7108 1731 50  0000 C CNN
F 1 "Conn_01x04_Male" H 7108 1640 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7000 1450 50  0001 C CNN
F 3 "~" H 7000 1450 50  0001 C CNN
	1    7000 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 6034A78E
P 7200 1350
F 0 "#PWR0104" H 7200 1200 50  0001 C CNN
F 1 "+5V" H 7215 1523 50  0000 C CNN
F 2 "" H 7200 1350 50  0001 C CNN
F 3 "" H 7200 1350 50  0001 C CNN
	1    7200 1350
	1    0    0    -1  
$EndComp
Text GLabel 7200 1450 2    50   Input ~ 0
USB_DM
Text GLabel 7200 1550 2    50   Input ~ 0
USB_DP
$Comp
L power:GND #PWR0105
U 1 1 60352852
P 7200 1650
F 0 "#PWR0105" H 7200 1400 50  0001 C CNN
F 1 "GND" H 7205 1477 50  0000 C CNN
F 2 "" H 7200 1650 50  0001 C CNN
F 3 "" H 7200 1650 50  0001 C CNN
	1    7200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R39
U 1 1 6036F928
P 5800 5150
F 0 "R39" V 5604 5150 50  0000 C CNN
F 1 "27" V 5695 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 5800 5150 50  0001 C CNN
F 3 "~" H 5800 5150 50  0001 C CNN
	1    5800 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R40
U 1 1 60370D0A
P 5800 5250
F 0 "R40" V 5604 5250 50  0000 C CNN
F 1 "27" V 5695 5250 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 5800 5250 50  0001 C CNN
F 3 "~" H 5800 5250 50  0001 C CNN
	1    5800 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	5500 5150 5700 5150
Wire Wire Line
	5500 5250 5700 5250
Wire Wire Line
	5900 5150 6100 5150
Wire Wire Line
	5900 5250 6100 5250
$EndSCHEMATC
