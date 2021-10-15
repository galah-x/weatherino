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
L swinggate_sch:TP4056_BD U1
U 1 1 5C68DB1D
P 3400 3150
F 0 "U1" H 3500 3591 79  0000 C CNN
F 1 "TP4056_BD" H 3500 3456 79  0000 C CNN
F 2 "" H 3400 3150 79  0001 C CNN
F 3 "" H 3400 3150 79  0001 C CNN
	1    3400 3150
	1    0    0    -1  
$EndComp
$Comp
L swinggate_sch:Moteino_hcw U4
U 1 1 5C68DC28
P 7700 3400
F 0 "U4" H 7500 2500 79  0000 C CNN
F 1 "Moteino_hcw" H 7700 2650 79  0000 C CNN
F 2 "" H 7600 4200 79  0001 C CNN
F 3 "" H 7600 4200 79  0001 C CNN
	1    7700 3400
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4001 D1
U 1 1 5C68DD4E
P 2200 3000
F 0 "D1" H 2200 2784 50  0000 C CNN
F 1 "1N4001" H 2200 2875 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 2200 2825 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 2200 3000 50  0001 C CNN
	1    2200 3000
	-1   0    0    1   
$EndComp
$Comp
L Device:Solar_Cells SC1
U 1 1 5C68DE4D
P 1100 3200
F 0 "SC1" H 1208 3246 50  0000 L CNN
F 1 "6V Solar_Cell" H 1208 3155 50  0000 L CNN
F 2 "" V 1100 3260 50  0001 C CNN
F 3 "~" V 1100 3260 50  0001 C CNN
	1    1100 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery_Cell BT1
U 1 1 5C68DF5A
P 4700 3200
F 0 "BT1" H 4818 3296 50  0000 L CNN
F 1 "Battery_Cell" H 4818 3205 50  0000 L CNN
F 2 "" V 4700 3260 50  0001 C CNN
F 3 "~" V 4700 3260 50  0001 C CNN
	1    4700 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3000 3100 3000
Wire Wire Line
	1100 3000 2050 3000
Wire Wire Line
	4700 3000 3900 3000
Wire Wire Line
	7150 4000 6550 4000
Wire Wire Line
	4700 3300 4700 4000
Connection ~ 4700 4000
Wire Wire Line
	4700 4000 3900 4000
Wire Wire Line
	3900 3300 3900 4000
Wire Wire Line
	3900 4000 3100 4000
Wire Wire Line
	3100 3300 3100 4000
Connection ~ 3900 4000
Wire Wire Line
	3100 4000 1100 4000
Wire Wire Line
	1100 4000 1100 3400
Connection ~ 3100 4000
Wire Wire Line
	7150 3900 5850 3900
Connection ~ 4700 3000
Text Notes 4750 3400 0    50   ~ 0
3.7V 1000mAH LiPo
$Comp
L swinggate_sch:SHT31_BD U3
U 1 1 5C68EA9F
P 6600 1600
F 0 "U3" H 6550 2150 79  0000 L CNN
F 1 "SHT31_BD" H 6500 2000 79  0000 L CNN
F 2 "" H 6600 1600 79  0001 C CNN
F 3 "" H 6600 1600 79  0001 C CNN
	1    6600 1600
	1    0    0    -1  
$EndComp
$Comp
L swinggate_sch:BMP280_BD U2
U 1 1 5C68EC67
P 5150 1600
F 0 "U2" H 5000 2150 79  0000 L CNN
F 1 "BMP280_BD" H 4950 2000 79  0000 L CNN
F 2 "" H 5150 1600 79  0001 C CNN
F 3 "" H 5150 1600 79  0001 C CNN
	1    5150 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5C68ED0B
P 7750 2250
F 0 "R4" H 7820 2296 50  0000 L CNN
F 1 "4K7" H 7820 2205 50  0000 L CNN
F 2 "" V 7680 2250 50  0001 C CNN
F 3 "~" H 7750 2250 50  0001 C CNN
	1    7750 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5C68EDCB
P 7400 2250
F 0 "R3" H 7330 2204 50  0000 R CNN
F 1 "4K7" H 7330 2295 50  0000 R CNN
F 2 "" V 7330 2250 50  0001 C CNN
F 3 "~" H 7400 2250 50  0001 C CNN
	1    7400 2250
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 5C68EDFB
P 6200 3500
F 0 "R1" V 5993 3500 50  0000 C CNN
F 1 "390K" V 6084 3500 50  0000 C CNN
F 2 "" V 6130 3500 50  0001 C CNN
F 3 "~" H 6200 3500 50  0001 C CNN
	1    6200 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5C68EE23
P 6350 2950
F 0 "R2" H 6280 2904 50  0000 R CNN
F 1 "1M" H 6280 2995 50  0000 R CNN
F 2 "" V 6280 2950 50  0001 C CNN
F 3 "~" H 6350 2950 50  0001 C CNN
	1    6350 2950
	-1   0    0    1   
$EndComp
Wire Wire Line
	7150 2800 6350 2800
Wire Wire Line
	6350 3500 7150 3500
Wire Wire Line
	5850 3000 5850 3500
Wire Wire Line
	6050 3500 5850 3500
Connection ~ 5850 3500
Wire Wire Line
	5850 3500 5850 3900
Wire Wire Line
	6350 3100 6350 3500
Connection ~ 6350 3500
Wire Wire Line
	5200 1900 5200 4000
Connection ~ 5200 4000
Wire Wire Line
	5200 4000 4700 4000
Wire Wire Line
	7150 3800 6850 3800
Wire Wire Line
	6850 3800 6850 4450
Wire Wire Line
	6850 4450 8850 4450
Wire Wire Line
	8850 4450 8850 2000
Wire Wire Line
	8850 2000 7750 2000
Wire Wire Line
	7750 2000 7750 2100
Wire Wire Line
	7750 2000 7400 2000
Wire Wire Line
	7400 2000 7400 2100
Connection ~ 7750 2000
Wire Wire Line
	6450 2000 6450 1900
Connection ~ 7400 2000
Wire Wire Line
	6550 1900 6550 4000
Connection ~ 6550 4000
Wire Wire Line
	6550 4000 5200 4000
Wire Wire Line
	7150 3300 6700 3300
Wire Wire Line
	6700 3300 6700 2600
Wire Wire Line
	6700 2600 6650 2600
Wire Wire Line
	6700 2600 7750 2600
Wire Wire Line
	7750 2600 7750 2400
Connection ~ 6700 2600
Wire Wire Line
	6650 1900 6650 2450
Wire Wire Line
	7150 3200 6750 3200
Wire Wire Line
	6750 3200 6750 2500
Wire Wire Line
	7400 2400 7400 2500
Wire Wire Line
	7400 2500 6750 2500
Connection ~ 6750 2500
Wire Wire Line
	6750 2500 6750 2300
Wire Wire Line
	5500 1900 5500 2300
Wire Wire Line
	5500 2300 6750 2300
Connection ~ 6750 2300
Wire Wire Line
	6750 2300 6750 1900
Wire Wire Line
	5300 1900 5300 2450
Wire Wire Line
	5300 2450 6650 2450
Connection ~ 6650 2450
Wire Wire Line
	6650 2450 6650 2600
Text Label 5900 2300 0    50   ~ 0
SDA
Text Label 5900 2450 0    50   ~ 0
SCL
Text Label 8050 2000 0    50   ~ 0
3V3
Text Label 5400 3000 0    50   ~ 0
VBATT
Text Label 5400 4000 0    50   ~ 0
GND
NoConn ~ 5600 1900
NoConn ~ 5400 1900
NoConn ~ 6850 1900
NoConn ~ 6950 1900
NoConn ~ 7050 1900
NoConn ~ 8250 2800
NoConn ~ 8250 2900
NoConn ~ 8250 3000
NoConn ~ 8250 3100
NoConn ~ 8250 3200
NoConn ~ 8250 3300
NoConn ~ 8250 4000
NoConn ~ 8250 3900
NoConn ~ 8250 3800
NoConn ~ 8250 3700
NoConn ~ 8250 3600
NoConn ~ 8250 3500
NoConn ~ 8250 3400
NoConn ~ 7150 3700
NoConn ~ 7150 3600
NoConn ~ 7150 3400
NoConn ~ 7150 3100
NoConn ~ 7150 3000
NoConn ~ 7150 2900
Text Notes 7450 7500 0    79   ~ 0
WEATHERINO_temperature - Temperature/Humidity/Pressure
Text Notes 8650 7650 0    50   ~ 0
17 Feb 2019
Text Notes 10750 7650 0    50   ~ 0
1
Text Notes 9100 2350 0    50   ~ 0
Note R3,R4, I2C pullups,\nare on lugin boards so not fitted
Text Notes 8050 4350 0    50   ~ 0
CW board used
Wire Wire Line
	4700 3000 5850 3000
Wire Wire Line
	5100 2000 5100 1900
Wire Wire Line
	5100 2000 6450 2000
Connection ~ 6450 2000
Wire Wire Line
	6450 2000 7400 2000
Wire Wire Line
	5100 2000 5000 2000
Wire Wire Line
	5000 2000 5000 1900
Connection ~ 5100 2000
Text Notes 1000 2600 0    50   ~ 0
D1 helps reduce SC voltage to well within \n8V input range of regulator
$EndSCHEMATC
