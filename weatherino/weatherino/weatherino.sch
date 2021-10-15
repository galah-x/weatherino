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
	3100 4000 2000 4000
Wire Wire Line
	1100 4000 1100 3400
Connection ~ 3100 4000
Wire Wire Line
	7150 3900 5850 3900
Connection ~ 4700 3000
Text Notes 4700 3500 0    50   ~ 0
3.7V 3000mAH LiPo \n18650 Samsung\nPink
$Comp
L Device:R R6
U 1 1 5C68ED0B
P 4100 5800
F 0 "R6" V 4170 5846 50  0000 L CNN
F 1 "47K" V 4284 5755 50  0000 L CNN
F 2 "" V 4030 5800 50  0001 C CNN
F 3 "~" H 4100 5800 50  0001 C CNN
	1    4100 5800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5C68EDFB
P 6200 3500
F 0 "R1" V 5993 3500 50  0000 C CNN
F 1 "499K" V 6084 3500 50  0000 C CNN
F 2 "" V 6130 3500 50  0001 C CNN
F 3 "~" H 6200 3500 50  0001 C CNN
	1    6200 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5C68EE23
P 6500 3650
F 0 "R2" H 6430 3604 50  0000 R CNN
F 1 "1M" H 6430 3695 50  0000 R CNN
F 2 "" V 6430 3650 50  0001 C CNN
F 3 "~" H 6500 3650 50  0001 C CNN
	1    6500 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	6350 3500 6500 3500
Wire Wire Line
	5850 3000 5850 3500
Wire Wire Line
	6050 3500 5850 3500
Connection ~ 5850 3500
Wire Wire Line
	5850 3500 5850 3900
Wire Wire Line
	7150 3800 6850 3800
Text Label 5600 4600 0    50   ~ 0
3V3
Text Label 5400 3000 0    50   ~ 0
VBATT
Text Label 5400 4000 0    50   ~ 0
GND
Text Notes 8000 7500 0    79   ~ 0
WEATHERINO - Rain, wind drn, wind speed.
Text Notes 8650 7650 0    50   ~ 0
15 Oct 2021
Text Notes 10750 7650 0    50   ~ 0
1
Text Notes 8050 4350 0    50   ~ 0
HCW board used
Wire Wire Line
	4700 3000 5850 3000
Text Notes 1000 2600 0    50   ~ 0
D1 helps reduce SC voltage to well within \n8V input range of regulator
Wire Wire Line
	4700 4000 6500 4000
Wire Wire Line
	6500 3800 6500 4000
Connection ~ 6500 4000
Wire Wire Line
	6500 4000 7150 4000
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 616B0CA5
P 2150 6650
F 0 "J2" H 2258 6831 50  0000 C CNN
F 1 "Conn_01x02_Male" H 2258 6740 50  0000 C CNN
F 2 "" H 2150 6650 50  0001 C CNN
F 3 "~" H 2150 6650 50  0001 C CNN
	1    2150 6650
	1    0    0    -1  
$EndComp
$Comp
L Connector:6P4C J1
U 1 1 616B1BBC
P 1950 5800
F 0 "J1" H 2007 6367 50  0000 C CNN
F 1 "6P4C" H 2007 6276 50  0000 C CNN
F 2 "" V 1950 5825 50  0001 C CNN
F 3 "~" V 1950 5825 50  0001 C CNN
	1    1950 5800
	1    0    0    -1  
$EndComp
Text Notes 750  5250 0    50   ~ 0
Davis Instruments\nAnemometer
Text Notes 800  6700 0    50   ~ 0
Davis Instruments \nTipping bucket\nRain gauge
Text Notes 800  7000 0    50   ~ 0
Old, recently calibrated\nto 0.25mm / tip
Wire Wire Line
	2350 5900 2850 5900
Wire Wire Line
	2850 5900 2850 4600
Wire Wire Line
	2850 4600 3350 4600
Text Notes 700  5850 0    50   ~ 0
Power Yellow 2/6
Text Notes 700  5750 0    50   ~ 0
Green Wind Drn 3/6
Text Notes 700  5650 0    50   ~ 0
Red Ground 4/6
Text Notes 700  5550 0    50   ~ 0
Black Wind Speed 5/6
Text Notes 700  5450 0    50   ~ 0
NC 6/6
Text Notes 700  5950 0    50   ~ 0
NC 1/6
$Comp
L Device:R R5
U 1 1 616C92A8
P 4100 5350
F 0 "R5" V 4170 5396 50  0000 L CNN
F 1 "47K" V 4284 5305 50  0000 L CNN
F 2 "" V 4030 5350 50  0001 C CNN
F 3 "~" H 4100 5350 50  0001 C CNN
	1    4100 5350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 616CC4FE
P 3350 4950
F 0 "R4" H 3280 4904 50  0000 R CNN
F 1 "10K" H 3280 4995 50  0000 R CNN
F 2 "" V 3280 4950 50  0001 C CNN
F 3 "~" H 3350 4950 50  0001 C CNN
	1    3350 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 616CDA02
P 2950 6400
F 0 "R3" H 2880 6354 50  0000 R CNN
F 1 "10K" H 2880 6445 50  0000 R CNN
F 2 "" V 2880 6400 50  0001 C CNN
F 3 "~" H 2950 6400 50  0001 C CNN
	1    2950 6400
	-1   0    0    1   
$EndComp
Wire Wire Line
	2950 6550 2950 6650
$Comp
L Device:R R7
U 1 1 616CE8C0
P 4100 6650
F 0 "R7" V 4170 6696 50  0000 L CNN
F 1 "47K" V 4284 6605 50  0000 L CNN
F 2 "" V 4030 6650 50  0001 C CNN
F 3 "~" H 4100 6650 50  0001 C CNN
	1    4100 6650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2950 6250 2950 5900
Wire Wire Line
	2950 5900 2850 5900
Connection ~ 2850 5900
Wire Wire Line
	3350 4800 3350 4600
Connection ~ 3350 4600
Wire Wire Line
	3350 5350 3950 5350
Wire Wire Line
	3350 5100 3350 5350
Wire Wire Line
	2350 5600 3350 5600
Wire Wire Line
	3350 5600 3350 5350
Connection ~ 3350 5350
Wire Wire Line
	2350 5800 3950 5800
Wire Wire Line
	2350 6650 2950 6650
Wire Wire Line
	2950 6650 3950 6650
Connection ~ 2950 6650
Wire Wire Line
	6850 3800 6850 4600
Wire Wire Line
	3350 4600 6850 4600
Wire Wire Line
	2350 6750 2650 6750
Wire Wire Line
	2650 6750 2650 7150
Wire Wire Line
	2350 5700 2600 5700
Wire Wire Line
	2600 5700 2600 6200
Wire Wire Line
	2000 4000 2000 4250
Connection ~ 2000 4000
Wire Wire Line
	2000 4000 1100 4000
$Comp
L power:Earth #PWR0101
U 1 1 616D9AB5
P 2000 4250
F 0 "#PWR0101" H 2000 4000 50  0001 C CNN
F 1 "Earth" H 2000 4100 50  0001 C CNN
F 2 "" H 2000 4250 50  0001 C CNN
F 3 "~" H 2000 4250 50  0001 C CNN
	1    2000 4250
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0102
U 1 1 616DA620
P 2600 6200
F 0 "#PWR0102" H 2600 5950 50  0001 C CNN
F 1 "Earth" H 2600 6050 50  0001 C CNN
F 2 "" H 2600 6200 50  0001 C CNN
F 3 "~" H 2600 6200 50  0001 C CNN
	1    2600 6200
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0103
U 1 1 616DC3A2
P 2650 7150
F 0 "#PWR0103" H 2650 6900 50  0001 C CNN
F 1 "Earth" H 2650 7000 50  0001 C CNN
F 2 "" H 2650 7150 50  0001 C CNN
F 3 "~" H 2650 7150 50  0001 C CNN
	1    2650 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 5350 9150 5350
Wire Wire Line
	7150 3400 6500 3400
Wire Wire Line
	6500 3400 6500 3500
Connection ~ 6500 3500
Wire Wire Line
	7000 3500 7150 3500
Wire Wire Line
	4250 5800 7000 5800
Wire Wire Line
	6500 6650 6500 6150
Wire Wire Line
	6500 6150 9600 6150
Wire Wire Line
	9600 6150 9600 3500
Wire Wire Line
	4250 6650 6500 6650
Wire Wire Line
	8250 3500 9600 3500
Wire Wire Line
	8250 3400 9150 3400
Wire Wire Line
	9150 3400 9150 5350
Wire Wire Line
	7000 3500 7000 5800
Text Notes 3150 2250 0    50   ~ 0
This is a 4056 charge controller board\nthat implements current limit and overvolt \nprotection.  I should have got one \nthat did undervoltage too. \nIn which case load and battery are on separate \noutput pin pairs
Text Notes 4350 2750 0    50   ~ 0
Battery is in an 18650 socket
Text Notes 7100 2400 0    50   ~ 0
Moteino board also has an antenna soldered on. Its 915MHz, 82mm
Text Notes 3500 7650 0    50   ~ 0
Note: This is built on perf board, with partially pinned socket pins for the Moteino\nResistors are 0805 soldered on the back. Could be thru hole I guess.\nU1 is an ebay module, 4 wires a bit above the mounting perfboard.\nConnectors (including solar) soldered to the board.  
Text Notes 4600 6050 0    50   ~ 0
Series resistors here are mostly to protect the Mteino in the event \nof lightning  nearby,   but not too close.
Text Notes 500  7550 0    50   ~ 0
Keep external wires shortish. About 5m in my case.
Text Notes 6100 2000 0    50   ~ 0
A moteino is an arduino board wiith a RFM69 915MHz radio module soldered on the back.  \nI see a couple of hundred metres around the farm from the RFM69\nAnd up to 5 Km for the RFM95 LoRa guys at  slow baud. 
$EndSCHEMATC
