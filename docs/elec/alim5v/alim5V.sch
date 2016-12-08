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
LIBS:robair
LIBS:alim5V-cache
EELAYER 25 0
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
L UEI30-050-Q12P-C U1
U 1 1 5840745A
P 5050 3600
F 0 "U1" H 5050 3500 60  0000 C CNN
F 1 "UEI30-050-Q12P-C" H 5100 3800 60  0000 C CNN
F 2 "robair:UEI30-050-Q12P-C" H 5100 3550 60  0001 C CNN
F 3 "" H 5100 3550 60  0001 C CNN
	1    5050 3600
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P2
U 1 1 58407563
P 4000 3650
F 0 "P2" H 4000 3800 50  0000 C CNN
F 1 "5V out" V 4100 3650 50  0000 C CNN
F 2 "robair:Bornier5mm" H 4000 3650 50  0001 C CNN
F 3 "" H 4000 3650 50  0000 C CNN
	1    4000 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	5800 3600 6200 3600
Wire Wire Line
	5800 3700 6100 3700
Wire Wire Line
	4400 3600 4200 3600
Wire Wire Line
	4400 3700 4200 3700
$Comp
L CONN_01X01 P1
U 1 1 58407645
P 4000 3100
F 0 "P1" H 4000 3200 50  0000 C CNN
F 1 "GND Ardui" V 4100 3100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 4000 3100 50  0001 C CNN
F 3 "" H 4000 3100 50  0000 C CNN
	1    4000 3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 3100 4300 3100
Wire Wire Line
	4300 3100 4300 3600
Connection ~ 4300 3600
Text Label 5850 3600 0    60   ~ 0
GND24
Text Label 4250 3600 0    60   ~ 0
GND5
NoConn ~ 5800 3500
NoConn ~ 4400 3500
$Comp
L CONN_01X02 P5
U 1 1 5841694E
P 8100 3400
F 0 "P5" H 8100 3550 50  0000 C CNN
F 1 "Batt" V 8200 3400 50  0000 C CNN
F 2 "robair:Bornier5mm" H 8100 3400 50  0001 C CNN
F 3 "" H 8100 3400 50  0000 C CNN
	1    8100 3400
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 58416998
P 8100 3000
F 0 "P4" H 8100 3150 50  0000 C CNN
F 1 "Charg" V 8200 3000 50  0000 C CNN
F 2 "robair:Bornier5mm" H 8100 3000 50  0001 C CNN
F 3 "" H 8100 3000 50  0000 C CNN
	1    8100 3000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P3
U 1 1 584169C6
P 5850 3150
F 0 "P3" H 5850 3300 50  0000 C CNN
F 1 "MD49" V 5950 3150 50  0000 C CNN
F 2 "robair:Bornier5mm" H 5850 3150 50  0001 C CNN
F 3 "" H 5850 3150 50  0000 C CNN
	1    5850 3150
	-1   0    0    1   
$EndComp
$Comp
L SW_PUSH_SMALL SW1
U 1 1 58416A61
P 7450 3350
F 0 "SW1" H 7600 3460 50  0000 C CNN
F 1 "SW" H 7450 3271 50  0000 C CNN
F 2 "robair:Bornier5mm" H 7450 3350 50  0001 C CNN
F 3 "" H 7450 3350 50  0000 C CNN
	1    7450 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2950 7800 2950
Wire Wire Line
	7800 2950 7800 3350
Wire Wire Line
	6050 3350 7900 3350
Wire Wire Line
	7900 3450 7900 3050
Wire Wire Line
	7900 3450 7550 3450
Wire Wire Line
	6200 3600 6200 3350
Connection ~ 7800 3350
Wire Wire Line
	6050 3350 6050 3200
Connection ~ 6200 3350
Wire Wire Line
	7350 3250 7350 3100
Wire Wire Line
	7350 3100 6050 3100
Wire Wire Line
	6100 3700 6100 3100
Connection ~ 6100 3100
$EndSCHEMATC
