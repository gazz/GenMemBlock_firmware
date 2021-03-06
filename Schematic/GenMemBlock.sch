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
L 74xx:74HCT595 U5
U 1 1 61AE46FD
P 1750 6850
F 0 "U5" H 1750 7600 50  0000 C CNN
F 1 "74HCT595" H 1750 7500 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 1750 6850 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT595.pdf" H 1750 6850 50  0001 C CNN
	1    1750 6850
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT595 U4
U 1 1 61AD5FED
P 1750 5250
F 0 "U4" H 1750 6031 50  0000 C CNN
F 1 "74HCT595" H 1750 5940 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 1750 5250 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT595.pdf" H 1750 5250 50  0001 C CNN
	1    1750 5250
	1    0    0    -1  
$EndComp
$Comp
L 2021-12-04_22-42-34:74ACT16543DL U10
U 1 1 61AD7B68
P 5750 1100
F 0 "U10" H 6600 1450 60  0000 C CNN
F 1 "74ACT16543DL" H 6600 1350 60  0000 C CNN
F 2 "ACT16543:74ACT16543DL" H 7150 1340 60  0001 C CNN
F 3 "" H 5750 1100 60  0000 C CNN
	1    5750 1100
	1    0    0    -1  
$EndComp
$Comp
L 2021-12-04_22-42-34:74ACT16543DL U8
U 1 1 61AF75BA
P 850 1100
F 0 "U8" H 1750 1450 60  0000 C CNN
F 1 "74ACT16543DL" H 1750 1350 60  0000 C CNN
F 2 "ACT16543:74ACT16543DL" H 2250 1340 60  0001 C CNN
F 3 "" H 850 1100 60  0000 C CNN
	1    850  1100
	1    0    0    -1  
$EndComp
Text Label 2600 1500 0    50   ~ 0
DI_0
Text Label 2600 1600 0    50   ~ 0
DI_1
Text Label 700  1500 0    50   ~ 0
D0
Text Label 700  1600 0    50   ~ 0
D1
Text Label 700  1800 0    50   ~ 0
D2
Text Label 700  1900 0    50   ~ 0
D3
Text Label 700  2000 0    50   ~ 0
D4
Text Label 700  2200 0    50   ~ 0
D5
Text Label 700  2300 0    50   ~ 0
D6
Text Label 700  2400 0    50   ~ 0
D7
Text Label 700  2500 0    50   ~ 0
D8
Text Label 700  2600 0    50   ~ 0
D9
Text Label 700  2700 0    50   ~ 0
D10
Text Label 700  2900 0    50   ~ 0
D11
Text Label 700  3000 0    50   ~ 0
D12
Text Label 700  3100 0    50   ~ 0
D13
Text Label 700  3300 0    50   ~ 0
D14
Text Label 700  3400 0    50   ~ 0
D15
Entry Wire Line
	2750 1700 2850 1600
Entry Wire Line
	2750 1800 2850 1700
Entry Wire Line
	2750 1900 2850 1800
Entry Wire Line
	2750 2000 2850 1900
Entry Wire Line
	2750 2200 2850 2100
Entry Wire Line
	2750 2300 2850 2200
Entry Wire Line
	2750 2400 2850 2300
Entry Wire Line
	2750 2500 2850 2400
Entry Wire Line
	2750 2600 2850 2500
Entry Wire Line
	2750 2700 2850 2600
Entry Wire Line
	2750 2900 2850 2800
Entry Wire Line
	2750 3000 2850 2900
Entry Wire Line
	2750 3100 2850 3000
Entry Wire Line
	2750 3200 2850 3100
Entry Wire Line
	2750 3300 2850 3200
Entry Wire Line
	2750 1600 2850 1500
Text Label 2600 1800 0    50   ~ 0
DI_2
Text Label 2600 1900 0    50   ~ 0
DI_3
Text Label 2600 2000 0    50   ~ 0
DI_4
Text Label 2600 2200 0    50   ~ 0
DI_5
Text Label 2600 2300 0    50   ~ 0
DI_6
Text Label 2600 2400 0    50   ~ 0
DI_7
Text Label 2600 2500 0    50   ~ 0
DI_8
Text Label 2600 2600 0    50   ~ 0
DI_9
Text Label 2600 2700 0    50   ~ 0
DI_10
Text Label 2600 2900 0    50   ~ 0
DI_11
Text Label 2600 3000 0    50   ~ 0
DI_12
Text Label 2600 3100 0    50   ~ 0
DI_13
Text Label 2600 3300 0    50   ~ 0
DI_14
Text Label 2600 3400 0    50   ~ 0
DI_15
Entry Wire Line
	600  1900 700  1800
Entry Wire Line
	600  2000 700  1900
Entry Wire Line
	600  2100 700  2000
Entry Wire Line
	600  2300 700  2200
Entry Wire Line
	600  2400 700  2300
Entry Wire Line
	600  2500 700  2400
Entry Wire Line
	600  2600 700  2500
Entry Wire Line
	600  2700 700  2600
Entry Wire Line
	600  2800 700  2700
Entry Wire Line
	600  3000 700  2900
Entry Wire Line
	600  3100 700  3000
Entry Wire Line
	600  3200 700  3100
Entry Wire Line
	600  3400 700  3300
$Comp
L 74xx:74HC165 U7
U 1 1 61ABE035
P 6900 5350
F 0 "U7" H 6900 6431 50  0000 C CNN
F 1 "74HC165" H 6900 6340 50  0000 C CNN
F 2 "Package_SO:SSOP-16_5.3x6.2mm_P0.65mm" H 6900 5350 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT165.pdf" H 6900 5350 50  0001 C CNN
	1    6900 5350
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC165 U6
U 1 1 61AC05DB
P 8250 5350
F 0 "U6" H 8250 6431 50  0000 C CNN
F 1 "74HC165" H 8250 6340 50  0000 C CNN
F 2 "Package_SO:SSOP-16_5.3x6.2mm_P0.65mm" H 8250 5350 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT165.pdf" H 8250 5350 50  0001 C CNN
	1    8250 5350
	1    0    0    -1  
$EndComp
$Comp
L 2021-12-04_22-42-34:74ACT16543DL U9
U 1 1 61AED935
P 3300 1100
F 0 "U9" H 4150 1450 60  0000 C CNN
F 1 "74ACT16543DL" H 4150 1350 60  0000 C CNN
F 2 "ACT16543:74ACT16543DL" H 4700 1340 60  0001 C CNN
F 3 "" H 3300 1100 60  0000 C CNN
	1    3300 1100
	1    0    0    -1  
$EndComp
Entry Wire Line
	600  1600 700  1500
Entry Wire Line
	600  1700 700  1600
Entry Wire Line
	600  3300 700  3200
Entry Wire Line
	600  2900 700  2800
Entry Wire Line
	600  1800 700  1700
Entry Wire Line
	600  2200 700  2100
Text Label 700  1700 0    50   ~ 0
VCC
Text Label 700  2100 0    50   ~ 0
GND
Text Label 700  2800 0    50   ~ 0
GND
Text Label 700  3200 0    50   ~ 0
VCC
Text Label 600  2500 3    50   ~ 0
DI[0..15],VCC,GND,OE
Entry Wire Line
	600  3500 700  3400
Entry Wire Line
	600  3600 700  3500
Entry Wire Line
	600  3700 700  3600
Entry Wire Line
	600  3800 700  3700
Entry Wire Line
	600  3900 700  3800
Text Label 700  3500 0    50   ~ 0
GND
Text Label 2600 3600 0    50   ~ 0
VCC
Text Label 700  3700 0    50   ~ 0
GND
Entry Wire Line
	600  1200 700  1100
Entry Wire Line
	600  1300 700  1200
Entry Wire Line
	600  1400 700  1300
Entry Wire Line
	600  1500 700  1400
Text Label 700  1400 0    50   ~ 0
GND
Text Label 700  1200 0    50   ~ 0
GND
Text Label 2600 1100 0    50   ~ 0
GND
Entry Wire Line
	2750 1100 2850 1000
Entry Wire Line
	2750 1200 2850 1100
Entry Wire Line
	2750 1300 2850 1200
Entry Wire Line
	2750 1400 2850 1300
Entry Wire Line
	2750 1500 2850 1400
Entry Wire Line
	2750 2100 2850 2000
Entry Wire Line
	2750 2800 2850 2700
Entry Wire Line
	2750 3400 2850 3300
Entry Wire Line
	2750 3500 2850 3400
Entry Wire Line
	2750 3600 2850 3500
Entry Wire Line
	2750 3700 2850 3600
Entry Wire Line
	2750 3800 2850 3700
Text Label 700  1100 0    50   ~ 0
DO_OE
Text Label 2600 1200 0    50   ~ 0
GND
Text Label 2600 1400 0    50   ~ 0
GND
Text Label 2600 1700 0    50   ~ 0
VCC
Text Label 2600 2100 0    50   ~ 0
GND
Text Label 2600 2800 0    50   ~ 0
GND
Text Label 2600 3200 0    50   ~ 0
VCC
Text Label 2600 3500 0    50   ~ 0
GND
Text Label 700  3600 0    50   ~ 0
GND
Text Label 2600 3700 0    50   ~ 0
GND
Text Label 2150 4850 0    50   ~ 0
D0
Text Label 2150 4950 0    50   ~ 0
D1
Text Label 2150 5050 0    50   ~ 0
D2
Text Label 2150 5150 0    50   ~ 0
D3
Text Label 2150 5250 0    50   ~ 0
D4
Text Label 2150 5350 0    50   ~ 0
D5
Text Label 2150 5450 0    50   ~ 0
D6
Text Label 2150 5550 0    50   ~ 0
D7
Text Label 2150 6450 0    50   ~ 0
D8
Text Label 2150 6550 0    50   ~ 0
D9
Text Label 2150 6650 0    50   ~ 0
D10
Text Label 2150 6750 0    50   ~ 0
D11
Text Label 2150 6850 0    50   ~ 0
D12
Text Label 2150 6950 0    50   ~ 0
D13
Text Label 2150 7050 0    50   ~ 0
D14
Text Label 2150 7150 0    50   ~ 0
D15
Entry Wire Line
	2300 4950 2400 4850
Entry Wire Line
	2300 5050 2400 4950
Entry Wire Line
	2300 5150 2400 5050
Entry Wire Line
	2300 5250 2400 5150
Entry Wire Line
	2300 5350 2400 5250
Entry Wire Line
	2300 5450 2400 5350
Entry Wire Line
	2300 5550 2400 5450
Entry Wire Line
	2300 6450 2400 6350
Entry Wire Line
	2300 6550 2400 6450
Entry Wire Line
	2300 6650 2400 6550
Entry Wire Line
	2300 6750 2400 6650
Entry Wire Line
	2300 6850 2400 6750
Entry Wire Line
	2300 6950 2400 6850
Entry Wire Line
	2300 7050 2400 6950
Entry Wire Line
	2300 7150 2400 7050
Entry Wire Line
	2300 6250 2400 6150
Text Label 2150 6250 0    50   ~ 0
VCC
Entry Wire Line
	2300 4850 2400 4750
Entry Wire Line
	700  4950 800  4850
Entry Wire Line
	700  5150 800  5050
Entry Wire Line
	700  5250 800  5150
Entry Wire Line
	700  5450 800  5350
Entry Wire Line
	700  5550 800  5450
Entry Wire Line
	700  6050 800  5950
Entry Wire Line
	700  6750 800  6650
Entry Wire Line
	700  6850 800  6750
Entry Wire Line
	700  7050 800  6950
Entry Wire Line
	700  7150 800  7050
Entry Wire Line
	800  7550 700  7450
Text Label 800  4850 0    50   ~ 0
QH3'
Text Label 800  5050 0    50   ~ 0
SER_CLK
Text Label 800  5150 0    50   ~ 0
VCC
Text Label 800  5350 0    50   ~ 0
595_LATCH#
Text Label 800  5450 0    50   ~ 0
595_DATA_OE#
Text Label 800  5950 0    50   ~ 0
GND
Text Label 800  6650 0    50   ~ 0
SER_CLK
Text Label 800  6750 0    50   ~ 0
VCC
Text Label 800  6950 0    50   ~ 0
595_LATCH#
Text Label 800  7050 0    50   ~ 0
595_DATA_OE#
$Comp
L 74xx:74HCT595 U2
U 1 1 61AE7838
P 4400 5850
F 0 "U2" H 4400 6631 50  0000 C CNN
F 1 "74HCT595" H 4400 6540 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 4400 5850 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT595.pdf" H 4400 5850 50  0001 C CNN
	1    4400 5850
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT595 U3
U 1 1 61AE5EDB
P 5200 6850
F 0 "U3" H 5200 7600 50  0000 C CNN
F 1 "74HCT595" H 5200 7500 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 5200 6850 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT595.pdf" H 5200 6850 50  0001 C CNN
	1    5200 6850
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT595 U1
U 1 1 61AE5432
P 3600 4850
F 0 "U1" H 3600 5600 50  0000 C CNN
F 1 "74HCT595" H 3600 5500 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 3600 4850 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT595.pdf" H 3600 4850 50  0001 C CNN
	1    3600 4850
	1    0    0    -1  
$EndComp
Text Label 5300 7550 0    50   ~ 0
QH3'
Entry Wire Line
	5200 7650 5300 7550
Text Label 3450 5650 0    50   ~ 0
SER_CLK
Text Label 3450 5750 0    50   ~ 0
VCC
Text Label 3450 5950 0    50   ~ 0
595_LATCH#
Text Label 3450 6050 0    50   ~ 0
595_ADDR_OE#
Entry Wire Line
	3350 5750 3450 5650
Entry Wire Line
	3350 5850 3450 5750
Entry Wire Line
	3350 6050 3450 5950
Entry Wire Line
	3350 6150 3450 6050
Text Label 4250 6650 0    50   ~ 0
SER_CLK
Text Label 4250 6750 0    50   ~ 0
VCC
Text Label 4250 6950 0    50   ~ 0
595_LATCH#
Text Label 4250 7050 0    50   ~ 0
595_ADDR_OE#
Entry Wire Line
	4150 6750 4250 6650
Entry Wire Line
	4150 6850 4250 6750
Entry Wire Line
	4150 7050 4250 6950
Entry Wire Line
	4150 7150 4250 7050
Entry Wire Line
	4800 7650 4900 7550
Entry Wire Line
	4000 6650 4100 6550
Entry Wire Line
	3200 5650 3300 5550
Text Label 3300 5550 0    50   ~ 0
GND
Text Label 4100 6550 0    50   ~ 0
GND
Text Label 4900 7550 0    50   ~ 0
GND
Text Label 1850 6050 0    50   ~ 0
QH4'
Text Label 4000 5350 0    50   ~ 0
QH1'
Text Label 4800 6350 0    50   ~ 0
QH2'
Text Label 2650 4450 0    50   ~ 0
SER_MOSI
Text Label 2650 4650 0    50   ~ 0
SER_CLK
Text Label 2650 4750 0    50   ~ 0
VCC
Text Label 2650 4950 0    50   ~ 0
595_LATCH#
Text Label 2650 5050 0    50   ~ 0
595_ADDR_OE#
Entry Wire Line
	2550 4550 2650 4450
Entry Wire Line
	2550 4750 2650 4650
Entry Wire Line
	2550 4850 2650 4750
Entry Wire Line
	2550 5050 2650 4950
Entry Wire Line
	2550 5150 2650 5050
Entry Wire Line
	2300 4650 2400 4550
Text Label 4000 4450 0    50   ~ 0
A0
Text Label 4000 4550 0    50   ~ 0
A1
Text Label 4000 4650 0    50   ~ 0
A2
Text Label 4000 4750 0    50   ~ 0
A3
Text Label 4000 4850 0    50   ~ 0
A4
Text Label 4000 4950 0    50   ~ 0
A5
Text Label 4000 5050 0    50   ~ 0
A6
Text Label 4000 5150 0    50   ~ 0
A7
Text Label 4800 5450 0    50   ~ 0
A8
Text Label 4800 5550 0    50   ~ 0
A9
Text Label 4800 5650 0    50   ~ 0
A10
Text Label 4800 5750 0    50   ~ 0
A11
Text Label 4800 5850 0    50   ~ 0
A12
Text Label 4800 5950 0    50   ~ 0
A13
Text Label 4800 6050 0    50   ~ 0
A14
Text Label 4800 6150 0    50   ~ 0
A15
Text Label 5600 6450 0    50   ~ 0
A16
Text Label 5600 6550 0    50   ~ 0
A17
Text Label 5600 6650 0    50   ~ 0
A18
Text Label 5600 6750 0    50   ~ 0
A19
Text Label 5600 6850 0    50   ~ 0
A20
Text Label 5600 6950 0    50   ~ 0
A21
Text Label 5600 7050 0    50   ~ 0
A22
Text Label 5600 7150 0    50   ~ 0
A23
Entry Wire Line
	4100 4450 4200 4350
Entry Wire Line
	4100 4550 4200 4450
Entry Wire Line
	4100 4650 4200 4550
Entry Wire Line
	4100 4750 4200 4650
Entry Wire Line
	4100 4850 4200 4750
Entry Wire Line
	4100 4950 4200 4850
Entry Wire Line
	4100 5050 4200 4950
Entry Wire Line
	4100 5150 4200 5050
Entry Wire Line
	4950 5450 5050 5350
Entry Wire Line
	4950 5550 5050 5450
Entry Wire Line
	4950 5650 5050 5550
Entry Wire Line
	4950 5750 5050 5650
Entry Wire Line
	4950 5850 5050 5750
Entry Wire Line
	4950 5950 5050 5850
Entry Wire Line
	4950 6050 5050 5950
Entry Wire Line
	4950 6150 5050 6050
Entry Wire Line
	5750 6450 5850 6350
Entry Wire Line
	5750 6550 5850 6450
Entry Wire Line
	5750 6650 5850 6550
Entry Wire Line
	5750 6750 5850 6650
Entry Wire Line
	5750 6850 5850 6750
Entry Wire Line
	5750 6950 5850 6850
Entry Wire Line
	5750 7050 5850 6950
Text Label 4800 5250 0    50   ~ 0
VCC
Text Label 5600 6250 0    50   ~ 0
VCC
Text Label 3950 4250 0    50   ~ 0
VCC
Entry Wire Line
	4100 4250 4200 4150
Entry Wire Line
	5750 7150 5850 7050
Entry Wire Line
	5750 6250 5850 6150
Entry Wire Line
	4950 5250 5050 5150
$Comp
L Device:R R2
U 1 1 62588156
P 3250 7400
F 0 "R2" V 3043 7400 50  0000 C CNN
F 1 "10K" V 3134 7400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3180 7400 50  0001 C CNN
F 3 "~" H 3250 7400 50  0001 C CNN
	1    3250 7400
	0    1    1    0   
$EndComp
Entry Wire Line
	4050 7400 4150 7300
Text Label 3400 7400 0    50   ~ 0
595_ADDR_OE#
Entry Wire Line
	4050 7050 4150 6950
Text Label 3400 7050 0    50   ~ 0
595_DATA_OE#
$Comp
L Device:R R1
U 1 1 625E16E2
P 3250 7050
F 0 "R1" V 3043 7050 50  0000 C CNN
F 1 "10K" V 3134 7050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3180 7050 50  0001 C CNN
F 3 "~" H 3250 7050 50  0001 C CNN
	1    3250 7050
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0105
U 1 1 625F2C5B
P 2900 7050
F 0 "#PWR0105" H 2900 6900 50  0001 C CNN
F 1 "+5V" H 2915 7223 50  0000 C CNN
F 2 "" H 2900 7050 50  0001 C CNN
F 3 "" H 2900 7050 50  0001 C CNN
	1    2900 7050
	1    0    0    -1  
$EndComp
Text Label 6050 5750 0    50   ~ 0
165_LD#
Entry Wire Line
	5950 4850 6050 4750
Entry Wire Line
	5950 4950 6050 4850
Entry Wire Line
	5950 5050 6050 4950
Entry Wire Line
	5950 5150 6050 5050
Entry Wire Line
	5950 5250 6050 5150
Entry Wire Line
	5950 5350 6050 5250
Entry Wire Line
	5950 5450 6050 5350
Entry Wire Line
	5950 5550 6050 5450
Entry Wire Line
	5950 5650 6050 5550
Entry Wire Line
	5950 5850 6050 5750
Entry Wire Line
	5950 6050 6050 5950
Entry Wire Line
	5950 6150 6050 6050
Text Label 6050 5950 0    50   ~ 0
SER_CLK
Text Label 6050 6050 0    50   ~ 0
GND
Text Label 6300 4850 0    50   ~ 0
D0
Text Label 6300 4950 0    50   ~ 0
D1
Text Label 6300 5050 0    50   ~ 0
D2
Text Label 6300 5150 0    50   ~ 0
D3
Text Label 6300 5250 0    50   ~ 0
D4
Text Label 6300 5350 0    50   ~ 0
D5
Text Label 6300 5450 0    50   ~ 0
D6
Text Label 6300 5550 0    50   ~ 0
D7
Text Label 7600 4850 0    50   ~ 0
D8
Text Label 7600 4950 0    50   ~ 0
D9
Text Label 7600 5050 0    50   ~ 0
D10
Text Label 7600 5150 0    50   ~ 0
D11
Text Label 7600 5250 0    50   ~ 0
D12
Text Label 7600 5350 0    50   ~ 0
D13
Text Label 7600 5450 0    50   ~ 0
D14
Text Label 7600 5550 0    50   ~ 0
D15
Entry Wire Line
	7500 4950 7600 4850
Entry Wire Line
	7500 5050 7600 4950
Entry Wire Line
	7500 5150 7600 5050
Entry Wire Line
	7500 5250 7600 5150
Entry Wire Line
	7500 5350 7600 5250
Entry Wire Line
	7500 5450 7600 5350
Entry Wire Line
	7500 5550 7600 5450
Entry Wire Line
	7500 5650 7600 5550
Text Label 7400 6050 0    50   ~ 0
GND
Entry Wire Line
	7300 5850 7400 5750
Entry Wire Line
	7300 6150 7400 6050
Entry Wire Line
	7300 6050 7400 5950
Text Label 7400 5750 0    50   ~ 0
165_LD#
Text Label 7400 5950 0    50   ~ 0
SER_CLK
Text Label 6750 6350 0    50   ~ 0
GND
Text Label 8000 6350 0    50   ~ 0
GND
Entry Wire Line
	6650 6450 6750 6350
Entry Wire Line
	7450 6450 7550 6350
Text GLabel 5950 6300 2    50   Input ~ 0
SER_BUS
Entry Wire Line
	7400 4350 7500 4250
Text Label 7150 4350 0    50   ~ 0
VCC
Entry Wire Line
	7500 4550 7600 4450
Text Label 7600 4450 0    50   ~ 0
VCC
Text Label 5050 1100 0    50   ~ 0
AI_OE
Text Label 3100 1200 0    50   ~ 0
GND
Text Label 5050 1300 0    50   ~ 0
GND
Text Label 3100 1400 0    50   ~ 0
GND
Text Label 5050 1500 0    50   ~ 0
AI_0
Text Label 5050 1600 0    50   ~ 0
AI_1
Text Label 5050 1800 0    50   ~ 0
AI_2
Text Label 5050 1900 0    50   ~ 0
AI_3
Text Label 5050 2200 0    50   ~ 0
AI_5
Text Label 5050 2300 0    50   ~ 0
AI_6
Text Label 5050 2400 0    50   ~ 0
AI_7
Text Label 5050 2500 0    50   ~ 0
AI_8
Text Label 5050 2600 0    50   ~ 0
AI_9
Text Label 5050 2700 0    50   ~ 0
AI_10
Text Label 5050 2900 0    50   ~ 0
AI_11
Text Label 5050 3000 0    50   ~ 0
AI_12
Text Label 5050 3100 0    50   ~ 0
AI_13
Text Label 5050 3300 0    50   ~ 0
AI_14
Text Label 5050 3400 0    50   ~ 0
AI_15
Text Label 5050 2000 0    50   ~ 0
AI_4
Text Label 3100 1700 0    50   ~ 0
VCC
Text Label 3100 2100 0    50   ~ 0
GND
Text Label 3100 2800 0    50   ~ 0
GND
Text Label 3100 3200 0    50   ~ 0
VCC
Text Label 3100 3500 0    50   ~ 0
GND
Text Label 5050 3600 0    50   ~ 0
GND
Text Label 3100 3700 0    50   ~ 0
GND
Text Label 5050 3800 0    50   ~ 0
AI_OE
Entry Wire Line
	3000 1200 3100 1100
Entry Wire Line
	3000 1300 3100 1200
Entry Wire Line
	3000 1400 3100 1300
Entry Wire Line
	3000 1500 3100 1400
Entry Wire Line
	3000 1600 3100 1500
Entry Wire Line
	3000 1700 3100 1600
Entry Wire Line
	3000 1800 3100 1700
Entry Wire Line
	3000 1900 3100 1800
Entry Wire Line
	3000 2000 3100 1900
Entry Wire Line
	3000 2100 3100 2000
Entry Wire Line
	3000 2200 3100 2100
Entry Wire Line
	3000 2300 3100 2200
Entry Wire Line
	3000 2400 3100 2300
Entry Wire Line
	3000 2500 3100 2400
Entry Wire Line
	3000 2600 3100 2500
Entry Wire Line
	3000 2700 3100 2600
Entry Wire Line
	3000 2800 3100 2700
Entry Wire Line
	3000 2900 3100 2800
Entry Wire Line
	3000 3000 3100 2900
Entry Wire Line
	3000 3100 3100 3000
Entry Wire Line
	3000 3200 3100 3100
Entry Wire Line
	3000 3300 3100 3200
Entry Wire Line
	3000 3400 3100 3300
Entry Wire Line
	3000 3500 3100 3400
Entry Wire Line
	3000 3600 3100 3500
Entry Wire Line
	3000 3700 3100 3600
Entry Wire Line
	3000 3800 3100 3700
Entry Wire Line
	3000 3900 3100 3800
Text Label 3100 1100 0    50   ~ 0
VCC
Text Label 5050 1200 0    50   ~ 0
GND
Text Label 3100 1300 0    50   ~ 0
VCC
Text Label 5050 1400 0    50   ~ 0
GND
Text Label 3100 1500 0    50   ~ 0
A0
Text Label 3100 1600 0    50   ~ 0
A1
Text Label 3100 1800 0    50   ~ 0
A2
Text Label 3100 1900 0    50   ~ 0
A3
Text Label 3100 2000 0    50   ~ 0
A4
Text Label 3100 2200 0    50   ~ 0
A5
Text Label 3100 2300 0    50   ~ 0
A6
Text Label 3100 2400 0    50   ~ 0
A7
Text Label 3100 2500 0    50   ~ 0
A8
Text Label 3100 2600 0    50   ~ 0
A9
Text Label 3100 2700 0    50   ~ 0
A10
Text Label 3100 2900 0    50   ~ 0
A11
Text Label 3100 3000 0    50   ~ 0
A12
Text Label 3100 3100 0    50   ~ 0
A13
Text Label 3100 3300 0    50   ~ 0
A14
Text Label 3100 3400 0    50   ~ 0
A15
Text Label 5050 1700 0    50   ~ 0
VCC
Text Label 5050 2100 0    50   ~ 0
GND
Text Label 5050 2800 0    50   ~ 0
GND
Text Label 5050 3200 0    50   ~ 0
VCC
Text Label 5050 3500 0    50   ~ 0
GND
Text Label 3100 3600 0    50   ~ 0
VCC
Text Label 5050 3700 0    50   ~ 0
GND
Text Label 3100 3800 0    50   ~ 0
VCC
Entry Wire Line
	5250 1100 5350 1000
Entry Wire Line
	5250 1200 5350 1100
Entry Wire Line
	5250 1300 5350 1200
Entry Wire Line
	5250 1400 5350 1300
Entry Wire Line
	5250 1500 5350 1400
Entry Wire Line
	5250 1600 5350 1500
Entry Wire Line
	5250 1700 5350 1600
Entry Wire Line
	5250 1800 5350 1700
Entry Wire Line
	5250 1900 5350 1800
Entry Wire Line
	5250 2000 5350 1900
Entry Wire Line
	5250 2100 5350 2000
Entry Wire Line
	5250 2200 5350 2100
Entry Wire Line
	5250 2300 5350 2200
Entry Wire Line
	5250 2400 5350 2300
Entry Wire Line
	5250 2500 5350 2400
Entry Wire Line
	5250 2600 5350 2500
Entry Wire Line
	5250 2700 5350 2600
Entry Wire Line
	5250 2800 5350 2700
Entry Wire Line
	5250 2900 5350 2800
Entry Wire Line
	5250 3000 5350 2900
Entry Wire Line
	5250 3100 5350 3000
Entry Wire Line
	5250 3200 5350 3100
Entry Wire Line
	5250 3300 5350 3200
Entry Wire Line
	5250 3400 5350 3300
Entry Wire Line
	5250 3500 5350 3400
Entry Wire Line
	5250 3600 5350 3500
Entry Wire Line
	5250 3700 5350 3600
Entry Wire Line
	5250 3800 5350 3700
Text Label 7500 1100 0    50   ~ 0
AI_OE
Text Label 5550 1200 0    50   ~ 0
GND
Text Label 7500 1300 0    50   ~ 0
GND
Text Label 5550 1400 0    50   ~ 0
GND
Text Label 7500 1500 0    50   ~ 0
AI_16
Text Label 7500 1600 0    50   ~ 0
AI_17
Text Label 7500 1800 0    50   ~ 0
AI_18
Text Label 7500 1900 0    50   ~ 0
AI_19
Text Label 7500 2000 0    50   ~ 0
AI_20
Text Label 7500 2200 0    50   ~ 0
AI_21
Text Label 7500 2300 0    50   ~ 0
AI_22
Text Label 7500 2400 0    50   ~ 0
AI_23
Text Label 5550 1700 0    50   ~ 0
VCC
Text Label 5550 2100 0    50   ~ 0
GND
Text Label 7500 2600 0    50   ~ 0
GEN_OE#
Text Label 5550 2800 0    50   ~ 0
GND
Text Label 5550 3200 0    50   ~ 0
VCC
Text Label 5550 3500 0    50   ~ 0
GND
Text Label 7500 3600 0    50   ~ 0
GND
Text Label 5550 3700 0    50   ~ 0
GND
Text Label 7500 3800 0    50   ~ 0
CTRL_OE
Entry Wire Line
	5450 3900 5550 3800
Entry Wire Line
	5450 3800 5550 3700
Entry Wire Line
	5450 3700 5550 3600
Entry Wire Line
	5450 3600 5550 3500
Entry Wire Line
	5450 3300 5550 3200
Entry Wire Line
	5450 2900 5550 2800
Entry Wire Line
	5450 2800 5550 2700
Entry Wire Line
	5450 2700 5550 2600
Entry Wire Line
	5450 2600 5550 2500
Entry Wire Line
	5450 2500 5550 2400
Entry Wire Line
	5450 2400 5550 2300
Entry Wire Line
	5450 2300 5550 2200
Entry Wire Line
	5450 2200 5550 2100
Entry Wire Line
	5450 2100 5550 2000
Entry Wire Line
	5450 2000 5550 1900
Entry Wire Line
	5450 1900 5550 1800
Entry Wire Line
	5450 1800 5550 1700
Entry Wire Line
	5450 1700 5550 1600
Entry Wire Line
	5450 1600 5550 1500
Entry Wire Line
	5450 1500 5550 1400
Entry Wire Line
	5450 1400 5550 1300
Entry Wire Line
	5450 1300 5550 1200
Entry Wire Line
	5450 1200 5550 1100
Text Label 7500 3500 0    50   ~ 0
GND
Text Label 5550 3600 0    50   ~ 0
VCC
Text Label 7500 3700 0    50   ~ 0
GND
Text Label 5550 3800 0    50   ~ 0
VCC
Text Label 5550 1100 0    50   ~ 0
VCC
Text Label 7500 1200 0    50   ~ 0
GND
Text Label 5550 1300 0    50   ~ 0
VCC
Text Label 7500 1700 0    50   ~ 0
VCC
Text Label 7500 2100 0    50   ~ 0
GND
Text Label 7500 2800 0    50   ~ 0
GND
Text Label 7500 1400 0    50   ~ 0
GND
Text Label 5550 1500 0    50   ~ 0
A16
Text Label 5550 1600 0    50   ~ 0
A17
Text Label 5550 1800 0    50   ~ 0
A18
Text Label 5550 1900 0    50   ~ 0
A19
Text Label 5550 2000 0    50   ~ 0
A20
Text Label 5550 2200 0    50   ~ 0
A21
Text Label 5550 2300 0    50   ~ 0
A22
Text Label 5550 2400 0    50   ~ 0
A23
Text Label 5550 2500 0    50   ~ 0
ROM_CE#
Text Label 5550 2600 0    50   ~ 0
ROM_OE#
Text Label 5550 2700 0    50   ~ 0
ROM_WE#
NoConn ~ 7500 2900
NoConn ~ 7500 3000
NoConn ~ 5750 2900
NoConn ~ 5750 3000
NoConn ~ 5750 3100
NoConn ~ 5750 3300
NoConn ~ 5750 3400
NoConn ~ 7500 3300
NoConn ~ 7400 4850
NoConn ~ 8750 4850
NoConn ~ 2150 7350
$Comp
L power:GND #PWR0102
U 1 1 648FBC66
P 3050 6500
F 0 "#PWR0102" H 3050 6250 50  0001 C CNN
F 1 "GND" H 3055 6327 50  0000 C CNN
F 2 "" H 3050 6500 50  0001 C CNN
F 3 "" H 3050 6500 50  0001 C CNN
	1    3050 6500
	1    0    0    -1  
$EndComp
Text Label 2750 6500 0    50   ~ 0
GND
Entry Wire Line
	3250 6500 3350 6400
Entry Wire Line
	3250 6300 3350 6200
Text Label 2750 6300 0    50   ~ 0
VCC
$Comp
L power:+5V #PWR0103
U 1 1 649762F5
P 3050 6300
F 0 "#PWR0103" H 3050 6150 50  0001 C CNN
F 1 "+5V" H 3065 6473 50  0000 C CNN
F 2 "" H 3050 6300 50  0001 C CNN
F 3 "" H 3050 6300 50  0001 C CNN
	1    3050 6300
	1    0    0    -1  
$EndComp
Entry Wire Line
	2400 6400 2500 6300
Entry Wire Line
	2400 6600 2500 6500
$Comp
L Connector_Generic:Conn_02x10_Odd_Even J4
U 1 1 64C21677
P 8750 2700
F 0 "J4" V 8800 2100 50  0000 R CNN
F 1 "CART_A20" V 8800 2850 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x10_P2.54mm_Vertical" H 8750 2700 50  0001 C CNN
F 3 "~" H 8750 2700 50  0001 C CNN
	1    8750 2700
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J7
U 1 1 64C23BBF
P 10600 2700
F 0 "J7" V 10650 2400 50  0000 R CNN
F 1 "CART_A10" V 10650 2850 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 10600 2700 50  0001 C CNN
F 3 "~" H 10600 2700 50  0001 C CNN
	1    10600 2700
	0    -1   -1   0   
$EndComp
Entry Wire Line
	9150 5650 9250 5550
Entry Wire Line
	9150 5750 9250 5650
Entry Wire Line
	9150 6250 9250 6150
Entry Wire Line
	9150 6450 9250 6350
Entry Wire Line
	10950 5450 11050 5350
Entry Wire Line
	10950 5650 11050 5550
Entry Wire Line
	10950 6250 11050 6150
Text Label 10550 5450 0    50   ~ 0
SER_MOSI
NoConn ~ 10250 5550
Text Label 10350 5650 0    50   ~ 0
595_LATCH#
Text GLabel 10250 5750 2    50   Input ~ 0
ROM_CE#
NoConn ~ 10250 5850
Text GLabel 10250 6050 2    50   Input ~ 0
GEN_RESET#
Text GLabel 10250 6150 2    50   Input ~ 0
ROM_OE#
Text Label 10650 6250 0    50   ~ 0
165_LD#
$Comp
L power:+5V #PWR0108
U 1 1 651E9970
P 10300 6350
F 0 "#PWR0108" H 10300 6200 50  0001 C CNN
F 1 "+5V" V 10315 6478 50  0000 L CNN
F 2 "" H 10300 6350 50  0001 C CNN
F 3 "" H 10300 6350 50  0001 C CNN
	1    10300 6350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 6515AB13
P 10300 4800
F 0 "#PWR0107" H 10300 4550 50  0001 C CNN
F 1 "GND" H 10400 4650 50  0000 R CNN
F 2 "" H 10300 4800 50  0001 C CNN
F 3 "" H 10300 4800 50  0001 C CNN
	1    10300 4800
	1    0    0    -1  
$EndComp
Text GLabel 9750 5850 0    50   Input ~ 0
TX_ENABLE#
Text GLabel 9750 5750 0    50   Input ~ 0
ROM_WE#
Text Label 9250 6350 0    50   ~ 0
SER_MISO
NoConn ~ 9750 6250
Text Label 9250 6150 0    50   ~ 0
595_DATA_OE#
NoConn ~ 10250 5950
NoConn ~ 9750 6050
NoConn ~ 9750 5950
Text Label 9250 5650 0    50   ~ 0
595_ADDR_OE#
Text Label 9250 5550 0    50   ~ 0
SER_CLK
$Comp
L Connector_Generic:Conn_02x10_Odd_Even J1
U 1 1 64BEF784
P 9950 5850
F 0 "J1" H 10000 6467 50  0000 C CNN
F 1 "MCU_HEADER" H 10000 6376 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x10_P2.54mm_Vertical" H 9950 5850 50  0001 C CNN
F 3 "~" H 9950 5850 50  0001 C CNN
	1    9950 5850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 64A46511
P 5500 5300
F 0 "#PWR0106" H 5500 5150 50  0001 C CNN
F 1 "+5V" H 5515 5473 50  0000 C CNN
F 2 "" H 5500 5300 50  0001 C CNN
F 3 "" H 5500 5300 50  0001 C CNN
	1    5500 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 64A46509
P 5500 5400
F 0 "#PWR0104" H 5500 5150 50  0001 C CNN
F 1 "GND" H 5505 5227 50  0000 C CNN
F 2 "" H 5500 5400 50  0001 C CNN
F 3 "" H 5500 5400 50  0001 C CNN
	1    5500 5400
	1    0    0    -1  
$EndComp
Entry Wire Line
	7800 3800 7900 3700
Entry Wire Line
	7800 3700 7900 3600
Entry Wire Line
	7800 3600 7900 3500
Entry Wire Line
	7800 3500 7900 3400
Entry Wire Line
	7800 3200 7900 3100
Entry Wire Line
	7800 3100 7900 3000
Entry Wire Line
	7800 2800 7900 2700
Entry Wire Line
	7800 2700 7900 2600
Entry Wire Line
	7800 2600 7900 2500
Entry Wire Line
	7800 2500 7900 2400
Entry Wire Line
	7800 2400 7900 2300
Entry Wire Line
	7800 2300 7900 2200
Entry Wire Line
	7800 2200 7900 2100
Entry Wire Line
	7800 2100 7900 2000
Entry Wire Line
	7800 2000 7900 1900
Entry Wire Line
	7800 1900 7900 1800
Entry Wire Line
	7800 1800 7900 1700
Entry Wire Line
	7800 1700 7900 1600
Entry Wire Line
	7800 1600 7900 1500
Entry Wire Line
	7800 1500 7900 1400
Entry Wire Line
	7800 1400 7900 1300
Entry Wire Line
	7800 1300 7900 1200
Entry Wire Line
	7800 1200 7900 1100
Entry Wire Line
	7800 1100 7900 1000
Entry Wire Line
	5050 5400 5150 5300
Entry Wire Line
	5850 5300 5950 5200
Entry Wire Line
	5850 5400 5950 5300
Entry Wire Line
	5050 5500 5150 5400
Text Label 5600 5300 0    50   ~ 0
VCC
Text Label 5600 5400 0    50   ~ 0
GND
Text Label 7500 3200 0    50   ~ 0
VCC
$Comp
L power:+5V #PWR0109
U 1 1 65CCA2A8
P 9750 1050
F 0 "#PWR0109" H 9750 900 50  0001 C CNN
F 1 "+5V" H 9650 1150 50  0000 C CNN
F 2 "" H 9750 1050 50  0001 C CNN
F 3 "" H 9750 1050 50  0001 C CNN
	1    9750 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 65CCAEC6
P 10650 1050
F 0 "#PWR0110" H 10650 800 50  0001 C CNN
F 1 "GND" H 10550 1050 50  0000 C CNN
F 2 "" H 10650 1050 50  0001 C CNN
F 3 "" H 10650 1050 50  0001 C CNN
	1    10650 1050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 65CCC08A
P 10650 1550
F 0 "#PWR0111" H 10650 1300 50  0001 C CNN
F 1 "GND" H 10750 1550 50  0000 C CNN
F 2 "" H 10650 1550 50  0001 C CNN
F 3 "" H 10650 1550 50  0001 C CNN
	1    10650 1550
	1    0    0    -1  
$EndComp
Text Label 10550 1050 1    50   ~ 0
D0
Text Label 10550 1650 1    50   ~ 0
D1
Text Label 10450 1050 1    50   ~ 0
D2
Text Label 10450 1650 1    50   ~ 0
D3
Text Label 10350 1050 1    50   ~ 0
D4
Text Label 10350 1650 1    50   ~ 0
D5
Text Label 10250 1050 1    50   ~ 0
D6
Text Label 10250 1650 1    50   ~ 0
D7
Text Label 10150 1050 1    50   ~ 0
D8
Text Label 10150 1650 1    50   ~ 0
D9
Text Label 10050 1050 1    50   ~ 0
D10
Text Label 10050 1650 1    50   ~ 0
D11
Text Label 9950 1050 1    50   ~ 0
D12
Text Label 9950 1650 1    50   ~ 0
D13
Text Label 9850 1050 1    50   ~ 0
D14
Text Label 9850 1650 1    50   ~ 0
D15
Entry Wire Line
	9750 1750 9850 1650
Entry Wire Line
	9850 1750 9950 1650
Entry Wire Line
	9950 1750 10050 1650
Entry Wire Line
	10050 1750 10150 1650
Entry Wire Line
	10150 1750 10250 1650
Entry Wire Line
	10250 1750 10350 1650
Entry Wire Line
	10350 1750 10450 1650
Entry Wire Line
	10450 1750 10550 1650
Entry Wire Line
	9850 900  9950 800 
Entry Wire Line
	9950 900  10050 800 
Entry Wire Line
	10050 900  10150 800 
Entry Wire Line
	10150 900  10250 800 
Entry Wire Line
	10250 900  10350 800 
Entry Wire Line
	10350 900  10450 800 
Entry Wire Line
	10450 900  10550 800 
Entry Wire Line
	10550 900  10650 800 
$Comp
L Connector_Generic:Conn_02x13_Odd_Even J2
U 1 1 64C1FCE5
P 8800 1350
F 0 "J2" V 8850 650 50  0000 R CNN
F 1 "ADDR_BUS" V 8850 1550 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x13_P2.54mm_Vertical" H 8800 1350 50  0001 C CNN
F 3 "~" H 8800 1350 50  0001 C CNN
	1    8800 1350
	0    -1   -1   0   
$EndComp
Text Label 9400 1050 1    50   ~ 0
A0
Text Label 9400 1700 1    50   ~ 0
A1
Text Label 9300 1050 1    50   ~ 0
A2
Text Label 9300 1700 1    50   ~ 0
A3
Text Label 9200 1050 1    50   ~ 0
A4
Text Label 9200 1700 1    50   ~ 0
A5
Text Label 9100 1050 1    50   ~ 0
A6
Text Label 9100 1700 1    50   ~ 0
A7
Text Label 9000 1050 1    50   ~ 0
A8
Text Label 9000 1700 1    50   ~ 0
A9
Text Label 8900 1050 1    50   ~ 0
A10
Text Label 8900 1700 1    50   ~ 0
A11
Text Label 8800 1050 1    50   ~ 0
A12
Text Label 8800 1700 1    50   ~ 0
A13
Text Label 8700 1050 1    50   ~ 0
A14
Text Label 8700 1700 1    50   ~ 0
A15
Text Label 8600 1050 1    50   ~ 0
A16
Text Label 8600 1700 1    50   ~ 0
A17
Text Label 8500 1050 1    50   ~ 0
A18
Text Label 8500 1700 1    50   ~ 0
A19
Text Label 8400 1050 1    50   ~ 0
A20
Text Label 8400 1700 1    50   ~ 0
A21
Text Label 8200 1900 1    50   ~ 0
ROM_WE#
Text Label 8300 1050 1    50   ~ 0
ROM_CE#
Text Label 8200 1050 1    50   ~ 0
ROM_OE#
Entry Wire Line
	8200 750  8300 650 
Entry Wire Line
	8300 750  8400 650 
Entry Wire Line
	8400 750  8500 650 
Entry Wire Line
	8500 750  8600 650 
Entry Wire Line
	8600 750  8700 650 
Entry Wire Line
	8700 750  8800 650 
Entry Wire Line
	8800 750  8900 650 
Entry Wire Line
	8900 750  9000 650 
Entry Wire Line
	9000 750  9100 650 
Entry Wire Line
	9100 750  9200 650 
Entry Wire Line
	9200 750  9300 650 
Entry Wire Line
	9300 750  9400 650 
Entry Wire Line
	9400 750  9500 650 
Entry Wire Line
	8200 2000 8300 1900
Entry Wire Line
	8300 2000 8400 1900
Entry Wire Line
	8400 2000 8500 1900
Entry Wire Line
	8500 2000 8600 1900
Entry Wire Line
	8600 2000 8700 1900
Entry Wire Line
	8700 2000 8800 1900
Entry Wire Line
	8800 2000 8900 1900
Entry Wire Line
	8900 2000 9000 1900
Entry Wire Line
	9000 2000 9100 1900
Entry Wire Line
	9100 2000 9200 1900
Entry Wire Line
	9200 2000 9300 1900
Entry Wire Line
	9300 2000 9400 1900
Entry Wire Line
	8100 2000 8200 1900
NoConn ~ 7500 3400
NoConn ~ -1350 800 
Text Label 8350 3100 1    50   ~ 0
VCC
Text Label 8450 3100 1    50   ~ 0
AI_10
Text Label 8550 3100 1    50   ~ 0
AI_11
Text Label 8650 3100 1    50   ~ 0
AI_12
Text Label 8750 3100 1    50   ~ 0
AI_13
Text Label 8850 3100 1    50   ~ 0
AI_14
Text Label 8950 3100 1    50   ~ 0
AI_15
Text Label 9050 3100 1    50   ~ 0
AI_16
Text Label 9150 3100 1    50   ~ 0
GND
Text Label 9250 3100 1    50   ~ 0
DI_0
Text Label 8350 2400 1    50   ~ 0
AI_7
Text Label 8450 2400 1    50   ~ 0
AI_6
Text Label 8550 2400 1    50   ~ 0
AI_5
Text Label 8650 2400 1    50   ~ 0
AI_4
Text Label 8750 2400 1    50   ~ 0
AI_3
Text Label 8850 2400 1    50   ~ 0
AI_2
Text Label 8950 2400 1    50   ~ 0
AI_1
Text Label 9050 2400 1    50   ~ 0
AI_0
Text Label 9150 2400 1    50   ~ 0
DI_7
Text Label 9250 2400 1    50   ~ 0
DI_8
Entry Wire Line
	8350 2250 8450 2150
Entry Wire Line
	8450 2250 8550 2150
Entry Wire Line
	8550 2250 8650 2150
Entry Wire Line
	8650 2250 8750 2150
Entry Wire Line
	8750 2250 8850 2150
Entry Wire Line
	8850 2250 8950 2150
Entry Wire Line
	8950 2250 9050 2150
Entry Wire Line
	9050 2250 9150 2150
Entry Wire Line
	9150 2250 9250 2150
Entry Wire Line
	9250 2250 9350 2150
Entry Wire Line
	8250 3200 8350 3100
Entry Wire Line
	8350 3200 8450 3100
Entry Wire Line
	8450 3200 8550 3100
Entry Wire Line
	8550 3200 8650 3100
Entry Wire Line
	8650 3200 8750 3100
Entry Wire Line
	8750 3200 8850 3100
Entry Wire Line
	8850 3200 8950 3100
Entry Wire Line
	8950 3200 9050 3100
Entry Wire Line
	9050 3200 9150 3100
Entry Wire Line
	9150 3200 9250 3100
NoConn ~ 9600 2400
Text Label 9700 2400 1    50   ~ 0
AI_9
Text Label 9800 2400 1    50   ~ 0
AI_18
Text Label 9900 2400 1    50   ~ 0
AI_20
Text Label 10000 2400 1    50   ~ 0
AI_22
Text Label 9700 3100 1    50   ~ 0
AI_8
Text Label 9800 3100 1    50   ~ 0
AI_17
Text Label 9900 3100 1    50   ~ 0
AI_19
Text Label 10000 3100 1    50   ~ 0
AI_21
Entry Wire Line
	9700 2250 9800 2150
Entry Wire Line
	9800 2250 9900 2150
Entry Wire Line
	9900 2250 10000 2150
Entry Wire Line
	10000 2250 10100 2150
Entry Wire Line
	9600 3200 9700 3100
Entry Wire Line
	9700 3200 9800 3100
Entry Wire Line
	9800 3200 9900 3100
Entry Wire Line
	9900 3200 10000 3100
Entry Wire Line
	9150 3200 9250 3100
Text Label 10400 2400 1    50   ~ 0
DI_1
Text Label 10500 2400 1    50   ~ 0
DI_5
Text Label 10600 2400 1    50   ~ 0
DI_10
Text Label 10700 2400 1    50   ~ 0
DI_3
Text Label 10800 2400 1    50   ~ 0
VCC
Text Label 10400 3100 1    50   ~ 0
DI_6
Text Label 10500 3100 1    50   ~ 0
DI_9
Text Label 10600 3100 1    50   ~ 0
DI_2
Text Label 10700 3100 1    50   ~ 0
DI_4
Text Label 10800 3100 1    50   ~ 0
DI_11
Entry Wire Line
	10400 2250 10500 2150
Entry Wire Line
	10500 2250 10600 2150
Entry Wire Line
	10600 2250 10700 2150
Entry Wire Line
	10700 2250 10800 2150
Entry Wire Line
	10800 2250 10900 2150
Entry Wire Line
	10700 3200 10800 3100
Entry Wire Line
	10600 3200 10700 3100
Entry Wire Line
	10500 3200 10600 3100
Entry Wire Line
	10400 3200 10500 3100
Entry Wire Line
	10300 3200 10400 3100
NoConn ~ 9950 3650
NoConn ~ 10050 3650
NoConn ~ 9950 4150
NoConn ~ 10050 4150
NoConn ~ 10250 3650
NoConn ~ 10350 3650
NoConn ~ 10250 4150
Text Label 10150 3650 1    50   ~ 0
GEN_CE#
Text Label 7500 2500 0    50   ~ 0
GEN_CE#
Text Label 10450 3650 1    50   ~ 0
DI_14
Text Label 10550 3650 1    50   ~ 0
DI_12
Text Label 10450 4500 1    50   ~ 0
DI_15
Text Label 10550 4500 1    50   ~ 0
DI_13
Text Label 10150 4500 1    50   ~ 0
GEN_OE#
NoConn ~ 10650 4150
NoConn ~ 10750 4150
NoConn ~ 10750 3650
NoConn ~ 10850 3650
Entry Wire Line
	10150 3300 10250 3200
Entry Wire Line
	10450 3300 10550 3200
Entry Wire Line
	10550 3300 10650 3200
Entry Wire Line
	10050 4600 10150 4500
Entry Wire Line
	10350 4600 10450 4500
Entry Wire Line
	10450 4600 10550 4500
$Comp
L power:+5V #PWR0112
U 1 1 682C1FA1
P 9550 2000
F 0 "#PWR0112" H 9550 1850 50  0001 C CNN
F 1 "+5V" H 9565 2173 50  0000 C CNN
F 2 "" H 9550 2000 50  0001 C CNN
F 3 "" H 9550 2000 50  0001 C CNN
	1    9550 2000
	1    0    0    -1  
$EndComp
Text Label 9550 2100 0    50   ~ 0
VCC
Text Label 10300 4800 0    50   ~ 0
GND
$Comp
L power:GND #PWR0113
U 1 1 6840DAD3
P 9700 5450
F 0 "#PWR0113" H 9700 5200 50  0001 C CNN
F 1 "GND" V 9650 5350 50  0000 R CNN
F 2 "" H 9700 5450 50  0001 C CNN
F 3 "" H 9700 5450 50  0001 C CNN
	1    9700 5450
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 61B89206
P 8100 3650
F 0 "C1" H 8192 3696 50  0000 L CNN
F 1 "0.1pf" H 8192 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8100 3650 50  0001 C CNN
F 3 "~" H 8100 3650 50  0001 C CNN
	1    8100 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 61D09ADC
P 8400 3650
F 0 "C2" H 8492 3696 50  0000 L CNN
F 1 "0.1pf" H 8492 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8400 3650 50  0001 C CNN
F 3 "~" H 8400 3650 50  0001 C CNN
	1    8400 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 61D09F86
P 8700 3650
F 0 "C3" H 8792 3696 50  0000 L CNN
F 1 "0.1pf" H 8792 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8700 3650 50  0001 C CNN
F 3 "~" H 8700 3650 50  0001 C CNN
	1    8700 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 61D0A462
P 9300 3650
F 0 "C5" H 9392 3696 50  0000 L CNN
F 1 "0.1pf" H 9392 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9300 3650 50  0001 C CNN
F 3 "~" H 9300 3650 50  0001 C CNN
	1    9300 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 61D42A97
P 8300 3850
F 0 "C7" H 8392 3896 50  0000 L CNN
F 1 "0.1pf" H 8392 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8300 3850 50  0001 C CNN
F 3 "~" H 8300 3850 50  0001 C CNN
	1    8300 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 61D432E5
P 8650 3850
F 0 "C8" H 8742 3896 50  0000 L CNN
F 1 "0.1pf" H 8742 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8650 3850 50  0001 C CNN
F 3 "~" H 8650 3850 50  0001 C CNN
	1    8650 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 61D4350D
P 9100 3850
F 0 "C9" H 9192 3896 50  0000 L CNN
F 1 "0.1pf" H 9192 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9100 3850 50  0001 C CNN
F 3 "~" H 9100 3850 50  0001 C CNN
	1    9100 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 61D43B0B
P 9450 3850
F 0 "C10" H 9542 3896 50  0000 L CNN
F 1 "0.1pf" H 9542 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9450 3850 50  0001 C CNN
F 3 "~" H 9450 3850 50  0001 C CNN
	1    9450 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0115
U 1 1 61E9C109
P 9700 3750
F 0 "#PWR0115" H 9700 3600 50  0001 C CNN
F 1 "+5V" V 9715 3878 50  0000 L CNN
F 2 "" H 9700 3750 50  0001 C CNN
F 3 "" H 9700 3750 50  0001 C CNN
	1    9700 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 61FABEC9
P 8100 3850
F 0 "C6" H 7900 3900 50  0000 L CNN
F 1 "0.1pf" H 7800 3800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8100 3850 50  0001 C CNN
F 3 "~" H 8100 3850 50  0001 C CNN
	1    8100 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 61FB855B
P 9050 3650
F 0 "C4" H 9142 3696 50  0000 L CNN
F 1 "0.1pf" H 9142 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9050 3650 50  0001 C CNN
F 3 "~" H 9050 3650 50  0001 C CNN
	1    9050 3650
	1    0    0    -1  
$EndComp
Text Label 1850 4650 0    50   ~ 0
VCC
Text Label 9350 4350 0    50   ~ 0
AI_OE
Text Label 9350 4450 0    50   ~ 0
CTRL_OE
Text Label 9350 4250 0    50   ~ 0
DO_OE
Text GLabel 9300 4450 0    50   Input ~ 0
TX_ENABLE#
Text Label 2600 3800 0    50   ~ 0
GND
Text Label 700  3800 0    50   ~ 0
DO_OE
Text Label 700  1300 0    50   ~ 0
GND
Text Label 2600 1300 0    50   ~ 0
VCC
$Comp
L Connector_Generic:Conn_02x10_Odd_Even J6
U 1 1 64C2291D
P 10350 3950
F 0 "J6" V 10400 4450 50  0000 L CNN
F 1 "CART_B20" V 10400 3800 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x10_P2.54mm_Vertical" H 10350 3950 50  0001 C CNN
F 3 "~" H 10350 3950 50  0001 C CNN
	1    10350 3950
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_02x10_Odd_Even J3
U 1 1 64BF0EFF
P 10150 1350
F 0 "J3" V 10200 750 50  0000 R CNN
F 1 "DATA_BUS" V 10200 1500 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x10_P2.54mm_Vertical" H 10150 1350 50  0001 C CNN
F 3 "~" H 10150 1350 50  0001 C CNN
	1    10150 1350
	0    -1   -1   0   
$EndComp
Text Label 7400 4750 1    50   ~ 0
SER_MISO
Text Label 1450 7550 0    50   ~ 0
GND
Entry Wire Line
	7400 4750 7500 4850
Entry Wire Line
	7500 4650 7600 4750
Text Label 8750 4750 0    50   ~ 0
QH_165
Text Label 6100 4750 0    50   ~ 0
QH_165
Text Label 7600 4750 0    50   ~ 0
GND
Text Label 7500 2700 0    50   ~ 0
VCC
$Comp
L power:GND #PWR0114
U 1 1 61D44FC6
P 8900 3950
F 0 "#PWR0114" H 8900 3700 50  0001 C CNN
F 1 "GND" H 8950 3800 50  0000 R CNN
F 2 "" H 8900 3950 50  0001 C CNN
F 3 "" H 8900 3950 50  0001 C CNN
	1    8900 3950
	1    0    0    -1  
$EndComp
Text Label 8650 650  0    50   ~ 0
ADDR_BUS
Text Label 10750 800  0    50   ~ 0
DATA_BUS
Text Label 7550 650  0    50   ~ 0
CART_BUS
Text Label 4650 5050 0    50   ~ 0
ADDR_BUS
Text Label 7150 4200 0    50   ~ 0
DATA_BUS
Text Label 10700 5150 0    50   ~ 0
SER_BUS
Text Label 2400 7650 0    50   ~ 0
SER_BUS
Text Label 3000 950  0    50   ~ 0
ADDR_BUS
Text Label 3000 750  0    50   ~ 0
CART_BUS
Text Label 600  2150 1    50   ~ 0
DATA_BUS
Wire Wire Line
	850  1500 700  1500
Wire Wire Line
	850  1600 700  1600
Wire Wire Line
	700  3200 850  3200
Wire Wire Line
	700  2800 850  2800
Wire Wire Line
	700  2100 850  2100
Wire Wire Line
	2600 3200 2750 3200
Wire Wire Line
	2600 3100 2750 3100
Wire Wire Line
	2600 3000 2750 3000
Wire Wire Line
	2600 2900 2750 2900
Wire Wire Line
	2600 2700 2750 2700
Wire Wire Line
	2600 2600 2750 2600
Wire Wire Line
	2600 2500 2750 2500
Wire Wire Line
	2600 2400 2750 2400
Wire Wire Line
	2600 2300 2750 2300
Wire Wire Line
	2600 2200 2750 2200
Wire Wire Line
	2600 2000 2750 2000
Wire Wire Line
	2600 1900 2750 1900
Wire Wire Line
	2600 1800 2750 1800
Wire Wire Line
	2600 1700 2750 1700
Wire Wire Line
	2600 1600 2750 1600
Wire Wire Line
	850  3300 700  3300
Wire Wire Line
	850  3100 700  3100
Wire Wire Line
	850  3000 700  3000
Wire Wire Line
	850  2900 700  2900
Wire Wire Line
	850  2700 700  2700
Wire Wire Line
	850  2600 700  2600
Wire Wire Line
	850  2500 700  2500
Wire Wire Line
	850  2400 700  2400
Wire Wire Line
	850  2300 700  2300
Wire Wire Line
	850  2200 700  2200
Wire Wire Line
	850  2000 700  2000
Wire Wire Line
	850  1900 700  1900
Wire Wire Line
	850  1800 700  1800
Wire Wire Line
	700  1700 850  1700
Wire Wire Line
	850  3400 700  3400
Wire Wire Line
	850  3500 700  3500
Wire Wire Line
	850  3600 700  3600
Wire Wire Line
	850  3700 700  3700
Wire Wire Line
	850  3800 700  3800
Wire Wire Line
	850  1100 700  1100
Wire Wire Line
	850  1200 700  1200
Wire Wire Line
	850  1300 700  1300
Wire Wire Line
	850  1400 700  1400
Wire Wire Line
	2750 3300 2600 3300
Wire Wire Line
	2750 3400 2600 3400
Wire Wire Line
	2750 3500 2600 3500
Wire Wire Line
	2750 3600 2600 3600
Wire Wire Line
	2750 3700 2600 3700
Wire Wire Line
	2750 3800 2600 3800
Wire Wire Line
	2600 2800 2750 2800
Wire Wire Line
	2600 2100 2750 2100
Wire Wire Line
	2600 1100 2750 1100
Wire Wire Line
	2600 1200 2750 1200
Wire Wire Line
	2600 1300 2750 1300
Wire Wire Line
	2600 1400 2750 1400
Wire Wire Line
	2600 1500 2750 1500
Wire Wire Line
	2150 4950 2300 4950
Wire Wire Line
	2150 5050 2300 5050
Wire Wire Line
	2150 5150 2300 5150
Wire Wire Line
	2150 5250 2300 5250
Wire Wire Line
	2150 5350 2300 5350
Wire Wire Line
	2150 5450 2300 5450
Wire Wire Line
	2150 5550 2300 5550
Wire Wire Line
	2150 5750 2150 6050
Wire Wire Line
	2150 6050 1350 6050
Wire Wire Line
	1350 6050 1350 6450
Wire Wire Line
	2150 6450 2300 6450
Wire Wire Line
	2150 6550 2300 6550
Wire Wire Line
	2150 6650 2300 6650
Wire Wire Line
	2150 6750 2300 6750
Wire Wire Line
	2150 6850 2300 6850
Wire Wire Line
	2150 6950 2300 6950
Wire Wire Line
	2150 7050 2300 7050
Wire Wire Line
	2150 7150 2300 7150
Wire Wire Line
	1350 4850 800  4850
Wire Wire Line
	1350 5050 800  5050
Wire Wire Line
	1350 5350 800  5350
Wire Wire Line
	1350 5450 800  5450
Wire Wire Line
	1350 6650 800  6650
Wire Wire Line
	1350 6750 800  6750
Wire Wire Line
	1350 6950 800  6950
Wire Wire Line
	1350 7050 800  7050
Wire Wire Line
	1750 5950 800  5950
Wire Wire Line
	1750 7550 800  7550
Wire Wire Line
	1750 6250 2300 6250
Wire Wire Line
	2150 4850 2300 4850
Wire Wire Line
	1750 4650 2300 4650
Wire Wire Line
	800  5150 1350 5150
Wire Wire Line
	4000 5350 4000 5450
Wire Wire Line
	4800 6350 4800 6450
Wire Wire Line
	5600 7350 5750 7350
Wire Wire Line
	5750 7350 5750 7550
Wire Wire Line
	5750 7550 5300 7550
Wire Wire Line
	4000 5650 3450 5650
Wire Wire Line
	4000 5950 3450 5950
Wire Wire Line
	4000 5750 3450 5750
Wire Wire Line
	4000 6050 3450 6050
Wire Wire Line
	4250 6650 4800 6650
Wire Wire Line
	4800 6750 4250 6750
Wire Wire Line
	4800 6950 4250 6950
Wire Wire Line
	4800 7050 4250 7050
Wire Wire Line
	5200 7550 4900 7550
Wire Wire Line
	4400 6550 4100 6550
Wire Wire Line
	3600 5550 3300 5550
Wire Wire Line
	3200 4450 2650 4450
Wire Wire Line
	3200 4650 2650 4650
Wire Wire Line
	3200 4750 2650 4750
Wire Wire Line
	3200 4950 2650 4950
Wire Wire Line
	3200 5050 2650 5050
Wire Wire Line
	4000 4450 4100 4450
Wire Wire Line
	4000 4550 4100 4550
Wire Wire Line
	4000 4650 4100 4650
Wire Wire Line
	4000 4750 4100 4750
Wire Wire Line
	4000 4850 4100 4850
Wire Wire Line
	4000 4950 4100 4950
Wire Wire Line
	4000 5050 4100 5050
Wire Wire Line
	4000 5150 4100 5150
Wire Wire Line
	4800 5450 4950 5450
Wire Wire Line
	4800 5550 4950 5550
Wire Wire Line
	4800 5650 4950 5650
Wire Wire Line
	4800 5750 4950 5750
Wire Wire Line
	4800 5850 4950 5850
Wire Wire Line
	4800 5950 4950 5950
Wire Wire Line
	4800 6050 4950 6050
Wire Wire Line
	4800 6150 4950 6150
Wire Wire Line
	5600 6450 5750 6450
Wire Wire Line
	5600 6550 5750 6550
Wire Wire Line
	5600 6650 5750 6650
Wire Wire Line
	5600 6750 5750 6750
Wire Wire Line
	5600 6850 5750 6850
Wire Wire Line
	5600 6950 5750 6950
Wire Wire Line
	5600 7050 5750 7050
Wire Wire Line
	5600 7150 5750 7150
Wire Wire Line
	4400 5250 4950 5250
Wire Wire Line
	5200 6250 5750 6250
Wire Wire Line
	3600 4250 4100 4250
Wire Bus Line
	4200 5050 5050 5050
Wire Bus Line
	5050 6050 5850 6050
Wire Wire Line
	4050 7400 3400 7400
Wire Bus Line
	4150 7650 700  7650
Connection ~ 4150 7650
Wire Wire Line
	4050 7050 3400 7050
Wire Wire Line
	2900 7050 3100 7050
Connection ~ 3100 7050
Wire Wire Line
	3100 7400 3100 7050
Wire Wire Line
	6400 4750 6050 4750
Wire Wire Line
	6050 4850 6400 4850
Wire Wire Line
	6050 4950 6400 4950
Wire Wire Line
	6050 5050 6400 5050
Wire Wire Line
	6050 5150 6400 5150
Wire Wire Line
	6050 5250 6400 5250
Wire Wire Line
	6050 5350 6400 5350
Wire Wire Line
	6050 5450 6400 5450
Wire Wire Line
	6050 5550 6400 5550
Wire Wire Line
	6050 5950 6400 5950
Wire Wire Line
	6050 6050 6400 6050
Wire Wire Line
	6400 5750 6050 5750
Wire Wire Line
	7600 4850 7750 4850
Wire Wire Line
	7750 4950 7600 4950
Wire Wire Line
	7600 5050 7750 5050
Wire Wire Line
	7600 5150 7750 5150
Wire Wire Line
	7600 5250 7750 5250
Wire Wire Line
	7600 5350 7750 5350
Wire Wire Line
	7600 5450 7750 5450
Wire Wire Line
	7600 5550 7750 5550
Wire Wire Line
	7750 5750 7400 5750
Wire Wire Line
	7750 6050 7400 6050
Wire Wire Line
	7750 5950 7400 5950
Wire Wire Line
	6900 6350 6750 6350
Wire Wire Line
	8250 6350 7550 6350
Wire Bus Line
	5950 4200 7500 4200
Connection ~ 7300 6450
Wire Bus Line
	7300 6450 7450 6450
Wire Wire Line
	6900 4350 7400 4350
Wire Wire Line
	8250 4450 7600 4450
Wire Wire Line
	3300 1100 3100 1100
Wire Wire Line
	3300 1200 3100 1200
Wire Wire Line
	3300 1300 3100 1300
Wire Wire Line
	3300 1400 3100 1400
Wire Wire Line
	3300 1500 3100 1500
Wire Wire Line
	3300 1600 3100 1600
Wire Wire Line
	3300 1700 3100 1700
Wire Wire Line
	3300 1800 3100 1800
Wire Wire Line
	3300 1900 3100 1900
Wire Wire Line
	3300 2000 3100 2000
Wire Wire Line
	3300 2100 3100 2100
Wire Wire Line
	3300 2200 3100 2200
Wire Wire Line
	3300 2300 3100 2300
Wire Wire Line
	3300 2400 3100 2400
Wire Wire Line
	3300 2500 3100 2500
Wire Wire Line
	3300 2600 3100 2600
Wire Wire Line
	3300 2700 3100 2700
Wire Wire Line
	3300 2800 3100 2800
Wire Wire Line
	3300 2900 3100 2900
Wire Wire Line
	3300 3000 3100 3000
Wire Wire Line
	3300 3100 3100 3100
Wire Wire Line
	3300 3200 3100 3200
Wire Wire Line
	3300 3300 3100 3300
Wire Wire Line
	3300 3400 3100 3400
Wire Wire Line
	3300 3500 3100 3500
Wire Wire Line
	3300 3600 3100 3600
Wire Wire Line
	3300 3700 3100 3700
Wire Wire Line
	3300 3800 3100 3800
Wire Wire Line
	5050 1100 5250 1100
Wire Wire Line
	5050 1200 5250 1200
Wire Wire Line
	5050 1300 5250 1300
Wire Wire Line
	5050 1400 5250 1400
Wire Wire Line
	5050 1500 5250 1500
Wire Wire Line
	5050 1600 5250 1600
Wire Wire Line
	5050 1700 5250 1700
Wire Wire Line
	5050 1800 5250 1800
Wire Wire Line
	5050 1900 5250 1900
Wire Wire Line
	5050 2000 5250 2000
Wire Wire Line
	5050 2100 5250 2100
Wire Wire Line
	5050 2200 5250 2200
Wire Wire Line
	5050 2300 5250 2300
Wire Wire Line
	5050 2400 5250 2400
Wire Wire Line
	5050 2500 5250 2500
Wire Wire Line
	5050 2600 5250 2600
Wire Wire Line
	5050 2700 5250 2700
Wire Wire Line
	5050 2800 5250 2800
Wire Wire Line
	5050 2900 5250 2900
Wire Wire Line
	5050 3000 5250 3000
Wire Wire Line
	5050 3100 5250 3100
Wire Wire Line
	5050 3200 5250 3200
Wire Wire Line
	5050 3300 5250 3300
Wire Wire Line
	5050 3400 5250 3400
Wire Wire Line
	5050 3500 5250 3500
Wire Wire Line
	5050 3600 5250 3600
Wire Wire Line
	5050 3700 5250 3700
Wire Wire Line
	5050 3800 5250 3800
Wire Wire Line
	5750 1100 5550 1100
Wire Wire Line
	5550 1200 5750 1200
Wire Wire Line
	5550 1300 5750 1300
Wire Wire Line
	5550 1400 5750 1400
Wire Wire Line
	5550 1500 5750 1500
Wire Wire Line
	5550 1600 5750 1600
Wire Wire Line
	5550 1700 5750 1700
Wire Wire Line
	5550 1800 5750 1800
Wire Wire Line
	5550 1900 5750 1900
Wire Wire Line
	5550 2000 5750 2000
Wire Wire Line
	5550 2100 5750 2100
Wire Wire Line
	5550 2200 5750 2200
Wire Wire Line
	5550 2300 5750 2300
Wire Wire Line
	5550 2400 5750 2400
Wire Wire Line
	5550 2500 5750 2500
Wire Wire Line
	5550 2600 5750 2600
Wire Wire Line
	5550 2700 5750 2700
Wire Wire Line
	5550 2800 5750 2800
Wire Wire Line
	5550 3200 5750 3200
Wire Wire Line
	5550 3500 5750 3500
Wire Wire Line
	5550 3600 5750 3600
Wire Wire Line
	5550 3700 5750 3700
Wire Wire Line
	5550 3800 5750 3800
Wire Wire Line
	3050 6500 3250 6500
Wire Wire Line
	3250 6300 3050 6300
Wire Wire Line
	3050 6300 2500 6300
Connection ~ 3050 6300
Wire Wire Line
	3050 6500 2500 6500
Connection ~ 3050 6500
Wire Bus Line
	9150 5150 11050 5150
Wire Wire Line
	10250 5450 10950 5450
Wire Wire Line
	10250 5650 10950 5650
Wire Wire Line
	10250 6250 10950 6250
Wire Wire Line
	10250 6350 10300 6350
Wire Wire Line
	9250 6350 9750 6350
Wire Wire Line
	9250 6150 9750 6150
Wire Wire Line
	9250 5650 9750 5650
Wire Wire Line
	9250 5550 9750 5550
Connection ~ 5500 5400
Wire Wire Line
	5500 5400 5850 5400
Connection ~ 5500 5300
Wire Wire Line
	5500 5300 5850 5300
Wire Wire Line
	5500 5400 5150 5400
Wire Wire Line
	5500 5300 5150 5300
Wire Wire Line
	7500 3800 7800 3800
Wire Wire Line
	7500 3700 7800 3700
Wire Wire Line
	7500 3600 7800 3600
Wire Wire Line
	7500 3500 7800 3500
Wire Wire Line
	7500 3100 7800 3100
Wire Wire Line
	7500 2800 7800 2800
Wire Wire Line
	7500 2700 7800 2700
Wire Wire Line
	7500 2600 7800 2600
Wire Wire Line
	7500 2500 7800 2500
Wire Wire Line
	7500 2400 7800 2400
Wire Wire Line
	7500 2300 7800 2300
Wire Wire Line
	7500 2200 7800 2200
Wire Wire Line
	7500 2100 7800 2100
Wire Wire Line
	7500 2000 7800 2000
Wire Wire Line
	7500 1900 7800 1900
Wire Wire Line
	7500 1800 7800 1800
Wire Wire Line
	7500 1700 7800 1700
Wire Wire Line
	7500 1600 7800 1600
Wire Wire Line
	7500 1500 7800 1500
Wire Wire Line
	7500 1400 7800 1400
Wire Wire Line
	7500 1300 7800 1300
Wire Wire Line
	7500 1200 7800 1200
Wire Wire Line
	7500 1100 7800 1100
Wire Bus Line
	5350 650  7900 650 
Wire Wire Line
	7500 3200 7800 3200
Wire Wire Line
	9850 1050 9850 900 
Wire Wire Line
	9950 900  9950 1050
Wire Wire Line
	10050 900  10050 1050
Wire Wire Line
	10150 900  10150 1050
Wire Wire Line
	10250 900  10250 1050
Wire Wire Line
	10350 900  10350 1050
Wire Wire Line
	10450 900  10450 1050
Wire Wire Line
	10550 900  10550 1050
Wire Wire Line
	9850 1550 9850 1650
Wire Wire Line
	9950 1550 9950 1650
Wire Wire Line
	10050 1550 10050 1650
Wire Wire Line
	10150 1550 10150 1650
Wire Wire Line
	10250 1550 10250 1650
Wire Wire Line
	10350 1550 10350 1650
Wire Wire Line
	10450 1550 10450 1650
Wire Wire Line
	10550 1550 10550 1650
Wire Wire Line
	9750 1550 9650 1550
Wire Wire Line
	9650 1550 9650 1050
Wire Wire Line
	9650 1050 9750 1050
Wire Bus Line
	11150 800  11150 1750
Wire Wire Line
	8200 1050 8200 750 
Wire Wire Line
	8300 1050 8300 750 
Wire Wire Line
	8400 1050 8400 750 
Wire Wire Line
	8500 750  8500 1050
Wire Wire Line
	8600 1050 8600 750 
Wire Wire Line
	8700 750  8700 1050
Wire Wire Line
	8800 1050 8800 750 
Wire Wire Line
	8900 750  8900 1050
Wire Wire Line
	9000 750  9000 1050
Wire Wire Line
	9100 750  9100 1050
Wire Wire Line
	9200 750  9200 1050
Wire Wire Line
	9300 750  9300 1050
Wire Wire Line
	9400 750  9400 1050
Wire Wire Line
	8200 1550 8200 1900
Wire Wire Line
	8300 1550 8300 1900
Wire Wire Line
	8400 1550 8400 1900
Wire Wire Line
	8500 1550 8500 1900
Wire Wire Line
	8600 1900 8600 1550
Wire Wire Line
	8700 1550 8700 1900
Wire Wire Line
	8800 1550 8800 1900
Wire Wire Line
	8900 1550 8900 1900
Wire Wire Line
	9000 1550 9000 1900
Wire Wire Line
	9100 1550 9100 1900
Wire Wire Line
	9200 1550 9200 1900
Wire Wire Line
	9300 1550 9300 1900
Wire Wire Line
	9400 1550 9400 1900
Wire Wire Line
	8350 3100 8350 2900
Wire Wire Line
	8450 3100 8450 2900
Wire Wire Line
	8550 3100 8550 2900
Wire Wire Line
	8650 3100 8650 2900
Wire Wire Line
	8750 3100 8750 2900
Wire Wire Line
	8850 3100 8850 2900
Wire Wire Line
	8950 2900 8950 3100
Wire Wire Line
	9050 3100 9050 2900
Wire Wire Line
	9150 2900 9150 3100
Wire Wire Line
	9250 3100 9250 2900
Wire Wire Line
	9250 2400 9250 2250
Wire Wire Line
	9150 2250 9150 2400
Wire Wire Line
	9050 2250 9050 2400
Wire Wire Line
	8950 2250 8950 2400
Wire Wire Line
	8850 2250 8850 2400
Wire Wire Line
	8750 2250 8750 2400
Wire Wire Line
	8650 2250 8650 2400
Wire Wire Line
	8550 2250 8550 2400
Wire Wire Line
	8450 2250 8450 2400
Wire Wire Line
	8350 2250 8350 2400
Wire Bus Line
	8150 2150 8150 3200
Wire Wire Line
	10000 2400 10000 2250
Wire Wire Line
	9900 2250 9900 2400
Wire Wire Line
	9800 2250 9800 2400
Wire Wire Line
	9700 2250 9700 2400
Wire Bus Line
	7900 2150 8150 2150
Connection ~ 7900 2150
Connection ~ 8150 2150
Wire Wire Line
	10400 3100 10400 2900
Wire Wire Line
	10500 3100 10500 2900
Wire Wire Line
	10600 3100 10600 2900
Wire Wire Line
	10700 3100 10700 2900
Wire Wire Line
	10800 3100 10800 2900
Wire Wire Line
	10800 2250 10800 2400
Wire Wire Line
	10700 2250 10700 2400
Wire Wire Line
	10600 2250 10600 2400
Wire Wire Line
	10500 2250 10500 2400
Wire Wire Line
	10400 2250 10400 2400
Wire Wire Line
	10150 4500 10150 4150
Wire Wire Line
	10450 4500 10450 4150
Wire Wire Line
	10550 4500 10550 4150
Wire Wire Line
	10150 3300 10150 3650
Connection ~ 9850 3200
Wire Wire Line
	10350 4150 10350 4250
Wire Wire Line
	9700 5450 9750 5450
Wire Wire Line
	9550 2150 9550 2000
Wire Wire Line
	8300 3950 8650 3950
Connection ~ 9100 3950
Wire Wire Line
	9100 3950 9450 3950
Wire Wire Line
	8100 3550 8400 3550
Wire Wire Line
	8100 3750 8300 3750
Connection ~ 8300 3750
Wire Wire Line
	8300 3750 8400 3750
Connection ~ 9100 3750
Wire Wire Line
	9100 3750 9300 3750
Connection ~ 9300 3750
Wire Wire Line
	9300 3750 9450 3750
Wire Wire Line
	8100 3550 8000 3550
Wire Wire Line
	8000 3550 8000 3950
Connection ~ 8100 3550
Connection ~ 8300 3950
Wire Wire Line
	8000 3950 8100 3950
Connection ~ 8100 3750
Connection ~ 8100 3950
Wire Wire Line
	8100 3950 8300 3950
Connection ~ 9450 3750
Connection ~ 9450 3950
Wire Wire Line
	9450 3950 9500 3950
Connection ~ 8650 3750
Connection ~ 8650 3950
Wire Wire Line
	8650 3750 8700 3750
Connection ~ 8400 3550
Connection ~ 8400 3750
Connection ~ 8700 3550
Connection ~ 8700 3750
Wire Wire Line
	8700 3550 9050 3550
Wire Wire Line
	8700 3750 9050 3750
Connection ~ 9050 3550
Wire Wire Line
	9050 3550 9300 3550
Connection ~ 9050 3750
Wire Wire Line
	9050 3750 9100 3750
Wire Wire Line
	8400 3750 8650 3750
Wire Wire Line
	9450 3750 9700 3750
Wire Wire Line
	8400 3550 8700 3550
Wire Wire Line
	9750 4350 9300 4350
Wire Wire Line
	9300 4350 9300 4450
Wire Wire Line
	9300 4450 9750 4450
Connection ~ 9300 4450
Wire Wire Line
	10550 3300 10550 3650
Wire Wire Line
	10450 3300 10450 3650
Connection ~ 9750 1050
Wire Wire Line
	7750 4750 7600 4750
Wire Wire Line
	6900 4350 6900 4450
Wire Wire Line
	8650 3950 8900 3950
Wire Wire Line
	8900 3950 9100 3950
Connection ~ 8900 3950
Wire Bus Line
	3000 950  3400 950 
Wire Bus Line
	2850 750  3400 750 
Wire Bus Line
	600  950  1050 950 
Wire Bus Line
	2400 4350 1850 4350
Text Label 1850 4350 0    50   ~ 0
DATA_BUS
$Comp
L Device:R R3
U 1 1 622C1781
P 9450 4800
F 0 "R3" V 9243 4800 50  0000 C CNN
F 1 "10K" V 9334 4800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9380 4800 50  0001 C CNN
F 3 "~" H 9450 4800 50  0001 C CNN
	1    9450 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	9600 4800 10300 4800
Wire Bus Line
	11950 100  15000 100 
$Comp
L md-cart:MD-CART-FINGERS-doragasu P1
U 1 1 61DD050A
P 13450 3000
F 0 "P1" H 13450 5587 60  0000 C CNN
F 1 "MD-CART-FINGERS-doragasu" H 13450 5481 60  0000 C CNN
F 2 "md-fingers:md-cart" H 13000 2300 60  0001 C CNN
F 3 "" H 13000 2300 60  0000 C CNN
	1    13450 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	12650 800  12550 800 
Wire Wire Line
	12550 900  12650 900 
Text Label 11950 100  0    50   ~ 0
CART_BUS
Wire Wire Line
	12550 1200 12550 1300
Wire Wire Line
	12550 1400 12650 1400
Wire Wire Line
	12550 1200 12650 1200
Wire Wire Line
	12650 1300 12550 1300
Connection ~ 12550 1300
Wire Wire Line
	12550 1300 12550 1400
Wire Wire Line
	12550 900  12550 800 
Wire Wire Line
	12550 1400 12550 1700
Wire Wire Line
	12550 1700 12650 1700
Connection ~ 12550 1400
Entry Wire Line
	11950 3200 12050 3300
Wire Wire Line
	12050 3300 12650 3300
Entry Wire Line
	11950 3500 12050 3600
Wire Wire Line
	12050 3600 12650 3600
Wire Wire Line
	12050 3700 12650 3700
Text Label 12050 3300 0    50   ~ 0
RESET#
Text Label 12050 3600 0    50   ~ 0
H_RESET#
Entry Wire Line
	11950 3600 12050 3700
Text Label 12050 3700 0    50   ~ 0
S_RESET#
Entry Wire Line
	11950 4200 12050 4300
Wire Wire Line
	12050 4300 12650 4300
Text Label 12050 4300 0    50   ~ 0
DTACK#
NoConn ~ 12650 4500
NoConn ~ 12650 4000
NoConn ~ 12650 2900
NoConn ~ 12650 3000
NoConn ~ 12650 2600
NoConn ~ 12650 2300
NoConn ~ 12650 2000
NoConn ~ 12650 4800
NoConn ~ 12650 4900
NoConn ~ 12650 5100
Entry Wire Line
	14900 700  15000 800 
Entry Wire Line
	14900 800  15000 900 
Entry Wire Line
	14900 900  15000 1000
Entry Wire Line
	14900 1000 15000 1100
Entry Wire Line
	14900 1100 15000 1200
Entry Wire Line
	14900 1200 15000 1300
Entry Wire Line
	14900 1300 15000 1400
Entry Wire Line
	14900 1400 15000 1500
Entry Wire Line
	14900 1500 15000 1600
Entry Wire Line
	14900 1600 15000 1700
Entry Wire Line
	14900 1700 15000 1800
Entry Wire Line
	14900 1800 15000 1900
Entry Wire Line
	14900 1900 15000 2000
Entry Wire Line
	14900 2000 15000 2100
Entry Wire Line
	14900 2100 15000 2200
Entry Wire Line
	14900 2200 15000 2300
Entry Wire Line
	14900 2300 15000 2400
Entry Wire Line
	14900 2400 15000 2500
Entry Wire Line
	14900 2500 15000 2600
Entry Wire Line
	14900 2600 15000 2700
Entry Wire Line
	14900 2700 15000 2800
Entry Wire Line
	14900 2800 15000 2900
Entry Wire Line
	14900 2900 15000 3000
Entry Wire Line
	14900 3100 15000 3200
Entry Wire Line
	14900 3200 15000 3300
Entry Wire Line
	14900 3300 15000 3400
Entry Wire Line
	14900 3400 15000 3500
Entry Wire Line
	14900 3500 15000 3600
Entry Wire Line
	14900 3600 15000 3700
Entry Wire Line
	14900 3700 15000 3800
Entry Wire Line
	14900 3800 15000 3900
Entry Wire Line
	14900 3900 15000 4000
Entry Wire Line
	14900 4000 15000 4100
Entry Wire Line
	14900 4100 15000 4200
Entry Wire Line
	14900 4200 15000 4300
Entry Wire Line
	14900 4300 15000 4400
Entry Wire Line
	14900 4400 15000 4500
Entry Wire Line
	14900 4500 15000 4600
Entry Wire Line
	14900 4600 15000 4700
Entry Wire Line
	14900 4800 15000 4900
Entry Wire Line
	14900 4900 15000 5000
Wire Wire Line
	14900 700  14250 700 
Wire Wire Line
	14250 800  14900 800 
Wire Wire Line
	14900 900  14250 900 
Wire Wire Line
	14900 1000 14250 1000
Wire Wire Line
	14900 1100 14250 1100
Wire Wire Line
	14900 1200 14250 1200
Wire Wire Line
	14900 1300 14250 1300
Wire Wire Line
	14900 1400 14250 1400
Wire Wire Line
	14900 1500 14250 1500
Wire Wire Line
	14900 1600 14250 1600
Wire Wire Line
	14900 1700 14250 1700
Wire Wire Line
	14900 1800 14250 1800
Wire Wire Line
	14900 1900 14250 1900
Wire Wire Line
	14900 2000 14250 2000
Wire Wire Line
	14900 2100 14250 2100
Wire Wire Line
	14900 2200 14250 2200
Wire Wire Line
	14900 2300 14250 2300
Wire Wire Line
	14900 2400 14250 2400
Wire Wire Line
	14900 2500 14250 2500
Wire Wire Line
	14900 2600 14250 2600
Wire Wire Line
	14900 2700 14250 2700
Wire Wire Line
	14900 2800 14250 2800
Wire Wire Line
	14900 2900 14250 2900
Wire Wire Line
	14900 3100 14250 3100
Wire Wire Line
	14900 3200 14250 3200
Wire Wire Line
	14900 3300 14250 3300
Wire Wire Line
	14900 3400 14250 3400
Wire Wire Line
	14900 3500 14250 3500
Wire Wire Line
	14900 3600 14250 3600
Wire Wire Line
	14900 3700 14250 3700
Wire Wire Line
	14900 3800 14250 3800
Wire Wire Line
	14900 3900 14250 3900
Wire Wire Line
	14900 4000 14250 4000
Wire Wire Line
	14900 4100 14250 4100
Wire Wire Line
	14900 4200 14250 4200
Wire Wire Line
	14900 4300 14250 4300
Wire Wire Line
	14900 4400 14250 4400
Wire Wire Line
	14900 4500 14250 4500
Wire Wire Line
	14900 4600 14250 4600
Wire Wire Line
	14900 4800 14250 4800
Wire Wire Line
	14900 4900 14250 4900
Text Label 14300 700  0    50   ~ 0
AI_0
Text Label 14300 800  0    50   ~ 0
AI_1
Text Label 14300 900  0    50   ~ 0
AI_2
Text Label 14300 1000 0    50   ~ 0
AI_3
Text Label 14300 1100 0    50   ~ 0
AI_4
Text Label 14300 1200 0    50   ~ 0
AI_5
Text Label 14300 1300 0    50   ~ 0
AI_6
Text Label 14300 1400 0    50   ~ 0
AI_7
Text Label 14300 1500 0    50   ~ 0
AI_8
Text Label 14300 1600 0    50   ~ 0
AI_9
Text Label 14300 1700 0    50   ~ 0
AI_10
Text Label 14300 1800 0    50   ~ 0
AI_11
Text Label 14300 1900 0    50   ~ 0
AI_12
Text Label 14300 2000 0    50   ~ 0
AI_13
Text Label 14300 2100 0    50   ~ 0
AI_14
Text Label 14300 2200 0    50   ~ 0
AI_15
Text Label 14300 2300 0    50   ~ 0
AI_16
Text Label 14300 2400 0    50   ~ 0
AI_17
Text Label 14300 2500 0    50   ~ 0
AI_18
Text Label 14300 2600 0    50   ~ 0
AI_19
Text Label 14300 2700 0    50   ~ 0
AI_20
Text Label 14300 2800 0    50   ~ 0
AI_21
Text Label 14300 2900 0    50   ~ 0
AI_22
Text Label 14300 3100 0    50   ~ 0
DI_0
Text Label 14300 3200 0    50   ~ 0
DI_1
Text Label 14300 3300 0    50   ~ 0
DI_2
Text Label 14300 3400 0    50   ~ 0
DI_3
Text Label 14300 3500 0    50   ~ 0
DI_4
Text Label 14300 3600 0    50   ~ 0
DI_5
Text Label 14300 3700 0    50   ~ 0
DI_6
Text Label 14300 3800 0    50   ~ 0
DI_7
Text Label 14300 3900 0    50   ~ 0
DI_8
Text Label 14300 4000 0    50   ~ 0
DI_9
Text Label 14300 4100 0    50   ~ 0
DI_10
Text Label 14300 4200 0    50   ~ 0
DI_11
Text Label 14300 4300 0    50   ~ 0
DI_12
Text Label 14300 4400 0    50   ~ 0
DI_13
Text Label 14300 4500 0    50   ~ 0
DI_14
Text Label 14300 4600 0    50   ~ 0
DI_15
Text Label 14300 4900 0    50   ~ 0
GEN_CE#
Text Label 14300 4800 0    50   ~ 0
GEN_OE#
NoConn ~ 14250 5000
NoConn ~ 14250 5100
NoConn ~ 14250 5200
Entry Wire Line
	9500 3200 9600 3100
Wire Wire Line
	9900 3100 9900 2900
Wire Wire Line
	9800 3100 9800 2900
Wire Wire Line
	9700 3100 9700 2900
Wire Wire Line
	10000 3100 10000 2900
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J5
U 1 1 64C24C0B
P 9800 2700
F 0 "J5" V 9850 2400 50  0000 R CNN
F 1 "CART_B10" V 9850 2850 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 9800 2700 50  0001 C CNN
F 3 "~" H 9800 2700 50  0001 C CNN
	1    9800 2700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9600 2900 9600 3100
Text Label 9600 3100 1    50   ~ 0
H_RESET#
Text Label 10650 3650 1    50   ~ 0
RESET#
Wire Wire Line
	10650 3400 10650 3650
Wire Wire Line
	10350 4250 10250 4250
Wire Wire Line
	10250 4250 10250 4700
Wire Wire Line
	10250 4700 9800 4700
Text Label 9800 4700 0    50   ~ 0
DTACK#
Wire Wire Line
	10850 4150 10850 4600
Text Label 10850 4600 1    50   ~ 0
S_RESET#
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J8
U 1 1 620DA531
P 13450 5750
F 0 "J8" H 13500 6067 50  0000 C CNN
F 1 "Conn_02x04_Odd_Even" H 13500 5976 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 13450 5750 50  0001 C CNN
F 3 "~" H 13450 5750 50  0001 C CNN
	1    13450 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	13250 5950 13150 5950
Wire Wire Line
	13150 5950 13150 5850
Wire Wire Line
	13250 5750 13150 5750
Wire Wire Line
	13250 5850 13150 5850
Connection ~ 13150 5850
Wire Wire Line
	13150 5850 13150 5750
Wire Wire Line
	13150 5750 13050 5750
Connection ~ 13150 5750
Text GLabel 13050 5750 0    50   Input ~ 0
GEN_RESET#
Wire Wire Line
	13750 5950 14150 5950
Text Label 13750 5950 0    50   ~ 0
S_RESET#
Text Label 13750 5850 0    50   ~ 0
H_RESET#
Text Label 13750 5750 0    50   ~ 0
RESET#
Wire Wire Line
	13750 5650 14150 5650
Wire Wire Line
	13750 5750 14150 5750
Wire Wire Line
	13750 5850 14150 5850
Entry Wire Line
	14150 5950 14250 6050
Entry Wire Line
	14150 5850 14250 5950
Entry Wire Line
	14150 5750 14250 5850
Entry Wire Line
	14150 5650 14250 5750
Wire Bus Line
	14250 5550 15000 5550
Text Label 13750 5650 0    50   ~ 0
DTACK#
Wire Wire Line
	13250 5650 12850 5650
$Comp
L power:+5V #PWR0101
U 1 1 626A5866
P 12850 5650
F 0 "#PWR0101" H 12850 5500 50  0001 C CNN
F 1 "+5V" H 12865 5823 50  0000 C CNN
F 2 "" H 12850 5650 50  0001 C CNN
F 3 "" H 12850 5650 50  0001 C CNN
	1    12850 5650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0116
U 1 1 626A7824
P 12550 800
F 0 "#PWR0116" H 12550 650 50  0001 C CNN
F 1 "+5V" H 12565 973 50  0000 C CNN
F 2 "" H 12550 800 50  0001 C CNN
F 3 "" H 12550 800 50  0001 C CNN
	1    12550 800 
	1    0    0    -1  
$EndComp
Connection ~ 12550 800 
$Comp
L power:GND #PWR0117
U 1 1 627A17F6
P 12550 1700
F 0 "#PWR0117" H 12550 1450 50  0001 C CNN
F 1 "GND" H 12600 1550 50  0000 R CNN
F 2 "" H 12550 1700 50  0001 C CNN
F 3 "" H 12550 1700 50  0001 C CNN
	1    12550 1700
	1    0    0    -1  
$EndComp
Connection ~ 12550 1700
Wire Bus Line
	600  4150 1000 4150
Text Label 600  4150 0    50   ~ 0
CART_BUS
Text Label 600  950  0    50   ~ 0
CART_BUS
Entry Wire Line
	10300 4700 10400 4600
Wire Wire Line
	10300 4700 10300 4800
Connection ~ 10300 4800
Wire Wire Line
	9300 4450 9300 4800
Entry Wire Line
	9750 4450 9850 4550
Entry Wire Line
	9750 4350 9850 4450
Entry Wire Line
	9750 4250 9850 4350
Entry Wire Line
	9750 4150 9850 4250
Wire Wire Line
	9750 4150 9300 4150
Wire Wire Line
	9300 4150 9300 4250
Wire Wire Line
	9300 4250 9750 4250
Wire Bus Line
	5950 6450 7300 6450
Wire Bus Line
	2550 5650 3350 5650
Wire Bus Line
	3350 6650 4150 6650
Wire Bus Line
	4150 7650 5200 7650
Wire Bus Line
	5950 5850 5950 6450
Wire Bus Line
	7300 5850 7300 6450
Wire Bus Line
	11050 5150 11050 6150
Wire Bus Line
	9850 4600 10450 4600
Wire Bus Line
	9150 5150 9150 6450
Wire Bus Line
	2550 4550 2550 5650
Wire Bus Line
	14250 5550 14250 6050
Wire Bus Line
	11950 100  11950 4500
Wire Bus Line
	600  3600 600  4150
Wire Bus Line
	600  950  600  1500
Wire Bus Line
	4150 6650 4150 7650
Wire Bus Line
	3350 5650 3350 6650
Wire Bus Line
	9850 3200 9850 4600
Wire Bus Line
	9850 3200 10700 3200
Wire Bus Line
	9950 800  11150 800 
Wire Bus Line
	9750 1750 11150 1750
Wire Bus Line
	5950 4200 5950 5650
Wire Bus Line
	5850 6050 5850 7050
Wire Bus Line
	5050 5050 5050 6050
Wire Bus Line
	4200 4150 4200 5050
Wire Bus Line
	700  4950 700  7650
Wire Bus Line
	600  1600 600  3500
Wire Bus Line
	2400 4350 2400 7050
Wire Bus Line
	7500 4200 7500 5650
Wire Bus Line
	7900 2150 7900 3700
Wire Bus Line
	7900 650  7900 2150
Wire Bus Line
	8300 650  9500 650 
Wire Bus Line
	8100 2000 9300 2000
Wire Bus Line
	8150 3200 9850 3200
Wire Bus Line
	8150 2150 10900 2150
Wire Bus Line
	2850 750  2850 3700
Wire Bus Line
	3000 950  3000 3900
Wire Bus Line
	5350 650  5350 3700
Wire Bus Line
	5450 1200 5450 3900
Wire Bus Line
	15000 100  15000 5550
Text Label 9350 4150 0    50   ~ 0
GEN_CE#
$EndSCHEMATC
