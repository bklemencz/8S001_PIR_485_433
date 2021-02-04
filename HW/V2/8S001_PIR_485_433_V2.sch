EESchema Schematic File Version 4
LIBS:8S001_PIR_485_433_V2-cache
EELAYER 26 0
EELAYER END
$Descr User 8268 5827
encoding utf-8
Sheet 1 1
Title "PIR Sensor with RS485 Interface"
Date "2021-01-22"
Rev "V1.0"
Comp "BK"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Interface_UART:SP3485EN U3
U 1 1 5FAD2AEA
P 5625 2675
F 0 "U3" H 5375 3025 50  0000 C CNN
F 1 "SP3485EN" H 5875 3025 50  0000 C CNN
F 2 "KiCad/kicad-footprints/Package_SO.pretty:SOIC-8_3.9x4.9mm_P1.27mm" H 6675 2325 50  0001 C CIN
F 3 "http://www.icbase.com/pdf/SPX/SPX00480106.pdf" H 5625 2675 50  0001 C CNN
	1    5625 2675
	1    0    0    -1  
$EndComp
$Comp
L 8S001_PIR_485_433_V2-rescue:STM8S001J3M-8S001_PIR_485_433 U2
U 1 1 5FAD2BF8
P 3275 2525
F 0 "U2" H 3025 2875 50  0000 C CNN
F 1 "STM8S001J3M" H 3575 2875 50  0000 C CNN
F 2 "KiCad/kicad-footprints/Package_SO.pretty:SOIC-8_3.9x4.9mm_P1.27mm" H 3325 3075 50  0001 L CNN
F 3 "https://www.st.com/resource/en/datasheet/stm8s001j3.pdf" H 3175 2625 50  0001 C CNN
	1    3275 2525
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5FAD2E08
P 2775 2925
F 0 "C3" H 2867 2971 50  0000 L CNN
F 1 "1u" H 2867 2880 50  0000 L CNN
F 2 "KiCad/kicad-footprints/Capacitor_SMD.pretty:C_0805_2012Metric" H 2775 2925 50  0001 C CNN
F 3 "~" H 2775 2925 50  0001 C CNN
	1    2775 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	2875 2725 2775 2725
Wire Wire Line
	2775 2725 2775 2825
Wire Wire Line
	2775 3025 2775 3075
Wire Wire Line
	3275 2925 3275 3075
Wire Wire Line
	3275 1975 3275 2125
Wire Wire Line
	5625 2175 5625 2275
$Comp
L power:GND #PWR017
U 1 1 5FAD30E4
P 5625 3175
F 0 "#PWR017" H 5625 2925 50  0001 C CNN
F 1 "GND" H 5630 3002 50  0000 C CNN
F 2 "" H 5625 3175 50  0001 C CNN
F 3 "" H 5625 3175 50  0001 C CNN
	1    5625 3175
	1    0    0    -1  
$EndComp
Wire Wire Line
	5625 3075 5625 3175
$Comp
L power:GND #PWR013
U 1 1 5FAD31CE
P 3025 3225
F 0 "#PWR013" H 3025 2975 50  0001 C CNN
F 1 "GND" H 3030 3052 50  0000 C CNN
F 2 "" H 3025 3225 50  0001 C CNN
F 3 "" H 3025 3225 50  0001 C CNN
	1    3025 3225
	1    0    0    -1  
$EndComp
Wire Wire Line
	2775 3075 3025 3075
Wire Wire Line
	3025 3225 3025 3075
Connection ~ 3025 3075
Wire Wire Line
	3025 3075 3275 3075
Wire Wire Line
	5225 2575 5125 2575
Wire Wire Line
	5125 2775 5225 2775
Wire Wire Line
	3675 2425 3975 2425
Text Label 3875 2425 0    50   ~ 0
Rx
Wire Wire Line
	3675 2525 3975 2525
Text Label 3775 2525 0    50   ~ 0
SWIM_485TX
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 5FAD3AE8
P 700 950
F 0 "J1" H 620 1267 50  0000 C CNN
F 1 "SWIM" H 620 1176 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 700 950 50  0001 C CNN
F 3 "~" H 700 950 50  0001 C CNN
	1    700  950 
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 5FAD3E33
P 1050 800
F 0 "#PWR01" H 1050 650 50  0001 C CNN
F 1 "+5V" H 1065 973 50  0000 C CNN
F 2 "" H 1050 800 50  0001 C CNN
F 3 "" H 1050 800 50  0001 C CNN
	1    1050 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  850  1050 850 
Wire Wire Line
	1050 850  1050 800 
$Comp
L power:GND #PWR05
U 1 1 5FAD4059
P 1050 1100
F 0 "#PWR05" H 1050 850 50  0001 C CNN
F 1 "GND" H 1055 927 50  0000 C CNN
F 2 "" H 1050 1100 50  0001 C CNN
F 3 "" H 1050 1100 50  0001 C CNN
	1    1050 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  1050 1050 1050
Wire Wire Line
	1050 1050 1050 1100
$Comp
L power:+5V #PWR07
U 1 1 5FAD46D4
P 1025 1700
F 0 "#PWR07" H 1025 1550 50  0001 C CNN
F 1 "+5V" H 1040 1873 50  0000 C CNN
F 2 "" H 1025 1700 50  0001 C CNN
F 3 "" H 1025 1700 50  0001 C CNN
	1    1025 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	925  1800 1025 1800
Wire Wire Line
	1025 1800 1025 1700
$Comp
L power:GND #PWR09
U 1 1 5FAD4A04
P 1025 2150
F 0 "#PWR09" H 1025 1900 50  0001 C CNN
F 1 "GND" H 1030 1977 50  0000 C CNN
F 2 "" H 1025 2150 50  0001 C CNN
F 3 "" H 1025 2150 50  0001 C CNN
	1    1025 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 5FAD4EB7
P 725 2900
F 0 "J3" H 725 2450 50  0000 C CNN
F 1 "485" H 725 2550 50  0000 C CNN
F 2 "KiCad/kicad-footprints/Connector_PinHeader_2.54mm.pretty:PinHeader_1x04_P2.54mm_Vertical" H 725 2900 50  0001 C CNN
F 3 "~" H 725 2900 50  0001 C CNN
	1    725  2900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5125 2575 5125 2675
NoConn ~ 5225 2475
Wire Wire Line
	5025 2325 5025 2675
Wire Wire Line
	5025 2675 5125 2675
Connection ~ 5125 2675
Wire Wire Line
	5125 2675 5125 2775
Text Label 6625 2575 0    50   ~ 0
A
Text Label 6625 2775 0    50   ~ 0
B
Wire Wire Line
	925  2800 1025 2800
Wire Wire Line
	1025 2800 1025 2700
$Comp
L power:GND #PWR012
U 1 1 5FAD65CC
P 1025 3200
F 0 "#PWR012" H 1025 2950 50  0001 C CNN
F 1 "GND" H 1030 3027 50  0000 C CNN
F 2 "" H 1025 3200 50  0001 C CNN
F 3 "" H 1025 3200 50  0001 C CNN
	1    1025 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	925  3100 1025 3100
Wire Wire Line
	1025 3100 1025 3200
Wire Wire Line
	925  2900 1325 2900
Wire Wire Line
	925  3000 1325 3000
Text Label 1275 2900 0    50   ~ 0
A
Text Label 1275 3000 0    50   ~ 0
B
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5FAD79BF
P 725 3650
F 0 "J4" H 645 3325 50  0000 C CNN
F 1 "SerIn" H 645 3416 50  0000 C CNN
F 2 "KiCad/kicad-footprints/Connector_PinHeader_2.54mm.pretty:PinHeader_1x02_P2.54mm_Vertical" H 725 3650 50  0001 C CNN
F 3 "~" H 725 3650 50  0001 C CNN
	1    725  3650
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5FAD9628
P 1025 3800
F 0 "#PWR016" H 1025 3550 50  0001 C CNN
F 1 "GND" H 1030 3627 50  0000 C CNN
F 2 "" H 1025 3800 50  0001 C CNN
F 3 "" H 1025 3800 50  0001 C CNN
	1    1025 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	925  3750 1025 3750
Wire Wire Line
	1025 3750 1025 3800
Wire Wire Line
	925  3650 1225 3650
Text Label 1125 3650 0    50   ~ 0
Rx
Wire Wire Line
	3675 2625 3975 2625
Wire Wire Line
	2875 2525 2475 2525
Text Label 2475 2525 0    50   ~ 0
RF
Wire Wire Line
	2875 2425 2475 2425
Text Label 2475 2425 0    50   ~ 0
BTN
Text Label 3775 2625 0    50   ~ 0
PIR
$Comp
L Device:C_Small C2
U 1 1 5FADC895
P 3775 1100
F 0 "C2" H 3867 1146 50  0000 L CNN
F 1 "10u" H 3867 1055 50  0000 L CNN
F 2 "KiCad/kicad-footprints/Capacitor_SMD.pretty:C_1206_3216Metric" H 3775 1100 50  0001 C CNN
F 3 "~" H 3775 1100 50  0001 C CNN
	1    3775 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5FADC9A8
P 2675 1100
F 0 "C1" H 2767 1146 50  0000 L CNN
F 1 "10u" H 2767 1055 50  0000 L CNN
F 2 "KiCad/kicad-footprints/Capacitor_SMD.pretty:C_1206_3216Metric" H 2675 1100 50  0001 C CNN
F 3 "~" H 2675 1100 50  0001 C CNN
	1    2675 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2375 950  2675 950 
Wire Wire Line
	2675 1000 2675 950 
Connection ~ 2675 950 
Wire Wire Line
	3775 1000 3775 950 
Wire Wire Line
	3775 950  4175 950 
Wire Wire Line
	2675 1200 2675 1400
Wire Wire Line
	2675 1400 3275 1400
Wire Wire Line
	3275 1400 3275 1250
Wire Wire Line
	3275 1400 3775 1400
Wire Wire Line
	3775 1400 3775 1200
Connection ~ 3275 1400
$Comp
L power:GND #PWR06
U 1 1 5FAE08E8
P 3275 1400
F 0 "#PWR06" H 3275 1150 50  0001 C CNN
F 1 "GND" H 3280 1227 50  0000 C CNN
F 2 "" H 3275 1400 50  0001 C CNN
F 3 "" H 3275 1400 50  0001 C CNN
	1    3275 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 5FAE09C4
P 4175 950
F 0 "#PWR03" H 4175 800 50  0001 C CNN
F 1 "+5V" H 4190 1123 50  0000 C CNN
F 2 "" H 4175 950 50  0001 C CNN
F 3 "" H 4175 950 50  0001 C CNN
	1    4175 950 
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5FAE272D
P 7150 1700
F 0 "SW1" V 7104 1848 50  0000 L CNN
F 1 "Sw" V 7195 1848 50  0000 L CNN
F 2 "KiCad/kicad-footprints/Button_Switch_SMD.pretty:SW_SPST_CK_RS282G05A3" H 7150 1900 50  0001 C CNN
F 3 "" H 7150 1900 50  0001 C CNN
	1    7150 1700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5FAE2A22
P 7150 1200
F 0 "R1" H 7209 1246 50  0000 L CNN
F 1 "10K" H 7209 1155 50  0000 L CNN
F 2 "KiCad/kicad-footprints/Resistor_SMD.pretty:R_0805_2012Metric" H 7150 1200 50  0001 C CNN
F 3 "~" H 7150 1200 50  0001 C CNN
	1    7150 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 950  7150 1100
Wire Wire Line
	7150 1300 7150 1400
$Comp
L power:GND #PWR08
U 1 1 5FAE468E
P 7150 2000
F 0 "#PWR08" H 7150 1750 50  0001 C CNN
F 1 "GND" H 7155 1827 50  0000 C CNN
F 2 "" H 7150 2000 50  0001 C CNN
F 3 "" H 7150 2000 50  0001 C CNN
	1    7150 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1900 7150 2000
Wire Wire Line
	7150 1400 6550 1400
Connection ~ 7150 1400
Wire Wire Line
	7150 1400 7150 1500
Text Label 6550 1400 0    50   ~ 0
BTN
$Comp
L Connector_Generic:Conn_01x03 J5
U 1 1 5FAE772B
P 750 4650
F 0 "J5" H 670 4967 50  0000 C CNN
F 1 "RF" H 670 4876 50  0000 C CNN
F 2 "Shutter_RF:SYN115_Mod" H 750 4650 50  0001 C CNN
F 3 "~" H 750 4650 50  0001 C CNN
	1    750  4650
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5FAE89BC
P 1050 4800
F 0 "#PWR019" H 1050 4550 50  0001 C CNN
F 1 "GND" H 1055 4627 50  0000 C CNN
F 2 "" H 1050 4800 50  0001 C CNN
F 3 "" H 1050 4800 50  0001 C CNN
	1    1050 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  4750 1050 4750
Wire Wire Line
	1050 4750 1050 4800
Wire Wire Line
	950  4650 1450 4650
Text Label 1050 4550 0    50   ~ 0
RF
Wire Wire Line
	6025 2575 6675 2575
Wire Wire Line
	6025 2775 6675 2775
$Comp
L power:+5V #PWR014
U 1 1 5FAF0FE6
P 5625 2175
F 0 "#PWR014" H 5625 2025 50  0001 C CNN
F 1 "+5V" H 5640 2348 50  0000 C CNN
F 2 "" H 5625 2175 50  0001 C CNN
F 3 "" H 5625 2175 50  0001 C CNN
	1    5625 2175
	1    0    0    -1  
$EndComp
Text Label 4675 2875 0    50   ~ 0
SWIM_485TX
Wire Wire Line
	4675 2875 5225 2875
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 5FE351BE
P 725 1900
F 0 "J2" H 645 2217 50  0000 C CNN
F 1 "PIR" H 645 2126 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 725 1900 50  0001 C CNN
F 3 "~" H 725 1900 50  0001 C CNN
	1    725  1900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	925  2000 1025 2000
Wire Wire Line
	1025 2000 1025 2150
Wire Wire Line
	925  1900 1225 1900
Text Label 1125 1900 0    50   ~ 0
PIR
Wire Wire Line
	900  950  1300 950 
Connection ~ 3775 950 
Text Label 1025 950  0    50   ~ 0
SWIM_485TX
$Comp
L Regulator_Linear:L78L05_SOT89 U1
U 1 1 601A900E
P 3275 950
F 0 "U1" H 3275 1192 50  0000 C CNN
F 1 "L78L05_SOT89" H 3275 1101 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-89-3" H 3275 1150 50  0001 C CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/15/55/e5/aa/23/5b/43/fd/CD00000446.pdf/files/CD00000446.pdf/jcr:content/translations/en.CD00000446.pdf" H 3275 900 50  0001 C CNN
	1    3275 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2675 950  2975 950 
Wire Wire Line
	3575 950  3775 950 
$Comp
L power:VCC #PWR02
U 1 1 601AB8B2
P 2375 900
F 0 "#PWR02" H 2375 750 50  0001 C CNN
F 1 "VCC" H 2392 1073 50  0000 C CNN
F 2 "" H 2375 900 50  0001 C CNN
F 3 "" H 2375 900 50  0001 C CNN
	1    2375 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2375 950  2375 900 
Wire Wire Line
	950  4550 1150 4550
$Comp
L power:+5V #PWR018
U 1 1 601ADF0A
P 1450 4600
F 0 "#PWR018" H 1450 4450 50  0001 C CNN
F 1 "+5V" H 1465 4773 50  0000 C CNN
F 2 "" H 1450 4600 50  0001 C CNN
F 3 "" H 1450 4600 50  0001 C CNN
	1    1450 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4600 1450 4650
$Comp
L power:VCC #PWR015
U 1 1 601AF588
P 1025 2700
F 0 "#PWR015" H 1025 2550 50  0001 C CNN
F 1 "VCC" H 1042 2873 50  0000 C CNN
F 2 "" H 1025 2700 50  0001 C CNN
F 3 "" H 1025 2700 50  0001 C CNN
	1    1025 2700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 601AF882
P 7150 950
F 0 "#PWR04" H 7150 800 50  0001 C CNN
F 1 "+5V" H 7165 1123 50  0000 C CNN
F 2 "" H 7150 950 50  0001 C CNN
F 3 "" H 7150 950 50  0001 C CNN
	1    7150 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 601AF92A
P 5025 2325
F 0 "#PWR011" H 5025 2175 50  0001 C CNN
F 1 "+5V" H 5040 2498 50  0000 C CNN
F 2 "" H 5025 2325 50  0001 C CNN
F 3 "" H 5025 2325 50  0001 C CNN
	1    5025 2325
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR010
U 1 1 601AF9B7
P 3275 1975
F 0 "#PWR010" H 3275 1825 50  0001 C CNN
F 1 "+5V" H 3290 2148 50  0000 C CNN
F 2 "" H 3275 1975 50  0001 C CNN
F 3 "" H 3275 1975 50  0001 C CNN
	1    3275 1975
	1    0    0    -1  
$EndComp
$EndSCHEMATC
