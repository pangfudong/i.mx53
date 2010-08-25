#if 0
PCIHDR.H: PCI Vendors, Devices, and Class Type information

  Created automatically from the web using the following URL:
      http://www.yourvote.com/pci
  Software to create and maintain the PCICODE List written by:
      Jim Boemler (jboemler@halcyon.com) 

  This header created on Thu, 13 Feb 2003 08:15:25 UTC

Too many people have contributed to this list to acknowledge them all, but
a few have provided the majority of the input and deserve special mention:
   Frederic Potter, who maintains a list for Linux.
   Chris Aston at Madge Networks.
   Thomas Dippon of Hewlett-Packard GmbH.
   Jurgen ("Josh") Thelen
   William H. Avery III at Altitech
   Sergei Shtylyov of Brain-dead Software in Russia
#endif

//  NOTE that the 0xFFFF of 0xFF entries at the end of some tables below are
//  not properly list terminators, but are actually the printable definitions
//  of values that are legitimately found on the PCI bus.  The size
//  definitions should be used for loop control when the table is searched.

typedef struct _PCI_VENTABLE
{
	unsigned short	VenId ;
	char *	VenShort ;
	char *	VenFull ;
}  PCI_VENTABLE, *PPCI_VENTABLE ;

PCI_VENTABLE	PciVenTable [] =
{
	{ 0x0000, "Gammagraphx", "Gammagraphx, Inc." } ,
	{ 0x0033, "", "Paradyne Corp." } ,
	{ 0x003D, "Lockheed Martin", "Lockheed Martin Corp" } ,
	{ 0x0070, "Hauppauge", "Hauppauge Computer Works Inc." } ,
	{ 0x0100, "", "Ncipher Corp. Ltd" } ,
	{ 0x0123, "", "General Dynamics" } ,
	{ 0x0160, "USR", "us robotics" } ,
	{ 0x0315, "", "SK - Electronics Co., Ltd." } ,
	{ 0x0675, "Dynalink", "Dynalink" } ,
	{ 0x0700, "Lava", "Lava Computer MFG Inc." } ,
	{ 0x0815, "LinTech", "LinTech GmbH" } ,
	{ 0x0871, "Berkom", "Berkom" } ,
	{ 0x0914, "SDT", "Shanghai Dare Technologies Ltd." } ,
	{ 0x09C1, "Arris", "Arris" } ,
	{ 0x0A89, "BREA", "BREA Technologies Inc." } ,
	{ 0x0E11, "Compaq", "Compaq Computer Corp." } ,
	{ 0x1000, "LSI", "LSI Logic" } ,
	{ 0x1001, "KOLTER", "Kolter Electronic - Germany" } ,
	{ 0x1002, "ATI", "ATI Technologies" } ,
	{ 0x1003, "ULSI", "ULSI" } ,
	{ 0x1004, "VLSI", "VLSI Technology" } ,
	{ 0x1005, "Avance", "Avance Logic Inc." } ,
	{ 0x1006, "Reply", "Reply Group" } ,
	{ 0x1007, "NetFrame", "Netframe Systems Inc." } ,
	{ 0x1008, "Epson", "Epson" } ,
	{ 0x100A, "Phoenix", "Phoenix Technologies Ltd." } ,
	{ 0x100B, "NSC", "National Semiconductor" } ,
	{ 0x100C, "Tseng", "Tseng Labs" } ,
	{ 0x100D, "AST", "AST Research" } ,
	{ 0x100E, "Weitek", "Weitek" } ,
	{ 0x1010, "VLogic", "Video Logic Ltd." } ,
	{ 0x1011, "DEC", "Digital Equipment Corporation" } ,
	{ 0x1012, "Micronics", "Micronics Computers Inc." } ,
	{ 0x1013, "Cirrus", "Cirrus Logic" } ,
	{ 0x1014, "IBM", "International Business Machines Corp." } ,
	{ 0x1015, "LSIL", "LSI Logic Corp of Canada" } ,
	{ 0x1016, "Fujitsu ICL", "Fujitsu ICL Computers" } ,
	{ 0x1017, "Spea", "Spea Software AG" } ,
	{ 0x1018, "Unisys", "Unisys Systems" } ,
	{ 0x1019, "ECS", "Elitegroup Computer Sys" } ,
	{ 0x101A, "NCR", "NCR/AT&T GIS" } ,
	{ 0x101B, "Vitesse", "Vitesse Semiconductor" } ,
	{ 0x101C, "WDC", "Western Digital" } ,
	{ 0x101E, "AMI", "American Megatrends Inc." } ,
	{ 0x101F, "PictureTel", "PictureTel Corp." } ,
	{ 0x1020, "Hitachi", "Hitachi Computer Electronics" } ,
	{ 0x1021, "OKI", "Oki Electric Industry" } ,
	{ 0x1022, "AMD", "Advanced Micro Devices" } ,
	{ 0x1023, "Trident", "Trident Microsystems" } ,
	{ 0x1024, "Zenith", "Zenith Data Systems" } ,
	{ 0x1025, "Acer", "Acer Incorporated" } ,
	{ 0x1028, "Dell", "Dell Computer Corporation" } ,
	{ 0x1029, "Siem-Nix", "Siemens Nixdorf AG" } ,
	{ 0x102A, "LSI", "LSI Logic Headland Division" } ,
	{ 0x102B, "Matrox", "Matrox" } ,
	{ 0x102C, "C&T", "Asiliant (Chips And Technologies)" } ,
	{ 0x102D, "Wyse", "Wyse Technologies" } ,
	{ 0x102E, "Olivetti", "Olivetti Advanced Technology" } ,
	{ 0x102F, "Toshiba", "Toshiba America" } ,
	{ 0x1030, "TMC", "TMC Research" } ,
	{ 0x1031, "miro", "miro Computer Products AG" } ,
	{ 0x1032, "Compaq", "Compaq" } ,
	{ 0x1033, "NEC", "NEC Corporation" } ,
	{ 0x1034, "Burndy", "Burndy Corporation" } ,
	{ 0x1035, "C&CRL", "Computer&Communication Research Lab" } ,
	{ 0x1036, "FDomain", "Future Domain" } ,
	{ 0x1037, "Hitachi", "Hitachi Micro Systems Inc" } ,
	{ 0x1038, "AMP", "AMP Incorporated" } ,
	{ 0x1039, "SiS", "Silicon Integrated System" } ,
	{ 0x103A, "Seiko", "Seiko Epson Corporation" } ,
	{ 0x103B, "Tatung", "Tatung Corp. Of America" } ,
	{ 0x103C, "HP", "Hewlett-Packard Company" } ,
	{ 0x103E, "Solliday", "Solliday Engineering" } ,
	{ 0x103F, "Logic Mod.", "Logic Modeling" } ,
	{ 0x1040, "Kubota", "Kubota Pacific Computer Inc." } ,
	{ 0x1041, "Computrend", "Computrend" } ,
	{ 0x1042, "PC Tech.", "PC Technology Inc." } ,
	{ 0x1043, "Asustek", "Asustek Computer Inc." } ,
	{ 0x1044, "DPT", "Distributed Processing Tech" } ,
	{ 0x1045, "OPTi", "OPTi Inc." } ,
	{ 0x1046, "IPC", "IPC Corporation LTD" } ,
	{ 0x1047, "Genoa", "Genoa Systems Corp." } ,
	{ 0x1048, "ELSA", "ELSA GmbH" } ,
	{ 0x1049, "Fountain", "Fountain Technology" } ,
	{ 0x104A, "STM", "STMicroelectronics" } ,
	{ 0x104B, "Mylex", "Mylex Corporation" } ,
	{ 0x104C, "TI", "Texas Instruments" } ,
	{ 0x104D, "Sony", "Sony Corporation" } ,
	{ 0x104E, "Oak", "Oak Technology" } ,
	{ 0x104F, "Co-Time", "Co-Time Computer Ltd." } ,
	{ 0x1050, "Winbond", "Winbond Electronics Corp." } ,
	{ 0x1051, "Anigma", "Anigma Corp." } ,
	{ 0x1052, "Young", "Young Micro Systems" } ,
	{ 0x1053, "Young", "Young Micro Systems" } ,
	{ 0x1054, "Hitachi", "Hitachi Ltd" } ,
	{ 0x1055, "SMSC", "Standard Microsystems Corp." } ,
	{ 0x1056, "ICL", "ICL" } ,
	{ 0x1057, "Motorola", "Motorola" } ,
	{ 0x1058, "E&TR", "Electronics & Telecommunication Res" } ,
	{ 0x1059, "Kontron", "Kontron Canada" } ,
	{ 0x105A, "Promise", "Promise Technology" } ,
	{ 0x105B, "Foxconn", "Foxconn International Inc." } ,
	{ 0x105C, "Wipro", "Wipro Infotech Limited" } ,
	{ 0x105D, "Number-Nine", "Number Nine Visual Technology" } ,
	{ 0x105E, "Vtech", "Vtech Engineering Canada Ltd." } ,
	{ 0x105F, "Infotronic", "Infotronic America Inc." } ,
	{ 0x1060, "UMC", "United Microelectronics" } ,
	{ 0x1061, "8x8", "8x8 Inc." } ,
	{ 0x1062, "Maspar", "Maspar Computer Corp." } ,
	{ 0x1063, "OOA", "Ocean Office Automation" } ,
	{ 0x1064, "Alcatel", "Alcatel Cit" } ,
	{ 0x1065, "TM", "Texas Microsystems" } ,
	{ 0x1066, "Picopower", "Picopower Technology" } ,
	{ 0x1067, "Mitsubishi", "Mitsubishi Electronics" } ,
	{ 0x1068, "Div. Tech.", "Diversified Technology" } ,
	{ 0x1069, "Mylex", "IBM Corporation" } ,
	{ 0x106A, "Aten", "Aten Research Inc." } ,
	{ 0x106B, "Apple", "Apple Computer Inc." } ,
	{ 0x106C, "Hyundai", "Hyundai Electronics America" } ,
	{ 0x106D, "Sequent", "Sequent Computer Systems" } ,
	{ 0x106E, "DFI", "DFI Inc." } ,
	{ 0x106F, "CityGate", "City Gate Development LTD" } ,
	{ 0x1070, "Daewoo", "Daewoo Telecom Ltd." } ,
	{ 0x1071, "Mitac", "Mitac" } ,
	{ 0x1072, "GIT", "GIT Co. Ltd." } ,
	{ 0x1073, "Yamaha", "Yamaha Corporation" } ,
	{ 0x1074, "Nexgen", "Nexgen Microsystems" } ,
	{ 0x1075, "AIR", "Advanced Integration Research" } ,
	{ 0x1076, "Chaintech", "Chaintech Computer Co. Ltd." } ,
	{ 0x1077, "QLogic", "QLogic Corporation" } ,
	{ 0x1078, "Cyrix", "Cyrix Corporation" } ,
	{ 0x1079, "I-Bus", "I-Bus" } ,
	{ 0x107A, "Networth", "Networth" } ,
	{ 0x107B, "Gateway", "Gateway 2000" } ,
	{ 0x107C, "Goldstar", "Goldstar Co. Ltd." } ,
	{ 0x107D, "Leadtek", "Leadtek Research" } ,
	{ 0x107E, "Interphase", "Interphase Corporation" } ,
	{ 0x107F, "DTC", "Data Technology Corporation" } ,
	{ 0x1080, "Cypress", "Cypress Semiconductor" } ,
	{ 0x1081, "Radius Inc.", "Radius Inc." } ,
	{ 0x1082, "EFA", "EFA Corporation Of America" } ,
	{ 0x1083, "Forex", "Forex Computer Corporation" } ,
	{ 0x1084, "Parador", "Parador" } ,
	{ 0x1085, "Tulip", "Tulip Computers Int'l BV" } ,
	{ 0x1086, "J. Bond", "J. Bond Computer Systems" } ,
	{ 0x1087, "Cache", "Cache Computer" } ,
	{ 0x1088, "MS Son", "Microcomputer Systems (M) Son" } ,
	{ 0x1089, "DG", "Data General Corporation" } ,
	{ 0x108A, "Bit3", "SBS  Operations" } ,
	{ 0x108C, "Oakleigh", "Oakleigh Systems Inc." } ,
	{ 0x108D, "Olicom", "Olicom" } ,
	{ 0x108E, "Sun", "Sun Microsystems" } ,
	{ 0x108F, "Systemsoft", "Systemsoft Corporation" } ,
	{ 0x1090, "Encore", "Encore Computer Corporation" } ,
	{ 0x1091, "Intergraph", "Intergraph Corporation" } ,
	{ 0x1092, "Diamond", "Diamond Computer Systems" } ,
	{ 0x1093, "Nat. Inst.", "National Instruments" } ,
	{ 0x1094, "FIC", "First Int'l Computers" } ,
	{ 0x1095, "Silicon Image", "Silicon Image, Inc." } ,
	{ 0x1096, "Alacron", "Alacron" } ,
	{ 0x1097, "Appian", "Appian Graphics" } ,
	{ 0x1098, "Quantum", "Quantum Designs Ltd." } ,
	{ 0x1099, "Samsung", "Samsung Electronics Co. Ltd." } ,
	{ 0x109A, "Packard-Bell", "Packard Bell" } ,
	{ 0x109B, "Gemlight", "Gemlight Computer Ltd." } ,
	{ 0x109C, "Megachips", "Megachips Corporation" } ,
	{ 0x109D, "Zida", "Zida Technologies Ltd." } ,
	{ 0x109E, "Brooktree", "Brooktree Corporation" } ,
	{ 0x109F, "Trigem", "Trigem Computer Inc." } ,
	{ 0x10A0, "Meidensha", "Meidensha Corporation" } ,
	{ 0x10A1, "Juko", "Juko Electronics Inc. Ltd." } ,
	{ 0x10A2, "Quantum", "Quantum Corporation" } ,
	{ 0x10A3, "Everex", "Everex Systems Inc." } ,
	{ 0x10A4, "Globe", "Globe Manufacturing Sales" } ,
	{ 0x10A5, "Racal", "Racal Interlan" } ,
	{ 0x10A6, "Informtech", "Informtech Industrial Ltd." } ,
	{ 0x10A7, "Benchmarq", "Benchmarq Microelectronics" } ,
	{ 0x10A8, "Sierra", "Sierra Semiconductor" } ,
	{ 0x10A9, "SG", "Silicon Graphics" } ,
	{ 0x10AA, "ACC", "ACC Microelectronics" } ,
	{ 0x10AB, "Digicom", "Digicom" } ,
	{ 0x10AC, "Honeywell", "Honeywell IASD" } ,
	{ 0x10AD, "Winbond", "Winbond Systems Labs" } ,
	{ 0x10AE, "Cornerstone", "Cornerstone Technology" } ,
	{ 0x10AF, "MCS", "Micro Computer Systems Inc." } ,
	{ 0x10B0, "CardExpert", "CardExpert Technology" } ,
	{ 0x10B1, "Cabletron", "Cabletron Systems Inc." } ,
	{ 0x10B2, "Raytheon", "Raytheon Company" } ,
	{ 0x10B3, "Databook", "Databook Inc." } ,
	{ 0x10B4, "STB", "STB Systems" } ,
	{ 0x10B5, "PLX", "PLX Technology" } ,
	{ 0x10B6, "Madge", "Madge Networks" } ,
	{ 0x10B7, "3Com", "3Com Corporation" } ,
	{ 0x10B8, "SMC", "Standard Microsystems Corporation" } ,
	{ 0x10B9, "ALi", "Acer Labs Inc." } ,
	{ 0x10BA, "Mitsubishi", "Mitsubishi Electronics Corp." } ,
	{ 0x10BB, "Dapha", "Dapha Electronics Corporation" } ,
	{ 0x10BC, "ALR", "Advanced Logic Research Inc." } ,
	{ 0x10BD, "Surecom", "Surecom Technology" } ,
	{ 0x10BE, "Tseng", "Tsenglabs International Corp." } ,
	{ 0x10BF, "MOST", "MOST Corp." } ,
	{ 0x10C0, "Boca", "Boca Research Inc." } ,
	{ 0x10C1, "ICM", "ICM Corp. Ltd." } ,
	{ 0x10C2, "Auspex", "Auspex Systems Inc." } ,
	{ 0x10C3, "Samsung", "Samsung Semiconductors" } ,
	{ 0x10C4, "Award", "Award Software Int'l Inc." } ,
	{ 0x10C5, "Xerox", "Xerox Corporation" } ,
	{ 0x10C6, "Rambus", "Rambus Inc." } ,
	{ 0x10C7, "Media Vision", "Media Vision" } ,
	{ 0x10C8, "Neomagic", "Neomagic Corporation" } ,
	{ 0x10C9, "Dataexpert", "Dataexpert Corporation" } ,
	{ 0x10CA, "Fujitsu", "Fujitsu" } ,
	{ 0x10CB, "Omron", "Omron Corporation" } ,
	{ 0x10CC, "MAI", "Mai Logic Incorporated" } ,
	{ 0x10CD, "AdvanSys", "Advanced System Products" } ,
	{ 0x10CE, "Radius", "Radius Inc." } ,
	{ 0x10CF, "Fujitsu", "Fujitsu Ltd." } ,
	{ 0x10D0, "Fujitsu", "Fujitsu Limited" } ,
	{ 0x10D1, "Future+", "Future+ Systems" } ,
	{ 0x10D2, "Molex", "Molex Incorporated" } ,
	{ 0x10D3, "Jabil", "Jabil Circuit Inc." } ,
	{ 0x10D4, "Hualon", "Hualon Microelectronics" } ,
	{ 0x10D5, "Autologic", "Autologic Inc." } ,
	{ 0x10D6, "Cetia", "Cetia" } ,
	{ 0x10D7, "BCM", "BCM Advanced Research" } ,
	{ 0x10D8, "APL", "Advanced Peripherals Labs" } ,
	{ 0x10D9, "Macronix", "Macronix International Co. Ltd." } ,
	{ 0x10DA, "T-C", "Thomas-Conrad Corporation" } ,
	{ 0x10DB, "Rohm", "Rohm Research" } ,
	{ 0x10DC, "CERN", "CERN-European Lab. for Particle Physics" } ,
	{ 0x10DD, "E&S", "Evans & Sutherland" } ,
	{ 0x10DE, "NVIDIA", "NVIDIA Corporation" } ,
	{ 0x10DF, "Emulex", "Emulex Corporation" } ,
	{ 0x10E0, "IMS", "Integrated Micro Solutions" } ,
	{ 0x10E1, "Tekram", "Tekram Technology Corp. Ltd." } ,
	{ 0x10E2, "Aptix", "Aptix Corporation" } ,
	{ 0x10E3, "Tundra", "Tundra Semiconductor Corp." } ,
	{ 0x10E4, "Tandem", "Tandem Computers" } ,
	{ 0x10E5, "MIC", "Micro Industries Corporation" } ,
	{ 0x10E6, "Gainbery", "Gainbery Computer Products Inc." } ,
	{ 0x10E7, "Vadem", "Vadem" } ,
	{ 0x10E8, "AMCC", "Applied Micro Circuits Corp." } ,
	{ 0x10E9, "Alps", "Alps Electronic Corp. Ltd." } ,
	{ 0x10EA, "Tvia", "Tvia, Inc." } ,
	{ 0x10EB, "Artist", "Artist Graphics" } ,
	{ 0x10EC, "Realtek", "Realtek Semiconductor" } ,
	{ 0x10ED, "Ascii", "Ascii Corporation" } ,
	{ 0x10EE, "Xilinx", "Xilinx Corporation" } ,
	{ 0x10EF, "Racore", "Racore Computer Products" } ,
	{ 0x10F0, "Peritek", "Peritek Corporation" } ,
	{ 0x10F1, "Tyan", "Tyan Computer" } ,
	{ 0x10F2, "Achme", "Achme Computer Inc." } ,
	{ 0x10F3, "Alaris", "Alaris Inc." } ,
	{ 0x10F4, "S-Mos", "S-Mos Systems" } ,
	{ 0x10F5, "NKK", "NKK Corporation" } ,
	{ 0x10F6, "Creative", "Creative Electronic Systems SA" } ,
	{ 0x10F7, "Matsushita", "Matsushita Electric Industrial Corp." } ,
	{ 0x10F8, "Altos", "Altos India Ltd." } ,
	{ 0x10F9, "PC-Direct", "PC Direct" } ,
	{ 0x10FA, "Truevision", "Truevision" } ,
	{ 0x10FB, "Thesys", "Thesys Microelectronic's" } ,
	{ 0x10FC, "I-O", "I-O Data Device Inc." } ,
	{ 0x10FD, "Soyo", "Soyo Technology Corp. Ltd." } ,
	{ 0x10FE, "Fast", "Fast Electronic GmbH" } ,
	{ 0x10FF, "Ncube", "Ncube" } ,
	{ 0x1100, "Jazz", "Jazz Multimedia" } ,
	{ 0x1101, "Initio", "Initio Corporation" } ,
	{ 0x1102, "Creative Labs", "Creative Labs" } ,
	{ 0x1103, "HighPoint", "HighPoint Technologies Inc." } ,
	{ 0x1104, "Rasterops", "Rasterops" } ,
	{ 0x1105, "Sigma", "Sigma Designs Inc." } ,
	{ 0x1106, "VIA", "VIA Technologies Inc" } ,
	{ 0x1107, "Stratus", "Stratus Computer" } ,
	{ 0x1108, "Proteon", "Proteon Inc." } ,
	{ 0x1109, "Cogent", "Adaptec/Cogent Data Technologies" } ,
	{ 0x110A, "Infineon", "Infineon Technologies" } ,
	{ 0x110B, "Chromatic", "Chromatic Research Inc" } ,
	{ 0x110C, "Mini-Max", "Mini-Max Technology Inc." } ,
	{ 0x110D, "ZNYX", "ZNYX Corporation" } ,
	{ 0x110E, "CPU Tech.", "CPU Technology" } ,
	{ 0x110F, "Ross", "Ross Technology" } ,
	{ 0x1110, "Powerhouse", "Powerhouse Systems" } ,
	{ 0x1111, "SCO", "SCO Group" } ,
	{ 0x1112, "Osicom", "Osicom Technologies Inc." } ,
	{ 0x1113, "Accton", "Accton Technology Corporation" } ,
	{ 0x1114, "Atmel", "Atmel Corp." } ,
	{ 0x1115, "Dupont", "Dupont Pixel Systems Ltd." } ,
	{ 0x1116, "Media 100", "Media 100, Inc." } ,
	{ 0x1117, "Datacube", "Datacube Inc." } ,
	{ 0x1118, "Berg", "Berg Electronics" } ,
	{ 0x1119, "Vortex", "ICP vortex Computersysteme GmbH" } ,
	{ 0x111A, "Eff. Net.", "Efficent Networks" } ,
	{ 0x111B, "Teledyne", "Teledyne Electronic Systems" } ,
	{ 0x111C, "Tricord", "Tricord Systems Inc." } ,
	{ 0x111D, "IDT", "Integrated Device Technology Inc." } ,
	{ 0x111E, "Eldec", "Eldec Corp." } ,
	{ 0x111F, "PDI", "Precision Digital Images" } ,
	{ 0x1120, "EMC", "EMC Corp." } ,
	{ 0x1121, "Zilog", "Zilog" } ,
	{ 0x1122, "Multi-Tech", "Multi-Tech Systems Inc." } ,
	{ 0x1123, "EDI", "Excellent Design Inc." } ,
	{ 0x1124, "Leutron", "Leutron Vision AG" } ,
	{ 0x1125, "Eurocore", "Eurocore/Vigra" } ,
	{ 0x1126, "Vigra", "Vigra" } ,
	{ 0x1127, "FORE", "FORE Systems" } ,
	{ 0x1129, "Firmworks", "Firmworks" } ,
	{ 0x112A, "Hermes", "Hermes Electronics Co. Ltd." } ,
	{ 0x112B, "Linotype", "Linotype - Hell AG" } ,
	{ 0x112C, "Zenith", "Zenith Data Systems" } ,
	{ 0x112D, "Ravicad", "Ravicad" } ,
	{ 0x112E, "Infomedia", "Infomedia" } ,
	{ 0x112F, "ImagTech", "Imaging Technology" } ,
	{ 0x1130, "Computervision", "Computervision" } ,
	{ 0x1131, "Philips", "Philips Semiconductors" } ,
	{ 0x1132, "Mitel", "Mitel Corp." } ,
	{ 0x1133, "EIC", "Eicon Networks Corporation" } ,
	{ 0x1134, "MCS", "Mercury Computer Systems Inc." } ,
	{ 0x1135, "Fuji", "Fuji Xerox Co Ltd" } ,
	{ 0x1136, "Momentum", "Momentum Data Systems" } ,
	{ 0x1137, "Cisco", "Cisco Systems Inc" } ,
	{ 0x1138, "Ziatech", "Ziatech Corporation" } ,
	{ 0x1139, "Dyn. Pict.", "Dynamic Pictures Inc" } ,
	{ 0x113A, "FWB", "FWB  Inc" } ,
	{ 0x113B, "NCD", "Network Computing Devices" } ,
	{ 0x113C, "Cyclone", "Cyclone Microsystems Inc." } ,
	{ 0x113D, "Leading Edge", "Leading Edge Products Inc" } ,
	{ 0x113E, "Sanyo", "Sanyo Electric Co" } ,
	{ 0x113F, "Equinox", "Equinox Systems" } ,
	{ 0x1140, "Intervoice", "Intervoice Inc" } ,
	{ 0x1141, "Crest", "Crest Microsystem Inc" } ,
	{ 0x1142, "Alliance", "Alliance Semiconductor CA - USA" } ,
	{ 0x1143, "Netpower", "Netpower Inc" } ,
	{ 0x1144, "Cinn. Mil.", "Cincinnati Milacron" } ,
	{ 0x1145, "Workbit", "Workbit Corp" } ,
	{ 0x1146, "Force", "Force Computers" } ,
	{ 0x1147, "Interface", "Interface Corp" } ,
	{ 0x1148, "S&K", "Schneider & Koch" } ,
	{ 0x1149, "Win System", "Win System Corporation" } ,
	{ 0x114A, "VMIC", "VMIC" } ,
	{ 0x114B, "Canopus", "Canopus Co. Ltd" } ,
	{ 0x114C, "Annabooks", "Annabooks" } ,
	{ 0x114D, "IC Corp.", "IC Corporation" } ,
	{ 0x114E, "Nikon", "Nikon Systems Inc" } ,
	{ 0x114F, "Digi", "Digi International" } ,
	{ 0x1150, "TMC", "Thinking Machines Corporation" } ,
	{ 0x1151, "JAE", "JAE Electronics Inc." } ,
	{ 0x1152, "Megatek", "Megatek" } ,
	{ 0x1153, "Land Win", "Land Win Electronic Corp" } ,
	{ 0x1154, "Melco", "Melco Inc" } ,
	{ 0x1155, "Pine", "Pine Technology Ltd" } ,
	{ 0x1156, "Periscope", "Periscope Engineering" } ,
	{ 0x1157, "Avsys", "Avsys Corporation" } ,
	{ 0x1158, "Voarx", "Voarx R&D Inc" } ,
	{ 0x1159, "Mutech", "Mutech" } ,
	{ 0x115A, "Harlequin", "Harlequin Ltd" } ,
	{ 0x115B, "Parallax", "Parallax Graphics" } ,
	{ 0x115C, "Photron", "Photron Ltd." } ,
	{ 0x115D, "Xircom", "Xircom" } ,
	{ 0x115E, "Peer", "Peer Protocols Inc" } ,
	{ 0x115F, "Maxtor", "Maxtor Corporation" } ,
	{ 0x1160, "Megasoft", "Megasoft Inc" } ,
	{ 0x1161, "PFU", "PFU Ltd" } ,
	{ 0x1162, "OA Lab", "OA Laboratory Co Ltd" } ,
	{ 0x1163, "Rendition", "Rendition Inc" } ,
	{ 0x1164, "APT", "Advanced Peripherals Tech" } ,
	{ 0x1165, "Imagraph", "Imagraph Corporation" } ,
	{ 0x1166, "RCC/ServerWorks", "Reliance Computer Corp./ServerWorks" } ,
	{ 0x1167, "Mutoh", "Mutoh Industries Inc" } ,
	{ 0x1168, "Thine", "Thine Electronics Inc" } ,
	{ 0x1169, "CDAC", "Centre f/Dev. of Adv. Computing" } ,
	{ 0x116A, "Polaris", "Polaris Communications" } ,
	{ 0x116B, "Connectware", "Connectware Inc" } ,
	{ 0x116C, "Int Res.", "Intelligent Resources" } ,
	{ 0x116E, "EFI", "Electronics for Imaging" } ,
	{ 0x116F, "WkSta. Tech.", "Workstation Technology" } ,
	{ 0x1170, "Inventec", "Inventec Corporation" } ,
	{ 0x1171, "Lough. Sound", "Loughborough Sound Images" } ,
	{ 0x1172, "Altera", "Altera Corporation" } ,
	{ 0x1173, "Adobe", "Adobe Systems" } ,
	{ 0x1174, "Bridgeport", "Bridgeport Machines" } ,
	{ 0x1175, "Mitron", "Mitron Computer Inc." } ,
	{ 0x1176, "SBE", "SBE" } ,
	{ 0x1177, "Silicon Eng.", "Silicon Engineering" } ,
	{ 0x1178, "Alfa", "Alfa Inc" } ,
	{ 0x1179, "Toshiba", "Toshiba America Info Systems" } ,
	{ 0x117A, "A-Trend", "A-Trend Technology" } ,
	{ 0x117B, "LG Elec.", "LG Electronics Inc." } ,
	{ 0x117C, "Atto", "Atto Technology" } ,
	{ 0x117D, "B&D", "Becton & Dickinson" } ,
	{ 0x117E, "T/R", "T/R Systems" } ,
	{ 0x117F, "ICS", "Integrated Circuit Systems" } ,
	{ 0x1180, "CAC", "Communication Automation Corporation" } ,
	{ 0x1181, "Telmatics", "Telmatics International" } ,
	{ 0x1183, "Fujikura", "Fujikura Ltd" } ,
	{ 0x1184, "Forks", "Forks Inc" } ,
	{ 0x1185, "Dataworld", "Dataworld" } ,
	{ 0x1186, "D-Link", "D-Link System Inc" } ,
	{ 0x1187, "ATL", "Advanced Technology Laboratories" } ,
	{ 0x1188, "Shima", "Shima Seiki Manufacturing Ltd." } ,
	{ 0x1189, "Matsushita", "Matsushita Electronics" } ,
	{ 0x118A, "Hilevel", "Hilevel Technology" } ,
	{ 0x118B, "Hypertec", "Hypertec Pty Ltd" } ,
	{ 0x118C, "Corollary", "Corollary Inc" } ,
	{ 0x118D, "BitFlow", "BitFlow Inc" } ,
	{ 0x118E, "Hermstedt", "Hermstedt AG" } ,
	{ 0x118F, "Green", "Green Logic" } ,
	{ 0x1190, "Tripace", "Tripace" } ,
	{ 0x1191, "ACARD", "ACARD Technology" } ,
	{ 0x1192, "Densan", "Densan Co. Ltd" } ,
	{ 0x1193, "Zeitnet", "Zeitnet Inc." } ,
	{ 0x1194, "Toucan", "Toucan Technology" } ,
	{ 0x1195, "Ratoc", "Ratoc System Inc" } ,
	{ 0x1196, "Hytec", "Hytec Electronics Ltd" } ,
	{ 0x1197, "Gage", "Gage Applied Sciences Inc." } ,
	{ 0x1198, "Lambda", "Lambda Systems Inc" } ,
	{ 0x1199, "Attachmate", "Attachmate Corp." } ,
	{ 0x119A, "Mind Share", "Mind Share Inc." } ,
	{ 0x119B, "Omega", "Omega Micro Inc." } ,
	{ 0x119C, "ITI", "Information Technology Inst." } ,
	{ 0x119D, "Bug", "Bug Sapporo Japan" } ,
	{ 0x119E, "Fujitsu", "Fujitsu Microelectronics Ltd." } ,
	{ 0x119F, "Bull", "Bull Hn Information Systems" } ,
	{ 0x11A0, "Convex", "Convex Computer Corporation" } ,
	{ 0x11A1, "Hamamatsu", "Hamamatsu Photonics K.K." } ,
	{ 0x11A2, "Sierra", "Sierra Research and Technology" } ,
	{ 0x11A3, "Deuretzbacher", "Deuretzbacher GmbH & Co. Eng. KG" } ,
	{ 0x11A4, "Barco", "Barco" } ,
	{ 0x11A5, "MicroUnity", "MicroUnity Systems Engineering Inc." } ,
	{ 0x11A6, "Pure Data", "Pure Data" } ,
	{ 0x11A7, "Power Comp.", "Power Computing Corp." } ,
	{ 0x11A8, "Systech", "Systech Corp." } ,
	{ 0x11A9, "InnoSys", "InnoSys Inc." } ,
	{ 0x11AA, "Actel", "Actel" } ,
	{ 0x11AB, "Marvell", "Marvell Semiconductor, Inc." } ,
	{ 0x11AC, "Canon", "Canon Information Systems" } ,
	{ 0x11AD, "Lite-On", "Lite-On Communications Inc" } ,
	{ 0x11AE, "Scitex", "Scitex Corporation Ltd" } ,
	{ 0x11AF, "Avid", "Avid Technology Inc." } ,
	{ 0x11B0, "V3", "V3 Semiconductor Inc." } ,
	{ 0x11B1, "Apricot", "Apricot Computers" } ,
	{ 0x11B2, "Kodak", "Eastman Kodak" } ,
	{ 0x11B3, "Barr", "Barr Systems Inc." } ,
	{ 0x11B4, "Leitch", "Leitch Technology International" } ,
	{ 0x11B5, "Radstone", "Radstone Technology Plc" } ,
	{ 0x11B6, "United Video", "United Video Corp" } ,
	{ 0x11B7, "Motorola", "Motorola" } ,
	{ 0x11B8, "Xpoint", "Xpoint Technologies Inc" } ,
	{ 0x11B9, "Pathlight", "Pathlight Technology Inc." } ,
	{ 0x11BA, "Videotron", "Videotron Corp" } ,
	{ 0x11BB, "Pyramid", "Pyramid Technology" } ,
	{ 0x11BC, "Net. Periph.", "Network Peripherals Inc" } ,
	{ 0x11BD, "Pinnacle", "Pinnacle Systems Inc." } ,
	{ 0x11BE, "IMI", "International Microcircuits Inc" } ,
	{ 0x11BF, "Astrodesign", "Astrodesign Inc." } ,
	{ 0x11C0, "H-P", "Hewlett-Packard" } ,
	{ 0x11C1, "Agere", "Agere Systems" } ,
	{ 0x11C2, "Sand", "Sand Microelectronics" } ,
	{ 0x11C3, "NEC", "NEC Corporation" } ,
	{ 0x11C4, "Doc. Tech.", "Document Technologies Ind." } ,
	{ 0x11C5, "Shiva", "Shiva Corporatin" } ,
	{ 0x11C6, "Dainippon", "Dainippon Screen Mfg. Co" } ,
	{ 0x11C7, "D.C.M.", "D.C.M. Data Systems" } ,
	{ 0x11C8, "Dolphin", "Dolphin Interconnect Solutions" } ,
	{ 0x11C9, "MAGMA", "MAGMA" } ,
	{ 0x11CA, "LSI Sys.", "LSI Systems Inc" } ,
	{ 0x11CB, "Specialix", "Specialix International Ltd." } ,
	{ 0x11CC, "M&K", "Michels & Kleberhoff Computer GmbH" } ,
	{ 0x11CD, "HAL", "HAL Computer Systems Inc." } ,
	{ 0x11CE, "PRI", "Primary Rate Inc" } ,
	{ 0x11CF, "PEC", "Pioneer Electronic Corporation" } ,
	{ 0x11D0, "BAE", "BAE SYSTEMS - Manassas" } ,
	{ 0x11D1, "AuraVision", "AuraVision Corporation" } ,
	{ 0x11D2, "Intercom", "Intercom Inc." } ,
	{ 0x11D3, "Trancell", "Trancell Systems Inc" } ,
	{ 0x11D4, "ADI", "Analog Devices, Inc." } ,
	{ 0x11D5, "Ikon", "Ikon Corp" } ,
	{ 0x11D6, "Tekelec", "Tekelec Technologies" } ,
	{ 0x11D7, "Trenton", "Trenton Terminals Inc" } ,
	{ 0x11D8, "ITD", "Image Technologies Development" } ,
	{ 0x11D9, "Tec", "Tec Corporation" } ,
	{ 0x11DA, "Novell", "Novell" } ,
	{ 0x11DB, "Sega", "Sega Enterprises Ltd" } ,
	{ 0x11DC, "Questra", "Questra Corp" } ,
	{ 0x11DD, "Crosfield", "Crosfield Electronics Ltd" } ,
	{ 0x11DE, "Zoran", "Zoran Corporation" } ,
	{ 0x11DF, "New Wave", "New Wave Pdg" } ,
	{ 0x11E0, "Cray", "Cray Communications A/S" } ,
	{ 0x11E1, "Gec Plessey", "Gec Plessey Semi Inc" } ,
	{ 0x11E2, "Samsung", "Samsung Information Systems America" } ,
	{ 0x11E3, "Quicklogic", "Quicklogic Corp" } ,
	{ 0x11E4, "Second Wave", "Second Wave Inc" } ,
	{ 0x11E5, "IIX", "IIX Consulting" } ,
	{ 0x11E6, "Mitsui", "Mitsui-Zosen System Research" } ,
	{ 0x11E7, "Toshiba", "Toshiba America Elec. Co" } ,
	{ 0x11E8, "DPSI", "Digital Processing Systems Inc" } ,
	{ 0x11E9, "Highwater", "Highwater Designs Ltd" } ,
	{ 0x11EA, "Elsag", "Elsag Bailey" } ,
	{ 0x11EB, "Formation", "Formation, Inc" } ,
	{ 0x11EC, "Coreco", "Coreco Inc" } ,
	{ 0x11ED, "Mediamatics", "Mediamatics" } ,
	{ 0x11EE, "Dome", "Dome Imaging Systems Inc" } ,
	{ 0x11EF, "Nicolet", "Nicolet Technologies BV" } ,
	{ 0x11F0, "Compu-Shack", "Compu-Shack GmbH" } ,
	{ 0x11F1, "Symbios", "Symbios Logic Inc" } ,
	{ 0x11F2, "Pic-Tel", "Picture Tel Japan KK" } ,
	{ 0x11F3, "Keithley", "Keithley Metrabyte" } ,
	{ 0x11F4, "Kinetic", "Kinetic Systems Corporation" } ,
	{ 0x11F5, "Comp Dev", "Computing Devices Intl" } ,
	{ 0x11F6, "Powermatic", "Powermatic Data Systems Ltd" } ,
	{ 0x11F7, "S-A", "Scientific Atlanta" } ,
	{ 0x11F8, "PMC-Sierra", "PMC-Sierra Inc." } ,
	{ 0x11F9, "I-Cube", "I-Cube Inc" } ,
	{ 0x11FA, "Kasan", "Kasan Electronics Co Ltd" } ,
	{ 0x11FB, "Datel", "Datel Inc" } ,
	{ 0x11FC, "Silicon Magic", "Silicon Magic" } ,
	{ 0x11FD, "High Street", "High Street Consultants" } ,
	{ 0x11FE, "Comtrol", "Comtrol Corp" } ,
	{ 0x11FF, "Scion", "Scion Corp" } ,
	{ 0x1200, "CSS", "CSS Corp" } ,
	{ 0x1201, "Vista", "Vista Controls Corp" } ,
	{ 0x1202, "Network Gen", "Network General Corp" } ,
	{ 0x1203, "Agfa", "Bayer Corporation Agfa Div" } ,
	{ 0x1204, "Lattice", "Lattice Semiconductor Corp" } ,
	{ 0x1205, "Array", "Array Corp" } ,
	{ 0x1206, "Amdahl", "Amdahl Corp" } ,
	{ 0x1208, "Parsytec", "Parsytec GmbH" } ,
	{ 0x1209, "Sci Sys", "Sci Systems Inc" } ,
	{ 0x120A, "Synaptel", "Synaptel" } ,
	{ 0x120B, "Adaptive", "Adaptive Solutions" } ,
	{ 0x120D, "Comp Labs", "Compression Labs Inc." } ,
	{ 0x120E, "Cyclades", "Cyclades Corporation" } ,
	{ 0x120F, "Essential", "Essential Communications" } ,
	{ 0x1210, "Hyperparallel", "Hyperparallel Technologies" } ,
	{ 0x1211, "Braintech", "Braintech Inc" } ,
	{ 0x1212, "Kingston", "Kingston Technology Corp" } ,
	{ 0x1213, "AISI", "Applied Intelligent Systems Inc" } ,
	{ 0x1214, "Perf Tech", "Performance Technologies Inc" } ,
	{ 0x1215, "Interware", "Interware Co Ltd" } ,
	{ 0x1216, "Purup Eskofot", "Purup-Eskofot A/S" } ,
	{ 0x1217, "O2Micro", "O2Micro Inc" } ,
	{ 0x1218, "Hybricon", "Hybricon Corp" } ,
	{ 0x1219, "First Virtual", "First Virtual Corp" } ,
	{ 0x121A, "3dfx", "3dfx Interactive Inc" } ,
	{ 0x121B, "ATM", "Advanced Telecommunications Modules" } ,
	{ 0x121C, "Nippon Texa", "Nippon Texa Co Ltd" } ,
	{ 0x121D, "Lippert", "Lippert Automationstechnik GmbH" } ,
	{ 0x121E, "CSPI", "CSPI" } ,
	{ 0x121F, "Arcus", "Arcus Technology Inc" } ,
	{ 0x1220, "Ariel", "Ariel Corporation" } ,
	{ 0x1221, "Contec", "Contec Co Ltd" } ,
	{ 0x1222, "Ancor", "Ancor Communications Inc" } ,
	{ 0x1223, "Heurikon", "Heurikon/Computer Products" } ,
	{ 0x1224, "Int. Img.", "Interactive Images" } ,
	{ 0x1225, "Power IO", "Power I/O Inc." } ,
	{ 0x1227, "Tech-Source", "Tech-Source" } ,
	{ 0x1228, "Norsk", "Norsk Elektro Optikk A/S" } ,
	{ 0x1229, "Data Kin", "Data Kinesis Inc." } ,
	{ 0x122A, "Int. Telecom", "Integrated Telecom" } ,
	{ 0x122B, "LG Ind.", "LG Industrial Systems Co. Ltd." } ,
	{ 0x122C, "sci-worx", "sci-worx GmbH" } ,
	{ 0x122D, "Aztech", "Aztech System Ltd" } ,
	{ 0x122E, "Xyratex", "Xyratex" } ,
	{ 0x122F, "Andrew", "Andrew Corp." } ,
	{ 0x1230, "Fishcamp", "Fishcamp Engineering" } ,
	{ 0x1231, "WMI", "Woodward McCoach Inc." } ,
	{ 0x1232, "GPT", "GPT Ltd." } ,
	{ 0x1233, "Bus-Tech", "Bus-Tech Inc." } ,
	{ 0x1234, "Technical", "Technical Corp" } ,
	{ 0x1235, "Risq Mod", "Risq Modular Systems Inc." } ,
	{ 0x1236, "Sigma Designs", "Sigma Designs, Inc" } ,
	{ 0x1237, "Alta Tech", "Alta Technology Corp." } ,
	{ 0x1238, "Adtran", "Adtran" } ,
	{ 0x1239, "3DO", "The 3DO Company" } ,
	{ 0x123A, "Visicom", "Visicom Laboratories Inc." } ,
	{ 0x123B, "Seeq", "Seeq Technology Inc." } ,
	{ 0x123C, "Century Sys", "Century Systems Inc." } ,
	{ 0x123D, "EDT", "Engineering Design Team Inc." } ,
	{ 0x123F, "C-Cube", "C-Cube Microsystems" } ,
	{ 0x1240, "Marathon", "Marathon Technologies Corp." } ,
	{ 0x1241, "DSC", "DSC Communications" } ,
	{ 0x1242, "JNI", "JNI Corporation" } ,
	{ 0x1243, "Delphax", "Delphax" } ,
	{ 0x1244, "AVM", "AVM AUDIOVISUELLES MKTG & Computer GmbH" } ,
	{ 0x1245, "APD", "APD S.A." } ,
	{ 0x1246, "Dipix", "Dipix Technologies Inc" } ,
	{ 0x1247, "Xylon", "Xylon Research Inc." } ,
	{ 0x1248, "Central Data", "Central Data Corp." } ,
	{ 0x1249, "Samsung", "Samsung Electronics Co. Ltd." } ,
	{ 0x124A, "AEG", "AEG Electrocom GmbH" } ,
	{ 0x124B, "GreenSpring", "GreenSpring Computers" } ,
	{ 0x124C, "Solitron", "Solitron Technologies Inc." } ,
	{ 0x124D, "Stallion", "Stallion Technologies" } ,
	{ 0x124E, "Cylink", "Cylink" } ,
	{ 0x124F, "Infortrend", "Infortrend Technology Inc" } ,
	{ 0x1250, "Hitachi", "Hitachi Microcomputer System Ltd." } ,
	{ 0x1251, "VLSI Sol.", "VLSI Solution OY" } ,
	{ 0x1253, "Guzik", "Guzik Technical Enterprises" } ,
	{ 0x1254, "Linear Systems", "Linear Systems Ltd." } ,
	{ 0x1255, "Optibase", "Optibase Ltd." } ,
	{ 0x1256, "Perceptive", "Perceptive Solutions Inc." } ,
	{ 0x1257, "Vertex", "Vertex Networks Inc." } ,
	{ 0x1258, "Gilbarco", "Gilbarco Inc." } ,
	{ 0x1259, "Allied Tsyn", "Allied Telesyn International" } ,
	{ 0x125A, "ABB Pwr", "ABB Power Systems" } ,
	{ 0x125B, "Asix", "Asix Electronics Corp." } ,
	{ 0x125C, "Aurora", "Aurora Technologies Inc." } ,
	{ 0x125D, "ESS", "ESS Technology" } ,
	{ 0x125E, "Specvideo", "Specialvideo Engineering SRL" } ,
	{ 0x125F, "Concurrent", "Concurrent Technologies Inc." } ,
	{ 0x1260, "Intersil", "Intersil Corporation" } ,
	{ 0x1261, "Matsushita", "Matsushita-Kotobuki Electronics Indu" } ,
	{ 0x1262, "ES Comp.", "ES Computer Co. Ltd." } ,
	{ 0x1263, "Sonic Sol.", "Sonic Solutions" } ,
	{ 0x1264, "Aval Nag.", "Aval Nagasaki Corp." } ,
	{ 0x1265, "Casio", "Casio Computer Co. Ltd." } ,
	{ 0x1266, "Microdyne", "Microdyne Corp." } ,
	{ 0x1267, "SA Telecom", "S.A. Telecommunications" } ,
	{ 0x1268, "Tektronix", "Tektronix" } ,
	{ 0x1269, "Thomson-CSF", "Thomson-CSF/TTM" } ,
	{ 0x126A, "Lexmark", "Lexmark International Inc." } ,
	{ 0x126B, "Adax", "Adax Inc." } ,
	{ 0x126C, "Nortel", "Nortel Networks Corp." } ,
	{ 0x126D, "Splash", "Splash Technology Inc." } ,
	{ 0x126E, "Sumitomo", "Sumitomo Metal Industries Ltd." } ,
	{ 0x126F, "Sil Motion", "Silicon Motion" } ,
	{ 0x1270, "Olympus", "Olympus Optical Co. Ltd." } ,
	{ 0x1271, "GW Instr.", "GW Instruments" } ,
	{ 0x1272, "Telematics", "Telematics International" } ,
	{ 0x1273, "Hughes", "Hughes Network Systems" } ,
	{ 0x1274, "Ensoniq", "Ensoniq" } ,
	{ 0x1275, "NetApp", "Network Appliance" } ,
	{ 0x1276, "Sw Net Tech", "Switched Network Technologies Inc." } ,
	{ 0x1277, "Comstream", "Comstream" } ,
	{ 0x1278, "Transtech", "Transtech Parallel Systems" } ,
	{ 0x1279, "Transmeta", "Transmeta Corp." } ,
	{ 0x127A, "Conexant", "Conexant Systems" } ,
	{ 0x127B, "Pixera", "Pixera Corp" } ,
	{ 0x127C, "Crosspoint", "Crosspoint Solutions Inc." } ,
	{ 0x127D, "Vela Res", "Vela Research" } ,
	{ 0x127E, "Winnow", "Winnov L.P." } ,
	{ 0x127F, "Fujifilm", "Fujifilm" } ,
	{ 0x1280, "Photoscript", "Photoscript Group Ltd." } ,
	{ 0x1281, "Yokogawa", "Yokogawa Electronic Corp." } ,
	{ 0x1282, "Davicom", "Davicom Semiconductor Inc." } ,
	{ 0x1283, "ITExpress", "Integrated Technology Express Inc." } ,
	{ 0x1284, "Sahara", "Sahara Networks Inc." } ,
	{ 0x1285, "Plat Tech", "Platform Technologies Inc." } ,
	{ 0x1286, "Mazet", "Mazet GmbH" } ,
	{ 0x1287, "LuxSonor", "LuxSonor Inc." } ,
	{ 0x1288, "Timestep", "Timestep Corp." } ,
	{ 0x1289, "AVC Tech", "AVC Technology Inc." } ,
	{ 0x128A, "Asante", "Asante Technologies Inc." } ,
	{ 0x128B, "Transwitch", "Transwitch Corp." } ,
	{ 0x128C, "Retix", "Retix Corp." } ,
	{ 0x128D, "G2 Net", "G2 Networks Inc." } ,
	{ 0x128E, "Samho", "Samho Multi Tech Ltd." } ,
	{ 0x128F, "Tateno", "Tateno Dennou Inc." } ,
	{ 0x1290, "Sord", "Sord Computer Corp." } ,
	{ 0x1291, "NCS Comp", "NCS Computer Italia" } ,
	{ 0x1292, "Tritech", "Tritech Microelectronics Intl PTE" } ,
	{ 0x1293, "M Reality", "Media Reality Technology" } ,
	{ 0x1294, "Rhetorex", "Rhetorex Inc." } ,
	{ 0x1295, "Imagenation", "Imagenation Corp." } ,
	{ 0x1296, "Kofax", "Kofax Image Products" } ,
	{ 0x1297, "Holco", "Holco Enterprise" } ,
	{ 0x1298, "Spellcaster", "Spellcaster Telecommunications Inc." } ,
	{ 0x1299, "Know Tech", "Knowledge Technology Laboratories" } ,
	{ 0x129A, "VMETRO", "VMETRO Inc." } ,
	{ 0x129B, "Img Access", "Image Access" } ,
	{ 0x129D, "CompCore", "CompCore Multimedia Inc." } ,
	{ 0x129E, "Victor Jpn", "Victor Co. of Japan Ltd." } ,
	{ 0x129F, "OEC Med", "OEC Medical Systems Inc." } ,
	{ 0x12A0, "A-B", "Allen Bradley Co." } ,
	{ 0x12A1, "Simpact", "Simpact Inc" } ,
	{ 0x12A2, "NewGen", "NewGen Systems Corp." } ,
	{ 0x12A3, "Lucent", "Lucent Technologies AMR" } ,
	{ 0x12A4, "NTT Elect", "NTT Electronics Technology Co." } ,
	{ 0x12A5, "Vision Dyn", "Vision Dynamics Ltd." } ,
	{ 0x12A6, "Scalable", "Scalable Networks Inc." } ,
	{ 0x12A7, "AMO", "AMO GmbH" } ,
	{ 0x12A8, "News Datacom", "News Datacom" } ,
	{ 0x12A9, "Xiotech", "Xiotech Corp." } ,
	{ 0x12AA, "SDL", "SDL Communications Inc." } ,
	{ 0x12AB, "Yuan Yuan", "Yuan Yuan Enterprise Co. Ltd." } ,
	{ 0x12AC, "MeasureX", "MeasureX Corp." } ,
	{ 0x12AD, "Multidata", "Multidata GmbH" } ,
	{ 0x12AE, "Alteon", "Alteon Networks Inc." } ,
	{ 0x12AF, "TDK USA", "TDK USA Corp." } ,
	{ 0x12B0, "Jorge Sci", "Jorge Scientific Corp." } ,
	{ 0x12B1, "GammaLink", "GammaLink" } ,
	{ 0x12B2, "Gen Signal", "General Signal Networks" } ,
	{ 0x12B3, "Inter-Face", "Inter-Face Co. Ltd." } ,
	{ 0x12B4, "Future Tel", "Future Tel Inc." } ,
	{ 0x12B5, "Granite", "Granite Systems Inc." } ,
	{ 0x12B6, "Nat Micro", "Natural Microsystems" } ,
	{ 0x12B7, "Acumen", "Acumen" } ,
	{ 0x12B8, "Korg", "Korg" } ,
	{ 0x12B9, "US Robotics", "US Robotics" } ,
	{ 0x12BA, "Bittware", "Bittware, Inc" } ,
	{ 0x12BB, "Nippon Uni", "Nippon Unisoft Corp." } ,
	{ 0x12BC, "Array Micro", "Array Microsystems" } ,
	{ 0x12BD, "Computerm", "Computerm Corp." } ,
	{ 0x12BE, "Anchor Chips", "Anchor Chips Inc." } ,
	{ 0x12BF, "Fujifilm", "Fujifilm Microdevices" } ,
	{ 0x12C0, "Infimed", "Infimed" } ,
	{ 0x12C1, "GMM Res", "GMM Research Corp." } ,
	{ 0x12C2, "Mentec", "Mentec Ltd." } ,
	{ 0x12C3, "Holtek", "Holtek Microelectronics Inc." } ,
	{ 0x12C4, "Connect Tech", "Connect Tech Inc." } ,
	{ 0x12C5, "PicturEl", "Picture Elements Inc." } ,
	{ 0x12C6, "Mitani", "Mitani Corp." } ,
	{ 0x12C7, "Dialogic", "Dialogic Corp." } ,
	{ 0x12C8, "G Force", "G Force Co. Ltd." } ,
	{ 0x12C9, "Gigi Ops", "Gigi Operations" } ,
	{ 0x12CA, "ICE", "Integrated Computing Engines, Inc." } ,
	{ 0x12CB, "Antex", "Antex Electronics Corp." } ,
	{ 0x12CC, "Pluto", "Pluto Technologies International" } ,
	{ 0x12CD, "Aims Lab", "Aims Lab" } ,
	{ 0x12CE, "Netspeed", "Netspeed Inc." } ,
	{ 0x12CF, "Prophet", "Prophet Systems Inc." } ,
	{ 0x12D0, "GDE Sys", "GDE Systems Inc." } ,
	{ 0x12D1, "PsiTech", "PsiTech" } ,
	{ 0x12D2, "NVidia", "NVidia / SGS Thomson" } ,
	{ 0x12D3, "Vingmed", "Vingmed Sound A/S" } ,
	{ 0x12D4, "DGM&S", "DGM & S" } ,
	{ 0x12D5, "Equator", "Equator Technologies" } ,
	{ 0x12D6, "Analogic", "Analogic Corp." } ,
	{ 0x12D7, "Biotronic", "Biotronic SRL" } ,
	{ 0x12D8, "Pericom", "Pericom Semiconductor" } ,
	{ 0x12D9, "Aculab", "Aculab Plc." } ,
	{ 0x12DA, "TrueTime", "TrueTime" } ,
	{ 0x12DB, "Annapolis", "Annapolis Micro Systems Inc." } ,
	{ 0x12DC, "Symicron", "Symicron Computer Communication Ltd." } ,
	{ 0x12DD, "MGI", "Management Graphics Inc." } ,
	{ 0x12DE, "Rainbow", "Rainbow Technologies" } ,
	{ 0x12DF, "SBS Tech", "SBS Technologies Inc." } ,
	{ 0x12E0, "Chase", "Chase Research PLC" } ,
	{ 0x12E1, "Nintendo", "Nintendo Co. Ltd." } ,
	{ 0x12E2, "Datum", "Datum Inc. Bancomm-Timing Division" } ,
	{ 0x12E3, "Imation", "Imation Corp. - Medical Imaging Syst" } ,
	{ 0x12E4, "Brooktrout", "Brooktrout Technology Inc." } ,
	{ 0x12E6, "Cirel", "Cirel Systems" } ,
	{ 0x12E7, "Sebring", "Sebring Systems Inc" } ,
	{ 0x12E8, "CRISC", "CRISC Corp." } ,
	{ 0x12E9, "GE Spacenet", "GE Spacenet" } ,
	{ 0x12EA, "Zuken", "Zuken" } ,
	{ 0x12EB, "Aureal", "Aureal Semiconductor" } ,
	{ 0x12EC, "3A Intl", "3A International Inc." } ,
	{ 0x12ED, "Optivision", "Optivision Inc." } ,
	{ 0x12EE, "Orange Micro", "Orange Micro, Inc." } ,
	{ 0x12EF, "Vienna", "Vienna Systems" } ,
	{ 0x12F0, "Pentek", "Pentek" } ,
	{ 0x12F1, "Sorenson", "Sorenson Vision Inc." } ,
	{ 0x12F2, "Gammagraphx", "Gammagraphx Inc." } ,
	{ 0x12F4, "Megatel", "Megatel" } ,
	{ 0x12F5, "Forks", "Forks" } ,
	{ 0x12F6, "Dawson Fr", "Dawson France" } ,
	{ 0x12F7, "Cognex", "Cognex" } ,
	{ 0x12F8, "Electronic-Design", "Electronic-Design GmbH" } ,
	{ 0x12F9, "FFT", "FourFold Technologies" } ,
	{ 0x12FB, "SSP", "Spectrum Signal Processing" } ,
	{ 0x12FC, "", "Capital Equipment Corp" } ,
	{ 0x12FE, "esd", "esd Electronic System Design GmbH" } ,
	{ 0x1303, "", "Innovative Integration" } ,
	{ 0x1304, "", "Juniper Networks Inc." } ,
	{ 0x1307, "ComputerBoards", "ComputerBoards" } ,
	{ 0x1308, "Jato", "Jato Technologies Inc." } ,
	{ 0x130A, "", "Mitsubishi Electric Microcomputer" } ,
	{ 0x130B, "", "Colorgraphic Communications Corp" } ,
	{ 0x130F, "", "Advanet Inc." } ,
	{ 0x1310, "", "Gespac" } ,
	{ 0x1312, "RVSI", "Robotic Vision Systems Incorporated" } ,
	{ 0x1313, "", "Yaskawa Electric Co." } ,
	{ 0x1316, "", "Teradyne Inc." } ,
	{ 0x1317, "", "Admtek Inc" } ,
	{ 0x1318, "Packet Engines", "Packet Engines, Inc." } ,
	{ 0x1319, "Forte Media", "Forte Media, Inc." } ,
	{ 0x131F, "", "SIIG" } ,
	{ 0x1325, "", "Salix Technologies Inc" } ,
	{ 0x1326, "", "Seachange International" } ,
	{ 0x1331, "RadiSys", "RadiSys Corporation" } ,
	{ 0x1332, "Micro Memory", "Micro Memory, LLC" } ,
	{ 0x1335, "Videomail", "Videomail Inc." } ,
	{ 0x133D, "", "Prisa Networks" } ,
	{ 0x133F, "", "SCM Microsystems" } ,
	{ 0x1342, "", "Promax Systems Inc" } ,
	{ 0x1344, "Micron", "Micron Technology, Inc." } ,
	{ 0x1347, "Odetics", "Odetics" } ,
	{ 0x134A, "DTC", "DTC Technology Corp." } ,
	{ 0x134B, "", "ARK Research Corp." } ,
	{ 0x134C, "", "Chori Joho System Co. Ltd" } ,
	{ 0x134D, "PCTEL", "PCTEL Inc." } ,
	{ 0x135A, "", "Brain Boxes Limited" } ,
	{ 0x135B, "", "Giganet Inc." } ,
	{ 0x135C, "", "Quatech Inc" } ,
	{ 0x135D, "ABB Network Partn", "ABB Network Partner AB" } ,
	{ 0x135E, "Sealevel", "Sealevel Systems Inc." } ,
	{ 0x135F, "", "I-Data International A-S" } ,
	{ 0x1360, "", "Meinberg Funkuhren" } ,
	{ 0x1361, "", "Soliton Systems K.K." } ,
	{ 0x1363, "", "Phoenix Technologies Ltd" } ,
	{ 0x1365, "Hypercope", "Hypercope Corp." } ,
	{ 0x1366, "Teijin", "Teijin Seiki Co. Ltd." } ,
	{ 0x1367, "", "Hitachi Zosen Corporation" } ,
	{ 0x1368, "", "Skyware Corporation" } ,
	{ 0x1369, "", "Digigram" } ,
	{ 0x136B, "", "Kawasaki Steel Corporation" } ,
	{ 0x136C, "", "Adtek System Science Co Ltd" } ,
	{ 0x1375, "", "Boeing - Sunnyvale" } ,
	{ 0x1377, "", "GMBH" } ,
	{ 0x137A, "", "Mark Of The Unicorn Inc" } ,
	{ 0x137B, "", "PPT Vision" } ,
	{ 0x137C, "", "Iwatsu Electric Co Ltd" } ,
	{ 0x137D, "", "Dynachip Corporation" } ,
	{ 0x137E, "PTSC", "Patriot Scientific Corp." } ,
	{ 0x1380, "", "Sanritz Automation Co LTC" } ,
	{ 0x1381, "", "Brains Co. Ltd" } ,
	{ 0x1382, "Marian", "Marian - Electronic & Software" } ,
	{ 0x1384, "", "Stellar Semiconductor Inc" } ,
	{ 0x1385, "Netgear", "Netgear" } ,
	{ 0x1387, "", "Systran Corp" } ,
	{ 0x1388, "", "Hitachi Information Technology Co Ltd" } ,
	{ 0x1389, "Applicom", "Applicom International" } ,
	{ 0x138B, "", "Tokimec Inc" } ,
	{ 0x138E, "", "Basler GMBH" } ,
	{ 0x138F, "", "Patapsco Designs Inc" } ,
	{ 0x1390, "CDI", "Concept Development Inc." } ,
	{ 0x1393, "", "Moxa Technologies Co Ltd" } ,
	{ 0x1394, "Level One", "Level One Communications" } ,
	{ 0x1395, "", "Ambicom Inc" } ,
	{ 0x1396, "", "Cipher Systems Inc" } ,
	{ 0x1397, "Cologne", "Cologne Chip Designs GmbH" } ,
	{ 0x1398, "", "Clarion Co. Ltd" } ,
	{ 0x139A, "", "Alacritech Inc" } ,
	{ 0x139D, "", "Xstreams PLC/ EPL Limited" } ,
	{ 0x139E, "", "Echostar Data Networks" } ,
	{ 0x13A0, "", "Crystal Group Inc" } ,
	{ 0x13A1, "", "Kawasaki Heavy Industries Ltd" } ,
	{ 0x13A3, "HI-FN", "HI-FN Inc." } ,
	{ 0x13A4, "", "Rascom Inc" } ,
	{ 0x13A7, "", "Teles AG" } ,
	{ 0x13A8, "XR", "Exar Corp." } ,
	{ 0x13A9, "", "Siemens Medical Solutions" } ,
	{ 0x13AA, "", "Nortel Networks - BWA Division" } ,
	{ 0x13AF, "", "T.Sqware" } ,
	{ 0x13B1, "", "Tamura Corporation" } ,
	{ 0x13B4, "", "Wellbean Co Inc" } ,
	{ 0x13B5, "", "ARM Ltd" } ,
	{ 0x13B6, "", "DLoG GMBH" } ,
	{ 0x13B8, "", "Nokia Telecommunications OY" } ,
	{ 0x13BD, "SHARP", "Sharp Corporation" } ,
	{ 0x13BF, "", "Sharewave Inc" } ,
	{ 0x13C0, "Microgate", "Microgate Corp." } ,
	{ 0x13C1, "3ware", "3ware Inc." } ,
	{ 0x13C2, "", "Technotrend Systemtechnik GMBH" } ,
	{ 0x13C3, "", "Janz Computer AG" } ,
	{ 0x13C7, "", "Blue Chip Technology Ltd" } ,
	{ 0x13CC, "", "Metheus Corporation" } ,
	{ 0x13CF, "", "Studio Audio & Video Ltd" } ,
	{ 0x13D0, "", "B2C2 Inc" } ,
	{ 0x13D1, "", "Abocom Systems Inc" } ,
	{ 0x13D4, "", "Graphics Microsystems Inc" } ,
	{ 0x13D6, "", "K.I. Technology Co Ltd" } ,
	{ 0x13D7, "", "Toshiba Engineering Corporation" } ,
	{ 0x13D8, "", "Phobos Corporation" } ,
	{ 0x13D9, "", "Apex Inc" } ,
	{ 0x13DC, "", "Netboost Corporation" } ,
	{ 0x13DE, "", "ABB Robotics Products AB" } ,
	{ 0x13DF, "E-Tech", "E-Tech Inc." } ,
	{ 0x13E0, "", "GVC Corporation" } ,
	{ 0x13E3, "", "Nest Inc" } ,
	{ 0x13E4, "", "Calculex Inc" } ,
	{ 0x13E5, "", "Telesoft Design Ltd" } ,
	{ 0x13E9, "", "Intraserver Technology Inc" } ,
	{ 0x13EA, "", "Dallas Semiconductor" } ,
	{ 0x13F0, "", "Sundance Technology Inc" } ,
	{ 0x13F1, "", "OCE - Industries S.A." } ,
	{ 0x13F4, "", "Troika Networks Inc" } ,
	{ 0x13F6, "C-Media", "C-Media Electronics Inc." } ,
	{ 0x13F9, "", "NTT Advanced Technology Corp." } ,
	{ 0x13FA, "Pentland", "Pentland Systems Ltd." } ,
	{ 0x13FB, "", "Aydin Corp" } ,
	{ 0x13FD, "", "Micro Science Inc" } ,
	{ 0x13FE, "Advantech", "Advantech Co., Ltd." } ,
	{ 0x13FF, "", "Silicon Spice Inc." } ,
	{ 0x1400, "", "ARTX Inc" } ,
	{ 0x1402, "Meilhaus Electronic", "Meilhaus Electronic GmbH Germany" } ,
	{ 0x1404, "", "Fundamental Software Inc" } ,
	{ 0x1406, "Oc�", "Oc� Printing Systems" } ,
	{ 0x1407, "LAVA", "Lava Computer MFG Inc." } ,
	{ 0x1408, "", "Aloka Co. Ltd" } ,
	{ 0x1409, "", "eTIMedia Technology Co Ltd" } ,
	{ 0x140A, "", "DSP Research Inc" } ,
	{ 0x140B, "", "Ramix Inc" } ,
	{ 0x140D, "", "Matsushita Electric Works Ltd" } ,
	{ 0x140F, "", "Salient Systems Corp" } ,
	{ 0x1412, "IC Ensemble", "IC Ensemble, Inc." } ,
	{ 0x1413, "", "Addonics" } ,
	{ 0x1415, "Oxford", "Oxford Semiconductor Ltd" } ,
	{ 0x1418, "", "Kyushu Electronics Systems Inc" } ,
	{ 0x1419, "", "Excel Switching Corp" } ,
	{ 0x141B, "", "Zoom Telephonics Inc" } ,
	{ 0x141C, "Zoom", "Zoom Telephonics, Inc" } ,
	{ 0x141E, "", "Fanuc Co. Ltd" } ,
	{ 0x141F, "", "Visiontech Ltd" } ,
	{ 0x1420, "", "Psion Dacom PLC" } ,
	{ 0x1425, "", "ASIC Designers Inc" } ,
	{ 0x1428, "", "Edec Co Ltd" } ,
	{ 0x1429, "", "Unex Technology Corp." } ,
	{ 0x142A, "", "Kingmax Technology Inc" } ,
	{ 0x142B, "", "Radiolan" } ,
	{ 0x142C, "", "Minton Optic Industry Co Ltd" } ,
	{ 0x142D, "", "Pixstream Inc" } ,
	{ 0x1430, "", "ITT Aerospace/Communications Division" } ,
	{ 0x1433, "", "Eltec Elektronik AG" } ,
	{ 0x1435, "RTD-USA", "Real Time Devices USA, Inc." } ,
	{ 0x1436, "", "CIS Technology Inc" } ,
	{ 0x1437, "", "Nissin Inc Co" } ,
	{ 0x1438, "", "Atmel-Dream" } ,
	{ 0x143F, "", "Lightwell Co Ltd - Zax Division" } ,
	{ 0x1441, "", "Agie SA." } ,
	{ 0x1445, "", "Logical Co Ltd" } ,
	{ 0x1446, "", "Graphin Co. Ltd" } ,
	{ 0x1447, "", "Aim GMBH" } ,
	{ 0x1448, "Alesis", "Alesis Studio" } ,
	{ 0x144A, "ADLINK", "ADLINK Technology Inc" } ,
	{ 0x144B, "Loronix", "Loronix Information Systems, Inc." } ,
	{ 0x144D, "", "Samsung Electronics Co Ltd" } ,
	{ 0x1450, "", "Octave Communications Ind." } ,
	{ 0x1451, "", "SP3D Chip Design GMBH" } ,
	{ 0x1453, "", "Mycom Inc" } ,
	{ 0x1455, "", "Logic Plus PLUS Inc" } ,
	{ 0x1458, "Giga-Byte", "Giga-Byte Technologies" } ,
	{ 0x145C, "", "Cryptek" } ,
	{ 0x145F, "Baldor", "Baldor Electric Company" } ,
	{ 0x1460, "", "Dynarc Inc" } ,
	{ 0x1462, "", "Micro-Star International Co Ltd" } ,
	{ 0x1463, "", "Fast Corporation" } ,
	{ 0x1464, "ICS", "Interactive Circuits & Systems Ltd" } ,
	{ 0x1465, "", "GN Nettest Telecom Div." } ,
	{ 0x1468, "", "Ambit Microsystems Corp." } ,
	{ 0x1469, "", "Cleveland Motion Controls" } ,
	{ 0x146C, "", "Ruby Tech Corp." } ,
	{ 0x146D, "", "Tachyon Inc." } ,
	{ 0x146E, "", "WMS Gaming" } ,
	{ 0x1471, "", "Integrated Telecom Express Inc" } ,
	{ 0x1473, "", "Zapex Technologies Inc" } ,
	{ 0x1474, "", "Doug Carson & Associates" } ,
	{ 0x1477, "", "Net Insight" } ,
	{ 0x1478, "", "Diatrend Corporation" } ,
	{ 0x147B, "", "Abit Computer Corp." } ,
	{ 0x147F, "", "Nihon Unisys Ltd." } ,
	{ 0x1482, "", "Isytec - Integrierte Systemtechnik Gmbh" } ,
	{ 0x1483, "", "Labway Coporation" } ,
	{ 0x1485, "", "Erma - Electronic GMBH" } ,
	{ 0x1489, "", "KYE Systems Corporation" } ,
	{ 0x148A, "", "Opto 22" } ,
	{ 0x148B, "", "Innomedialogic Inc." } ,
	{ 0x148D, "Digicom", "Digicom Systems Inc." } ,
	{ 0x148E, "", "OSI Plus Corporation" } ,
	{ 0x148F, "", "Plant Equipment Inc." } ,
	{ 0x1490, "", "TC Labs Pty Ltd." } ,
	{ 0x1493, "", "Maker Communications" } ,
	{ 0x1495, "", "Tokai Communications Industry Co. Ltd" } ,
	{ 0x1496, "", "Joytech Computer Co. Ltd." } ,
	{ 0x1497, "", "SMA Regelsysteme GMBH" } ,
	{ 0x1498, "Tews", "Tews Technologies" } ,
	{ 0x1499, "", "Micro-Technology Co Ltd" } ,
	{ 0x149B, "", "Seiko Instruments Inc" } ,
	{ 0x149E, "", "Mapletree Networks Inc." } ,
	{ 0x149F, "", "Lectron Co Ltd" } ,
	{ 0x14A0, "", "Softing GMBH" } ,
	{ 0x14A2, "", "Millennium Engineering Inc" } ,
	{ 0x14A4, "", "GVC/BCM Advanced Research" } ,
	{ 0x14A5, "", "Xionics Document Technologies Inc." } ,
	{ 0x14A9, "", "Hivertec Inc." } ,
	{ 0x14AB, "", "Mentor Graphics Corp." } ,
	{ 0x14B1, "", "Nextcom K.K." } ,
	{ 0x14B3, "Xpeed", "Xpeed Inc." } ,
	{ 0x14B4, "", "Philips Business Electronics B.V." } ,
	{ 0x14B5, "Creamware", "Creamware GmbH" } ,
	{ 0x14B6, "", "Quantum Data Corp." } ,
	{ 0x14B7, "Proxim", "Proxim Inc." } ,
	{ 0x14B9, "Aironet", "Aironet Wireless Communication" } ,
	{ 0x14BA, "", "Internix Inc." } ,
	{ 0x14BB, "", "Semtech Corporation" } ,
	{ 0x14BE, "", "L3 Communications" } ,
	{ 0x14C0, "Compal", "Compal Electronics, Inc." } ,
	{ 0x14C1, "", "Myricom Inc." } ,
	{ 0x14C2, "", "DTK Computer" } ,
	{ 0x14C4, "", "Iwasaki Information Systems Co Ltd" } ,
	{ 0x14C5, "", "ABB Automation Products AB" } ,
	{ 0x14C6, "", "Data Race Inc" } ,
	{ 0x14C7, "Modtech", "Modular Technology Ltd." } ,
	{ 0x14C8, "Turbocomm", "Turbocomm Tech Inc" } ,
	{ 0x14C9, "", "Odin Telesystems Inc" } ,
	{ 0x14CB, "", "Billionton Systems Inc./Cadmus Micro Inc" } ,
	{ 0x14CD, "", "Universal Scientific Ind." } ,
	{ 0x14CF, "", "TEK Microsystems Inc." } ,
	{ 0x14D2, "OX", "Oxford Semiconductor" } ,
	{ 0x14D4, "PANACOM", "Panacom Technology Corporation" } ,
	{ 0x14D5, "", "Nitsuko Corporation" } ,
	{ 0x14D6, "", "Accusys Inc" } ,
	{ 0x14D7, "", "Hirakawa Hewtech Corp" } ,
	{ 0x14D8, "", "Hopf Elektronik GMBH" } ,
	{ 0x14D9, "", "Alpha Processor Inc" } ,
	{ 0x14DB, "Avlab", "Avlab Technology Inc." } ,
	{ 0x14DC, "Amplicon", "Amplicon Liveline Limited" } ,
	{ 0x14DD, "", "Imodl Inc." } ,
	{ 0x14DE, "", "Applied Integration Corporation" } ,
	{ 0x14E3, "", "Amtelco" } ,
	{ 0x14E4, "Broadcom", "Broadcom Corporation" } ,
	{ 0x14EA, "Planex", "Planex Communications, Inc." } ,
	{ 0x14EB, "", "Seiko Epson Corporation" } ,
	{ 0x14EC, "", "Acqiris" } ,
	{ 0x14ED, "", "Datakinetics Ltd" } ,
	{ 0x14EF, "", "Carry Computer Eng. Co Ltd" } ,
	{ 0x14F1, "Conexant", "Conexant Systems, Inc." } ,
	{ 0x14F2, "Mobility", "Mobility Electronics, Inc." } ,
	{ 0x14F4, "", "Tokyo Electronic Industry Co. Ltd." } ,
	{ 0x14F5, "", "Sopac Ltd" } ,
	{ 0x14F6, "", "Coyote Technologies LLC" } ,
	{ 0x14F7, "", "Wolf Technology Inc" } ,
	{ 0x14F8, "", "Audiocodes Inc" } ,
	{ 0x14F9, "", "AG Communications" } ,
	{ 0x14FB, "", "Transas Marine (UK) Ltd" } ,
	{ 0x14FC, "", "Quadrics Supercomputers World" } ,
	{ 0x14FD, "", "Japan Computer Industry Inc." } ,
	{ 0x14FE, "", "Archtek Telecom Corp." } ,
	{ 0x14FF, "", "Twinhead International Corp." } ,
	{ 0x1500, "DELTA", "DELTA Electronics, Inc." } ,
	{ 0x1501, "", "Banksoft Canada Ltd" } ,
	{ 0x1502, "", "Mitsubishi Electric Logistics Support Co" } ,
	{ 0x1503, "", "Kawasaki LSI USA Inc" } ,
	{ 0x1504, "", "Kaiser Electronics" } ,
	{ 0x1506, "", "Chameleon Systems Inc" } ,
	{ 0x1507, "Htec", "Htec Ltd." } ,
	{ 0x1509, "", "First International Computer Inc" } ,
	{ 0x150B, "", "Yamashita Systems Corp" } ,
	{ 0x150C, "", "Kyopal Co Ltd" } ,
	{ 0x150D, "", "Warpspped Inc" } ,
	{ 0x150E, "", "C-Port Corporation" } ,
	{ 0x150F, "", "Intec GMBH" } ,
	{ 0x1510, "", "Behavior Tech Computer Corp" } ,
	{ 0x1511, "", "Centillium Technology Corp" } ,
	{ 0x1512, "", "Rosun Technologies Inc" } ,
	{ 0x1513, "", "Raychem" } ,
	{ 0x1514, "", "TFL LAN Inc" } ,
	{ 0x1515, "", "ICS Advent" } ,
	{ 0x1516, "", "Myson Technology Inc" } ,
	{ 0x1517, "", "Echotek Corporation" } ,
	{ 0x1518, "", "PEP Modular Computers GMBH" } ,
	{ 0x1519, "", "Telefon Aktiebolaget LM Ericsson" } ,
	{ 0x151A, "Globetek", "Globetek Inc." } ,
	{ 0x151B, "", "Combox Ltd" } ,
	{ 0x151C, "", "Digital Audio Labs Inc" } ,
	{ 0x151D, "", "Fujitsu Computer Products Of America" } ,
	{ 0x151E, "", "Matrix Corp." } ,
	{ 0x151F, "", "Topic Semiconductor Corp" } ,
	{ 0x1520, "", "Chaplet System Inc" } ,
	{ 0x1521, "", "Bell Corporation" } ,
	{ 0x1522, "Mainpine", "Mainpine Limited" } ,
	{ 0x1523, "", "Music Semiconductors" } ,
	{ 0x1524, "", "ENE Technology Inc" } ,
	{ 0x1525, "", "Impact Technologies" } ,
	{ 0x1526, "", "ISS Inc" } ,
	{ 0x1527, "", "Solectron" } ,
	{ 0x1528, "", "Acksys" } ,
	{ 0x1529, "", "American Microsystems Inc" } ,
	{ 0x152A, "", "Quickturn Design Systems" } ,
	{ 0x152B, "", "Flytech Technology Co Ltd" } ,
	{ 0x152C, "", "Macraigor Systems LLC" } ,
	{ 0x152D, "", "Quanta Computer Inc" } ,
	{ 0x152E, "", "Melec Inc" } ,
	{ 0x152F, "", "Philips - Crypto" } ,
	{ 0x1532, "", "Echelon Corporation" } ,
	{ 0x1533, "", "Baltimore" } ,
	{ 0x1534, "", "Road Corporation" } ,
	{ 0x1535, "", "Evergreen Technologies Inc" } ,
	{ 0x1537, "", "Datalex Communcations" } ,
	{ 0x1538, "", "Aralion Inc." } ,
	{ 0x1539, "", "Atelier Informatiques et Electronique Et" } ,
	{ 0x153A, "", "ONO Sokki" } ,
	{ 0x153B, "", "Terratec Electronic GMBH" } ,
	{ 0x153C, "", "Antal Electronic" } ,
	{ 0x153D, "", "Filanet Corporation" } ,
	{ 0x153E, "", "Techwell Inc" } ,
	{ 0x153F, "", "MIPS Denmark" } ,
	{ 0x1540, "", "Provideo Multimedia Co Ltd" } ,
	{ 0x1541, "", "Telocity Inc." } ,
	{ 0x1542, "", "Vivid Technology Inc" } ,
	{ 0x1543, "", "Silicon Laboratories" } ,
	{ 0x1544, "DCM", "DCM Technologies Ltd." } ,
	{ 0x1545, "", "Visiontek" } ,
	{ 0x1546, "", "IOI Technology Corp." } ,
	{ 0x1547, "", "Mitutoyo Corporation" } ,
	{ 0x1548, "", "Jet Propulsion Laboratory" } ,
	{ 0x1549, "ISS", "Interconnect Systems Solutions" } ,
	{ 0x154A, "", "Max Technologies Inc." } ,
	{ 0x154B, "", "Computex Co Ltd" } ,
	{ 0x154C, "", "Visual Technology Inc." } ,
	{ 0x154D, "", "PAN International Industrial Corp" } ,
	{ 0x154E, "", "Servotest Ltd" } ,
	{ 0x154F, "", "Stratabeam Technology" } ,
	{ 0x1550, "", "Open Network Co Ltd" } ,
	{ 0x1551, "", "Smart Electronic Development GMBH" } ,
	{ 0x1552, "", "Racal Airtech Ltd" } ,
	{ 0x1553, "", "Chicony Electronics Co Ltd" } ,
	{ 0x1554, "PMC", "Prolink Microsystems Corp." } ,
	{ 0x1555, "Gesytec", "Gesytec GmbH" } ,
	{ 0x1556, "", "PLD Applications" } ,
	{ 0x1557, "", "Mediastar Co. Ltd" } ,
	{ 0x1558, "", "Clevo/Kapok Computer" } ,
	{ 0x1559, "", "SI Logic Ltd" } ,
	{ 0x155A, "", "Innomedia Inc" } ,
	{ 0x155B, "", "Protac International Corp" } ,
	{ 0x155C, "", "Cemax-Icon Inc" } ,
	{ 0x155D, "", "MAC System Co Ltd" } ,
	{ 0x155E, "", "LP Elektronik GMBH" } ,
	{ 0x155F, "", "Perle Systems Limited" } ,
	{ 0x1560, "", "Terayon Communications Systems" } ,
	{ 0x1561, "", "Viewgraphics Inc" } ,
	{ 0x1562, "", "Symbol Technologies" } ,
	{ 0x1563, "", "A-Trend Technology Co Ltd" } ,
	{ 0x1564, "", "Yamakatsu Electronics Industry Co Ltd" } ,
	{ 0x1565, "", "Biostar Microtech Intl Corp" } ,
	{ 0x1566, "", "Ardent Technologies Inc" } ,
	{ 0x1567, "", "Jungsoft" } ,
	{ 0x1568, "", "DDK Electronics Inc" } ,
	{ 0x1569, "", "Palit Microsystems Inc" } ,
	{ 0x156A, "", "Avtec Systems" } ,
	{ 0x156B, "", "2wire Inc" } ,
	{ 0x156C, "", "Vidac Electronics GMBH" } ,
	{ 0x156D, "", "Alpha-Top Corp" } ,
	{ 0x156E, "", "Alfa Inc." } ,
	{ 0x156F, "", "M-Systems Flash Disk Pioneers Ltd" } ,
	{ 0x1570, "", "Lecroy Corporation" } ,
	{ 0x1571, "", "Contemporary Controls" } ,
	{ 0x1572, "", "Otis Elevator Company" } ,
	{ 0x1573, "", "Lattice - Vantis" } ,
	{ 0x1574, "", "Fairchild Semiconductor" } ,
	{ 0x1575, "", "Voltaire Advanced Data Security Ltd" } ,
	{ 0x1576, "", "Viewcast Com" } ,
	{ 0x1578, "", "Hitt" } ,
	{ 0x1579, "", "Dual Technology Corporation" } ,
	{ 0x157A, "", "Japan Elecronics Ind. Inc" } ,
	{ 0x157B, "", "Star Multimedia Corp." } ,
	{ 0x157C, "Eurosoft", "Eurosoft (UK)" } ,
	{ 0x157D, "", "Gemflex Networks" } ,
	{ 0x157E, "", "Transition Networks" } ,
	{ 0x157F, "", "PX Instruments Technology Ltd" } ,
	{ 0x1580, "", "Primex Aerospace Co." } ,
	{ 0x1581, "", "SEH Computertechnik GMBH" } ,
	{ 0x1582, "", "Cytec Corporation" } ,
	{ 0x1583, "", "Inet Technologies Inc" } ,
	{ 0x1584, "", "Uniwill Computer Corp." } ,
	{ 0x1585, "", "Marconi Commerce Systems SRL" } ,
	{ 0x1586, "", "Lancast Inc" } ,
	{ 0x1587, "", "Konica Corporation" } ,
	{ 0x1588, "", "Solidum Systems Corp" } ,
	{ 0x1589, "", "Atlantek Microsystems Pty Ltd" } ,
	{ 0x158A, "", "Digalog Systems Inc" } ,
	{ 0x158B, "", "Allied Data Technologies" } ,
	{ 0x158C, "", "Hitachi Semiconductor & Devices Sales Co" } ,
	{ 0x158D, "", "Point Multimedia Systems" } ,
	{ 0x158E, "", "Lara Technology Inc" } ,
	{ 0x158F, "", "Ditect Coop" } ,
	{ 0x1590, "", "3pardata Inc." } ,
	{ 0x1591, "", "ARN" } ,
	{ 0x1592, "Syba", "Syba Tech Ltd." } ,
	{ 0x1593, "", "Bops Inc" } ,
	{ 0x1594, "", "Netgame Ltd" } ,
	{ 0x1595, "", "Diva Systems Corp." } ,
	{ 0x1596, "", "Folsom Research Inc" } ,
	{ 0x1597, "", "Memec Design Services" } ,
	{ 0x1598, "", "Granite Microsystems" } ,
	{ 0x1599, "", "Delta Electronics Inc" } ,
	{ 0x159A, "", "General Instrument" } ,
	{ 0x159B, "", "Faraday Technology Corp" } ,
	{ 0x159C, "", "Stratus Computer Systems" } ,
	{ 0x159D, "", "Ningbo Harrison Electronics Co Ltd" } ,
	{ 0x159E, "", "A-Max Technology Co Ltd" } ,
	{ 0x159F, "", "Galea Network Security" } ,
	{ 0x15A0, "", "Compumaster SRL" } ,
	{ 0x15A1, "", "Geocast Network Systems Inc" } ,
	{ 0x15A2, "", "Catalyst Enterprises Inc" } ,
	{ 0x15A3, "", "Italtel" } ,
	{ 0x15A4, "", "X-Net OY" } ,
	{ 0x15A5, "", "Toyota MACS Inc" } ,
	{ 0x15A6, "", "Sunlight Ultrasound Technologies Ltd" } ,
	{ 0x15A7, "", "SSE Telecom Inc" } ,
	{ 0x15A8, "", "Shanghai Communications Technologies Cen" } ,
	{ 0x15AA, "", "Moreton Bay" } ,
	{ 0x15AB, "", "Bluesteel Networks Inc" } ,
	{ 0x15AC, "", "North Atlantic Instruments" } ,
	{ 0x15AD, "VMware", "VMware Inc." } ,
	{ 0x15AE, "", "Amersham Pharmacia Biotech" } ,
	{ 0x15B0, "", "Zoltrix International Limited" } ,
	{ 0x15B1, "", "Source Technology Inc" } ,
	{ 0x15B2, "", "Mosaid Technologies Inc." } ,
	{ 0x15B3, "", "Mellanox Technology" } ,
	{ 0x15B4, "", "CCI/Triad" } ,
	{ 0x15B5, "", "Cimetrics Inc" } ,
	{ 0x15B6, "", "Texas Memory Systems Inc" } ,
	{ 0x15B7, "", "Sandisk Corp." } ,
	{ 0x15B8, "", "Addi-Data GMBH" } ,
	{ 0x15B9, "", "Maestro Digital Communications" } ,
	{ 0x15BA, "", "Impacct Technology Corp" } ,
	{ 0x15BB, "", "Portwell Inc" } ,
	{ 0x15BC, "Agilent", "Agilent Technologies" } ,
	{ 0x15BD, "", "DFI Inc." } ,
	{ 0x15BE, "", "Sola Electronics" } ,
	{ 0x15BF, "", "High Tech Computer Corp (HTC)" } ,
	{ 0x15C0, "BVM", "BVM Limited" } ,
	{ 0x15C1, "", "Quantel" } ,
	{ 0x15C2, "", "Newer Technology Inc" } ,
	{ 0x15C3, "", "Taiwan Mycomp Co Ltd" } ,
	{ 0x15C4, "", "EVSX Inc" } ,
	{ 0x15C5, "", "Procomp Informatics Ltd" } ,
	{ 0x15C6, "", "Technical University Of Budapest" } ,
	{ 0x15C7, "", "Tateyama System Laboratory Co Ltd" } ,
	{ 0x15C8, "", "Penta Media Co. Ltd" } ,
	{ 0x15C9, "", "Serome Technology Inc" } ,
	{ 0x15CA, "", "Bitboys OY" } ,
	{ 0x15CB, "", "AG Electronics Ltd" } ,
	{ 0x15CC, "", "Hotrail Inc." } ,
	{ 0x15CD, "", "Dreamtech Co Ltd" } ,
	{ 0x15CE, "", "Genrad Inc." } ,
	{ 0x15CF, "", "Hilscher GMBH" } ,
	{ 0x15D1, "Infineon", "Infineon Technologies AG" } ,
	{ 0x15D2, "", "FIC (First International Computer Inc)" } ,
	{ 0x15D3, "", "NDS Technologies Israel Ltd" } ,
	{ 0x15D4, "", "Iwill Corporation" } ,
	{ 0x15D5, "", "Tatung Co." } ,
	{ 0x15D6, "", "Entridia Corporation" } ,
	{ 0x15D7, "", "Rockwell-Collins Inc" } ,
	{ 0x15D8, "", "Cybernetics Technology Co Ltd" } ,
	{ 0x15D9, "", "Super Micro Computer Inc" } ,
	{ 0x15DA, "", "Cyberfirm Inc." } ,
	{ 0x15DB, "", "Applied Computing Systems Inc." } ,
	{ 0x15DC, "Litronic", "Litronic Inc." } ,
	{ 0x15DD, "", "Sigmatel Inc." } ,
	{ 0x15DE, "", "Malleable Technologies Inc" } ,
	{ 0x15DF, "", "Infinilink Corp." } ,
	{ 0x15E0, "", "Cacheflow Inc" } ,
	{ 0x15E1, "VTG", "Voice Technologies Group" } ,
	{ 0x15E2, "", "Quicknet Technologies Inc" } ,
	{ 0x15E3, "", "Networth Technologies Inc" } ,
	{ 0x15E4, "", "VSN Systemen BV" } ,
	{ 0x15E5, "", "Valley Technologies Inc" } ,
	{ 0x15E6, "", "Agere Inc." } ,
	{ 0x15E7, "", "GET Engineering Corp." } ,
	{ 0x15E8, "", "National Datacomm Corp." } ,
	{ 0x15E9, "", "Pacific Digital Corp." } ,
	{ 0x15EA, "", "Tokyo Denshi Sekei K.K." } ,
	{ 0x15EB, "", "Drsearch GMBH" } ,
	{ 0x15EC, "", "Beckhoff GMBH" } ,
	{ 0x15ED, "", "Macrolink Inc" } ,
	{ 0x15EE, "", "IN Win Development Inc." } ,
	{ 0x15EF, "", "Intelligent Paradigm Inc" } ,
	{ 0x15F0, "", "B-Tree Systems Inc" } ,
	{ 0x15F1, "", "Times N Systems Inc" } ,
	{ 0x15F2, "", "Diagnostic Instruments Inc" } ,
	{ 0x15F3, "", "Digitmedia Corp." } ,
	{ 0x15F4, "", "Valuesoft" } ,
	{ 0x15F5, "", "Power Micro Research" } ,
	{ 0x15F6, "", "Extreme Packet Device Inc" } ,
	{ 0x15F7, "", "Banctec" } ,
	{ 0x15F8, "", "Koga Electronics Co" } ,
	{ 0x15F9, "", "Zenith Electronics Corporation" } ,
	{ 0x15FA, "", "J.P. Axzam Corporation" } ,
	{ 0x15FB, "", "Zilog Inc." } ,
	{ 0x15FC, "", "Techsan Electronics Co Ltd" } ,
	{ 0x15FD, "", "N-Cubed.Net" } ,
	{ 0x15FE, "", "Kinpo Electronics Inc" } ,
	{ 0x15FF, "", "Fastpoint Technologies Inc." } ,
	{ 0x1600, "", "Northrop Grumman - Canada Ltd" } ,
	{ 0x1601, "", "Tenta Technology" } ,
	{ 0x1602, "", "Prosys-TEC Inc." } ,
	{ 0x1603, "", "Nokia Wireless Business Communications" } ,
	{ 0x1604, "", "Central System Research Co Ltd" } ,
	{ 0x1605, "", "Pairgain Technologies" } ,
	{ 0x1606, "", "Europop AG" } ,
	{ 0x1607, "", "Lava Semiconductor Manufacturing Inc." } ,
	{ 0x1608, "", "Automated Wagering International" } ,
	{ 0x1609, "", "Sciemetric Instruments Inc" } ,
	{ 0x160A, "", "Kollmorgen Servotronix" } ,
	{ 0x160B, "", "Onkyo Corp." } ,
	{ 0x160C, "", "Oregon Micro Systems Inc." } ,
	{ 0x160D, "", "Aaeon Electronics Inc" } ,
	{ 0x160E, "", "CML Emergency Services" } ,
	{ 0x160F, "", "ITEC Co Ltd" } ,
	{ 0x1610, "", "Tottori Sanyo Electric Co Ltd" } ,
	{ 0x1611, "", "Bel Fuse Inc." } ,
	{ 0x1612, "", "Telesynergy Research Inc." } ,
	{ 0x1613, "", "System Craft Inc." } ,
	{ 0x1614, "", "Jace Tech Inc." } ,
	{ 0x1615, "", "Equus Computer Systems Inc" } ,
	{ 0x1616, "", "Iotech Inc." } ,
	{ 0x1617, "", "Rapidstream Inc" } ,
	{ 0x1618, "", "Esec SA" } ,
	{ 0x1619, "FarSite", "FarSite Communications Limited" } ,
	{ 0x161A, "", "Wvinten Ltd" } ,
	{ 0x161B, "", "Mobilian Israel Ltd" } ,
	{ 0x161C, "", "Berkshire Products" } ,
	{ 0x161D, "", "Gatec" } ,
	{ 0x161E, "", "Kyoei Sangyo Co Ltd" } ,
	{ 0x161F, "", "Arima Computer Co" } ,
	{ 0x1620, "", "Sigmacom Co Ltd" } ,
	{ 0x1621, "", "Lynx Studio Technology Inc" } ,
	{ 0x1622, "NHC", "Nokia Home Communications" } ,
	{ 0x1623, "", "KRF Tech Ltd" } ,
	{ 0x1624, "", "CE Infosys GMBH" } ,
	{ 0x1625, "", "Warp Nine Engineering" } ,
	{ 0x1626, "", "TDK Semiconductor Corp." } ,
	{ 0x1627, "", "BCom Electronics Inc" } ,
	{ 0x1629, "", "Kongsberg Spacetec a.s." } ,
	{ 0x162A, "", "Sejin Computerland Co Ltd" } ,
	{ 0x162B, "", "Shanghai Bell Company Limited" } ,
	{ 0x162C, "", "C&H Technologies Inc" } ,
	{ 0x162D, "", "Reprosoft Co Ltd" } ,
	{ 0x162E, "", "Margi Systems Inc" } ,
	{ 0x162F, "", "Rohde & Schwarz GMBH & Co KG" } ,
	{ 0x1630, "", "Sky Computers Inc" } ,
	{ 0x1631, "", "NEC Computer International" } ,
	{ 0x1632, "", "Verisys Inc" } ,
	{ 0x1633, "", "Adac Corporation" } ,
	{ 0x1634, "", "Visionglobal Network Corp." } ,
	{ 0x1635, "", "Decros" } ,
	{ 0x1636, "", "Jean Company Ltd" } ,
	{ 0x1637, "", "NSI" } ,
	{ 0x1638, "", "Eumitcom Technology Inc" } ,
	{ 0x163A, "", "Air Prime Inc" } ,
	{ 0x163B, "", "Glotrex Co Ltd" } ,
	{ 0x163C, "", "Smart Link" } ,
	{ 0x163D, "", "Heidelberg Digital LLC" } ,
	{ 0x163E, "", "3dpower" } ,
	{ 0x163F, "", "Renishaw PLC" } ,
	{ 0x1640, "", "Intelliworxx Inc" } ,
	{ 0x1641, "", "MKNet Corporation" } ,
	{ 0x1642, "", "Bitland" } ,
	{ 0x1643, "", "Hajime Industries Ltd" } ,
	{ 0x1644, "", "Western Avionics Ltd" } ,
	{ 0x1645, "", "Quick-Serv. Computer Co. Ltd" } ,
	{ 0x1646, "", "Nippon Systemware Co Ltd" } ,
	{ 0x1647, "", "Hertz Systemtechnik GMBH" } ,
	{ 0x1648, "", "MeltDown Systems LLC" } ,
	{ 0x1649, "", "Jupiter Systems" } ,
	{ 0x164A, "", "Aiwa Co. Ltd" } ,
	{ 0x164C, "", "Department Of Defense" } ,
	{ 0x164D, "", "Ishoni Networks" } ,
	{ 0x164E, "", "Micrel Inc." } ,
	{ 0x164F, "", "Datavoice (Pty) Ltd." } ,
	{ 0x1650, "", "Admore Technology Inc." } ,
	{ 0x1651, "", "Chaparral Network Storage" } ,
	{ 0x1652, "", "Spectrum Digital Inc." } ,
	{ 0x1653, "", "Nature Worldwide Technology Corp" } ,
	{ 0x1654, "", "Sonicwall Inc" } ,
	{ 0x1655, "", "Dazzle Multimedia Inc." } ,
	{ 0x1656, "", "Insyde Software Corp" } ,
	{ 0x1657, "", "Brocade Communications Systems" } ,
	{ 0x1658, "", "Med Associates Inc." } ,
	{ 0x1659, "", "Shiba Denshi Systems Inc." } ,
	{ 0x165A, "", "Epix Inc." } ,
	{ 0x165B, "", "Real-Time Digital Inc." } ,
	{ 0x165C, "", "Gidel Ltd." } ,
	{ 0x165D, "", "Hsing Tech. Enterprise Co. Ltd." } ,
	{ 0x165E, "", "Hyunju Computer Co. Ltd." } ,
	{ 0x165F, "", "Add One Company" } ,
	{ 0x1660, "", "Network Security Technologies Inc. (Net " } ,
	{ 0x1661, "", "Worldspace Corp." } ,
	{ 0x1662, "", "Int Labs" } ,
	{ 0x1663, "", "Elmec Inc. Ltd." } ,
	{ 0x1664, "", "Fastfame Technology Co. Ltd." } ,
	{ 0x1665, "", "Edax Inc." } ,
	{ 0x1666, "", "Norpak Corporation" } ,
	{ 0x1667, "", "CoSystems Inc." } ,
	{ 0x1668, "Actiontec", "Actiontec Electronics Inc." } ,
	{ 0x166A, "", "Komatsu Ltd." } ,
	{ 0x166B, "", "Supernet Inc." } ,
	{ 0x166C, "", "Shade Ltd." } ,
	{ 0x166D, "", "Sibyte Inc." } ,
	{ 0x166E, "", "Schneider Automation Inc." } ,
	{ 0x166F, "", "Televox Software Inc." } ,
	{ 0x1670, "", "Rearden Steel" } ,
	{ 0x1671, "", "Atan Technology Inc." } ,
	{ 0x1672, "", "Unitec Co. Ltd." } ,
	{ 0x1673, "", "Connex" } ,
	{ 0x1675, "", "Square Wave Technology" } ,
	{ 0x1676, "", "Emachines Inc." } ,
	{ 0x1677, "", "Bernecker + Rainer" } ,
	{ 0x1678, "", "INH Semiconductor" } ,
	{ 0x1679, "", "Tokyo Electron Device Ltd." } ,
	{ 0x167F, "iba", "Ingenieurbuero Anhaus GmbH" } ,
	{ 0x1680, "Dunti", "Dunti Corp." } ,
	{ 0x1681, "Hercules", "Hercules" } ,
	{ 0x1682, "PINE", "PINE Technology, Ltd." } ,
	{ 0x1688, "CastleNet", "CastleNet Technology Inc." } ,
	{ 0x168A, "USA", "Utimaco Safeware AG" } ,
	{ 0x168B, "", "Circut Assembly Corp." } ,
	{ 0x168C, "Atheros", "Atheros Communications Inc." } ,
	{ 0x168D, "NMI", "NMI Electronics Ltd." } ,
	{ 0x168E, "Hyundai MultiCAV", "Hyundai MultiCAV Computer Co. Ltd." } ,
	{ 0x168F, "KDSI", "KDS Innotech Corp." } ,
	{ 0x1690, "NetContinuum", "NetContinuum, Inc." } ,
	{ 0x1693, "FERMA", "FERMA" } ,
	{ 0x1695, "EPoX", "EPoX Computer Co., Ltd." } ,
	{ 0x16AE, "SFNT", "SafeNet Inc." } ,
	{ 0x16B3, "", "CNF Mobile Solutions" } ,
	{ 0x16CA, "Cenatek", "Cenatek Inc." } ,
	{ 0x16CB, "Minolta", "Minolta Co. Ltd." } ,
	{ 0x16CC, "Inari", "Inari Inc." } ,
	{ 0x16D0, "", "Systemax" } ,
	{ 0x16E0, "3MTS", "Third Millenium Test Solutions, Inc." } ,
	{ 0x16F0, "", "TLA Inc." } ,
	{ 0x16F1, "Adicti", "Adicti Corp." } ,
	{ 0x16F3, "Jetway", "Jetway Information Co., Ltd" } ,
	{ 0x16F6, "VideoTele.com", "VideoTele.com Inc." } ,
	{ 0x1700, "Antara", "Antara LLC" } ,
	{ 0x1701, "", "Interactive Computer Products Inc." } ,
	{ 0x1702, "", "Internet Machines Corp." } ,
	{ 0x1703, "Desana", "Desana Systems" } ,
	{ 0x1704, "Clearwater", "Clearwater Networks" } ,
	{ 0x1705, "Digital First", "Digital First" } ,
	{ 0x1706, "PBC", "Pacific Broadband Communications" } ,
	{ 0x1707, "Cogency", "Cogency Semiconductor Inc." } ,
	{ 0x1708, "Harris", "Harris Corp." } ,
	{ 0x1709, "Zarlink", "Zarlink Semiconductor" } ,
	{ 0x170A, "Alpine", "Alpine Electronics Inc." } ,
	{ 0x170B, "NetOctave", "NetOctave Inc." } ,
	{ 0x170C, "YottaYotta", "YottaYotta Inc." } ,
	{ 0x170D, "SMI", "SensoMotoric Instruments GmbH" } ,
	{ 0x170E, "San Valley", "San Valley Systems, Inc." } ,
	{ 0x170F, "Cyberdyne", "Cyberdyne Inc." } ,
	{ 0x1710, "Pelago", "Pelago Networks" } ,
	{ 0x1711, "NetScreen", "NetScreen Technologies, Inc." } ,
	{ 0x1712, "NICE", "NICE Systems Inc." } ,
	{ 0x1713, "TOPCON", "TOPCON Corp." } ,
	{ 0x1734, "Fujitsu-Siemens", "Fujitsu-Siemens Computers GmbH" } ,
	{ 0x173B, "Altima", "Altima" } ,
	{ 0x1743, "Peppercon", "Peppercon AG" } ,
	{ 0x1752, "GBM", "Global Brands Manufacture Ltd." } ,
	{ 0x1755, "Alchemy", "Alchemy Semiconductor Inc." } ,
	{ 0x176A, "GDC", "General Dynamics Canada" } ,
	{ 0x1789, "Ennyah", "Ennyah Technologies Corp" } ,
	{ 0x1793, "Unitech", "Unitech Electronics Co., Ltd" } ,
	{ 0x17A7, "Start Network", "Start Network Technology Co., Ltd." } ,
	{ 0x17AA, "Legend", "Legend Ltd. (Beijing)" } ,
	{ 0x17AB, "", "Phillips Components" } ,
	{ 0x17AF, "Hightech", "Hightech Information Systems, Ltd." } ,
	{ 0x17C0, "Wistron", "Wistron Corp." } ,
	{ 0x17C4, "", "Movita Technologies, Inc" } ,
	{ 0x17E9, "", "DH electronics GmbH" } ,
	{ 0x1813, "", "Modem Silicon Operation" } ,
	{ 0x1815, "devolo", "devolo AG" } ,
	{ 0x1820, "InfiniCon", "InfiniCon Systems, Inc." } ,
	{ 0x1860, "Primagraphics", "Primagraphics Ltd." } ,
	{ 0x1888, "", "Varisys Limited" } ,
	{ 0X1898, "DICIT", "DIC INFORMATION TECHNOLOGY ,LTD" } ,
	{ 0x1978, "", "HARPO sp. z o.o." } ,
	{ 0x1A08, "Sierra", "Sierra Semiconductor" } ,
	{ 0x1B13, "", "Jaton Corporation" } ,
	{ 0x1C1C, "Symphony", "Symphony" } ,
	{ 0x1D44, "DPT", "Distributed Processing Technology" } ,
	{ 0x1DE1, "Tekram", "Tekram" } ,
	{ 0x1DE2, "A/DHOC", "A/DHOC Syst�mes" } ,
	{ 0x1DEA, "LST", "Lasentec" } ,
	{ 0x2001, "", "Temporal Research Ltd" } ,
	{ 0x2002, "AT", "Automation Technology GmbH" } ,
	{ 0x2014, "NONTECH", "NONTECH Nonnenmacher GmbH" } ,
	{ 0x2348, "Racore", "Racore" } ,
	{ 0x2646, "", "Kingston Technology Co." } ,
	{ 0x270F, "ChainTech", "ChainTech Computer Co. Ltd." } ,
	{ 0x2EC1, "", "Zenic Inc" } ,
	{ 0x3000, "Hansol", "Hansol Electronics Inc." } ,
	{ 0x3142, "PostImpressions", "Post Impressions Systems" } ,
	{ 0x3333, "Bertin", "Bertin Technologies" } ,
	{ 0x3388, "Hint", "Hint Corp." } ,
	{ 0x3411, "", "Quantum Designs (H.K.) Inc." } ,
	{ 0x3513, "ARCOM", "ARCOM Control Systems Ltd." } ,
	{ 0x38D0, "MFP", "MFP GmbH" } ,
	{ 0x38EF, "", "4links" } ,
	{ 0x3D3D, "3DLabs", "3Dlabs, Inc. Ltd" } ,
	{ 0x4005, "Avance", "Avance Logic Inc." } ,
	{ 0x4033, "Addtron", "Addtron Technology Co., Inc." } ,
	{ 0x4130, "AT", "Allied Telesyn" } ,
	{ 0x4143, "DEC", "Digital Equipment Corp." } ,
	{ 0x4144, "Alpha Data", "Alpha Data" } ,
	{ 0x416C, "", "Aladdin Knowledge Systems" } ,
	{ 0x4444, "Conexant", "Conexant Inc." } ,
	{ 0x4468, "Bridgeport", "Bridgeport Machines" } ,
	{ 0x4550, "TDT", "Tucker-Davis Technologies" } ,
	{ 0x4594, "", "Cogetec Informatique Inc." } ,
	{ 0x45FB, "Baldor", "Baldor Electric Company" } ,
	{ 0x4680, "UMAX Comp", "UMAX Computer Corp." } ,
	{ 0x4843, "Hercules", "Hercules Computer Technology" } ,
	{ 0x4943, "", "Growth Networks" } ,
	{ 0x494F, "ICS", "ICS Advent" } ,
	{ 0x4954, "Integral", "Integral Technologies" } ,
	{ 0x4978, "Axil", "Axil Computer Inc." } ,
	{ 0x4A14, "NetVin", "NetVin" } ,
	{ 0x4ABD, "ABI", "Applied Biosystems Division" } ,
	{ 0x4B10, "Buslogic", "Buslogic Inc" } ,
	{ 0x4C48, "Lung Hwa", "Lung Hwa Electronics" } ,
	{ 0x4C53, "", "SBS-OR Industrial Computers" } ,
	{ 0x4CA1, "", "Seanix Technology Inc" } ,
	{ 0x4D51, "Mediaq", "Mediaq Inc." } ,
	{ 0x4D54, "", "Microtechnica Co Ltd" } ,
	{ 0x4DDC, "ILC", "ILC Data Device Corp." } ,
	{ 0x5046, "Guillemot", "Guillemot" } ,
	{ 0x5053, "TBS/Voyetra", "TBS/Voyetra Technologies" } ,
	{ 0x5112, "", "Sigmatek GmbH & CoKG" } ,
	{ 0x5136, "", "S S Technologies" } ,
	{ 0x5143, "Qualcomm", "Qualcomm Inc." } ,
	{ 0x5145, "ENSONIQ", "ENSONIQ" } ,
	{ 0x5301, "Alliance", "Alliance Semicondutor Corp." } ,
	{ 0x5333, "S3", "S3 Incorporated" } ,
	{ 0x5401, "Ericsson", "Ericsson" } ,
	{ 0x5430, "Evergreen", "Evergreen Technologies Inc." } ,
	{ 0x544C, "", "Teralogic Inc" } ,
	{ 0x5455, "TU-Berlin", "Technische Universtiaet Berlin" } ,
	{ 0x5519, "Cnet", "Cnet Technoliges, Inc." } ,
	{ 0x5544, "Dunord", "Dunord Technologies" } ,
	{ 0x5555, "Genroco", "Genroco Inc." } ,
	{ 0x55CF, "", "Unisys Corporation" } ,
	{ 0x5700, "Netpower", "Netpower" } ,
	{ 0x5AB7, "Sabtech", "Sabtech Industries" } ,
	{ 0x6356, "UltraStor", "UltraStor" } ,
	{ 0x6374, "C4T", "c't Magazin f_r Computertechnik" } ,
	{ 0x6409, "", "Logitec Corp." } ,
	{ 0x6666, "Decision", "Decision Computer International Co." } ,
	{ 0x7604, "O.N.", "O.N. Electric Co. Ltd." } ,
	{ 0x7747, "DaoGuo", "DaoGuo Technology Co.,Ltd" } ,
	{ 0x7BDE, "MIDAC", "MIDAC Corporation" } ,
	{ 0x7FED, "PowerTV", "PowerTV" } ,
	{ 0x8001, "BEYERTONE", "Beyertone AG - Germany" } ,
	{ 0x8008, "QUANCOM", "QUANCOM Informationssysteme GmbH" } ,
	{ 0x801F, "", "ARS Technologies" } ,
	{ 0x8086, "Intel", "Intel Corporation" } ,
	{ 0x8800, "Trigem", "Trigem Computer" } ,
	{ 0x8866, "T-Square", "T-Square Design Inc." } ,
	{ 0x8888, "Sil Magic", "Silicon Magic" } ,
	{ 0x8912, "TRX", "TRX" } ,
	{ 0x8967, "", "Woopy systems" } ,
	{ 0x8E0E, "Computone", "Computone Corporation" } ,
	{ 0x8E2E, "KTI", "KTI" } ,
	{ 0x9004, "Adaptec", "Adaptec" } ,
	{ 0x9005, "Adaptec", "Adaptec" } ,
	{ 0x907F, "Atronics", "Atronics" } ,
	{ 0x919A, "", "Gigapixel Corp" } ,
	{ 0x9412, "Holtek", "Holtek" } ,
	{ 0x9699, "", "Omni Media Technology Inc." } ,
	{ 0x9710, "NetMos", "NetMos Techology" } ,
	{ 0x9875, "", "wouteronline" } ,
	{ 0x9877, "surveyors", "pan asia superitendence corp" } ,
	{ 0x9902, "StarGen", "StarGen, Inc." } ,
	{ 0xA0A0, "Aopen", "Aopen Inc." } ,
	{ 0xA0F1, "", "Unisys Corporation" } ,
	{ 0xA200, "NEC", "NEC Corp." } ,
	{ 0xA259, "", "Hewlett Packard" } ,
	{ 0xA25B, "Hewlett Packard", "Hewlett Packard GmbH PL24-MKT" } ,
	{ 0xA304, "Sony", "Sony" } ,
	{ 0xA727, "", "3com Corporation" } ,
	{ 0xAA42, "Scitex", "Scitex Digital Video" } ,
	{ 0xABCD, "", "PROGSTAR" } ,
	{ 0xAC1E, "", "Digital Receiver Technology Inc" } ,
	{ 0xAECB, "", "Adrienne Electronics Corporation" } ,
	{ 0xB00C, "IC Book", "IC Book Labs" } ,
	{ 0xB1B3, "Shiva", "Shiva Europe Ltd." } ,
	{ 0xB894, "", "Brown & Sharpe Mfg. Co." } ,
	{ 0xBEEF, "Mindstream Computing", "Mindstream Computing" } ,
	{ 0xC001, "TSI", "TSI Telsys" } ,
	{ 0xC0A9, "Micron/Crucial", "Micron/Crucial Technology" } ,
	{ 0xC0DE, "", "Motorola" } ,
	{ 0xC0FE, "Mot Engrg", "Motion Engineering Inc." } ,
	{ 0xC622, "", "Hudson Soft Co Ltd" } ,
	{ 0xCA50, "Varian", "Varian Australia Pty. Ltd." } ,
	{ 0xCAFE, "", "Chrysalis-ITS" } ,
	{ 0xCCCC, "", "Catapult Communications" } ,
	{ 0xCDDD, "Tyzx Inc.", "Tyzx Inc." } ,
	{ 0xD4D4, "DY4", "DY4 Systems Inc." } ,
	{ 0xD531, "I+ME ACTIA", "I+ME ACTIA GmBH" } ,
	{ 0xD84D, "Exsys", "Exsys" } ,
	{ 0xDC93, "", "Dawicontrol" } ,
	{ 0xDEAD, "Indigita", "Indigita Corporation" } ,
	{ 0xDEAF, "", "Middle Digital, Inc" } ,
	{ 0xE000, "Winbond", "Winbond" } ,
	{ 0xE159, "Tiger Jet", "Tiger Jet Network Inc" } ,
	{ 0xE4BF, "", "EKF Elektronik GMBH" } ,
	{ 0xEA01, "", "Eagle Technology" } ,
	{ 0xEABB, "Aashima", "Aashima Technology B.V." } ,
	{ 0xEACE, "Endace", "Endace Measurement Systems Ltd." } ,
	{ 0xECC0, "Echo", "Echo Corporation" } ,
	{ 0xEDD8, "ARK Logic", "ARK Logic, Inc" } ,
	{ 0xF1D0, "", "AJA Video" } ,
	{ 0xF5F5, "", "F5 Networks Inc." } ,
	{ 0xFA57, "Interagon", "Interagon AS" } ,
	{ 0xFEBD, "Ultraview", "Ultraview Corp." } ,
	{ 0xFEDA, "Broadcom", "Broadcom" } ,
	{ 0xFFFE, "VMware", "VMware Inc." } ,
	{ 0xFFFF, "BAD!", "ILLEGITIMATE VENDOR ID" }
} ;

// Use this value for loop control during searching:
#define	PCI_VENTABLE_LEN	(sizeof(PciVenTable)/sizeof(PCI_VENTABLE))

typedef struct _PCI_DEVTABLE
{
	unsigned short	VenId ;
	unsigned short	DevId ;
	char *	Chip ;
	char *	ChipDesc ;
}  PCI_DEVTABLE, *PPCI_DEVTABLE ;

PCI_DEVTABLE	PciDevTable [] =
{
	{ 0x0000, 0x07, "", "" } ,
	{ 0x0000, 0x1073, "", "" } ,
	{ 0x003D, 0x00D1, "", "i740 PCI" } ,
	{ 0x0675, 0x1700, "IS64PH", "ISDN Adapter" } ,
	{ 0x0675, 0x1702, "IS64PH", "ISDN Adapter" } ,
	{ 0x0700, 0x0100, "", "LavaPort PCI" } ,
	{ 0x0700, 0x0200, "", "LavaPort PCI" } ,
	{ 0x0815, 0x0002, "ELKA SO-PCI", "" } ,
	{ 0x0871, 0xFFA1, "A1T", "HCF-PCI card" } ,
	{ 0x0871, 0xFFA2, "T-Concept", "HCF-PCI card" } ,
	{ 0x0914, 0x0201, "83820", "32bit pci mac" } ,
	{ 0x0914, 0x0202, "83821", "64bit pci gnic" } ,
	{ 0x09C1, 0x0704, "CM 200E", "Cable Modem" } ,
	{ 0x0E11, 0x0001, "", "PCI to EISA Bridge" } ,
	{ 0x0E11, 0x0002, "ISA Bridge", "" } ,
	{ 0x0E11, 0x000F, "CPQB1A9", "StorageWorks Library Adapter (HVD)" } ,
	{ 0x0E11, 0x0049, "NC7132", "Gigabit Upgrade Module" } ,
	{ 0x0E11, 0x004A, "NC6136", "Gigabit Server Adapter" } ,
	{ 0x0E11, 0x00C0, "Adaptec AIC-7899G", "64Bit,66MHz,Dual Channel WideUltra3 SCSI" } ,
	{ 0x0E11, 0x0508, "Neteligent 4/16 TR", "PCI UTP/STP Controller" } ,
	{ 0x0E11, 0x1000, "Triflex Model 1000", "Pentium Bridge" } ,
	{ 0x0E11, 0x2000, "Triflex Model 2000", "Pentium Bridge" } ,
	{ 0x0E11, 0x3032, "QVision 1280/p v0", "GUI Accelerator" } ,
	{ 0x0E11, 0x3033, "QVision 1280/p v1", "GUI Accelerator" } ,
	{ 0x0E11, 0x3034, "QVision 1280/p v2", "GUI Accelerator" } ,
	{ 0x0E11, 0x4000, "Triflex Model 4000", "Pentium Bridge" } ,
	{ 0x0E11, 0x6010, "Model 6010", "HotPlug PCI Bridge" } ,
	{ 0x0E11, 0x7020, "", "USB Controller" } ,
	{ 0x0E11, 0xA0EC, "", "Original Compaq fibre Channel HBA" } ,
	{ 0x0E11, 0xA0F0, "", "Advanced System Management Controller" } ,
	{ 0x0E11, 0xA0F3, "", "Triflex PCI to ISA PnP Bridge" } ,
	{ 0x0E11, 0xA0F7, "", "PCI Hotplug Controller" } ,
	{ 0x0E11, 0xA0F8, "ZFMicro", "USB Open Host Controller" } ,
	{ 0x0E11, 0xA0FC, "HPFC-5166A", "Tachyon TL 64-bit/66-Mhz FC HBA" } ,
	{ 0x0E11, 0xAe10, "", "Smart-2 Array Controller" } ,
	{ 0x0E11, 0xAE29, "MIS-L", "PCI to ISA Bridge" } ,
	{ 0x0E11, 0xAE2A, "MPC", "CPU to PCI Bridge" } ,
	{ 0x0E11, 0xAE2B, "MIS-E", "PCI to ISA PnP Bridge" } ,
	{ 0x0E11, 0xAE31, "", "System Management Controller" } ,
	{ 0x0E11, 0xAE32, "", "Netelligent 10/100 TX PCI UTP TLAN 2.3" } ,
	{ 0x0E11, 0xAE33, "Triflex", "Dual EIDE Controller" } ,
	{ 0x0E11, 0xAE34, "", "Netelligent 10 T PCI UTP TLAN 2.3" } ,
	{ 0x0E11, 0xAE35, "", "Integrated NetFlex 3/P TLAN 2.3" } ,
	{ 0x0E11, 0xAE40, "", "Dual Port Netelligent 10/100 TX PCI TLAN" } ,
	{ 0x0E11, 0xAE43, "", "Integrated Netelligent 10/100 TX PCI" } ,
	{ 0x0E11, 0xAE69, "CETUS-L", "PCI to ISA Bridge" } ,
	{ 0x0E11, 0xAE6C, "DRACO", "PCI Bridge" } ,
	{ 0x0E11, 0xAE6D, "NorthStar", "CPU to PCI Bridge" } ,
	{ 0x0E11, 0xB011, "", "Dual Port Netelligent 10/100 TX" } ,
	{ 0x0E11, 0xB012, "Netelligent 10 T/2", "UTP/Coax PCI" } ,
	{ 0x0E11, 0xB01E, "NC3120", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB01F, "NC3122", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB02F, "NC1120", "Ethernet NIC" } ,
	{ 0x0E11, 0xB030, "Netelligent WS 1500", "10/100TX Embedded UTP/Coax Controller" } ,
	{ 0x0E11, 0xB04A, "", "10/100TX WOL UTP Controller" } ,
	{ 0x0E11, 0XB060, "CISS", "SMART2 Array Controller" } ,
	{ 0x0E11, 0xB0C6, "NC3161", "Fast Ethernet Embedded Controller w/ WOL" } ,
	{ 0x0E11, 0xB0C7, "NC3160", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB0D7, "NC3121 rev. A & B", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB0DD, "NC3131", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB0DE, "NC3132", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB0DF, "NC6132", "Gigabit Module" } ,
	{ 0x0E11, 0xB0E0, "NC6133", "Gigabit Module" } ,
	{ 0x0E11, 0xB0E1, "NC3133", "Fast Ethernet Module" } ,
	{ 0x0E11, 0xB123, "NC6134", "Gigabit NIC" } ,
	{ 0x0E11, 0xB134, "NC3163", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB13C, "NC3162", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB144, "NC3123", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB163, "NC3134", "Fast Ethernet NIC" } ,
	{ 0x0E11, 0xB164, "NC3135", "Fast Ethernet Upgrade Module" } ,
	{ 0x0E11, 0xB178, "CISSB", "SMART2 Array Controller" } ,
	{ 0x0E11, 0xB196, "", "Conexant SoftK56 Modem" } ,
	{ 0x0E11, 0xB1A4, "NC7131", "Gigabit Server Adapter" } ,
	{ 0x0E11, 0xB203, "iLo", "Integrated Lights Out Processor" } ,
	{ 0x0E11, 0xB204, "iLo", "Integrated Lights Out Processor" } ,
	{ 0x0E11, 0xF130, "", "ThunderLAN 1.0 NetFlex-3/P" } ,
	{ 0x0E11, 0xF150, "", "ThunderLAN 2.3 NetFlex-3/P with BNC" } ,
	{ 0x0E11, 0xF700, "", "LP7000 Compaq/Emulex Fibre Channel HBA" } ,
	{ 0x0E11, 0xF800, "", "LP8000 Compaq/Emulex Fibre Channel HBA" } ,
	{ 0x1000, 0x0001, "LSI53C810/810A/810AE", "PCI-SCSI I/O Processor" } ,
	{ 0x1000, 0x0002, "53C820", "Fast-wide SCSI" } ,
	{ 0x1000, 0x0003, "LSI53C825/825A/825AE", "PCI to SCSI I/O Processor" } ,
	{ 0x1000, 0x0004, "53C815", "Fast SCSI" } ,
	{ 0x1000, 0x0005, "53C810AP", "Fast SCSI" } ,
	{ 0x1000, 0x0006, "LSI53C860/860E", "PCI to Ultra SCSI I/O Processor" } ,
	{ 0x1000, 0x000A, "LSI53C1510", "PCI Dual Channel Wide Ultra2 SCSI Ctrlr" } ,
	{ 0x1000, 0x000B, "LSI53C896/897", "PCI Dual Channel Wide Ultra2 SCSI Ctrlr" } ,
	{ 0x1000, 0x000C, "LSI53C895", "PCI to Ultra2 SCSI I/O Processor" } ,
	{ 0x1000, 0x000D, "53C885", "Ultra Wide SCSI, Ethernet" } ,
	{ 0x1000, 0x000F, "53C875/875E/876/876E", "PCI to Ultra SCSI I/O Processor" } ,
	{ 0x1000, 0x0010, "LSI53C1510", "I2O-Ready PCI RAID Ultra2 SCSI Ctrlr" } ,
	{ 0x1000, 0x0012, "LSI53C895A", "PCI to Ultra2 SCSI Controller" } ,
	{ 0x1000, 0x0013, "LSI53C875A", "PCI to Ultra SCSI Controller" } ,
	{ 0x1000, 0x0020, "LSI53C1010-33", "PCI to Dual Channel Ultra3 SCSI Ctrlr" } ,
	{ 0x1000, 0x0021, "LSI53C1000/1010-66", "PCI to Ultra160 SCSI Controller" } ,
	{ 0x1000, 0x0030, "LSI53C1020/1030", "PCI-X to Ultra320 SCSI Controller" } ,
	{ 0x1000, 0x0031, "LSI53C1030ZC", "PCI-X SCSI Controller" } ,
	{ 0x1000, 0x0035, "LSI53C1035", "PCI-X SCSI Controller" } ,
	{ 0x1000, 0x0040, "LSI53C1035", "PCI-X SCSI Controller" } ,
	{ 0x1000, 0x008F, "53C810", "LSI 53C8xx SCSI host adapter chip" } ,
	{ 0x1000, 0x0621, "LSIFC909", "Fibre Channel I/O Processor" } ,
	{ 0x1000, 0x0622, "LSIFC929", "Dial Channel Fibre Channel I/O Processor" } ,
	{ 0x1000, 0x0623, "LSIFC929", "Dual Channel Fibre Channel I/O Processor" } ,
	{ 0x1000, 0x0624, "LSIFC919", "Fibre Channel I/O Processor" } ,
	{ 0x1000, 0x0625, "LSIFC919", "Fibre Channel I/O Processor" } ,
	{ 0x1000, 0x0630, "LSIFC920", "Fibre Channel I/O Processor" } ,
	{ 0x1000, 0x0701, "53C885", "10/100 MBit Ethernet" } ,
	{ 0x1000, 0x0702, "Yellowfin G-NIC", "Gigabit Ethernet Controller" } ,
	{ 0x1000, 0x0901, "61C102", "USB Controller" } ,
	{ 0x1000, 0x1000, "63C815", "Fast SCSI Controller" } ,
	{ 0x1000, 0x1001, "53C895", "Symbios Ultra2 SCSI controller" } ,
	{ 0x1001, 0x0010, "ispLSI1032E", "PCI 1616, 16 TTL-IN, 16 TTL-OUT" } ,
	{ 0x1001, 0x0011, "ispLSI1032E", "OPTO-PCI, 16 IN / 16 OUT 24 VDC" } ,
	{ 0x1001, 0x0012, "ispLSI1032E", "PCI-AD, PCI-ADDA analog I/O-card" } ,
	{ 0x1001, 0x0013, "ispLSI1032E", "PCI-OptoRel, PCI-Relais 16 Relais & Opto" } ,
	{ 0x1001, 0x0014, "ispLSI1032E", "Timer, Pulse & Counter-card 16..32 bit" } ,
	{ 0x1001, 0x0015, "ispLSI1032E", "PCI-DAC416, 4 channel D/A16bit precision" } ,
	{ 0x1001, 0x0016, "ispLSI1032E", "PCI-MFB high-speed analog I/O" } ,
	{ 0x1001, 0x0017, "ispLSI1032E", "PROTO-3 PCI, digital I/O with chipselect" } ,
	{ 0x1001, 0x0020, "ispLSI1032E", "Universal digital I/O PCI-Interface" } ,
	{ 0x1002, 0x4136, "", "Radeon Mobility A3" } ,
	{ 0x1002, 0x4144, "R300", "Radeon 9700/9500 Series" } ,
	{ 0x1002, 0x4147, "", "ATI Fire GL Z1 4P Video Accelerator" } ,
	{ 0x1002, 0x4158, "68800AX", "Mach 32" } ,
	{ 0x1002, 0x4164, "", "Fire GL Z1/Z1 Pro Video Accelerator" } ,
	{ 0x1002, 0x4167, "", "ATI Fire GL Z1 4P SECONDARY Video" } ,
	{ 0x1002, 0x4242, "R200AIW", "All-In-Wonder 8500DV" } ,
	{ 0x1002, 0x4243, "", "Lucent OHCI IEEE1394 Host Controller" } ,
	{ 0x1002, 0x4336, "", "Radeon IGP 320M" } ,
	{ 0x1002, 0x4337, "RS200M", "Mobility M6 (U2)" } ,
	{ 0x1002, 0x4354, "215CT222", "Mach 64 CT" } ,
	{ 0x1002, 0x4358, "210888CX", "Mach64 CX" } ,
	{ 0x1002, 0x4554, "Mach64 ET", "" } ,
	{ 0x1002, 0x4654, "Mach64 VT", "" } ,
	{ 0x1002, 0x4742, "ATI GTC (GT-C2U2)", "ATI 3D Rage Pro Turbo AGP 2X" } ,
	{ 0x1002, 0x4744, "Rage 3D Pro AGP 1x", "" } ,
	{ 0x1002, 0x4747, "Rage 3D Pro", "" } ,
	{ 0x1002, 0x4749, "Rage Pro Turbo PCI", "ATI ALL IN WONDER PRO (8MB)" } ,
	{ 0x1002, 0x474C, "Rage XC PCI-66", "" } ,
	{ 0x1002, 0x474D, "Rage XL AGP 2x", "" } ,
	{ 0x1002, 0x474E, "Rage XC AGP 2x", "" } ,
	{ 0x1002, 0x474F, "Rage XL PCI-66", "" } ,
	{ 0x1002, 0x4750, "Rage 3D Pro PCI", "Graphics Accelerator" } ,
	{ 0x1002, 0x4751, "Rage 3D Pro PCI", "" } ,
	{ 0x1002, 0x4752, "Rage XL PCI", "" } ,
	{ 0x1002, 0x4753, "Rage XC PCI", "" } ,
	{ 0x1002, 0x4754, "Mach 64 GT", "Rage 3D II Graphics Accelerator" } ,
	{ 0x1002, 0x4755, "Rage 3D II+", "" } ,
	{ 0x1002, 0x4756, "Rage 3D IIC PCI", "Graphics Accelerator" } ,
	{ 0x1002, 0x4757, "3D 11C AGP", "Rage 3D IIC AGP" } ,
	{ 0x1002, 0x4758, "210888GX", "Mach 64 GX (WinTurbo)" } ,
	{ 0x1002, 0x4759, "", "Rage 3D IIC" } ,
	{ 0x1002, 0x475A, "", "Rage 3D IIC AGP" } ,
	{ 0x1002, 0x4966, "RV250", "Radeon 9000/9000 Pro" } ,
	{ 0x1002, 0x496E, "RV250", "Radeon 9000/9000 Pro - Secondary" } ,
	{ 0x1002, 0x4C42, "", "Rage 3D LT Pro AGP 133 MHz" } ,
	{ 0x1002, 0x4C44, "", "Rage 3D LT Pro AGP 66 MHz" } ,
	{ 0x1002, 0x4C45, "", "Rage Mobility M3 AGP" } ,
	{ 0x1002, 0x4C46, "Mobility M3 AGP 2x", "" } ,
	{ 0x1002, 0x4C47, "", "Rage 3D LT-G" } ,
	{ 0x1002, 0x4C49, "", "Rage 3D LT Pro PCI" } ,
	{ 0x1002, 0x4C4D, "01541014", "Rage P/M Mobility AGP 2x" } ,
	{ 0x1002, 0x4C4E, "", "Rage L Mobility AGP 2x" } ,
	{ 0x1002, 0x4C50, "", "Rage 3D LT Pro PCI" } ,
	{ 0x1002, 0x4C51, "", "Rage 3D LT Pro PCI" } ,
	{ 0x1002, 0x4C52, "", "Rage P/M Mobility PCI" } ,
	{ 0x1002, 0x4C53, "", "Rage L Mobility PCI" } ,
	{ 0x1002, 0x4C54, "", "Mach 64 LT" } ,
	{ 0x1002, 0x4C57, "", "Radeon Mobility M7 LW" } ,
	{ 0x1002, 0x4C58, "", "FireGL Mobility" } ,
	{ 0x1002, 0x4C59, "Mobility 6", "Radeon Mobility M6 LY" } ,
	{ 0x1002, 0x4C5A, "", "Radeon Mobility M6 LZ" } ,
	{ 0x1002, 0x4C64, "", "Radeon Mobility M9-GL" } ,
	{ 0x1002, 0x4C66, "", "Radeon Mobility M9" } ,
	{ 0x1002, 0x4D46, "", "Rage Mobility 128 AGP 4x" } ,
	{ 0x1002, 0x4D4C, "", "Rage Mobility 128 AGP" } ,
	{ 0x1002, 0x4E44, "R300", "Radeon 9700/9500 Series" } ,
	{ 0x1002, 0x4E45, "R300", "Radeon 9700/9500 Series" } ,
	{ 0x1002, 0x4E47, "", "ATI Fire GL X1/Z1 Video Accelerator" } ,
	{ 0x1002, 0x4E64, "R300", "Radeon 9700/9500 Series - Secondary" } ,
	{ 0x1002, 0x4E65, "R300", "Radeon 9700/9500 Series - Secondary" } ,
	{ 0x1002, 0x4E67, "", "ATI Fire GL X1/Z1 SECONDARY Video" } ,
	{ 0x1002, 0x5041, "", "Rage 128 Pro PA PCI" } ,
	{ 0x1002, 0x5042, "", "Rage 128 Pro PB AGP 2x" } ,
	{ 0x1002, 0x5043, "", "Rage 128 Pro PC AGP 4x" } ,
	{ 0x1002, 0x5044, "", "Rage 128 Pro PD PCI" } ,
	{ 0x1002, 0x5045, "", "Rage 128 Pro PE AGP 2x" } ,
	{ 0x1002, 0x5046, "", "Rage 128 Pro PF AGP 4x" } ,
	{ 0x1002, 0x5047, "", "Rage 128 Pro PG PCI" } ,
	{ 0x1002, 0x5048, "Rage 128 Pro PH AGP ", "Rage 128 Pro PH AGP 2x" } ,
	{ 0x1002, 0x5049, "Rage 128 Pro PI AGP ", "Rage 128 Pro PI AGP 4x" } ,
	{ 0x1002, 0x504A, "Rage 128 Pro PJ PCI", "Rage 128 Pro PJ PCI (TMDS)" } ,
	{ 0x1002, 0x504B, "Rage 128 Pro PK AGP ", "Rage 128 Pro PK AGP 2x (TMDS)" } ,
	{ 0x1002, 0x504C, "Rage 128 Pro PL AGP ", "Rage 128 Pro PL AGP 4x (TMDS)" } ,
	{ 0x1002, 0x504D, "Rage 128 Pro PM PCI", "Rage 128 Pro PM PCI" } ,
	{ 0x1002, 0x504E, "Rage 128 Pro PN AGP ", "Rage 128 Pro PN AGP 2x" } ,
	{ 0x1002, 0x504F, "Rage 128 Pro PO AGP ", "Rage 128 Pro PO AGP 4x" } ,
	{ 0x1002, 0x5050, "Rage 128 Pro PP PCI", "Rage 128 Pro PP PCI (TMDS)" } ,
	{ 0x1002, 0x5051, "Rage 128 Pro PQ AGP ", "Rage 128 Pro PQ AGP 2x (TMDS)" } ,
	{ 0x1002, 0x5052, "Rage 128 Pro PR AGP ", "Rage 128 Pro PR AGP 4x (TMDS)" } ,
	{ 0x1002, 0x5053, "Rage 128 Pro PS PCI", "Rage 128 Pro PS PCI" } ,
	{ 0x1002, 0x5054, "Rage 128 Pro PT AGP ", "Rage 128 Pro PT AGP 2x" } ,
	{ 0x1002, 0x5055, "Rage 128 Pro PU AGP ", "Rage 128 Pro PU AGP 4x" } ,
	{ 0x1002, 0x5056, "Rage 128 Pro PV PCI", "Rage 128 Pro PV PCI (TMDS)" } ,
	{ 0x1002, 0x5057, "Rage 128 Pro PW AGP ", "Rage 128 Pro PW AGP 2x (TMDS)" } ,
	{ 0x1002, 0x5058, "Rage 128 Pro PX AGP ", "Rage 128 Pro PX AGP 4x (TMDS)" } ,
	{ 0x1002, 0x5144, "", "Radeon 7200 QD SDR/DDR" } ,
	{ 0x1002, 0x5145, "", "Radeon QE" } ,
	{ 0x1002, 0x5146, "", "Radeon QF" } ,
	{ 0x1002, 0x5147, "", "Radeon QG" } ,
	{ 0x1002, 0x5148, "R200", "Radeon R200 QH" } ,
	{ 0x1002, 0x5149, "", "Radeon R200 QI" } ,
	{ 0x1002, 0x514A, "", "Radeon R200 QJ" } ,
	{ 0x1002, 0x514B, "", "Radeon R200 QK" } ,
	{ 0x1002, 0x514C, "R200", "Radeon 8500 / 8500LE" } ,
	{ 0x1002, 0x514E, "", "Radeon R200 QM" } ,
	{ 0x1002, 0x514F, "", "Radeon R200 QN" } ,
	{ 0x1002, 0x5157, "RV200", "Radeon 7500" } ,
	{ 0x1002, 0x5158, "", "Radeon 7500 QX" } ,
	{ 0x1002, 0x5159, "Radeon VE QY", "Radeon 7000 / Radeon VE" } ,
	{ 0x1002, 0x515A, "", "Radeon VE QZ" } ,
	{ 0x1002, 0x5168, "", "Radeon R200 Qh" } ,
	{ 0x1002, 0x5169, "", "Radeon R200 Qi" } ,
	{ 0x1002, 0x516A, "", "Radeon R200 Qj" } ,
	{ 0x1002, 0x516B, "", "Radeon R200 Qk" } ,
	{ 0x1002, 0x516C, "", "Radeon 8500 / 8500LE" } ,
	{ 0x1002, 0x5245, "", "Rage 128 GL PCI" } ,
	{ 0x1002, 0x5246, "Rage 128 GL AGP 2x", "Rage Fury 16/32MB" } ,
	{ 0x1002, 0x5247, "", "Rage 128 RG" } ,
	{ 0x1002, 0x524B, "", "Rage 128 VR RK PCI" } ,
	{ 0x1002, 0x524C, "", "Rage 128 VR RL AGP 2x" } ,
	{ 0x1002, 0x5345, "", "Rage 128 4x SE PCI" } ,
	{ 0x1002, 0x5346, "", "Rage 128 SF 4x AGP 2x" } ,
	{ 0x1002, 0x5347, "", "Rage 128 SG 4x AGP 4x" } ,
	{ 0x1002, 0x5348, "", "Rage 128 4x SH" } ,
	{ 0x1002, 0x534B, "Rage 128 SK PCI", "Rage 128 4x SK PCI" } ,
	{ 0x1002, 0x534C, "Rage 128 SL AGP 2x", "Rage 128 4x SL AGP 2x" } ,
	{ 0x1002, 0x534D, "Rage 128 SM AGP 4x", "Rage 128 4x SM AGP 4x" } ,
	{ 0x1002, 0x534E, "Rage 128 4x", "" } ,
	{ 0x1002, 0x5354, "", "Mach 64 ST" } ,
	{ 0x1002, 0x5446, "Rage 128 PRO ULTRA", "" } ,
	{ 0x1002, 0x544C, "", "Rage 128 Pro TL" } ,
	{ 0x1002, 0x5452, "", "Rage 128 Pro TR" } ,
	{ 0x1002, 0x5455, "", "Rage 128 Pro Ultra TU" } ,
	{ 0x1002, 0x5654, "215VT222", "Mach 64 VT VIDEO XPRESSION" } ,
	{ 0x1002, 0x5655, "", "Mach 64 VT3" } ,
	{ 0x1002, 0x5656, "", "Mach 64 VT4 PCI" } ,
	{ 0x1002, 0x700F, "", "PCI to AGP Bridge" } ,
	{ 0x1002, 0xCAB0, "", "CPU to PCI Bridge" } ,
	{ 0x1003, 0x0201, "US201", "GUI Accelerator" } ,
	{ 0x1004, 0x0005, "82C591/2-FC1", "CPU Bridge" } ,
	{ 0x1004, 0x0006, "82C593", "ISA Bridge" } ,
	{ 0x1004, 0x0007, "82C594", "Wildcat System Controller" } ,
	{ 0x1004, 0x0008, "82C596/597", "Wildcat ISA Bridge" } ,
	{ 0x1004, 0x0009, "82C597-AFC2", "" } ,
	{ 0x1004, 0x000C, "82C541", "" } ,
	{ 0x1004, 0x000D, "82C543", "" } ,
	{ 0x1004, 0x0100, "", "CPU to PCI Bridge for notebook" } ,
	{ 0x1004, 0x0101, "82C532", "Peripheral Controller" } ,
	{ 0x1004, 0x0102, "82C534", "PCI to PCI Bridge" } ,
	{ 0x1004, 0x0103, "82C538", "PCI to ISA Bridge" } ,
	{ 0x1004, 0x0104, "82C535", "Host Bridge" } ,
	{ 0x1004, 0x0105, "82C147", "IrDA Controller" } ,
	{ 0x1004, 0x0200, "82C975", "RISC GUI Accelerator" } ,
	{ 0x1004, 0x0280, "82C925", "RISC GUI Accelerator" } ,
	{ 0x1004, 0x0304, "SAA7785", "ThunderBird PCI Audio Accelerator" } ,
	{ 0x1004, 0x0305, "SAA7785", "ThunderBird joystick port" } ,
	{ 0x1004, 0x0306, "SAA7785", "ThunderBird 16650 UART" } ,
	{ 0x1004, 0x0307, "", "Philips Seismic Edge 705" } ,
	{ 0x1004, 0x0308, "", "Philips PSC705 GamePort Enumerator" } ,
	{ 0x1004, 0x0702, "VAS96011", "Golden Gate II" } ,
	{ 0x1005, 0x2064, "ALG2032/2064", "alg2032  63067s1" } ,
	{ 0x1005, 0x2128, "ALG2364A", "" } ,
	{ 0x1005, 0x2301, "ALG2301", "GUI Accelerator" } ,
	{ 0x1005, 0x2302, "ALG2302", "GUI Accelerator" } ,
	{ 0x1005, 0x2364, "AL2364", "GUI Accelerator" } ,
	{ 0x1005, 0x2464, "ALG2364A", "" } ,
	{ 0x1005, 0x2501, "ALG2564A/25128A", "" } ,
	{ 0x100B, 0x0001, "DP83810", "10/100 Ethernet MAC" } ,
	{ 0x100B, 0x0002, "PC87415", "PCI-IDE DMA Master Mode Interface Ctrlr" } ,
	{ 0x100B, 0x000E, "PC87560", "Legacy I/O Controller" } ,
	{ 0x100B, 0x000F, "CS4210", "IEEE 1394 OHCI Controller" } ,
	{ 0x100B, 0x0011, "PC87560", "PCI System I/O" } ,
	{ 0x100B, 0x0012, "", "USB Controller" } ,
	{ 0x100B, 0x001B, "LM4560", "Advanced PCI Audio Accelerator" } ,
	{ 0x100B, 0x0020, "DP83815/16", "MacPhyter 10/100 Mb/s Ethernet MAC & PHY" } ,
	{ 0x100B, 0x0021, "PC87200", "PCI to ISA Bridge" } ,
	{ 0x100B, 0x0022, "DP83820/1", "10/100/1000 Mb/s PCI Ethernet NIC" } ,
	{ 0x100B, 0x0500, "SCx200", "Bridge" } ,
	{ 0x100B, 0x0501, "SCx200", "SMI" } ,
	{ 0x100B, 0x0502, "SCx200", "IDE" } ,
	{ 0x100B, 0x0503, "SCx200", "Audio" } ,
	{ 0x100B, 0x0504, "SCx200", "Video" } ,
	{ 0x100B, 0x0505, "SCx200", "XBus" } ,
	{ 0x100B, 0xD001, "PC87410", "PCI-IDE Interface" } ,
	{ 0x100C, 0x3202, "ET4000W32P-A", "GUI Accelerator" } ,
	{ 0x100C, 0x3205, "ET4000W32P-B", "GUI Accelerator" } ,
	{ 0x100C, 0x3206, "ET4000W32P-C", "GUI Accelerator" } ,
	{ 0x100C, 0x3207, "ET4000W32P-D", "GUI Accelerator" } ,
	{ 0x100C, 0x3208, "ET6000", "Graphics/Multimedia Engine" } ,
	{ 0x100C, 0x4702, "ET6300", "" } ,
	{ 0x100E, 0x0564, "STPC Client", "Host Bridge" } ,
	{ 0x100E, 0x55CC, "STPC Client", "South Bridge" } ,
	{ 0x100E, 0x9000, "P9000", "WeitekPower GUI Accelerator" } ,
	{ 0x100E, 0x9001, "P9000", "GUI Accelerator" } ,
	{ 0x100E, 0x9100, "P9100", "GUI Accelerator" } ,
	{ 0x1011, 0x0001, "DC21050", "PCI-PCI Bridge" } ,
	{ 0x1011, 0x0002, "DC21040", "Tulip Ethernet Adapter" } ,
	{ 0x1011, 0x0004, "DC21030", "PCI Graphics Accelerator" } ,
	{ 0x1011, 0x0007, "Zephyr", "NV-RAM" } ,
	{ 0x1011, 0x0008, "KZPSA", "SCSI to SCSI Adapter" } ,
	{ 0x1011, 0x0009, "DC21140", "Fast Ethernet Ctrlr" } ,
	{ 0x1011, 0x000A, "DC21230", "Video Codec" } ,
	{ 0x1011, 0x000C, "DC21130", "PCI Integrated Graphics & Video Accel" } ,
	{ 0x1011, 0x000D, "TGA2", "TGA2 PDXGB" } ,
	{ 0x1011, 0x000F, "DEFPA", "FDDI" } ,
	{ 0x1011, 0x0014, "DC21041", "Tulip Plus Ethernet Adapter" } ,
	{ 0x1011, 0x0016, "DGLPB", "ATM" } ,
	{ 0x1011, 0x0019, "DC21142/3", "PCI/CardBus 10/100 Mbit Ethernet Ctlr" } ,
	{ 0x1011, 0x0021, "21052[-AB]", "PCI-PCI Bridge" } ,
	{ 0x1011, 0x0022, "DC21150-AA", "PCI-PCI Bridge" } ,
	{ 0x1011, 0x0023, "DC21150", "PCI to PCI Bridge" } ,
	{ 0x1011, 0x0024, "DC21151/2", "PCI-PCI Bridge" } ,
	{ 0x1011, 0x0025, "21153", "PCI-PCI Bridge" } ,
	{ 0x1011, 0x0026, "21154", "PCI-PCI Bridge" } ,
	{ 0x1011, 0x0034, "Modem56", "CardBus" } ,
	{ 0x1011, 0x0045, "DC21553", "PCI to PCI Bridge" } ,
	{ 0x1011, 0x0046, "21554", "PCI-to-PCI Bridge" } ,
	{ 0x1011, 0x1065, "21285", "Core Logic for SA-110 Microprocessor" } ,
	{ 0x1013, 0x0038, "CL-GD7548", "GUI-Accelerated XGA/SVGA LCD Controller" } ,
	{ 0x1013, 0x0040, "CL-GD7555", "Flat Panel GUI Accelerator" } ,
	{ 0x1013, 0x004C, "CL-GD7556", "64-bit Accelerated LCD/CRT Controller" } ,
	{ 0x1013, 0x00A0, "CL-GD5340", "GUI Accelerator" } ,
	{ 0x1013, 0x00A2, "CL-GD5432", "Alpine GUI Accelerator" } ,
	{ 0x1013, 0x00A4, "CL-GD5434", "Alpine GUI Accelerator" } ,
	{ 0x1013, 0x00A8, "CL-GD5434", "Alpine GUI Accelerator" } ,
	{ 0x1013, 0x00AC, "CL-GD5436", "Alpine GUI Accelerator" } ,
	{ 0x1013, 0x00B8, "CL-GD5446", "64-bit VisualMedia Accelerator" } ,
	{ 0x1013, 0x00BC, "CL-GD5480", "64-bit SGRAM GUI accelerator" } ,
	{ 0x1013, 0x00D0, "CL-GD5462", "Laguna VisualMedia graphics accelerator" } ,
	{ 0x1013, 0x00D4, "CL-GD5464", "Laguna 3D VisualMedia Graphics Accel" } ,
	{ 0x1013, 0x00D5, "CL-GD5464", "Laguna BD" } ,
	{ 0x1013, 0x00D6, "CL-GD5465", "Laguna 3D VisualMedia Graphics Accel" } ,
	{ 0x1013, 0x00E8, "CL-GD5436U", "" } ,
	{ 0x1013, 0x1100, "CL-PD6729", "PCI-to-PC Card host adapter" } ,
	{ 0x1013, 0x1110, "CL-PD6832", "PCMCIA/CardBus Controller" } ,
	{ 0x1013, 0x1112, "CL-PD6834", "PCMCIA/CardBus Controller" } ,
	{ 0x1013, 0x1113, "CL-PD6833", "PCI-to-CardBus Host Adapter" } ,
	{ 0x1013, 0x1200, "CL-GD7542", "Nordic GUI Accelerator" } ,
	{ 0x1013, 0x1202, "CL-GD7543", "Viking GUI Accelerator" } ,
	{ 0x1013, 0x1204, "CL-GD7541", "Nordic-lite VGA Cntrlr" } ,
	{ 0x1013, 0x4400, "CL-CD4400", "Communications Controller" } ,
	{ 0x1013, 0x6001, "CS4610", "CrystalClear SoundFusion PCI Audio Accel" } ,
	{ 0x1013, 0x6003, "CS4610/14/22/24/30", "CrystalClear SoundFusion PCI Audio Accel" } ,
	{ 0x1013, 0x6004, "CS4615", "CrystalClear SoundFusion PCI Audio Accel" } ,
	{ 0x1013, 0x6005, "CS4281", "CrystalClear PCI Audio Interface" } ,
	{ 0x1014, 0x0002, "MCA Bridge", "MCA Bridge" } ,
	{ 0x1014, 0x0005, "Alta Lite", "CPU Bridge" } ,
	{ 0x1014, 0x0007, "Alta MP", "CPU Bridge" } ,
	{ 0x1014, 0x000A, "Fire Coral", "ISA Bridge w/PnP" } ,
	{ 0x1014, 0x0017, "", "CPU to PCI Bridge" } ,
	{ 0x1014, 0x0018, "Auto LANStreamer", "TR Auto LANStreamer" } ,
	{ 0x1014, 0x001B, "GXT-150P", "Graphics Adapter" } ,
	{ 0x1014, 0x001D, "82G2675", "scsi-2 fast pci adapter" } ,
	{ 0x1014, 0x0020, "", "MCA Bridge" } ,
	{ 0x1014, 0x0022, "82351/2", "PCI to PCI Bridge" } ,
	{ 0x1014, 0x002D, "Python", "" } ,
	{ 0x1014, 0x002E, "ServeRAID I/II/3x/4H", "Coppertime RAID SCSI Adapter" } ,
	{ 0x1014, 0x0036, "Miami/PCI", "32-bit LocalBus Bridge" } ,
	{ 0x1014, 0x0037, "IBM27-82660", "PowerPC to PCI Bridge and Memory Ctrlr" } ,
	{ 0x1014, 0x003A, "", "CPU to PCI Bridge" } ,
	{ 0x1014, 0x003E, "85H9533", "16/4 Token Ring PCI IBM UTP/STP Ctrlr" } ,
	{ 0x1014, 0x0045, "", "SSA Adapter" } ,
	{ 0x1014, 0x0046, "MPIC", "Interrupt Controller" } ,
	{ 0x1014, 0x0047, "", "PCI to PCI Bridge" } ,
	{ 0x1014, 0x0048, "", "PCI to PCI Bridge" } ,
	{ 0x1014, 0x0049, "", "Warhead SCSI Controller" } ,
	{ 0x1014, 0x004E, "", "ATM Controller" } ,
	{ 0x1014, 0x004F, "", "ATM Controller" } ,
	{ 0x1014, 0x0050, "", "ATM Controller" } ,
	{ 0x1014, 0x0053, "", "25 MBit ATM controller" } ,
	{ 0x1014, 0x0057, "", "MPEG PCI Bridge" } ,
	{ 0x1014, 0x005C, "i82557B", "10/100 PCI Ethernet Adapter" } ,
	{ 0x1014, 0x005D, "05J3506", "TCP/IP networking device" } ,
	{ 0x1014, 0x007C, "", "ATM Controller" } ,
	{ 0x1014, 0x007D, "3780IDSP", "MPEG-2 Decoder" } ,
	{ 0x1014, 0x0090, "GXT-3000P", "" } ,
	{ 0x1014, 0x0095, "20H2999", "PCI Docking Bridge" } ,
	{ 0x1014, 0x0096, "", "Chukar chipset SCSI Controller" } ,
	{ 0x1014, 0x00A1, "PowerNP NPr2.7", "ATM support device" } ,
	{ 0x1014, 0x00A5, "", "ATM Controller" } ,
	{ 0x1014, 0x00A6, "", "ATM 155Mbps MM Controller" } ,
	{ 0x1014, 0x00B7, "GXT2000", "256-bit Graphics Rasterizer" } ,
	{ 0x1014, 0x00BE, "", "ATM 622Mbps Controller" } ,
	{ 0x1014, 0x00CE, "02li537", "Adapter 2 Token Ring Card" } ,
	{ 0x1014, 0x00F9, "CPC700", "Memory Controller and PCI Bridge" } ,
	{ 0x1014, 0x00FC, "CPC710", "PCI-64 Bridge" } ,
	{ 0x1014, 0x0105, "CPC710", "PCI-32 Bridge" } ,
	{ 0x1014, 0x010F, "", "Remote Supervisor+Serial Port+Mouse/Keyb" } ,
	{ 0x1014, 0x011B, "", "Raid controller" } ,
	{ 0x1014, 0x0142, "Yotta", "Video Compositor Input" } ,
	{ 0x1014, 0x0144, "Yotta", "Video Compositor Output" } ,
	{ 0x1014, 0x0156, "405GP", "PLB to PCI Bridge" } ,
	{ 0x1014, 0x0170, "RC1000", "Rasterizer/IBM GT1000 Geometr" } ,
	{ 0x1014, 0x01A7, "IBM 133", "PCI-X Bridge R1.1" } ,
	{ 0x1014, 0x01BD, "ServeRAID 4/5", "Morpheus SCSI RAID Controller" } ,
	{ 0x1014, 0x01ef, "440GP", "PLB to PCI-X Bridge" } ,
	{ 0x1014, 0x0302, "", "PCI-X Host Bridge" } ,
	{ 0x1014, 0xFFFF, "MPIC 2", "Interrupt Controller" } ,
	{ 0x1017, 0x5343, "", "SPEA 3D Accelerator" } ,
	{ 0x101A, 0x0005, "8156", "100VG/AnyLAN Adapter" } ,
	{ 0x101A, 0x0009, "Altera FLEX", "??? Raid Controller ???" } ,
	{ 0x101C, 0x0193, "WD33C193A", "8-bit SCSI Cntrlr" } ,
	{ 0x101C, 0x0196, "WD33C196A", "PCI-SCSI Bridge" } ,
	{ 0x101C, 0x0197, "WD33C197A", "16-bit SCSI Cntrlr" } ,
	{ 0x101C, 0x0296, "WD33C296A", "high perf 16-bit SCSI Cntrlr" } ,
	{ 0x101C, 0x3193, "WD7193", "Fast SCSI-II" } ,
	{ 0x101C, 0x3197, "WD7197", "Fast-wide SCSI-II" } ,
	{ 0x101C, 0x3296, "WD33C296A", "Fast Wide SCSI bridge" } ,
	{ 0x101C, 0x4296, "WD34C296", "Wide Fast-20 Bridge" } ,
	{ 0x101C, 0x9710, "Pipeline 9710", "" } ,
	{ 0x101C, 0x9712, "Pipeline 9712", "" } ,
	{ 0x101C, 0xC24A, "90C", "" } ,
	{ 0x101e, 0x1960, "80960RP", "i960RP Microprocessor" } ,
	{ 0x101E, 0x9010, "MegaRAID 428", "Ultra Wide SCSI RAID Controller" } ,
	{ 0x101E, 0x9030, "", "EIDE Controller" } ,
	{ 0x101E, 0x9031, "", "EIDE Controller" } ,
	{ 0x101E, 0x9032, "", "IDE and SCSI Cntrlr" } ,
	{ 0x101E, 0x9033, "", "SCSI Controller" } ,
	{ 0x101E, 0x9040, "", "Multimedia card" } ,
	{ 0x101E, 0x9060, "MegaRAID 434", "Ultra GT RAID Controller" } ,
	{ 0x101E, 0x9063, "MegaRAC", "" } ,
	{ 0x1022, 0x2000, "Am79C970/1/2/3/5/6", "PCnet LANCE PCI Ethernet Controller" } ,
	{ 0x1022, 0x2001, "Am79C978", "PCnet-Home Networking Ctrlr (1/10 Mbps)" } ,
	{ 0x1022, 0x2020, "53C974", "SCSI Ctrlr" } ,
	{ 0x1022, 0x2040, "79C974", "Ethernet & SCSI Ctrlr" } ,
	{ 0x1022, 0x3000, "", "ELAN SC520 Rev. B0" } ,
	{ 0x1022, 0x7004, "AMD-751", "CPU to PCI Bridge" } ,
	{ 0x1022, 0x7006, "AMD-751", "Processor-to-PCI Bridge / Memory Ctrlr" } ,
	{ 0x1022, 0x7007, "AMD-751", "AGP and PCI-to-PCI Bridge (1x/2x AGP)" } ,
	{ 0x1022, 0x700C, "AMD-762", "CPU to PCI Bridge (SMP chipset)" } ,
	{ 0x1022, 0x700D, "AMD-762", "CPU to PCI Bridge (AGP 4x)" } ,
	{ 0x1022, 0x700E, "AMD-761", "North Bridge" } ,
	{ 0x1022, 0x700F, "AMD-761", "CPU to AGP Bridge  (AGP 4x)" } ,
	{ 0x1022, 0x7400, "AMD-755", "PCI to ISA Bridge" } ,
	{ 0x1022, 0x7401, "AMD-755", "Bus Master IDE Controller" } ,
	{ 0x1022, 0x7403, "AMD-755", "Power Management Controller" } ,
	{ 0x1022, 0x7404, "AMD-755", "PCI to USB Open Host Controller" } ,
	{ 0x1022, 0x7408, "AMD-756", "PCI-ISA Bridge" } ,
	{ 0x1022, 0x7409, "AMD-756", "EIDE Controller" } ,
	{ 0x1022, 0x740B, "AMD-756", "Power Management" } ,
	{ 0x1022, 0x740C, "AMD-756", "USB Open Host Controller" } ,
	{ 0x1022, 0x7410, "AMD-766", "PCI to ISA/LPC Bridge" } ,
	{ 0x1022, 0x7411, "AMD-766", "Enhanced IDE Controller" } ,
	{ 0x1022, 0x7412, "AMD-766", "USB Controller" } ,
	{ 0x1022, 0x7413, "AMD-766", "Power Management Controller" } ,
	{ 0x1022, 0x7414, "AMD-766", "USB OpenHCI Host Controller" } ,
	{ 0x1022, 0x7440, "AMD-768", "LPC Bridge" } ,
	{ 0x1022, 0x7441, "AMD-768", "EIDE Controller" } ,
	{ 0x1022, 0x7443, "AMD-768", "System Management" } ,
	{ 0x1022, 0x7445, "AMD-768", "AC97 Audio" } ,
	{ 0x1022, 0x7446, "AMD-768", "AC97 Modem" } ,
	{ 0x1022, 0x7448, "AMD-768", "PCI Bridge" } ,
	{ 0x1022, 0x7449, "AMD-768", "USB Controller" } ,
	{ 0x1022, 0x7454, "AMD-8151", "System Controller" } ,
	{ 0x1022, 0x7455, "AMD-8151", "AGP Bridge" } ,
	{ 0x1022, 0x7460, "AMD-8111", "PCI Bridge" } ,
	{ 0x1022, 0x7461, "AMD-8111", "USB 2.0 Controller" } ,
	{ 0x1022, 0x7462, "AMD-8111", "Ethernet Controller" } ,
	{ 0x1022, 0x7463, "", "Enhanced USB Controller" } ,
	{ 0x1022, 0x7464, "", "OpenHCI USB Host Controller" } ,
	{ 0x1022, 0x7468, "AMD-8111", "LPC Bridge" } ,
	{ 0x1022, 0x7469, "AMD-8111", "UltraATA/133 Controller" } ,
	{ 0x1022, 0x746A, "AMD-8111", "SMBus 2.0 Controller" } ,
	{ 0x1022, 0x746B, "AMD-8111", "ACPI Controller" } ,
	{ 0x1022, 0x746D, "AMD-8111", "AC97 Audio Controller" } ,
	{ 0x1022, 0x756B, "AMD-8111", "ACPI Controller" } ,
	{ 0x1023, 0x0194, "82C194", "CardBus Controller" } ,
	{ 0x1023, 0x2000, "4DWAVE-DX", "advanced PCI DirectSound accelerator" } ,
	{ 0x1023, 0x2001, "4DWAVE-NX", "PCI Audio" } ,
	{ 0x1023, 0x8400, "CyberBlade i7", "" } ,
	{ 0x1023, 0x8420, "CyberBlade i7 AGP", "" } ,
	{ 0x1023, 0x8500, "CyberBlade i1", "AGP 51 (77?)" } ,
	{ 0x1023, 0x8520, "CyberBlade i1 AGP", "" } ,
	{ 0x1023, 0x8620, "CyberBlade-A i1", "" } ,
	{ 0x1023, 0x8820, "CyberBlade XPAi1", "" } ,
	{ 0x1023, 0x9320, "TGUI9320", "32-bit GUI Accelerator" } ,
	{ 0x1023, 0x9350, "TGUI9350", "32-bit GUI Accelerator" } ,
	{ 0x1023, 0x9360, "", "Flat panel Cntrlr" } ,
	{ 0x1023, 0x9382, "Cyber9382", "" } ,
	{ 0x1023, 0x9383, "Cyber9383", "" } ,
	{ 0x1023, 0x9385, "Cyber9385", "" } ,
	{ 0x1023, 0x9386, "Cyber9386", "Video Accelerator" } ,
	{ 0x1023, 0x9388, "Cyber9388", "Video Accelerator" } ,
	{ 0x1023, 0x9397, "Cyber9397", "Video Accelerator" } ,
	{ 0x1023, 0x939A, "Cyber9397DVD", "Video Accelerator" } ,
	{ 0x1023, 0x9420, "TGUI9420", "DGi GUI Accelerator" } ,
	{ 0x1023, 0x9430, "TGUI9430", "GUI Accelerator" } ,
	{ 0x1023, 0x9440, "TGUI9440", "DGi GUI Acclerator" } ,
	{ 0x1023, 0x9460, "TGUI9460", "32-bit GUI Accelerator" } ,
	{ 0x1023, 0x9470, "TGUI9470", "" } ,
	{ 0x1023, 0x9520, "Cyber9520", "Video Accelerator" } ,
	{ 0x1023, 0x9525, "Cyber9525", "Video Accelerator" } ,
	{ 0x1023, 0x9540, "Cyber9540", "Video Acclerator" } ,
	{ 0x1023, 0x9660, "TGUI9660XGi", "GUI Accelerator" } ,
	{ 0x1023, 0x9680, "TGUI9680", "GUI Accelerator" } ,
	{ 0x1023, 0x9682, "TGUI9682", "Multimedia Accelerator" } ,
	{ 0x1023, 0x9683, "TGUI9683", "GUI Accelerator" } ,
	{ 0x1023, 0x9685, "ProVIDIA 9685", "" } ,
	{ 0x1023, 0x9750, "3DImage 9750 PCI/AGP", "trident dgi" } ,
	{ 0x1023, 0x9753, "TGUI9753", "Video Accelerator" } ,
	{ 0x1023, 0x9754, "TGUI9753", "Wave Video Accelerator" } ,
	{ 0x1023, 0x9759, "TGUI975?", "Image GUI Accelerator" } ,
	{ 0x1023, 0x9783, "TGUI9783", "" } ,
	{ 0x1023, 0x9785, "TGUI9785", "" } ,
	{ 0x1023, 0x9850, "3D Image 9850 AGP", "" } ,
	{ 0x1023, 0x9880, "Blade 3D 9880", "" } ,
	{ 0x1023, 0x9910, "", "CyberBlade XP" } ,
	{ 0x1023, 0x9930, "CyberBlade XPm", "" } ,
	{ 0x1024, 0x1024, "R6785-61", "HCF 56k PCI Modem" } ,
	{ 0x1025, 0x1435, "M1435", "CPU to PCI & PCI to ISA Bridge" } ,
	{ 0x1025, 0x1445, "M1445", "VL Bridge & EIDE" } ,
	{ 0x1025, 0x1449, "M1449", "ISA Bridge" } ,
	{ 0x1025, 0x1451, "M1451", "Pentium Chipset" } ,
	{ 0x1025, 0x1461, "M1461", "P54C Chipset" } ,
	{ 0x1025, 0x1489, "M1489", "" } ,
	{ 0x1025, 0x1511, "M1511", "" } ,
	{ 0x1025, 0x1512, "M1512", "" } ,
	{ 0x1025, 0x1513, "M1513", "" } ,
	{ 0x1025, 0x1521, "M1521", "CPU Bridge" } ,
	{ 0x1025, 0x1523, "M1523", "ISA Bridge" } ,
	{ 0x1025, 0x1531, "M1531", "North Bridge" } ,
	{ 0x1025, 0x1533, "M1533", "ISA South Bridge" } ,
	{ 0x1025, 0x1535, "M1535", "PCI South Bridge" } ,
	{ 0x1025, 0x1541, "M1541", "AGP PCI North Bridge Aladdin V/V+" } ,
	{ 0x1025, 0x1542, "M1542", "AGP+PCI North Bridge" } ,
	{ 0x1025, 0x1543, "M1543C", "PCi South Bridge Aladdin IV+/V" } ,
	{ 0x1025, 0x1561, "M1561", "Northbridge" } ,
	{ 0x1025, 0x1621, "M1621", "PCI North Bridge Aladdin Pro II" } ,
	{ 0x1025, 0x1631, "M1631", "PCI North Bridge Aladdin Pro III" } ,
	{ 0x1025, 0x1641, "M1641", "PCI North Bridge Aladdin Pro IV" } ,
	{ 0x1025, 0x3141, "M3141", "GUI Accelerator" } ,
	{ 0x1025, 0x3143, "M3143", "GUI Accelerator" } ,
	{ 0x1025, 0x3145, "M3145", "GUI Accelerator" } ,
	{ 0x1025, 0x3147, "M3147", "GUI Accelerator" } ,
	{ 0x1025, 0x3149, "M3149", "GUI Accelerator" } ,
	{ 0x1025, 0x3151, "M3151", "GUI Accelerator" } ,
	{ 0x1025, 0x3307, "M3307", "MPEG-1 Decoder" } ,
	{ 0x1025, 0x3309, "M3309", "MPEG Decoder" } ,
	{ 0x1025, 0x5212, "M4803", "" } ,
	{ 0x1025, 0x5215, "M5217", "EIDE Controller" } ,
	{ 0x1025, 0x5217, "M5217", "I/O Controller" } ,
	{ 0x1025, 0x5219, "M5219", "I/O Controller" } ,
	{ 0x1025, 0x5225, "M5225", "EIDE Controller" } ,
	{ 0x1025, 0x5229, "M5229", "EIDE Controller" } ,
	{ 0x1025, 0x5235, "M5235", "I/O Controller" } ,
	{ 0x1025, 0x5237, "M5237", "PCI USB Host Controller" } ,
	{ 0x1025, 0x5240, "", "EIDE Controller" } ,
	{ 0x1025, 0x5241, "", "PCMCIA Bridge" } ,
	{ 0x1025, 0x5242, "", "General Purpose Controller" } ,
	{ 0x1025, 0x5243, "", "PCI to PCI Bridge" } ,
	{ 0x1025, 0x5244, "", "Floppy Disk Controller" } ,
	{ 0x1025, 0x5247, "M1541", "PCI-PCI Bridge" } ,
	{ 0x1025, 0x5427, "", "PCI to AGP Bridge" } ,
	{ 0x1025, 0x5451, "M5451", "PCI AC-Link Controller Audio Device" } ,
	{ 0x1025, 0x5453, "M5453", "M5453 AC-Link Controller Modem Device" } ,
	{ 0x1025, 0x7101, "M7101", "PCI PMU Power Management Controller" } ,
	{ 0x1028, 0x0001, "PowerEdge 2 /Si", "Expandable RAID Controller" } ,
	{ 0x1028, 0x0002, "PowerEdge 3/Di", "Expandable RAID Controller" } ,
	{ 0x1028, 0x0003, "PowerEdge 3/Si", "Expandable RAID Controller" } ,
	{ 0x1028, 0x0004, "PowerEdge 3/Si", "Expandable RAID Controller" } ,
	{ 0x1028, 0x0005, "PowerEdge 3/Di", "Expandable RAID Controller" } ,
	{ 0x1028, 0x0006, "PowerEdge 3/Di", "Expandable RAID Controller" } ,
	{ 0x1028, 0x0007, "", "Remote Assistant Card 3" } ,
	{ 0x1028, 0x0008, "PowerEdge 3/Di", "Expandable RAID Controller" } ,
	{ 0x1028, 0x000A, "PowerEdge 3/Di", "Expandable RAID Controller" } ,
	{ 0x1028, 0x000C, "", "Embedded Systems Management Device 4" } ,
	{ 0x102A, 0x0000, "HYDRA", "P5 Chipset" } ,
	{ 0x102A, 0x0010, "ASPEN", "i486 Chipset" } ,
	{ 0x102a, 0x0310, "L64364", "ATMizer II+ ATM SAR Chip" } ,
	{ 0x102B, 0x0010, "MGA-I", "Impression?" } ,
	{ 0x102B, 0x0518, "MGA-PX2085", "Atlas GUI Accelerator" } ,
	{ 0x102B, 0x0519, "MGA-2064W", "Strorm GUI Accelerator" } ,
	{ 0x102B, 0x051A, "MGA 1064SG", "Hurricane/Cyclone 64-bit graphics chip" } ,
	{ 0x102B, 0x051B, "MGA-21164W", "Mistral" } ,
	{ 0x102B, 0x051E, "MGA-1164SG", "Chinook" } ,
	{ 0x102B, 0x051F, "MGA2164WA-B", "Mistral" } ,
	{ 0x102B, 0x0520, "MGA-G200B", "Eclipse/Calao" } ,
	{ 0x102B, 0x0521, "MGA-G200", "Eclipse/Calao" } ,
	{ 0x102b, 0x0525, "MGA-G400/450", "Toucan/Condor" } ,
	{ 0x102B, 0x0527, "Parhelia AGP", "" } ,
	{ 0x102B, 0x0D10, "MGA-I", "Athena GUI accelerator" } ,
	{ 0x102B, 0x1000, "MGA-G100", "Twister" } ,
	{ 0x102B, 0x1001, "MGA-G100", "Twister AGP" } ,
	{ 0x102B, 0x1525, "Fusion G450 AGP", "" } ,
	{ 0x102B, 0x1527, "Fusion Plus G800 AGP", "" } ,
	{ 0x102B, 0x2007, "Mistral", "GUI+3D Accelerator" } ,
	{ 0x102B, 0x2527, "MGA-G550", "AGP Chipset" } ,
	{ 0x102B, 0x4536, "Meteor 2 STD/MC/Dig", "Video Capture Card" } ,
	{ 0x102B, 0x6573, "Shark", "10/100 Multiport Switch NIC" } ,
	{ 0x102C, 0x00B8, "64310", "Wingine DGX - DRAM Graphics Accelerator" } ,
	{ 0x102C, 0x00C0, "69000", "AGP/PCI Flat Panel/CRT VGA Accelerator" } ,
	{ 0x102C, 0x00D0, "65545", "Flat panel/crt VGA Cntrlr" } ,
	{ 0x102C, 0x00D8, "65540", "Flat Panel/CRT VGA Controller" } ,
	{ 0x102C, 0x00DC, "65548", "GUI Accelerator" } ,
	{ 0x102C, 0x00E0, "65550", "LCD/CRT controller" } ,
	{ 0x102C, 0x00E4, "65554", "Flat Panel/LCD CRT GUI Accelerator" } ,
	{ 0x102C, 0x00E5, "65555", "VGA GUI Accelerator" } ,
	{ 0x102C, 0x00F0, "68554", "GUI Controller" } ,
	{ 0x102C, 0x00F4, "68554", "HiQVision Flat Panel/CRT GUI Controller" } ,
	{ 0x102C, 0x00F5, "68555", "GUI Controller" } ,
	{ 0x102C, 0x01E0, "65560", "PCI Flat Panel/CRT VGA Accelerator" } ,
	{ 0x102C, 0x0C30, "69030", "AGP/PCI Flat Panel/CRT VGA Accelerator" } ,
	{ 0x102D, 0x50DC, "3328", "Audio" } ,
	{ 0x102F, 0x0009, "r4x00", "CPU Bridge" } ,
	{ 0x102F, 0x0020, "Meteor 155", "ATM PCI Adapter" } ,
	{ 0x1031, 0x5601, "MiroVIDEO DC20", "I/O & JPEG" } ,
	{ 0x1031, 0x5607, "", "video in and out with motion jpeg compression and deco" } ,
	{ 0x1031, 0x5631, "Media 3D", "" } ,
	{ 0x1031, 0x6057, "MiroVIDEO DC10/DC30", "" } ,
	{ 0x1033, 0x0001, "upD98409", "PCI to 486 like bus Bridge" } ,
	{ 0x1033, 0x0002, "", "PCI to VL98 Bridge" } ,
	{ 0x1033, 0x0003, "", "ATM Controller" } ,
	{ 0x1033, 0x0004, "R4000", "PCI bus Bridge" } ,
	{ 0x1033, 0x0005, "", "PCI to 486 like peripheral bus Bridge" } ,
	{ 0x1033, 0x0006, "", "GUI Accelerator" } ,
	{ 0x1033, 0x0007, "", "PCI to ux-bus Bridge" } ,
	{ 0x1033, 0x0008, "", "GUI Accelerator (vga equivalent)" } ,
	{ 0x1033, 0x0009, "", "graphic Cntrlr for 98" } ,
	{ 0x1033, 0x001A, "Nile II", "" } ,
	{ 0x1033, 0x001D, "uPD98405", "NEASCOT-S20 ATM Integrated SAR Ctrlr" } ,
	{ 0x1033, 0x0021, "Vrc4373", "Nile I" } ,
	{ 0x1033, 0x0029, "PoverVR PCX1", "3D Accelerator" } ,
	{ 0x1033, 0x002A, "PoverVR", "3D Accelerator" } ,
	{ 0x1033, 0x0035, "uPD9210FGC-7EA", "USB Host Controller" } ,
	{ 0x1033, 0x0036, "uPD98409", "NEASCOT-S40C ATM Light SAR Controller" } ,
	{ 0x1033, 0x003E, "uPD66369", "NAPCCARD CardBus Controller" } ,
	{ 0x1033, 0x0046, "PoverVR PCX2", "3D Accelerator" } ,
	{ 0x1033, 0x005A, "Vrc5074", "Nile 4" } ,
	{ 0x1033, 0x0063, "uPD72862", "Firewarden IEEE1394 OHCI Host Controller" } ,
	{ 0x1033, 0x0067, "PowerVR Neon 250", "PowerVR series II graphics processor" } ,
	{ 0x1033, 0x0074, "", "56k Voice Modem" } ,
	{ 0x1033, 0x009B, "Vrc5476", "" } ,
	{ 0x1033, 0x00BE, "VR4122", "64-bit CPU with Northbridge" } ,
	{ 0x1033, 0x00CD, "uPD72870", "IEEE1394 1-Chip OHCI Host Controller" } ,
	{ 0x1033, 0x00CE, "uPD72871/2", "IEEE1394 1-Chip OHCI Host Controller" } ,
	{ 0x1033, 0x00E0, "uPD720100A", "USB 2.0 Host Controller" } ,
	{ 0x1033, 0x00E7, "uPD72873", "IEEE1394 OHCI 1.1 2-port PHY-Link Ctrlr" } ,
	{ 0x1033, 0x00F2, "uPD72874", "IEEE1394 OHCI 1.1 3-port PHY-Link Ctrlr" } ,
	{ 0x1036, 0x0000, "TMC-18C30", "Fast SCSI" } ,
	{ 0x1039, 0x0001, "SiS 530", "Virtual PCI-to-PCI bridge (AGP)" } ,
	{ 0x1039, 0x0002, "SiS 6201/02", "PCI True-Color Graphics Accelerator" } ,
	{ 0x1039, 0x0005, "", "Pentium chipset" } ,
	{ 0x1039, 0x0006, "SiS 85C501", "PCI/ISA Cache Memory Controller (PCMC)" } ,
	{ 0x1039, 0x0008, "SiS 85C503", "PCI System I/O (PSIO)" } ,
	{ 0x1039, 0x0009, "SiS 5595", "Power Management Unit (PMU)" } ,
	{ 0x1039, 0x0018, "SiS950", "PCI to ISA Bridge (LPC Bridge)" } ,
	{ 0x1039, 0x0200, "SiS5597/98", "Onboard Graphics Controller" } ,
	{ 0x1039, 0x0204, "SiS 6215", "PCI Graphics & Video Accelerator" } ,
	{ 0x1039, 0x0205, "SiS 6205", "PCI Graphics & Video Accelerator" } ,
	{ 0x1039, 0x0300, "SiS300/305/630", "GUI Accelerator+3D" } ,
	{ 0x1039, 0x0305, "SiS305", "2D/3D/Video/DVD Accelerator" } ,
	{ 0x1039, 0x0315, "SiS 315", "" } ,
	{ 0x1039, 0x0325, "SiS325", "2D/3D Accelerator" } ,
	{ 0x1039, 0x0330, "SiS330", "Xabre 2D/3D Accelerator" } ,
	{ 0x1039, 0x0406, "85C501", "PCI/ISA Cache Memory Controller (PCMC)" } ,
	{ 0x1039, 0x0496, "85C496", "CPU to PCI & PCI to ISA Bridge" } ,
	{ 0x1039, 0x0530, "SiS530", "Host-to-PCI bridge" } ,
	{ 0x1039, 0x0540, "SiS540", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0550, "SiS550/1/2", "North Bridge" } ,
	{ 0x1039, 0x0596, "SiS596", "Pentium PCI chipset with IDE" } ,
	{ 0x1039, 0x0597, "SiS5513", "EIDE Controller (step C)" } ,
	{ 0x1039, 0x0601, "SiS83C601", "PCI EIDE Controller" } ,
	{ 0x1039, 0x0620, "SiS620", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0630, "SiS630", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0635, "SiS 635", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0640, "SiS 640", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0645, "SiS 645", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0646, "SiS645DX", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0648, "SiS648", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0649, "", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0650, "SiS 650", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0651, "SiS651", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0655, "SiS655", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0660, "", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0730, "SiS 730", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0735, "SiS 735", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0740, "SiS 740", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0745, "SiS745", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0746, "SiS746", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0755, "", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x0900, "SiS900", "Fast Ethernet/Home Networking Ctrlr" } ,
	{ 0x1039, 0x0963, "SiS963", "PCI to ISA Bridge" } ,
	{ 0x1039, 0x3602, "SiS83C602", "IDE Controller" } ,
	{ 0x1039, 0x5107, "SiS5107", "Hot Docking Controller" } ,
	{ 0x1039, 0x5300, "SiS540", "AGP" } ,
	{ 0x1039, 0x5315, "SiS550/1/2", "GUI Accelerator" } ,
	{ 0x1039, 0x5401, "SiS5401", "486 PCI Chipset" } ,
	{ 0x1039, 0x5511, "SiS5511/5512", "PCI/ISA System Memory Controller" } ,
	{ 0x1039, 0x5513, "SiS5513", "PCI IDE Controller" } ,
	{ 0x1039, 0x5517, "SiS5517", "CPU to PCI Bridge" } ,
	{ 0x1039, 0x5571, "SiS5571", "Memory/PCI bridge" } ,
	{ 0x1039, 0x5581, "SiS 5581", "p5 chipset" } ,
	{ 0x1039, 0x5582, "SiS5582", "PCI to ISA Bridge" } ,
	{ 0x1039, 0x5591, "SiS 5591/5592", "PCI AGP & CPU Memory Controller" } ,
	{ 0x1039, 0x5596, "SiS5596", "PCI, Memory & VGA Controller" } ,
	{ 0x1039, 0x5597, "SiS5597", "Host to PCI bridge" } ,
	{ 0x1039, 0x5600, "SiS600", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x5630, "SiS630", "Host-to-PCI Bridge" } ,
	{ 0x1039, 0x6204, "SiS6204", "video decoder/mpeg interface" } ,
	{ 0x1039, 0x6205, "SiS6205", "PCI VGA Controller" } ,
	{ 0x1039, 0x6225, "SiS 6225", "PCI Graphics & Video Accelerator" } ,
	{ 0x1039, 0x6236, "SiS6236", "Graphics" } ,
	{ 0x1039, 0x6300, "SiS630/730", "GUI Accelerator+3D" } ,
	{ 0x1039, 0x6306, "SiS530/620", "Integrated 3D VGA Controller" } ,
	{ 0x1039, 0x6325, "SiS650/651/740", "GUI 2D/3D Accelerator" } ,
	{ 0x1039, 0x6326, "SiS 86C326", "AGP/PCI Graphics & Video Accelerator" } ,
	{ 0x1039, 0x7001, "SiS5571", "USB Host Controller" } ,
	{ 0x1039, 0x7002, "SiS7002", "USB 2.0 Enhanced Host Controller" } ,
	{ 0x1039, 0x7005, "SiS551/2", "Memory Stick Controller" } ,
	{ 0x1039, 0x7007, "", "OHCI Compliant FireWire Controller" } ,
	{ 0x1039, 0x7012, "SiS7012", "PCI Audio Accelerator" } ,
	{ 0x1039, 0x7013, "SiS7013", "56k Modem" } ,
	{ 0x1039, 0x7015, "SiS550/1/2", "Software Audio" } ,
	{ 0x1039, 0x7016, "SiS7016", "10/100 Ethernet Adapter" } ,
	{ 0x1039, 0x7018, "SiS7018", "PCI Audio Accelerator" } ,
	{ 0x1039, 0x7019, "SiS550/1/2", "Hardware Audio" } ,
	{ 0x1039, 0x7300, "SiS730", "GUI Accelerator+3D" } ,
	{ 0x103C, 0x1005, "A4977A", "Visialize EG" } ,
	{ 0x103C, 0x1008, "Donner GFX", "" } ,
	{ 0x103C, 0x100A, "hpVisualizeFX", "Hewlett-Packard VisualizeFX Series Video" } ,
	{ 0x103C, 0x1028, "", "Tachyon TL Fibre Channel Adapter" } ,
	{ 0x103C, 0x1029, "HPFC-5200B", "Tachyon XL2 Fibre Channel Adapter" } ,
	{ 0x103C, 0x102A, "Tach TS", "Tachyon TS Fibre Channel Host Adapter" } ,
	{ 0x103C, 0x1030, "J2585A", "DeskDirect 10/100VG LAN Adapter" } ,
	{ 0x103C, 0x1031, "J2585B", "DeskDirect 10/100 NIC" } ,
	{ 0x103C, 0x1040, "J2973A", "DeskDirect 10BaseT NIC" } ,
	{ 0x103C, 0x1041, "J2585B", "DeskDirect 10/100VG NIC" } ,
	{ 0x103C, 0x1042, "J2970A", "DeskDirect 10BaseT/2 NIC" } ,
	{ 0x103C, 0x1048, "SAS", "" } ,
	{ 0x103C, 0x1049, "DIVA1", "" } ,
	{ 0x103C, 0x104A, "DIVA2", "" } ,
	{ 0x103C, 0x104B, "SP2", "" } ,
	{ 0x103C, 0x104D, "J3242A", "EL-10 Ethernet Adapter" } ,
	{ 0x103C, 0x1064, "79C970", "PCnet Ethernet Controller" } ,
	{ 0x103C, 0x10C1, "", "NetServer Smart IRQ Router" } ,
	{ 0x103C, 0x10ED, "TopTools", "Remote Control" } ,
	{ 0x103C, 0x1200, "82557B", "10/100 NIC" } ,
	{ 0x103C, 0x1219, "", "NetServer PCI Hot-Plug Controller" } ,
	{ 0x103C, 0x121A, "", "NetServer SMIC Controller" } ,
	{ 0x103C, 0x121B, "", "NetServer Legacy COM Port Decoder" } ,
	{ 0x103C, 0x121C, "", "NetServer PCI COM Port Decoder" } ,
	{ 0x103C, 0x1229, "zx1", "System Bus Adapter" } ,
	{ 0x103C, 0x122A, "zx1", "I/O Controller" } ,
	{ 0x103C, 0x122B, "zx1", "Local Bus Adapter" } ,
	{ 0x103C, 0x2910, "E2910A", "PCI Bus Exerciser" } ,
	{ 0x103C, 0x2920, "", "Fast Host Interface" } ,
	{ 0x103C, 0x2924, "E2924A", "PCI Host Interface Adapter" } ,
	{ 0x103C, 0x2925, "E2925A", "32 bit PCI Bus Exerciser and Analyzer" } ,
	{ 0x103C, 0x2926, "E2926A", "64 bit PCI Bus Exerciser and Analyzer" } ,
	{ 0x103C, 0x2927, "E2927A", "64 Bit, 66/50MHz PCI Analyzer & Exerciser" } ,
	{ 0x103c, 0x2928, "E2928A", "64Bit, 66MHz PCI Exerciser/Analyzer" } ,
	{ 0x103C, 0x2940, "E2940A", "64 bit, 66/50MHz CompactPCI Analyzer&Exerciser" } ,
	{ 0x1042, 0x1000, "FDC 37C665", "EIDE" } ,
	{ 0x1042, 0x1000, "RZ1000", "IDE Ctrlr" } ,
	{ 0x1042, 0x1001, "37C922", "" } ,
	{ 0x1042, 0x3000, "Samurai device 0", "CPU to PCI Bridge" } ,
	{ 0x1042, 0x3010, "Samurai device 1", "CPU to PCI Bridge" } ,
	{ 0x1042, 0x3020, "Samurai IDE device", "IDE Controller" } ,
	{ 0x1042, 0x3030, "MT82P664", "Samurai 64M2" } ,
	{ 0x1042, 0x3120, "Samurai-DDR", "CPU to PCI bridge" } ,
	{ 0x1042, 0x3130, "Samurai-DDR", "AGP controller" } ,
	{ 0x1043, 0x0675, "HFC-S PCI A", "Cologne Chip" } ,
	{ 0x1044, 0x1012, "Domino", "RAID Engine" } ,
	{ 0x1044, 0xA400, "2124A/9X", "SmartCache III/RAID SCSI Controller" } ,
	{ 0x1044, 0xA500, "", "PCI Bridge" } ,
	{ 0x1044, 0xA501, "", "I2O SmartRAID V Controller" } ,
	{ 0x1044, 0xA511, "Raptor", "SmartRAID Controller" } ,
	{ 0x1045, 0x0005, "", "" } ,
	{ 0x1045, 0xA0F8, "82C750", "PCI USB Controller" } ,
	{ 0x1045, 0xC101, "82C264", "GUI Accelerator" } ,
	{ 0x1045, 0xC178, "82C178", "LCD GUI Accelerator" } ,
	{ 0x1045, 0xC556, "82C556", "Viper" } ,
	{ 0x1045, 0xC557, "82C557", "CPU Bridge (Viper)" } ,
	{ 0x1045, 0xC558, "82C558", "ISA Bridge w/PnP" } ,
	{ 0x1045, 0xC567, "82C750", "Vendetta chipset: host bridge" } ,
	{ 0x1045, 0xC568, "82C750", "Vendetta chipset: ISA bridge" } ,
	{ 0x1045, 0xC569, "82C579", "Pentium to PCI Bridge" } ,
	{ 0x1045, 0xC621, "82C621", "PCI IDE Controller (PIC)" } ,
	{ 0x1045, 0xC700, "82C700", "FireStar chipset, PCI-ISA bridge???" } ,
	{ 0x1045, 0xC701, "82C700", "FireStar mobile chipset: host bridge" } ,
	{ 0x1045, 0xC814, "82C814", "FireBridge II Docking Station Controller" } ,
	{ 0x1045, 0xC822, "82C822", "CPU to PCI & PCI to ISA PnP bridge" } ,
	{ 0x1045, 0xC824, "82C824", "FireFox 32-Bit PC Card Controller" } ,
	{ 0x1045, 0xC825, "82C825 function 0", "PCI-to-ISA Bridge" } ,
	{ 0x1045, 0xC832, "82C832", "CPU-to-PCI and PCI-to-ISA Bridge" } ,
	{ 0x1045, 0xC861, "82C861/2/3", "FireLink PCI-to-USB Bridge" } ,
	{ 0x1045, 0xC881, "82C881", "FireLink 1394 OHCI Link Controller" } ,
	{ 0x1045, 0xC895, "82C895", "" } ,
	{ 0x1045, 0xC935, "82C935", "MachOne integrated PCI audio processor" } ,
	{ 0x1045, 0xD568, "82C825", "PCI bus master IDE controller" } ,
	{ 0x1045, 0xD768, "82C750", "Ultra DMA IDE controller" } ,
	{ 0x1048, 0x0C60, "Elsa Gladiac MX", "NVidia Geforce 2 MX" } ,
	{ 0x1048, 0x1000, "Quick Step 1000", "" } ,
	{ 0x1048, 0x3000, "QuickStep 3000", "" } ,
	{ 0x1048, 0x8901, "", "ELSA GLoria XL" } ,
	{ 0x104A, 0x0008, "STG 2000X", "" } ,
	{ 0x104A, 0x0009, "STG 1764X", "" } ,
	{ 0x104A, 0x0010, "STG4000", "PowerVR KYRO series 3 graphics processor" } ,
	{ 0x104A, 0x0209, "STPC Consmr/Indstrl", "North/South Bridges" } ,
	{ 0x104A, 0x020A, "STPC Atlas/Elite", "North Bridge" } ,
	{ 0x104A, 0x0210, "STPC Atlas", "ISA Bridge" } ,
	{ 0x104A, 0x021A, "STPC Consmr-S/Elite", "ISA Bridge" } ,
	{ 0x104A, 0x021B, "STPC Consumer-II", "ISA Bridge" } ,
	{ 0x104A, 0x0228, "STPC Atlas", "IDE Controller" } ,
	{ 0x104A, 0x0230, "STPC Atlas", "USB Controller" } ,
	{ 0x104A, 0x0500, "ST70137", "ADSL" } ,
	{ 0x104A, 0x0981, "", "10/100 Ethernet Adapter" } ,
	{ 0x104A, 0x1746, "STG 1746X", "" } ,
	{ 0x104A, 0x2774, "STE10/100A", "PCI 10/100 Ethernet Controller" } ,
	{ 0x104A, 0x3520, "", "MPEG-II Video Decoder" } ,
	{ 0x104B, 0x0140, "BT-946C", "Multimaster NC (SCSI-2)" } ,
	{ 0x104B, 0x1040, "BA80C30", "Multimaster" } ,
	{ 0x104B, 0x8130, "BT-930/32/50/52", "Flashpoint LT/DL/LW/DW Ultra (Wide) SCSI" } ,
	{ 0x104C, 0x0500, "TNETE100A/110A/211", "ThunderLAN 100 Mbit LAN Controller" } ,
	{ 0x104C, 0x0508, "TI380PCI", "PCI interface for TI380 compressors" } ,
	{ 0x104C, 0x1000, "TI PCI Eagle i/f AS", "" } ,
	{ 0x104C, 0x3D04, "TVP4010", "Permedia" } ,
	{ 0x104C, 0x3D07, "TVP4020", "AGP Permedia 2" } ,
	{ 0x104C, 0x8000, "TSB12LV21", "LYNX IEEE1394 FireWire Host Controller" } ,
	{ 0x104C, 0x8009, "TSB12LV22", "OHCI-Lynx PCI IEEE 1394 Host Controller" } ,
	{ 0x104C, 0x8011, "PCI4450", "OHCI-Lynx IEEE 1394 Controller" } ,
	{ 0x104C, 0x8017, "PCI4410", "OHCI-Lynx IEEE 1394 Controller" } ,
	{ 0x104C, 0x8019, "TSB12LV23", "OHCI-Lynx PCI IEEE 1394 Host Controller" } ,
	{ 0x104C, 0x8020, "TSB12LV26", "OHCI-Lynx PCI IEEE 1394 Host Controller" } ,
	{ 0x104C, 0x8021, "TSB43AA22", "1394a-2000 OHCI PHY/Link Layer Ctrlr" } ,
	{ 0x104C, 0x8023, "TSB43AB22/A", "IEEE1394a-2000 OHCI PHY/Link-Layer Ctrlr" } ,
	{ 0x104C, 0x8024, "TSB43AB23", "IEEE 1394a-2000 OHCI PHY/Link Layer Ctrl" } ,
	{ 0x104C, 0x8026, "TSB43AB21", "1394a-2000 OHCI PHY/Link Layer Ctrlr" } ,
	{ 0x104C, 0x8027, "PCI4451", "OHCI-Lynx IEEE 1394 Controller" } ,
	{ 0x104C, 0x8400, "", "802.11b+ 22Mbps Wireless Adapter" } ,
	{ 0x104C, 0xA001, "TDC1570", "64-bit PCI ATM SAR" } ,
	{ 0x104C, 0xA100, "TDC1561", "32-bit PCI ATM SAR" } ,
	{ 0x104C, 0xA102, "TNETA1575", "HyperSAR Plus w/PCI host & UTOPIA i/f" } ,
	{ 0x104C, 0xAC10, "PCI1050", "PC Card Controller" } ,
	{ 0x104C, 0xAC11, "PCI1030/1053", "PC Card Controller" } ,
	{ 0x104C, 0xAC12, "PCI1130", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC13, "PCI1031", "PCI-TO-PC CARD16 CONTROLLER UNIT" } ,
	{ 0x104c, 0xAC15, "PCI1131", "Dual Socket PCI CardBus Controller" } ,
	{ 0x104C, 0xAC16, "PCI1250", "PC Card CardBus Controller" } ,
	{ 0x104C, 0xAC17, "PCI1220", "CardBus Controller" } ,
	{ 0x104C, 0xAC18, "PCI1260", "PC card CardBus Controller" } ,
	{ 0x104c, 0xAC19, "PCI1221", "PC Card Controller" } ,
	{ 0x104C, 0xAC1A, "PCI1210", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC1B, "PCI1450", "PC card CardBus Controller" } ,
	{ 0x104c, 0xAC1C, "PCI1225 GHK/PDV", "PC card CardBus Controller" } ,
	{ 0x104c, 0xAC1D, "PCI1251A", "PC Card Controller" } ,
	{ 0x104c, 0xAC1E, "PCI1211", "High Performance PC Card Controller" } ,
	{ 0x104C, 0xAC1F, "PCI1251B", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC20, "PCI2030", "PCI to PCI Bridge" } ,
	{ 0x104C, 0xAC21, "PCI2031", "PCI to PCI Bridge" } ,
	{ 0x104C, 0xAC22, "PCI2032", "PCI Docking Bridge" } ,
	{ 0x104C, 0xAC23, "PCI2250", "PCI-to-PCI Bridge" } ,
	{ 0x104C, 0xAC28, "PCI2050/2050I", "PCI-to-PCI Bridge" } ,
	{ 0x104C, 0xAC30, "PCI1260", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC40, "PCI4450", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC41, "PCI4410", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC42, "PCI4451", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC43, "PCI4550", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC44, "PCI4510", "PC Card Controller" } ,
	{ 0x104C, 0xAC46, "PCI4520", "PCCard CardBus Controller" } ,
	{ 0x104C, 0xAC50, "PCI1410", "PC card cardBus Controller" } ,
	{ 0x104c, 0xAC51, "PCI1420", "PC Card Controller" } ,
	{ 0x104C, 0xAC52, "PCI1451", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC53, "PCI1421", "PC card CardBus Controller" } ,
	{ 0x104C, 0xAC54, "PCI1620", "PCCard CardBus Controller w/UltraMedia" } ,
	{ 0x104C, 0xAC55, "PCI1520", "PCCard CardBus Controller" } ,
	{ 0x104C, 0xAC56, "PCI1510", "PCCard CardBus Controller" } ,
	{ 0x104C, 0xAC57, "PCI1530", "PCCard CardBus Controller" } ,
	{ 0x104C, 0xAC58, "PCI1515", "PCCard CardBus Controller" } ,
	{ 0x104C, 0xAC59, "PCI1621", "PCCard CardBus Controller w/UltraMedia" } ,
	{ 0x104C, 0xAC5A, "PCI1610", "PCCard CardBus Controller w/UltraMedia" } ,
	{ 0x104c, 0xAC60, "PCI2040", "PCI-DSP Bridge Controller" } ,
	{ 0x104C, 0xFE00, "", "FireWire Host Controller" } ,
	{ 0x104C, 0xFE03, "12C01A", "FireWire Host Controller" } ,
	{ 0x104D, 0x8009, "CXD1947", "i.LINK FireWire PCI Host Controller" } ,
	{ 0x104D, 0x8039, "CXD3222", "OHCI i.LINK (IEEE 1394) PCI Host Ctrlr" } ,
	{ 0x104D, 0x8056, "Rockwell HCF 56K", "Modem" } ,
	{ 0x104D, 0x808A, "", "Memory Stick Controller" } ,
	{ 0x104E, 0x0017, "OTI-64017", "" } ,
	{ 0x104E, 0x0107, "OTI107", "Spitfire VGA Accelerator" } ,
	{ 0x104E, 0x0109, "", "Video Adapter" } ,
	{ 0x104E, 0x0217, "OTI-64217", "" } ,
	{ 0x104E, 0x0317, "OTI-64317", "" } ,
	{ 0x104E, 0x0611, "OTI-610", "" } ,
	{ 0x104F, 0x104F, "iatca8392", "Multi I/O" } ,
	{ 0x1050, 0x0000, "004005-34c8c8", "Ethernet Controller (NE2000 compatible)" } ,
	{ 0x1050, 0x0001, "W83769F", "Ethernet Adapter" } ,
	{ 0x1050, 0x0105, "W82C105", "Ethernet Adapter" } ,
	{ 0x1050, 0x0628, "W83628F/629D", "PCI to ISA Bridge Set" } ,
	{ 0x1050, 0x0840, "W89C840F", "100/10Mbps Ethernet Controller" } ,
	{ 0x1050, 0x0940, "w89c940f", "winbond pci ethernet" } ,
	{ 0x1050, 0x5A5A, "W89C940F", "ELANC-PCI Twisted-pair Ether-LAN Ctrlr" } ,
	{ 0x1050, 0x6692, "W6692/92A/92CF", "PCI BusISDN S/T-Controller" } ,
	{ 0x1050, 0x9922, "W9922PF", "ISDN Controller" } ,
	{ 0x1050, 0x9960, "W9960CF", "Video Codec" } ,
	{ 0x1050, 0x9961, "W9961CF", "H.263/H.261 Video Codec" } ,
	{ 0x1050, 0x9970, "W9970CF", "VGA controller" } ,
	{ 0x1050, 0x9971, "W9971CF", "Video Graphics Controller With TV Encode" } ,
	{ 0x1051, 0x0100, "Motorola MC145575", "" } ,
	{ 0x1054, 0x0001, "", "PCI Bridge" } ,
	{ 0x1054, 0x0002, "", "PCI bus Cntrlr" } ,
	{ 0x1054, 0x3505, "SH7751", "SuperH (SH) 32-Bit RISC MCU/MPU Series" } ,
	{ 0x1055, 0x0810, "", "EFAR 486 host Bridge" } ,
	{ 0x1055, 0x0922, "", "Pentium/p54c host Bridge" } ,
	{ 0x1055, 0x0926, "", "ISA Bridge" } ,
	{ 0x1055, 0x9130, "SLC90E66", "Ultra ATA/66 IDE Controller" } ,
	{ 0x1055, 0x9460, "SLC90E66", "Victory66 PCI to ISA Bridge" } ,
	{ 0x1055, 0x9461, "SLC90E66", "Victory66 UDMA EIDE Controller" } ,
	{ 0x1055, 0x9462, "SLC90E66", "Victory66 USB Host Controller" } ,
	{ 0x1055, 0x9463, "SLC90E66", "Victory66 Power Management Controller" } ,
	{ 0x1057, 0x0001, "MPC105", "PCI Bridge / Memory Controller (PCIB/MC)" } ,
	{ 0x1057, 0x0002, "MPC106", "PCI Bridge/Memory Controller (PCIB/MC)" } ,
	{ 0x1057, 0x0003, "MPC8240", "Integrated Processor" } ,
	{ 0x1057, 0x0004, "MPC107", "PCI Bridge/Memory Controller for PPC" } ,
	{ 0x1057, 0x0006, "MPC8245", "Integrated Processor" } ,
	{ 0x1057, 0x0100, "MC145575", "HCF-PCI" } ,
	{ 0x1057, 0x0431, "KTI829c", "100VG Ethernet Controller" } ,
	{ 0x1057, 0x1801, "DSP56301", "24-bit Digital Signal Processor" } ,
	{ 0x1057, 0x1802, "DSP56305", "24-Bit Digital Signal Processor" } ,
	{ 0x1057, 0x18C0, "MPC8265A/66", "PowerQUICC II PCI Bridge" } ,
	{ 0x1057, 0x3421, "56IVMR/Phoenix 56ISM", "Modem" } ,
	{ 0x1057, 0x4801, "Raven", "PowerPC Chipset" } ,
	{ 0x1057, 0x4802, "Falcon", "" } ,
	{ 0x1057, 0x4803, "Hawk", "" } ,
	{ 0x1057, 0x4806, "CPX8216", "" } ,
	{ 0x1057, 0x4809, "CPX8216T", "HotSwap Controller" } ,
	{ 0x1057, 0x5600, "SM56", "PCI Speakerphone/Data,Fax  Modem" } ,
	{ 0x1057, 0x5602, "SM56", "PCI Modem" } ,
	{ 0x1057, 0x5608, "SM56", "PCI Fax Voice Modem" } ,
	{ 0x1057, 0x6400, "MPC190", "Security Co-Processor" } ,
	{ 0x105a, 0x0D30, "PDC20265", "Ultra100 EIDE controller (on M/B)" } ,
	{ 0x105A, 0x0D38, "PDC20263", "FastTrak66 EIDE Controller" } ,
	{ 0x105A, 0x1275, "PDC20275", "FastTrack TX EIDE Controller" } ,
	{ 0x105A, 0x3318, "PDC20318?", "FastTrak SATA150 TX4 Controller" } ,
	{ 0x105A, 0x3319, "PDC20319?", "FastTrak SATA150 TX4 Controller" } ,
	{ 0x105A, 0x3371, "PDC20371?", "FastTrak SATA150 TX2plus Controller" } ,
	{ 0x105A, 0x3375, "PDC20375?", "FastTrak SATA150 TX2plus Controller" } ,
	{ 0x105A, 0x3376, "PDC20376", "FastTrak 376 Controller" } ,
	{ 0x105A, 0x4D30, "PDC20267", "Ultra100 EIDE Controller" } ,
	{ 0x105A, 0x4D33, "PDC20246", "FastTrak Ultra ATA RAID controller" } ,
	{ 0x105A, 0x4D38, "PDC20262", "Ultra66 EIDE Controller" } ,
	{ 0x105A, 0x4D68, "PDC20268", "Ultra100 EIDE Controller" } ,
	{ 0x105a, 0x4D69, "PDC20269", "Ultra133TX2 EIDE Controller" } ,
	{ 0x105A, 0x5275, "PDC20276", "MBUltra133 EIDE Controller" } ,
	{ 0x105A, 0x5300, "DC5300", "EIDE Controller" } ,
	{ 0x105A, 0x6268, "PDC20268R", "FastTrak100 TX2/TX4/LP EIDE controller" } ,
	{ 0x105A, 0x6269, "PDC20271", "FastTrak TX2000 EIDE controller" } ,
	{ 0x105A, 0x6629, "", "FastTrak TX4000 Controller" } ,
	{ 0x105a, 0x7275, "PDC20277", "SBFastTrak SX6000 EIDE Controller" } ,
	{ 0x105D, 0x2309, "Imagine 128", "GUI Accelerator" } ,
	{ 0x105D, 0x2339, "I128s2", "Imagine 128 Series 2" } ,
	{ 0x105D, 0x493D, "T2R", "Revolution 3D" } ,
	{ 0x105D, 0x5348, "Revolution IV", "Revolution IV" } ,
	{ 0x1060, 0x0001, "UM82C881", "486 Chipset" } ,
	{ 0x1060, 0x0002, "UM82C886", "ISA Bridge" } ,
	{ 0x1060, 0x0101, "UM8673F", "EIDE Controller" } ,
	{ 0x1060, 0x0881, "UM8881", "" } ,
	{ 0x1060, 0x0881, "UM8881", "HB4 486 PCI Chipset" } ,
	{ 0x1060, 0x0886, "UM8886F", "ISA Bridge" } ,
	{ 0x1060, 0x0891, "UM82C891", "Pentium CPU to PCI bridge" } ,
	{ 0x1060, 0x1001, "UM886A", "IDE Cntrlr (dual function)" } ,
	{ 0x1060, 0x673A, "UM8886BF", "EIDE Controller" } ,
	{ 0x1060, 0x673B, "", "EIDE Master/DMA" } ,
	{ 0x1060, 0x8710, "UM8710", "VGA Cntrlr" } ,
	{ 0x1060, 0x8821, "", "CPU/PCI Bridge" } ,
	{ 0x1060, 0x8822, "", "PCI/ISA Bridge" } ,
	{ 0x1060, 0x8851, "", "Pentium CPU/PCI Bridge" } ,
	{ 0x1060, 0x8852, "", "Pentium CPU/ISA Bridge" } ,
	{ 0x1060, 0x886A, "UM8886A", "ISA Bridge with EIDE" } ,
	{ 0x1060, 0x8881, "UM8881F", "HB4 486 PCI Chipset" } ,
	{ 0x1060, 0x8886, "UM8886", "ISA Bridge (w/o IDE support)" } ,
	{ 0x1060, 0x888A, "UM8886A", "" } ,
	{ 0x1060, 0x8891, "UM8891", "586 Chipset" } ,
	{ 0x1060, 0x9017, "UM9017F", "Ethernet" } ,
	{ 0x1060, 0x9018, "UM9018", "Ethernet" } ,
	{ 0x1060, 0x9026, "UM9026", "Fast Ethernet" } ,
	{ 0x1060, 0xE881, "UM8881", "486 Chipset" } ,
	{ 0x1060, 0xE886, "UM8886N", "ISA Bridge w/EIDE" } ,
	{ 0x1060, 0xE88A, "UM8886N", "PCI / ISA Bridge" } ,
	{ 0x1060, 0xE891, "UM8891N", "" } ,
	{ 0x1061, 0x0001, "AGX013/016", "GUI Accelerator" } ,
	{ 0x1061, 0x0002, "IIT3204/3501", "MPEG Decoder" } ,
	{ 0x1065, 0x8139, "", "Realtek 8139C Network Card" } ,
	{ 0x1066, 0x0000, "PT80C826", "VL Bridge" } ,
	{ 0x1066, 0x0001, "PT86C521", "Vesuvius V1-LS System Controller" } ,
	{ 0x1066, 0x0002, "PT86C523", "Vesuvius V3-LS ISA Bridge" } ,
	{ 0x1066, 0x0003, "PT80C524", "Nile PCI to PCI Bridge" } ,
	{ 0x1066, 0x0004, "PT80C525", "Nile-II PCI to PCI Bridge" } ,
	{ 0x1066, 0x0005, "PC87550", "System Controller" } ,
	{ 0x1066, 0x8002, "PT86C523", "ISA Bridge" } ,
	{ 0x1067, 0x1002, "VG500", "VolumePro Volume Rendering Accelerator" } ,
	{ 0x1069, 0x0001, "DAC960P", "DAC960P 3 ch SCSI RAID Controller" } ,
	{ 0x1069, 0x0002, "DAC960PD", "DAC960PD 3 ch SCSI RAID Controller" } ,
	{ 0x1069, 0x0010, "DAC960PJ", "DAC960PJ 3 ch SCSI RAID Controller" } ,
	{ 0x1069, 0x0050, "i960", "AcceleRAID 170" } ,
	{ 0x1069, 0xBA55, "1100", "eXtremeRAID support device" } ,
	{ 0x1069, 0xBA56, "", "eXtremeRAID Disk Array" } ,
	{ 0x106B, 0x0001, "Bandit", "PowerPC Host-PCI Bridge" } ,
	{ 0x106B, 0x0002, "Grand Central", "I/O Controller" } ,
	{ 0x106B, 0x0003, "Control Video", "" } ,
	{ 0x106B, 0x0004, "PlanB", "Video-in" } ,
	{ 0x106B, 0x0007, "OHare", "I/O Controller" } ,
	{ 0x106b, 0x0008, "Bandit", "Host-PCI bridge" } ,
	{ 0x106B, 0x000E, "Hydra", "Mac I/O Controller" } ,
	{ 0x106B, 0x0010, "Heathrow", "Mac I/O Controller" } ,
	{ 0x106B, 0x0017, "Paddington", "Mac I/O Controller" } ,
	{ 0x106B, 0x0018, "UniNorth", "FireWire Controller" } ,
	{ 0x106b, 0x0019, "KeyLargo", "USB controller" } ,
	{ 0x106b, 0x001E, "UniNorth", "Host-PCI bridge" } ,
	{ 0x106B, 0x001F, "UniNorth", "Host-PCI bridge" } ,
	{ 0x106B, 0x0020, "UniNorth", "AGP interface" } ,
	{ 0x106b, 0x0021, "UniNorth GMAC", "Ethernet controller" } ,
	{ 0x106b, 0x0022, "KeyLargo", "Mac I/O controller" } ,
	{ 0x106b, 0x0024, "GMAC", "Ethernet controller" } ,
	{ 0x106b, 0x0025, "Pangea", "Mac I/O controller" } ,
	{ 0x106B, 0x0026, "Pangea", "USB Interface" } ,
	{ 0x106B, 0x0027, "Pangea", "AGP interface" } ,
	{ 0x106b, 0x0028, "Pangea", "Host-PCI bridge" } ,
	{ 0x106b, 0x0029, "Pangea", "Host-PCI bridge" } ,
	{ 0x106B, 0x002D, "UniNorth 1.5", "AGP Bridge" } ,
	{ 0x106B, 0x002E, "UniNorth 1.5", "PCI Bridge" } ,
	{ 0x106B, 0x002F, "UniNorth 1.5", "Internal PCI" } ,
	{ 0x106B, 0x0030, "UniNorth/Pangea", "FireWire Controller" } ,
	{ 0x106C, 0x8801, "", "Dual Pentium ISA/PCI Motherboard" } ,
	{ 0x106C, 0x8802, "", "PowerPC ISA/PCI Motherboard" } ,
	{ 0x106C, 0x8803, "", "Dual Window Graphics Accelerator" } ,
	{ 0x106C, 0x8804, "", "PCI LAN Controller" } ,
	{ 0x106C, 0x8805, "", "100-BaseT LAN Controller" } ,
	{ 0x1073, 0x0001, "", "3D graphics Cntrlr" } ,
	{ 0x1073, 0x0002, "YGV615", "RPA3 3D-Graphics Controller" } ,
	{ 0x1073, 0x0003, "YMF740", "" } ,
	{ 0x1073, 0x0004, "YMF724", "PCI Audio Controller" } ,
	{ 0x1073, 0x0005, "DS1", "DS1 Audio" } ,
	{ 0x1073, 0x0006, "DS1", "DS1 Audio" } ,
	{ 0x1073, 0x0008, "DS1", "DS1 Audio" } ,
	{ 0x1073, 0x000A, "YMF740", "DS-1L PCI Audio Controller" } ,
	{ 0x1073, 0x000C, "YMF740C", "DS-1L PCI audio controller" } ,
	{ 0x1073, 0x000D, "YMF724F", "Yamaha Onboard Sound System" } ,
	{ 0x1073, 0x0010, "YMF744B-V", "DS-1 PCI audio controller" } ,
	{ 0x1073, 0x0012, "YMF754B", "DS-1E PCI Audio Controller" } ,
	{ 0x1073, 0x0020, "", "DS-1 Audio" } ,
	{ 0x1073, 0x1000, "SW1000XG", "Sound system" } ,
	{ 0x1073, 0x2000, "DS2416", "Digital Mixing Card" } ,
	{ 0x1074, 0x4E78, "82C500/1", "Nx586 Chipset" } ,
	{ 0x1077, 0x1016, "ISP10160", "Single Channel Ultra3 SCSI Processor" } ,
	{ 0x1077, 0x1020, "ISP1040B/1020A", "Fast-wide SCSI - Sparc PCI" } ,
	{ 0x1077, 0x1022, "ISP1022A", "Fast-wide SCSI" } ,
	{ 0x1077, 0x1080, "ISP1080", "SCSI Host Adapter" } ,
	{ 0x1077, 0x1216, "ISP12160", "Dual Channel Ultra3 SCSI Processor" } ,
	{ 0x1077, 0x1240, "ISP1240", "SCSI Host Adapter" } ,
	{ 0x1077, 0x1280, "ISP1280", "SCSI Host Adapter" } ,
	{ 0x1077, 0x2020, "ISP2020A", "Fast!SCSI Basic Adapter" } ,
	{ 0x1077, 0x2100, "QLA2100", "64-bit Fibre Channel Adapter" } ,
	{ 0x1077, 0x2200, "QLA2200", "PCI Fibre Channel Adapter" } ,
	{ 0x1077, 0x2300, "ISP 2300", "64-bit PCI FC-AL Adapter" } ,
	{ 0x1077, 0x2312, "ISP 2312", "Fibre Channel Adapter" } ,
	{ 0x1078, 0x0000, "Cx5520", "ISA Bridge" } ,
	{ 0x1078, 0x0001, "MediaGXm MMX", "Cyrix Integrated CPU" } ,
	{ 0x1078, 0x0002, "Cx5520", "ISA Bridge" } ,
	{ 0x1078, 0x0100, "Cx5530", "ISA bridge" } ,
	{ 0x1078, 0x0101, "Cx5530", "SMI status and ACPI timer" } ,
	{ 0x1078, 0x0102, "Cx5530", "IDE Controller" } ,
	{ 0x1078, 0x0103, "Cx5530", "XpressAUDIO" } ,
	{ 0x1078, 0x0104, "Cx5530", "Video Controller" } ,
	{ 0x1078, 0x0400, "ZFMicro", "CPU to PCI Bridge" } ,
	{ 0x1078, 0x0401, "ZFMicro", "Power Management Controller" } ,
	{ 0x1078, 0x0402, "ZFMicro", "IDE Controller" } ,
	{ 0x1078, 0x0403, "ZFMicro", "Expansion Bus" } ,
	{ 0x107D, 0x0000, "P86C850", "Graphic GLU-Logic" } ,
	{ 0x107E, 0x0001, "FLIPPER", "FRED Local Bus I/F to PCI Peripheral" } ,
	{ 0x107E, 0x0002, "", "100 vg anylan Cntrlr" } ,
	{ 0x107E, 0x0004, "5526", "Fibre Channel Host Adapter" } ,
	{ 0x107E, 0x0005, "x526", "Fibre Channel Host Adapter" } ,
	{ 0x107E, 0x0008, "4575/5525/5575/6575", "(i)chipSAR+ 155 MBit ATM controller" } ,
	{ 0x107E, 0x9003, "5535-4P-BRI-ST", "" } ,
	{ 0x107E, 0x9007, "5535-4P-BRI-U", "" } ,
	{ 0x107E, 0x9008, "5535-1P-SR", "" } ,
	{ 0x107E, 0x900C, "5535-1P-SR-ST", "" } ,
	{ 0x107E, 0x900E, "5535-1P-SR-U", "" } ,
	{ 0x107E, 0x9011, "5535-1P-PRI", "" } ,
	{ 0x107E, 0x9013, "5535-2P-PRI", "" } ,
	{ 0x107E, 0x9023, "5535-4P-BRI-ST", "" } ,
	{ 0x107E, 0x9027, "5536-4P-BRI-U", "" } ,
	{ 0x107E, 0x9031, "5535-1P-PRI", "" } ,
	{ 0x107E, 0x9033, "5536-2P-PRI", "" } ,
	{ 0x107E, 0x9060, "6535", "CompactPCI T1/E1/J1Communications Ctrlr" } ,
	{ 0x107E, 0x9070, "4538", "PMC T1/E1/J1 Communications Controller" } ,
	{ 0x107E, 0x9080, "4532-002/005", "PMC ATM Over OC-3/STM-1 Comm Controller" } ,
	{ 0x107E, 0x9081, "4532-001/004", "PMC ATM Over OC-3/STM-1 Comm Controller" } ,
	{ 0x107E, 0x9082, "4532-000/003", "PMC ATM Over OC-3/STM-1 Comm Controller" } ,
	{ 0x107E, 0x9090, "4531S-000/001", "PMC ATM Over T3/E3 Communications Ctrlr" } ,
	{ 0x107E, 0x90A0, "4539", "PMC Quad T1/E1/J1 Communications Ctrlr" } ,
	{ 0x107F, 0x0802, "SL82C105", "EIDE Ctrlr" } ,
	{ 0x107F, 0x0803, "", "EIDE Bus Master Controller" } ,
	{ 0x107F, 0x0806, "", "EIDE Controller" } ,
	{ 0x107f, 0x1138, "1138", "" } ,
	{ 0x107F, 0x2015, "", "EIDE Controller" } ,
	{ 0x1080, 0x0600, "82C596/9", "CPU to PCI & PCI to ISA Bridge" } ,
	{ 0x1080, 0xC691, "Cypress CY82C691", "" } ,
	{ 0x1080, 0xC693, "82C693", "PCI to ISA Bridge" } ,
	{ 0x1081, 0x0D47, "", "Radius PCI to NuBUS Bridge" } ,
	{ 0x1083, 0x0001, "FR710", "PCI Enhanced IDE Adapter" } ,
	{ 0x1083, 0x0613, "", "Host Bridge" } ,
	{ 0x1085, 0x0001, "UsbDgn", "Datalaster Interface for OBD automotive" } ,
	{ 0x1087, 0x9200, "", "" } ,
	{ 0x108A, 0x0001, "Model 617", "PCI-VME Bus Adapter" } ,
	{ 0x108A, 0x0010, "Model 618", "VME Bridge" } ,
	{ 0x108A, 0x0040, "dataBLIZZARD", "" } ,
	{ 0x108A, 0x3000, "Model 2106", "VME Bridge" } ,
	{ 0x108D, 0x0001, "OC-3136/37", "Token-Ring 16/4 PCI Adapter" } ,
	{ 0x108D, 0x0002, "OC-3139f", "Fastload 16/4 PCI/III Token Ring Adapter" } ,
	{ 0x108D, 0x0004, "OC-3139/40", "RapidFire Token Ring 16/4 Adapter" } ,
	{ 0x108D, 0x0005, "OC-3250", "GoCard Token Ring 16/4 Adapter" } ,
	{ 0x108D, 0x0006, "OC-3530", "RapidFire Token Ring 100 Adapter" } ,
	{ 0x108D, 0x0007, "OC-3141", "RapidFire Token Ring 16/4 Adapter" } ,
	{ 0x108D, 0x0008, "OC-3540", "RapidFire HSTR 100/16/4 Adapter" } ,
	{ 0x108D, 0x000A, "OC-3150", "RapidFire Token-Ring 16/4 PCI Adapter" } ,
	{ 0x108D, 0x0011, "OC-2805", "Ethernet Controller" } ,
	{ 0x108D, 0x0012, "OC-2325", "Ethernet PCI/II 10/100 Controller" } ,
	{ 0x108D, 0x0013, "OC-2183/85", "PCI/II Ethernet Controller" } ,
	{ 0x108D, 0x0014, "OC-2326", "Ethernet PCI/II 10/100 Controller" } ,
	{ 0x108D, 0x0019, "OC-2327/50", "10/100 Ethernet Controller" } ,
	{ 0x108D, 0x0021, "OC-6151/52", "155 Mbit ATM Adapter" } ,
	{ 0x108D, 0x0022, "", "ATM Adapter" } ,
	{ 0x108E, 0x0001, "SPARC EBUS", "" } ,
	{ 0x108E, 0x1000, "PCIO", "PCI Input/Output Controller" } ,
	{ 0x108E, 0x1001, "PCIO", "Happy Meal Ethernet" } ,
	{ 0x108E, 0x1100, "RIO EBUS", "" } ,
	{ 0x108E, 0x1101, "RIO GEM", "" } ,
	{ 0x108E, 0x1102, "RIO 1394", "" } ,
	{ 0x108E, 0x1103, "RIO USB", "" } ,
	{ 0x108E, 0x2BAD, "GEM", "Sun Gigabit Ethernet Card" } ,
	{ 0x108E, 0x5000, "SME2411", "UltraSPARC-IIi Advanced PCI Bridge" } ,
	{ 0x108E, 0x5043, "SunPCI", "Co-processor" } ,
	{ 0x108E, 0x8000, "STP2223BGA", "UPA to PCI Interface (UPA)" } ,
	{ 0x108E, 0x8001, "Schizo", "PCI Bus Module" } ,
	{ 0x108E, 0xA000, "UltraSPARC IIi", "Sabre" } ,
	{ 0x108E, 0xA001, "UltraSPARC IIe", "Hummingbird" } ,
	{ 0x1091, 0x0020, "", "3D Graphics Processor" } ,
	{ 0x1091, 0x0021, "", "3D graphics processor w/texturing" } ,
	{ 0x1091, 0x0040, "", "3D graphics frame buffer" } ,
	{ 0x1091, 0x0041, "", "3D graphics frame buffer" } ,
	{ 0x1091, 0x0060, "", "Proprietary bus Bridge" } ,
	{ 0x1091, 0x00E4, "Powerstorm 4D50T", "" } ,
	{ 0x1091, 0x0720, "", "Motion JPEG Codec" } ,
	{ 0x1092, 0x00A0, "SpeedStar Pro SE", "GUI Accelerator" } ,
	{ 0x1092, 0x00A8, "SpeedStar 64", "GUI Accelerator" } ,
	{ 0x1092, 0x0550, "Viper V550", "" } ,
	{ 0x1092, 0x08D4, "Supra 2260", "WinModem" } ,
	{ 0x1092, 0x094C, "SupraExpress 56i Pro", "SupraExpress 56i Pro" } ,
	{ 0x1092, 0x09C8, "SUP2760", "SupraExpress 56i Pro VCC" } ,
	{ 0x1092, 0x1092, "Viper V330", "" } ,
	{ 0x1092, 0x6120, "Maximum", "DVD" } ,
	{ 0x1092, 0x8810, "Stealth SE", "GUI Accelerator" } ,
	{ 0x1092, 0x8811, "Stealth 64/SE", "GUI Accelerator" } ,
	{ 0x1092, 0x8880, "Stealth Video", "" } ,
	{ 0x1092, 0x8881, "Stealth Video", "GUI Accelerator" } ,
	{ 0x1092, 0x88B0, "Stealth 64 Video", "GUI Accelerator" } ,
	{ 0x1092, 0x88B1, "Stealth 64 Video", "GUI Accelerator" } ,
	{ 0x1092, 0x88C0, "Stealth 64", "GUI Accelerator" } ,
	{ 0x1092, 0x88C1, "Stealth 64", "GUI Accelerator" } ,
	{ 0x1092, 0x88D0, "Stealth 64", "GUI Accelerator" } ,
	{ 0x1092, 0x88D1, "Stealth 64", "GUI Accelerator" } ,
	{ 0x1092, 0x88F0, "Stealth 64 Video", "GUI Accelerator" } ,
	{ 0x1092, 0x88F1, "Stealth 64 Video", "GUI Accelerator" } ,
	{ 0x1092, 0x9876, "", "Supra Express 56i Pro CW #2" } ,
	{ 0x1092, 0x9999, "Monster Sound", "Diamand Technology DT0398" } ,
	{ 0x1093, 0x0160, "PCI-DIO-96", "data adquisition input and output" } ,
	{ 0x1093, 0x0161, "PCI-1200", "Multifunction data acquisition board" } ,
	{ 0x1093, 0x0162, "PCI-MIO-16XE-50", "24MIO  6-03-2" } ,
	{ 0x1093, 0x1150, "PCI-DIO-32HS", "High Speed Digital I/O Board" } ,
	{ 0x1093, 0x1170, "PCI-MIO-16XE-10", "" } ,
	{ 0x1093, 0x1180, "PCI-MIO-16E-1", "" } ,
	{ 0x1093, 0x1190, "PCI-MIO-16E-4", "" } ,
	{ 0x1093, 0x1270, "PCI-6032E", "Multifunction Data Acquisition Card" } ,
	{ 0x1093, 0x1310, "PCI-6602", "Data Acquisition Device" } ,
	{ 0x1093, 0x1330, "PCI-6031E", "" } ,
	{ 0x1093, 0x1340, "PCI-6033E", "Multifunction Data Acquisition Card" } ,
	{ 0x1093, 0x1350, "PCI-6071E", "" } ,
	{ 0x1093, 0x17D0, "PCI-6503", "" } ,
	{ 0x1093, 0x2A60, "PCI-6023E", "" } ,
	{ 0x1093, 0x2A70, "PCI-6024E", "Multifunction Data Acquisition Card" } ,
	{ 0x1093, 0x2A80, "PCI-6025E", "Multifunction Data Acquisition Card" } ,
	{ 0x1093, 0x2C80, "PCI-6035E", "" } ,
	{ 0x1093, 0xB001, "IMAQ-PCI-1408", "" } ,
	{ 0x1093, 0xB011, "IMAQ-PXI-1408", "" } ,
	{ 0x1093, 0xB021, "IMAQ-PCI-1424", "" } ,
	{ 0x1093, 0xB031, "IMAQ-PCI-1413", "" } ,
	{ 0x1093, 0xB041, "IMAQ-PCI-1407", "" } ,
	{ 0x1093, 0xB051, "IMAQ-PXI-1407", "" } ,
	{ 0x1093, 0xB061, "IMAQ-PCI-1411", "" } ,
	{ 0x1093, 0xB071, "IMAQ-PCI-1422", "" } ,
	{ 0x1093, 0xB081, "IMAQ-PXI-1422", "" } ,
	{ 0x1093, 0xB091, "IMAQ-PXI-1411", "" } ,
	{ 0x1093, 0xC801, "PCI-GPIB", "GPIB Controller Interface Board" } ,
	{ 0x1093, 0xd130, "PCI-232/2", "2-port RS-232 Serial Interface Board" } ,
	{ 0x1095, 0x0640, "PCI0640A/B", "EIDE Ctrlr" } ,
	{ 0x1095, 0x0641, "PCI0640", "PCI EIDE Adapter with RAID 1" } ,
	{ 0x1095, 0x0642, "PCI0642", "IDE Cntrlr w/RAID 1" } ,
	{ 0x1095, 0x0643, "PCI0643", "PCI EIDE controller" } ,
	{ 0x1095, 0x0646, "PCI0646", "bus master IDE" } ,
	{ 0x1095, 0x0647, "PCI0647", "" } ,
	{ 0x1095, 0x0648, "PCI-648", "Bus Master Ultra DMA PCI-IDE/ATA Chip" } ,
	{ 0x1095, 0x0649, "PCI-649", "Ultra ATA/100 PCI to IDE/ATA Controller" } ,
	{ 0x1095, 0x0650, "PBC0650A", "Fast SCSI-II Ctrlr" } ,
	{ 0x1095, 0x0670, "USB0670", "PCI-USB ASIC" } ,
	{ 0x1095, 0x0673, "USB0673", "PCI-USB ASIC" } ,
	{ 0x1095, 0x0680, "SiI 0680", "UltraATA/133 EIDE Controller" } ,
	{ 0x1095, 0x3112, "SiI 3112", "SATARaid Controller" } ,
	{ 0x1097, 0x0038, "", "EIDE Controller (single FIFO)" } ,
	{ 0x1098, 0x0001, "QD8500", "EIDE Controller" } ,
	{ 0x1098, 0x0002, "QD8580", "EIDE Controller" } ,
	{ 0x109E, 0x0350, "BT848", "TV/PCI with DMA Push" } ,
	{ 0x109E, 0x0351, "Bt849", "Video Capture" } ,
	{ 0x109E, 0x0369, "Bt878", "Video Capture" } ,
	{ 0x109E, 0x036C, "Bt879KHF", "Video Capture" } ,
	{ 0x109E, 0x036E, "Bt878", "MediaStream Controller" } ,
	{ 0x109E, 0x036F, "Bt879", "Video Capture" } ,
	{ 0x109E, 0x0370, "Bt880", "Video Capture (10 bit High qualtiy cap)" } ,
	{ 0x109E, 0x0878, "Bt878", "Video Capture (Audio Section)" } ,
	{ 0x109E, 0x0879, "Bt879", "Video Capture (Audio Section)" } ,
	{ 0x109E, 0x0880, "Bt880", "Video Capture (Audio Section)" } ,
	{ 0x109E, 0x2115, "BtV 2115", "BtV Mediastream Controller" } ,
	{ 0x109E, 0x2125, "BtV 2125", "BtV Mediastream Controller" } ,
	{ 0x109E, 0x2164, "BtV 2164", "Display Adapter" } ,
	{ 0x109E, 0x2165, "BtV 2165", "MediaStream Controller" } ,
	{ 0x109E, 0x8230, "BtV 8230", "ATM Segment/Reassembly Controller (SRC)" } ,
	{ 0x109E, 0x8472, "Bt8471/72", "32/64-channel HDLC Controllers" } ,
	{ 0x109E, 0x8474, "Bt8474", "128-channel HDLC Controller" } ,
	{ 0x10A4, 0X5969, "", "" } ,
	{ 0x10A8, 0x0000, "?", "64-bit GUI Accelerator" } ,
	{ 0x10a9, 0x0001, "", "Crosstalk to PCi Bridge" } ,
	{ 0x10a9, 0x0002, "Linc", "I/O Controller" } ,
	{ 0x10a9, 0x0003, "IOC3", "I/O Controller" } ,
	{ 0x10A9, 0x0004, "O2 MACE", "" } ,
	{ 0x10A9, 0x0005, "RAD Audio", "" } ,
	{ 0x10A9, 0x0006, "HPCEX", "" } ,
	{ 0x10A9, 0x0007, "RPCEX", "" } ,
	{ 0x10A9, 0x0008, "DiVO VIP", "" } ,
	{ 0x10A9, 0x0009, "Alteon", "Gigabit Ethernet" } ,
	{ 0x10A9, 0x0010, "AMP", "Video I/O" } ,
	{ 0x10A9, 0x0011, "GRIP", "" } ,
	{ 0x10A9, 0x0012, "SGH PSHAC GSN", "" } ,
	{ 0x10A9, 0x1001, "Magic Carpet", "" } ,
	{ 0x10A9, 0x1002, "Lithium", "" } ,
	{ 0x10A9, 0x1003, "Dual JPEG 1", "" } ,
	{ 0x10A9, 0x1004, "Dual JPEG 2", "" } ,
	{ 0x10A9, 0x1005, "Dual JPEG 3", "" } ,
	{ 0x10A9, 0x1006, "Dual JPEG 4", "" } ,
	{ 0x10A9, 0x1007, "Dual JPEG 5", "" } ,
	{ 0x10A9, 0x1008, "Cesium", "" } ,
	{ 0x10A9, 0x2001, "", "Fibre Channel" } ,
	{ 0x10A9, 0x2002, "ASDE", "" } ,
	{ 0x10A9, 0x8001, "O2 1394", "" } ,
	{ 0x10A9, 0x8002, "G-net NT", "" } ,
	{ 0x10AA, 0x0000, "ACC 2056/2188", "CPU to PCI Bridge (Pentium)" } ,
	{ 0x10AA, 0x2051, "", "Laptop Chipset CPU Bridge" } ,
	{ 0x10AA, 0x5842, "", "Laptop Chipset ISA Bridge" } ,
	{ 0x10AD, 0x0001, "W83769F", "EIDE Ctrlr" } ,
	{ 0x10ad, 0x0003, "SL82C103", "EIDE Controller" } ,
	{ 0x10ad, 0x0005, "SL82C105", "EIDE Busmaster Controller" } ,
	{ 0x10AD, 0x0103, "sl82c103", "PCI-ide mode 4.5 Cntrlr" } ,
	{ 0x10AD, 0x0105, "W83789F", "Sonata bus master PCI-IDE controller" } ,
	{ 0x10ad, 0x0150, "", "EIDE Controller" } ,
	{ 0x10ad, 0x0565, "W83C553F/554F", "ISA Bridge" } ,
	{ 0x10ae, 0x0002, "", "Graphics Controller" } ,
	{ 0x10af, 0x0001, "", "IDE Controller" } ,
	{ 0x10b3, 0x3106, "DB87144", "CardBus Controller" } ,
	{ 0x10b3, 0xB106, "DB87144", "PCI-to-CardBus bridge" } ,
	{ 0x10b4, 0x1B1D, "Velocity 128 3D", "" } ,
	{ 0x10b5, 0x0364, "PCI 9080RDK-RC32364", "PCI Reference Design Kit for PCI 9080" } ,
	{ 0x10b5, 0x0401, "PCI 9080RDK-401B", "PCI Reference Design Kit for PCI 9080" } ,
	{ 0x10B5, 0x0480, "IOP 480", "Integrated PowerPC I/O Processor" } ,
	{ 0x10b5, 0x0860, "PCI 9080RDK-860", "PCI Reference Design Kit for PCI 9080" } ,
	{ 0x10B5, 0x0960, "PCI 9080RDK-960", "PCI Reference Design Kit for PCI 9080" } ,
	{ 0x10B5, 0x1030, "Gazel R685", "ISDN card" } ,
	{ 0x10b5, 0x1076, "PCI 9050", "Vision Systems VScom 800" } ,
	{ 0x10b5, 0x1077, "PCI 9050", "Vision Systems VScom 400" } ,
	{ 0x10B5, 0x1078, "PCI 9050", "Vision Systems VScom PCI-210" } ,
	{ 0x10B5, 0x1103, "PCI 9050", "Vision Systems VScom PCI-200" } ,
	{ 0x10B5, 0x1146, "PCI 9050", "Vision Systems VScom PCI-010S" } ,
	{ 0x10B5, 0x1147, "PCI 9050", "Vision Systems VScom PCI-020S" } ,
	{ 0x10B5, 0x1151, "Gazel R753", "ISDN card" } ,
	{ 0x10B5, 0x1152, "Gazel R753", "ISDN card" } ,
	{ 0x10b5, 0x1860, "PCI 9054RDK-860", "Reference Design Kit for PCI 9054" } ,
	{ 0x10b5, 0x2021, "PCI9080", "Daktronics VMax Quad Transmitter Card" } ,
	{ 0x10b5, 0x2288, "", "Chrislin Industries Memory" } ,
	{ 0x10B5, 0x2724, "", "Thales PCSM Security Card" } ,
	{ 0x10b5, 0x3001, "PCI 9030RDK-LITE", "PCI Reference Design Kit for PCI 9030" } ,
	{ 0x10b5, 0x30C1, "cPCI 9030RDK-LITE", "CompactPCI Reference Design Kit for 9030" } ,
	{ 0x10b5, 0x5201, "PCI 9052RDK-LITE", "Rapid Development Kit for ISA to PCI" } ,
	{ 0x10b5, 0x5406, "PCI 9054RDK-LITE", "PCI Bus Master Prototyping Kit for 9054" } ,
	{ 0x10b5, 0x5601, "PCI 9056RDK-Lite", "PCI 9056 Rapid Development Kit" } ,
	{ 0x10b5, 0x56C2, "cPCI 9056RDK-860", "CompactPCI Rapid Design Kit for PCI 9056" } ,
	{ 0x10b5, 0x6466, "GBP32", "PCI Adaptive Switch Fabric Controller" } ,
	{ 0x10b5, 0x7709, "PCI 9080RDK-SH3", "PCI Reference Design Kit for PCI 9080" } ,
	{ 0x10B5, 0x9030, "PCI 9030", "PCI SMARTarget I/O Accelerator" } ,
	{ 0x10B5, 0x9036, "PCI9036", "Interface chip" } ,
	{ 0x10B5, 0x9050, "PCI 9050", "Target PCI Interface Chip" } ,
	{ 0x10B5, 0x9052, "PCI 9052", "PCI 9052 Target PCI Interface Chip" } ,
	{ 0x10B5, 0x9054, "PCI 9054", "PCI I/O Accelerator" } ,
	{ 0x10b5, 0x9056, "PCI 9056", "32-bit, 66MHz PCI Master I/O Accelerator" } ,
	{ 0x10B5, 0x9060, "PCI9060", "PCI Bus Master Interface Chip" } ,
	{ 0x10B5, 0x906D, "PCI 9060SD", "PCI Bus Master Interface Chip" } ,
	{ 0x10B5, 0x906E, "PCI 9060ES", "PCI Bus Master Interface Chip" } ,
	{ 0x10B5, 0x9080, "PCI 9080", "High performance PCI to Local Bus chip" } ,
	{ 0x10b5, 0x9601, "PCI 9656RDK-Lite", "PCI Rapid Development Kit for PCI 9656" } ,
	{ 0x10b5, 0x9656, "PCI 9656", "64-bit 66 MHz PCI Master I/O Accelerator" } ,
	{ 0x10b5, 0x96C2, "cPCI 9656RDK-860", "CompactPCI Rapid Development Kit" } ,
	{ 0x10b5, 0xC860, "cPCI 9054RDK-860", "CompactPCI Reference Design Kit for 9054" } ,
	{ 0x10B6, 0x0001, "Smart 16/4", "Ringnode (PCI1b)" } ,
	{ 0x10B6, 0x0002, "Smart 16/4", "Ringnode (PCIBM2/CardBus)" } ,
	{ 0x10B6, 0x0003, "Smart 16/4", "Ringnode" } ,
	{ 0x10B6, 0x0004, "", "Smart 16/4 Ringnode Mk1 (PCIBM1)" } ,
	{ 0x10B6, 0x0006, "", "16/4 CardBus Adapter (Eric 2)" } ,
	{ 0x10B6, 0x0007, "Presto PCI", "" } ,
	{ 0x10B6, 0x0009, "", "Smart 100/16/4 PCi-HS Ringnode" } ,
	{ 0x10B6, 0x000A, "", "Smart 100/16/4 PCI Ringnode" } ,
	{ 0x10B6, 0x000B, "", "16/4 CardBus  Adapter Mk2" } ,
	{ 0x10B6, 0x1000, "Horizon", "ATM adapter" } ,
	{ 0x10B6, 0x1001, "Ambassador", "ATM adapter" } ,
	{ 0x10B6, 0x1002, "Ambassador", "ATM Adapter" } ,
	{ 0x10B7, 0x0001, "3C985", "1000BaseSX Gigabit Etherlink" } ,
	{ 0x10B7, 0x1000, "3C905CX-TXNM", "3COM 3C905CX-TXNM with 40-0664-003 ASIC" } ,
	{ 0x10B7, 0x1006, "0038TA <- AC101 - TF", "Mini-PCI 56K V.90 Modem" } ,
	{ 0x10B7, 0x1007, "3C556", "V.90 Mini-PCI Modem" } ,
	{ 0x10B7, 0x1F1F, "3CRWE777A", "AirConnect Wireless LAN PCI Card" } ,
	{ 0x10B7, 0x3390, "3C339", "Token Link Velocity" } ,
	{ 0x10B7, 0x3590, "3C359", "TokenLink Velocity XL Adapter" } ,
	{ 0x10B7, 0x4500, "3C450", "Cyclone" } ,
	{ 0x10B7, 0x5055, "3C555", "Laptop Hurricane" } ,
	{ 0x10B7, 0x5057, "3C575", "Megahertz 10/100 LAN CardBus PC Card" } ,
	{ 0x10B7, 0x5157, "3C575B", "Megahertz 10/100 LAN CardBus PC Card" } ,
	{ 0x10B7, 0x5257, "3CCFE575CT", "Cyclone Fast Ethernet CardBus PC Card" } ,
	{ 0x10B7, 0x5900, "3C590", "Ethernet III Bus Fast PCI" } ,
	{ 0x10B7, 0x5920, "3C592", "PCI/EISA 10Mbps Demon/Vortex" } ,
	{ 0x10B7, 0x5950, "3C595", "Fast EtherLink PCI TX" } ,
	{ 0x10B7, 0x5951, "3C595", "Fast EtherLink PCI T4" } ,
	{ 0x10B7, 0x5952, "3C595", "Fast EtherLink PCI MII" } ,
	{ 0x10B7, 0x5970, "3C597", "PCI/EISA Fast Demon/Vortex" } ,
	{ 0x10B7, 0x5B57, "3C595", "Megahertz 10/100 LAN CardBus" } ,
	{ 0x10B7, 0x6055, "3C556", "10/100 Fast Ethernet MiniPCI Adapter" } ,
	{ 0x10B7, 0x6056, "3CN3AC1556B", "MiniPCI 10/100 Ethernet+Modem56k (see devid:1007)" } ,
	{ 0x10B7, 0x6560, "3CCFE656", "Cyclone CardBus PC Card" } ,
	{ 0x10B7, 0x6561, "FEM656", "10/100 LAN+56K Modem CardBus PC Card" } ,
	{ 0x10B7, 0x6562, "3CCFEM656", "Cyclone CardBus PC Card" } ,
	{ 0x10B7, 0x6563, "FEM656B", "10/100 LAN+56K Modem CardBus PC Card" } ,
	{ 0x10B7, 0x6564, "3CCFEM656", "Cyclone CardBus PC Card" } ,
	{ 0x10B7, 0x6565, "3CCFEM656C", "Global 10/100 Fast Ethernet+56K Modem" } ,
	{ 0x10B7, 0x7646, "3CSOHO100-TX", "Hurricane" } ,
	{ 0x10B7, 0x8811, "", "Token Ring" } ,
	{ 0x10B7, 0x9000, "3C900-TPO", "Fast Etherlink PCI TPO NIC" } ,
	{ 0x10B7, 0x9001, "3C900-COMBO", "Fast Etherlink XL PCI Combo NIC" } ,
	{ 0x10B7, 0x9004, "3C900B-TPO", "EtherLink XL TPO 10Mb" } ,
	{ 0x10B7, 0x9005, "3C900B-COMBO", "Fast Etherlink 10Mbps Combo NIC" } ,
	{ 0x10B7, 0x9006, "3C900B-TPC", "EtherLink XL TPC" } ,
	{ 0x10B7, 0x900A, "3C900B-FL", "EtherLink PCI Fiber NIC" } ,
	{ 0x10B7, 0x9050, "3C905-TX", "Fast Etherlink XL PCI 10/100" } ,
	{ 0x10B7, 0x9051, "3C905-T4", "Fast Etherlink XL 10/100" } ,
	{ 0x10B7, 0x9055, "3C905B-TX", "Fast Etherlink 10/100 PCI TX NIC" } ,
	{ 0x10B7, 0x9056, "3C905B-T4", "Fast EtherLink XL 10/100" } ,
	{ 0x10B7, 0x9058, "3C905B-COMBO", "Deluxe EtherLink 10/100 PCI Combo NIC" } ,
	{ 0x10B7, 0x905A, "3C905B-FX", "Fast EtherLink 100 Fiber NIC" } ,
	{ 0x10B7, 0x9200, "3C905C-TX", "Fast EtherLink for PC Management NIC" } ,
	{ 0x10B7, 0x9800, "3C980-TX", "Fast EtherLink XL Server Adapter" } ,
	{ 0x10B7, 0x9805, "3C980-TX", "Python-T 10/100baseTX NIC" } ,
	{ 0x10B7, 0x9902, "3CR990-TX-95", "EtherLink 10/100 PCI with 3XP Processor" } ,
	{ 0x10B7, 0x9903, "3CR990-TX-97", "EtherLink 10/100 PCI with 3XP Processor" } ,
	{ 0x10B7, 0x9908, "3CR990SVR95", "EtherLink 10/100 Server PCI with 3XP" } ,
	{ 0x10B7, 0x9909, "3CR990SVR97", "EtherLink 10/100 Server PCI with 3XP" } ,
	{ 0x10B8, 0x0005, "LAN83C170QF/171", "EPIC/XF 10/100 Mbps Fast Ethernet Ctrlr" } ,
	{ 0x10B8, 0x0006, "LAN83C175", "EPIC/C Ethernet CardBus Integrated Ctrlr" } ,
	{ 0x10B8, 0x1000, "37C665", "FDC" } ,
	{ 0x10B8, 0x1001, "37C922", "FDC" } ,
	{ 0x10B8, 0xA011, "83C170QF", "Fast ethernet controller" } ,
	{ 0x10B8, 0xB106, "SMC34C90", "CardBus Controller" } ,
	{ 0x10B9, 0x0111, "CMI8738/C3DX", "C-Media Audio Device (OEM)" } ,
	{ 0x10B9, 0x1435, "M1435", "VL Bridge" } ,
	{ 0x10B9, 0x1445, "M1445", "CPU to PCI & PCI to ISA Bridge w/EIDE" } ,
	{ 0x10B9, 0x1449, "M1449", "ISA Bridge" } ,
	{ 0x10B9, 0x1451, "M1451", "Pentium CPU to PCI Bridge" } ,
	{ 0x10B9, 0x1461, "M1461", "P54C Chipset" } ,
	{ 0x10B9, 0x1489, "M1489", "486 PCI Chipset" } ,
	{ 0x10B9, 0x1511, "M1511", "Aladdin 2 Host Bridge" } ,
	{ 0x10B9, 0x1513, "M1513", "Aladdin 2 South Bridge" } ,
	{ 0x10B9, 0x1521, "M1521", "Alladin III CPU to PCI Bridge" } ,
	{ 0x10B9, 0x1523, "M1523", "ISA Bridge" } ,
	{ 0x10b9, 0x1531, "M1531B", "ALi Aladdin IV Host Bridge" } ,
	{ 0x10B9, 0x1533, "M1533", "PCI South Bridge" } ,
	{ 0x10B9, 0x1535, "M1535x", "ISA Bridge" } ,
	{ 0x10B9, 0x1541, "M1541", "Aladdin V AGPset Host Bridge" } ,
	{ 0x10B9, 0x1543, "M1543", "Aladdin V AGPset South Bridge" } ,
	{ 0x10B9, 0x1561, "M1561", "North Bridge" } ,
	{ 0x10B9, 0x1563, "M1563", "South Bridge with Hypertransport Support" } ,
	{ 0x10b9, 0x1621, "M1621", "Aladdin-Pro II Northbridge" } ,
	{ 0x10b9, 0x1631, "M1631", "Aladdin Pro III Northbridge" } ,
	{ 0x10B9, 0x1632, "M1632", "North Bridge" } ,
	{ 0x10B9, 0x1641, "M1641", "CPU to PCI Bridge" } ,
	{ 0x10B9, 0x1644, "M1644", "AGP System Controller" } ,
	{ 0x10B9, 0x1646, "M1646", "AGP System Controller" } ,
	{ 0x10B9, 0x1647, "M1647", "CPU to PCI Bridge" } ,
	{ 0x10B9, 0x1651, "M1651", "CPU to PCI Bridge" } ,
	{ 0x10B9, 0x1661, "M1661", "AGP System Controller" } ,
	{ 0x10B9, 0x1667, "M1667", "AGP System Controller" } ,
	{ 0x10B9, 0x1671, "M1671", "Super P4 Nouth Bridge" } ,
	{ 0x10B9, 0x1681, "M1681", "P4 Nouth Bridge with HyperTransport" } ,
	{ 0x10B9, 0x1687, "M1687", "K8 North Bridge with HyperTransport" } ,
	{ 0x10B9, 0x3141, "M3141", "GUI Accelerator" } ,
	{ 0x10B9, 0x3143, "M3143", "GUI Accelerator" } ,
	{ 0x10B9, 0x3145, "M3145", "GUI Accelerator" } ,
	{ 0x10B9, 0x3147, "M3147", "GUI Accelerator" } ,
	{ 0x10B9, 0x3149, "M3149", "GUI Accelerator" } ,
	{ 0x10B9, 0x3151, "M3151", "GUI Accelerator" } ,
	{ 0x10B9, 0x3307, "M3307", "MPEG-1 Decoder" } ,
	{ 0x10B9, 0x3309, "M3309", "MPEG Decoder" } ,
	{ 0x10B9, 0x5212, "M4803", "" } ,
	{ 0x10B9, 0x5215, "MS4803", "EIDE Ctrlr" } ,
	{ 0x10B9, 0x5217, "m5217h", "I/O (?)" } ,
	{ 0x10B9, 0x5219, "m5219", "PCI Bus Master IDE Controller" } ,
	{ 0x10B9, 0x5225, "M5225", "IDE Controller" } ,
	{ 0x10B9, 0x5229, "M1543 Southbridge", "EIDE Controller" } ,
	{ 0x10B9, 0x5235, "M5235", "I/O Controller" } ,
	{ 0x10b9, 0x5237, "M5237", "USB Host Controller" } ,
	{ 0x10b9, 0x5240, "", "EIDE Controller" } ,
	{ 0x10b9, 0x5241, "", "PCMCIA Bridge" } ,
	{ 0x10b9, 0x5242, "", "General Purpose Controller" } ,
	{ 0x10b9, 0x5243, "M1541A", "Aladdin V PCI-to-PCI Bridge" } ,
	{ 0x10b9, 0x5244, "", "Floppy Disk Controller" } ,
	{ 0x10b9, 0x5247, "M1621", "Aladdin V built-in PCI-to-PCI bridge" } ,
	{ 0x10B9, 0x5249, "M5249", "HyperTransport to PCI Bridge" } ,
	{ 0x10B9, 0x5251, "M5251", "IEEE P1394 OpenHCI 1.0 Controller" } ,
	{ 0x10B9, 0x5253, "M5253", "IEEE P1394 OpenHCI 1.0 Controller" } ,
	{ 0x10b9, 0x5427, "M6VLR", "PCI to AGP Bridge" } ,
	{ 0x10B9, 0x5450, "", "Agere Systems AC97 Modem" } ,
	{ 0x10b9, 0x5451, "M5451", "PCI AC-link Controller Audio Device" } ,
	{ 0x10b9, 0x5453, "M5453", "PCI AC-link Controller Modem Device" } ,
	{ 0x10B9, 0x5457, "ALI N5457", "Agere Systems AC97 Modem" } ,
	{ 0x10B9, 0X5459, "MDV92XP NetoDragon", "PCI Soft Modem V92 NetoDragon" } ,
	{ 0x10B9, 0x7101, "M7101", "Power Management Controller" } ,
	{ 0x10ba, 0x0301, "", "GUI Accelerator" } ,
	{ 0x10BA, 0x0304, "", "GUI Accelerator" } ,
	{ 0x10BD, 0x0E34, "NE34", "Ethernet Adapter (NE2000 PCI clone)" } ,
	{ 0x10BD, 0x5240, "", "IDE Cntrlr" } ,
	{ 0x10BD, 0x5241, "", "PCMCIA Bridge" } ,
	{ 0x10BD, 0x5242, "", "General Purpose Cntrlr" } ,
	{ 0x10BD, 0x5243, "", "Bus Cntrlr" } ,
	{ 0x10BD, 0x5244, "", "FCD Cntrlr" } ,
	{ 0x10c3, 0x1100, "SC1100", "SmartEther100 LAN Adapter (i82557B)" } ,
	{ 0x10C4, 0x8363, "", "" } ,
	{ 0x10C8, 0x0000, "", "Graphics Cntrlr" } ,
	{ 0x10c8, 0x0001, "NM2070", "MagicGraph 128" } ,
	{ 0x10c8, 0x0002, "NM2090", "MagicGraph 128V" } ,
	{ 0x10C8, 0x0003, "NM2093", "MagicGraph 128ZV Video Controller" } ,
	{ 0x10C8, 0x0004, "NM2160", "MagicGraph 128XD" } ,
	{ 0x10C8, 0x0005, "NM2200", "MagicMedia 256AV" } ,
	{ 0x10C8, 0x0006, "NM2360", "MagicMedia 256ZX/256M6D" } ,
	{ 0x10C8, 0x0016, "NM2380", "MagicMedia 256XL+" } ,
	{ 0x10C8, 0x0025, "NM2230", "MagicMedia 256AV+" } ,
	{ 0x10C8, 0x0083, "NM2097", "Graphic Controller NeoMagic MagicGraph128ZV+" } ,
	{ 0x10C8, 0x8005, "NM2200", "MagicMedia 256AV Audio Device" } ,
	{ 0x10C8, 0x8006, "NM2360", "MagicMedia 256ZX Audio Device" } ,
	{ 0x10C8, 0x8016, "NM2380", "MagicMedia 256XL+ Audio Device" } ,
	{ 0x10cc, 0x0226, "Articia S", "RISC chipset with AGP2X" } ,
	{ 0x10cc, 0x0257, "Articia Sa", "RISC chipset with PCIX, AGP2X, DDR, DMA" } ,
	{ 0x10CD, 0x1100, "ASC1100", "PCI SCSI Host Adapter" } ,
	{ 0x10CD, 0x1200, "ASC1200", "Fast SCSI-II" } ,
	{ 0x10CD, 0x1300, "ASC-3050", "ASC-3150" } ,
	{ 0x10CD, 0x2300, "ASC2300", "PCI Ultra Wide SCSI-2 Host Adapter" } ,
	{ 0x10CD, 0x2500, "ASC38C0800/1600", "PCI Ultra 80/160 SCSI Controllers" } ,
	{ 0x10CD, 0x4000, "ASC30C0400", "IEEE-1394 OHCI PCI Controller" } ,
	{ 0x10CF, 0x10C5, "FMV-103", "Serial Parallel Card" } ,
	{ 0x10CF, 0x2001, "MB86605", "PCI SCSI Host Adapter (Fast Wide SCSI-2)" } ,
	{ 0x10CF, 0x2002, "MB86606", "Fast Wide SCSI Controller" } ,
	{ 0x10CF, 0x2005, "MB86974", "10/100 Fast Ethernet Adapter" } ,
	{ 0x10CF, 0x200C, "MB86974", "IEEE1394 OpenHCI Controller" } ,
	{ 0x10CF, 0x2010, "", "OHCI FireWire Controller" } ,
	{ 0x10CF, 0x2011, "", "MPEG2 R-Engine (MPEG2 Hardware Encoder)" } ,
	{ 0x10D9, 0x0066, "MX86101P", "" } ,
	{ 0x10D9, 0x0512, "MX98713", "Fast Ethernet Adapter" } ,
	{ 0x10D9, 0x0531, "MX98715/725", "Single Chip Fast Ethernet NIC Controller" } ,
	{ 0x10D9, 0x0532, "MX98723/727", "PCI/CardBus Fast Ethernet Controller" } ,
	{ 0x10D9, 0x0553, "MX987x5", "Ethernet Adapter" } ,
	{ 0x10D9, 0x8625, "MX86250", "" } ,
	{ 0x10D9, 0x8626, "MX86251", "" } ,
	{ 0x10D9, 0x8627, "MX86251", "" } ,
	{ 0x10D9, 0x8888, "MX86200", "9619E" } ,
	{ 0x10D9, 0xC115, "lc82c115", "" } ,
	{ 0x10DA, 0x0508, "TC4048", "Token Ring" } ,
	{ 0x10DA, 0x3390, "Tl3c3x9", "Token Ring" } ,
	{ 0x10DC, 0x0001, "STAR/RD24", "SCSI (PMC)" } ,
	{ 0x10DC, 0x0002, "ATT 2C15-3 (FPGA)", "SCI bridge  on PCI 5 Volt card" } ,
	{ 0x10DC, 0x0010, "680-1110-150/400", "Simple PMC/PCI to S-LINK interface" } ,
	{ 0x10DC, 0x0011, "680-1110-200/450", "Simple S-LINK to PMC/PCI interface" } ,
	{ 0x10DC, 0x0012, "S32PCI64", "32-bit S-LINK to 64-bit PCI interface" } ,
	{ 0x10DC, 0x0021, "", "HIPPI destination" } ,
	{ 0x10DC, 0x0022, "", "HIPPI source" } ,
	{ 0x10dc, 0x0033, "", "ALICE DDL PCI adapter card (RORC)" } ,
	{ 0x10DC, 0x10DC, "ATT 2C15-3 (FPGA)", "" } ,
	{ 0x10DD, 0x0001, "", "3D graphics processor" } ,
	{ 0x10DE, 0x0008, "NV1", "Edge 3D" } ,
	{ 0x10DE, 0x0009, "NV1", "Edge 3D" } ,
	{ 0x10DE, 0x0010, "NV2", "Mutara V08" } ,
	{ 0x10DE, 0x0018, "NV3", "Riva 128" } ,
	{ 0x10DE, 0x0019, "NV3", "Riva 128ZX" } ,
	{ 0x10DE, 0x0020, "NV4", "Riva TNT" } ,
	{ 0x10DE, 0x0028, "NV5", "TNT2 / TNT2 Pro" } ,
	{ 0x10DE, 0x0029, "NV5", "TNT2 Ultra" } ,
	{ 0x10DE, 0x002A, "NV5", "TNT2" } ,
	{ 0x10DE, 0x002B, "NV5", "Riva TNT2" } ,
	{ 0x10DE, 0x002C, "NV5", "Vanta/Vanta LT" } ,
	{ 0x10DE, 0x002D, "NV5", "TNT2 Model 64 / TNT2 Model 64 Pro" } ,
	{ 0x10DE, 0x002E, "NV6", "VANTA" } ,
	{ 0x10DE, 0x002F, "NV6", "VANTA" } ,
	{ 0x10DE, 0x0060, "nForce MCP2", "ISA Bridge" } ,
	{ 0x10DE, 0x0064, "nForce MCP-T?", "SMBus Controller" } ,
	{ 0x10DE, 0x0065, "nForce MCP2", "EIDE Controller" } ,
	{ 0x10DE, 0x0066, "nForce MCP2", "Networking Adapter" } ,
	{ 0x10DE, 0x0067, "nForce MCP2", "OpenHCI USB Controller" } ,
	{ 0x10DE, 0x0068, "nForce MCP2", "EHCI USB 2.0 Controller" } ,
	{ 0x10DE, 0x006A, "nForce MCP2", "Audio Codec Interface" } ,
	{ 0x10DE, 0x006B, "nForce MCP-T?", "Audio Processing Unit (Dolby Digital)" } ,
	{ 0x10DE, 0x006C, "nForce", "PCI to PCI Bridge" } ,
	{ 0x10DE, 0x006E, "nForce MCP2", "OHCI Compliant IEEE 1394 Controller" } ,
	{ 0x10DE, 0x00A0, "NV5", "Aladdin TNT2" } ,
	{ 0x10DE, 0x00D4, "nForce MCP3?", "SMBus Controller" } ,
	{ 0x10DE, 0x00DA, "nForce MCP3", "Audio Codec Interface" } ,
	{ 0x10DE, 0x0100, "NV10", "GeForce 256" } ,
	{ 0x10DE, 0x0101, "NV10", "GeForce 256 DDR" } ,
	{ 0x10DE, 0x0102, "NV10", "GeForce 256 Ultra" } ,
	{ 0x10DE, 0x0103, "NV10GL", "Quadro (GeForce 256 GL)" } ,
	{ 0x10DE, 0x0110, "NV11", "GeForce2 MX / MX 400" } ,
	{ 0x10DE, 0x0111, "NV11DDR", "GeForce2 MX 100/200� (DDR)" } ,
	{ 0x10DE, 0x0112, "NV11", "GeForce2 Go / MX Ultra" } ,
	{ 0x10DE, 0x0113, "NV11GL", "Quadro2 MXR / EX / Go" } ,
	{ 0x10DE, 0x0150, "NV15", "GeForce2 GTS / Pro" } ,
	{ 0x10DE, 0x0151, "NV15 DDR", "GeForce2 Ti�(DDR)" } ,
	{ 0x10DE, 0x0152, "NV15BR", "GeForce2 Ultra (BladeRunner)" } ,
	{ 0x10DE, 0x0153, "NV15GL", "Quadro2 Pro" } ,
	{ 0x10DE, 0x0170, "NV17.1", "GeForce4 MX 460" } ,
	{ 0x10DE, 0x0171, "NV17.2", "GeForce4 MX 440" } ,
	{ 0x10DE, 0x0172, "NV17.3", "GeForce4 MX 420" } ,
	{ 0x10DE, 0x0173, "NV17", "" } ,
	{ 0x10DE, 0x0174, "NV17M", "GeForce4 440 Go" } ,
	{ 0x10DE, 0x0175, "NV17M", "GeForce4 420 Go" } ,
	{ 0x10DE, 0x0176, "NV17M", "GeForce4 420 Go 32M" } ,
	{ 0x10DE, 0x0177, "NV17M", "GeForce4 460 Go" } ,
	{ 0x10DE, 0x0178, "NV17GL.1", "Quadro4 500/550 XGL" } ,
	{ 0x10DE, 0x0179, "NV17M", "GeForce4 440 Go 64M" } ,
	{ 0x10DE, 0x017A, "NV17GL.2", "Quadro4 200/400 NVS" } ,
	{ 0x10DE, 0x017B, "NV17GL.3", "Quadro4 550 XGL" } ,
	{ 0x10DE, 0x017C, "NV17M-GL", "Quadro4 500 GoGL" } ,
	{ 0x10de, 0x0180, "NV18", "GeForce4 MX 440 with AGP8X" } ,
	{ 0x10de, 0x0181, "NV18", "GeForce4 MX 440SE with AGP8X" } ,
	{ 0x10de, 0x0182, "NV18", "GeForce4 MX 420 with AGP8X" } ,
	{ 0x10DE, 0x0183, "", "GeForce4 MX 460?" } ,
	{ 0x10de, 0x0188, "NV18GL", "Quadro4 580 XGL" } ,
	{ 0x10de, 0x018A, "NV18GL", "Quadro4 280 NVS" } ,
	{ 0x10de, 0x018B, "NV18GL", "Quadro4 380 XGL" } ,
	{ 0x10De, 0x01A0, "Crush11", "GeForce2 Integrated graphics" } ,
	{ 0x10DE, 0x01A4, "nForce", "AGP Controller" } ,
	{ 0x10DE, 0x01A5, "nForce", "AGP Controller" } ,
	{ 0x10DE, 0x01A6, "nForce", "AGP Controller" } ,
	{ 0x10DE, 0x01A8, "nForce 220", "Memory Controller (SDR)" } ,
	{ 0x10DE, 0x01A9, "nForce 420", "Memory Controller (SDR)" } ,
	{ 0x10DE, 0x01AA, "nForce 220/230", "Memory Controller (DDR)" } ,
	{ 0x10DE, 0x01AB, "nForce 415/420/430", "Memory Controller (DDR)" } ,
	{ 0x10DE, 0x01AC, "nForce 2x0/415/4x0", "Memory Controller" } ,
	{ 0x10DE, 0x01AD, "nForce 2x0/415/4x0", "Memory Controller" } ,
	{ 0x10DE, 0x01B0, "nForce MCP", "Audio Processing Unit (Dolby Digital)" } ,
	{ 0x10DE, 0x01B1, "nForce MCP", "Audio Codec Interface" } ,
	{ 0x10DE, 0x01B2, "nForce", "HUB Interface" } ,
	{ 0x10DE, 0x01B4, "nForce MCP", "SMBus Controller" } ,
	{ 0x10DE, 0x01B7, "nForce AGP", "Host to PCI Bridge" } ,
	{ 0x10DE, 0x01B8, "nForce", "PCI Bridge" } ,
	{ 0x10DE, 0x01BC, "nForce MCP", "ATA Controller" } ,
	{ 0x10DE, 0x01C1, "nForce MCP", "AC97 Modem" } ,
	{ 0x10DE, 0x01C2, "nForce MCP", "OHCI USB Controller" } ,
	{ 0x10DE, 0x01C3, "nForce MCP", "Networking Adapter" } ,
	{ 0x10DE, 0x01E0, "nForce2", "AGP Controller" } ,
	{ 0x10DE, 0x01E1, "nForce2", "AGP Controller" } ,
	{ 0x10DE, 0x01E8, "nForce2", "AGP Host to PCI Bridge" } ,
	{ 0x10DE, 0x01EA, "nForce2", "Memory Controller 0" } ,
	{ 0x10DE, 0x01EB, "nForce2", "Memory Controller 1" } ,
	{ 0x10DE, 0x01EC, "nForce2", "Memory Controller 2" } ,
	{ 0x10DE, 0x01ED, "nForce2", "Memory Controller 3" } ,
	{ 0x10DE, 0x01EE, "nForce2", "Memory Controller 4" } ,
	{ 0x10DE, 0x01EF, "nForce2", "Memory Controller 5" } ,
	{ 0x10de, 0x01F0, "nForce2 IGP", "GeForce4 MX Integrated GPU" } ,
	{ 0x10DE, 0x0200, "NV20", "GeForce3" } ,
	{ 0x10de, 0x0201, "NV20DDR", "GeForce3 Ti 200" } ,
	{ 0x10DE, 0x0202, "NV20BR", "GeForce3 Ti 500" } ,
	{ 0x10DE, 0x0203, "NV20DCC", "Quadro DCC" } ,
	{ 0x10de, 0x0250, "NV25.1", "GeForce4 Ti 4600" } ,
	{ 0x10de, 0x0251, "NV25.2", "GeForce4 Ti 4400" } ,
	{ 0x10de, 0x0252, "NV25.3", "" } ,
	{ 0x10de, 0x0253, "NV25.4", "GeForce4 Ti 4200" } ,
	{ 0x10de, 0x0258, "NV25GL.1", "Quadro4 900 XGL" } ,
	{ 0x10de, 0x0259, "NV25GL.2", "Quadro4 750 XGL" } ,
	{ 0x10de, 0x025B, "NV25GL.4", "Quadro4 700 XGL" } ,
	{ 0x10de, 0x0280, "NV28", "GeForce4 Ti 4600 with AGP 8X" } ,
	{ 0x10de, 0x0281, "NV28", "GeForce4 Ti 4200 with AGP8X" } ,
	{ 0x10de, 0x0282, "NV28", "GeForce4 Ti 4800 SE" } ,
	{ 0x10de, 0x0288, "NV28GL", "Quadro4 980 XGL" } ,
	{ 0x10de, 0x0289, "NV28GL", "Quadro4 780 XGL" } ,
	{ 0x10DE, 0x02A0, "NV2A", "GeForce3 Integrated GPU" } ,
	{ 0x10de, 0x0301, "NV30", "GeForceFX 5800" } ,
	{ 0x10de, 0x0302, "NV30", "GeForceFX 5800" } ,
	{ 0x10de, 0x0308, "NV30GL", "QuadroFX" } ,
	{ 0x10de, 0x0309, "NV30GL", "QuadroFX" } ,
	{ 0x10DF, 0x10DF, "Light Pulse", "Fibre Channel Adapter" } ,
	{ 0x10DF, 0x1AE5, "LP6000", "Fibre Channel Host Adapter" } ,
	{ 0x10DF, 0xF700, "LP7000", "Fibre Channel Host Adapter" } ,
	{ 0x10DF, 0xF800, "LP8000", "Fibre Channel Host Adapter" } ,
	{ 0x10E0, 0x5026, "IMS5026/27/28", "VL Bridge" } ,
	{ 0x10E0, 0x5027, "IMS5027", "" } ,
	{ 0x10E0, 0x5028, "IMS5028", "ISA Bridge" } ,
	{ 0x10E0, 0x8849, "IMS8849/48", "VL Bridge" } ,
	{ 0x10E0, 0x8853, "IMS8853", "ATM Network Adapter" } ,
	{ 0x10E0, 0x9128, "IMS9129", "GUI Accelerator" } ,
	{ 0x10E1, 0x0391, "TRM-S1040", "" } ,
	{ 0x10E1, 0x690C, "DC-690c", "" } ,
	{ 0x10E1, 0xDC20, "DC-290", "EIDE Controller" } ,
	{ 0x10E3, 0x0000, "CA91C042/142", "Universe/II VMEbus Bridge" } ,
	{ 0x10E3, 0x0513, "Tsi320", "Dual-Mode PCI-to-PCI Bus Bridge" } ,
	{ 0x10E3, 0x0850, "Tsi850", "Power PC Dual PCI Host Bridge" } ,
	{ 0x10E3, 0x0854, "Tsi850", "Power PC Single PCI Host Bridge" } ,
	{ 0x10E3, 0x0860, "CA91C860", "QSpan Motorola Processor Bridge" } ,
	{ 0x10E3, 0x0862, "CA91L862A", "QSpan II PCI-to-Motorola CPU Bridge" } ,
	{ 0x10E3, 0x8260, "CA91L8200/8260", "PowerSpan II PowerPC-to-PCI Bus Switch" } ,
	{ 0x10E3, 0x8261, "CA91L8200/8260", "PowerSpan II PowerPC-to-PCI Bus Switch" } ,
	{ 0x10E8, 0x2011, "Q-Motion pci 200", "Video Capture/Edit board" } ,
	{ 0x10E8, 0x4750, "S5930/33/35", "PCI MatchMaker" } ,
	{ 0x10E8, 0x5920, "S5920", "32-Bit PCI Bus Target Interface" } ,
	{ 0x10e8, 0x8001, "S5933", "Daktronics VMax Transmitter Card" } ,
	{ 0x10E8, 0x8033, "BBK-PCI light", "Transputer Link Interface" } ,
	{ 0x10E8, 0x8043, "LANai4.x", "Myrinet LANai interface chip" } ,
	{ 0x10E8, 0x8062, "S5933", "Parastation" } ,
	{ 0x10E8, 0x807D, "S5933", "PCI44" } ,
	{ 0x10E8, 0x8088, "FS", "Kingsberg Spacetec Format Synchronizer" } ,
	{ 0x10E8, 0x8089, "SOB", "Kingsberg Spacetec Serial Output Board" } ,
	{ 0x10E8, 0x809C, "S5933", "Traquair HEPC3" } ,
	{ 0x10E8, 0x80D7, "PCI-9112", "" } ,
	{ 0x10E8, 0x80D8, "PCI-7200", "" } ,
	{ 0x10E8, 0x80D9, "PCI-9118", "" } ,
	{ 0x10E8, 0x811A, "PCI-DSlink", "PCI-IEEE1355-DS-DE interface" } ,
	{ 0x10E8, 0x8170, "S5933", "Matchmaker PCI Chipset Development Tool" } ,
	{ 0x10e8, 0x81B7, "S5933 / NTV", "AJAVideo NTV ITU-R.601 video stillstore" } ,
	{ 0x10EA, 0x1680, "IGA-1680", "svga" } ,
	{ 0x10EA, 0x1682, "IGA-1682", "" } ,
	{ 0x10EA, 0x1683, "IGA-1683", "" } ,
	{ 0x10EA, 0x2000, "CyberPro 2010", "TV output ram 2MB  Cyberpro2010" } ,
	{ 0x10EA, 0x2010, "CyberPro 20xx/2000A", "" } ,
	{ 0x10EA, 0x5000, "CyberPro 5000", "" } ,
	{ 0x10EA, 0x5050, "CyberPro 5050", "" } ,
	{ 0x10EB, 0x0101, "3GA", "64 bit graphics processor" } ,
	{ 0x10EB, 0x8111, "Twist3", "Frame Grabber" } ,
	{ 0x10EC, 0x8029, "RTL8029", "NE2000 compatible Ethernet" } ,
	{ 0x10EC, 0x8129, "RTL8129", "10/100 Fast Ethernet Controller" } ,
	{ 0x10EC, 0x8138, "RT8139B/C", "CardBus Fast Ethernet Adapter" } ,
	{ 0x10EC, 0x8139, "RT8139A/B/C", "Fast Ethernet Adapter" } ,
	{ 0x10ED, 0x7310, "V7310", "VGA Video Overlay Adapter" } ,
	{ 0x10EE, 0X1001, "8343176", "PCI to H.100 audio interface" } ,
	{ 0x10EE, 0x3FC0, "RME Digi96", "" } ,
	{ 0x10EE, 0x3FC1, "RME Digi96/8", "" } ,
	{ 0x10EE, 0x3FC2, "RME Digi 96/8 Pro", "" } ,
	{ 0x10EE, 0x3FC3, "RME Digi96/8 Pad", "" } ,
	{ 0x10EE, 0x3FC4, "Digi9652", "Hammerfall" } ,
	{ 0x10EE, 0x5343, "Seamont SC100", "Security Adapter" } ,
	{ 0x10EE, 0x8130, "Durango PMC", "Virtex-II Bridge, XC2V1000-4FG456C" } ,
	{ 0x10EE, 0x8381, "", "" } ,
	{ 0x10EF, 0x8154, "M815x", "Token Ring Adapter" } ,
	{ 0x10F0, 0xA800, "VCL-P", "Graphics board" } ,
	{ 0x10F0, 0xB300, "VCL-M", "graphics board" } ,
	{ 0x10F1, 0x1566, "", "IDE/SCSI" } ,
	{ 0x10F1, 0x1677, "", "Multimedia" } ,
	{ 0x10F4, 0x1300, "rev1.1", "PCI to S5U13x06B0B Bridge Adapter" } ,
	{ 0x10F5, 0xA001, "NDR4000", "NR4600 Bridge" } ,
	{ 0x10FA, 0x0000, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0001, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0002, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0003, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0004, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0005, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0006, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0007, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0008, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0009, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x000A, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x000B, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x000C, "Targa 1000", "Video Capture & Editing card" } ,
	{ 0x10FA, 0x000D, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x000E, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x000F, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0010, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0011, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0012, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0013, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0014, "", "GUI Accelerator" } ,
	{ 0x10FA, 0x0015, "", "GUI Accelerator" } ,
	{ 0x10FB, 0x186f, "TH6255", "" } ,
	{ 0x1101, 0x0002, "INI-920", "Ultra SCSI Adapter" } ,
	{ 0x1101, 0x1060, "INI-A100U2W", "Orchid Ultra-2 SCSI Controller" } ,
	{ 0x1101, 0x134A, "", "Ultra SCSI Adapter" } ,
	{ 0x1101, 0x9100, "INI-9010/9010W", "Fast Wide SCSI Controller" } ,
	{ 0x1101, 0x9400, "INIC-940", "Fast Wide SCSI Controller" } ,
	{ 0x1101, 0x9401, "INIC-935", "Fast Wide SCSI Controller" } ,
	{ 0x1101, 0x9500, "INIC-950", "SCSI Controller" } ,
	{ 0x1101, 0x9700, "", "Fast Wide SCSI" } ,
	{ 0x1102, 0x0002, "EMU10000", "Sound Blaster Live!" } ,
	{ 0x1102, 0x0003, "EMU8008", "AWE64D OEM (CT4600)" } ,
	{ 0x1102, 0x0004, "EMU10K2", "Audigy Audio Processor" } ,
	{ 0x1102, 0x0006, "emu10k1x", "Soundblaster Live! 5.1" } ,
	{ 0x1102, 0x1017, "Banshee", "3D Blaster Banshee PCI CT6760" } ,
	{ 0x1102, 0x1047, "", "3D Blaster Annihilator 2" } ,
	{ 0x1102, 0x2898, "", "" } ,
	{ 0x1102, 0x4001, "EMU10K2", "Audigy IEEE1394 Firewire Controller" } ,
	{ 0x1102, 0x7002, "EMU10000", "Game Port" } ,
	{ 0x1102, 0x7003, "EMU10K2", "Audigy Gameport" } ,
	{ 0x1102, 0x8938, "EV1938", "Sound" } ,
	{ 0x1103, 0x0003, "HPT343/345/363", "UDMA EIDE Controller" } ,
	{ 0x1103, 0x0004, "HPT366/368/370/372", "UDMA66/100 EIDE Controller" } ,
	{ 0x1103, 0x0004, "HPT366/368/370/372", "" } ,
	{ 0x1103, 0x0005, "HPT372A", "UDMA/ATA133 RAID Controller" } ,
	{ 0x1103, 0x0006, "HPT302", "UDMA/ATA133 EIDE Controller" } ,
	{ 0x1103, 0x0007, "HPT371", "UDMA/ATA133 EIDE Controller" } ,
	{ 0x1103, 0x0008, "HPT374", "UDMA/ATA133 RAID Controller" } ,
	{ 0x1105, 0x5000, "", "Multimedia" } ,
	{ 0x1105, 0x8300, "EM8300", "MPEG-2 Decoder" } ,
	{ 0x1105, 0x8400, "EM8400", "MPEG-2 Decoder" } ,
	{ 0x1105, 0x8475, "EM8475", "MPEG-4 Decoder" } ,
	{ 0x1106, 0x0130, "VT6305", "VIA Fire 1394.A OHCI Link Layer Ctrlr" } ,
	{ 0x1106, 0x0198, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x0305, "VT8363A/8365", "Host Bridge" } ,
	{ 0x1106, 0x0391, "VT8363/71", "Host Bridge" } ,
	{ 0x1106, 0x0501, "VT8501", "Host Bridge" } ,
	{ 0x1106, 0x0505, "82C505", "VLB to PCI Bridge" } ,
	{ 0x1106, 0x0561, "82C570 MV", "IDE Controller" } ,
	{ 0x1106, 0x0571, "VT82C586/596/686", "PCI IDE Controller" } ,
	{ 0x1106, 0x0576, "82C576", "P54 Ctrlr" } ,
	{ 0x1106, 0x0585, "VT82C585VP/VPX", "Host Bus-PCI Bridge" } ,
	{ 0x1106, 0x0586, "VT82C586VP", "PCI-to-ISA Bridge" } ,
	{ 0x1106, 0x0595, "VT82C595", "Apollo VP2 PCI North Bridge" } ,
	{ 0x1106, 0x0596, "VT82C596/596A/596B", "PCI ISA Bridge" } ,
	{ 0x1106, 0x0597, "VT82C597", "Host Bridge (Apollo VP3)" } ,
	{ 0x1106, 0x0598, "VT82C598", "Host Bridge" } ,
	{ 0x1106, 0x0601, "VT8601", "System Controller" } ,
	{ 0x1106, 0x0605, "VT8605", "PM133 System Controller" } ,
	{ 0x1106, 0x0680, "VT82C680", "Apollo P6" } ,
	{ 0x1106, 0x0686, "VT82C686/686A/686B", "PCI-to-ISA bridge" } ,
	{ 0x1106, 0x0691, "VT82C691/693A/694X", "Host Bridge" } ,
	{ 0x1106, 0x0692, "", "North Bridge" } ,
	{ 0x1106, 0x0693, "VT82C693", "Host Bridge" } ,
	{ 0x1106, 0x0926, "VT86C926", "Amazon PCI Ethernet Controller" } ,
	{ 0x1106, 0x1000, "82C570MV", "Host Bridge" } ,
	{ 0x1106, 0x1106, "82C570MV", "ISA Bridge w/IDE" } ,
	{ 0x1106, 0x1571, "VT82C416", "IDE Controller" } ,
	{ 0x1106, 0x1595, "VT82C595/97", "Host Bridge" } ,
	{ 0x1106, 0x3038, "VT83C572", "PCI USB Controller" } ,
	{ 0x1106, 0x3040, "VT83C572", "Power Management Controller" } ,
	{ 0x1106, 0x3043, "VT86C100A", "Rhine 10/100 Ethernet Adapter" } ,
	{ 0x1106, 0x3044, "VT6306", "VIA Fire II 1394a OHCI Link Layer Ctrlr" } ,
	{ 0x1106, 0x3050, "VT82C596/596A/596", "Power Management and SMBus Controller" } ,
	{ 0x1106, 0x3051, "", "Power Management Controller" } ,
	{ 0x1106, 0x3053, "VT6105M", "Rhine III Management Adapter" } ,
	{ 0x1106, 0x3057, "VT82C686A", "ACPI Power Management Controller" } ,
	{ 0x1106, 0x3058, "VT82C686A/B", "AC97 Audio Codec" } ,
	{ 0x1106, 0x3059, "VT8233/33A", "AC97 Enhanced Audio Controller" } ,
	{ 0x1106, 0x3065, "VT6102", "Rhine II PCI Fast Ethernet Controller" } ,
	{ 0x1106, 0x3068, "VT82C686/686A/686B", "AC97 Modem Codec" } ,
	{ 0x1106, 0x3074, "VT8233", "PCI to ISA Bridge" } ,
	{ 0x1106, 0x3086, "VT82C686", "Power management" } ,
	{ 0x1106, 0x3091, "VT8633", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3099, "VT8366/66A/67", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3101, "VT8653", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3102, "VT8362", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3103, "VT8615", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3104, "VT6202", "USB 2.0 Enhanced Host Controller" } ,
	{ 0x1106, 0x3106, "VT6105M/LOM", "Rhine III PCI Fast Ethernet Controller" } ,
	{ 0x1106, 0x3109, "VT8233C", "PCI to ISA Bridge" } ,
	{ 0x1106, 0x3112, "VT8361", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3113, "", "PCI to PCI Bridge" } ,
	{ 0x1106, 0x3116, "VT8375", "CPU-to-PCI Bridge" } ,
	{ 0x1106, 0x3123, "VT8623", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3128, "VT8753", "CPU-to-PCI Bridge" } ,
	{ 0x1106, 0x3133, "VT3133", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3147, "VT8233", "PCI to ISA Bridge" } ,
	{ 0x1106, 0x3148, "VT8751", "CPU-to-PCI Bridge" } ,
	{ 0x1106, 0x3156, "VT8372", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3158, "", "CPU-to-PCI Bridge" } ,
	{ 0x1106, 0x3168, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3177, "VT8235", "PCI to ISA Bridge" } ,
	{ 0x1106, 0x3178, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3188, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3189, "VT8377", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3198, "", "CPU-to-PCI Bridge" } ,
	{ 0x1106, 0x3202, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3204, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3205, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3208, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3209, "", "CPU to PCI Bridge" } ,
	{ 0x1106, 0x3213, "", "PCI to PCI Bridge" } ,
	{ 0x1106, 0x5030, "VT82C596", "ACPI Power Management Controller" } ,
	{ 0x1106, 0x6100, "VT86C100A", "Rhine II PCI Fast Ethernet Controller" } ,
	{ 0x1106, 0x8231, "VT8231", "PCI to ISA Bridge" } ,
	{ 0x1106, 0x8235, "VT8235", "Power Management Controller" } ,
	{ 0x1106, 0x8305, "VT8363A/65", "PCI to AGP Bridge" } ,
	{ 0x1106, 0x8391, "VT8363/71", "PCI to AGP Bridge" } ,
	{ 0x1106, 0x8501, "VT8501", "PCI to AGP Bridge" } ,
	{ 0x1106, 0x8596, "VT82C596", "PCI to AGP Bridge" } ,
	{ 0x1106, 0x8597, "VT82C597", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0x8598, "VT82C598MVP/694X", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0x8601, "VT82C601", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0x8602, "", "CPU to AGP Bridge" } ,
	{ 0x1106, 0x8605, "VT8605", "PCI-to-PCI Bridge(AGP)" } ,
	{ 0x1106, 0x8691, "VT82C691/693A/694X", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0x8693, "VT82C693", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0x9398, "VT8601", "2D/3D Graphics Accelerator" } ,
	{ 0x1106, 0xB091, "VT8633", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB099, "VT8366/A", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB101, "VT8653", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB102, "VT8362", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB103, "VT8615", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB112, "VT8361", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB113, "", "I/O APIC" } ,
	{ 0x1106, 0xB115, "VT8363/65", "CPU to AGP Controller" } ,
	{ 0x1106, 0xB116, "VT8375", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB133, "vt686b", "CPU to AGP Controller" } ,
	{ 0x1106, 0xB148, "VT8751", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB156, "VT8372", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB158, "", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x1106, 0xB168, "", "PCI-to-PCI Bridge (AGP 2.0/3.0)" } ,
	{ 0x1106, 0xB188, "", "PCI-to-PCI Bridge (AGP 2.0/3.0)" } ,
	{ 0x1106, 0xB198, "", "PCI-to-PCI Bridge (AGP 2.0/3.0)" } ,
	{ 0x1106, 0xB213, "", "I/O APIC" } ,
	{ 0x1106, 0xD213, "", "PCI to PCI Bridge" } ,
	{ 0x1107, 0x8576, "", "PCI Host Bridge" } ,
	{ 0x1108, 0x0100, "p1690plus-AA", "Token Ring Adapter" } ,
	{ 0x1108, 0x0101, "p1690plus-AB", "2-Port Token Ring Adapter" } ,
	{ 0x1108, 0x0105, "P1690Plus", "Token Ring Adapter" } ,
	{ 0x1108, 0x0108, "P1690Plus", "Token Ring Adapter" } ,
	{ 0x1108, 0x0138, "P1690Plus", "Token Ring Adapter" } ,
	{ 0x1108, 0x0139, "P1690Plus", "Token Ring Adapter" } ,
	{ 0x1108, 0x013C, "P1690Plus", "Token Ring Adapter" } ,
	{ 0x1108, 0x013D, "P1690Plus", "Token Ring Adapter" } ,
	{ 0x1109, 0x1400, "EM110TX", "EX110TX PCI Fast Ethernet Adapter" } ,
	{ 0x110A, 0x0002, "Piranha", "PCI-EIDE Adapter (2-port)" } ,
	{ 0x110A, 0x0005, "SAB 82C206-N", "Tulip-Ctrlr, Power-Mgmt, Switch Extender" } ,
	{ 0x110A, 0x0006, "PINC", "" } ,
	{ 0x110A, 0x0015, "", "Multiprocessor Interrupt Ctrlr (MINT)" } ,
	{ 0x110A, 0x0017, "20534", "PCI-WAN Adapter (SiemensCard PWAN)" } ,
	{ 0x110A, 0x001D, "Copernicus", "Management Controller" } ,
	{ 0x110A, 0x007b, "", "" } ,
	{ 0x110A, 0x113C, "FPGA-CPTR", "Hardware Tracer for CP113C / CP113D" } ,
	{ 0x110A, 0x113E, "FPGA-CPTRE", "Hardware Tracer for CP113E" } ,
	{ 0x110A, 0x2101, "PEB 20321", "MUNICH32X Multichannel NIC for HDLC" } ,
	{ 0x110A, 0x2102, "PEB/PEF 20534", "DSCC4 Multiprotocol HDLC Controller" } ,
	{ 0x110A, 0x2103, "PEB 20324", "MUNICH128X NIC for HDLC + extensions" } ,
	{ 0x110A, 0x2104, "PSB 4600/4610", "PCI I/F for Telephony/Data Apps (PITA)" } ,
	{ 0x110A, 0x2106, "PEB 20256 E", "MUNICH256 (NIC HDLC/PPP w/256 channels)" } ,
	{ 0x110A, 0x2108, "PEB 20256M E", "MUNICH256FM Multichnl NIC for HDLC/PPP" } ,
	{ 0x110A, 0x3160, "MCCA", "Pentium-PCI Host Bridge Core ASIC" } ,
	{ 0x110A, 0x4942, "FPGA-IBTR", "I-Bus Tracer for MBD" } ,
	{ 0x110A, 0x6120, "SZB6120", "Multimedia Adapter" } ,
	{ 0x110B, 0x0001, "Mpact", "Media Processor" } ,
	{ 0x110B, 0x0002, "GM90C7110VX", "MPACT DVD decoder." } ,
	{ 0x1110, 0x6037, "Firepower", "Powerized SMP I/O ASIC" } ,
	{ 0x1110, 0x6073, "Firepower", "Powerized SMP I/O ASIC" } ,
	{ 0x1112, 0x2200, "2200", "FDDI adapter" } ,
	{ 0x1112, 0x2300, "2300", "Fast Ethernet adapter" } ,
	{ 0x1112, 0X2340, "2340", "4 Port 10/100 UTP Fast Ethernet Adapter" } ,
	{ 0x1112, 0x2400, "2400", "ATM adapter" } ,
	{ 0x1113, 0x1211, "EN-1207D", "Fast Ethernet Adapter" } ,
	{ 0x1113, 0x1216, "EN5251", "Ethernet Controller" } ,
	{ 0x1113, 0x1217, "EN-1217", "Ethernet Adapter" } ,
	{ 0x1113, 0x5105, "EN-1660", "" } ,
	{ 0x1113, 0x9211, "EN-1207D", "Fast Ethernet Adapter" } ,
	{ 0x1116, 0x0022, "DT3001", "" } ,
	{ 0x1116, 0x0023, "DT3002", "" } ,
	{ 0x1116, 0x0024, "DT3003", "" } ,
	{ 0x1116, 0x0025, "DT3004", "" } ,
	{ 0x1116, 0x0026, "Dt3005", "" } ,
	{ 0x1116, 0x0027, "DT3001-PGL", "" } ,
	{ 0x1116, 0x0028, "DT3003-PGL", "" } ,
	{ 0x1117, 0x9500, "", "max-lc SVGA card" } ,
	{ 0x1117, 0x9501, "", "MaxPCI image processing board" } ,
	{ 0x1119, 0x0000, "GDT6000/6020/6050", "PCI SCSI RAID Controller" } ,
	{ 0x1119, 0x0001, "GDT6000/6010", "PCI 1-channel SCSI RAID Controller" } ,
	{ 0x1119, 0x0002, "GDT6110/6510", "PCI 1-channel SCSI RAID Controller" } ,
	{ 0x1119, 0x0003, "GDT6120/6520", "PCI 2-channel SCSI RAID Controller" } ,
	{ 0x1119, 0x0004, "GDT6530", "PCI 3-channel SCSI RAID Controller" } ,
	{ 0x1119, 0x0005, "GDT6550", "PCI 5-channel SCSI RAID Controller" } ,
	{ 0x1119, 0x0006, "GDT6117/6517", "Wide Ultra SCSI Controller" } ,
	{ 0x1119, 0x0007, "GDT6127/6527", "Wide Ultra SCSI Controller" } ,
	{ 0x1119, 0x0008, "GDT6537", "Wide Ultra SCSI Controller" } ,
	{ 0x1119, 0x0009, "GDT6557/6557-ECC", "Wide Ultra SCSI Controller" } ,
	{ 0x1119, 0x000A, "GDT6115/6515", "Ultra SCSI Controller" } ,
	{ 0x1119, 0x000B, "GDT6125/6525", "Wide SCSI Controller" } ,
	{ 0x1119, 0x000C, "GDT6535", "Wide SCSI Controller" } ,
	{ 0x1119, 0x000D, "GDT6555/6555-ECC", "Wide SCSI Controller" } ,
	{ 0x1119, 0x0100, "GDT6117RP/6517RP", "2 Channel Wide Ultra SCSI" } ,
	{ 0x1119, 0x0101, "GDT6127RP/6527RP", "Wide Ultra SCSI HBA" } ,
	{ 0x1119, 0x0102, "GDT6537RP", "Wide Ultra SCSI HBA" } ,
	{ 0x1119, 0x0103, "GDT6557RP", "Wide Ultra SCSI HBA" } ,
	{ 0x1119, 0x0104, "GDT6111RP/6511RP", "Ultra SCSI HBA" } ,
	{ 0x1119, 0x0105, "GDT6121RP/6521RP", "Ultra SCSI HBA" } ,
	{ 0x1119, 0x0110, "GDT6117RD/6517RD", "Wide Ultra SCSI HBA" } ,
	{ 0x1119, 0x0111, "GDT6127RD/6527RD", "Wide Ultra SCSI HBA" } ,
	{ 0x1119, 0x0112, "GDT6537RD", "Wide Ultra SCSI HBA" } ,
	{ 0x1119, 0x0113, "GDT6557RD", "Wide Ultra SCSI HBA" } ,
	{ 0x1119, 0x0114, "GDT6111RD/6511RD", "Ultra SCSI HBA" } ,
	{ 0x1119, 0x0115, "GDT6127RD/6527RD", "Ultra SCSI HBA" } ,
	{ 0x1119, 0x0118, "GDT6x18RD", "Wide Ultra2 SCSI HBA" } ,
	{ 0x1119, 0x0119, "GDT6x28RD", "Wide Ultra2 SCSI HBA" } ,
	{ 0x1119, 0x011A, "GDT6538RD/6638RD", "Wide Ultra2 SCSI HBA" } ,
	{ 0x1119, 0x011B, "GDT6558RD/6658RD", "Wide Ultra2 SCSI HBA" } ,
	{ 0x1119, 0x0120, "GDT6117RP2/6517RP2", "" } ,
	{ 0x1119, 0x0121, "GDT6127RP2/6527RP2", "" } ,
	{ 0x1119, 0x0122, "GDT6537RP2", "" } ,
	{ 0x1119, 0x0123, "GDT6557RP2", "" } ,
	{ 0x1119, 0x0124, "GDT6111RP2/6511RP2", "" } ,
	{ 0x1119, 0x0125, "GDT6127RP2/6527RP2", "" } ,
	{ 0x1119, 0x0136, "GDT 6x13RS", "" } ,
	{ 0x1119, 0x0137, "GDT 6x23RS", "Disk Array Controller" } ,
	{ 0x1119, 0x0138, "GDT 6x18RS", "" } ,
	{ 0x1119, 0x0139, "GDT 6x28RS", "" } ,
	{ 0x1119, 0x013A, "GDT 6x38RS", "" } ,
	{ 0x1119, 0x013B, "GDT 6x58RS", "" } ,
	{ 0x1119, 0x013C, "GDT 6x33RS", "" } ,
	{ 0x1119, 0x013D, "GDT 6x43RS", "" } ,
	{ 0x1119, 0x013E, "GDT 6x53RS", "" } ,
	{ 0x1119, 0x013F, "GDT 6x63RS", "" } ,
	{ 0x1119, 0x0166, "GDT 7x13RN", "" } ,
	{ 0x1119, 0x0167, "GDT 7x23RN", "" } ,
	{ 0x1119, 0x0168, "GDT7x18RN", "64-bit PCI Wide Untra2 SCSI HBA" } ,
	{ 0x1119, 0x0169, "GDT7x28RN", "64-bit PCI Wide Ultra2 SCSI HBA" } ,
	{ 0x1119, 0x016A, "GDT7538RN/7638RN", "64-bit PCI Wide Ultra2 SCSI HBA" } ,
	{ 0x1119, 0x016B, "GDT7558RN/7658RN", "64-bit PCI Wide Ultra2 SCSI HBA" } ,
	{ 0x1119, 0x016C, "GDT 7x33RN", "" } ,
	{ 0x1119, 0x016D, "GDT 7x43RN", "" } ,
	{ 0x1119, 0x016E, "GDT 7x53RN", "" } ,
	{ 0x1119, 0x016F, "GDT 7x63RN", "" } ,
	{ 0x1119, 0x01D6, "GDT 4x13RZ", "" } ,
	{ 0x1119, 0x01D7, "GDT 4x23RZ", "" } ,
	{ 0x1119, 0x01F6, "GDT 8x13RZ", "" } ,
	{ 0x1119, 0x01F7, "GDT 8x23RZ", "" } ,
	{ 0x1119, 0x01FC, "GDT 8x33RZ", "" } ,
	{ 0x1119, 0x01FD, "GDT 8x43RZ", "" } ,
	{ 0x1119, 0x01FE, "GDT 8x53RZ", "" } ,
	{ 0x1119, 0x01FF, "GDT 8x63RZ", "" } ,
	{ 0x1119, 0x0210, "GDT6519RD/6619RD", "Fibre Channel HBA" } ,
	{ 0x1119, 0x0211, "GDT6529RD/6629RD", "Fibre Channel HBA" } ,
	{ 0x1119, 0x0260, "GDT7519RN/7619RN", "64-bit PCI Fibre Channel HBA" } ,
	{ 0x1119, 0x0261, "GDT7529RN/7629RN", "64-bit PCI Fibre Channel HBA" } ,
	{ 0x1119, 0x0300, "GDT Rx", "" } ,
	{ 0x111A, 0x0000, "155P-MF1", "" } ,
	{ 0x111A, 0x0002, "166P-MF1", "" } ,
	{ 0x111A, 0x0003, "ENI-25P", "ATM Adapter" } ,
	{ 0x111a, 0x0005, "ENI-30x0", "SpeedStream ADSL Adapter" } ,
	{ 0x111C, 0x0001, "", "Powerbus Bridge" } ,
	{ 0x111D, 0x0001, "IDT77201/211", "NICStAR ATM Adapter" } ,
	{ 0x111D, 0x0003, "IDT77222/252", "MICRO ABR SAR PCI ATM Controller" } ,
	{ 0x111D, 0x0004, "IDT77V252", "MICRO ABR SAR PCI ATM Controller" } ,
	{ 0x111F, 0x4A47, "Precision MX", "Video engine interface" } ,
	{ 0x111F, 0x5243, "", "Frame Capture Bus Interface" } ,
	{ 0x1127, 0x0200, "FireRunner PCA-200", "ATM" } ,
	{ 0x1127, 0x0210, "PCA-200PC", "ATM" } ,
	{ 0x1127, 0x0250, "", "ATM" } ,
	{ 0x1127, 0x0300, "PCA-200E", "ATM adapter" } ,
	{ 0x1127, 0x0310, "", "ATM" } ,
	{ 0x1127, 0x0400, "ForeRunner HE", "ATM Adapter" } ,
	{ 0x112E, 0x0000, "", "EIDE/hdd and IDE/cd-rom Ctrlr" } ,
	{ 0x112E, 0x000B, "", "EIDE/hdd and IDE/cd-rom Ctrlr" } ,
	{ 0x112F, 0x0000, "ICPCI", "" } ,
	{ 0x112F, 0x0001, "", "video frame grabber/processor" } ,
	{ 0x112F, 0x0007, "?", "PCVisionPlus Image Capture Device" } ,
	{ 0x1131, 0x1201, "PTD3000", "VPN IPSEC coprocessor" } ,
	{ 0x1131, 0x1234, "", "EHCI USB 2.0 Controller" } ,
	{ 0x1131, 0x1301, "PTD3210", "SSL Accelerator" } ,
	{ 0x1131, 0x1562, "ISP1561", "EHCI USB 2.0 Controller" } ,
	{ 0x1131, 0x2780, "TDA2780AQ", "TV deflection controller" } ,
	{ 0x1131, 0x3400, "UCB1500", "Modem" } ,
	{ 0x1131, 0x3401, "UCB1500", "Multimedia Audio Device" } ,
	{ 0x1131, 0x5400, "TriMedia TM1000/1100", "Multimedia processor" } ,
	{ 0x1131, 0x5402, "TriMedia TM-1300", "Media Processor" } ,
	{ 0x1131, 0x7130, "SAA7130HL", "Multi Media Capture Device" } ,
	{ 0x1131, 0x7133, "SAA7135HL", "Multi Media Capture Device" } ,
	{ 0x1131, 0x7134, "SAA7134HL", "Multi Media Capture Device" } ,
	{ 0x1131, 0x7145, "SAA7145", "Multimedia PCI Bridge" } ,
	{ 0x1131, 0x7146, "SAA7146", "Multi Media Bridge Scaler" } ,
	{ 0x1131, 0x9730, "SAA9730", "Ethernet controller" } ,
	{ 0x1133, 0x7711, "EiconCard C91", "" } ,
	{ 0x1133, 0x7901, "EiconCard S90", "" } ,
	{ 0x1133, 0x7902, "", "" } ,
	{ 0x1133, 0x7911, "", "" } ,
	{ 0x1133, 0x7912, "", "" } ,
	{ 0x1133, 0x7941, "", "" } ,
	{ 0x1133, 0x7942, "", "" } ,
	{ 0x1133, 0x7943, "", "EiconCard S94" } ,
	{ 0x1133, 0x7944, "", "EiconCard S94" } ,
	{ 0x1133, 0xB921, "", "" } ,
	{ 0x1133, 0xB922, "", "" } ,
	{ 0x1133, 0xB923, "", "EiconCard P92" } ,
	{ 0x1133, 0xE001, "DIVA Pro 2.0 S/T", "" } ,
	{ 0x1133, 0xE002, "DIVA 2.0 S/T", "" } ,
	{ 0x1133, 0xE003, "DIVA Pro 2.0 U", "" } ,
	{ 0x1133, 0xE004, "DIVA 2.0 U", "" } ,
	{ 0x1133, 0xE005, "DIVA 2.01 S/T", "Eicon ISDN card using Siemens IPAC chip" } ,
	{ 0x1133, 0xE00B, "DIVA ISDN 2.02 PCI", "Eicon ISDN card using Infineon chip" } ,
	{ 0x1133, 0xE010, "Maestra", "DIVA Server BRI-2M" } ,
	{ 0x1133, 0xE012, "MaestraQ", "DIVA Server BRI-8M" } ,
	{ 0x1133, 0xE013, "MaestraQ-U", "DIVA Server 4BRI/PCI" } ,
	{ 0x1133, 0xE014, "MaestraP", "DIVA Server PRI-30M" } ,
	{ 0x1133, 0xE015, "", "Diva Server PRI-30M PCI v.2" } ,
	{ 0x1133, 0xE018, "", "DIVA Server BRI-2M/-2F" } ,
	{ 0x1134, 0x0001, "", "Raceway Bridge" } ,
	{ 0x1135, 0x0001, "", "Printer Cntrlr" } ,
	{ 0x1138, 0x8905, "8905", "STD 32 Bridge" } ,
	{ 0x113C, 0x0000, "PCI9060", "i960 Bridge" } ,
	{ 0x113C, 0x0001, "PCI9060", "i960 Bridge / Evaluation Platform" } ,
	{ 0x113C, 0x0911, "PCI911", "i960Jx I/O Controller" } ,
	{ 0x113C, 0x0912, "PCI912", "i960Cx I/O Controller" } ,
	{ 0x113C, 0x0913, "PCI913", "i960Hx I/O Controller" } ,
	{ 0x113C, 0x0914, "PCI914", "I/O Controller with secondary PCI bus" } ,
	{ 0x113F, 0x0808, "SST-64P", "Adapter" } ,
	{ 0x113F, 0x1010, "SST-128P", "Adapter" } ,
	{ 0x113F, 0x80C0, "", "" } ,
	{ 0x113F, 0x80C4, "", "" } ,
	{ 0x113F, 0x80C8, "", "" } ,
	{ 0x113F, 0x8888, "", "" } ,
	{ 0x113F, 0x9090, "", "" } ,
	{ 0x1141, 0x0001, "", "EIDE/ATAPI super adapter" } ,
	{ 0x1142, 0x3210, "ProMotion 3210", "VGA/AVI Playback Accelerator" } ,
	{ 0x1142, 0x6410, "6410 6422", "GUI Accelerator" } ,
	{ 0x1142, 0x6412, "", "GUI Accelerator" } ,
	{ 0x1142, 0x6420, "", "GUI Accelerator" } ,
	{ 0x1142, 0x6422, "Provideo 6422", "ProMotion-6422" } ,
	{ 0x1142, 0x6424, "ProVideo 6424", "ProMotion AT24 GUI Accelerator" } ,
	{ 0x1142, 0x6425, "ProMotion AT25", "bbu67b 9811" } ,
	{ 0x1142, 0x6426, "", "GUI Accelerator" } ,
	{ 0x1142, 0x643D, "AT25", "ProMotion-AT3D" } ,
	{ 0x1144, 0x0001, "", "Noservo Cntrlr" } ,
	{ 0x1145, 0xF020, "", "CardBus ATAPI Host Adapter" } ,
	{ 0x1148, 0x4000, "SK-NET", "FDDI adapter" } ,
	{ 0x1148, 0x4200, "", "Token Ring Adapter" } ,
	{ 0x1148, 0x4300, "SK-984x", "SK-NET Gigabit Ethernet Adapter" } ,
	{ 0x114A, 0x5579, "VMIPCI-5579", "Reflective Memory Card" } ,
	{ 0x114A, 0x5588, "VMICPCI5588", "VMICPCI5588 Reflective Memory Card" } ,
	{ 0x114A, 0x6504, "", "Timer/SRAM FPGA" } ,
	{ 0x114A, 0x7587, "VMIVME-7587", "" } ,
	{ 0x114F, 0x0002, "AccelePort EPC", "" } ,
	{ 0x114F, 0x0003, "RightSwitch SE-6", "" } ,
	{ 0x114F, 0x0004, "AccelePort Xem", "" } ,
	{ 0x114F, 0x0005, "AccelePort Xr", "" } ,
	{ 0x114F, 0x0006, "AccelePort C/X", "" } ,
	{ 0x114F, 0x0007, "DataFire PCI 1 S/T", "" } ,
	{ 0x114F, 0x0009, "AccelePort Xr/J", "" } ,
	{ 0x114F, 0x000A, "AccelePort EPC/J", "" } ,
	{ 0x114F, 0x000C, "DataFirePRIme T1", "" } ,
	{ 0x114F, 0x000D, "SyncPort", "X.25/FR 2-port" } ,
	{ 0x114F, 0x0011, "AccelePort8r EIA-232", "" } ,
	{ 0x114F, 0x0012, "AccelePort8r EIA-422", "" } ,
	{ 0x114F, 0x0013, "AccelePort Xr", "" } ,
	{ 0x114F, 0x0014, "AccelePort8r EIA-422", "" } ,
	{ 0x114F, 0x0015, "AccelePort Xem", "" } ,
	{ 0x114F, 0x0016, "AccelePort EPC/X", "" } ,
	{ 0x114F, 0x0017, "AccelePort C/X", "" } ,
	{ 0x114F, 0x0019, "DataFire PCI 1 U", "" } ,
	{ 0x114F, 0x001A, "DataFirePRIme E1", "" } ,
	{ 0x114F, 0x001B, "AccelePort C/X (IBM)", "" } ,
	{ 0x114F, 0x001D, "DataFire RAS", "T1/E1/PRI" } ,
	{ 0x114F, 0x001F, "", "ClydeNonCsu6034" } ,
	{ 0x114F, 0x0020, "", "ClydeNonCsu6032" } ,
	{ 0x114F, 0x0021, "", "ClydeNonCsu4" } ,
	{ 0x114F, 0x0022, "", "ClydeNonCsu2" } ,
	{ 0x114F, 0x0023, "AccelePort RAS", "" } ,
	{ 0x114F, 0x0024, "DataFire RAS B4 ST/U", "" } ,
	{ 0x114F, 0x0026, "AccelePort 4r 920", "" } ,
	{ 0x114F, 0x0027, "AccelePort 8r 920", "" } ,
	{ 0x114F, 0x0029, "DigiClassic PCI", "" } ,
	{ 0x114F, 0x0034, "AccelePort 2r 920", "" } ,
	{ 0x114F, 0x0035, "DataFire DSP", "T1/E1/PRI, Compact PCI" } ,
	{ 0x114F, 0x0040, "AccelePort Xp", "" } ,
	{ 0x114F, 0x0042, "AccelePort 2p PCI", "" } ,
	{ 0x114F, 0x0070, "DataFire Micro V", "" } ,
	{ 0x114F, 0x0071, "DataFire Micro V", "" } ,
	{ 0x114F, 0x0072, "DataFire Micro V", "" } ,
	{ 0x114F, 0x0073, "DataFire Micro V", "" } ,
	{ 0x114F, 0x6001, "Avanstar", "" } ,
	{ 0x1155, 0x0810, "", "486 CPU/PCI Bridge" } ,
	{ 0x1155, 0x0922, "", "Pentium CPU/PCI Bridge" } ,
	{ 0x1155, 0x0926, "", "PCI/ISA Bridge" } ,
	{ 0x1158, 0x3011, "", "Tokenet/vg 1001/10m anylan" } ,
	{ 0x1158, 0x9050, "", "Lanfleet/Truevalue" } ,
	{ 0x1158, 0x9051, "", "Lanfleet/Truevalue" } ,
	{ 0x1159, 0x0001, "MV1000", "" } ,
	{ 0x1159, 0x0002, "MV-1500", "Frame Grabber" } ,
	{ 0x115D, 0x0003, "", "CardBus Ethernet 10/100" } ,
	{ 0x115D, 0x0005, "", "CardBus Ethernet 10/100" } ,
	{ 0x115D, 0x0007, "", "CardBus Ethernet 10/100" } ,
	{ 0x115D, 0x000B, "", "CardBus Ethernet 10/100" } ,
	{ 0x115D, 0x000C, "MPCI 3A56GSP-100 PA", "Mini-PCI V.90 56k Modem" } ,
	{ 0x115D, 0x000F, "", "CardBus Ethernet 10/100" } ,
	{ 0x115D, 0x002b, "", "Winmodem built into NEC Versa VXi" } ,
	{ 0x115D, 0x0076, "", "Xircom MPCI3B-56G (Lucent SCORPIO) Soft" } ,
	{ 0x115D, 0x00d3, "", "Xircom MPCI Modem 56" } ,
	{ 0x115D, 0x00D4, "MPCI", "Modem 56k" } ,
	{ 0x115D, 0x0101, "", "CardBus 56k Modem" } ,
	{ 0x115D, 0x0103, "", "CardBus Ehternet + 56k Modem" } ,
	{ 0x115d, 0x1181, "", "" } ,
	{ 0x1161, 0x0001, "", "Host Bridge" } ,
	{ 0x1163, 0x0001, "Verite 1000", "3D Blaster" } ,
	{ 0x1163, 0x2000, "Verite 2x00", "" } ,
	{ 0x1165, 0x0001, "", "Motion JPEG rec/play w/audio" } ,
	{ 0x1166, 0x0005, "NB6536 (CNB20-LE)", "PCI to PCI Bridge, bus/dev/func 0/0/1" } ,
	{ 0x1166, 0x0006, "NB6536 (CNB20-HE)", "Host Bridge, function 2 and function 3" } ,
	{ 0x1166, 0x0007, "NB6635 (CNB20-LE/HE)", "CPU to PCI Bridge" } ,
	{ 0x1166, 0x0008, "NB6536 (CNB20-HE)", "Hostbridge & MCH, bus/dev/func 0/0/0" } ,
	{ 0x1166, 0x0009, "NB6536 (CNB20-LE)", "AGP interface" } ,
	{ 0x1166, 0x0010, "CIOB30", "" } ,
	{ 0x1166, 0x0011, "CMIC-HE", "" } ,
	{ 0x1166, 0x0012, "CMIC-LE", "" } ,
	{ 0x1166, 0x0013, "CNB20-HE", "Hostbridge and MCH" } ,
	{ 0x1166, 0x0014, "CNB20-HE", "Host Bridge" } ,
	{ 0x1166, 0x0015, "CMIC-GC", "Hostbridge and MCH" } ,
	{ 0x1166, 0x0016, "CMIC-GC", "Host Bridge" } ,
	{ 0x1166, 0x0017, "CMIC-SL", "" } ,
	{ 0x1166, 0x0101, "CIOB-X2", "" } ,
	{ 0x1166, 0x0200, "OSB4", "PCI to ISA Bridge" } ,
	{ 0x1166, 0x0201, "CSB5", "ISA bridge" } ,
	{ 0x1166, 0x0211, "OSB4", "EIDE Controller" } ,
	{ 0x1166, 0x0212, "CSB5", "IDE interface" } ,
	{ 0x1166, 0x0220, "OSB4", "OpenHCI Compliant USB Controller" } ,
	{ 0x1166, 0x0225, "CSB5", "PCI Bridge" } ,
	{ 0x1166, 0x0230, "", "ISA bridge" } ,
	{ 0x1169, 0x2001, "Ql5032-33APQ208C", "PCI to C-DAC RTU bus interface FPGA" } ,
	{ 0x116A, 0x6100, "", "Bus/Tag Channel" } ,
	{ 0x116A, 0x6800, "", "Escon Channel" } ,
	{ 0x116A, 0x7100, "", "Bus/Tag Channel" } ,
	{ 0x116A, 0x7800, "", "Escon Channel" } ,
	{ 0x1172, 0x0001, "", "" } ,
	{ 0x1178, 0xAFA1, "", "Fast Ethernet" } ,
	{ 0x1179, 0x0102, "", "Extended PCI IDE Controller" } ,
	{ 0x1179, 0x0103, "", "Extended PCI IDE Controller Type-B" } ,
	{ 0x1179, 0x0404, "", "" } ,
	{ 0x1179, 0x0406, "Tecra", "Video Capture device" } ,
	{ 0x1179, 0x0407, "", "" } ,
	{ 0x1179, 0x0601, "", "Toshiba CPU to PCI bridge" } ,
	{ 0x1179, 0x0602, "", "PCI to ISA Bridge for Notebooks" } ,
	{ 0x1179, 0x0603, "ToPIC95", "PCI to CardBus Bridge for Notebooks" } ,
	{ 0x1179, 0x0604, "", "PCI to PCI Bridge for Notebooks" } ,
	{ 0x1179, 0x0605, "", "PCI to ISA Bridge for Notebooks" } ,
	{ 0x1179, 0x0606, "", "PCI to ISA Bridge for Notebooks" } ,
	{ 0x1179, 0x0609, "", "PCI to PCI Bridge for Notebooks" } ,
	{ 0x1179, 0x060A, "ToPIC95B", "Toshiba ToPIC95 CardBus Controller" } ,
	{ 0x1179, 0x060F, "ToPIC97", "CardBus Controller" } ,
	{ 0x1179, 0x0611, "", "PCI to ISA Bridge" } ,
	{ 0x1179, 0x0617, "ToPIC100", "PCI to CardBus Bridge with ZV support" } ,
	{ 0x1179, 0x0618, "", "CPU to PCI and PCI to ISA Bridge" } ,
	{ 0x1179, 0x0701, "", "PCI Communication Device" } ,
	{ 0x1179, 0x0805, "", "SD Card Controller" } ,
	{ 0x1179, 0x0D01, "", "FIR Port Type-DO" } ,
	{ 0x1179, 0x13A8, "XR17C158/154/152", "Multi-channel PCI UART" } ,
	{ 0x117E, 0x0001, "", "Printer Host" } ,
	{ 0x1180, 0x0465, "RL5C465", "CardBus controller" } ,
	{ 0x1180, 0x0466, "RL5C466", "CardBus controller" } ,
	{ 0x1180, 0x0475, "RL5C475", "CardBus controller" } ,
	{ 0x1180, 0x0476, "RL5C476 II", "CardBus controller" } ,
	{ 0x1180, 0x0477, "RLc477", "CardBus Controller" } ,
	{ 0x1180, 0x0478, "RLc478", "CardBus Controller" } ,
	{ 0x1180, 0x0521, "R5C521", "1394 Host Controller" } ,
	{ 0x1180, 0X0551, "", "IEEE1394 Controller" } ,
	{ 0x1180, 0x0552, "RL5c552", "FireWire (IEEE1394) Controller. IBM A31p" } ,
	{ 0x1180, 0x0576, "", "SD-Card Interface" } ,
	{ 0x1185, 0x8929, "", "EIDE Controller" } ,
	{ 0x1186, 0x0100, "DC21041", "Ethernet Adapter" } ,
	{ 0x1186, 0x1002, "DFE-550TX", "Fast Ethernet Adapter" } ,
	{ 0x1186, 0x1100, "", "Fast Ethernet Adapter" } ,
	{ 0x1186, 0x1300, "DFE-530TX+", "Fast Ethernet Adapter" } ,
	{ 0x1186, 0x1340, "DFE-690TXD", "CardBus PC Card" } ,
	{ 0x1186, 0x1561, "DRP-32TXD", "CardBus PC Card" } ,
	{ 0x1186, 0x4000, "DL2000", "Gigabit Ethernet Adapter" } ,
	{ 0x1186, 0x4001, "DFE-650TX", "D Link Fast Ethernet PCMCIA Card" } ,
	{ 0x1189, 0x1592, "", "VL/PCI Bridge" } ,
	{ 0x118C, 0x0014, "PCIB", "C-bus II to PCI bus host bridge chip" } ,
	{ 0x118C, 0x1117, "MAC-94C201B3", "Corollary/Intel Memory Controller Chip" } ,
	{ 0x118D, 0x0001, "n/a", "Raptor-PCI framegrabber" } ,
	{ 0x118D, 0x0012, "Model 12", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0014, "Model 14", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0024, "Model 24", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0044, "Model 44", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0112, "Model 12", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0114, "Model 14", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0124, "Model 24", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0144, "Model 44", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0212, "Model 12", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0214, "Model 14", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0224, "Model 24", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0244, "Model 44", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0312, "Model 12", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0314, "Model 14", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0324, "Model 24", "Road Runner Frame Grabber" } ,
	{ 0x118D, 0x0344, "Model 44", "Road Runner Frame Grabber" } ,
	{ 0x1190, 0x2550, "TC-2550", "Single Chip Ultra (Wide) SCSI Processor" } ,
	{ 0x1190, 0xC721, "", "EIDE" } ,
	{ 0x1190, 0xC731, "TP-910/920/940", "PCI Ultra (Wide) SCSI Adapter" } ,
	{ 0x1191, 0x0001, "", "IDE Ctrlr" } ,
	{ 0x1191, 0x0002, "ATP850UF", "UltraDMA33 EIDE Controller (AEC6210UF)" } ,
	{ 0x1191, 0x0003, "", "SCSI-2 cache Cntrlr" } ,
	{ 0x1191, 0x0004, "ATP8400", "UltraDMA33 EIDE Controller" } ,
	{ 0x1191, 0x0005, "ATP850UF", "UltraDMA33 EIDE Controller (AEC6210UF)" } ,
	{ 0x1191, 0x0006, "ATP860A", "UltraDMA66 EDIE Controller (AEC6260)" } ,
	{ 0x1191, 0x0007, "ATP860R", "UltraDMA66 EIDE Controller (AEC6260)" } ,
	{ 0x1191, 0x0008, "ATP865", "" } ,
	{ 0x1191, 0x0009, "ATP865", "" } ,
	{ 0x1191, 0x8001, "ATP8600", "SCSI-2 RAID (cache?) Adapter (AEC6820U)" } ,
	{ 0x1191, 0x8002, "ATP850S", "SCSI-2 Host Adapter (AEC6710L/F)" } ,
	{ 0x1191, 0x8010, "ATP870", "Ultra Wide SCSI Controller" } ,
	{ 0x1191, 0x8020, "ATP870", "Ultra SCSI Controller" } ,
	{ 0x1191, 0x8030, "ATP870", "Ultra SCSI Controller" } ,
	{ 0x1191, 0x8040, "ATP870", "SCSI Controller" } ,
	{ 0x1191, 0x8050, "", "Ultra Wide SCSI Controller" } ,
	{ 0x1191, 0x8060, "AEC671x", "SCSI Host Adapter" } ,
	{ 0x1191, 0x8081, "AEC-67160", "PCI Ultra160 LVD/SE SCSI Adapter" } ,
	{ 0x1193, 0x0001, "ZN1221", "ATM adapter" } ,
	{ 0x1193, 0x0002, "ZN1225", "ATM adapter" } ,
	{ 0x1199, 0x0001, "", "IRMA 3270 PCI Adapter" } ,
	{ 0x1199, 0x0002, "", "Advanced ISCA PCI Adapter" } ,
	{ 0x1199, 0x0201, "", "SDLC PCI Adapter" } ,
	{ 0x119B, 0x1221, "82C092G", "PCI PCMCIA bridge" } ,
	{ 0x119E, 0x0001, "MB86697", "FireStream 155 ATM adapter" } ,
	{ 0x119E, 0x0003, "MB86695", "FireStream 50 ATM adapter" } ,
	{ 0x11A8, 0x7302, "", "NTX-8023-PCI 2MB Long Card" } ,
	{ 0x11A8, 0x7308, "", "NTX-8023-PCI 8MB Long Card" } ,
	{ 0x11A8, 0x7402, "", "NTX-8023-PCI 2MB Short Card" } ,
	{ 0x11A8, 0x7408, "", "NTX-8023-PCI 8MB Short Card" } ,
	{ 0x11A9, 0x4240, "AMCC S5933Q", "Intelligent Serial Card" } ,
	{ 0x11AB, 0x0146, "GT-64010/A", "System Ctrlr for R4xxx/5000 Family CPUs" } ,
	{ 0x11ab, 0x4620, "GT64120", "System Ctrlr for R5K & R7K w/64bit PCI" } ,
	{ 0x11AB, 0x4801, "GT-48001", "8 port switched ethernet ctrlr" } ,
	{ 0x11AB, 0x4809, "EV-48300", "Evaluation board for the GT-48300" } ,
	{ 0x11AB, 0x6320, "GT-64130/131", "System Controller for PowerPC Processors" } ,
	{ 0x11ab, 0x6430, "GT-64260A", "System Controller for PowerPC Processors" } ,
	{ 0x11AB, 0x9653, "GT-96100A", "Advanced Communication Controller" } ,
	{ 0x11AB, 0xF003, "GT-64010", "Primary Image Piranha Image Generator" } ,
	{ 0x11AB, 0xF004, "GT64120", "Primary Image Barracuda Image Generator" } ,
	{ 0x11ab, 0xF006, "GT64120A", "Primary Image Cruncher Geometry Acclrtr" } ,
	{ 0x11AD, 0x0002, "NGMC169B", "10/100 Ethernet (NetGear FA310TX)" } ,
	{ 0x11AD, 0xC115, "LC82C115", "PNIC II 10/100 PCI MAC/PHY" } ,
	{ 0x11AE, 0x4153, "", "Bridge Controller" } ,
	{ 0x11AE, 0x5842, "", "Bridge Controller" } ,
	{ 0x11B0, 0x0001, "V960PBC/PSC", "i960 Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0002, "V961PBC/PSC", "i960Jx Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0004, "V962PBC/PSC", "i960Cx/Hx Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0010, "V292PBC/PSC", "Am29K Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0021, "V363EPC", "i960Sx Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0022, "V363EPC", "i960Jx Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0024, "V363EPC", "i960Cx/Hx Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0030, "V363EPC", "Am29K Local Bus to PCI Bridge" } ,
	{ 0x11B0, 0x0100, "V320USC", "PCI System Ctrlr for 32-bit MIPS CPU" } ,
	{ 0x11B0, 0x0101, "V320USC", "PCI System Ctrlr for 32-bit MIPS CPU" } ,
	{ 0x11B0, 0x0102, "V320USC", "PCI System Ctrlr for Super-H SH3 CPU" } ,
	{ 0x11B0, 0x0103, "V320USC", "PCI System Ctrlr for Super-H SH4 CPU" } ,
	{ 0x11B0, 0x0200, "V370PDC", "High Performance PCI SDRAM Controller" } ,
	{ 0x11B0, 0x0292, "V292PBC", "Am29030/40 Bridge" } ,
	{ 0x11B0, 0x0500, "V340HPC", "PCI System Ctrlr for 64-bit MIPS CPU" } ,
	{ 0x11B0, 0x0960, "V96xPBC", "i960 Bridges for i960 Processors" } ,
	{ 0x11B0, 0xC960, "V96DPC", "i960 Dual PCI Bridge" } ,
	{ 0x11b3, 0x0001, "", "CHANNEL-IN (BT) Rev 1" } ,
	{ 0x11b3, 0x0002, "", "CHANNEL-OUT (BT) Rev 1" } ,
	{ 0x11b3, 0x0010, "", "CHANNEL-IN (ES)" } ,
	{ 0x11b3, 0x0100, "", "SYNC MAX PCI" } ,
	{ 0x11b3, 0x1001, "", "CHANNEL-IN (BT) Rev 2" } ,
	{ 0x11b3, 0x1002, "", "CHANNEL-OUT (BT) Rev 2" } ,
	{ 0x11B8, 0x0001, "Quad PeerMaster", "" } ,
	{ 0x11B9, 0xC0ED, "SSA Ctrlr", "" } ,
	{ 0x11BC, 0x0001, "NPI NuCard", "PCI FDDI" } ,
	{ 0x11C1, 0x0440, "LT Winmodem 56k", "Data+Fax+Voice+DSVD" } ,
	{ 0x11C1, 0x0441, "LT Winmodem 56k", "Data+Fax" } ,
	{ 0x11C1, 0x0442, "1646T00", "V.90 Lucent Modem" } ,
	{ 0x11C1, 0x0443, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0444, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0445, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0446, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0447, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0448, "LT Winmodem 56k", "" } ,
	{ 0x11C1, 0x0449, "LT Winmodem 56k", "" } ,
	{ 0x11C1, 0x044A, "LT Winmodem 56k", "" } ,
	{ 0x11C1, 0x044B, "LT Winmodem", "" } ,
	{ 0x11C1, 0x044C, "LT Winmodem", "" } ,
	{ 0x11C1, 0x044D, "LT Winmodem", "" } ,
	{ 0x11C1, 0x044E, "lucent 1646T00", "LT WinModem 56k Data+Fax" } ,
	{ 0x11C1, 0x044F, "90094-1", "LT V.90+DSL WildFire Modem" } ,
	{ 0x11C1, 0x0450, "1456VQH19R-1(INT)", "LT Winmodem 56K" } ,
	{ 0x11C1, 0x0451, "LT Winmodem", "LT WinModem 56k Data+Fax+Voice+DSVD" } ,
	{ 0x11C1, 0x0452, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0453, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0454, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0455, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0456, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0457, "LT Winmodem", "" } ,
	{ 0x11C1, 0x0458, "1648C", "Mars 3 Mercury v.92 v.44" } ,
	{ 0x11C1, 0x0459, "LT Winmodem", "" } ,
	{ 0x11C1, 0x045A, "LT Winmodem", "" } ,
	{ 0x11c1, 0x045C, "", "LT Winmodem" } ,
	{ 0x11C1, 0x045D, "LT WinModem", "" } ,
	{ 0x11C1, 0x0461, "", "V90 Wildfire Modem" } ,
	{ 0x11C1, 0x0462, "", "56K.V90/ADSL Wildfire Modem" } ,
	{ 0x11C1, 0x0464, "Riptide", "Audio PCI Legacy Resources" } ,
	{ 0x11C1, 0x0480, "Venus Winmodem", "" } ,
	{ 0x11c1, 0x048C, "LU97", "AC-Link Soft Modem Chip Set - 56Kbps" } ,
	{ 0x11C1, 0x048F, "SV92P-T00", "" } ,
	{ 0x11c1, 0x5400, "Lucent OR3TP12 FPSC", "FPGA w embedded PCI core" } ,
	{ 0x11C1, 0x5801, "", "USB Open Host Controller" } ,
	{ 0x11C1, 0x5802, "USS-312", "2-port PCI-to-USB OpenHCI Host Ctrlr" } ,
	{ 0x11C1, 0x5803, "USS-344", "QuadraBus 4-port USB OpenHCI Host Ctrlr" } ,
	{ 0x11C1, 0x5805, "", "USB Advanced Host Controller" } ,
	{ 0x11C1, 0x5811, "FW322/323", "1394A PCI PHY/Link Open Host Ctrlr I/F" } ,
	{ 0x11C1, 0xAB20, "WaveLAN", "PCI Wireless LAN Adapter" } ,
	{ 0x11C6, 0x3001, "1", "VM-1200 Opto Unit Controller" } ,
	{ 0x11C8, 0x0658, "PSB 32", "32 bit , 33 Mhz PCI-SCI Bridge" } ,
	{ 0x11C8, 0xD665, "PSB64", "64 bit , 33 Mhz PCI-SCI Bridge" } ,
	{ 0x11C8, 0xD667, "PSB66", "64 bit , 66 Mhz PCI-SCI Bridge. (D33x)" } ,
	{ 0x11C9, 0x0010, "", "16-line serial port w/DMA" } ,
	{ 0x11C9, 0x0011, "", "4-line serial port w/DMA" } ,
	{ 0x11CB, 0x2000, "PCI-9050", "Target Interface" } ,
	{ 0x11CB, 0x4000, "SUPI-1", "XIO/SIO Host" } ,
	{ 0x11CB, 0x8000, "T225", "Bridge RIO Host" } ,
	{ 0x11D1, 0x01F7, "VxP524", "PCI Video Processor" } ,
	{ 0x11d4, 0x11d4, "ad1807js", "erl3227a-0,8" } ,
	{ 0x11D4, 0x1535, "ADSP-21535", "Blackfin DSP PCI Bus Interface" } ,
	{ 0x11D4, 0x1805, "62412-51", "erl3227a-0.8" } ,
	{ 0x11d4, 0x1807, "AD1807JS", "3Com winmodem chip" } ,
	{ 0x11D4, 0x1889, "AD1889", "Sound Chip" } ,
	{ 0x11D4, 0x2192, "ADSP-2192", "DSP Microcomputer (function #0)" } ,
	{ 0x11D4, 0x219A, "ADSP-2192", "DSP Microcomputer (function #1)" } ,
	{ 0x11D4, 0x219E, "ADSP-2192", "DSP Microcomputer (function #2)" } ,
	{ 0x11D4, 0x2F44, "ADSP-2141", "SafeNet Crypto Accelerator chip" } ,
	{ 0x11D5, 0x0115, "10115", "Greensheet" } ,
	{ 0x11D5, 0x0117, "10117", "Greensheet" } ,
	{ 0x11DB, 0x1234, "", "Dreamcast Broadband Adapter" } ,
	{ 0x11DE, 0x6057, "ZR36057/36067", "AV PCI Controller" } ,
	{ 0x11de, 0x6120, "ZR36120/5", "DVD Decoder" } ,
	{ 0x11F0, 0x4231, "", "" } ,
	{ 0x11F0, 0x4232, "FASTline UTP Quattro", "" } ,
	{ 0x11F0, 0x4233, "FASTline FO", "" } ,
	{ 0x11F0, 0x4234, "FASTline UTP", "" } ,
	{ 0x11F0, 0x4235, "FASTline-II UTP", "" } ,
	{ 0x11F0, 0x4236, "FASTline-II FO", "" } ,
	{ 0x11F0, 0x4731, "GIGAline", "" } ,
	{ 0x11F4, 0x2915, "2915", "" } ,
	{ 0x11F6, 0x0112, "", "ReadyLink ENET100-VG4" } ,
	{ 0x11F6, 0x0113, "", "FreedomLine 100" } ,
	{ 0x11F6, 0x1401, "832AE28030680", "ReadyLink RL2000 (Winbond W89C940)" } ,
	{ 0x11F6, 0x2011, "RL100-ATX", "10/100 Fast Ethernet Adapter" } ,
	{ 0x11F6, 0x2201, "", "ReadyLink 100TX (Winbond W89C840)" } ,
	{ 0x11F6, 0x9881, "RL100TX", "Fast Ethernet Adapter" } ,
	{ 0x11F8, 0x7364, "PM7364", "FREEDM-32 Frame Engine & Datalink Mgr" } ,
	{ 0x11F8, 0x7366, "PM7366", "FREEDM-8 Frame Engine & Datalink Manager" } ,
	{ 0x11F8, 0x7367, "PM7367", "FREEDM-32P32 Frame Engine & Datalink Mgr" } ,
	{ 0x11F8, 0x7375, "PM7375", "LASAR-155 ATM SAR" } ,
	{ 0x11F8, 0x7380, "PM7380", "FREEDM-32P672 Frm Engine & Datalink Mgr" } ,
	{ 0x11F8, 0x7382, "PM7382", "FREEDM-32P256 Frm Engine & Datalink Mgr" } ,
	{ 0x11F8, 0x7384, "PM7384", "FREEDM-84P672 Frm Engine & Datalink Mgr" } ,
	{ 0x11FE, 0x0001, "RocketPort", "" } ,
	{ 0x11FE, 0x0002, "RocketPort", "" } ,
	{ 0x11FE, 0x0003, "RocketPort", "" } ,
	{ 0x11FE, 0x0004, "RocketPort", "" } ,
	{ 0x11FE, 0x0005, "RocketPort", "" } ,
	{ 0x11FE, 0x0006, "RocketPort", "" } ,
	{ 0x11FE, 0x0007, "RocketPort", "" } ,
	{ 0x11FE, 0x0008, "RocketPort", "" } ,
	{ 0x11FE, 0x0009, "RocketPort", "" } ,
	{ 0x11FE, 0x000A, "RocketPort", "" } ,
	{ 0x11FE, 0x000B, "RocketPort", "" } ,
	{ 0x11FE, 0x000C, "RocketPort", "" } ,
	{ 0x11FE, 0x000D, "RocketPort", "" } ,
	{ 0x11FE, 0x8015, "RocketPort", "4-port UART 16954" } ,
	{ 0x1202, 0x0001, "NAIATMPCI", "PCI ATM Adapter" } ,
	{ 0x1208, 0x4853, "", "HS-Link Device" } ,
	{ 0x120E, 0x0100, "Cyclom-Y", "Multiport Serial Card" } ,
	{ 0x120E, 0x0101, "Cyclom-Y", "Multiport Serial Card" } ,
	{ 0x120E, 0x0102, "Cyclom-4Y", "Multiport Serial Card" } ,
	{ 0x120E, 0x0103, "Cyclom-4Y", "Multiport Serial Card" } ,
	{ 0x120E, 0x0104, "Cyclom-8Y", "Multiport Serial Card" } ,
	{ 0x120E, 0x0105, "Cyclom-8Y", "Multiport Serial Card" } ,
	{ 0x120E, 0x0200, "Cyclom-Z", "Intelligent Multiport Serial" } ,
	{ 0x120E, 0x0201, "Cyclom-Z", "Intelligent Serial Card" } ,
	{ 0x120E, 0x0300, "PC300 RX 2", "" } ,
	{ 0x120E, 0x0301, "PC300 RX 1", "" } ,
	{ 0x120E, 0x0302, "PC300 TE 2", "" } ,
	{ 0x120E, 0x0303, "PC300 TE 1", "" } ,
	{ 0x120F, 0x0001, "Roadrunner", "" } ,
	{ 0x1217, 0x6729, "OZ6729", "PCI to PCMCIA Bridge" } ,
	{ 0x1217, 0x673A, "OZ6730", "PCI to PCMCIA Bridge" } ,
	{ 0x1217, 0x6832, "OZ6832/3", "CardBus Controller" } ,
	{ 0x1217, 0x6836, "OZ6836/6860", "CardBus Controller" } ,
	{ 0x1217, 0x6872, "OZ6812", "CardBus Controller" } ,
	{ 0x1217, 0x6925, "OZ6922", "CardBus Controller" } ,
	{ 0x1217, 0x6933, "OZ6933", "CardBus Controller" } ,
	{ 0x1217, 0x6972, "OZ6912", "CardBus Controller" } ,
	{ 0x121A, 0x0001, "Voodoo", "Voodoo 3D Acceleration Chip" } ,
	{ 0x121A, 0x0002, "Voodoo2", "Voodoo 2 3D Accelerator" } ,
	{ 0x121A, 0x0003, "Voodoo Banshee", "Voodoo Banshee" } ,
	{ 0x121a, 0x0004, "", "Voodoo Banshee (Velocity 100)" } ,
	{ 0x121A, 0x0005, "Voodoo3", "All Voodoo3 chips, 3000" } ,
	{ 0x121A, 0x0007, "Voodoo4", "" } ,
	{ 0x121A, 0x0009, "Voodoo5", "" } ,
	{ 0x121A, 0x0010, "Rampage", "Rev.A AGPx4, 0.25�m, 200/2x200 core/RAM" } ,
	{ 0x121A, 0x0057, "Voodoo 3/3000", "Avenger" } ,
	{ 0x1220, 0x1220, "", "AMCC 5933 TMS320C80 DSP/Imaging Board" } ,
	{ 0x122D, 0x1206, "368DSP", "" } ,
	{ 0x122D, 0x4201, "MR2800W", "AMR 56K modem" } ,
	{ 0x122D, 0x50DC, "PCI168/3328", "Audio" } ,
	{ 0x122D, 0x80DA, "3328", "Audio" } ,
	{ 0x1236, 0x0000, "RealMagic64/GX", "SD6425" } ,
	{ 0x1236, 0x6401, "REALmagic64/GX", "GUI Accelerator" } ,
	{ 0x123D, 0x0010, "PCI-DV", "PCI-DV Digital Video Interface" } ,
	{ 0x123F, 0x00E4, "", "MPEG" } ,
	{ 0x123F, 0x6120, "", "DVD device" } ,
	{ 0x123F, 0x8120, "176", "E4?" } ,
	{ 0x123F, 0x8888, "", "Cinemaster C 3.0 DVD Decoder" } ,
	{ 0x1242, 0x1460, "JNIC-1460", "2-Gb/s Fibre Channel-PCI 64-bit 66 MHz" } ,
	{ 0x1242, 0x1560, "JNIC-1560", "Dual Channel 2 Gb/s Fibre Channel-PCI-X" } ,
	{ 0x1244, 0x0700, "B1", "ISDN controller" } ,
	{ 0x1244, 0x0800, "C4", "ISDN Controller" } ,
	{ 0x1244, 0x0A00, "A1", "ISDN Controller" } ,
	{ 0x1244, 0x0E00, "", "Fritz!PCI 2.0 ISDN Controller" } ,
	{ 0x1244, 0x1100, "C2", "ISDN Controller" } ,
	{ 0x1244, 0x1200, "T1", "ISDN Controller" } ,
	{ 0x124B, 0x0040, "cPCI-200", "Four Slot IndustryPack Carrier" } ,
	{ 0x124D, 0x0000, "EasyConnect 8/32", "" } ,
	{ 0x124D, 0x0002, "EasyConnect 8/64", "" } ,
	{ 0x124D, 0x0003, "EasyIO PCI", "" } ,
	{ 0x124F, 0x0041, "IFT-2000", "PCI RAID Controller" } ,
	{ 0x1250, 0x2898, "", "" } ,
	{ 0x1255, 0x1110, "MPEG Forge", "" } ,
	{ 0x1255, 0x1210, "MPEG Fusion", "" } ,
	{ 0x1255, 0x2110, "VideoPlex", "" } ,
	{ 0x1255, 0x2120, "VideoPlex CC", "" } ,
	{ 0x1255, 0x2130, "VideoQuest", "" } ,
	{ 0x1256, 0x4201, "PCI-2240i", "EIDE Adapter" } ,
	{ 0x1256, 0x4401, "PCI-2220i", "Dale EIDE Adapter" } ,
	{ 0x1256, 0x5201, "PCI-2000", "IntelliCache SCSI Adapter" } ,
	{ 0x1259, 0x2503, "Realtek 8139b", "" } ,
	{ 0x1259, 0x2560, "", "AT-2560 Fast Ethernet Adapter (i82557B)" } ,
	{ 0x125D, 0x0000, "ESS336H", "PCI Fax Modem (early model)" } ,
	{ 0x125D, 0x1968, "ES1968", "Maestro-2 PCI audio accelerator" } ,
	{ 0x125D, 0x1969, "ES1938/41/46", "Solo-1 PCI AudioDrive family" } ,
	{ 0x125d, 0x1978, "ES1978", "Maestro-2 PCI Audio Accelerator" } ,
	{ 0x125d, 0x1988, "ES1989", "Allegro-1 Audiodrive" } ,
	{ 0x125d, 0x1989, "ES56CVM-PI", "Allegro-1.COMM PCI Voice+Fax Modem" } ,
	{ 0x125d, 0x1998, "ES1980", "Maestro-3 PCI Audio Accelerator" } ,
	{ 0x125d, 0x1999, "ES1983", "Maestro-3.COMM PCI Voice+Fax Modem" } ,
	{ 0x125d, 0x199A, "ES1980", "Maestro-3 PCI Audio Accelerator" } ,
	{ 0x125D, 0x199B, "ES1938", "Maestro-3.COMM PCI Voice+Fax Modem" } ,
	{ 0x125D, 0x2808, "ES336H", "PCI Fax Modem (later model)" } ,
	{ 0x125d, 0x2838, "ES2838S/2839", "SuperLink PCI Fax Modem V.90" } ,
	{ 0x125d, 0x2843, "ES2838/2839", "SuperLink-MLP Voice Modem" } ,
	{ 0x125d, 0x2847, "ES2838/2839", "SuperLink-MLP 10  Voice Modem" } ,
	{ 0x125D, 0x2898, "ES2898S", "TelDrive ES56T-PI family V.90 PCI modem" } ,
	{ 0x1260, 0x3873, "ISL3874A", "PRISMII.5 IEE802.11B Wireless LAN" } ,
	{ 0x1260, 0x8130, "HMP8130", "NTSC/PAL Video Decoder" } ,
	{ 0x1260, 0x8131, "HMP8131", "NTSC/PAL Video Decoder" } ,
	{ 0x1266, 0x0001, "", "NE10/100 Adapter (i82557B)" } ,
	{ 0x1266, 0x1910, "", "NE2000Plus (RT8029) Ethernet Adapter" } ,
	{ 0x1267, 0x1016, "", "NICCY PCI card" } ,
	{ 0x1267, 0x4243, "", "Satellite receiver board / MPEG2 decoder" } ,
	{ 0x1267, 0x5352, "PCR2101", "" } ,
	{ 0x1267, 0x5A4B, "telsatturbo", "" } ,
	{ 0x126C, 0x1F1F, "", "e-mobility 802.11b Wireless LAN PCI Card" } ,
	{ 0x126F, 0x0710, "SM710", "LynxEM" } ,
	{ 0x126F, 0x0712, "SM712", "LynxEM+" } ,
	{ 0x126F, 0x0720, "SM720", "Lynx3DM" } ,
	{ 0x126F, 0x0810, "SM810", "LynxE" } ,
	{ 0x126F, 0x0811, "SM811", "LynxE" } ,
	{ 0x126F, 0x0820, "SM820", "Lynx3D" } ,
	{ 0x126F, 0x0910, "SM910", "Lynx" } ,
	{ 0x1273, 0x0002, "DirecPC", "" } ,
	{ 0x1274, 0x1274, "5880", "multimedia audio device" } ,
	{ 0x1274, 0x1371, "ES1371", "AudioPCI" } ,
	{ 0x1274, 0x1373, "ES1373", "Sound Blaster Audio(PCI)" } ,
	{ 0x1274, 0x5000, "ES1370", "AudioPCI" } ,
	{ 0x1274, 0x5880, "5880", "AudioPCI" } ,
	{ 0x1278, 0x0701, "TPE3/TM3", "PowerPC Node" } ,
	{ 0x1279, 0x0295, "", "Virtual Northbridge" } ,
	{ 0x1279, 0x0395, "LongRun", "Northbridge" } ,
	{ 0x1279, 0x0396, "", "SDRAM Controller" } ,
	{ 0x1279, 0x0397, "", "BIOS scratchpad" } ,
	{ 0x127A, 0x1002, "RH56D-PCI", "Modem enumerator" } ,
	{ 0x127A, 0x1003, "65785467", "HCF 56k V.90 Modem" } ,
	{ 0x127A, 0x1004, "R6795-12", "HCF 56k V.90 Modem" } ,
	{ 0x127A, 0x1005, "r6793-17", "HSF 56k V.90 Speakerphone/Voice Modem" } ,
	{ 0x127A, 0x1022, "1456VQH20E", "HCF V.90 Modem" } ,
	{ 0x127A, 0x1023, "", "V.90 Modem" } ,
	{ 0x127A, 0x1024, "R6785-61", "HCF 56k PCI Modem (Amquest)" } ,
	{ 0x127A, 0x1025, "1456VQH-R1", "HCF 56k PCI Modem" } ,
	{ 0x127A, 0x1026, "R6785-61", "HCF 56k V.90 Speakerphone Modem" } ,
	{ 0x127A, 0x1032, "", "HCF 56k PCI Modem" } ,
	{ 0x127A, 0x1033, "R6795 12", "HCF P85 DATA/FAX PCI Modem" } ,
	{ 0x127A, 0x1034, "", "HCF 56k PCI Modem" } ,
	{ 0x127A, 0x1035, "RH56D/SP-PCI", "HCF 56k Speakerphone Modem" } ,
	{ 0x127A, 0x1036, "", "HCF 56k PCI Modem" } ,
	{ 0x127A, 0x1085, "r6793-11", "Volcano SoftK56 Data,Fax,Speak PCI Modem" } ,
	{ 0x127a, 0x1125, "", "" } ,
	{ 0x127A, 0x2004, "SoftK56VB2.1V2.08.02", "K56 modem" } ,
	{ 0x127A, 0x2005, "RS56/SP-PCI11P1", "Single chip 56K V90 modem/spkrphone" } ,
	{ 0x127A, 0x2013, "", "Volcano SoftK56 PCI modem" } ,
	{ 0x127A, 0x2014, "rs56 sp-pci", "PCI modem" } ,
	{ 0x127A, 0x2015, "r6793-11", "Conexant SoftK56 Speakerphone Modem" } ,
	{ 0x127A, 0x2016, "RS56/SP-PCI11P1", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x127A, 0x2114, "R6793-11", "Soft 56K Data, Fax, PCI modem" } ,
	{ 0x127A, 0x2F00, "cx11252-11", "HSF 56k HSFi" } ,
	{ 0x127A, 0x4310, "123456", "Master Riptide PCI Audio Device" } ,
	{ 0x127A, 0x4311, "V1456VQH-R6", "Conexant PCI Modem Enumerator" } ,
	{ 0x127A, 0x4312, "", "Riptide PCI Game Controller" } ,
	{ 0x127A, 0x4320, "810E", "Riptide PCI Audio Controller" } ,
	{ 0x127A, 0x4321, "RLVDL56DPF/SP", "Riptide HCF 56k PCI Modem" } ,
	{ 0x127A, 0x4322, "", "Riptide PCI Game Controller" } ,
	{ 0x127A, 0x5278, "Harmonic DVB", "Network Adapter" } ,
	{ 0x127A, 0x8234, "RS8234", "ATM ServiceSAR Plus" } ,
	{ 0x1282, 0x9009, "DM9009", "Ethernet Adapter" } ,
	{ 0x1282, 0x9102, "DM9102/A/AF", "GFast Ethernet Adapter" } ,
	{ 0x1283, 0x0801, "IT8152F/G", "Audio Digital Controller" } ,
	{ 0x1283, 0x673A, "IT8330G", "IDE Controller" } ,
	{ 0x1283, 0x8152, "IT8152F/G", "Advanced RISC-to-PCI Companion Chip" } ,
	{ 0x1283, 0x8172, "IT8172G", "Ultra RISC (MIPS, SH4) Companion Chip" } ,
	{ 0x1283, 0x8330, "IT8330G", "Host Bridge" } ,
	{ 0x1283, 0x8872, "IT8871/72", "PCI to ISA I/O chip" } ,
	{ 0x1283, 0x8875, "IT8875F", "PCI Parallel Port" } ,
	{ 0x1283, 0x8888, "IT8888F", "PCI to ISA Bridge with SMB" } ,
	{ 0x1283, 0x8889, "IT8889F", "PCI to ISA Bridge" } ,
	{ 0x1283, 0x9876, "IT8875F", "PCI I/O CARD" } ,
	{ 0x1283, 0xE886, "IT8330G", "PCI to ISA Bridge" } ,
	{ 0x1285, 0x0100, "ES1849", "Maestro-1 AudioDrive" } ,
	{ 0x1287, 0x001E, "LS220D", "DVD Decoder" } ,
	{ 0x1287, 0x001F, "LS220C", "DVD Decoder" } ,
	{ 0x1287, 0x0020, "LS242", "MPEG/DVD video decoder" } ,
	{ 0x1289, 0x1006, "", "" } ,
	{ 0x128A, 0xF001, "Ethernet 10/100", "AsanteFAST 10/100 PCI Ethernet Adapter" } ,
	{ 0x128D, 0x0021, "", "ATM Adapter" } ,
	{ 0x128E, 0x0008, "ST128", "WSS/SB" } ,
	{ 0x128E, 0x0009, "ST128", "SAM9407" } ,
	{ 0x128E, 0x000A, "ST128", "Game Port" } ,
	{ 0x128E, 0x000B, "ST128", "MPU Port" } ,
	{ 0x128E, 0x000C, "ST128", "Control Port" } ,
	{ 0x129A, 0x0415, "PBT-415", "PCI 66MHz Analyzer and 33MHz Exerciser" } ,
	{ 0x129A, 0x0515, "PBT-515", "PCI 66MHz Analyzer and Exerciser" } ,
	{ 0x129A, 0x0615, "PBT-615", "PCI 66MHz and PCI-X 100MHz Bus Analyzer" } ,
	{ 0x12A3, 0xECB8, "1646T00", "" } ,
	{ 0x12aa, 0x5568, "", "WANic 400 series" } ,
	{ 0x12AA, 0x556C, "", "NAI HSSI Sniffer PCI Adapter" } ,
	{ 0x12AB, 0x3000, "TUN-200/MPG-200C", "PCI TV (and DVD Decoder?) Card" } ,
	{ 0x12AE, 0x0001, "3C986", "ACEnic 1000 BASE-SX Ethernet adapter" } ,
	{ 0x12AE, 0x0002, "3C986-T", "Copper Gigabit Ethernet Adapter" } ,
	{ 0x12B9, 0x0062, "erk41926a-0.6", "usr 56k internal modem" } ,
	{ 0x12B9, 0x1006, "3cp803598", "USR 56k Internal Voice WinModem" } ,
	{ 0x12B9, 0x1007, "ERL3263A-0", "USR 56k Internal DF GWPCI PC99" } ,
	{ 0x12B9, 0x1008, "3cp803598", "USR 56k Internal Modem" } ,
	{ 0x12BA, 0x0032, "Hammerhead-Lite-PCI", "DSP Prototyping & Development Card" } ,
	{ 0x12BE, 0x3041, "AN3041Q", "CO-MEM PCI Bus Interface and Cache" } ,
	{ 0x12be, 0x3042, "CY7C09449PV-AC", "128Kbit Dual-Port SRAM w PCI Bus Ctrlr" } ,
	{ 0x12C1, 0x9080, "Sync4hs/CCP/PCI/MP", "Communications Processor" } ,
	{ 0x12C3, 0x0058, "HT80232", "LAN Adapter (NE2000-compatible)" } ,
	{ 0x12C3, 0x5598, "HT80229", "Ethernet Adapter (NE2000-compatible)" } ,
	{ 0x12C5, 0x007F, "ISE", "PEI Imaging Subsystem Engine" } ,
	{ 0x12C5, 0x0081, "PCIVST", "PCI Thresholding Engine" } ,
	{ 0x12C5, 0x0085, "", "Video Simulator/Sender" } ,
	{ 0x12C5, 0x0086, "THR2", "Multi-scale Thresholder" } ,
	{ 0x12C7, 0x0546, "", "D120JCT-LS Card" } ,
	{ 0x12C7, 0x0561, "", "BRI/2 Type Card (Voice Driver)" } ,
	{ 0x12C7, 0x0647, "", "D/240JCT-T1 Card" } ,
	{ 0x12C7, 0x0648, "", "D/300JCT-E1 Card" } ,
	{ 0x12C7, 0x0649, "", "D/300JCT-E1 Card" } ,
	{ 0x12C7, 0x0651, "", "MSI PCI Card" } ,
	{ 0x12C7, 0x0673, "", "BRI/160-PCI Card" } ,
	{ 0x12C7, 0x0674, "", "BRI/120-PCI Card" } ,
	{ 0x12C7, 0x0675, "", "BRI/80-PCI Card" } ,
	{ 0x12C7, 0x0676, "", "D/41JCT Card" } ,
	{ 0x12C7, 0x0685, "", "D/480JCT-2T1 Card" } ,
	{ 0x12C7, 0x0687, "", "D/600JCT-2E1 (75 Ohm) Card" } ,
	{ 0x12C7, 0x0689, "D/600JCT-2E1", "Dialogic 2E1 - JCT series" } ,
	{ 0x12C7, 0x0707, "", "D/320JCT (Resource Only) Card" } ,
	{ 0x12C7, 0x0708, "", "D/160JCT (Resource Only) Card" } ,
	{ 0x12CB, 0x0027, "StudioCard", "" } ,
	{ 0x12CB, 0x002D, "BX-12", "" } ,
	{ 0x12CB, 0x002E, "SC-2000", "" } ,
	{ 0x12CB, 0x002F, "LX-44", "" } ,
	{ 0x12CB, 0x0030, "SC-22", "" } ,
	{ 0x12CB, 0x0031, "BX-44", "" } ,
	{ 0x12CB, 0x0032, "LX-24M", "20-bit 2-in, 4-out audio card w/MPEG-2" } ,
	{ 0x12CB, 0x0033, "LX-22M", "" } ,
	{ 0x12CB, 0x0034, "BX-8", "" } ,
	{ 0x12CB, 0x0035, "BX-12e", "" } ,
	{ 0x12D2, 0x0008, "NV1", "EDGE 3D Accelerator" } ,
	{ 0x12D2, 0x0009, "DAC64", "EDGE 3D" } ,
	{ 0x12d2, 0x0010, "Mutara V08", "" } ,
	{ 0x12D2, 0x0018, "RIVA 128", "128-bit 3D Multimedia Accelerator" } ,
	{ 0x12D2, 0x0019, "RIVA 128ZX", "2D/3D GUI Accelerator" } ,
	{ 0x12D2, 0x0020, "NV4", "Riva TNT GUI+3D Accelerator" } ,
	{ 0x12D2, 0x0028, "NV5", "Riva TNT2 /TNT2 Pro" } ,
	{ 0x12D2, 0x0029, "NVULTRA", "Riva TNT2 Ultra" } ,
	{ 0x12D2, 0x002A, "NV5", "Riva TNT2" } ,
	{ 0x12D2, 0x002B, "NV5", "Riva TNT2" } ,
	{ 0x12D2, 0x002C, "NVVANTA", "Vanta / Vanta LT" } ,
	{ 0x12D2, 0x002D, "NVM64", "Riva TNT2 Model 64" } ,
	{ 0x12D2, 0x002E, "NV6", "Vanta" } ,
	{ 0x12D2, 0x002F, "NV6", "Vanta" } ,
	{ 0x12D2, 0x00A0, "NVA0", "Riva TNT2 Aladdin" } ,
	{ 0x12D5, 0x1000, "MAP-CA", "Broadband Signal Processor" } ,
	{ 0x12D5, 0x1002, "MAP-1000", "Digital Signal Processor" } ,
	{ 0x12D8, 0x71E2, "PI7C7300", "3 Port PCI to PCI bridge" } ,
	{ 0x12D8, 0x8150, "PI7C8150", "2-Port PCI to PCI Bridge" } ,
	{ 0x12DB, 0x0003, "", "FoxFire II" } ,
	{ 0x12DE, 0x0200, "", "Cryptoswift 200" } ,
	{ 0x12E0, 0x0010, "ST16C654", "Quad UART" } ,
	{ 0x12E0, 0x0020, "ST16C654", "Quad UART" } ,
	{ 0x12E0, 0x0030, "ST16C654", "Quad UART" } ,
	{ 0x12E4, 0x1140, "", "ISDN Controller" } ,
	{ 0x12EB, 0x0001, "AU8820", "Vortex 1 Digital Audio Processor" } ,
	{ 0x12eb, 0x0002, "AU8830", "Vortex 2 3D Digital Audio Processor" } ,
	{ 0x12eb, 0x0003, "AU8810", "Vortex Digital Audio Processor" } ,
	{ 0x12eb, 0x8803, "", "Vortex 56k Software Modem" } ,
	{ 0x12F8, 0x0002, "VideoMaker", "" } ,
	{ 0x1303, 0x0001, "", "cM67 CompactPCI DSP Card" } ,
	{ 0x1307, 0x0001, "PCI-DAS1602/16", "" } ,
	{ 0x1307, 0x0006, "PCI-GPIB", "" } ,
	{ 0x1307, 0x000B, "PCI-DIO48H", "" } ,
	{ 0x1307, 0x000C, "PCI-PDISO8", "" } ,
	{ 0x1307, 0x000D, "PCI-PDISO16", "" } ,
	{ 0x1307, 0x000F, "PCI-DAS1200", "" } ,
	{ 0x1307, 0x0010, "PCI-DAS1602/12", "" } ,
	{ 0x1307, 0x0014, "PCI-DIO24H", "" } ,
	{ 0x1307, 0x0015, "PCI-DIO24H/CTR3", "" } ,
	{ 0x1307, 0x0016, "PCI-DIO24H/CTR16", "" } ,
	{ 0x1307, 0x0017, "PCI-DIO96H", "" } ,
	{ 0x1307, 0x0018, "PCI-CTR05", "" } ,
	{ 0x1307, 0x0019, "PCI-DAS1200/JR", "" } ,
	{ 0x1307, 0x001A, "PCI-DAS1001", "" } ,
	{ 0x1307, 0x001B, "PCI-DAS1002", "" } ,
	{ 0x1307, 0x001C, "PCI-DAS1602JR/16", "" } ,
	{ 0x1307, 0x001D, "PCI-DAS6402/16", "" } ,
	{ 0x1307, 0x001E, "PCI-DAS6402/12", "" } ,
	{ 0x1307, 0x001F, "PCI-DAS16/M1", "" } ,
	{ 0x1307, 0x0020, "PCI-DDA02/12", "" } ,
	{ 0x1307, 0x0021, "PCI-DDA04/12", "" } ,
	{ 0x1307, 0x0022, "PCI-DDA08/12", "" } ,
	{ 0x1307, 0x0023, "PCI-DDA02/16", "" } ,
	{ 0x1307, 0x0024, "PCI-DDA04/16", "" } ,
	{ 0x1307, 0x0025, "PCI-DDA08/16", "" } ,
	{ 0x1307, 0x0026, "PCI-DAC04/12-HS", "" } ,
	{ 0x1307, 0x0027, "PCI-DAC04/16-HS", "" } ,
	{ 0x1307, 0x0028, "CIO-DIO24", "24 Bit Digital Input/Output Board" } ,
	{ 0x1307, 0x0029, "PCI-DAS08", "" } ,
	{ 0x1307, 0x002C, "PCI-INT32", "" } ,
	{ 0x1307, 0x0033, "PCI-DUAL-AC5", "" } ,
	{ 0x1307, 0x0034, "PCI-DAS-TC", "" } ,
	{ 0x1307, 0x0035, "PCI-DAS64/M1/16", "" } ,
	{ 0x1307, 0x0036, "PCI-DAS64/M2/16", "" } ,
	{ 0x1307, 0x0037, "PCI-DAS64/M3/16", "" } ,
	{ 0x1307, 0x004C, "PCI-DAS1000", "" } ,
	{ 0x1308, 0x0001, "", "NetCelerator Adapter" } ,
	{ 0x1310, 0x0003, "9060", "CompactPCI Interface" } ,
	{ 0x1310, 0x000D, "", "FPGA PCI Bridge" } ,
	{ 0x1317, 0x0531, "", "" } ,
	{ 0x1317, 0x0981, "AM981", "Ethernet Adapter" } ,
	{ 0x1317, 0x0985, "ADM983", "fast ethernet controller" } ,
	{ 0x1317, 0x1985, "ADM985", "10/100 cardbus ethernet controller" } ,
	{ 0x1317, 0x9511, "ADM9511", "cardbus ethernet-modem controller" } ,
	{ 0x1317, 0x9513, "ADM9513", "cardbus ethernet-modem controller" } ,
	{ 0x1318, 0x0911, "G-NIC II", "1000BT Network Interface Card" } ,
	{ 0x1319, 0x0801, "FM801", "Xwave PCI audio controller" } ,
	{ 0x1319, 0x0802, "FM801", "Xwave PCI Joystick" } ,
	{ 0x1319, 0x1000, "FM801", "PCI Audio" } ,
	{ 0x1319, 0x1001, "FM801", "PCI Joystick" } ,
	{ 0x131f, 0x1000, "", "PCI Serial Card" } ,
	{ 0x131f, 0x1001, "", "CyberSerial 16550 (1-port)" } ,
	{ 0x131f, 0x1002, "", "CyberSerial 16850 (1-port)" } ,
	{ 0x131f, 0x1010, "", "Duet1S(16550)+1P" } ,
	{ 0x131f, 0x1011, "", "Duet 1S(16550)+1P" } ,
	{ 0x131f, 0x1012, "", "Duet 1S(16550)+1P" } ,
	{ 0x131f, 0x1020, "", "CyberParallel PCI Card" } ,
	{ 0x131f, 0x1021, "", "CyberParallel PCI Card" } ,
	{ 0x131f, 0x1030, "", "CyberSerial 16550" } ,
	{ 0x131f, 0x1031, "", "CyberSerial 16650" } ,
	{ 0x131f, 0x1032, "", "CyberSerial 16850" } ,
	{ 0x131f, 0x1034, "", "Trio 2S(16550)+1P" } ,
	{ 0x131f, 0x1035, "", "Trio 2S(16650)+1P" } ,
	{ 0x131f, 0x1036, "", "Trio 2S(16850)+1P" } ,
	{ 0x131f, 0x1050, "", "CyberSerial 16550" } ,
	{ 0x131f, 0x1051, "", "CyberSerial 16650" } ,
	{ 0x131f, 0x1052, "", "CyberSerial 16850" } ,
	{ 0x131f, 0x2000, "", "CyberSerial 16550" } ,
	{ 0x131f, 0x2001, "", "CyberSerial 16650" } ,
	{ 0x131F, 0x2002, "", "CyberSerial 16850" } ,
	{ 0x131f, 0x2010, "", "Duet 1S(16550)+1P" } ,
	{ 0x131F, 0x2011, "", "Duet 1S(16650)+1P" } ,
	{ 0x131F, 0x2012, "", "Duet 1S(16850)+1P" } ,
	{ 0x131f, 0x2020, "CyberParallel", "" } ,
	{ 0x131f, 0x2021, "CyberParallel", "" } ,
	{ 0x131f, 0x2030, "", "CyberSerial 16550/PCI SERIAL 8000 IO1875" } ,
	{ 0x131f, 0x2031, "", "CyberSerial 16650" } ,
	{ 0x131f, 0x2032, "", "CyberSerial 16850" } ,
	{ 0x131f, 0x2040, "", "Trio 1S(16550)+2P" } ,
	{ 0x131f, 0x2041, "", "Trio 1S(16650)+2P" } ,
	{ 0x131F, 0x2042, "", "Trio 1S(16850)+2P" } ,
	{ 0x131f, 0x2050, "", "CyberSerial 16550" } ,
	{ 0x131F, 0x2051, "", "CyberSerial 16650" } ,
	{ 0x131F, 0x2052, "", "CyberSerial 16850" } ,
	{ 0x131F, 0x2060, "", "Trio 2S(16550)+1P" } ,
	{ 0x131F, 0x2061, "", "Trio 2S(16650)+1P" } ,
	{ 0x131F, 0x2062, "", "Trio 2S(16850)+1P" } ,
	{ 0x1332, 0x5415, "MM-5415CN", "PCI Memory Module with Battery Backup" } ,
	{ 0x133D, 0x1000, "SST-5136-PFB-PCI", "Industrial I/O Card" } ,
	{ 0x1344, 0x3240, "", "CopperHead CopperTail SC1 AMC AC97" } ,
	{ 0x1344, 0x3320, "MT8LLN21PADF", "North Bridge" } ,
	{ 0x1344, 0x3470, "MT7LLN22NCNE", "South Bridge" } ,
	{ 0x1344, 0x4020, "", "CopperHead CopperTail SC1 IDE Controller" } ,
	{ 0x1344, 0x4030, "", "CopperHead CopperTail SC1 USB Controller" } ,
	{ 0x134A, 0x0001, "F01 2ASV17184.1", "Domex DMX 3191 PCI SCSI Controller" } ,
	{ 0x134A, 0x0002, "", "Domex DMX3192U/3194UP SCSI Adapter" } ,
	{ 0x134D, 0x2486, "2304WT", "V.92 MDC Modem" } ,
	{ 0x134D, 0x7890, "PCT789T-C1", "HSP MicroModem 56" } ,
	{ 0x134D, 0x7891, "PCT 789T", "HSP MicroModem 56" } ,
	{ 0x134D, 0x7892, "PCT 789T-A", "HSP MicroModem 56" } ,
	{ 0x134D, 0x7893, "S911 K017", "HSP MicroModem 56" } ,
	{ 0x134D, 0x7894, "FT13", "HSP MicroModem 56" } ,
	{ 0x134D, 0x7895, "PCT789T-C1", "HSP MicroModem 56" } ,
	{ 0x134D, 0x7896, "pct789t-c1", "HSP MicroModem 56" } ,
	{ 0x134D, 0x7897, "97860963", "HSP MicroModem 56/PCT789T" } ,
	{ 0x134D, 0x9714, "PCT 288-1A", "PCTEL" } ,
	{ 0x134D, 0xD800, "pct388p-a", "pctel 56k modem" } ,
	{ 0x135a, 0x0042, "PLX PCI9050 + 16C554", "" } ,
	{ 0x135a, 0x0043, "PLX PCI9050 + 16C554", "" } ,
	{ 0x135a, 0x0061, "PLX PCI9050 + 16C552", "2-port RS232 card with printer port" } ,
	{ 0x135a, 0x0062, "PLX PCI9050 + 16C552", "2-port RS232 card with printer port, r 3" } ,
	{ 0x135a, 0x0063, "PLX PCI9050 + 16C552", "2-port RS232 card with printer port, r 4" } ,
	{ 0x135A, 0x0224, "PLX9050", "PLX PCI Bus Logic" } ,
	{ 0x135a, 0x04A3, "PLX PCI9050 + 16C552", "2-port RS232 card, r 4" } ,
	{ 0x135a, 0x04A4, "PLX PCI9050 + 16C552", "2-port RS232 card, r 5" } ,
	{ 0x135E, 0x5101, "5101", "Route 56" } ,
	{ 0x135E, 0x7101, "", "Single Port RS-232/422/485/520" } ,
	{ 0x135E, 0x7201, "", "Dual Port RS-232/422/485 Interface" } ,
	{ 0x135E, 0x7202, "", "Dual Port RS-232 Interface" } ,
	{ 0x135E, 0x7401, "", "Four Port RS-232 Interface" } ,
	{ 0x135E, 0x7402, "", "Four Port RS-422/485 Interface" } ,
	{ 0x135E, 0x7801, "", "Eight Port RS-232 Interface" } ,
	{ 0x135E, 0x8001, "8001", "Digital I/O Adapter" } ,
	{ 0x1365, 0x9050, "HYSDN", "" } ,
	{ 0x1382, 0x2048, "", "Prodif Plus sound card" } ,
	{ 0x1385, 0x4100, "MA301", "802.11b Wireless Adapter" } ,
	{ 0x1385, 0x620A, "GA620", "" } ,
	{ 0x1385, 0x622A, "GA622", "" } ,
	{ 0x1385, 0x630A, "GA630", "" } ,
	{ 0x1385, 0xF311, "FA311", "Fast Ethernet Adapter" } ,
	{ 0x1389, 0x0001, "PCI1500PFB", "Intelligent fieldbus Adapter" } ,
	{ 0x1393, 0x1040, "C104H/PCI", "Smartio" } ,
	{ 0x1393, 0x1320, "CP-132", "Industio" } ,
	{ 0x1393, 0x1680, "C168H/PCI", "Smartio" } ,
	{ 0x1393, 0x2040, "CP-204J", "Intellio" } ,
	{ 0x1393, 0x2180, "C218", "Intellio Turbo PCI" } ,
	{ 0x1393, 0x3200, "C320", "Intellio Turbo PCI" } ,
	{ 0x1394, 0x0001, "LXT1001", "Gigabit Ethernet Adapter" } ,
	{ 0x1397, 0x0B4D, "HFC-8S 16B8D8S0", "ISDN HDLC FIFO Controller" } ,
	{ 0x1397, 0x2BD0, "HFC-PCI A ISDN 2BDS0", "ISDN HDLC FIFO Controller" } ,
	{ 0x1397, 0x8B4D, "HFC-4S ISDN 8B4D4S0", "ISDN HDLC FIFO Controller" } ,
	{ 0x1397, 0xB000, "B000", "HCF-PCI card" } ,
	{ 0x1397, 0xB006, "B006", "HCF-PCI card" } ,
	{ 0x1397, 0xB007, "B007", "HCF-PCI card" } ,
	{ 0x1397, 0xB008, "B008", "HCF-PCI card" } ,
	{ 0x1397, 0xB009, "B009", "HCF-PCI card" } ,
	{ 0x1397, 0xB00A, "B00A", "HCF-PCI card" } ,
	{ 0x1397, 0xB00B, "B00B", "HCF-PCI card" } ,
	{ 0x1397, 0xB00C, "B00C", "HCF-PCI card" } ,
	{ 0x1397, 0xB100, "B100", "HCF-PCI card" } ,
	{ 0x13A3, 0x0005, "7751", "Security Processor" } ,
	{ 0x13A3, 0x0006, "6500", "Public Key Processor" } ,
	{ 0x13A3, 0x0007, "7811", "Security Processor" } ,
	{ 0x13A3, 0x0012, "7951", "Security Processor" } ,
	{ 0x13a8, 0x0152, "XR17C152", "Two channel PCI UART (5V)" } ,
	{ 0x13A8, 0x0154, "XR17C154", "Four Channel PCI Bus UART" } ,
	{ 0x13A8, 0x0158, "XR17C158", "Eight Channel PCI Bus UART (5V)" } ,
	{ 0x13C0, 0x0010, "", "SyncLink PCI WAN Adapter" } ,
	{ 0x13c0, 0x0020, "", "SyncLink SCC Adapter" } ,
	{ 0x13c0, 0x0030, "", "SyncLink Multiport Adapter" } ,
	{ 0x13C1, 0x1000, "", "ATA-RAID Controller" } ,
	{ 0x13C1, 0x1001, "7000 series", "ATA-100 Storage Controller" } ,
	{ 0x13C7, 0x0ADC, "", "Multi-Function Analogue/Digital IO card" } ,
	{ 0x13D0, 0x2103, "T228502", "B2C2 Sky2PC Core Chip" } ,
	{ 0x13D1, 0xAB06, "FE2000VX", "CardBus /Atelco Fibreline Ethernet Adptr" } ,
	{ 0x13D7, 0x8086, "", "" } ,
	{ 0x13DF, 0x0001, "PCI56RVP", "Modem" } ,
	{ 0x13EA, 0x3131, "DS3131", "BoSS Bit Synchronous HDLC Controller" } ,
	{ 0x13EA, 0x3134, "DS3134", "Chateau Channelized T1/E1/HDLC Controller" } ,
	{ 0x13F0, 0x0201, "ST201", "Fast Ehternet Adapter" } ,
	{ 0x13F6, 0x0100, "CMI8338/PCI C3DX", "PCI Audio Chip" } ,
	{ 0x13F6, 0x0101, "CMI8338-031", "PCI Audio Device" } ,
	{ 0x13F6, 0x0111, "CMI8738/PCI C3DX", "PCI Audio Chip" } ,
	{ 0x13F6, 0x0112, "CMI-8378B/PCI-6CH", "PCI Audio Chip" } ,
	{ 0x13F6, 0x0211, "HSP56", "Audiomodem Riser" } ,
	{ 0x13FE, 0x1240, "PCI-1240-A", "4-Axis Stepping/Servo Motor Card" } ,
	{ 0x13fe, 0x1750, "PCI-1750", "Opto Isolated Digital I/O Card w/Counter" } ,
	{ 0x1400, 0x1401, "9432 TX", "" } ,
	{ 0x1407, 0x0100, "", "Lava Dual Serial 550 PCI" } ,
	{ 0x1407, 0x0101, "", "Lava Quatro PCI A/B" } ,
	{ 0x1407, 0x0102, "", "Lava Quatro PCI C/D" } ,
	{ 0x1407, 0x0110, "", "Lava DSerial PCI Port A" } ,
	{ 0x1407, 0x0111, "", "Lava DSerial PCI Port B" } ,
	{ 0x1407, 0x0180, "", "Lava Octopus PCI Ports 1-4" } ,
	{ 0x1407, 0x0181, "", "Lava Octopus PCI Ports 5-8" } ,
	{ 0x1407, 0x0200, "", "LavaPort Dual-650 PCI" } ,
	{ 0x1407, 0x0201, "", "LavaPort Quad-650 PCI A/B" } ,
	{ 0x1407, 0x0202, "", "LavaPort Quad-650 PIC C/D" } ,
	{ 0x1407, 0x0400, "", "Lava 8255 PIO PCI" } ,
	{ 0x1407, 0x0500, "", "Lava Single Serial 550 PCI" } ,
	{ 0x1407, 0x0510, "", "Lava SP Serial 550 PCI" } ,
	{ 0x1407, 0x0511, "", "Lava SP BIDIR Parallel PCI" } ,
	{ 0x1407, 0x0600, "", "Lava Port 650 PCI" } ,
	{ 0x1407, 0x0A00, "LavaPort PCI", "COM Port Accelerator" } ,
	{ 0x1407, 0x8000, "", "Lava Parallel" } ,
	{ 0x1407, 0x8001, "", "Lava Dual Parallel port A" } ,
	{ 0x1407, 0x8002, "", "Lava Dual Parallel port A" } ,
	{ 0x1407, 0x8003, "", "Lava Dual Parallel port B" } ,
	{ 0x1407, 0x8800, "", "BOCA Research IOPPAR" } ,
	{ 0x1412, 0x1712, "ICE1712", "Envy24 PCI Multi-Channel I/O Ctrlr" } ,
	{ 0x1415, 0x8401, "OX9162", "PCI Interface to local bus" } ,
	{ 0x1415, 0x8403, "OX9162", "Parallel Port" } ,
	{ 0x1415, 0x9500, "OX16PCI954", "Quad UART (disabled)" } ,
	{ 0x1415, 0x9501, "OX16PCI954", "Quad UART" } ,
	{ 0x1415, 0x950B, "OXCB950", "Integrated High Performance UART" } ,
	{ 0x1415, 0x9510, "OX16PCI954", "PCI Interface (disabled)" } ,
	{ 0x1415, 0x9511, "OX9160", "PCI Interface to 8-bit local bus" } ,
	{ 0x1415, 0x9512, "OX16PCI954", "PCI Interface to 32-bit bus" } ,
	{ 0x1415, 0x9513, "OX16PCI954", "Parallel Port" } ,
	{ 0x1415, 0x9521, "OX12PCI952", "Dual UART" } ,
	{ 0x1415, 0x9523, "OX12PCI952", "Integrated Parallel Port" } ,
	{ 0x141F, 0x6181, "KFIR", "MPEG decoder" } ,
	{ 0x1448, 0x0001, "ADAT/EDIT", "Audio Editing" } ,
	{ 0x144a, 0x6208, "PCI-6208", "" } ,
	{ 0x144A, 0x7248, "PCI-7248", "" } ,
	{ 0x144A, 0x7250, "PCI-7250", "PLX PCI9052" } ,
	{ 0x144A, 0x7296, "PCI-7296", "" } ,
	{ 0x144A, 0x7432, "PCI-7432", "" } ,
	{ 0x144A, 0x7433, "PCI-7433", "" } ,
	{ 0x144A, 0x7434, "PCI-7434", "" } ,
	{ 0x144A, 0x7841, "PCI-7841", "" } ,
	{ 0x144A, 0x8133, "PCI-8133", "" } ,
	{ 0x144A, 0x8554, "PCI-8554", "" } ,
	{ 0x144A, 0x9111, "PCI-9111", "" } ,
	{ 0x144A, 0x9113, "PCI-9113", "" } ,
	{ 0x144A, 0x9114, "PCI-9114", "" } ,
	{ 0x144B, 0x0601, "", "" } ,
	{ 0x145F, 0x0001, "NextMove PCI", "" } ,
	{ 0x1462, 0x5071, "", "Audio controller" } ,
	{ 0x1471, 0x0188, "RoadRunner 10", "ADSL PCI" } ,
	{ 0x148D, 0x1003, "Rockwell HCF chipset", "Creative ModemBlaster V.90 PCI DI5635" } ,
	{ 0x14B3, 0x0000, "", "DSL NIC" } ,
	{ 0x14B5, 0x0200, "Scope", "" } ,
	{ 0x14B5, 0x0300, "Pulsar", "" } ,
	{ 0x14B5, 0x0400, "Pulsar SRB", "" } ,
	{ 0x14B5, 0x0600, "Pulsar 2", "" } ,
	{ 0x14B5, 0x0800, "", "DSP-Board" } ,
	{ 0x14B5, 0x0900, "", "DSP-Board" } ,
	{ 0x14B5, 0x0A00, "", "DSP-Board" } ,
	{ 0x14B5, 0x0B00, "", "DSP-Board" } ,
	{ 0x14B7, 0x0001, "Symphony 4110", "" } ,
	{ 0x14B9, 0x0001, "PC4800", "" } ,
	{ 0x14B9, 0x0340, "", "Cisco Systems 340 PCI Wireless LAN Adptr" } ,
	{ 0x14b9, 0x0350, "AIR-PCI352", "AiroNet 350 PCI" } ,
	{ 0x14B9, 0x2500, "PC2500 DS", "Wireless PCI LAN Adapter" } ,
	{ 0x14B9, 0x3100, "PC3100 FH", "Wireless PCI LAN Adapter" } ,
	{ 0x14B9, 0x3101, "PC3100 FH", "Wireless PCI LAN Adapter" } ,
	{ 0x14B9, 0x3500, "PC3500 FH", "Wireless PCI LAN Adapter" } ,
	{ 0x14B9, 0x4500, "PC4500 DS", "Wireless PCI LAN Adapter" } ,
	{ 0x14B9, 0x4800, "PC4800 DS", "Wireless PCI LAN Adapter" } ,
	{ 0x14C1, 0x8043, "LANai 9.2 0129", "MyriNet" } ,
	{ 0x14D2, 0x8001, "PCI-010L", "Visial Systems VScom PCI-010L Contoller" } ,
	{ 0x14D2, 0x8002, "", "Visual Systems PCI-020L Controller" } ,
	{ 0x14D2, 0x8010, "", "Visual Systems VScom PCI-100L Controller" } ,
	{ 0x14D2, 0x8011, "", "Visual Systems VScom PCI-110L Controller" } ,
	{ 0x14D2, 0x8020, "", "Visual Systems VScom PCI-200L Controller" } ,
	{ 0x14D2, 0x8021, "", "Visual Systems VScom PCI-210L Controller" } ,
	{ 0x14D2, 0x8040, "", "Visual Systems VScom PCI-400L Controller" } ,
	{ 0x14D2, 0x8041, "", "Visual Systems VScom PCI-410L Controller" } ,
	{ 0x14D2, 0x8042, "", "Visual Systems VScom PCI-420L Controller" } ,
	{ 0x14D2, 0x8080, "", "Visual Systems VScom PCI-800L Controller" } ,
	{ 0x14D2, 0xA000, "", "Visual Systems VScom PCI-010H Controller" } ,
	{ 0x14D2, 0xA001, "", "Visual Systems VScom PCI-110H Controller" } ,
	{ 0x14D2, 0xA003, "16PCI954", "Visual Systems VScom PCI-400H Controller" } ,
	{ 0x14D2, 0xA004, "", "Visual Systems VScom PCI-400HF1 Ctrlr" } ,
	{ 0x14D2, 0xA005, "", "Visual Systems VScom PCI-200H Controller" } ,
	{ 0x14D2, 0xE001, "", "Visial Systems VScom PCI-010HV2" } ,
	{ 0x14D2, 0xE010, "", "Visual Systems VScom PCI-100HV2" } ,
	{ 0x14D2, 0xE020, "", "Visual Systems VScom PCI-200HV2" } ,
	{ 0x14D2, 0xFFFF, "", "Dummy Controller" } ,
	{ 0x14D4, 0x0400, "Panacom 7", "Interface chip" } ,
	{ 0x14DB, 0x2100, "PCI IO 1S", "" } ,
	{ 0x14DB, 0x2101, "PCI IO 1S-650", "" } ,
	{ 0x14DB, 0x2102, "PCI IO 1S-850", "" } ,
	{ 0x14DB, 0x2110, "PCI IO 1S1P", "" } ,
	{ 0x14DB, 0x2111, "PCI IO 1S1P-650", "" } ,
	{ 0x14DB, 0x2112, "PCI IO 1S1P-850", "" } ,
	{ 0x14DB, 0x2120, "TK9902", "" } ,
	{ 0x14DB, 0x2121, "PCI IO 2P", "" } ,
	{ 0x14DB, 0x2130, "PCI IO 2S", "2 Port PCI Serial Card" } ,
	{ 0x14DB, 0x2131, "PCI IO 2S-650", "" } ,
	{ 0x14DB, 0x2132, "PCI IO 2S-850", "" } ,
	{ 0x14DB, 0x2140, "PCI IO 2P1S", "" } ,
	{ 0x14DB, 0x2141, "PCI IO 2P1S-650", "" } ,
	{ 0x14DB, 0x2142, "PCI IO 2P1S-850", "" } ,
	{ 0x14DB, 0x2144, "PCI IO 2P2S", "" } ,
	{ 0x14DB, 0x2145, "PCI IO 2P2S-650", "" } ,
	{ 0x14DB, 0x2146, "PCI IO 2P2S-850", "" } ,
	{ 0x14DB, 0x2150, "PCI IO 4S", "" } ,
	{ 0x14DB, 0x2151, "PCI IO 4S-654", "" } ,
	{ 0x14DB, 0x2152, "PCI IO 4S-850", "" } ,
	{ 0x14DB, 0x2160, "PCI IO 2S1P", "" } ,
	{ 0x14DB, 0x2161, "PCI IO 2S1P-650", "" } ,
	{ 0x14DB, 0x2162, "PCI IO 2S1P-850", "" } ,
	{ 0x14DB, 0x2180, "PCI IO 8S", "" } ,
	{ 0x14DB, 0x2181, "PCI IO 8S-654", "" } ,
	{ 0x14DB, 0x2182, "PCI IO 8S-850", "" } ,
	{ 0x14DC, 0x0000, "PCI230", "" } ,
	{ 0x14DC, 0x0001, "PCI242H", "4-port high speed RS-232" } ,
	{ 0x14DC, 0x0002, "PCI244H", "8-port high speed RD-232" } ,
	{ 0x14DC, 0x0003, "PCI247H", "2-port high speed RS-232" } ,
	{ 0x14DC, 0x0004, "PCI248H", "2-port high speed RS-422/485" } ,
	{ 0x14DC, 0x0005, "PCI249H", "2-port high speed RS-232 and RS-422/485" } ,
	{ 0x14DC, 0x0006, "PCI260", "16-channel analog input (with timers)" } ,
	{ 0x14DC, 0x0007, "PCI224", "16-chan 12-bit analog output (w/ timers)" } ,
	{ 0x14DC, 0x0008, "PCI234", "4-chan 16-bit analog output (w/ timers)" } ,
	{ 0x14DC, 0x0009, "PCI236", "24-channel digital I/O" } ,
	{ 0x14DC, 0x000A, "PCI272", "72-channel digital I/O" } ,
	{ 0x14DC, 0x000B, "PCI215", "48-channel digital I/O (w/ 6 timers)" } ,
	{ 0x14DC, 0x000C, "PCI263", "16-channel reed relay output" } ,
	{ 0x14E4, 0x1644, "BCM5700", "NetXtreme Gigabit Ethernet Controller" } ,
	{ 0x14E4, 0x1645, "BCM5701", "NetXtreme Gigabit Ethernet" } ,
	{ 0x14E4, 0x1646, "BCM5702", "NetXtreme Gigabit Ethernet" } ,
	{ 0x14E4, 0x1647, "BCM5703", "NetXtreme Gigabit Ethernet" } ,
	{ 0x14E4, 0x164D, "BCM5702FE", "NetXtreme Fast Ethernet Controller" } ,
	{ 0x14E4, 0x16A6, "BCM5702X", "Gigabit Ethernet" } ,
	{ 0x14E4, 0x16A7, "BCM5703X", "Gigabit Ethernet" } ,
	{ 0x14E4, 0x4211, "BCM HPNA", "10Mb/s NIC" } ,
	{ 0x14E4, 0x4212, "BCM V.90", "56k Modem" } ,
	{ 0x14E4, 0x4301, "BCM4301 802.11b", "IEEE 802.11b WLAN client chipset" } ,
	{ 0x14E4, 0x4320, "BCM94306", "802.11g NIC" } ,
	{ 0x14E4, 0x4401, "BCM440x", "10/100 Integrated Ethernet Controller" } ,
	{ 0x14E4, 0x4402, "BCM440x", "10/100 Integrated Ethernet Controller" } ,
	{ 0x14E4, 0x5820, "BCM5820", "Crypto Accelerator" } ,
	{ 0x14EA, 0xAB06, "FNW-3603-TX", "10/100 Fast Ethernet CardBus (RTL8139)" } ,
	{ 0x14EB, 0x0020, "BEMx.x", "PCI to S5U13xxxB00B Bridge Adapter" } ,
	{ 0x14EB, 0x0C01, "S1D13808", "Embedded Memory Display Controller" } ,
	{ 0x14F1, 0x1002, "3251", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1003, "11242-11", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1004, "11242-11", "HCF 56k Modem   FCCID=H4TFM-PIB3PC" } ,
	{ 0x14F1, 0x1005, "", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1006, "", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1022, "", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1023, "", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1024, "", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1025, "r6793-15", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1026, "", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1032, "", "HCF 56k Modem" } ,
	{ 0x14f1, 0x1033, "ce0682x", "56k Winmodem" } ,
	{ 0x14F1, 0x1034, "R6795-12", "Single Chip Controllerless V.90 56k" } ,
	{ 0x14f1, 0x1035, "R6795-11", "PCI Modem Enumerator" } ,
	{ 0x14f1, 0x1036, "R6795-11", "Communications Controller" } ,
	{ 0x14F1, 0x1052, "", "HCF 56k Data/Fax Modem (Worldwide)" } ,
	{ 0x14F1, 0x1053, "", "HCF 56k Data/Fax Modem (Worldwide)" } ,
	{ 0x14F1, 0x1054, "", "HCF 56k Data/Fax/Voice Modem (Worldwide)" } ,
	{ 0x14F1, 0x1055, "", "HCF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14f1, 0x1056, "", "HCF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14f1, 0x1057, "", "HCF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x1058, "", "HCF P96 Data/Fax/Voice/Spkp Modem" } ,
	{ 0x14f1, 0x1059, "", "HCF 56k Data/Fax/Voice Modem (Worldwide)" } ,
	{ 0x14f1, 0x1063, "", "HCF 56k Data/Fax Modem" } ,
	{ 0x14f1, 0x1064, "", "HCF 56k Daat/Fax/Voice Modem" } ,
	{ 0x14f1, 0x1065, "", "HCF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14f1, 0x1066, "", "HCF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x1085, "CX11250", "SmartHSF Mobile Modem" } ,
	{ 0x14F1, 0x10B3, "", "HCF Data/Fax" } ,
	{ 0x14F1, 0x10B4, "", "HCF Data/Fax/Remote TAM" } ,
	{ 0x14F1, 0x10B5, "", "HCF Data/Fax/Voice/Speakerphone" } ,
	{ 0x14F1, 0x10B6, "", "HCF Data/Fax/Remote TAM/Speakerphone" } ,
	{ 0x14f1, 0x1433, "", "HCF 56k Data/Fax Modem" } ,
	{ 0x14f1, 0x1434, "", "HCF 56k Data/Fax/Voice Modem" } ,
	{ 0x14f1, 0x1435, "", "HCF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14f1, 0x1436, "", "HCF 56k Data/Fax Modem" } ,
	{ 0x14f1, 0x1453, "", "HCf 56k Data/Fax Modem" } ,
	{ 0x14f1, 0x1454, "", "HCF 56k Data/Fax/Voice Modem" } ,
	{ 0x14f1, 0x1455, "", "HCf 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14f1, 0x1456, "", "HCF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x1620, "P5100-xx", "ARM controller" } ,
	{ 0x14F1, 0x1621, "20463-xx", "HSF modem" } ,
	{ 0x14F1, 0x1622, "11627-xx", "ADSL modem" } ,
	{ 0x14F1, 0x1623, "", "HPNA 1" } ,
	{ 0x14F1, 0x1624, "", "Ethernet 10/100" } ,
	{ 0x14F1, 0x1625, "", "HomePNA2" } ,
	{ 0x14f1, 0x1803, "", "HCF 56k Modem" } ,
	{ 0x14f1, 0x1815, "", "HCF 56k Modem" } ,
	{ 0x14F1, 0x1F10, "", "HCF Data/Fax (USA)" } ,
	{ 0x14F1, 0x1F11, "", "HCF Data/Fax (Worldwide)" } ,
	{ 0x14F1, 0x1F14, "", "HCF Data/Fax/Voice (USA)" } ,
	{ 0x14F1, 0x1F15, "", "HCF Data/Fax/Voice (Worldwide)" } ,
	{ 0x14F1, 0x2003, "", "SoftK56 Winmodem" } ,
	{ 0x14F1, 0x2004, "", "SoftK56 RemoteTAM Winmodem" } ,
	{ 0x14F1, 0x2005, "", "SoftK56 Speakerphone Winmodem" } ,
	{ 0x14F1, 0x2006, "", "SoftK56 Speakerphone Winmodem" } ,
	{ 0x14F1, 0x2013, "8850155D", "SoftK56 Winmodem/HSF GENERIC" } ,
	{ 0x14f1, 0x2014, "r6793-15", "SoftK56 RemoteTAM Winmodem" } ,
	{ 0x14F1, 0x2015, "", "SoftK56 Speakerphone Winmodem" } ,
	{ 0x14F1, 0x2016, "", "HSF Data/Fax/TAM/Speakerphone" } ,
	{ 0x14f1, 0x2043, "1049144F", "HSF 56k Data/Fax Modem" } ,
	{ 0x14F1, 0x2044, "smarthsf+11242-11", "HSF 56k Data/Fax/Voice Modem" } ,
	{ 0x14F1, 0x2045, "11242-11", "SoftK56 Modem Controller" } ,
	{ 0x14F1, 0x2046, "1456VQH-R4", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x2053, "", "HSF Data/Fax" } ,
	{ 0x14F1, 0x2054, "", "HSF Data/Fax/TAM" } ,
	{ 0x14F1, 0x2055, "", "HSF Data/Fax/Voice/Speakerphone" } ,
	{ 0x14F1, 0x2056, "", "HSF Data/Fax/TAM/Speakerphone" } ,
	{ 0x14F1, 0x2063, "", "HSF 56k Data/Fax Modem" } ,
	{ 0x14F1, 0x2064, "11242-11", "HSF 56k Data/Fax/Voice Modem" } ,
	{ 0x14F1, 0x2065, "", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x2066, "", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x2093, "", "HSF 56k Modem" } ,
	{ 0x14F1, 0x2143, "", "HSF 56k Data/Fax/Cell Modem" } ,
	{ 0x14F1, 0x2144, "", "HSF 56k Data/Fax/Voice/Cell Modem" } ,
	{ 0x14F1, 0x2145, "", "HSF 56k Data/Fax/Voice/Spkr/Cell Modem" } ,
	{ 0x14F1, 0x2146, "", "HSF 56k Data/Fax/Voice/Spkr/Cell Modem" } ,
	{ 0x14F1, 0x2163, "", "HSF 56k Data/Fax/Cell Modem" } ,
	{ 0x14F1, 0x2164, "", "HSF 56k Data/Fax/Voice/Cell Modem" } ,
	{ 0x14F1, 0x2165, "", "HSF 56k Data/Fax/Voice/Spkr/Cell Modem" } ,
	{ 0x14F1, 0x2166, "", "HSF 56k Data/Fax/Voice/Spkr/Cell Modem" } ,
	{ 0x14F1, 0x2343, "", "HSF 56k Data/Fax CardBus Modem" } ,
	{ 0x14F1, 0x2344, "", "HSF 56k Data/Fax/Voice CardBus Modem" } ,
	{ 0x14F1, 0x2345, "r6793-11", "HSF 56k Data/Fax/Voice/Sp CardBus Modem" } ,
	{ 0x14F1, 0x2346, "", "HSF 56k Data/Fax/Voice/Sp CardBus Modem" } ,
	{ 0x14F1, 0x2363, "", "HSF 56k Data/Fax CardBus Modem" } ,
	{ 0x14F1, 0x2364, "", "HSF 56k Data/Fax/Voice CardBus Modem" } ,
	{ 0x14F1, 0x2365, "", "HSF 56k Data/Fax/Voice/Sp CardBus Modem" } ,
	{ 0x14F1, 0x2366, "", "HSF 56k Data/Fax/Voice/Sp CardBus Modem" } ,
	{ 0x14f1, 0x2443, "E30965.1", "HSF 56k Data/Fax Modem" } ,
	{ 0x14F1, 0x2444, "", "HSF 56k Data/Fax/Voice Modem" } ,
	{ 0x14F1, 0x2445, "", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x2446, "", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x2463, "", "HSF 56k Data/Fax Modem" } ,
	{ 0x14F1, 0x2464, "", "HSF 56k Data/Fax/Voice Modem" } ,
	{ 0x14F1, 0x2465, "11242-11", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14F1, 0x2466, "", "HSF 56k Data/Fax/Voice/Spkrphone Modem" } ,
	{ 0x14f1, 0x2F00, "cx11252-11", "HSF 56k HSFi" } ,
	{ 0x14f1, 0x2F10, "E124327.1", "USRobotics  56k Soft FCC PCI Faxmodem" } ,
	{ 0x14f1, 0x2F12, "", "HSF Data/Fax/Voice (USA)" } ,
	{ 0x14f1, 0x2F14, "", "56K PCI Software Modem" } ,
	{ 0x14F1, 0x8237, "CN8237", "ATM OC2 ServiceSAR+ controller" } ,
	{ 0x14F1, 0x8471, "CN8471A", "32-channel HDLC Controller" } ,
	{ 0x14F1, 0x8472, "CN8472A", "64-channel HDLC Controller" } ,
	{ 0x14F1, 0x8474, "CN8474A", "128-channel HDLC Controller" } ,
	{ 0x14F1, 0x8478, "CN8478", "256-channel HDLC Controller" } ,
	{ 0x14F1, 0x8502, "CX28500", "676-channel HDLC Controller" } ,
	{ 0x14F1, 0x8503, "CX28500", "1024-channel HDLC Controller" } ,
	{ 0x14F1, 0x8563, "CX28560", "2047-channel HDLC Controller" } ,
	{ 0x14F2, 0x0001, "", "Moselle Split Bridge" } ,
	{ 0x14F2, 0x0002, "", "Capilano Split Bridge" } ,
	{ 0x14F2, 0x0120, "", "Merlin Split Bridge" } ,
	{ 0x14F2, 0x0121, "", "PCI Parallel Port" } ,
	{ 0x14F2, 0x0122, "", "PCI Serial Port" } ,
	{ 0x14F2, 0x0123, "", "PCI PS/2 Keyboard Port" } ,
	{ 0x14F2, 0x0124, "", "PCI PS/2 Mouse Port" } ,
	{ 0x1500, 0x1300, "SIS900", "10/100M PCI Fast Ethernet Controller" } ,
	{ 0x1500, 0x1320, "VT86C100A", "10/100M PCI Fast Ethernet Controler" } ,
	{ 0x1500, 0x1360, "RTL8139A", "10/100 Mbps PCI Fast Ethernet Controller" } ,
	{ 0x1500, 0x1380, "DEC21143PD", "10/100M PCI Fast Ethernet Controller" } ,
	{ 0x1507, 0x0001, "MPC105 Eagle", "" } ,
	{ 0x1507, 0x0002, "MPC106 Grackle", "" } ,
	{ 0x1507, 0x0003, "MPC8240 Kahlua", "" } ,
	{ 0x1507, 0x0100, "MPC145575 HFC-PCI", "" } ,
	{ 0x1507, 0x0431, "KTI829c 100VG", "" } ,
	{ 0x1507, 0x4801, "Raven", "" } ,
	{ 0x1507, 0x4802, "Falcon", "" } ,
	{ 0x1507, 0x4803, "Hawk", "" } ,
	{ 0x1507, 0x4806, "CPX8216", "" } ,
	{ 0x1516, 0x0803, "803 series", "PCI Ethernet controller" } ,
	{ 0x151A, 0x1002, "PCI-1002", "" } ,
	{ 0x151A, 0x1004, "PCI-1004", "" } ,
	{ 0x151A, 0x1008, "PCI-1008", "" } ,
	{ 0x151c, 0x0002, "Xilinx PLD", "CardDeluxe Analog + Digital Audio" } ,
	{ 0x151f, 0x0000, "TP560i", "56K Data/Fax/Voice/Speaker Phone Modem" } ,
	{ 0x1522, 0x0100, "PBridge+", "PCI Interface Chip" } ,
	{ 0x153B, 0x6003, "CS4614/22/24", "CrystalClear SoundFusion PCI Audio Accel" } ,
	{ 0x1549, 0x80FF, "PCI-ISA-001", "PCI/ISA Bus Bridge" } ,
	{ 0x1555, 0x0002, "PLX PCI 9050", "Easylon PCI Bus Interface" } ,
	{ 0x1558, 0x1558, "", "" } ,
	{ 0x1562, 0x0001, "LA-41x3", "Spectrum24 Wireless LAN PCI Card" } ,
	{ 0x1571, 0xA001, "CCSI PCI20-485", "ARCnet" } ,
	{ 0x1571, 0xA002, "CCSI PCI20-485D", "ARCnet" } ,
	{ 0x1571, 0xA003, "CCSI PCI20-485X", "ARCnet" } ,
	{ 0x1571, 0xA004, "CCSI PCI20-CXB", "ARCnet" } ,
	{ 0x1571, 0xA005, "CCSI PCI20-CXS", "ARCnet" } ,
	{ 0x1571, 0xA006, "CCSI PCI20-FOG-SMA", "ARCnet" } ,
	{ 0x1571, 0xA007, "CCSI PCI20-FOG-ST", "ARCnet" } ,
	{ 0x1571, 0xA008, "CCSI PCI20-TB5", "ARCnet" } ,
	{ 0x1571, 0xA009, "CCSI PCI20-5-485", "5 Mbit ARCnet" } ,
	{ 0x1571, 0xA00A, "CCSI PCI20-5-485D", "5 Mbit ARCnet" } ,
	{ 0x1571, 0xA00B, "CCSI PCI20-5-485X", "5 Mbit ARCnet" } ,
	{ 0x1571, 0xA00C, "CCSI PIC20-5-FOG-ST", "5 Mbit ARCnet" } ,
	{ 0x1571, 0xA00D, "CCSI PCI20-5-FOG-SMA", "5 Mbit ARCnet" } ,
	{ 0x1571, 0xA201, "CCSI PCI22-485", "10 Mbit ARCnet" } ,
	{ 0x1571, 0xA202, "CCSI PCI22-485D", "10 Mbit ARCnet" } ,
	{ 0x1571, 0xA203, "CCSI PCI22-485X", "10 Mbit ARCnet" } ,
	{ 0x1571, 0xA204, "CCSI PCI22-CHB", "10 Mbit ARCnet" } ,
	{ 0x1571, 0xA205, "CCSI PCI22-FOG-ST", "10 Mbit ARCnet" } ,
	{ 0x1571, 0xA206, "CCSI PCI22-THB", "10 Mbit ARCnet" } ,
	{ 0x157C, 0x8001, "Fix2000", "PCI Y2K Compliance Card" } ,
	{ 0x1592, 0x0781, "", "Multi-IO Card" } ,
	{ 0x1592, 0x0782, "", "Dual Parallel Port Card (EPP)" } ,
	{ 0x1592, 0x0783, "TC32001 PCI I/O", "Multi-IO Card" } ,
	{ 0x1592, 0x0785, "", "Multi-IO Card" } ,
	{ 0x1592, 0x0786, "", "Multi-IO Card" } ,
	{ 0x1592, 0x0787, "winbond w83787if", "Multi-IO Card 2 series" } ,
	{ 0x1592, 0x0788, "", "Multi-IO Card" } ,
	{ 0x1592, 0x078A, "", "Multi-IO Card" } ,
	{ 0x15A2, 0x0001, "TA700", "PCI Bus Analyzer/Exerciser" } ,
	{ 0x15AD, 0x0710, "", "Virtual SVGA" } ,
	{ 0x15B0, 0x0001, "FM-1789", "Pctel" } ,
	{ 0x15B0, 0x2BD0, "2BD0", "HCF-PCI card" } ,
	{ 0x15B3, 0x5274, "MT21108", "InfiniBridge" } ,
	{ 0x15bc, 0x0100, "n2530a", "DX2 FC-AL Adapter" } ,
	{ 0x15BC, 0x0101, "n2530a", "DX2+ FC-AL Adapter" } ,
	{ 0x15bc, 0x2432, "N2530A", "Remote Management Card" } ,
	{ 0x15bc, 0x2530, "N2530A", "Agilent N2530A Service Processor" } ,
	{ 0x15bc, 0x2531, "Agilent N2530A etc.", "Remote Management Card" } ,
	{ 0x15bc, 0x2532, "N2530A", "Remote Management Card" } ,
	{ 0x15bc, 0x25A1, "N2530A", "Remote Management Card - BMC" } ,
	{ 0x15bc, 0x2922, "E2922A", "64 Bit, 133MHz PCI-X Exerciser/Observer" } ,
	{ 0x15bc, 0x2929, "E2929A", "64 Bit, 133MHz PCI-X Exerciser/Analyzer" } ,
	{ 0x15D1, 0x0001, "TC11IB", "TriCore 32-bit Single-chip Microctrlr" } ,
	{ 0x15d1, 0x0002, "PEB 20535 E v3.1", "4 Channel Serial Comm Controller (DSCC4)" } ,
	{ 0x15D1, 0x0003, "PEB 20544 E v1.1", "6 Port Optimized Comm Ctrlr (SPOCC)" } ,
	{ 0x15D1, 0x0004, "PEB 3454 E v1.1", "TE3-SPICCE 6 Port Integrated Comm Ctrlr" } ,
	{ 0x15D8, 0x9001, "", "" } ,
	{ 0x15DC, 0x0001, "Argus 300", "PCI Cryptography Module" } ,
	{ 0x15E2, 0x0500, "", "Internet PhoneJack PCI Card" } ,
	{ 0x15E8, 0x0130, "NCP130", "Wireless NIC" } ,
	{ 0x15E8, 0x0131, "Prism II", "InstantWave HR PCI card" } ,
	{ 0x15E9, 0x1841, "NetStaQ ADMA-100", "ATA controller" } ,
	{ 0x15F2, 0x0001, "Spot RT", "Spot RT Interface Board" } ,
	{ 0x15F2, 0x0002, "Spot RT #2", "Spot RT Interface Board" } ,
	{ 0x15F2, 0x0003, "Spot Insight", "Spot Insight Interface Board" } ,
	{ 0x1619, 0x0400, "FarSync T2P", "Two Port Intelligent Sync Comms Card" } ,
	{ 0x1619, 0x0440, "FarSync T4P", "Four Port Intelligent Sync Comms Card" } ,
	{ 0x1619, 0x0610, "FarSync T1U", "One Port Intelligent Sync Comms Card" } ,
	{ 0x1619, 0x0620, "FarSync T2U", "Two Port Intelligent Sync Comms Card" } ,
	{ 0x1619, 0x0640, "FarSync T4U", "Four Port Intelligent Sync Comms Card" } ,
	{ 0x1629, 0x1003, "", "Format Synchronizer v3.0" } ,
	{ 0x1629, 0x2002, "", "Fast Universal Data Output" } ,
	{ 0x162D, 0x0100, "", "Repeographics controller" } ,
	{ 0x162D, 0x0101, "", "Reprographics Controller" } ,
	{ 0x162D, 0x0102, "", "Reprographics Controller" } ,
	{ 0x162D, 0x0103, "", "Reprographics Controller" } ,
	{ 0x162F, 0x1111, "TS-PRL1", "General Purpose Relay Card" } ,
	{ 0x162F, 0x1112, "TS-PMA", "Matrix Card" } ,
	{ 0x1638, 0x1100, "WL11000P", "" } ,
	{ 0x163C, 0xFF02, "", "Aztech CNR V.92 Modem" } ,
	{ 0x165A, 0xC100, "PIXCI CL1", "PCI camera link video capture board" } ,
	{ 0x165A, 0xD200, "PIXCI D2X", "PCI digital video capture board" } ,
	{ 0x165A, 0xD300, "PIXCI D3X", "PCI digital video capture board" } ,
	{ 0x1668, 0x0100, "", "PCI to PCI Bridge" } ,
	{ 0x167F, 0x4634, "", "FOB-IO Card" } ,
	{ 0x167F, 0x4C32, "", "L2B PCI Board" } ,
	{ 0x167F, 0x5344, "", "FOB-SD Card" } ,
	{ 0x167F, 0x5443, "", "FOB-TDC Card" } ,
	{ 0x168C, 0x0007, "", "802.11a Wireless Adapter" } ,
	{ 0x1693, 0x0212, "PLX PCI9054", "EPONINE ESR-PCI Board" } ,
	{ 0x1693, 0x0213, "Motorola MPC8245", "EPONINE MTM120 PCI Board" } ,
	{ 0x16CA, 0x0001, "Rocket Drive", "Solid State Disk" } ,
	{ 0x173B, 0x03E8, "AC1000", "Gigabit Ethernet Adapter" } ,
	{ 0x1743, 0x8139, "ROL/F-100", "Fast Ethernet Adapter with ROL" } ,
	{ 0x17e9, 0x0001, "XLON PCI", "LonWorks Network Interface Card" } ,
	{ 0x1813, 0x4000, "MD5628D-L", "Modem" } ,
	{ 0x1813, 0x4100, "MD8820", "Modem" } ,
	{ 0x1888, 0x2503, "Bt881", "Video Capture (10 bit High qualtiy cap)" } ,
	{ 0x1888, 0x2504, "Bt878", "Video Capture" } ,
	{ 0x1888, 0x3503, "nVidia NV28", "VGA Geforce4 MX440" } ,
	{ 0x1888, 0x3505, "nVidia NV28", "VGA Geforce4 Ti4200" } ,
	{ 0X1898, 0X2001, "", "DVB Receiver Card" } ,
	{ 0x1A08, 0x0000, "SC15064", "" } ,
	{ 0x1C1C, 0x0001, "FR710", "EIDE Ctrlr" } ,
	{ 0x1C1C, 0x0001, "82C101", "IDE Ctrlr" } ,
	{ 0x1D44, 0xA400, "PM2x24/3224", "SCSI Adapter" } ,
	{ 0x1DE1, 0x0391, "TRM-S1040", "SCSI ASIC" } ,
	{ 0x1DE1, 0x2020, "DC-390", "SCSI Controller" } ,
	{ 0x1DE1, 0x690C, "DC-690C", "IDE Cache Controller" } ,
	{ 0x1DE1, 0xDC29, "DC290M", "Bus Master IDE PCI 2 controllers" } ,
	{ 0x1DE2, 0x1190, "001", "Slave PCI protyting board" } ,
	{ 0x1DEA, 0x0001, "701-1350", "Lasentec FBRM Counter" } ,
	{ 0x1DEA, 0x0002, "701-1429", "Lasentec PVM Controller" } ,
	{ 0x2002, 0x2003, "0x2003", "PCI Frame Grabber" } ,
	{ 0x2014, 0x0004, "aa551234", "PCI Master Target" } ,
	{ 0x2014, 0x0040, "0xff8000", "PCI to Private Bus Bridge for Primary Ra" } ,
	{ 0x2348, 0x2010, "8142", "100VG/AnyLAN" } ,
	{ 0x3388, 0x0020, "HB6", "UNIVERSAL PCI-PCI BRIDGE" } ,
	{ 0x3388, 0x0021, "HB1-SE33", "PCI-to-PCI Bridge" } ,
	{ 0x3388, 0x8011, "", "CPU to PCI Bridge" } ,
	{ 0x3388, 0x8012, "", "PCI to ISA Bridge" } ,
	{ 0x3388, 0x8013, "", "EIDE Controller" } ,
	{ 0x38D0, 0x62D6, "fieldbus-master", "PCI Measurement bus controller for EPSI" } ,
	{ 0x3D3D, 0x0001, "GLint 300SX", "3D Accelerator" } ,
	{ 0x3D3D, 0x0002, "GLint 500TX", "Sapphire 3D Accelerator" } ,
	{ 0x3D3D, 0x0003, "GLint", "Delta Geometry processor" } ,
	{ 0x3D3D, 0x0004, "3C0SX", "2D+3D Accelerator" } ,
	{ 0x3D3D, 0x0005, "Permedia", "2D+3D Accelerator" } ,
	{ 0x3D3D, 0x0006, "GLint MX", "3D Accelerator" } ,
	{ 0x3D3D, 0x0007, "3D Extreme", "Permedia II 2D+3D Accelerator" } ,
	{ 0x3D3D, 0x0008, "GLint Gamma G1", "" } ,
	{ 0x3D3D, 0x0009, "Permedia2v", "2d+3d chipset, integrated ramdac" } ,
	{ 0x3D3D, 0x000A, "Permedia 3", "" } ,
	{ 0x3D3D, 0x000C, "Permedia 3", "" } ,
	{ 0x3D3D, 0x000D, "GLINT R4", "3D Accelerator" } ,
	{ 0x3D3D, 0x000E, "GLINT Gamma G2", "" } ,
	{ 0x3D3D, 0x0100, "Permedia II", "2D+3D Accelerator" } ,
	{ 0x3d3d, 0x07A0, "Wildcat 4xxx", "Wildcat 4xxx series" } ,
	{ 0x3d3d, 0x07A1, "Wildcat 5xxx", "Wildcat 5xxx series" } ,
	{ 0x3D3D, 0x1004, "Permedia", "3D+3D Accelerator" } ,
	{ 0x3d3d, 0x1040, "Wildcat III 6110", "Wildcat III 6110" } ,
	{ 0x3d3d, 0x1041, "Wildcat III 6210", "Wildcat III 6210" } ,
	{ 0x3D3D, 0x3D04, "Permedia", "2D+3D Accelerator" } ,
	{ 0x3d3d, 0x3d07, "Permedia IIv", "Permedia IIv" } ,
	{ 0x3D3D, 0xFFFF, "GLint VGA", "" } ,
	{ 0x4005, 0x0300, "ALS300", "PCI Audio Device" } ,
	{ 0x4005, 0x0308, "ALS300+", "PCI Audio Device" } ,
	{ 0x4005, 0x0309, "ALS300+", "PCI Input Controller" } ,
	{ 0x4005, 0x1064, "ALG2064", "GUI Accelerator" } ,
	{ 0x4005, 0x2064, "<DELETE>", "GUI Accelerator" } ,
	{ 0x4005, 0x2128, "ALG2364A", "GUI Accelerator" } ,
	{ 0x4005, 0x2301, "ALG2301", "GUI Accelerator" } ,
	{ 0x4005, 0x2302, "ALG2302", "GUI Accelerator" } ,
	{ 0x4005, 0x2303, "AVG2302", "GUI Accelerator" } ,
	{ 0x4005, 0x2364, "ALG2364", "GUI Accelerator" } ,
	{ 0x4005, 0x2464, "ALG2464", "GUI Accelerator" } ,
	{ 0x4005, 0x2501, "ALG2564A/25128A", "GUI Accelerator" } ,
	{ 0x4005, 0x4000, "ALS4000", "Audio Chipset" } ,
	{ 0x4033, 0x1300, "SIS900", "10/100Mbps Fast Ethernet Controller" } ,
	{ 0x4033, 0x1320, "VT86C100A", "10/100M PCI Fast Ethernet Controller" } ,
	{ 0x4033, 0x1360, "RTL8139A", "10/100 Mbps PCI Fast Ethernet Controller" } ,
	{ 0x4033, 0x1380, "DEC 21143PD", "10/100M PCI Fast Ethernet Controller" } ,
	{ 0x4144, 0x0043, "ADM-XPL", "Virtex-II Pro Bridge" } ,
	{ 0x416C, 0x0100, "AlladinCARD", "" } ,
	{ 0x416C, 0x0200, "CPC", "" } ,
	{ 0x4444, 0x0002, "iTVC12", "MPEG Encoder Card" } ,
	{ 0x4444, 0x0803, "iTVC15", "MPEG Coder" } ,
	{ 0x4550, 0x9054, "PLX9054", "" } ,
	{ 0x494F, 0x22C0, "WDT500", "Watchdog" } ,
	{ 0x494F, 0xACA8, "PCI-AI/1216", "ADC Card" } ,
	{ 0x494F, 0xACA9, "PCI-AI/1216(M)", "ADC Card" } ,
	{ 0x4A14, 0x5000, "NV5000", "RPTI RT8029-based Ethernet Adapter" } ,
	{ 0x4B10, 0x3080, "", "SCSI Host Adapter" } ,
	{ 0x4B10, 0x4010, "", "Fast/wide SCSI-2" } ,
	{ 0x4C53, 0x0000, "PLUSTEST", "Diagnostics Device" } ,
	{ 0x4D51, 0x0200, "MQ-200", "" } ,
	{ 0x5046, 0x1001, "", "Radio FM" } ,
	{ 0x5053, 0x2010, "", "Daytona Audio Adapter" } ,
	{ 0x5112, 0x0001, "DCP401", "Intelligent Dias Master" } ,
	{ 0x5145, 0x3031, "Concert", "AudioPCI" } ,
	{ 0x5301, 0x0001, "ProMotion aT3D", "" } ,
	{ 0x5333, 0x0551, "86C551", "Plato/PX" } ,
	{ 0x5333, 0x5631, "86C325", "Virge 3D GUI Accelerator" } ,
	{ 0x5333, 0x8800, "", "Vision 866 GUI Accelerator" } ,
	{ 0x5333, 0x8801, "", "Vision 964 GUI Accelerator" } ,
	{ 0x5333, 0x8810, "86C732", "Trio 32 GUI Accelerator rev. 0" } ,
	{ 0x5333, 0x8811, "86C764/765", "Trio 64/64V+ GUI Accelerator" } ,
	{ 0x5333, 0x8812, "86CM65?", "Aurora 64V+" } ,
	{ 0x5333, 0x8813, "86C764", "Trio 32/64 GUI Accelerator v3" } ,
	{ 0x5333, 0x8814, "86C767", "Trio 64UV+ GUI Accelerator" } ,
	{ 0x5333, 0x8815, "86CM66", "Aurora128" } ,
	{ 0x5333, 0x883D, "86C988", "ViRGE/VX 3D GUI Accelerator" } ,
	{ 0x5333, 0x8870, "Fire GL", "" } ,
	{ 0x5333, 0x8880, "86C868", "Vision 868 GUI Accelerator VRAM rev. 0" } ,
	{ 0x5333, 0x8881, "86C868", "Vision 868 GUI Accelerator VRAM rev. 1" } ,
	{ 0x5333, 0x8882, "86C868", "Vision 868 GUI Accelerator VRAM rev. 2" } ,
	{ 0x5333, 0x8883, "86C868", "Vision 868 GUI Accelerator VRAM rev. 3" } ,
	{ 0x5333, 0x88B0, "86C928", "Vision 928 GUI Accelerator VRAM rev. 0" } ,
	{ 0x5333, 0x88B1, "86C928", "Vision 928 GUI Accelerator VRAM rev. 1" } ,
	{ 0x5333, 0x88B2, "86C928", "Vision 928 GUI Accelerator VRAM rev. 2" } ,
	{ 0x5333, 0x88B3, "86C928", "Vision 928 GUI Accelerator VRAM rev. 3" } ,
	{ 0x5333, 0x88C0, "86C864", "Vision 864 GUI Accelerator DRAM rev. 0" } ,
	{ 0x5333, 0x88C1, "86C864", "Vision 864 GUI Accelerator DRAM rev. 1" } ,
	{ 0x5333, 0x88C2, "86C864", "Vision 864 GUI Accelerator DRAM rev. 2" } ,
	{ 0x5333, 0x88C3, "86C864", "Vision 864 GUI Accelerator DRAM rev. 3" } ,
	{ 0x5333, 0x88D0, "86C964", "Vision 964 GUI Accelerator VRAM rev. 0" } ,
	{ 0x5333, 0x88D1, "86C964", "Vision 964-P GUI Accelerator VRAM rev. 1" } ,
	{ 0x5333, 0x88D2, "86C964", "Vision 964-P GUI Accelerator DRAM rev 2" } ,
	{ 0x5333, 0x88D3, "86C964", "Vision 964-P GUI Accelerator VRAM rev. 3" } ,
	{ 0x5333, 0x88F0, "86C968", "Vision 968 GUI Accelerator VRAM rev. 0" } ,
	{ 0x5333, 0x88F1, "86C968", "Vision 968 GUI Accelerator VRAM rev. 1" } ,
	{ 0x5333, 0x88F2, "86C968", "Vision 968 GUI Accelerator VRAM rev. 2" } ,
	{ 0x5333, 0x88F3, "86C968", "Vision 968 GUI Accelerator VRAM rev. 3" } ,
	{ 0x5333, 0x8900, "86C775", "Trio64V2/DX" } ,
	{ 0x5333, 0x8901, "86C775/86C785", "Trio 64V2 DX/GX" } ,
	{ 0x5333, 0x8902, "86C551", "SMA Family" } ,
	{ 0x5333, 0x8903, "", "TrioV Family" } ,
	{ 0x5333, 0x8904, "86C365/366", "Trio3D" } ,
	{ 0x5333, 0x8905, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x8906, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x8907, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x8908, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x8909, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x890A, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x890B, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x890C, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x890D, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x890E, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x890F, "", "86C765 Trio64V+ compatible" } ,
	{ 0x5333, 0x8A01, "86C375/86C385", "ViRGE /DX & /GX" } ,
	{ 0x5333, 0x8A10, "86C357/86C359", "ViRGE /GX2 & /GX2+" } ,
	{ 0x5333, 0x8A11, "86C359", "ViRGE /GX2+ Macrovision" } ,
	{ 0x5333, 0x8A12, "86C359", "ViRGE /GX2+" } ,
	{ 0x5333, 0x8A13, "86C362/86C368", "Trio3D2x & Trio3D2x+ AGP" } ,
	{ 0x5333, 0x8A20, "86C390/391", "Savage3D" } ,
	{ 0x5333, 0x8A21, "86C390", "Savage3D/MV" } ,
	{ 0x5333, 0x8A22, "86C394-397", "Savage 4" } ,
	{ 0x5333, 0x8A23, "86C394-397", "Savage 4" } ,
	{ 0x5333, 0x8A25, "86C370", "Savage4" } ,
	{ 0x5333, 0x8A26, "86C395B", "ProSavage" } ,
	{ 0x5333, 0x8C00, "85C260", "ViRGE/M3 (ViRGE/MX)" } ,
	{ 0x5333, 0x8C01, "86C260", "ViRGE/M5 (ViRGE/MX)" } ,
	{ 0x5333, 0x8C02, "86C240", "ViRGE/MXC" } ,
	{ 0x5333, 0x8C03, "86C280", "ViRGE /MX+ Macrovision" } ,
	{ 0x5333, 0x8C10, "86C270/274/290/294", "Savage MX/IX/MX+MV/IX+MV" } ,
	{ 0x5333, 0x8C12, "86C270/274/290/294", "Savage MX/IX/MX+MV/IX+MV" } ,
	{ 0x5333, 0x8C22, "86C508", "SuperSavage 128/MX" } ,
	{ 0x5333, 0x8C2A, "86C544", "SuperSavage 128/IX" } ,
	{ 0x5333, 0x8C2B, "86C553", "SuperSavage 128/IX DDR" } ,
	{ 0x5333, 0x8C2C, "86C564", "SuperSavage/IX" } ,
	{ 0x5333, 0x8C2D, "86C573", "SuperSavage/IX DDR" } ,
	{ 0x5333, 0x8C2E, "86C584", "SuperSavage/IXC SDRAM" } ,
	{ 0x5333, 0x8C2F, "86C594", "SuperSavage/IXC DDR" } ,
	{ 0x5333, 0x8D01, "Twister", "" } ,
	{ 0x5333, 0x8D02, "Twister-K", "" } ,
	{ 0x5333, 0x8D04, "P4M266", "ProSavage DDR" } ,
	{ 0x5333, 0x9102, "86C410", "Savage 2000" } ,
	{ 0x5333, 0xCA00, "86C617", "SonicVibes PCI Audio Accelerator" } ,
	{ 0x5356, 0x4002, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4102, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4202, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4302, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4402, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4502, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4602, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4702, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4802, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4902, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4A02, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4B02, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4C02, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4D02, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4E02, "", "ULTRA24 SCSI Host" } ,
	{ 0x5356, 0x4F02, "", "ULTRA24 SCSI Host" } ,
	{ 0x5401, 0x0101, "DSSS", "Wireless LAN PCI Card" } ,
	{ 0x5430, 0x0100, "AcceleraPCI", "Upgrade Card Adapter" } ,
	{ 0x5455, 0x4458, "S5933", "PCI-MyBus-Bridge" } ,
	{ 0x5544, 0x0001, "I-30xx", "Scanner Interface" } ,
	{ 0x5555, 0x0003, "", "TURBOstor HFP-832 HiPPI NIC" } ,
	{ 0x55cF, 0x20D1, "", "UNISYS DCHA5" } ,
	{ 0x55CF, 0x2111, "", "UNISYS CP150P" } ,
	{ 0x6356, 0x4002, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4102, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4202, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4302, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4402, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4502, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4602, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4702, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4802, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4902, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4A02, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4B02, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4C02, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4D02, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4E02, "", "ULTRA24 SCSI Host" } ,
	{ 0x6356, 0x4F02, "", "ULTRA24 SCSI Host" } ,
	{ 0x6374, 0x6773, "GPPCI", "PCI Interface" } ,
	{ 0x6666, 0x0001, "", "PCCOM4" } ,
	{ 0x6666, 0x0002, "", "PCCOM8" } ,
	{ 0x8001, 0x0010, "ispLSI1032E", "PCI-decoder" } ,
	{ 0x8008, 0x0010, "PWDOG1/2", "PCI-Watchdog 1" } ,
	{ 0x8008, 0x0011, "PWDOG1/2", "Watchdog2/PCI" } ,
	{ 0x8008, 0x0016, "PROTO2", "" } ,
	{ 0x8008, 0x0100, "PREL8", "" } ,
	{ 0x8008, 0x0102, "PREL16", "" } ,
	{ 0x8008, 0x0103, "POPTOREL16", "" } ,
	{ 0x8008, 0x0105, "POPTO16IN", "" } ,
	{ 0x8008, 0x0106, "PTTL24IO", "" } ,
	{ 0x8008, 0x0107, "PUNIREL", "" } ,
	{ 0x8008, 0x1000, "PDAC4", "" } ,
	{ 0x8008, 0x1001, "PAD12DAC4", "" } ,
	{ 0x8008, 0x1002, "PAD16DAC4", "" } ,
	{ 0x8008, 0x1005, "PAD12", "" } ,
	{ 0x8008, 0x1006, "PAD16", "" } ,
	{ 0x8008, 0x3000, "POPTOLCA", "" } ,
	{ 0x801F, 0x0020, "", "" } ,
	{ 0x801F, 0x0040, "", "" } ,
	{ 0x8086, 0x0008, "", "Extended Express System Support Ctrlr" } ,
	{ 0x8086, 0x0309, "80303", "I/O Processor PCI-to-PCI Bridge Unit" } ,
	{ 0x8086, 0x030D, "80312", "I/O Companion Unit PCI-to-PCI Bridge" } ,
	{ 0x8086, 0x0482, "82375EB", "PCI-EISA Bridge (PCEB)" } ,
	{ 0x8086, 0x0483, "82424TX/ZX", "CPU (i486) Bridge (Saturn)" } ,
	{ 0x8086, 0x0484, "82378ZB/IB", "SIO ISA Bridge" } ,
	{ 0x8086, 0x0486, "82425EX", "PCI System Controller (PSC) for i486 (Aries)" } ,
	{ 0x8086, 0x04A3, "82434LX", "CPU (Pentium) Bridge (Mercury)" } ,
	{ 0x8086, 0x0500, "E8870", "Processor Bus Controller" } ,
	{ 0x8086, 0x0501, "E8870", "Memory Controller" } ,
	{ 0x8086, 0x0502, "E8870", "Scalability Port 0" } ,
	{ 0x8086, 0x0503, "E8870", "Scalability Port 1 / Glob. Perf. Monitor" } ,
	{ 0x8086, 0x0510, "E8870IO", "Hub Interface Port 0 (8-bit compatible)" } ,
	{ 0x8086, 0x0511, "E8870IO", "Hub Interface Port 1" } ,
	{ 0x8086, 0x0512, "E8870IO", "Hub Interface Port 2" } ,
	{ 0x8086, 0x0513, "E8870IO", "Hub Interface Port 3" } ,
	{ 0x8086, 0x0514, "E8870IO", "Hub Interface Port 4" } ,
	{ 0x8086, 0x0515, "E8870IO", "Server I/O Hub (SIOH)" } ,
	{ 0x8086, 0x0516, "E8870IO", "Reliabilty, Availability, Serviceability" } ,
	{ 0x8086, 0x0530, "E8870SP", "Scalability Port" } ,
	{ 0x8086, 0x0531, "E8870SP", "Scalability Port" } ,
	{ 0x8086, 0x0532, "E8870SP", "Scalability Port" } ,
	{ 0x8086, 0x0533, "E8870SP", "Scalability Port" } ,
	{ 0x8086, 0x0534, "E8870SP", "Scalability Port" } ,
	{ 0x8086, 0x0535, "E8870SP", "Scalability Port" } ,
	{ 0x8086, 0x0536, "E8870SP", "Scalability Port Switch Global Registers" } ,
	{ 0x8086, 0x0537, "E8870SP", "Interleave Configuration Registers" } ,
	{ 0x8086, 0x0600, "", "Storage RAID Controller" } ,
	{ 0x8086, 0x0960, "80960RP", "i960 RP Microprocessor/Bridge" } ,
	{ 0x8086, 0x0962, "80960RM/RN", "i960RM/RN Microprocessor/Bridge" } ,
	{ 0x8086, 0x0964, "80960RP", "i960 RP Microprocessor Bridge" } ,
	{ 0x8086, 0x1000, "82542", "Gigabit Ethernet Controller" } ,
	{ 0x8086, 0x1001, "82543GC", "10/100/1000 Ethernet Controller" } ,
	{ 0x8086, 0x1002, "", "Pro 100 LAN+Modem 56 CardBus II" } ,
	{ 0x8086, 0x1004, "82543GC", "Gigabit Ethernet Controller" } ,
	{ 0x8086, 0x1008, "82544EI/GC", "Gigabit Ethernet Controller (Copper)" } ,
	{ 0x8086, 0x1009, "82544EI", "Gigabit Ethernet Controller" } ,
	{ 0x8086, 0x100C, "82544 T", "Gigabit Ethernet Controller" } ,
	{ 0x8086, 0x100D, "82544GC", "Gigabit Ethernet Controller" } ,
	{ 0x8086, 0x100E, "82544XT", "Gigabit Ethernet Controller" } ,
	{ 0x8086, 0x100F, "82545EM", "Gigabit Ethernet Controller (copper)" } ,
	{ 0x8086, 0x1010, "82546EB", "Gigabit Ethernet Controller (copper)" } ,
	{ 0x8086, 0x1011, "82545EM", "Gigabit Ethernet Controller (fiber)" } ,
	{ 0x8086, 0x1012, "82546EB", "Gigabit Ethernet Controller (fiber)" } ,
	{ 0x8086, 0x1015, "82540EM", "PRO/1000 MT Mobile Connection" } ,
	{ 0x8086, 0x1029, "82559", "Fast Ethernet PCI/CardBus Controller" } ,
	{ 0x8086, 0x1030, "82559", "PCI Networking device" } ,
	{ 0x8086, 0x1031, "82801CAM", "PRO/100 VE Network Connection" } ,
	{ 0x8086, 0x1032, "", "PRO/100 VE Network Connection" } ,
	{ 0x8086, 0x1033, "", "PRO/100 VM Network Connection" } ,
	{ 0x8086, 0x1034, "", "PRO/100 VM Network Connection" } ,
	{ 0x8086, 0x1035, "82562EH", "Phoneline Network Connection" } ,
	{ 0x8086, 0x1036, "82562EH", "Phoneline Network Connection" } ,
	{ 0x8086, 0x1038, "82559 PRO/100 VM", "Networking device" } ,
	{ 0x8086, 0x1039, "82801DB", "LAN Controller with 82562ET/EZ PHY" } ,
	{ 0x8086, 0x103A, "82801DB", "LAN Controller with 82562ET/EZ (CNR) PHY" } ,
	{ 0x8086, 0x103B, "82801DB", "LAN Controller with 82562EM/EX PHY" } ,
	{ 0x8086, 0x103C, "82801DB", "LAN Controller with 82562EM/EX (CNR) PHY" } ,
	{ 0x8086, 0x103D, "82801DB", "PRO/100 VE Network Connection" } ,
	{ 0x8086, 0x103E, "82801DB", "PRO/100 VM Network Connection" } ,
	{ 0x8086, 0x1040, "536EP", "V.92 PCI (DSP) Data Fax Modem" } ,
	{ 0x8086, 0x1042, "", "PRO/Wireless 2011 LAN PCI Card" } ,
	{ 0x8086, 0x1059, "82551QM", "PRO/100 M Mobile Connection" } ,
	{ 0x8086, 0x1100, "82815", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x1101, "82815", "AGP Bridge" } ,
	{ 0x8086, 0x1102, "82815", "Internal Graphics Device" } ,
	{ 0x8086, 0x1110, "82815", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x1112, "82815", "Internal Graphics Device" } ,
	{ 0x8086, 0x1120, "82815", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x1121, "82815", "AGP Bridge" } ,
	{ 0x8086, 0x1130, "82815/82815EM/EP", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x1131, "82815/82815EM/EP", "AGP Bridge" } ,
	{ 0x8086, 0x1132, "82815", "Internal Graphics Device" } ,
	{ 0x8086, 0x1161, "82806AA", "I/O APIC Device" } ,
	{ 0x8086, 0x1162, "BECC", "XScale 80200 Companion Chip (FPGA)" } ,
	{ 0x8086, 0x1209, "82559ER", "Ethernet Controller" } ,
	{ 0x8086, 0x1221, "82092AA", "PCMCIA Bridge" } ,
	{ 0x8086, 0x1222, "82092AA", "IDE Ctrlr" } ,
	{ 0x8086, 0x1223, "SAA7116", "Video Controller" } ,
	{ 0x8086, 0x1225, "82452KX/GX", "Orion Extended Express CPU to PCI Bridge" } ,
	{ 0x8086, 0x1226, "82596", "EtherExpress PRO/10" } ,
	{ 0x8086, 0x1227, "82865", "EtherExpress PRO100" } ,
	{ 0x8086, 0x1228, "EE PRO/100 Smart", "Intelligent 10/100 Fast Ethernet Adapter" } ,
	{ 0x8086, 0x1229, "82557/8/9/0/1", "Fast Ethernet LAN Controller" } ,
	{ 0x8086, 0x122D, "82437FX", "System Controller (TSC)" } ,
	{ 0x8086, 0x122E, "82371FB", "PCI to ISA Bridge (Triton)" } ,
	{ 0x8086, 0x1230, "82371FB", "IDE Interface (Triton)" } ,
	{ 0x8086, 0x1231, "", "DSVD Modem" } ,
	{ 0x8086, 0x1234, "82371MX", "Mobile PCI I/O IDE Xcelerator (MPIIX)" } ,
	{ 0x8086, 0x1235, "82437MX", "Mobile System Controller (MTSC)" } ,
	{ 0x8086, 0x1237, "82441FX", "PCI & Memory Controller (PMC)" } ,
	{ 0x8086, 0x1239, "82371FB", "IDE Interface (Triton)" } ,
	{ 0x8086, 0x123B, "82380PB", "PCI to PCI Docking Bridge" } ,
	{ 0x8086, 0x123C, "82380AB", "Mobile PCI-to-ISA Bridge (MISA)" } ,
	{ 0x8086, 0x123D, "683053", "Programmable Interrupt Device" } ,
	{ 0x8086, 0x123E, "82466GX", "Integrated Hot-Plug Controller (IHPC)" } ,
	{ 0x8086, 0x123F, "82466GX", "Integrated Hot-Plug Controller (IHPC)" } ,
	{ 0x8086, 0x1240, "82752", "AGP Graphics Accelerator" } ,
	{ 0x8086, 0x124B, "82380FB", "Mobile PCI-to-PCI Bridge (MPCI2)" } ,
	{ 0x8086, 0x1250, "82439HX", "System Controller (TXC)" } ,
	{ 0x8086, 0x1360, "82806AA", "Hub Interface to PCI Bridge" } ,
	{ 0x8086, 0x1361, "82806AA", "Advanced Interrupt Controller" } ,
	{ 0x8086, 0x1460, "82870P2", "Hub Interface-to-PCI Bridge" } ,
	{ 0x8086, 0x1461, "82870P2", "I/OxAPIC Interrupt Controller" } ,
	{ 0x8086, 0x1462, "82870P2", "Hot Plug Controller" } ,
	{ 0x8086, 0x1960, "80960RP", "i960RP Microprocessor" } ,
	{ 0x8086, 0x1A20, "82840", "" } ,
	{ 0x8086, 0x1A21, "82840", "Host-Hub Interface A Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x1A22, "82840", "Host to I/O Hub Bridge (Quad PCI)" } ,
	{ 0x8086, 0x1A23, "82840", "AGP Bridge" } ,
	{ 0x8086, 0x1A24, "82840", "Hub Interface B Bridge" } ,
	{ 0x8086, 0x1A30, "82845[E/MP/MZ]", "Host-Hub Interface Bridge" } ,
	{ 0x8086, 0x1A31, "82845[MP/MZ]", "AGP Bridge" } ,
	{ 0x8086, 0x2125, "82801AB", "AC97 Audio Controller" } ,
	{ 0x8086, 0x2410, "82801AA", "LPC Interface" } ,
	{ 0x8086, 0x2411, "82801AA", "IDE Controller (UltraATA/66)" } ,
	{ 0x8086, 0x2412, "82801AA", "USB Controller" } ,
	{ 0x8086, 0x2413, "82801AA", "SMBus Controller" } ,
	{ 0x8086, 0x2415, "82801AA", "AC97 Audio Controller" } ,
	{ 0x8086, 0x2416, "82801AA", "AC97 Modem Controller" } ,
	{ 0x8086, 0x2418, "82801AA", "Hub Interface-to-PCI Bridge" } ,
	{ 0x8086, 0x2420, "82801AB", "LPC Interface" } ,
	{ 0x8086, 0x2421, "82801AB", "IDE Controller (UltraATA/33)" } ,
	{ 0x8086, 0x2422, "82801AB", "USB Controller" } ,
	{ 0x8086, 0x2423, "82801AB", "SMBus Controller" } ,
	{ 0x8086, 0x2425, "82801AB", "AC97 Audio Controller" } ,
	{ 0x8086, 0x2426, "82801AB", "AC97 Modem Controller" } ,
	{ 0x8086, 0x2428, "82801AB", "Hub Interface-to-PCI Bridge" } ,
	{ 0x8086, 0x2440, "82801BA", "LPC Interface Bridge, ICH2" } ,
	{ 0x8086, 0x2441, "82801BA", "IDE Controller (UltraATA/66)" } ,
	{ 0x8086, 0x2442, "82801BA/BAM", "USB Controller, USB-A" } ,
	{ 0x8086, 0x2443, "82801BA/BAM", "SMBus Controller" } ,
	{ 0x8086, 0x2444, "82801BA/BAM", "USB Controller, USB-B" } ,
	{ 0x8086, 0x2445, "82801BA/BAM", "Avance AC97 Audio Controller" } ,
	{ 0x8086, 0x2446, "82801BA/BAM", "AC97 Modem Controller" } ,
	{ 0x8086, 0x2448, "82801BAM/CAM/DBM", "Hub Interface to PCI Bridge" } ,
	{ 0x8086, 0x2449, "82801BA/BAM/CA", "LAN Controller" } ,
	{ 0x8086, 0x244A, "82801BAM", "IDE Controller" } ,
	{ 0x8086, 0x244B, "82801BA", "IDE Controller" } ,
	{ 0x8086, 0x244C, "82801BAM", "LPC Interface Bridge" } ,
	{ 0x8086, 0x244E, "82801BA/CA/DB", "Hub Interface to PCI Bridge" } ,
	{ 0x8086, 0x2450, "82801E", "LPC Interface Bridge" } ,
	{ 0x8086, 0x2452, "82801E", "USB Controller" } ,
	{ 0x8086, 0x2453, "82801E", "SMBus Controller" } ,
	{ 0x8086, 0x2459, "82801E", "LAN0 Controller" } ,
	{ 0x8086, 0x245B, "82801E", "IDE Controller" } ,
	{ 0x8086, 0x245D, "82801E", "LAN1 Controller" } ,
	{ 0x8086, 0x245E, "82801E", "Hub Interface to PCI Bridge" } ,
	{ 0x8086, 0x2480, "82801CA", "LPC Interface Bridge" } ,
	{ 0x8086, 0x2481, "82801CA", "IDE Controller (UltraATA/66)" } ,
	{ 0x8086, 0x2482, "82801CA/CAM", "USB Controller" } ,
	{ 0x8086, 0x2483, "82801CA/CAM", "SMBus Controller" } ,
	{ 0x8086, 0x2484, "82801CA/CAM", "USB Controller" } ,
	{ 0x8086, 0x2485, "82801CA/CAM", "AC97 Audio Controller" } ,
	{ 0x8086, 0x2486, "82801CA/CAM", "AC 97 Modem Controller" } ,
	{ 0x8086, 0x2487, "82801CA/CAM", "USB Controller" } ,
	{ 0x8086, 0x248A, "82801CAM", "UltraATA IDE Controller" } ,
	{ 0x8086, 0x248B, "82801CA", "UltraATA/100 IDE Controller" } ,
	{ 0x8086, 0x248C, "82801CAM", "LPC Interface or ISA bridge: see Notes" } ,
	{ 0x8086, 0x248D, "82801??", "USB 2.0 EHCI Contoroller" } ,
	{ 0x8086, 0x24C0, "82801DB", "LPC Interface Bridge" } ,
	{ 0x8086, 0x24C2, "82801DB/DBM", "USB UHCI Controller #1" } ,
	{ 0x8086, 0x24C3, "82801DB/DBM", "SMBus Controller" } ,
	{ 0x8086, 0x24C4, "82801DB/DBM", "USB UHCI Controller #2" } ,
	{ 0x8086, 0x24C5, "82801DB/DBM", "AC97 Audio Controller" } ,
	{ 0x8086, 0x24C6, "82801DB/DBM", "AC97 Modem Controller" } ,
	{ 0x8086, 0x24C7, "82801DB/DBM", "USB UHCI Controller #3" } ,
	{ 0x8086, 0x24CA, "82801DBM", "IDE Controller (UltraATA/100)" } ,
	{ 0x8086, 0x24CB, "82801DB", "IDE Controller (UltraATA/100)" } ,
	{ 0x8086, 0x24CC, "82801DBM", "LPC Interface Bridge" } ,
	{ 0x8086, 0x24CD, "82801DB/DBM", "USB EHCI Controller" } ,
	{ 0x8086, 0x2500, "82820", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x2501, "82820", "Host Bridge (MCH)" } ,
	{ 0x8086, 0x2502, "82820", "" } ,
	{ 0x8086, 0x2503, "82820", "" } ,
	{ 0x8086, 0x2504, "82820", "" } ,
	{ 0x8086, 0x250B, "82820", "Host Bridge (MCH)" } ,
	{ 0x8086, 0x250F, "82820", "AGP Bridge" } ,
	{ 0x8086, 0x2520, "82805AA", "Memory Translator Hub (MTH)" } ,
	{ 0x8086, 0x2521, "82804AA", "Memory Repeater Hub for SDRAM (MRH-S)" } ,
	{ 0x8086, 0x2530, "82850/850E", "Host-Hub Interface Bridge" } ,
	{ 0x8086, 0x2531, "82860", "Host-Hub Interface_A Bridge (DP mode)" } ,
	{ 0x8086, 0x2532, "82850/850E/860", "AGP Bridge" } ,
	{ 0x8086, 0x2533, "82860", "Hub Interface_B Bridge" } ,
	{ 0x8086, 0x2534, "82860", "Hub Interface_C Bridge" } ,
	{ 0x8086, 0x2535, "82860", "PCI Bridge" } ,
	{ 0x8086, 0x2536, "82860", "PCI Bridge" } ,
	{ 0x8086, 0x2539, "82860", "(Quad Processor mode)" } ,
	{ 0x8086, 0x2540, "E7500", "Host-HI Bridge & DRAM Controller" } ,
	{ 0x8086, 0x2541, "E7500/E7501", "DRAM Controller Error Reporting" } ,
	{ 0x8086, 0x2543, "E7500/E7501", "HI_B Virtual PCI-to-PCI Bridge" } ,
	{ 0x8086, 0x2544, "E7500/E7501", "HI_B PCI-to-PCI Bridge Error Reporting" } ,
	{ 0x8086, 0x2545, "E7500/E7501", "HI_C Virtual PCI-to-PCI Bridge" } ,
	{ 0x8086, 0x2546, "E7500/E7501", "HI_C PCI-to-PCI Bridge Error Reporting" } ,
	{ 0x8086, 0x2547, "E7500/E7501", "HI_D Virtual PCI-to-PCI Bridge" } ,
	{ 0x8086, 0x2548, "E7500/E7501", "HI_D PCI-to-PCI Bridge Error Reporting" } ,
	{ 0x8086, 0x254C, "E7501", "Host Controller" } ,
	{ 0x8086, 0x2550, "E7505", "Host Controller" } ,
	{ 0x8086, 0x2551, "E7205/E7505", "Host RAS Controller" } ,
	{ 0x8086, 0x2552, "E7205/E7505", "PCI-to-AGP Bridge" } ,
	{ 0x8086, 0x2553, "E7505", "Hub Interface_B PCI-to-PCI Bridge" } ,
	{ 0x8086, 0x2554, "E7505", "Hub I/F_B PCI-to-PCI Bridge Error Report" } ,
	{ 0x8086, 0x255d, "E7205", "Host Controller" } ,
	{ 0x8086, 0x2560, "82845G/GL/GV/GE/PE", "DRAM Controller / Host-Hub I/F Bridge" } ,
	{ 0x8086, 0x2561, "82845G/GL/GV/GE/PE", "Host-to-AGP Bridge" } ,
	{ 0x8086, 0x2562, "82845G/GL/GV/GE/PE", "Integrated Graphics Device" } ,
	{ 0x8086, 0x3092, "SRCU32", "I2O 1.5 RAID Controller" } ,
	{ 0x8086, 0x3200, "31244", "PCI-X to Serial ATA Controller" } ,
	{ 0x8086, 0x3575, "82830[MP]", "Host-Hub I/F Bridge / SDRAM Controller" } ,
	{ 0x8086, 0x3576, "82830M/MP", "Host-AGP Bridge" } ,
	{ 0x8086, 0x3577, "82830M/MG", "Integrated Graphics Device" } ,
	{ 0x8086, 0x3578, "82830[MP]", "CPU to I/O Bridge" } ,
	{ 0x8086, 0x3580, "852GM", "Host-Hub Interface Bridge" } ,
	{ 0x8086, 0x3582, "852GM", "Integrated Graphics Device" } ,
	{ 0x8086, 0x3584, "852GM", "System Memory Controller" } ,
	{ 0x8086, 0x3585, "852GM", "Configuration Process" } ,
	{ 0x8086, 0x4000, "Creatix", "V.90 HaM Modem" } ,
	{ 0x8086, 0x5001, "PRO/DSL 2100", "Modem - PPP" } ,
	{ 0x8086, 0x5005, "PRO/DSL 2200", "Modem - PPPoA" } ,
	{ 0x8086, 0x5200, "", "PCI to PCI Bridge" } ,
	{ 0x8086, 0x5201, "", "Network Controller" } ,
	{ 0x8086, 0x5309, "80303", "I/O Processor Address Translation Unit" } ,
	{ 0x8086, 0x530D, "80312", "I/O Companion Unit Address Translation" } ,
	{ 0x8086, 0x6960, "", "EHCI 960 emulator" } ,
	{ 0x8086, 0x7000, "82371SB", "PIIX3 PCI-to-ISA Bridge (Triton II)" } ,
	{ 0x8086, 0x7010, "82371SB", "PIIX3 IDE Interface (Triton II)" } ,
	{ 0x8086, 0x7020, "82371SB", "PIIX3 USB Host Controller (Triton II)" } ,
	{ 0x8086, 0x7030, "82437VX", "System Controller" } ,
	{ 0x8086, 0x7051, "PB 642365-003", "Intel Business Video Conferencing Card" } ,
	{ 0x8086, 0x7100, "82439TX", "System Controller (MTXC), part of 430TX chipset" } ,
	{ 0x8086, 0x7110, "82371AB/EB/MB", "PIIX4/4E/4M ISA Bridge" } ,
	{ 0x8086, 0x7111, "82371AB/EB/MB", "PIIX4/4E/4M IDE Controller" } ,
	{ 0x8086, 0x7112, "82371AB/EB/MB", "PIIX4/4E/4M USB Interface" } ,
	{ 0x8086, 0x7113, "82371AB/EB/MB", "PIIX4/4E/4M Power Management Controller" } ,
	{ 0x8086, 0x7120, "82810", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x7121, "82810", "Graphics Device" } ,
	{ 0x8086, 0x7122, "82810-DC100", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x7123, "82810-DC100", "Graphics Device" } ,
	{ 0x8086, 0x7124, "82810E", "Host-Hub Interface Bridge / DRAM Ctrlr" } ,
	{ 0x8086, 0x7125, "82810E", "Intel Direct AGP 810 Chipset" } ,
	{ 0x8086, 0x7126, "82810-DC133", "Host Bridge and Memory Controller Hub" } ,
	{ 0x8086, 0x7127, "82810-DC133", "Graphics Device (FSB 133 MHz)" } ,
	{ 0x8086, 0x7128, "82810-M DC-100", "Host Bridge and Memory Controller Hub" } ,
	{ 0x8086, 0x712A, "82810-M DC-133", "Host Bridge and Memory Controller Hub" } ,
	{ 0x8086, 0x7180, "82443LX/EX (PAC)", "Host/PCI bridge in 440LX/EX AGP chipset" } ,
	{ 0x8086, 0x7181, "", "AGP device in 440LX/EX AGP chipset" } ,
	{ 0x8086, 0x7182, "440LX/EX", "" } ,
	{ 0x8086, 0x7190, "82443BX/ZX", "440BX/ZX AGPset Host Bridge" } ,
	{ 0x8086, 0x7191, "82443BX/ZX", "440BX/ZX AGPset PCI-to-PCI bridge" } ,
	{ 0x8086, 0x7192, "82443BX/ZX", "440BX/ZX chipset Host-to-PCI Bridge" } ,
	{ 0x8086, 0x7194, "82443MX", "I/O Controller?" } ,
	{ 0x8086, 0x7195, "82443MX?", "AC97 Audio Controller" } ,
	{ 0x8086, 0x7196, "82440 - 443MX", "AC97 Modem Controller (Winmodem)" } ,
	{ 0x8086, 0x7198, "82443MX", "PCI to ISA Bridge" } ,
	{ 0x8086, 0x7199, "82443MX", "EIDE Controller" } ,
	{ 0x8086, 0x719A, "82443MX", "USB Universal Host Controller" } ,
	{ 0x8086, 0x719B, "82443MX", "Power Management Controller" } ,
	{ 0x8086, 0x71A0, "82443GX", "Host-to-PCI Bridge" } ,
	{ 0x8086, 0x71A1, "82443GX", "PCI-to-PCI Bridge (AGP)" } ,
	{ 0x8086, 0x71A2, "82443GX", "Host-to-PCI Bridge" } ,
	{ 0x8086, 0x7600, "82372FB/82468GX", "LPC/FWH Interface" } ,
	{ 0x8086, 0x7601, "82372FB/82468GX", "EIDE Controller" } ,
	{ 0x8086, 0x7602, "82372FB/82468GX", "USB Host Controller" } ,
	{ 0x8086, 0x7603, "82372FB/82468GX", "SM Bus Controller" } ,
	{ 0x8086, 0x7605, "82372FB", "IEEE1394 OpenHCI Host Controller" } ,
	{ 0x8086, 0x7800, "82740", "AGP Graphics Accelerator" } ,
	{ 0x8086, 0x84C4, "82454KX/GX", "450KX/GX PCI Bridge (Orion)" } ,
	{ 0x8086, 0x84C5, "82453KX/GX", "450KX/GX Memory Controller (Orion)" } ,
	{ 0x8086, 0x84CA, "82451NX", "450NX PCIset Memory & I/O Controller" } ,
	{ 0x8086, 0x84CB, "82454NX/82467GX", "PCI Expander Bridge" } ,
	{ 0x8086, 0x84E0, "82461GX", "System Address controller" } ,
	{ 0x8086, 0x84E1, "82462GX", "System Data Controller" } ,
	{ 0x8086, 0x84E2, "82465GX", "Graphics Expander Bridge" } ,
	{ 0x8086, 0x84E3, "82463GX", "Memory Address Controller" } ,
	{ 0x8086, 0x84E4, "82464GX", "Memory Data Controller" } ,
	{ 0x8086, 0x84E6, "82466GX", "Wide and fast PCI eXpander Bridge" } ,
	{ 0x8086, 0x84EA, "82460GX", "AGP Bridge (GXB function 1)" } ,
	{ 0x8086, 0x9620, "", "I2O RAID PCI to PCI Bridge" } ,
	{ 0x8086, 0x9621, "SRCU21", "I2O 1.5 RAID Controller" } ,
	{ 0x8086, 0x9622, "SRCUxx", "I2O 1.5 RAID Controller" } ,
	{ 0x8086, 0x9641, "SRCU31", "I2O 1.5 RAID Controller" } ,
	{ 0x8086, 0x96A1, "SRCU31L", "I2O 1.5 RAID Controller" } ,
	{ 0x8086, 0xB152, "S21152BB", "PCI to PCI Bridge" } ,
	{ 0x8086, 0xB154, "S21154AE/BE", "PCI to PCI Bridge" } ,
	{ 0x8086, 0xB555, "21555", "Non-Transparent PCI-to-PCI Bridge" } ,
	{ 0x8800, 0x2008, "", "video assistant component" } ,
	{ 0x8912, 0x0001, "", "" } ,
	{ 0x8E2E, 0x3000, "Et32/Px", "Ethernet Adapter" } ,
	{ 0x9004, 0x0078, "aic-7880p", "AHA-2940UW/CN" } ,
	{ 0x9004, 0x1078, "AIC-7810C", "RAID Coprocessor" } ,
	{ 0x9004, 0x1160, "AIC-1160", "Fibre Channel Adapter" } ,
	{ 0x9004, 0x2178, "AIC-7821", "SCSI Controller" } ,
	{ 0x9004, 0x3860, "", "AIC-2930U Ultra SCSI Ctrlr" } ,
	{ 0x9004, 0x3B78, "AHA-4944W/4944UW", "QuadChannel Fast-Wide/Ultra-Wide Diff. SCSI Ctrlr" } ,
	{ 0x9004, 0x5075, "AIC-755x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5078, "AIC-7850P", "Fast/Wide SCSI Controller" } ,
	{ 0x9004, 0x5175, "AIC-755x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5178, "AIC-7850", "FAST-SCSI Ctrlr" } ,
	{ 0x9004, 0x5275, "AIC-755x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5278, "AIC-7850", "Fast SCSI Ctrlr" } ,
	{ 0x9004, 0x5375, "AIC-755x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5378, "AIC-7850", "Fast SCSI Ctrlr" } ,
	{ 0x9004, 0x5475, "AIC-755x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5478, "AIC-7850", "Fast SCSI Ctrlr" } ,
	{ 0x9004, 0x5575, "AVA-2930", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5578, "AIC-7855", "Fast SCSI Ctrlr" } ,
	{ 0x9004, 0x5675, "AIC-755x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5678, "AIC-7856", "Fast SCSI Ctrlr" } ,
	{ 0x9004, 0x5775, "AIC-755x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x5778, "AIC-7850", "Fast SCSI Ctrlr" } ,
	{ 0x9004, 0x5800, "AIC-5800", "PCI-to-1394 Ctrlr" } ,
	{ 0x9004, 0x5900, "ANA-5910/30/40", "ATM155 & 25 LAN Controller" } ,
	{ 0x9004, 0x5905, "ANA-5910A/30A/40A", "ATM Adpater" } ,
	{ 0x9004, 0x6038, "AHA-2930C", "Ultra SCSI Adpater (VAR)" } ,
	{ 0x9004, 0x6075, "AIC-7560?", "CardBus Ultra SCSI Controller" } ,
	{ 0x9004, 0x6078, "AIC-7860", "PCI SCSI Controller" } ,
	{ 0x9004, 0x6178, "AIC-7861", "PCI SCSI Controller" } ,
	{ 0x9004, 0x6278, "AIC-7860", "SCSI Ctrlr" } ,
	{ 0x9004, 0x6378, "AIC-7860", "SCSI Ctrlr" } ,
	{ 0x9004, 0x6478, "AIC-786x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x6578, "AIC-786x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x6678, "AIC-786x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x6778, "AIC-786x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x6915, "ANA620xx/69011A", "Fast Ethernet" } ,
	{ 0x9004, 0x7078, "AIC-7870", "Fast and Wide SCSI Ctrlr" } ,
	{ 0x9004, 0x7178, "AHA-2940/2940W", "Fast/Fast-Wide SCSI Ctrlr" } ,
	{ 0x9004, 0x7278, "AHA-3940/3940W", "Multichannel Fast/Fast-Wide SCSI Ctrlr" } ,
	{ 0x9004, 0x7378, "AHA-3985", "4-chan RAID SCSI Ctrlr" } ,
	{ 0x9004, 0x7478, "AHA-2944", "SCSI Ctrlr" } ,
	{ 0x9004, 0x7578, "AHA-3944/3944W", "Multichannel Fast/Fast-Wide Diff. SCSI Ctrlr" } ,
	{ 0x9004, 0x7678, "AHA-4944W/4944UW", "QuadChannel Fast-Wide/Ultra-Wide Diff. SCSI Ctrlr" } ,
	{ 0x9004, 0x7778, "AIC-787x", "SCSI Ctrlr" } ,
	{ 0x9004, 0x7810, "aic 7810", "Memory control IC" } ,
	{ 0x9004, 0x7815, "AIC-7515", "RAID + Memory Controller IC" } ,
	{ 0x9004, 0x7850, "aic-7850", "Fast/Wide SCSI-2 Controller" } ,
	{ 0x9004, 0x7855, "AHA-2930", "Single channel SCSI Host Adapter" } ,
	{ 0x9004, 0x7860, "AIC-7860", "PCI SCSI Controller" } ,
	{ 0x9004, 0x7870, "AIC-7870", "Fast/Wide SCSI-2 Controller" } ,
	{ 0x9004, 0x7871, "aha 2940", "SCSI" } ,
	{ 0x9004, 0x7872, "aha 3940", "Multiple SCSI channels" } ,
	{ 0x9004, 0x7873, "aha 3980", "Multiple SCSI channels" } ,
	{ 0x9004, 0x7874, "aha 2944", "Differential SCSI" } ,
	{ 0x9004, 0x7880, "aic7880", "Fast 20 SCSI" } ,
	{ 0x9004, 0x7890, "AIC-7890", "SCSI controller" } ,
	{ 0x9004, 0x7891, "AIC-789x", "SCSI controller" } ,
	{ 0x9004, 0x7892, "AIC-789x", "SCSI controller" } ,
	{ 0x9004, 0x7893, "AIC-789x", "SCSI controller" } ,
	{ 0x9004, 0x7894, "AIC-789x", "SCSI controller" } ,
	{ 0x9004, 0x7895, "AIC-7895", "Ultra-Wide SCSI Ctrlr on AHA-2940 AHA-394x" } ,
	{ 0x9004, 0x7896, "AIC-789x", "SCSI controller" } ,
	{ 0x9004, 0x7897, "AIC-789x", "SCSI controller" } ,
	{ 0x9004, 0x8078, "AIC-7880", "Ultra Wide SCSI" } ,
	{ 0x9004, 0x8178, "AHA-2940U/2940UW", "Ultra/Ultra-Wide SCSI Ctrlr" } ,
	{ 0x9004, 0x8278, "AHA-3940Uxx", "AHA-3940U/3940UW/3940UWD SCSI Ctrlr" } ,
	{ 0x9004, 0x8378, "AIC-7883U", "SCSI Controller" } ,
	{ 0x9004, 0x8478, "AHA-2944UW", "Ultra-Wide Diff. SCSI Ctrlr" } ,
	{ 0x9004, 0x8578, "AHA-3944U/3944UWD", "Fast-Wide/Ultra-Wide Diff. SCSI Ctrlr" } ,
	{ 0x9004, 0x8678, "AHA-4944UW", "QuadChannel Ultra-Wide Diff. SCSI Ctrlr" } ,
	{ 0x9004, 0x8778, "AIC-788x", "Ultra-Wide SCSI Ctrlr" } ,
	{ 0x9004, 0x8878, "AIC-7888?", "Ultra Wide SCSI Controller" } ,
	{ 0x9004, 0x8B78, "ABA-1030", "" } ,
	{ 0x9004, 0xEC78, "AHA-4944W/4944UW", "QuadChannel Fast-Wide/Ultra-Wide Diff. SCSI Ctrlr" } ,
	{ 0x9005, 0x0010, "AIC-7890AB", "AHA-2940U2W/U2B,2950U2W Ultra2 SCSI" } ,
	{ 0x9005, 0x0011, "", "AHA-2930U2 Ultra2 SCSI Host Adapter" } ,
	{ 0x9005, 0x0013, "AIC-7890/1", "SCSI Controller" } ,
	{ 0x9005, 0x001F, "AIC-7890/1", "Ultra2-Wide SCSI controller" } ,
	{ 0x9005, 0x0020, "AIC-789x", "SCSI Controller" } ,
	{ 0x9005, 0x002F, "AIC-789x", "SCSI Controller" } ,
	{ 0x9005, 0x0030, "AIC-789x", "SCSI Controller" } ,
	{ 0x9005, 0x003F, "AIC-789x", "SCSI Controller" } ,
	{ 0x9005, 0x0050, "", "AHA-3940U2x/3950U2x Ultra2 SCSI Adapter" } ,
	{ 0x9005, 0x0051, "", "AHA-3950U2x Ultra2 SCSI Adapter" } ,
	{ 0x9005, 0x0053, "AIC-7896", "SCSI Controller" } ,
	{ 0x9005, 0x005F, "AIC-7896/7", "Ultra2 SCSI Controller" } ,
	{ 0x9005, 0x0080, "AIC-7892A", "Ultra160/m PCI SCSI Controller" } ,
	{ 0x9005, 0x0081, "AIC-7892B", "Ultra160 SCSI Controller" } ,
	{ 0x9005, 0x0083, "AIC-7892D", "Ultra160 SCSI Controller" } ,
	{ 0x9005, 0x008F, "AIC-7892", "Ultra160 SCSI Controller" } ,
	{ 0x9005, 0x00C0, "AIC-7899A", "Ultra160 SCSI Controller" } ,
	{ 0x9005, 0x00C1, "AIC-7899B", "Ultra160 SCSI Controller" } ,
	{ 0x9005, 0x00C3, "AIC-7899D", "Ultra160 SCSI Controller" } ,
	{ 0x9005, 0x00C5, "", "RAID Subsystem HBA" } ,
	{ 0x9005, 0x00CF, "AIC-7899", "Ultra160 SCSI Controller" } ,
	{ 0x9005, 0x0258, "AAC-RAID", "RAID Controller" } ,
	{ 0x9005, 0x801F, "AIC-7902", "Ultra320 SCSI Controller" } ,
	{ 0x907F, 0x2015, "IDE-2015PL", "EIDE Ctrlr" } ,
	{ 0x9412, 0x6565, "HT6565", "IDE Controller?" } ,
	{ 0x9710, 0x9705, "Nm9705", "Parallel Port Adapter" } ,
	{ 0x9710, 0x9715, "Nm9715", "PCI Dual 1284 Printer Ports" } ,
	{ 0x9710, 0x9735, "Nm9735", "Dual UART" } ,
	{ 0x9710, 0x9745, "Nm9745", "Dual UART and PCI-ISA Bridge" } ,
	{ 0x9710, 0x9755, "Nm9755", "PCI Bridge with 1284 Parallel Port" } ,
	{ 0x9710, 0x9805, "Nm9805", "PCI + 1284 Printer Port" } ,
	{ 0x9710, 0x9815, "Nm9715CV", "Parallel Port Adapter" } ,
	{ 0x9710, 0x9820, "Nm9820", "Single PCI UART" } ,
	{ 0x9710, 0x9825, "Nm9825", "Single PCI UART" } ,
	{ 0x9710, 0x9835, "Nm9835", "PCI + Dual UART and 1284 Printer Port" } ,
	{ 0x9710, 0x9845, "Nm9845", "PCI Bridge with Dual UART" } ,
	{ 0x9710, 0x9855, "Nm9855", "Parallel/Serial Port Adapter" } ,
	{ 0x9902, 0x0001, "SG2010", "PCI-to-PCI Bridge" } ,
	{ 0x9902, 0x0002, "SG2010", "PCI to high speed serial bridge" } ,
	{ 0x9902, 0x0003, "SG1010", "6 port serial switch /PCI-to-PCI bridge" } ,
	{ 0xAA42, 0x03A3, "9400-0931", "CharKey" } ,
	{ 0xB00C, 0x001C, "IC80+PCI", "POST Diagnostics Card" } ,
	{ 0xb00c, 0x011c, "IC128", "PCI Parallel Port (LPT)" } ,
	{ 0xb00c, 0x021c, "", "Gunboat x4" } ,
	{ 0xb00c, 0x031C, "", "Gunboat x4 Pro" } ,
	{ 0xb00c, 0x041c, "", "Ironclad x8" } ,
	{ 0xb00c, 0x051c, "", "Ironclad x8 Pro" } ,
	{ 0xB00C, 0x061C, "", "IC 138 PCI" } ,
	{ 0xB00C, 0x081C, "", "Dreadnought x16 Pro" } ,
	{ 0xB00C, 0x091C, "", "Dreadnought x16 Lite" } ,
	{ 0xC0DE, 0x5600, "62802", "" } ,
	{ 0xC0DE, 0xC0DE, "62802-5", "QZ0022" } ,
	{ 0xCDDD, 0x0101, "", "DeepSea 1 Board" } ,
	{ 0xCDDD, 0x0200, "DeepSea 1", "High speed stereo correlation chip" } ,
	{ 0xD4D4, 0x0601, "", "PCI Mezzanine Card" } ,
	{ 0xDEAF, 0x9050, "", "PC Weasel PCI VGA Device" } ,
	{ 0xDEAF, 0x9051, "", "PC Weasel PCI Serial Comm. Device" } ,
	{ 0xDEAF, 0x9052, "", "PC Weasel PCI" } ,
	{ 0xE000, 0xE000, "W89C940", "Ethernet Adapter" } ,
	{ 0xE159, 0x0001, "Tiger 300/320", "PCI interface" } ,
	{ 0xE159, 0x0002, "", "Sedlbauer Speed PCI" } ,
	{ 0xE159, 0x0600, "Tiger 600", "PCI-to-PCI Bridge" } ,
	{ 0xEACE, 0x3100, "DAG 3.10", "OC-3/OC-12" } ,
	{ 0xEACE, 0x3200, "DAG 3.2x", "OC-3/OC-12" } ,
	{ 0xEACE, 0x320E, "DAG 3.2E", "Fast Ethernet" } ,
	{ 0xEACE, 0x340E, "DAG 3.4E", "Fast Ethernet" } ,
	{ 0xEACE, 0x341E, "DAG 3.41E", "Fast Ethernet" } ,
	{ 0xEACE, 0x3500, "DAG 3.5", "OC-3/OC-12" } ,
	{ 0xEACE, 0x351C, "DAG 3.5ECM", "Fast Ethernet" } ,
	{ 0xEACE, 0x4100, "DAG 4.10", "OC-48" } ,
	{ 0xEACE, 0x4110, "DAG 4.11", "OC-48" } ,
	{ 0xEACE, 0x4200, "DAG 4.2", "OC-48" } ,
	{ 0xEACE, 0x420E, "DAG 4.2E", "Dual Gigabit Ethernet" } ,
	{ 0xEDD8, 0xA091, "ARK1000PV", "Stingray GUI Accelerator" } ,
	{ 0xEDD8, 0xA099, "ARK2000PV", "Stingray GUI Accelerator" } ,
	{ 0xEDD8, 0xA0A1, "ARK2000MT", "Stingray 64" } ,
	{ 0xEDD8, 0xA0A9, "ARK2000MI", "Quadro645 GUI Accelerator" } ,
	{ 0xEDD8, 0xA0B1, "ARK2000MI+", "GUI Accelerator" } ,
	{ 0xFA57, 0x0001, "PMC", "Pattern Matching Chip" } ,
	{ 0xFEDA, 0xA0FA, "BCM4210", "OEM Chip for 10meg/s over phone line" } ,
	{ 0xFFFE, 0x0710, "", "Virtual SVGA" } ,
	{ 0xFFFF, 0xFFFF, "NO-DEVICE", "No device on PCI bus" }
} ;

// Use this value for loop control during searching:
#define	PCI_DEVTABLE_LEN	(sizeof(PciDevTable)/sizeof(PCI_DEVTABLE))


typedef struct _PCI_CLASSCODETABLE
{
	unsigned char	BaseClass ;
	unsigned char	SubClass ;
	unsigned char	ProgIf ;
	char *		BaseDesc ;
	char *		SubDesc ;
	char *		ProgDesc ;
}  PCI_CLASSCODETABLE, *PPCI_CLASSCODETABLE ;

PCI_CLASSCODETABLE PciClassCodeTable [] =
{
	{ 0x00, 0x00, 0x00, "Pre-2.0 PCI Specification Device", "Non-VGA","" } ,
	{ 0x00, 0x01, 0x00, "Pre-2.0 PCI Specification Device", "VGA Compatible", "" } ,

	{ 0x01, 0x00, 0x00, "Mass Storage Controller", "SCSI", "" } ,
	{ 0x01, 0x01, 0x00, "Mass Storage Controller", "IDE", "" } ,
	{ 0x01, 0x02, 0x00, "Mass Storage Controller", "Floppy", "" } ,
	{ 0x01, 0x03, 0x00, "Mass Storage Controller", "IPI", "" } ,
	{ 0x01, 0x04, 0x00, "Mass Storage Controller", "RAID", "" } ,
	{ 0x01, 0x80, 0x00, "Mass Storage Controller", "Other", "" } ,

	{ 0x02, 0x00, 0x00, "Network Controller", "Ethernet", "" } ,
	{ 0x02, 0x01, 0x00, "Network Controller", "Token Ring", "" } ,
	{ 0x02, 0x02, 0x00, "Network Controller", "FDDI", "" } ,
	{ 0x02, 0x03, 0x00, "Network Controller", "ATM", "" } ,
	{ 0x02, 0x04, 0x00, "Network Controller", "ISDN", "" } ,
	{ 0x02, 0x80, 0x00, "Network Controller", "Other", "" } ,

	{ 0x03, 0x00, 0x00, "Display Controller", "PC Compatible", "VGA" } ,
	{ 0x03, 0x00, 0x01, "Display Controller", "PC Compatible", "8514" } ,
	{ 0x03, 0x01, 0x00, "Display Controller", "XGA", "" } ,
	{ 0x03, 0x02, 0x00, "Display Controller", "3D", "" } ,
	{ 0x03, 0x80, 0x00, "Display Controller", "Other", "" } ,

	{ 0x04, 0x00, 0x00, "Multimedia Device", "Video", "" } ,
	{ 0x04, 0x01, 0x00, "Multimedia Device", "Audio", "" } ,
	{ 0x04, 0x02, 0x00, "Multimedia Device", "Telephony", "" } ,
	{ 0x04, 0x80, 0x00, "Multimedia Device", "Other", "" } ,

	{ 0x05, 0x00, 0x00, "Memory Controller", "RAM", "" } ,
	{ 0x05, 0x01, 0x00, "Memory Controller", "Flash", "" } ,
	{ 0x05, 0x80, 0x00, "Memory Controller", "Other", "" } ,

	{ 0x06, 0x00, 0x00, "Bridge Device", "Host/PCI", "" } ,
	{ 0x06, 0x01, 0x00, "Bridge Device", "PCI/ISA", "" } ,
	{ 0x06, 0x02, 0x00, "Bridge Device", "PCI/EISA", "" } ,
	{ 0x06, 0x03, 0x00, "Bridge Device", "PCI/MCA", "" } ,
	{ 0x06, 0x04, 0x00, "Bridge Device", "PCI/PCI", "" } ,
	{ 0x06, 0x04, 0x01, "Bridge Device", "PCI/PCI", "Subtractive" } ,
	{ 0x06, 0x05, 0x00, "Bridge Device", "PCI/PCMCIA", "" } ,
	{ 0x06, 0x06, 0x00, "Bridge Device", "PCI/NuBus", "" } ,
	{ 0x06, 0x07, 0x00, "Bridge Device", "PCI/CardBus", "" } ,
	{ 0x06, 0x08, 0x00, "Bridge Device", "PCI/RACEway", "Transparent" } ,
	{ 0x06, 0x08, 0x01, "Bridge Device", "PCI/RACEway", "End-Point" } ,
	{ 0x06, 0x80, 0x00, "Bridge Device", "Other", "" } ,

	{ 0x07, 0x00, 0x00, "Simple Communications Controller", "Serial", "Generic XT Compatible" } ,
	{ 0x07, 0x00, 0x01, "Simple Communications Controller", "Serial", "16450 Compatible" } ,
	{ 0x07, 0x00, 0x02, "Simple Communications Controller", "Serial", "16550 Compatible" } ,
	{ 0x07, 0x00, 0x03, "Simple Communications Controller", "Serial", "16650 Compatible" } ,
	{ 0x07, 0x00, 0x04, "Simple Communications Controller", "Serial", "16750 Compatible" } ,
	{ 0x07, 0x00, 0x05, "Simple Communications Controller", "Serial", "16850 Compatible" } ,
	{ 0x07, 0x00, 0x06, "Simple Communications Controller", "Serial", "16950 Compatible" } ,
	{ 0x07, 0x01, 0x00, "Simple Communications Controller", "Parallel", "Standard" } ,
	{ 0x07, 0x01, 0x01, "Simple Communications Controller", "Parallel", "Bidirectional" } ,
	{ 0x07, 0x01, 0x02, "Simple Communications Controller", "Parallel", "ECP 1.X Compliant" } ,
	{ 0x07, 0x01, 0x03, "Simple Communications Controller", "Parallel", "IEEE 1284 controller" } ,
	{ 0x07, 0x01, 0xFE, "Simple Communications Controller", "Parallel", "IEEE 1284 target" } ,
	{ 0x07, 0x02, 0x00, "Simple Communications Controller", "Multiport Serial", "" } ,
	{ 0x07, 0x03, 0x00, "Simple Communications Controller", "Generic Modem", "" } ,
	{ 0x07, 0x03, 0x01, "Simple Communications Controller", "Hayes-Compatible Modem", "16450" } ,
	{ 0x07, 0x03, 0x02, "Simple Communications Controller", "Hayes-Compatible Modem", "16550" } ,
	{ 0x07, 0x03, 0x03, "Simple Communications Controller", "Hayes-Compatible Modem", "16650" } ,
	{ 0x07, 0x03, 0x04, "Simple Communications Controller", "Hayes-Compatible Modem", "16750" } ,
	{ 0x07, 0x80, 0x02, "Simple Communications Controller", "Other", "" } ,

	{ 0x08, 0x00, 0x00, "Base Systems Peripheral", "PIC (Programmable Interrupt Controller)", "Generic 8259" } ,
	{ 0x08, 0x00, 0x01, "Base Systems Peripheral", "PIC (Programmable Interrupt Controller)", "ISA" } ,
	{ 0x08, 0x00, 0x02, "Base Systems Peripheral", "PIC (Programmable Interrupt Controller)", "EISA" } ,
	{ 0x08, 0x00, 0x10, "Base Systems Peripheral", "I/O APIC", "" } ,
	{ 0x08, 0x00, 0x20, "Base Systems Peripheral", "I/O(x) APIC", "" } ,
	{ 0x08, 0x01, 0x00, "Base Systems Peripheral", "DMA (Direct Memory Access)", "Generic 8237" } ,
	{ 0x08, 0x01, 0x01, "Base Systems Peripheral", "DMA (Direct Memory Access)", "ISA" } ,
	{ 0x08, 0x01, 0x02, "Base Systems Peripheral", "DMA (Direct Memory Access)", "EISA" } ,
	{ 0x08, 0x02, 0x00, "Base Systems Peripheral", "System Timer", "Generic 8254" } ,
	{ 0x08, 0x02, 0x01, "Base Systems Peripheral", "System Timer", "ISA" } ,
	{ 0x08, 0x02, 0x02, "Base Systems Peripheral", "System Timer", "EISA" } ,
	{ 0x08, 0x03, 0x00, "Base Systems Peripheral", "RTC (Real Time Clock)", "Generic" } ,
	{ 0x08, 0x03, 0x01, "Base Systems Peripheral", "RTC (Real Time Clock)", "ISA" } ,
	{ 0x08, 0x04, 0x00, "Base Systems Peripheral", "PCI Hot-Plug Controller", "Generic" } ,
	{ 0x08, 0x80, 0x00, "Base Systems Peripheral", "Other", "" } ,

	{ 0x09, 0x00, 0x00, "Input Device", "Keyboard", "" } ,
	{ 0x09, 0x01, 0x00, "Input Device", "Digitizer (Pen)", "" } ,
	{ 0x09, 0x02, 0x00, "Input Device", "Mouse", "" } ,
	{ 0x09, 0x03, 0x00, "Input Device", "Scanner", "" } ,
	{ 0x09, 0x04, 0x00, "Input Device", "Game Port", "Generic" } ,
	{ 0x09, 0x04, 0x10, "Input Device", "Game Port", "Legacy" } ,
	{ 0x09, 0x80, 0x00, "Input Device", "Other", "" } ,

	{ 0x0A, 0x00, 0x00, "Docking Station", "Generic", "" } ,
	{ 0x0A, 0x80, 0x00, "Docking Station", "Other", "" } ,

	{ 0x0B, 0x00, 0x00, "Processor", "i386", "" } ,
	{ 0x0B, 0x01, 0x00, "Processor", "i486", "" } ,
	{ 0x0B, 0x02, 0x00, "Processor", "Pentium", "" } ,
	{ 0x0B, 0x10, 0x00, "Processor", "Alpha", "" } ,
	{ 0x0B, 0x20, 0x00, "Processor", "Power PC", "" } ,
	{ 0x0B, 0x30, 0x00, "Processor", "MIPS", "" } ,
	{ 0x0B, 0x40, 0x00, "Processor", "Co-processor", "" } ,

	{ 0x0C, 0x00, 0x00, "Serial Bus Controller", "IEEE 1394", "Firewire" } ,
	{ 0x0C, 0x00, 0x10, "Serial Bus Controller", "IEEE 1394", "OpenHCI" } ,
	{ 0x0C, 0x01, 0x00, "Serial Bus Controller", "ACCESS.bus", "" } ,
	{ 0x0C, 0x02, 0x00, "Serial Bus Controller", "SSA (Serial Storage Archetecture)", "" } ,
	{ 0x0C, 0x03, 0x00, "Serial Bus Controller", "USB (Universal Serial Bus)", "Universal Host Controller" } ,
	{ 0x0C, 0x03, 0x10, "Serial Bus Controller", "USB (Universal Serial Bus)", "Open Host Controller" } ,
	{ 0x0C, 0x03, 0x80, "Serial Bus Controller", "USB (Universal Serial Bus)", "Non-specific Controller" } ,
	{ 0x0C, 0x03, 0xFE, "Serial Bus Controller", "USB (Universal Serial Bus)", "Device" } ,
	{ 0x0C, 0x04, 0x00, "Serial Bus Controller", "Fibre Channel", "" } ,
	{ 0x0C, 0x05, 0x00, "Serial Bus Controller", "SMBus", "" } ,

	{ 0x0D, 0x00, 0x00, "Wireless", "iRDA-compatible Controller", "" } ,
	{ 0x0D, 0x01, 0x00, "Wireless", "Consumer IR Controller", "" } ,
	{ 0x0D, 0x10, 0x00, "Wireless", "RF Controller", "" } ,
	{ 0x0D, 0x80, 0x00, "Wireless", "Other", "" } ,

	{ 0x0E, 0x00, 0x00, "Intelligent IO", "I2O", "" } ,

	{ 0x0F, 0x01, 0x00, "Satellite", "TV", "" } ,
	{ 0x0F, 0x02, 0x00, "Satellite", "Audio", "" } ,
	{ 0x0F, 0x03, 0x00, "Satellite", "Voice", "" } ,
	{ 0x0F, 0x04, 0x00, "Satellite", "Data", "" } ,

	{ 0x10, 0x00, 0x00, "Encryption", "Network/Computing", "" } ,
	{ 0x10, 0x10, 0x00, "Encryption", "Entertainment", "" } ,
	{ 0x10, 0x80, 0x00, "Encryption", "Other", "" } ,

	{ 0x11, 0x00, 0x00, "Signal Processing", "DPIO", "" } ,
	{ 0x11, 0x80, 0x00, "Signal Processing", "Other", "" } ,

	{ 0xFF, 0x00, 0x00, "Unknown", "Device Does Not Fit in a Defined Class", "UDF" } ,
} ;

// Use this value for loop control during searching:
#define	PCI_CLASSCODETABLE_LEN	(sizeof(PciClassCodeTable)/sizeof(PCI_CLASSCODETABLE))


char *	PciCommandFlags [] =
{
	"I/O Access",
	"Memory Access",
	"Bus Mastering",
	"Special Cycles",
	"Memory Write & Invalidate",
	"Palette Snoop",
	"Parity Errors",
	"Wait Cycles",
	"System Errors",
	"Fast Back-To-Back",
	"Reserved 10",
	"Reserved 11",
	"Reserved 12",
	"Reserved 13",
	"Reserved 14",
	"Reserved 15"
} ;

// Use this value for loop control during searching:
#define	PCI_COMMANDFLAGS_LEN	(sizeof(PciCommandFlags)/sizeof(char *))


char *	PciStatusFlags [] =
{
	"Reserved 0",
	"Reserved 1",
	"Reserved 2",
	"Reserved 3",
	"Reserved 4",
	"66 MHz Capable",
	"User-Defined Features",
	"Fast Back-To-Back",
	"Data Parity Reported",
	"",
	"",
	"Signalled Target Abort",
	"Received Target Abort",
	"Received Master Abort",
	"Signalled System Error",
	"Detected Parity Error"
} ;

// Use this value for loop control during searching:
#define	PCI_STATUSFLAGS_LEN	(sizeof(PciStatusFlags)/sizeof(char *))


char *	PciDevSelFlags [] =
{
	"Fast Devsel Speed",     // TypeC
	"Medium Devsel Speed",   // TypeB
	"Slow Devsel Speed",     // TypeA
	"Reserved 9&10"
} ;

// Use this value for loop control during searching:
#define	PCI_DEVSELFLAGS_LEN	(sizeof(PciDevSelFlags)/sizeof(char *))

