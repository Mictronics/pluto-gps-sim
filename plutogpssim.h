/**
 * This file is part of the pluto-gps-sim project at
 * https://github.com/mictronics/pluto-gps-sim.git
 * 
 * Copyright © 2015-2018 Mictronics
 * Distributed under the MIT License.
 * 
 */
#ifndef GPSSIM_H
#define GPSSIM_H

#define FLOAT_CARR_PHASE // For RKT simulation. Higher computational load, but smoother carrier phase.

/*! \brief Maximum length of a line in a text file (RINEX, motion) */
#define MAX_CHAR (100)

/*! \brief Maximum number of satellites in RINEX file */
#define MAX_SAT (32)

/*! \brief Maximum number of channels we simulate */
#define MAX_CHAN (12)

/*! \brief Number of subframes */
#define N_SBF (5) // 5 subframes per frame

/*! \brief Number of words per subframe */
#define N_DWRD_SBF (10) // 10 word per subframe

/*! \brief Number of words */
#define N_DWRD ((N_SBF+1)*N_DWRD_SBF) // Subframe word buffer size

/*! \brief C/A code sequence length */
#define CA_SEQ_LEN (1023)

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

#define POW2_M5  0.03125
#define POW2_M19 1.907348632812500e-6
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17

#define POW2_M50 8.881784197001252e-016
#define POW2_M30 9.313225746154785e-010
#define POW2_M27 7.450580596923828e-009
#define POW2_M24 5.960464477539063e-008

// Conventional values employed in GPS ephemeris model (ICD-GPS-200)
#define GM_EARTH 3.986005e14
#define OMEGA_EARTH 7.2921151467e-5
#define PI 3.1415926535898

#define WGS84_RADIUS 6378137.0
#define WGS84_ECCENTRICITY 0.0818191908426

#define R2D 57.2957795131

#define SPEED_OF_LIGHT 2.99792458e8
#define LAMBDA_L1 0.190293672798365

/*! \brief GPS L1 Carrier frequency */
#define CARR_FREQ (1575.42e6)
/*! \brief C/A code frequency */
#define CODE_FREQ (1.023e6)
#define CARR_TO_CODE (1.0/1540.0)

#define EPHEM_ARRAY_SIZE (13) // for daily GPS broadcast ephemeris file (brdc)

/*! \brief Structure representing GPS time */
typedef struct {
    int week; /*!< GPS week number (since January 1980) */
    double sec; /*!< second inside the GPS \a week */
} gpstime_t;

/*! \brief Structure representing UTC time */
typedef struct {
    int y; /*!< Calendar year */
    int m; /*!< Calendar month */
    int d; /*!< Calendar day */
    int hh; /*!< Calendar hour */
    int mm; /*!< Calendar minutes */
    double sec; /*!< Calendar seconds */
} datetime_t;

/*! \brief Structure representing ephemeris of a single satellite */
typedef struct {
    bool vflg; /*!< Valid Flag */
    datetime_t t;
    gpstime_t toc; /*!< Time of Clock */
    gpstime_t toe; /*!< Time of Ephemeris */
    int iodc; /*!< Issue of Data, Clock */
    int iode; /*!< Isuse of Data, Ephemeris */
    double deltan; /*!< Delta-N (radians/sec) */
    double cuc; /*!< Cuc (radians) */
    double cus; /*!< Cus (radians) */
    double cic; /*!< Correction to inclination cos (radians) */
    double cis; /*!< Correction to inclination sin (radians) */
    double crc; /*!< Correction to radius cos (meters) */
    double crs; /*!< Correction to radius sin (meters) */
    double ecc; /*!< e Eccentricity */
    double sqrta; /*!< sqrt(A) (sqrt(m)) */
    double m0; /*!< Mean anamoly (radians) */
    double omg0; /*!< Longitude of the ascending node (radians) */
    double inc0; /*!< Inclination (radians) */
    double aop;
    double omgdot; /*!< Omega dot (radians/s) */
    double idot; /*!< IDOT (radians/s) */
    double af0; /*!< Clock offset (seconds) */
    double af1; /*!< rate (sec/sec) */
    double af2; /*!< acceleration (sec/sec^2) */
    double tgd; /*!< Group delay L2 bias */
    int svhlth;
    int codeL2;
    // Working variables follow
    double n; /*!< Mean motion (Average angular velocity) */
    double sq1e2; /*!< sqrt(1-e^2) */
    double A; /*!< Semi-major axis */
    double omgkdot; /*!< OmegaDot-OmegaEdot */
} ephem_t;

typedef struct {
    bool enable;
    bool vflg;
    double alpha0, alpha1, alpha2, alpha3;
    double beta0, beta1, beta2, beta3;
    double A0, A1;
    int dtls, tot, wnt;
    int dtlsf, dn, wnlsf;
} ionoutc_t;

typedef struct {
    gpstime_t g;
    double range; // pseudorange
    double rate;
    double d; // geometric distance
    double azel[2];
    double iono_delay;
} range_t;

/*! \brief Structure representing a Channel */
typedef struct {
    int prn; /*< PRN Number */
    int ca[CA_SEQ_LEN]; /*< C/A Sequence */
    double f_carr; /*< Carrier frequency */
    double f_code; /*< Code frequency */
#ifdef FLOAT_CARR_PHASE
    double carr_phase;
#else
    unsigned int carr_phase; /*< Carrier phase */
    int carr_phasestep; /*< Carrier phasestep */
#endif
    double code_phase; /*< Code phase */
    gpstime_t g0; /*!< GPS time at start */
    unsigned long sbf[5][N_DWRD_SBF]; /*!< current subframe */
    unsigned long dwrd[N_DWRD]; /*!< Data words of sub-frame */
    int iword; /*!< initial word */
    int ibit; /*!< initial bit */
    int icode; /*!< initial code */
    int dataBit; /*!< current data bit */
    int codeCA; /*!< current C/A code */
    double azel[2];
    range_t rho0;
} channel_t;

/* Structure represending a single GPS monitoring station. */
typedef struct {
    const char *id_v2;
    const char *id_v3;
    const char *name;
} stations_t;

/**
 * Stations providing Rinex v3 format.
 * 4-characters station ID
 * 9-characters station ID
 * Station name
 * Including Ionosphere data in Rinex file
 */
const stations_t stations_v3[] = {
    {"func", "FUNC00PRT", "Funchal"},
    {"flrs", "FLRS00PRT", "Santa Cruz das Flore"},
    {"pdel", "PDEL00PRT", "PONTA DELGADA"}
};

/**
 * Stations providing Rinex v2 format.
 * 4-characters station ID
 * 9-characters station ID
 * Station name
 * Including Ionosphere data in Rinex file
 */
const stations_t stations_v2[] = {
    {"abpo", "ABPO00MDG", "Ambohimpanompo, Mada"},
    {"acrg", "ACRG00GHA", "Accra"},
    {"aira", "AIRA00JPN", "AIRA"},
    {"albh", "ALBH00CAN", "Albert Head"},
    {"alic", "ALIC00AUS", "Alice Springs AU012"},
    {"ankr", "ANKR00TUR", "Ankara / Turkey"},
    {"anmg", "ANMG00MYS", "National Space Center"},
    {"areq", "AREQ00PER", "Arequipa Laser Station"},
    {"arht", "ARHT00ATA", "Arrival Heights, McM"},
    {"aruc", "ARUC00ARM", "Aruch-Yerevan"},
    {"baie", "BAIE00CAN", "BAIE RACS-GSD"},
    {"bake", "BAKE00CAN", "BAKE CACS-GSD"},
    {"bele", "BELE00BRA", "BELEM"},
    {"bik0", "BIK000KGZ", "Bishkek"},
    {"boav", "BOAV00BRA", "Boa Vista"},
    {"bogt", "BOGT00COL", "Bogota, Colombia"},
    {"bor1", "BOR100POL", "BOROWIEC"},
    {"braz", "BRAZ00BRA", "Brasilia"},
    {"brew", "BREW00USA", "Brewster VLBA"},
    {"brun", "BRUN00BRN", "Buket Kayu Arang"},
    {"brux", "BRUX00BEL", "Brussels"},
    {"bshm", "BSHM00ISR", "Binyamin Shmuter Mem"},
    {"bucu", "BUCU00ROU", "Bucuresti / Romania"},
    {"bzrg", "BZRG00ITA", "Bolzano"},
    {"cas1", "CAS100ATA", "Casey"},
    {"ccj2", "CCJ200JPN", "CHICHIJIMA-A"},
    {"cebr", "CEBR00ESP", "Cebreros"},
    {"cedu", "CEDU00AUS", "Ceduna"},
    {"chof", "CHOF00JPN", "Chofu, Japan"},
    {"chpi", "CHPI00BRA", "Cachoeira Paulista"},
    {"chur", "CHUR00CAN", "CHUR CACS-GSD"},
    {"ckis", "CKIS00COK", "Cook Islands"},
    {"cmum", "CMUM00THA", "Chiang Mai"},
    {"cord", "CORD00ARG", "Cordoba"},
    {"cpnm", "CPNM00THA", "Chumphon"},
    {"cro1", "CRO100VIR", "St. Croix VLBA"},
    {"cusv", "CUSV00THA", "Chulalongkorn"},
    {"cut0", "CUT000AUS", "Curtin University Be"},
    {"cuut", "CUUT00THA", "Chulalongkorn Univer"},
    {"cztg", "CZTG00ATF", "CROZET ARCHIPELAGO ("},
    {"dae2", "DAE200KOR", "DAEJEON"},
    {"daej", "DAEJ00KOR", "DAEJEON"},
    {"darw", "DARW00AUS", "Darwin AU014"},
    {"dav1", "DAV100ATA", "Davis"},
    {"dgar", "DGAR00GBR", "Diego Garcia Island"},
    {"drao", "DRAO00CAN", "Dominion Radio Astro"},
    {"dubo", "DUBO00CAN", "DUBO CACS-GSD"},
    {"dyng", "DYNG00GRC", "DIONYSOS"},
    {"ebre", "EBRE00ESP", "Ebre"},
    {"enao", "ENAO00PRT", "Eastern North Atlant"},
    {"faa1", "FAA100PYF", "Tahiti-Faaa"},
    {"fair", "FAIR00USA", "Gilmore Creek Observ"},
    {"falk", "FALK00FLK", "Stanley, Falkland Is"},
    {"ffmj", "FFMJ00DEU", "Frankfurt / Main"},
    {"flin", "FLIN00CAN", "FLIN CACS-GSD"},
    {"flrs", "FLRS00PRT", "Santa Cruz das Flore"},
    {"frdn", "FRDN00CAN", "FRDN CACS-GSD"},
    {"func", "FUNC00PRT", "Funchal"},
    {"ganp", "GANP00SVK", "Ganovce"},
    {"gcgo", "GCGO00USA", "Gilmore Creek Geophy"},
    {"geno", "GENO00ITA", "Genova - Istituto Id"},
    {"glps", "GLPS00ECU", "Galapagos Permanent"},
    {"glsv", "GLSV00UKR", "Kiev/Golosiiv"},
    {"gmsd", "GMSD00JPN", "GUTS Masda"},
    {"gode", "GODE00USA", "GGAO (Greenbelt)"},
    {"godn", "GODN00USA", "GGAO NORTH DEEP DRIL"},
    {"gold", "GOLD00USA", "Goldstone Deep Space"},
    {"gop6", "GOP600CZE", "Pecny, Ondrejov / CZ"},
    {"gop7", "GOP700CZE", "Pecny, Ondrejov / CZ"},
    {"gope", "GOPE00CZE", "Pecny, Ondrejov / CZ"},
    {"graz", "GRAZ00AUT", "Graz-Lustbuehel"},
    {"guam", "GUAM00GUM", "USGS Guam Observator"},
    {"hal1", "HAL100USA", "Haleakala Observator"},
    {"harb", "HARB00ZAF", "Hartebeesthoek"},
    {"hers", "HERS00GBR", "Herstmonceux"},
    {"hksl", "HKSL00HKG", "Siu Lang Shui"},
    {"hkws", "HKWS00HKG", "Wong Shek"},
    {"hlfx", "HLFX00CAN", "HLFX CACS-GSD"},
    {"hob2", "HOB200AUS", "Hobart AU016"},
    {"hofn", "HOFN00ISL", "Hoefn / Iceland"},
    {"hrag", "HRAG00ZAF", "Hartebeesthoek RAO"},
    {"hrao", "HRAO00ZAF", "Hartebeesthoek RAO"},
    {"hueg", "HUEG00DEU", "Huegelheim"},
    {"ieng", "IENG00ITA", "Torino"},
    {"iisc", "IISC00IND", "Indian Institute of"},
    {"iqal", "IQAL00CAN", "IQAL CACS-GSD"},
    {"ishi", "ISHI00JPN", "ISHIOKA"},
    {"ista", "ISTA00TUR", "Istanbul / Turkey"},
    {"jctw", "JCTW00ZAF", "Cape Town"},
    {"jog2", "JOG200IDN", "YogYakarta"},
    {"joz2", "JOZ200POL", "JOZEFOSLAW"},
    {"joze", "JOZE00POL", "JOZEFOSLAW"},
    {"jplm", "JPLM00USA", "JPL Mesa"},
    {"jpre", "JPRE00ZAF", "Pretoria"},
    {"karr", "KARR00AUS", "Karratha AU013"},
    {"kat1", "KAT100AUS", "Katherine"},
    {"kir0", "KIR000SWE", "Kiruna"},
    {"kir8", "KIR800SWE", "Kiruna"},
    {"kiri", "KIRI00KIR", "Kiribati"},
    {"kiru", "KIRU00SWE", "Kiruna"},
    {"kit3", "KIT300UZB", "Kitab, Uzbekistan"},
    {"kokb", "KOKB00USA", "Kokee Park Geophysic"},
    {"kokv", "KOKV00USA", "Kokee Park Geophysic"},
    {"kos1", "KOS100NLD", "Kootwijk Observatory"},
    {"kour", "KOUR00GUF", "Kourou"},
    {"kuj2", "KUJ200CAN", "KUJ2 CACS-CGS"},
    {"kzn2", "KZN200RUS", "KAZAN"},
    {"laut", "LAUT00FJI", "Fiji CGPS"},
    {"leij", "LEIJ00DEU", "Leipzig"},
    {"lhaz", "LHAZ00CHN", "Lhasa / Tibet / China"},
    {"llag", "LLAG00ESP", "La Laguna, Tenerife"},
    {"lpal", "LPAL00ESP", "La Palma"},
    {"lpgs", "LPGS00ARG", "La Plata"},
    {"m0se", "M0SE00ITA", "Rome"},
    {"maju", "MAJU00MHL", "Majuro"},
    {"mal2", "MAL200KEN", "Malindi"},
    {"mar6", "MAR600SWE", "Maartsbo"},
    {"mar7", "MAR700SWE", "Maartsbo"},
    {"mas1", "MAS100ESP", "Maspalomas"},
    {"mat1", "MAT100ITA", "Matera"},
    {"mate", "MATE00ITA", "Matera"},
    {"matg", "MATG00ITA", "MateraG"},
    {"maw1", "MAW100ATA", "Mawson"},
    {"mbar", "MBAR00UGA", "Mbarara"},
    {"mchl", "MCHL00AUS", "Mitchell"},
    {"mcm4", "MCM400ATA", "McMurdo GPS Station"},
    {"mdo1", "MDO100USA", "McDonald Observatory"},
    {"medi", "MEDI00ITA", "Medicina(BO)"},
    {"meli", "MELI00ESP", "Melilla"},
    {"mers", "MERS00TUR", "Mersin / Turkey"},
    {"met3", "MET300FIN", "Metsahovi"},
    {"metg", "METG00FIN", "METSAHOVI"},
    {"mets", "METS00FIN", "METSAHOVI"},
    {"mgue", "MGUE00ARG", "Malargue"},
    {"mikl", "MIKL00UKR", "Mykolaiv"},
    {"mizu", "MIZU00JPN", "Mizusawa"},
    {"mkea", "MKEA00USA", "Mauna Kea"},
    {"mobs", "MOBS00AUS", "Melbourne Observator"},
    {"mrc1", "MRC100USA", "NRL Midway Research"},
    {"mro1", "MRO100AUS", "Murchison Radio Obse"},
    {"ncku", "NCKU00TWN", "MECLAB"},
    {"nico", "NICO00CYP", "Nicosia-Athalassa"},
    {"nium", "NIUM00NIU", "Niue Meteorological"},
    {"nlib", "NLIB00USA", "North Liberty VLBA s"},
    {"nnor", "NNOR00AUS", "New Norcia"},
    {"not1", "NOT100ITA", "Noto-Radioastronomy"},
    {"nrc1", "NRC100CAN", "NRC1 CACS-GSD"},
    {"nrmd", "NRMD00NCL", "NORMANDIE"},
    {"ntus", "NTUS00SGP", "Nanyang Technologica"},
    {"nvsk", "NVSK00RUS", "Novosibirsk"},
    {"nya1", "NYA100NOR", "Ny-Alesund"},
    {"nya2", "NYA200NOR", "Ny Alesund"},
    {"nyal", "NYAL00NOR", "Ny-Alesund"},
    {"obe4", "OBE400DEU", "Oberpfaffenhofen"},
    {"ohi3", "OHI300ATA", "O'Higgins / Antarcti"},
    {"ons1", "ONS100SWE", "Onsala"},
    {"onsa", "ONSA00SWE", "ONSALA"},
    {"op71", "OP7100FRA", "Observatoire de Pari"},
    {"orid", "ORID00MKD", "Ohrid / Macedonia"},
    {"ous2", "OUS200NZL", "Dunedin"},
    {"pado", "PADO00ITA", "UNIVERSITY OF PADOVA"},
    {"palm", "PALM00ATA", "Palmer Station, Anta"},
    {"park", "PARK00AUS", "Australian Telescope"},
    {"pdel", "PDEL00PRT", "PONTA DELGADA"},
    {"pert", "PERT00AUS", "Perth"},
    {"pfrr", "PFRR00USA", "Poker Flat Research"},
    {"picl", "PICL00CAN", "PICL CACS-ACP"},
    {"pie1", "PIE100USA", "Pietown VLBA Site"},
    {"pimo", "PIMO00PHL", "Manila Observatory"},
    {"pngm", "PNGM00PNG", "Manus Island CGPS"},
    {"poal", "POAL00BRA", "PORTO ALEGRE"},
    {"pohn", "POHN00FSM", "Pohnpei"},
    {"pol2", "POL200KGZ", "Poligan IVTAN 2"},
    {"polv", "POLV00UKR", "Poltava"},
    {"pots", "POTS00DEU", "Potsdam, GeoForschun"},
    {"pove", "POVE00BRA", "PORTO VELHO"},
    {"prds", "PRDS00CAN", "PRDS CACS-GSD"},
    {"ptvl", "PTVL00VUT", "Port Vila GNSS"},
    {"qaq1", "QAQ100GRL", "Qaqortoq"},
    {"quin", "QUIN00USA", "Quincy"},
    {"raeg", "RAEG00PRT", "RAEGE Santa Maria"},
    {"redu", "REDU00BEL", "Redu"},
    {"reyk", "REYK00ISL", "Reykjavik / Iceland"},
    {"riga", "RIGA00LVA", "RIGA permanent GPS"},
    {"rio2", "RIO200ARG", "Rio Grande"},
    {"roag", "ROAG00ESP", "San Fernando / Spain"},
    {"salu", "SALU00BRA", "São Luis"},
    {"samo", "SAMO00WSM", "Samoa"},
    {"sant", "SANT00CHL", "Santiago Tracking St"},
    {"sask", "SASK00CAN", "SASK CACS-GSD"},
    {"savo", "SAVO00BRA", "Salvador INCRA"},
    {"sch2", "SCH200CAN", "SCH2 CACS-GSD"},
    {"scor", "SCOR00GRL", "Scoresbysund"},
    {"scrz", "SCRZ00BOL", "SANTA CRUZ DE LA SIE"},
    {"scub", "SCUB00CUB", "CENAIS, Santiago de"},
    {"seme", "SEME00KAZ", "Semey"},
    {"sey2", "SEY200SYC", "Republic of the Seyc"},
    {"sfer", "SFER00ESP", "San Fernando"},
    {"sgoc", "SGOC00LKA", "Surveyor General's O"},
    {"sgpo", "SGPO00USA", "Southern Great Plain"},
    {"sin1", "SIN100SGP", "Nanyang Technologica"},
    {"sofi", "SOFI00BGR", "Sofia / Bulgaria"},
    {"solo", "SOLO00SLB", "Solomon Islands CGPS"},
    {"spt0", "SPT000SWE", "SP Boras"},
    {"sptu", "SPTU00BRA", "Tupa"},
    {"stfu", "STFU00USA", "Stanford University"},
    {"sthl", "STHL00GBR", "Saint Helena"},
    {"stjo", "STJO00CAN", "STJO CACS-GSD"},
    {"stk2", "STK200JPN", "SHINTOTSUKAWA-A"},
    {"stpm", "STPM00SPM", "Saint Pierre"},
    {"str1", "STR100AUS", "Stromlo"},
    {"str2", "STR200AUS", "Stromlo"},
    {"sulp", "SULP00UKR", "Lviv Polytechnic"},
    {"suth", "SUTH00ZAF", "Sutherland"},
    {"sutm", "SUTM00ZAF", "Sutherland"},
    {"svtl", "SVTL00RUS", "Svetloe"},
    {"sydn", "SYDN00AUS", "National Measurement"},
    {"syog", "SYOG00ATA", "Syowa"},
    {"tash", "TASH00UZB", "Tashkent"},
    {"thu2", "THU200GRL", "Thule"},
    {"tid1", "TID100AUS", "Tidbinbilla (Canberr"},
    {"tit2", "TIT200DEU", "Titz / Jackerath"},
    {"tlsg", "TLSG00FRA", "Toulouse"},
    {"tong", "TONG00TON", "Tonga"},
    {"topl", "TOPL00BRA", "Palmas"},
    {"tow2", "TOW200AUS", "Townsville AU028"},
    {"tro1", "TRO100NOR", "Tromsoe"},
    {"tsk2", "TSK200JPN", "TSUKUBA2-A"},
    {"tskb", "TSKB00JPN", "Tsukuba ROGUE site"},
    {"twtf", "TWTF00TWN", "Taoyuan"},
    {"ucal", "UCAL00CAN", "University of Calgar"},
    {"ufpr", "UFPR00BRA", "CURITIBA UFPR"},
    {"ulab", "ULAB00MNG", "Ulaanbataar"},
    {"unb3", "UNB300CAN", "University of New Br"},
    {"unbd", "UNBD00CAN", "University of New Br"},
    {"unsa", "UNSA00ARG", "UNSA Salta"},
    {"urum", "URUM00CHN", "Urumqi, People Repub"},
    {"usn7", "USN700USA", "U.S. Naval Observato"},
    {"usn8", "USN800USA", "U.S. Naval Observato"},
    {"usn9", "USN900USA", "U.S. Naval Observato"},
    {"usud", "USUD00JPN", "Usuda Deep Space Tra"},
    {"utqi", "UTQI00USA", "Utqiagvik"},
    {"vacs", "VACS00MUS", "Vacoas Meteo"},
    {"vald", "VALD00CAN", "VALD CACS-GSD"},
    {"vill", "VILL00ESP", "Villafranca"},
    {"vis0", "VIS000SWE", "Visby"},
    {"voim", "VOIM00MDG", "Ambalavao"},
    {"wab2", "WAB200CHE", "WAB2"},
    {"warn", "WARN00DEU", "Warnemuende"},
    {"wind", "WIND00NAM", "Windhoek"},
    {"wroc", "WROC00POL", "WROCLAW"},
    {"wsrt", "WSRT00NLD", "Westerbork Synthesis"},
    {"wtz3", "WTZ300DEU", "Wettzell / Germany"},
    {"wtzr", "WTZR00DEU", "Wettzell / Germany"},
    {"wtzs", "WTZS00DEU", "Wettzell / Germany"},
    {"wtzz", "WTZZ00DEU", "Wettzell / Germany"},
    {"wuh2", "WUH200CHN", "WUHAN"},
    {"wuhn", "WUHN00CHN", "WUHAN"},
    {"xmis", "XMIS00AUS", "Christmas Island"},
    {"yar2", "YAR200AUS", "Yarragadee"},
    {"yar3", "YAR300AUS", "Yarragadee"},
    {"yebe", "YEBE00ESP", "Yebes"},
    {"yell", "YELL00CAN", "YELL CACS-GSD"},
    {"ykro", "YKRO00CIV", "Yamoussoukro Trackin"},
    {"zim2", "ZIM200CHE", "Zimmerwald"},
    {"zim3", "ZIM300CHE", "Zimmerwald"},
    {"zimm", "ZIMM00CHE", "Zimmerwald L+T 88"}
};

#endif
