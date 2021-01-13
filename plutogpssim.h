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
    const bool has_ionodata;
} stations_t;

/**
 * Stations providing Rinex v3 format.
 * 4-characters station ID
 * 9-characters station ID
 * Station name
 * Including Ionosphere data in Rinex file
 */
const stations_t stations_v3[] = {
    {"func", "FUNC00PRT", "Funchal", true},
    {"flrs", "FLRS00PRT", "Santa Cruz das Flore", true},
    {"pdel", "PDEL00PRT", "PONTA DELGADA", true}
};

/**
 * Stations providing Rinex v2 format.
 * 4-characters station ID
 * 9-characters station ID
 * Station name
 * Including Ionosphere data in Rinex file
 */
const stations_t stations_v2[] = {
    {"abmf", "ABMF00GLP", "Aeroport du Raizet", false},
    {"abpo", "ABPO00MDG", "Ambohimpanompo, Mada", true},
    {"acrg", "ACRG00GHA", "Accra", true},
    {"aggo", "AGGO00ARG", "AGGO / Argentina", false},
    {"aira", "AIRA00JPN", "AIRA", true},
    {"ajac", "AJAC00FRA", "Ajaccio", false},
    {"albh", "ALBH00CAN", "Albert Head", true},
    {"alic", "ALIC00AUS", "Alice Springs AU012", true},
    {"ankr", "ANKR00TUR", "Ankara / Turkey", true},
    {"anmg", "ANMG00MYS", "National Space Center", true},
    {"areg", "AREG00PER", "Arequipa", false},
    {"areq", "AREQ00PER", "Arequipa Laser Station", true},
    {"arht", "ARHT00ATA", "Arrival Heights, McM", true},
    {"aruc", "ARUC00ARM", "Aruch-Yerevan", true},
    {"ascg", "ASCG00SHN", "Ascension", false},
    {"baie", "BAIE00CAN", "BAIE RACS-GSD", true},
    {"bake", "BAKE00CAN", "BAKE CACS-GSD", true},
    {"bele", "BELE00BRA", "BELEM", true},
    {"bik0", "BIK000KGZ", "Bishkek", true},
    {"boav", "BOAV00BRA", "Boa Vista", true},
    {"bogi", "BOGI00POL", "Borowa Gora", false},
    {"bogt", "BOGT00COL", "Bogota, Colombia", true},
    {"bor1", "BOR100POL", "BOROWIEC", true},
    {"braz", "BRAZ00BRA", "Brasilia", true},
    {"brew", "BREW00USA", "Brewster VLBA", true},
    {"brst", "BRST00FRA", "Brest", false},
    {"brun", "BRUN00BRN", "Buket Kayu Arang", true},
    {"brux", "BRUX00BEL", "Brussels", true},
    {"bshm", "BSHM00ISR", "Binyamin Shmuter Mem", true},
    {"bucu", "BUCU00ROU", "Bucuresti / Romania", true},
    {"bzrg", "BZRG00ITA", "Bolzano", true},
    {"cas1", "CAS100ATA", "Casey", true},
    {"ccj2", "CCJ200JPN", "CHICHIJIMA-A", true},
    {"cebr", "CEBR00ESP", "Cebreros", true},
    {"cedu", "CEDU00AUS", "Ceduna", true},
    {"chof", "CHOF00JPN", "Chofu, Japan", true},
    {"chpg", "CHPG00BRA", "Cachoeira Paulista", false},
    {"chpi", "CHPI00BRA", "Cachoeira Paulista", true},
    {"chur", "CHUR00CAN", "CHUR CACS-GSD", true},
    {"cibg", "CIBG00IDN", "Cibinong", false},
    {"ckis", "CKIS00COK", "Cook Islands", true},
    {"cmum", "CMUM00THA", "Chiang Mai", true},
    {"cord", "CORD00ARG", "Cordoba", true},
    {"cpnm", "CPNM00THA", "Chumphon", true},
    {"cpvg", "CPVG00CPV", "CAP-VERT", false},
    {"cro1", "CRO100VIR", "St. Croix VLBA", true},
    {"cusv", "CUSV00THA", "Chulalongkorn", true},
    {"cut0", "CUT000AUS", "Curtin University Be", true},
    {"cuut", "CUUT00THA", "Chulalongkorn Univer", true},
    {"cztg", "CZTG00ATF", "CROZET ARCHIPELAGO (", true},
    {"dae2", "DAE200KOR", "DAEJEON", true},
    {"daej", "DAEJ00KOR", "DAEJEON", true},
    {"darw", "DARW00AUS", "Darwin AU014", true},
    {"dav1", "DAV100ATA", "Davis", true},
    {"dgar", "DGAR00GBR", "Diego Garcia Island", true},
    {"djig", "DJIG00DJI", "Djibouti", false},
    {"dlf1", "DLF100NLD", "Delft", false},
    {"drao", "DRAO00CAN", "Dominion Radio Astro", true},
    {"dubo", "DUBO00CAN", "DUBO CACS-GSD", true},
    {"dyng", "DYNG00GRC", "DIONYSOS", true},
    {"ebre", "EBRE00ESP", "Ebre", true},
    {"enao", "ENAO00PRT", "Eastern North Atlant", true},
    {"faa1", "FAA100PYF", "Tahiti-Faaa", true},
    {"fair", "FAIR00USA", "Gilmore Creek Observ", true},
    {"falk", "FALK00FLK", "Stanley, Falkland Is", true},
    {"ffmj", "FFMJ00DEU", "Frankfurt / Main", true},
    {"flin", "FLIN00CAN", "FLIN CACS-GSD", true},
    {"flrs", "FLRS00PRT", "Santa Cruz das Flore", true},
    {"frdn", "FRDN00CAN", "FRDN CACS-GSD", true},
    {"ftna", "FTNA00WLF", "Futuna", false},
    {"func", "FUNC00PRT", "Funchal", true},
    {"gamb", "GAMB00PYF", "Rikitea", false},
    {"gamg", "GAMG00KOR", "Geochang", false},
    {"ganp", "GANP00SVK", "Ganovce", true},
    {"gcgo", "GCGO00USA", "Gilmore Creek Geophy", true},
    {"geno", "GENO00ITA", "Genova - Istituto Id", true},
    {"glps", "GLPS00ECU", "Galapagos Permanent", true},
    {"glsv", "GLSV00UKR", "Kiev/Golosiiv", true},
    {"gmsd", "GMSD00JPN", "GUTS Masda", true},
    {"gode", "GODE00USA", "GGAO (Greenbelt)", true},
    {"godn", "GODN00USA", "GGAO NORTH DEEP DRIL", true},
    {"gold", "GOLD00USA", "Goldstone Deep Space", true},
    {"gop6", "GOP600CZE", "Pecny, Ondrejov / CZ", true},
    {"gop7", "GOP700CZE", "Pecny, Ondrejov / CZ", true},
    {"gope", "GOPE00CZE", "Pecny, Ondrejov / CZ", true},
    {"grac", "GRAC00FRA", "GRASSE", false},
    {"gras", "GRAS00FRA", "Observatoire de Cale", false},
    {"graz", "GRAZ00AUT", "Graz-Lustbuehel", true},
    {"guam", "GUAM00GUM", "USGS Guam Observator", true},
    {"hal1", "HAL100USA", "Haleakala Observator", true},
    {"harb", "HARB00ZAF", "Hartebeesthoek", true},
    {"hers", "HERS00GBR", "Herstmonceux", true},
    {"hksl", "HKSL00HKG", "Siu Lang Shui", true},
    {"hkws", "HKWS00HKG", "Wong Shek", true},
    {"hlfx", "HLFX00CAN", "HLFX CACS-GSD", true},
    {"hob2", "HOB200AUS", "Hobart AU016", true},
    {"hofn", "HOFN00ISL", "Hoefn / Iceland", true},
    {"hrag", "HRAG00ZAF", "Hartebeesthoek RAO", true},
    {"hrao", "HRAO00ZAF", "Hartebeesthoek RAO", true},
    {"hueg", "HUEG00DEU", "Huegelheim", true},
    {"ieng", "IENG00ITA", "Torino", true},
    {"iisc", "IISC00IND", "Indian Institute of", true},
    {"iqal", "IQAL00CAN", "IQAL CACS-GSD", true},
    {"ishi", "ISHI00JPN", "ISHIOKA", true},
    {"ista", "ISTA00TUR", "Istanbul / Turkey", true},
    {"jctw", "JCTW00ZAF", "Cape Town", true},
    {"jfng", "JFNG00CHN", "JIUFENG", false},
    {"jog2", "JOG200IDN", "YogYakarta", true},
    {"joz2", "JOZ200POL", "JOZEFOSLAW", true},
    {"joze", "JOZE00POL", "JOZEFOSLAW", true},
    {"jplm", "JPLM00USA", "JPL Mesa", true},
    {"jpre", "JPRE00ZAF", "Pretoria", true},
    {"karr", "KARR00AUS", "Karratha AU013", true},
    {"kat1", "KAT100AUS", "Katherine", true},
    {"kerg", "KERG00ATF", "Kerguelen Islands", false},
    {"kir0", "KIR000SWE", "Kiruna", true},
    {"kir8", "KIR800SWE", "Kiruna", true},
    {"kiri", "KIRI00KIR", "Kiribati", true},
    {"kiru", "KIRU00SWE", "Kiruna", true},
    {"kit3", "KIT300UZB", "Kitab, Uzbekistan", true},
    {"kitg", "KITG00UZB", "Kitab", false},
    {"kokb", "KOKB00USA", "Kokee Park Geophysic", true},
    {"kokv", "KOKV00USA", "Kokee Park Geophysic", true},
    {"kos1", "KOS100NLD", "Kootwijk Observatory", true},
    {"koug", "KOUG00GUF", "KOUROU", false},
    {"kour", "KOUR00GUF", "Kourou", true},
    {"krgg", "KRGG00ATF", "Kerguelen Islands", false},
    {"kuj2", "KUJ200CAN", "KUJ2 CACS-CGS", true},
    {"kzn2", "KZN200RUS", "KAZAN", true},
    {"laut", "LAUT00FJI", "Fiji CGPS", true},
    {"leij", "LEIJ00DEU", "Leipzig", true},
    {"lhaz", "LHAZ00CHN", "Lhasa / Tibet / China", true},
    {"llag", "LLAG00ESP", "La Laguna, Tenerife", true},
    {"lmmf", "LMMF00MTQ", "Aeroport Aime CESAIR", false},
    {"lpal", "LPAL00ESP", "La Palma", true},
    {"lpgs", "LPGS00ARG", "La Plata", true},
    {"lroc", "LROC00FRA", "LA ROCHELLE", false},
    {"m0se", "M0SE00ITA", "Rome", true},
    {"madr", "MADR00ESP", "Madrid Deep Space Tr", false},
    {"maju", "MAJU00MHL", "Majuro", true},
    {"mal2", "MAL200KEN", "Malindi", true},
    {"mar6", "MAR600SWE", "Maartsbo", true},
    {"mar7", "MAR700SWE", "Maartsbo", true},
    {"mars", "MARS00FRA", "Marseille", false},
    {"mas1", "MAS100ESP", "Maspalomas", true},
    {"mat1", "MAT100ITA", "Matera", true},
    {"mate", "MATE00ITA", "Matera", true},
    {"matg", "MATG00ITA", "MateraG", true},
    {"maw1", "MAW100ATA", "Mawson", true},
    {"mayg", "MAYG00MYT", "Dzaoudzi", false},
    {"mbar", "MBAR00UGA", "Mbarara", true},
    {"mchl", "MCHL00AUS", "Mitchell", true},
    {"mcm4", "MCM400ATA", "McMurdo GPS Station", true},
    {"mdo1", "MDO100USA", "McDonald Observatory", true},
    {"medi", "MEDI00ITA", "Medicina(BO)", true},
    {"meli", "MELI00ESP", "Melilla", true},
    {"mers", "MERS00TUR", "Mersin / Turkey", true},
    {"met3", "MET300FIN", "Metsahovi", true},
    {"metg", "METG00FIN", "METSAHOVI", true},
    {"mets", "METS00FIN", "METSAHOVI", true},
    {"mgue", "MGUE00ARG", "Malargue", true},
    {"mikl", "MIKL00UKR", "Mykolaiv", true},
    {"mizu", "MIZU00JPN", "Mizusawa", true},
    {"mkea", "MKEA00USA", "Mauna Kea", true},
    {"mobs", "MOBS00AUS", "Melbourne Observator", true},
    {"mrc1", "MRC100USA", "NRL Midway Research", true},
    {"mro1", "MRO100AUS", "Murchison Radio Obse", true},
    {"ncku", "NCKU00TWN", "MECLAB", true},
    {"nico", "NICO00CYP", "Nicosia-Athalassa", true},
    {"nium", "NIUM00NIU", "Niue Meteorological", true},
    {"nklg", "NKLG00GAB", "N'KOLTANG (GABON)", false},
    {"nlib", "NLIB00USA", "North Liberty VLBA s", true},
    {"nnor", "NNOR00AUS", "New Norcia", true},
    {"not1", "NOT100ITA", "Noto-Radioastronomy", true},
    {"nrc1", "NRC100CAN", "NRC1 CACS-GSD", true},
    {"nrmd", "NRMD00NCL", "NORMANDIE", true},
    {"ntus", "NTUS00SGP", "Nanyang Technologica", true},
    {"nvsk", "NVSK00RUS", "Novosibirsk", true},
    {"nya1", "NYA100NOR", "Ny-Alesund", true},
    {"nya2", "NYA200NOR", "Ny Alesund", true},
    {"nyal", "NYAL00NOR", "Ny-Alesund", true},
    {"obe4", "OBE400DEU", "Oberpfaffenhofen", true},
    {"ohi2", "OHI200ATA", "O'Higgins / Antarcti", false},
    {"ohi3", "OHI300ATA", "O'Higgins / Antarcti", true},
    {"ons1", "ONS100SWE", "Onsala", true},
    {"onsa", "ONSA00SWE", "ONSALA", true},
    {"op71", "OP7100FRA", "Observatoire de Pari", true},
    {"orid", "ORID00MKD", "Ohrid / Macedonia", true},
    {"ous2", "OUS200NZL", "Dunedin", true},
    {"owmg", "OWMG00NZL", "Chatham Island", false},
    {"pado", "PADO00ITA", "UNIVERSITY OF PADOVA", true},
    {"palm", "PALM00ATA", "Palmer Station, Anta", true},
    {"park", "PARK00AUS", "Australian Telescope", true},
    {"pdel", "PDEL00PRT", "PONTA DELGADA", true},
    {"pert", "PERT00AUS", "Perth", true},
    {"pfrr", "PFRR00USA", "Poker Flat Research", true},
    {"picl", "PICL00CAN", "PICL CACS-ACP", true},
    {"pie1", "PIE100USA", "Pietown VLBA Site", true},
    {"pimo", "PIMO00PHL", "Manila Observatory", true},
    {"pngm", "PNGM00PNG", "Manus Island CGPS", true},
    {"poal", "POAL00BRA", "PORTO ALEGRE", true},
    {"pohn", "POHN00FSM", "Pohnpei", true},
    {"pol2", "POL200KGZ", "Poligan IVTAN 2", true},
    {"polv", "POLV00UKR", "Poltava", true},
    {"pots", "POTS00DEU", "Potsdam, GeoForschun", true},
    {"pove", "POVE00BRA", "PORTO VELHO", true},
    {"prds", "PRDS00CAN", "PRDS CACS-GSD", true},
    {"ptbb", "PTBB00DEU", "Braunschweig", false},
    {"ptgg", "PTGG00PHL", "Manilla", false},
    {"ptvl", "PTVL00VUT", "Port Vila GNSS", true},
    {"qaq1", "QAQ100GRL", "Qaqortoq", true},
    {"quin", "QUIN00USA", "Quincy", true},
    {"rabt", "RABT00MAR", "Rabat, EMI", false},
    {"raeg", "RAEG00PRT", "RAEGE Santa Maria", true},
    {"redu", "REDU00BEL", "Redu", true},
    {"reun", "REUN00REU", "La Reunion - Observa", false},
    {"reyk", "REYK00ISL", "Reykjavik / Iceland", true},
    {"rgdg", "RGDG00ARG", "Rio Grande", false},
    {"riga", "RIGA00LVA", "RIGA permanent GPS", true},
    {"rio2", "RIO200ARG", "Rio Grande", true},
    {"roag", "ROAG00ESP", "San Fernando / Spain", true},
    {"salu", "SALU00BRA", "São Luis", true},
    {"samo", "SAMO00WSM", "Samoa", true},
    {"sant", "SANT00CHL", "Santiago Tracking St", true},
    {"sask", "SASK00CAN", "SASK CACS-GSD", true},
    {"savo", "SAVO00BRA", "Salvador INCRA", true},
    {"sch2", "SCH200CAN", "SCH2 CACS-GSD", true},
    {"scor", "SCOR00GRL", "Scoresbysund", true},
    {"scrz", "SCRZ00BOL", "SANTA CRUZ DE LA SIE", true},
    {"scub", "SCUB00CUB", "CENAIS, Santiago de", true},
    {"seme", "SEME00KAZ", "Semey", true},
    {"sey2", "SEY200SYC", "Republic of the Seyc", true},
    {"seyg", "SEYG00SYC", "MAHE", false},
    {"sfer", "SFER00ESP", "San Fernando", true},
    {"sgoc", "SGOC00LKA", "Surveyor General's O", true},
    {"sgpo", "SGPO00USA", "Southern Great Plain", true},
    {"sin1", "SIN100SGP", "Nanyang Technologica", true},
    {"sod3", "SOD300FIN", "Sodankyla", false},
    {"sofi", "SOFI00BGR", "Sofia / Bulgaria", true},
    {"solo", "SOLO00SLB", "Solomon Islands CGPS", true},
    {"spt0", "SPT000SWE", "SP Boras", true},
    {"sptu", "SPTU00BRA", "Tupa", true},
    {"stfu", "STFU00USA", "Stanford University", true},
    {"sthl", "STHL00GBR", "Saint Helena", true},
    {"stj3", "STJ300CAN", "STJ3 CACS-GSD", false},
    {"stjo", "STJO00CAN", "STJO CACS-GSD", true},
    {"stk2", "STK200JPN", "SHINTOTSUKAWA-A", true},
    {"stpm", "STPM00SPM", "Saint Pierre", true},
    {"str1", "STR100AUS", "Stromlo", true},
    {"str2", "STR200AUS", "Stromlo", true},
    {"sulp", "SULP00UKR", "Lviv Polytechnic", true},
    {"suth", "SUTH00ZAF", "Sutherland", true},
    {"sutm", "SUTM00ZAF", "Sutherland", true},
    {"svtl", "SVTL00RUS", "Svetloe", true},
    {"sydn", "SYDN00AUS", "National Measurement", true},
    {"syog", "SYOG00ATA", "Syowa", true},
    {"tash", "TASH00UZB", "Tashkent", true},
    {"thtg", "THTG00PYF", "Papeete Tahiti", false},
    {"thu2", "THU200GRL", "Thule", true},
    {"tid1", "TID100AUS", "Tidbinbilla (Canberr", true},
    {"tit2", "TIT200DEU", "Titz / Jackerath", true},
    {"tlse", "TLSE00FRA", "Toulouse", false},
    {"tlsg", "TLSG00FRA", "Toulouse", true},
    {"tong", "TONG00TON", "Tonga", true},
    {"topl", "TOPL00BRA", "Palmas", true},
    {"tow2", "TOW200AUS", "Townsville AU028", true},
    {"tro1", "TRO100NOR", "Tromsoe", true},
    {"tsk2", "TSK200JPN", "TSUKUBA2-A", true},
    {"tskb", "TSKB00JPN", "Tsukuba ROGUE site", true},
    {"twtf", "TWTF00TWN", "Taoyuan", true},
    {"ucal", "UCAL00CAN", "University of Calgar", true},
    {"ufpr", "UFPR00BRA", "CURITIBA UFPR", true},
    {"ulab", "ULAB00MNG", "Ulaanbataar", true},
    {"unb3", "UNB300CAN", "University of New Br", true},
    {"unbd", "UNBD00CAN", "University of New Br", true},
    {"unsa", "UNSA00ARG", "UNSA Salta", true},
    {"urum", "URUM00CHN", "Urumqi, People Repub", true},
    {"usn7", "USN700USA", "U.S. Naval Observato", true},
    {"usn8", "USN800USA", "U.S. Naval Observato", true},
    {"usn9", "USN900USA", "U.S. Naval Observato", true},
    {"usud", "USUD00JPN", "Usuda Deep Space Tra", true},
    {"utqi", "UTQI00USA", "Utqiagvik", true},
    {"vacs", "VACS00MUS", "Vacoas Meteo", true},
    {"vald", "VALD00CAN", "VALD CACS-GSD", true},
    {"vill", "VILL00ESP", "Villafranca", true},
    {"vis0", "VIS000SWE", "Visby", true},
    {"voim", "VOIM00MDG", "Ambalavao", true},
    {"wab2", "WAB200CHE", "WAB2", true},
    {"warn", "WARN00DEU", "Warnemuende", true},
    {"whit", "WHIT00CAN", "WHIT CACS-GSD", false},
    {"wind", "WIND00NAM", "Windhoek", true},
    {"wroc", "WROC00POL", "WROCLAW", true},
    {"wsrt", "WSRT00NLD", "Westerbork Synthesis", true},
    {"wtz3", "WTZ300DEU", "Wettzell / Germany", true},
    {"wtzr", "WTZR00DEU", "Wettzell / Germany", true},
    {"wtzs", "WTZS00DEU", "Wettzell / Germany", true},
    {"wtzz", "WTZZ00DEU", "Wettzell / Germany", true},
    {"wuh2", "WUH200CHN", "WUHAN", true},
    {"wuhn", "WUHN00CHN", "WUHAN", true},
    {"xmis", "XMIS00AUS", "Christmas Island", true},
    {"yar2", "YAR200AUS", "Yarragadee", true},
    {"yar3", "YAR300AUS", "Yarragadee", true},
    {"yebe", "YEBE00ESP", "Yebes", true},
    {"yel2", "YEL200CAN", "YELLOWKNIFE", false},
    {"yell", "YELL00CAN", "YELL CACS-GSD", true},
    {"ykro", "YKRO00CIV", "Yamoussoukro Trackin", true},
    {"zeck", "ZECK00RUS", "Zelenchukskaya / Rus", false},
    {"zim2", "ZIM200CHE", "Zimmerwald", true},
    {"zim3", "ZIM300CHE", "Zimmerwald", true},
    {"zimm", "ZIMM00CHE", "Zimmerwald L+T 88", true}
};

#endif
