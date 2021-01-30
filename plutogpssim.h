/**
 * This file is part of the pluto-gps-sim project at
 * https://github.com/mictronics/pluto-gps-sim.git
 * 
 * Copyright © 2015-2018 Mictronics
 * Distributed under the MIT License.
 * 
 */
#ifndef PLUTOGPSSIM_H
#define PLUTOGPSSIM_H

#define FLOAT_CARR_PHASE // For RKT simulation. Higher computational load, but smoother carrier phase.

/*! \brief Maximum length of a line in a text file (RINEX, motion) */
#define MAX_CHAR (100)

/*! \brief Maximum number of satellites in RINEX file */
#define MAX_SAT (32)

/*! \brief Maximum number of channels we simulate */
#define MAX_CHAN (12)

/*! \brief Maximum number of user motion points */
#ifndef USER_MOTION_SIZE
#define USER_MOTION_SIZE (3000) // max duration at 10Hz
#endif

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
    {"abmf", "ABMF00GLP", "Aeroport du Raizet"},
    {"aggo", "AGGO00ARG", "AGGO"},
    {"ajac", "AJAC00FRA", "Ajaccio"},
    {"ankr", "ANKR00TUR", "Ankara"},
    {"areg", "AREG00PER", "Arequipa"},
    {"ascg", "ASCG00SHN", "Ascension"},
    {"bogi", "BOGI00POL", "Borowa Gora"},
    {"bor1", "BOR100POL", "Borowiec"},
    {"brst", "BRST00FRA", "Brest"},
    {"chpg", "CHPG00BRA", "Cachoeira Paulista"},
    {"cibg", "CIBG00IDN", "Cibinong"},
    {"cpvg", "CPVG00CPV", "CAP-VERT"},
    {"djig", "DJIG00DJI", "Djibouti"},
    {"dlf1", "DLF100NLD", "Delft"},
    {"ffmj", "FFMJ00DEU", "Frankfurt/Main"},
    {"ftna", "FTNA00WLF", "Futuna"},
    {"gamb", "GAMB00PYF", "Rikitea"},
    {"gamg", "GAMG00KOR", "Geochang"},
    {"glps", "GLPS00ECU", "Galapagos Permanent Station"},
    {"glsv", "GLSV00UKR", "Kiev/Golosiiv"},
    {"gmsd", "GMSD00JPN", "GUTS Masda"},
    {"gop6", "GOP600CZE", "Pecny, Ondrejov"},
    {"gop7", "GOP700CZE", "Pecny, Ondrejov"},
    {"gope", "GOPE00CZE", "Pecny, Ondrejov"},
    {"grac", "GRAC00FRA", "Grasse"},
    {"gras", "GRAS00FRA", "Observatoire de Calern - OCA"},
    {"holb", "HOLB00CAN", "Holberg"},
    {"hueg", "HUEG00DEU", "Huegelheim"},
    {"ieng", "IENG00ITA", "Torino"},
    {"ista", "ISTA00TUR", "Istanbul"},
    {"izmi", "IZMI00TUR", "Izmir"},
    {"jfng", "JFNG00CHN", "Juifeng"},
    {"joz2", "JOZ200POL", "Jozefoslaw"},
    {"joze", "JOZE00POL", "Jozefoslaw"},
    {"kerg", "KERG00ATF", "Kerguelen Islands"},
    {"kitg", "KITG00UZB", "Kitab"},
    {"koug", "KOUG00GUF", "Kourou"},
    {"krgg", "KRGG00ATF", "Kerguelen Islands"},
    {"krs1", "KRS100TUR", "Kars"},
    {"lama", "LAMA00POL", "Lamkowo"},
    {"leij", "LEIJ00DEU", "Leipzig"},
    {"lmmf", "LMMF00MTQ", "Aeroport Aime CESAIRE-LE LAMENTIN"},
    {"lroc", "LROC00FRA", "La Rochelle"},
    {"mad2", "MAD200ESP", "Madrid Deep Space Tracking Station"},
    {"madr", "MADR00ESP", "Madrid Deep Space Tracking Station"},
    {"mayg", "MAYG00MYT", "Dzaoudzi"},
    {"mers", "MERS00TUR", "Mersin"},
    {"mikl", "MIKL00UKR", "Mykolaiv"},
    {"morp", "MORP00GBR", "Morpeth"},
    {"nklg", "NKLG00GAB", "N'KOLTANG"},
    {"nyal", "NYAL00NOR", "Ny-Alesund"},
    {"nya1", "NYA100NOR", "Ny-Alesund"},
    {"ohi2", "OHI200ATA", "O'Higgins"},
    {"orid", "ORID00MKD", "Ohrid"},
    {"owmg", "OWMG00NZL", "Chatham Island"},
    {"polv", "POLV00UKR", "Poltava"},
    {"ptbb", "PTBB00DEU", "Braunschweig"},
    {"ptgg", "PTGG00PHL", "Manilla"},
    {"rabt", "RABT00MAR", "Rabat, EMI"},
    {"reun", "REUN00REU", "La Reunion - Observatoire Volcanologique"},
    {"rgdg", "RGDG00ARG", "Rio Grande"},
    {"riga", "RIGA00LVA", "RIGA permanent GPS"},
    {"seyg", "SEYG00SYC", "Mahe"},
    {"sofi", "SOFI00BGR", "Sofia"},
    {"stj3", "STJ300CAN", "STJ3 CACS-GSD"},
    {"sulp", "SULP00UKR", "Lviv Polytechnic"},
    {"svtl", "SVTL00RUS", "Svetloe"},
    {"tana", "TANA00ETH", "ILA, Bahir Dar University"},
    {"thtg", "THTG00PYF", "Papeete Tahiti"},
    {"thti", "THTI00PYF", "Tahiti"},
    {"tit2", "TIT200DEU", "Titz / Jackerath"},
    {"tlse", "TLSE00FRA", "Toulouse"},
    {"tro1", "TRO100NOR", "Tromsoe"},
    {"warn", "WARN00DEU", "Warnemuende"},
    {"whit", "WHIT00CAN", "WHIT CACS-GSD"},
    {"wroc", "WROC00POL", "Wroclaw"},
    {"wtza", "WTZA00DEU", "Wettzell"},
    {"yel2", "YEL200CAN", "Yellow Knife"},
    {"zeck", "ZECK00RUS", "Zelenchukskaya"},
    {"zim2", "ZIM200CHE", "Zimmerwald"},
    {"zimm", "ZIMM00CHE", "Zimmerwald L+T 88"},
};

#endif
