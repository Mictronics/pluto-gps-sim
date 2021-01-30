/**
 * pluto-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using the ADALM-Pluto SDR platform.
 *
 * This file is part of the pluto-gps-sim project at
 * https://github.com/mictronics/pluto-gps-sim.git
 *
 * Copyright © 2019 Mictronics
 * Distributed under the MIT License.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <sched.h>
#include <unistd.h>
#include <curl/curl.h>
#include <iio.h>
#include <ad9361.h>
#include <zlib.h>
#if defined(__MACH__) || defined(__APPLE__)
#include <mach/mach_init.h>
#include <mach/thread_act.h>
#include <mach/mach_port.h>
#endif
#include "plutogpssim.h"

#define RINEX2_FILE_NAME "rinex2.gz"
#define RINEX3_FILE_NAME "rinex3.gz"
#define RINEX_FTP_URL "ftp://igs.bkg.bund.de/IGS/"
#define RINEX2_SUBFOLDER "nrt"
#define RINEX3_SUBFOLDER "nrt_v3"
#define RINEX_FTP_FILE "%s/%03i/%02i/%4s%03i%c.%02in.gz"

#define NOTUSED(V) ((void) V)
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))
#define NUM_SAMPLES 260000
#define BUFFER_SIZE (NUM_SAMPLES * 2 * sizeof(int16_t))

#if defined(__MACH__) || defined(__APPLE__)

typedef struct cpu_set {
    uint32_t count;
} cpu_set_t;

static inline void
CPU_ZERO(cpu_set_t *cs) {
    cs->count = 0;
}

static inline void
CPU_SET(int num, cpu_set_t *cs) {
    cs->count |= (1 << num);
}

static inline int
CPU_ISSET(int num, cpu_set_t *cs) {
    return (cs->count & (1 << num));
}
#endif

struct stream_cfg {
    long long bw_hz; // Analog banwidth in Hz
    long long fs_hz; // Baseband sample rate in Hz
    long long lo_hz; // Local oscillator frequency in Hz
    const char* rfport; // Port name
    double gain_db; // Hardware gain
    pthread_cond_t data_cond;
    pthread_t tx_thread;
    pthread_mutex_t data_mutex; // Mutex to synchronize buffer access
    const char *uri;
    const char *hostname;
    bool exit; // Exit from the main loop when true
};

static struct stream_cfg plutotx;
static short *iq_buff = NULL;
static pthread_t pluto_thread;
static char rinex_date[21];

struct ftp_file {
    const char *filename;
    FILE *stream;
};

/* 16-bit IQ DAC amplitude */
const double iq_gain = 18.0;
const int iq_offset = 32;
const int iq_shift = 6;
const int sinTable512[] = {
    0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611, 4011, 4409, 4807, 5205, 5601, 5997,
    6392, 6786, 7179, 7571, 7961, 8351, 8739, 9126, 9511, 9895, 10278, 10659, 11038, 11416, 11792, 12166,
    12539, 12909, 13278, 13645, 14009, 14372, 14732, 15090, 15446, 15799, 16150, 16499, 16845, 17189, 17530, 17868,
    18204, 18537, 18867, 19194, 19519, 19840, 20159, 20474, 20787, 21096, 21402, 21705, 22004, 22301, 22594, 22883,
    23169, 23452, 23731, 24006, 24278, 24546, 24811, 25072, 25329, 25582, 25831, 26077, 26318, 26556, 26789, 27019,
    27244, 27466, 27683, 27896, 28105, 28309, 28510, 28706, 28897, 29085, 29268, 29446, 29621, 29790, 29955, 30116,
    30272, 30424, 30571, 30713, 30851, 30984, 31113, 31236, 31356, 31470, 31580, 31684, 31785, 31880, 31970, 32056,
    32137, 32213, 32284, 32350, 32412, 32468, 32520, 32567, 32609, 32646, 32678, 32705, 32727, 32744, 32757, 32764,
    32767, 32764, 32757, 32744, 32727, 32705, 32678, 32646, 32609, 32567, 32520, 32468, 32412, 32350, 32284, 32213,
    32137, 32056, 31970, 31880, 31785, 31684, 31580, 31470, 31356, 31236, 31113, 30984, 30851, 30713, 30571, 30424,
    30272, 30116, 29955, 29790, 29621, 29446, 29268, 29085, 28897, 28706, 28510, 28309, 28105, 27896, 27683, 27466,
    27244, 27019, 26789, 26556, 26318, 26077, 25831, 25582, 25329, 25072, 24811, 24546, 24278, 24006, 23731, 23452,
    23169, 22883, 22594, 22301, 22004, 21705, 21402, 21096, 20787, 20474, 20159, 19840, 19519, 19194, 18867, 18537,
    18204, 17868, 17530, 17189, 16845, 16499, 16150, 15799, 15446, 15090, 14732, 14372, 14009, 13645, 13278, 12909,
    12539, 12166, 11792, 11416, 11038, 10659, 10278, 9895, 9511, 9126, 8739, 8351, 7961, 7571, 7179, 6786,
    6392, 5997, 5601, 5205, 4807, 4409, 4011, 3611, 3211, 2811, 2410, 2009, 1607, 1206, 804, 402,
    0, -402, -804, -1206, -1607, -2009, -2410, -2811, -3211, -3611, -4011, -4409, -4807, -5205, -5601, -5997,
    -6392, -6786, -7179, -7571, -7961, -8351, -8739, -9126, -9511, -9895, -10278, -10659, -11038, -11416, -11792, -12166,
    -12539, -12909, -13278, -13645, -14009, -14372, -14732, -15090, -15446, -15799, -16150, -16499, -16845, -17189, -17530, -17868,
    -18204, -18537, -18867, -19194, -19519, -19840, -20159, -20474, -20787, -21096, -21402, -21705, -22004, -22301, -22594, -22883,
    -23169, -23452, -23731, -24006, -24278, -24546, -24811, -25072, -25329, -25582, -25831, -26077, -26318, -26556, -26789, -27019,
    -27244, -27466, -27683, -27896, -28105, -28309, -28510, -28706, -28897, -29085, -29268, -29446, -29621, -29790, -29955, -30116,
    -30272, -30424, -30571, -30713, -30851, -30984, -31113, -31236, -31356, -31470, -31580, -31684, -31785, -31880, -31970, -32056,
    -32137, -32213, -32284, -32350, -32412, -32468, -32520, -32567, -32609, -32646, -32678, -32705, -32727, -32744, -32757, -32764,
    -32767, -32764, -32757, -32744, -32727, -32705, -32678, -32646, -32609, -32567, -32520, -32468, -32412, -32350, -32284, -32213,
    -32137, -32056, -31970, -31880, -31785, -31684, -31580, -31470, -31356, -31236, -31113, -30984, -30851, -30713, -30571, -30424,
    -30272, -30116, -29955, -29790, -29621, -29446, -29268, -29085, -28897, -28706, -28510, -28309, -28105, -27896, -27683, -27466,
    -27244, -27019, -26789, -26556, -26318, -26077, -25831, -25582, -25329, -25072, -24811, -24546, -24278, -24006, -23731, -23452,
    -23169, -22883, -22594, -22301, -22004, -21705, -21402, -21096, -20787, -20474, -20159, -19840, -19519, -19194, -18867, -18537,
    -18204, -17868, -17530, -17189, -16845, -16499, -16150, -15799, -15446, -15090, -14732, -14372, -14009, -13645, -13278, -12909,
    -12539, -12166, -11792, -11416, -11038, -10659, -10278, -9895, -9511, -9126, -8739, -8351, -7961, -7571, -7179, -6786,
    -6392, -5997, -5601, -5205, -4807, -4409, -4011, -3611, -3211, -2811, -2410, -2009, -1607, -1206, -804, -402,
};

const int cosTable512[] = {
    32767, 32764, 32757, 32744, 32727, 32705, 32678, 32646, 32609, 32567, 32520, 32468, 32412, 32350, 32284, 32213,
    32137, 32056, 31970, 31880, 31785, 31684, 31580, 31470, 31356, 31236, 31113, 30984, 30851, 30713, 30571, 30424,
    30272, 30116, 29955, 29790, 29621, 29446, 29268, 29085, 28897, 28706, 28510, 28309, 28105, 27896, 27683, 27466,
    27244, 27019, 26789, 26556, 26318, 26077, 25831, 25582, 25329, 25072, 24811, 24546, 24278, 24006, 23731, 23452,
    23169, 22883, 22594, 22301, 22004, 21705, 21402, 21096, 20787, 20474, 20159, 19840, 19519, 19194, 18867, 18537,
    18204, 17868, 17530, 17189, 16845, 16499, 16150, 15799, 15446, 15090, 14732, 14372, 14009, 13645, 13278, 12909,
    12539, 12166, 11792, 11416, 11038, 10659, 10278, 9895, 9511, 9126, 8739, 8351, 7961, 7571, 7179, 6786,
    6392, 5997, 5601, 5205, 4807, 4409, 4011, 3611, 3211, 2811, 2410, 2009, 1607, 1206, 804, 402,
    0, -402, -804, -1206, -1607, -2009, -2410, -2811, -3211, -3611, -4011, -4409, -4807, -5205, -5601, -5997,
    -6392, -6786, -7179, -7571, -7961, -8351, -8739, -9126, -9511, -9895, -10278, -10659, -11038, -11416, -11792, -12166,
    -12539, -12909, -13278, -13645, -14009, -14372, -14732, -15090, -15446, -15799, -16150, -16499, -16845, -17189, -17530, -17868,
    -18204, -18537, -18867, -19194, -19519, -19840, -20159, -20474, -20787, -21096, -21402, -21705, -22004, -22301, -22594, -22883,
    -23169, -23452, -23731, -24006, -24278, -24546, -24811, -25072, -25329, -25582, -25831, -26077, -26318, -26556, -26789, -27019,
    -27244, -27466, -27683, -27896, -28105, -28309, -28510, -28706, -28897, -29085, -29268, -29446, -29621, -29790, -29955, -30116,
    -30272, -30424, -30571, -30713, -30851, -30984, -31113, -31236, -31356, -31470, -31580, -31684, -31785, -31880, -31970, -32056,
    -32137, -32213, -32284, -32350, -32412, -32468, -32520, -32567, -32609, -32646, -32678, -32705, -32727, -32744, -32757, -32764,
    -32767, -32764, -32757, -32744, -32727, -32705, -32678, -32646, -32609, -32567, -32520, -32468, -32412, -32350, -32284, -32213,
    -32137, -32056, -31970, -31880, -31785, -31684, -31580, -31470, -31356, -31236, -31113, -30984, -30851, -30713, -30571, -30424,
    -30272, -30116, -29955, -29790, -29621, -29446, -29268, -29085, -28897, -28706, -28510, -28309, -28105, -27896, -27683, -27466,
    -27244, -27019, -26789, -26556, -26318, -26077, -25831, -25582, -25329, -25072, -24811, -24546, -24278, -24006, -23731, -23452,
    -23169, -22883, -22594, -22301, -22004, -21705, -21402, -21096, -20787, -20474, -20159, -19840, -19519, -19194, -18867, -18537,
    -18204, -17868, -17530, -17189, -16845, -16499, -16150, -15799, -15446, -15090, -14732, -14372, -14009, -13645, -13278, -12909,
    -12539, -12166, -11792, -11416, -11038, -10659, -10278, -9895, -9511, -9126, -8739, -8351, -7961, -7571, -7179, -6786,
    -6392, -5997, -5601, -5205, -4807, -4409, -4011, -3611, -3211, -2811, -2410, -2009, -1607, -1206, -804, -402,
    0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611, 4011, 4409, 4807, 5205, 5601, 5997,
    6392, 6786, 7179, 7571, 7961, 8351, 8739, 9126, 9511, 9895, 10278, 10659, 11038, 11416, 11792, 12166,
    12539, 12909, 13278, 13645, 14009, 14372, 14732, 15090, 15446, 15799, 16150, 16499, 16845, 17189, 17530, 17868,
    18204, 18537, 18867, 19194, 19519, 19840, 20159, 20474, 20787, 21096, 21402, 21705, 22004, 22301, 22594, 22883,
    23169, 23452, 23731, 24006, 24278, 24546, 24811, 25072, 25329, 25582, 25831, 26077, 26318, 26556, 26789, 27019,
    27244, 27466, 27683, 27896, 28105, 28309, 28510, 28706, 28897, 29085, 29268, 29446, 29621, 29790, 29955, 30116,
    30272, 30424, 30571, 30713, 30851, 30984, 31113, 31236, 31356, 31470, 31580, 31684, 31785, 31880, 31970, 32056,
    32137, 32213, 32284, 32350, 32412, 32468, 32520, 32567, 32609, 32646, 32678, 32705, 32727, 32744, 32757, 32764,
};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
const double ant_pat_db[37] = {
    0.00, 0.00, 0.22, 0.44, 0.67, 1.11, 1.56, 2.00, 2.44, 2.89, 3.56, 4.22,
    4.89, 5.56, 6.22, 6.89, 7.56, 8.22, 8.89, 9.78, 10.67, 11.56, 12.44, 13.33,
    14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
    31.56
};

static int allocatedSat[MAX_SAT];

/*! \brief Subtract two vectors of double
 *  \param[out] y Result of subtraction
 *  \param[in] x1 Minuend of subtracion
 *  \param[in] x2 Subtrahend of subtracion
 */
static void subVect(double *y, const double *x1, const double *x2) {
    y[0] = x1[0] - x2[0];
    y[1] = x1[1] - x2[1];
    y[2] = x1[2] - x2[2];

    return;
}

/*! \brief Compute Norm of Vector
 *  \param[in] x Input vector
 *  \returns Length (Norm) of the input vector
 */
static double normVect(const double *x) {
    return (sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]));
}

/*! \brief Compute dot-product of two vectors
 *  \param[in] x1 First multiplicand
 *  \param[in] x2 Second multiplicand
 *  \returns Dot-product of both multiplicands
 */
static double dotProd(const double *x1, const double *x2) {
    return (x1[0] * x2[0] + x1[1] * x2[1] + x1[2] * x2[2]);
}

/* !\brief generate the C/A code sequence for a given Satellite Vehicle PRN
 *  \param[in] prn PRN nuber of the Satellite Vehicle
 *  \param[out] ca Caller-allocated integer array of 1023 bytes
 */
static void codegen(int *ca, int prn) {
    int delay[] = {
        5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
        252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
        473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
        861, 862
    };

    int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
    int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
    int c1, c2;
    int i, j;

    if (prn < 1 || prn > 32)
        return;

    for (i = 0; i < N_DWRD_SBF; i++)
        r1[i] = r2[i] = -1;

    for (i = 0; i < CA_SEQ_LEN; i++) {
        g1[i] = r1[9];
        g2[i] = r2[9];
        c1 = r1[2] * r1[9];
        c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];

        for (j = 9; j > 0; j--) {
            r1[j] = r1[j - 1];
            r2[j] = r2[j - 1];
        }
        r1[0] = c1;
        r2[0] = c2;
    }

    for (i = 0, j = CA_SEQ_LEN - delay[prn - 1]; i < CA_SEQ_LEN; i++, j++)
        ca[i] = (1 - g1[i] * g2[j % CA_SEQ_LEN]) / 2;

    return;
}

/*! \brief Convert a UTC date into a GPS date
 *  \param[in] t input date in UTC form
 *  \param[out] g output date in GPS form
 */
static void date2gps(const datetime_t *t, gpstime_t *g) {
    int doy[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int ye;
    int de;
    int lpdays;

    ye = t->y - 1980;

    // Compute the number of leap days since Jan 5/Jan 6, 1980.
    lpdays = ye / 4 + 1;
    if ((ye % 4) == 0 && t->m <= 2)
        lpdays--;

    // Compute the number of days elapsed since Jan 5/Jan 6, 1980.
    de = ye * 365 + doy[t->m - 1] + t->d + lpdays - 6;

    // Convert time to GPS weeks and seconds.
    g->week = de / 7;
    g->sec = (double) (de % 7) * SECONDS_IN_DAY + t->hh * SECONDS_IN_HOUR
            + t->mm * SECONDS_IN_MINUTE + t->sec;

    return;
}

static void gps2date(const gpstime_t *g, datetime_t *t) {
    // Convert Julian day number to calendar date
    int c = (int) (7 * g->week + floor(g->sec / 86400.0) + 2444245.0) + 1537;
    int d = (int) ((c - 122.1) / 365.25);
    int e = 365 * d + d / 4;
    int f = (int) ((c - e) / 30.6001);

    t->d = c - e - (int) (30.6001 * f);
    t->m = f - 1 - 12 * (f / 14);
    t->y = d - 4715 - ((7 + t->m) / 10);

    t->hh = ((int) (g->sec / 3600.0)) % 24;
    t->mm = ((int) (g->sec / 60.0)) % 60;
    t->sec = g->sec - 60.0 * floor(g->sec / 60.0);

    return;
}

/*! \brief Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Heighth
 *  \param[in] xyz Input Array of X, Y and Z ECEF coordinates
 *  \param[out] llh Output Array of Latitude, Longitude and Height
 */
static void xyz2llh(const double *xyz, double *llh) {
    double a, eps, e, e2;
    double x, y, z;
    double rho2, dz, zdz, nh, slat, n, dz_new;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;

    eps = 1.0e-3;
    e2 = e*e;

    if (normVect(xyz) < eps) {
        // Invalid ECEF vector
        llh[0] = 0.0;
        llh[1] = 0.0;
        llh[2] = -a;

        return;
    }

    x = xyz[0];
    y = xyz[1];
    z = xyz[2];

    rho2 = x * x + y*y;
    dz = e2*z;

    while (1) {
        zdz = z + dz;
        nh = sqrt(rho2 + zdz * zdz);
        slat = zdz / nh;
        n = a / sqrt(1.0 - e2 * slat * slat);
        dz_new = n * e2*slat;

        if (fabs(dz - dz_new) < eps)
            break;

        dz = dz_new;
    }

    llh[0] = atan2(zdz, sqrt(rho2));
    llh[1] = atan2(y, x);
    llh[2] = nh - n;

    return;
}

/*! \brief Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 *  \param[in] llh Input Array of Latitude, Longitude and Height
 *  \param[out] xyz Output Array of X, Y and Z ECEF coordinates
 */
static void llh2xyz(const double *llh, double *xyz) {
    double n;
    double a;
    double e;
    double e2;
    double clat;
    double slat;
    double clon;
    double slon;
    double d, nph;
    double tmp;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;
    e2 = e*e;

    clat = cos(llh[0]);
    slat = sin(llh[0]);
    clon = cos(llh[1]);
    slon = sin(llh[1]);
    d = e*slat;

    n = a / sqrt(1.0 - d * d);
    nph = n + llh[2];

    tmp = nph*clat;
    xyz[0] = tmp*clon;
    xyz[1] = tmp*slon;
    xyz[2] = ((1.0 - e2) * n + llh[2]) * slat;

    return;
}

/*! \brief Compute the intermediate matrix for LLH to ECEF
 *  \param[in] llh Input position in Latitude-Longitude-Height format
 *  \param[out] t Three-by-Three output matrix
 */
static void ltcmat(const double *llh, double t[3][3]) {
    double slat, clat;
    double slon, clon;

    slat = sin(llh[0]);
    clat = cos(llh[0]);
    slon = sin(llh[1]);
    clon = cos(llh[1]);

    t[0][0] = -slat*clon;
    t[0][1] = -slat*slon;
    t[0][2] = clat;
    t[1][0] = -slon;
    t[1][1] = clon;
    t[1][2] = 0.0;
    t[2][0] = clat*clon;
    t[2][1] = clat*slon;
    t[2][2] = slat;

    return;
}

/*! \brief Convert Earth-centered Earth-Fixed to ?
 *  \param[in] xyz Input position as vector in ECEF format
 *  \param[in] t Intermediate matrix computed by \ref ltcmat
 *  \param[out] neu Output position as North-East-Up format
 */
static void ecef2neu(const double *xyz, double t[3][3], double *neu) {
    neu[0] = t[0][0] * xyz[0] + t[0][1] * xyz[1] + t[0][2] * xyz[2];
    neu[1] = t[1][0] * xyz[0] + t[1][1] * xyz[1] + t[1][2] * xyz[2];
    neu[2] = t[2][0] * xyz[0] + t[2][1] * xyz[1] + t[2][2] * xyz[2];

    return;
}

/*! \brief Convert North-Eeast-Up to Azimuth + Elevation
 *  \param[in] neu Input position in North-East-Up format
 *  \param[out] azel Output array of azimuth + elevation as double
 */
static void neu2azel(double *azel, const double *neu) {
    double ne;

    azel[0] = atan2(neu[1], neu[0]);
    if (azel[0] < 0.0)
        azel[0] += (2.0 * PI);

    ne = sqrt(neu[0] * neu[0] + neu[1] * neu[1]);
    azel[1] = atan2(neu[2], ne);

    return;
}

/*! \brief Compute Satellite position, velocity and clock at given time
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at which position is to be computed
 *  \param[out] pos Computed position (vector)
 *  \param[out] vel Computed velociy (vector)
 *  \param[clk] clk Computed clock
 */
static void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk) {
    // Computing Satellite Velocity using the Broadcast Ephemeris
    // http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

    double tk;
    double mk;
    double ek;
    double ekold;
    double ekdot;
    double cek, sek;
    double pk;
    double pkdot;
    double c2pk, s2pk;
    double uk;
    double ukdot;
    double cuk, suk;
    double ok;
    double sok, cok;
    double ik;
    double ikdot;
    double sik, cik;
    double rk;
    double rkdot;
    double xpk, ypk;
    double xpkdot, ypkdot;

    double relativistic, OneMinusecosE, tmp;

    tk = g.sec - eph.toe.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk<-SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    mk = eph.m0 + eph.n*tk;
    ek = mk;
    ekold = ek + 1.0;

    OneMinusecosE = 0; // Suppress the uninitialized warning.
    while (fabs(ek - ekold) > 1.0E-14) {
        ekold = ek;
        OneMinusecosE = 1.0 - eph.ecc * cos(ekold);
        ek = ek + (mk - ekold + eph.ecc * sin(ekold)) / OneMinusecosE;
    }

    sek = sin(ek);
    cek = cos(ek);

    ekdot = eph.n / OneMinusecosE;

    relativistic = -4.442807633E-10 * eph.ecc * eph.sqrta*sek;

    pk = atan2(eph.sq1e2*sek, cek - eph.ecc) + eph.aop;
    pkdot = eph.sq1e2 * ekdot / OneMinusecosE;

    s2pk = sin(2.0 * pk);
    c2pk = cos(2.0 * pk);

    uk = pk + eph.cus * s2pk + eph.cuc*c2pk;
    suk = sin(uk);
    cuk = cos(uk);
    ukdot = pkdot * (1.0 + 2.0 * (eph.cus * c2pk - eph.cuc * s2pk));

    rk = eph.A * OneMinusecosE + eph.crc * c2pk + eph.crs*s2pk;
    rkdot = eph.A * eph.ecc * sek * ekdot + 2.0 * pkdot * (eph.crs * c2pk - eph.crc * s2pk);

    ik = eph.inc0 + eph.idot * tk + eph.cic * c2pk + eph.cis*s2pk;
    sik = sin(ik);
    cik = cos(ik);
    ikdot = eph.idot + 2.0 * pkdot * (eph.cis * c2pk - eph.cic * s2pk);

    xpk = rk*cuk;
    ypk = rk*suk;
    xpkdot = rkdot * cuk - ypk*ukdot;
    ypkdot = rkdot * suk + xpk*ukdot;

    ok = eph.omg0 + tk * eph.omgkdot - OMEGA_EARTH * eph.toe.sec;
    sok = sin(ok);
    cok = cos(ok);

    pos[0] = xpk * cok - ypk * cik*sok;
    pos[1] = xpk * sok + ypk * cik*cok;
    pos[2] = ypk*sik;

    tmp = ypkdot * cik - ypk * sik*ikdot;

    vel[0] = -eph.omgkdot * pos[1] + xpkdot * cok - tmp*sok;
    vel[1] = eph.omgkdot * pos[0] + xpkdot * sok + tmp*cok;
    vel[2] = ypk * cik * ikdot + ypkdot*sik;

    // Satellite clock correction
    tk = g.sec - eph.toc.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk<-SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    clk[0] = eph.af0 + tk * (eph.af1 + tk * eph.af2) + relativistic - eph.tgd;
    clk[1] = eph.af1 + 2.0 * tk * eph.af2;

    return;
}

/*! \brief Compute Subframe from Ephemeris
 *  \param[in] eph Ephemeris of given SV
 *  \param[out] sbf Array of five sub-frames, 10 long words each
 */
static void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, unsigned long sbf[5][N_DWRD_SBF]) {
    unsigned long wn;
    unsigned long toe;
    unsigned long toc;
    unsigned long iode;
    unsigned long iodc;
    long deltan;
    long cuc;
    long cus;
    long cic;
    long cis;
    long crc;
    long crs;
    unsigned long ecc;
    unsigned long sqrta;
    long m0;
    long omg0;
    long inc0;
    long aop;
    long omgdot;
    long idot;
    long af0;
    long af1;
    long af2;
    long tgd;
    int svhlth;
    int codeL2;

    unsigned long ura = 0UL;
    unsigned long dataId = 1UL;
    unsigned long sbf4_page25_svId = 63UL;
    unsigned long sbf5_page25_svId = 51UL;

    unsigned long wna;
    unsigned long toa;

    signed long alpha0, alpha1, alpha2, alpha3;
    signed long beta0, beta1, beta2, beta3;
    signed long A0, A1;
    signed long dtls, dtlsf;
    unsigned long tot, wnt, wnlsf, dn;
    unsigned long sbf4_page18_svId = 56UL;

    // FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
    //wn = (unsigned long)(eph.toe.week%1024);
    wn = 0UL;
    toe = (unsigned long) (eph.toe.sec / 16.0);
    toc = (unsigned long) (eph.toc.sec / 16.0);
    iode = (unsigned long) (eph.iode);
    iodc = (unsigned long) (eph.iodc);
    deltan = (long) (eph.deltan / POW2_M43 / PI);
    cuc = (long) (eph.cuc / POW2_M29);
    cus = (long) (eph.cus / POW2_M29);
    cic = (long) (eph.cic / POW2_M29);
    cis = (long) (eph.cis / POW2_M29);
    crc = (long) (eph.crc / POW2_M5);
    crs = (long) (eph.crs / POW2_M5);
    ecc = (unsigned long) (eph.ecc / POW2_M33);
    sqrta = (unsigned long) (eph.sqrta / POW2_M19);
    m0 = (long) (eph.m0 / POW2_M31 / PI);
    omg0 = (long) (eph.omg0 / POW2_M31 / PI);
    inc0 = (long) (eph.inc0 / POW2_M31 / PI);
    aop = (long) (eph.aop / POW2_M31 / PI);
    omgdot = (long) (eph.omgdot / POW2_M43 / PI);
    idot = (long) (eph.idot / POW2_M43 / PI);
    af0 = (long) (eph.af0 / POW2_M31);
    af1 = (long) (eph.af1 / POW2_M43);
    af2 = (long) (eph.af2 / POW2_M55);
    tgd = (long) (eph.tgd / POW2_M31);
    svhlth = (unsigned long) (eph.svhlth);
    codeL2 = (unsigned long) (eph.codeL2);

    wna = (unsigned long) (eph.toe.week % 256);
    toa = (unsigned long) (eph.toe.sec / 4096.0);

    alpha0 = (signed long) round(ionoutc.alpha0 / POW2_M30);
    alpha1 = (signed long) round(ionoutc.alpha1 / POW2_M27);
    alpha2 = (signed long) round(ionoutc.alpha2 / POW2_M24);
    alpha3 = (signed long) round(ionoutc.alpha3 / POW2_M24);
    beta0 = (signed long) round(ionoutc.beta0 / 2048.0);
    beta1 = (signed long) round(ionoutc.beta1 / 16384.0);
    beta2 = (signed long) round(ionoutc.beta2 / 65536.0);
    beta3 = (signed long) round(ionoutc.beta3 / 65536.0);
    A0 = (signed long) round(ionoutc.A0 / POW2_M30);
    A1 = (signed long) round(ionoutc.A1 / POW2_M50);
    dtls = (signed long) (ionoutc.dtls);
    tot = (unsigned long) (ionoutc.tot / 4096);
    wnt = (unsigned long) (ionoutc.wnt % 256);
    // TO DO: Specify scheduled leap seconds in command options
    // 2016/12/31 (Sat) -> WNlsf = 1929, DN = 7 (http://navigationservices.agi.com/GNSSWeb/)
    // Days are counted from 1 to 7 (Sunday is 1).
    wnlsf = 1929 % 256;
    dn = 7;
    dtlsf = 18;

    // Subframe 1
    sbf[0][0] = 0x8B0000UL << 6;
    sbf[0][1] = 0x1UL << 8;
    sbf[0][2] = ((wn & 0x3FFUL) << 20) | ((codeL2 & 0x3UL) << 18) | ((ura & 0xFUL) << 14) | ((svhlth & 0x3FUL) << 8) | (((iodc >> 8)&0x3UL) << 6);
    sbf[0][3] = 0UL;
    sbf[0][4] = 0UL;
    sbf[0][5] = 0UL;
    sbf[0][6] = (tgd & 0xFFUL) << 6;
    sbf[0][7] = ((iodc & 0xFFUL) << 22) | ((toc & 0xFFFFUL) << 6);
    sbf[0][8] = ((af2 & 0xFFUL) << 22) | ((af1 & 0xFFFFUL) << 6);
    sbf[0][9] = (af0 & 0x3FFFFFUL) << 8;

    // Subframe 2
    sbf[1][0] = 0x8B0000UL << 6;
    sbf[1][1] = 0x2UL << 8;
    sbf[1][2] = ((iode & 0xFFUL) << 22) | ((crs & 0xFFFFUL) << 6);
    sbf[1][3] = ((deltan & 0xFFFFUL) << 14) | (((m0 >> 24)&0xFFUL) << 6);
    sbf[1][4] = (m0 & 0xFFFFFFUL) << 6;
    sbf[1][5] = ((cuc & 0xFFFFUL) << 14) | (((ecc >> 24)&0xFFUL) << 6);
    sbf[1][6] = (ecc & 0xFFFFFFUL) << 6;
    sbf[1][7] = ((cus & 0xFFFFUL) << 14) | (((sqrta >> 24)&0xFFUL) << 6);
    sbf[1][8] = (sqrta & 0xFFFFFFUL) << 6;
    sbf[1][9] = (toe & 0xFFFFUL) << 14;

    // Subframe 3
    sbf[2][0] = 0x8B0000UL << 6;
    sbf[2][1] = 0x3UL << 8;
    sbf[2][2] = ((cic & 0xFFFFUL) << 14) | (((omg0 >> 24)&0xFFUL) << 6);
    sbf[2][3] = (omg0 & 0xFFFFFFUL) << 6;
    sbf[2][4] = ((cis & 0xFFFFUL) << 14) | (((inc0 >> 24)&0xFFUL) << 6);
    sbf[2][5] = (inc0 & 0xFFFFFFUL) << 6;
    sbf[2][6] = ((crc & 0xFFFFUL) << 14) | (((aop >> 24)&0xFFUL) << 6);
    sbf[2][7] = (aop & 0xFFFFFFUL) << 6;
    sbf[2][8] = (omgdot & 0xFFFFFFUL) << 6;
    sbf[2][9] = ((iode & 0xFFUL) << 22) | ((idot & 0x3FFFUL) << 8);

    if (ionoutc.vflg == true) {
        // Subframe 4, page 18
        sbf[3][0] = 0x8B0000UL << 6;
        sbf[3][1] = 0x4UL << 8;
        sbf[3][2] = (dataId << 28) | (sbf4_page18_svId << 22) | ((alpha0 & 0xFFUL) << 14) | ((alpha1 & 0xFFUL) << 6);
        sbf[3][3] = ((alpha2 & 0xFFUL) << 22) | ((alpha3 & 0xFFUL) << 14) | ((beta0 & 0xFFUL) << 6);
        sbf[3][4] = ((beta1 & 0xFFUL) << 22) | ((beta2 & 0xFFUL) << 14) | ((beta3 & 0xFFUL) << 6);
        sbf[3][5] = (A1 & 0xFFFFFFUL) << 6;
        sbf[3][6] = ((A0 >> 8)&0xFFFFFFUL) << 6;
        sbf[3][7] = ((A0 & 0xFFUL) << 22) | ((tot & 0xFFUL) << 14) | ((wnt & 0xFFUL) << 6);
        sbf[3][8] = ((dtls & 0xFFUL) << 22) | ((wnlsf & 0xFFUL) << 14) | ((dn & 0xFFUL) << 6);
        sbf[3][9] = (dtlsf & 0xFFUL) << 22;

    } else {
        // Subframe 4, page 25
        sbf[3][0] = 0x8B0000UL << 6;
        sbf[3][1] = 0x4UL << 8;
        sbf[3][2] = (dataId << 28) | (sbf4_page25_svId << 22);
        sbf[3][3] = 0UL;
        sbf[3][4] = 0UL;
        sbf[3][5] = 0UL;
        sbf[3][6] = 0UL;
        sbf[3][7] = 0UL;
        sbf[3][8] = 0UL;
        sbf[3][9] = 0UL;
    }

    // Subframe 5, page 25
    sbf[4][0] = 0x8B0000UL << 6;
    sbf[4][1] = 0x5UL << 8;
    sbf[4][2] = (dataId << 28) | (sbf5_page25_svId << 22) | ((toa & 0xFFUL) << 14) | ((wna & 0xFFUL) << 6);
    sbf[4][3] = 0UL;
    sbf[4][4] = 0UL;
    sbf[4][5] = 0UL;
    sbf[4][6] = 0UL;
    sbf[4][7] = 0UL;
    sbf[4][8] = 0UL;
    sbf[4][9] = 0UL;

    return;
}

/*! \brief Count number of bits set to 1
 *  \param[in] v long word in which bits are counted
 *  \returns Count of bits set to 1
 */
static unsigned long countBits(unsigned long v) {
    unsigned long c;
    const int S[] = {1, 2, 4, 8, 16};
    const unsigned long B[] = {
        0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF
    };

    c = v;
    c = ((c >> S[0]) & B[0]) + (c & B[0]);
    c = ((c >> S[1]) & B[1]) + (c & B[1]);
    c = ((c >> S[2]) & B[2]) + (c & B[2]);
    c = ((c >> S[3]) & B[3]) + (c & B[3]);
    c = ((c >> S[4]) & B[4]) + (c & B[4]);

    return (c);
}

/*! \brief Compute the Checksum for one given word of a subframe
 *  \param[in] source The input data
 *  \param[in] nib Does this word contain non-information-bearing bits?
 *  \returns Computed Checksum
 */
static unsigned long computeChecksum(unsigned long source, int nib) {
    /*
    Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
    Bits 29 to  6 = Source data bits, d1, d2, ..., d24
    Bits  5 to  0 = Empty parity bits
     */

    /*
    Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
    Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
    Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
     */

    /*
                      1            2           3
    bit    12 3456 7890 1234 5678 9012 3456 7890
    ---    -------------------------------------
    D25    11 1011 0001 1111 0011 0100 1000 0000
    D26    01 1101 1000 1111 1001 1010 0100 0000
    D27    10 1110 1100 0111 1100 1101 0000 0000
    D28    01 0111 0110 0011 1110 0110 1000 0000
    D29    10 1011 1011 0001 1111 0011 0100 0000
    D30    00 1011 0111 1010 1000 1001 1100 0000
     */

    unsigned long bmask[6] = {
        0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
        0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL
    };

    unsigned long D;
    unsigned long d = source & 0x3FFFFFC0UL;
    unsigned long D29 = (source >> 31)&0x1UL;
    unsigned long D30 = (source >> 30)&0x1UL;

    if (nib) // Non-information bearing bits for word 2 and 10
    {
        /*
        Solve bits 23 and 24 to presearve parity check
        with zeros in bits 29 and 30.
         */

        if ((D30 + countBits(bmask[4] & d)) % 2)
            d ^= (0x1UL << 6);
        if ((D29 + countBits(bmask[5] & d)) % 2)
            d ^= (0x1UL << 7);
    }

    D = d;
    if (D30)
        D ^= 0x3FFFFFC0UL;

    D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
    D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
    D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
    D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
    D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
    D |= ((D29 + countBits(bmask[5] & d)) % 2);

    D &= 0x3FFFFFFFUL;
    //D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits

    return (D);
}

/*! \brief Replace all 'E' exponential designators to 'D'
 *  \param str String in which all occurrences of 'E' are replaced with *  'D'
 *  \param len Length of input string in bytes
 *  \returns Number of characters replaced
 */
static int replaceExpDesignator(char *str, int len) {
    int i, n = 0;

    for (i = 0; i < len; i++) {
        if (str[i] == 0) {
            break;
        }

        if (str[i] == 'D' || str[i] == 'd') {
            n++;
            str[i] = 'E';
        }
    }

    return (n);
}

static double subGpsTime(gpstime_t g1, gpstime_t g0) {
    double dt;

    dt = g1.sec - g0.sec;
    dt += (double) (g1.week - g0.week) * SECONDS_IN_WEEK;

    return (dt);
}

static gpstime_t incGpsTime(gpstime_t g0, double dt) {
    gpstime_t g1;

    g1.week = g0.week;
    g1.sec = g0.sec + dt;

    g1.sec = round(g1.sec * 1000.0) / 1000.0; // Avoid rounding error

    while (g1.sec >= SECONDS_IN_WEEK) {
        g1.sec -= SECONDS_IN_WEEK;
        g1.week++;
    }

    while (g1.sec < 0.0) {
        g1.sec += SECONDS_IN_WEEK;
        g1.week--;
    }

    return (g1);
}

/*! \brief Read Ephemeris data from the RINEX v2 Navigation file */

/*  \param[out] eph Array of Output SV ephemeris data
 *  \param[in] fname File name of the RINEX file
 *  \returns Number of sets of ephemerides in the file
 */
static int readRinex2(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname) {
    struct gzFile_s *fp;
    int ieph;

    int sv;
    char str[MAX_CHAR];
    char tmp[20];
    double ver = 0.0;

    datetime_t t;
    gpstime_t g;
    gpstime_t g0;
    double dt;

    int flags = 0x0;

    if (NULL == (fp = gzopen(fname, "rt")))
        return (-1);

    // Clear valid flag
    for (ieph = 0; ieph < EPHEM_ARRAY_SIZE; ieph++)
        for (sv = 0; sv < MAX_SAT; sv++)
            eph[ieph][sv].vflg = false;

    // Read header lines
    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        if (strncmp(str + 60, "COMMENT", 7) == 0) {
            continue;
        } else if (strncmp(str + 60, "END OF HEADER", 13) == 0) {
            break;
        } else if (strncmp(str + 60, "RINEX VERSION / TYPE", 20) == 0) {
            strncpy(tmp, str, 9);
            tmp[9] = 0;
            replaceExpDesignator(tmp, 9);
            ver = atof(tmp);
            if (ver > 3.0) {
                gzclose(fp);
                return -2;
            }

            if (str[20] != 'N') {
                gzclose(fp);
                return -3;
            }
        } else if (strncmp(str + 60, "PGM / RUN BY / DATE", 19) == 0) {
            strncpy(rinex_date, str + 40, 20);
            rinex_date[20] = 0;
        } else if (strncmp(str + 60, "ION ALPHA", 9) == 0) {
            strncpy(tmp, str + 2, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha0 = atof(tmp);

            strncpy(tmp, str + 14, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha1 = atof(tmp);

            strncpy(tmp, str + 26, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha2 = atof(tmp);

            strncpy(tmp, str + 38, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->alpha3 = atof(tmp);

            flags |= 0x1;
        } else if (strncmp(str + 60, "ION BETA", 8) == 0) {
            strncpy(tmp, str + 2, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta0 = atof(tmp);

            strncpy(tmp, str + 14, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta1 = atof(tmp);

            strncpy(tmp, str + 26, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta2 = atof(tmp);

            strncpy(tmp, str + 38, 12);
            tmp[12] = 0;
            replaceExpDesignator(tmp, 12);
            ionoutc->beta3 = atof(tmp);

            flags |= 0x1 << 1;
        } else if (strncmp(str + 60, "DELTA-UTC", 9) == 0) {
            strncpy(tmp, str + 3, 19);
            tmp[19] = 0;
            replaceExpDesignator(tmp, 19);
            ionoutc->A0 = atof(tmp);

            strncpy(tmp, str + 22, 19);
            tmp[19] = 0;
            replaceExpDesignator(tmp, 19);
            ionoutc->A1 = atof(tmp);

            strncpy(tmp, str + 41, 9);
            tmp[9] = 0;
            ionoutc->tot = atoi(tmp);

            strncpy(tmp, str + 50, 9);
            tmp[9] = 0;
            ionoutc->wnt = atoi(tmp);

            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (strncmp(str + 60, "LEAP SECONDS", 12) == 0) {
            strncpy(tmp, str, 6);
            tmp[6] = 0;
            ionoutc->dtls = atoi(tmp);

            flags |= 0x1 << 3;
        }
    }

    ionoutc->vflg = false;
    if (flags == 0xF) // Read all Iono/UTC lines
        ionoutc->vflg = true;

    // Read ephemeris blocks
    g0.week = -1;
    ieph = 0;

    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // PRN
        strncpy(tmp, str, 2);
        tmp[2] = 0;
        sv = atoi(tmp) - 1;

        // EPOCH
        strncpy(tmp, str + 3, 2);
        tmp[2] = 0;
        t.y = atoi(tmp) + 2000;

        strncpy(tmp, str + 6, 2);
        tmp[2] = 0;
        t.m = atoi(tmp);

        strncpy(tmp, str + 9, 2);
        tmp[2] = 0;
        t.d = atoi(tmp);

        strncpy(tmp, str + 12, 2);
        tmp[2] = 0;
        t.hh = atoi(tmp);

        strncpy(tmp, str + 15, 2);
        tmp[2] = 0;
        t.mm = atoi(tmp);

        strncpy(tmp, str + 18, 4);
        tmp[2] = 0;
        t.sec = atof(tmp);

        date2gps(&t, &g);

        if (g0.week == -1)
            g0 = g;

        // Check current time of clock
        dt = subGpsTime(g, g0);

        if (dt > SECONDS_IN_HOUR) {
            g0 = g;
            ieph++; // a new set of ephemerides

            if (ieph >= EPHEM_ARRAY_SIZE)
                break;
        }

        // Date and time
        eph[ieph][sv].t = t;

        // SV CLK
        eph[ieph][sv].toc = g;

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19); // tmp[15]='E';
        eph[ieph][sv].af0 = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af1 = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af2 = atof(tmp);

        // BROADCAST ORBIT - 1
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iode = (int) atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crs = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].deltan = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].m0 = atof(tmp);

        // BROADCAST ORBIT - 2
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cuc = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].ecc = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cus = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].sqrta = atof(tmp);

        // BROADCAST ORBIT - 3
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.sec = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cic = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omg0 = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cis = atof(tmp);

        // BROADCAST ORBIT - 4
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].inc0 = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crc = atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].aop = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omgdot = atof(tmp);

        // BROADCAST ORBIT - 5
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 3, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].idot = atof(tmp);

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].codeL2 = (int) atof(tmp);

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.week = (int) atof(tmp);

        // BROADCAST ORBIT - 6
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 22, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].svhlth = (int) atof(tmp);
        if ((eph[ieph][sv].svhlth > 0) && (eph[ieph][sv].svhlth < 32))
            eph[ieph][sv].svhlth += 32; // Set MSB to 1

        strncpy(tmp, str + 41, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].tgd = atof(tmp);

        strncpy(tmp, str + 60, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iodc = (int) atof(tmp);

        // BROADCAST ORBIT - 7
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // Set valid flag
        eph[ieph][sv].vflg = true;

        // Update the working variables
        eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
        eph[ieph][sv].n = sqrt(GM_EARTH / (eph[ieph][sv].A * eph[ieph][sv].A * eph[ieph][sv].A)) + eph[ieph][sv].deltan;
        eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc * eph[ieph][sv].ecc);
        eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
    }

    gzclose(fp);

    if (g0.week >= 0)
        ieph += 1; // Number of sets of ephemerides

    return (ieph);
}

/*! \brief Read Ephemeris data from the RINEX v3 Navigation file */

/*  \param[out] eph Array of Output SV ephemeris data
 *  \param[in] fname File name of the RINEX file
 *  \returns Number of sets of ephemerides in the file
 */
static int readRinex3(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname) {
    struct gzFile_s *fp;
    int ieph;

    int sv;
    char str[MAX_CHAR];
    char tmp[20];
    double ver = 0.0;

    datetime_t t;
    gpstime_t g;
    gpstime_t g0;
    double dt;

    int flags = 0x0;

    if (NULL == (fp = gzopen(fname, "rt")))
        return (-1);

    // Clear valid flag
    for (ieph = 0; ieph < EPHEM_ARRAY_SIZE; ieph++)
        for (sv = 0; sv < MAX_SAT; sv++)
            eph[ieph][sv].vflg = false;

    // Read header lines
    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        if (strncmp(str + 60, "COMMENT", 7) == 0) {
            continue;
        } else if (strncmp(str + 60, "END OF HEADER", 13) == 0) {
            break;
        } else if (strncmp(str + 60, "RINEX VERSION / TYPE", 20) == 0) {
            strncpy(tmp, str, 9);
            tmp[9] = 0;
            replaceExpDesignator(tmp, 9);
            ver = atof(tmp);
            if (ver < 3.0) {
                gzclose(fp);
                return -2;
            }

            if (str[20] != 'N' && str[40] != 'G') {
                gzclose(fp);
                return -3;
            }
        } else if (strncmp(str + 60, "PGM / RUN BY / DATE", 19) == 0) {
            strncpy(rinex_date, str + 40, 20);
            rinex_date[20] = 0;
        } else if (strncmp(str + 60, "IONOSPHERIC CORR", 16) == 0) {
            if (strncmp(str, "GPSA", 4) == 0) {
                strncpy(tmp, str + 5, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha0 = atof(tmp);

                strncpy(tmp, str + 17, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha1 = atof(tmp);

                strncpy(tmp, str + 29, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha2 = atof(tmp);

                strncpy(tmp, str + 41, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->alpha3 = atof(tmp);

                flags |= 0x1;
            } else if (strncmp(str, "GPSB", 4) == 0) {
                strncpy(tmp, str + 5, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta0 = atof(tmp);

                strncpy(tmp, str + 17, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta1 = atof(tmp);

                strncpy(tmp, str + 29, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta2 = atof(tmp);

                strncpy(tmp, str + 41, 12);
                tmp[12] = 0;
                replaceExpDesignator(tmp, 12);
                ionoutc->beta3 = atof(tmp);

                flags |= 0x1 << 1;
            }
        } else if (strncmp(str + 60, "TIME SYSTEM CORR", 16) == 0 && strncmp(str, "GPUT", 4) == 0) {
            strncpy(tmp, str + 5, 17);
            tmp[17] = 0;
            replaceExpDesignator(tmp, 17);
            ionoutc->A0 = atof(tmp);

            strncpy(tmp, str + 22, 16);
            tmp[16] = 0;
            replaceExpDesignator(tmp, 16);
            ionoutc->A1 = atof(tmp);

            strncpy(tmp, str + 38, 7);
            tmp[7] = 0;
            replaceExpDesignator(tmp, 7);
            ionoutc->tot = atoi(tmp);

            strncpy(tmp, str + 45, 6);
            tmp[6] = 0;
            ionoutc->wnt = atoi(tmp);

            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (strncmp(str + 60, "LEAP SECONDS", 12) == 0) {
            strncpy(tmp, str, 6);
            tmp[6] = 0;
            ionoutc->dtls = atoi(tmp);

            flags |= 0x1 << 3;
        }
    }

    ionoutc->vflg = false;
    if (flags == 0xF) // Read all Iono/UTC lines
        ionoutc->vflg = true;

    // Read ephemeris blocks
    g0.week = -1;
    ieph = 0;

    while (1) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // Check for GPS data record
        if (str[0] != 'G') {
            continue;
        }

        // PRN
        strncpy(tmp, str + 1, 2);
        tmp[2] = 0;
        sv = atoi(tmp) - 1;

        // EPOCH
        strncpy(tmp, str + 4, 4);
        tmp[4] = 0;
        t.y = atoi(tmp);

        strncpy(tmp, str + 9, 2);
        tmp[2] = 0;
        t.m = atoi(tmp);

        strncpy(tmp, str + 12, 2);
        tmp[2] = 0;
        t.d = atoi(tmp);

        strncpy(tmp, str + 15, 2);
        tmp[2] = 0;
        t.hh = atoi(tmp);

        strncpy(tmp, str + 18, 2);
        tmp[2] = 0;
        t.mm = atoi(tmp);

        strncpy(tmp, str + 21, 2);
        tmp[2] = 0;
        t.sec = (double) atoi(tmp);

        date2gps(&t, &g);

        if (g0.week == -1)
            g0 = g;

        // Check current time of clock
        dt = subGpsTime(g, g0);

        if (dt > SECONDS_IN_HOUR) {
            g0 = g;
            ieph++; // a new set of ephemerides

            if (ieph >= EPHEM_ARRAY_SIZE)
                break;
        }

        // Date and time
        eph[ieph][sv].t = t;

        // SV CLK
        eph[ieph][sv].toc = g;

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af0 = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af1 = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].af2 = atof(tmp);

        // BROADCAST ORBIT - 1
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iode = (int) atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crs = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].deltan = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].m0 = atof(tmp);

        // BROADCAST ORBIT - 2
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cuc = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].ecc = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cus = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].sqrta = atof(tmp);

        // BROADCAST ORBIT - 3
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.sec = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cic = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omg0 = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].cis = atof(tmp);

        // BROADCAST ORBIT - 4
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].inc0 = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].crc = atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].aop = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].omgdot = atof(tmp);

        // BROADCAST ORBIT - 5
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        strncpy(tmp, str + 4, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].idot = atof(tmp);

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].codeL2 = (int) atof(tmp);

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].toe.week = (int) atof(tmp);

        // BROADCAST ORBIT - 6
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // SV accuracy not read

        strncpy(tmp, str + 23, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].svhlth = (int) atof(tmp);
        if ((eph[ieph][sv].svhlth > 0) && (eph[ieph][sv].svhlth < 32))
            eph[ieph][sv].svhlth += 32; // Set MSB to 1

        strncpy(tmp, str + 42, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].tgd = atof(tmp);

        strncpy(tmp, str + 61, 19);
        tmp[19] = 0;
        replaceExpDesignator(tmp, 19);
        eph[ieph][sv].iodc = (int) atof(tmp);

        // BROADCAST ORBIT - 7
        if (NULL == gzgets(fp, str, MAX_CHAR))
            break;

        // Set valid flag
        eph[ieph][sv].vflg = true;

        // Update the working variables
        eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
        eph[ieph][sv].n = sqrt(GM_EARTH / (eph[ieph][sv].A * eph[ieph][sv].A * eph[ieph][sv].A)) + eph[ieph][sv].deltan;
        eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc * eph[ieph][sv].ecc);
        eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
    }

    gzclose(fp);

    if (g0.week >= 0)
        ieph += 1; // Number of sets of ephemerides

    return (ieph);
}

static double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel) {
    double iono_delay = 0.0;
    double E, phi_u, lam_u, F;

    if (ionoutc->enable == false)
        return (0.0); // No ionospheric delay

    E = azel[1] / PI;
    phi_u = llh[0] / PI;
    lam_u = llh[1] / PI;

    // Obliquity factor
    F = 1.0 + 16.0 * pow((0.53 - E), 3.0);

    if (ionoutc->vflg == false)
        iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
    else {
        double t, psi, phi_i, lam_i, phi_m, phi_m2, phi_m3;
        double AMP, PER, X, X2, X4;

        // Earth's central angle between the user position and the earth projection of
        // ionospheric intersection point (semi-circles)
        psi = 0.0137 / (E + 0.11) - 0.022;

        // Geodetic latitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        phi_i = phi_u + psi * cos(azel[0]);
        if (phi_i > 0.416)
            phi_i = 0.416;
        else if (phi_i<-0.416)
            phi_i = -0.416;

        // Geodetic longitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        lam_i = lam_u + psi * sin(azel[0]) / cos(phi_i * PI);

        // Geomagnetic latitude of the earth projection of the ionospheric intersection
        // point (mean ionospheric height assumed 350 km) (semi-circles)
        phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * PI);
        phi_m2 = phi_m*phi_m;
        phi_m3 = phi_m2*phi_m;

        AMP = ionoutc->alpha0 + ionoutc->alpha1 * phi_m
                + ionoutc->alpha2 * phi_m2 + ionoutc->alpha3*phi_m3;
        if (AMP < 0.0)
            AMP = 0.0;

        PER = ionoutc->beta0 + ionoutc->beta1 * phi_m
                + ionoutc->beta2 * phi_m2 + ionoutc->beta3*phi_m3;
        if (PER < 72000.0)
            PER = 72000.0;

        // Local time (sec)
        t = SECONDS_IN_DAY / 2.0 * lam_i + g.sec;
        while (t >= SECONDS_IN_DAY)
            t -= SECONDS_IN_DAY;
        while (t < 0)
            t += SECONDS_IN_DAY;

        // Phase (radians)
        X = 2.0 * PI * (t - 50400.0) / PER;

        if (fabs(X) < 1.57) {
            X2 = X*X;
            X4 = X2*X2;
            iono_delay = F * (5.0e-9 + AMP * (1.0 - X2 / 2.0 + X4 / 24.0)) * SPEED_OF_LIGHT;
        } else
            iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
    }

    return (iono_delay);
}

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
static void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[]) {
    double pos[3], vel[3], clk[2];
    double los[3];
    double tau;
    double range, rate;
    double xrot, yrot;

    double llh[3], neu[3];
    double tmat[3][3];

    // SV position at time of the pseudorange observation.
    satpos(eph, g, pos, vel, clk);

    // Receiver to satellite vector and light-time.
    subVect(los, pos, xyz);
    tau = normVect(los) / SPEED_OF_LIGHT;

    // Extrapolate the satellite position backwards to the transmission time.
    pos[0] -= vel[0] * tau;
    pos[1] -= vel[1] * tau;
    pos[2] -= vel[2] * tau;

    // Earth rotation correction. The change in velocity can be neglected.
    xrot = pos[0] + pos[1] * OMEGA_EARTH*tau;
    yrot = pos[1] - pos[0] * OMEGA_EARTH*tau;
    pos[0] = xrot;
    pos[1] = yrot;

    // New observer to satellite vector and satellite range.
    subVect(los, pos, xyz);
    range = normVect(los);
    rho->d = range;

    // Pseudorange.
    rho->range = range - SPEED_OF_LIGHT * clk[0];

    // Relative velocity of SV and receiver.
    rate = dotProd(vel, los) / range;

    // Pseudorange rate.
    rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

    // Time of application.
    rho->g = g;

    // Azimuth and elevation angles.
    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);
    ecef2neu(los, tmat, neu);
    neu2azel(rho->azel, neu);

    // Add ionospheric delay
    rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
    rho->range += rho->iono_delay;

    return;
}

/*! \brief Compute the code phase for a given channel (satellite)
 *  \param chan Channel on which we operate (is updated)
 *  \param[in] rho1 Current range, after \a dt has expired
 *  \param[in dt delta-t (time difference) in seconds
 */
static void computeCodePhase(channel_t *chan, range_t rho1, double dt) {
    double ms;
    int ims;
    double rhorate;

    // Pseudorange rate.
    rhorate = (rho1.range - chan->rho0.range) / dt;

    // Carrier and code frequency.
    chan->f_carr = -rhorate / LAMBDA_L1;
    chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

    // Initial code phase and data bit counters.
    ms = ((subGpsTime(chan->rho0.g, chan->g0) + 6.0) - chan->rho0.range / SPEED_OF_LIGHT)*1000.0;

    ims = (int) ms;
    chan->code_phase = (ms - (double) ims) * CA_SEQ_LEN; // in chip

    chan->iword = ims / 600; // 1 word = 30 bits = 600 ms
    ims -= chan->iword * 600;

    chan->ibit = ims / 20; // 1 bit = 20 code = 20 ms
    ims -= chan->ibit * 20;

    chan->icode = ims; // 1 code = 1 ms

    chan->codeCA = chan->ca[(int) chan->code_phase]*2 - 1;
    chan->dataBit = (int) ((chan->dwrd[chan->iword]>>(29 - chan->ibit)) & 0x1UL)*2 - 1;

    // Save current pseudorange
    chan->rho0 = rho1;

    return;
}

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of ECEF vectors for user motion
 *  \param[[in] filename File name of the text input file
 *  \returns Number of user data motion records read, -1 on error
 */
static int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename) {
    FILE *fp;
    int numd;
    char str[MAX_CHAR];
    double t, x, y, z;

    if (NULL == (fp = fopen(filename, "rt")))
        return (-1);

    for (numd = 0; numd < USER_MOTION_SIZE; numd++) {
        if (fgets(str, MAX_CHAR, fp) == NULL)
            break;

        if (EOF == sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z)) // Read CSV line
            break;

        xyz[numd][0] = x;
        xyz[numd][1] = y;
        xyz[numd][2] = z;
    }

    fclose(fp);

    return (numd);
}

static int generateNavMsg(gpstime_t g, channel_t *chan, int init) {
    int iwrd, isbf;
    gpstime_t g0;
    unsigned long wn, tow;
    unsigned sbfwrd;
    unsigned long prevwrd;
    int nib;

    g0.week = g.week;
    g0.sec = (double) (((unsigned long) (g.sec + 0.5)) / 30UL) * 30.0; // Align with the full frame length = 30 sec
    chan->g0 = g0; // Data bit reference time

    wn = (unsigned long) (g0.week % 1024);
    tow = ((unsigned long) g0.sec) / 6UL;

    if (init == 1) // Initialize subframe 5
    {
        prevwrd = 0UL;

        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            sbfwrd = chan->sbf[4][iwrd];

            // Add TOW-count message into HOW
            if (iwrd == 1)
                sbfwrd |= ((tow & 0x1FFFFUL) << 13);

            // Compute checksum
            sbfwrd |= (prevwrd << 30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd == 1) || (iwrd == 9)) ? 1 : 0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[iwrd];
        }
    } else // Save subframe 5
    {
        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF * N_SBF + iwrd];

            prevwrd = chan->dwrd[iwrd];
        }
        /*
        // Sanity check
        if (((chan->dwrd[1])&(0x1FFFFUL<<13)) != ((tow&0x1FFFFUL)<<13))
        {
                fprintf(stderr, "\nWARNING: Invalid TOW in subframe 5.\n");
                return(0);
        }
         */
    }

    for (isbf = 0; isbf < N_SBF; isbf++) {
        tow++;

        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            sbfwrd = chan->sbf[isbf][iwrd];

            // Add transmission week number to Subframe 1
            if ((isbf == 0)&&(iwrd == 2))
                sbfwrd |= (wn & 0x3FFUL) << 20;

            // Add TOW-count message into HOW
            if (iwrd == 1)
                sbfwrd |= ((tow & 0x1FFFFUL) << 13);

            // Compute checksum
            sbfwrd |= (prevwrd << 30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd == 1) || (iwrd == 9)) ? 1 : 0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[(isbf + 1) * N_DWRD_SBF + iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[(isbf + 1) * N_DWRD_SBF + iwrd];
        }
    }

    return (1);
}

static int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel) {
    double llh[3], neu[3];
    double pos[3], vel[3], clk[3], los[3];
    double tmat[3][3];

    if (eph.vflg == false)
        return (-1); // Invalid

    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);

    satpos(eph, g, pos, vel, clk);
    subVect(los, pos, xyz);
    ecef2neu(los, tmat, neu);
    neu2azel(azel, neu);

    if (azel[1] * R2D > elvMask)
        return (1); // Visible
    // else
    return (0); // Invisible
}

static int allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc, gpstime_t grx, double *xyz, double elvMask) {
    NOTUSED(elvMask);
    int nsat = 0;
    int i, sv;
    double azel[2];

    range_t rho;
    double ref[3] = {0.0};
    double r_ref, r_xyz;
    double phase_ini;

    for (sv = 0; sv < MAX_SAT; sv++) {
        if (checkSatVisibility(eph[sv], grx, xyz, 0.0, azel) == 1) {
            nsat++; // Number of visible satellites

            if (allocatedSat[sv] == -1) // Visible but not allocated
            {
                // Allocated new satellite
                for (i = 0; i < MAX_CHAN; i++) {
                    if (chan[i].prn == 0) {
                        // Initialize channel
                        chan[i].prn = sv + 1;
                        chan[i].azel[0] = azel[0];
                        chan[i].azel[1] = azel[1];

                        // C/A code generation
                        codegen(chan[i].ca, chan[i].prn);

                        // Generate subframe
                        eph2sbf(eph[sv], ionoutc, chan[i].sbf);

                        // Generate navigation message
                        generateNavMsg(grx, &chan[i], 1);

                        // Initialize pseudorange
                        computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
                        chan[i].rho0 = rho;

                        // Initialize carrier phase
                        r_xyz = rho.range;

                        computeRange(&rho, eph[sv], &ionoutc, grx, ref);
                        r_ref = rho.range;

                        phase_ini = (2.0 * r_ref - r_xyz) / LAMBDA_L1;
#ifdef FLOAT_CARR_PHASE
                        chan[i].carr_phase = phase_ini - floor(phase_ini);
#else
                        phase_ini -= floor(phase_ini);
                        chan[i].carr_phase = (unsigned int) (512.0 * 65536.0 * phase_ini);
#endif
                        // Done.
                        break;
                    }
                }

                // Set satellite allocation channel
                if (i < MAX_CHAN)
                    allocatedSat[sv] = i;
            }
        } else if (allocatedSat[sv] >= 0) // Not visible but allocated
        {
            // Clear channel
            chan[allocatedSat[sv]].prn = 0;

            // Clear satellite allocation flag
            allocatedSat[sv] = -1;
        }
    }

    return (nsat);
}

static void usage(void) {
    fprintf(stderr, "Usage: pluto-gps-sim [options]\n"
            "Options:\n"
            "  -e <file name>   RINEX navigation file for GPS ephemerides (required)\n"
            "  -u <file name>   User motion file (dynamic mode) 10Hz, Max %u points\n"
            "  -3               Use RINEX version 3 format\n"
            "  -f               Pull actual RINEX navigation file from NASA FTP server\n"
            "  -c <location>    ECEF X,Y,Z in meters (static mode) e.g. 3967283.154,1022538.181,4872414.484\n"
            "  -l <location>    Lat,Lon,Hgt (static mode) e.g. 35.681298,139.766247,10.0\n"
            "  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss\n"
            "  -T <date,time>   Overwrite TOC and TOE to scenario start time (use 'now' for actual time)\n"
            "  -s <frequency>   Sampling frequency [Hz] (default: 2600000)\n"
            "  -i               Disable ionospheric delay for spacecraft scenario\n"
            "  -v               Show details about simulated channels\n"
            "  -A <attenuation> Set TX attenuation [dB] (default -20.0)\n"
            "  -B <bw>          Set RF bandwidth [MHz] (default 3.0)\n"
            "  -U <uri>         ADALM-Pluto URI\n"
            "  -N <network>     ADALM-Pluto network IP or hostname (default pluto.local)\n",
            (unsigned int) USER_MOTION_SIZE);

    return;
}

static void handle_sig(int sig) {
    NOTUSED(sig);
    signal(SIGINT, SIG_DFL); // reset signal handler - bit extra safety
    pthread_mutex_unlock(&plutotx.data_mutex);
    plutotx.exit = true;
    pthread_join(pluto_thread, NULL); /* Wait on Pluto TX thread exit */
    pthread_mutex_destroy(&plutotx.data_mutex);
    pthread_cond_destroy(&plutotx.data_cond);
}

#if defined(__MACH__) || defined(__APPLE__)

static int pthread_setaffinity_np(pthread_t thread, size_t cpu_size,
        cpu_set_t *cpu_set) {
    thread_port_t mach_thread;
    size_t core = 0;

    for (core = 0; core < 8 * cpu_size; core++) {
        if (CPU_ISSET(core, cpu_set)) break;
    }
    printf("binding to core %ld\n", core);
    thread_affinity_policy_data_t policy = {core};
    mach_thread = pthread_mach_thread_np(thread);
    thread_policy_set(mach_thread, THREAD_AFFINITY_POLICY,
            (thread_policy_t) & policy, 1);
    return 0;
}
#endif

// Set affinity of calling thread to specific core on a multi-core CPU

static int thread_to_core(int core_id) {
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (core_id < 0 || core_id >= num_cores)
        return EINVAL;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();
    return pthread_setaffinity_np(current_thread, sizeof (cpu_set_t), &cpuset);
}

void *pluto_tx_thread_ep(void *arg) {
    NOTUSED(arg);
    char buf[1024];
    struct iio_context *ctx = NULL;
    struct iio_device *tx = NULL;
    struct iio_device *phydev = NULL;
    struct iio_channel *tx0_i = NULL;
    struct iio_channel *tx0_q = NULL;
    struct iio_buffer *tx_buffer = NULL;

    // Try sticking this thread to core 2
    thread_to_core(2);

    // Create IIO context to access ADALM-Pluto
    ctx = iio_create_default_context();
    if (ctx == NULL) {
        if (plutotx.hostname != NULL) {
            ctx = iio_create_network_context(plutotx.hostname);
        } else if (plutotx.uri != NULL) {
            ctx = iio_create_context_from_uri(plutotx.uri);
        } else {
            ctx = iio_create_network_context("pluto.local");
        }
    }

    if (ctx == NULL) {
        iio_strerror(errno, buf, sizeof (buf));
        fprintf(stderr, "Failed creating IIO context: %s\n", buf);
        goto pluto_thread_exit;
    }

    int device_count = iio_context_get_devices_count(ctx);
    if (!device_count) {
        fprintf(stderr, "No supported PLUTOSDR devices found.\n");
        goto pluto_thread_exit;
    }

    tx = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (tx == NULL) {
        iio_strerror(errno, buf, sizeof (buf));
        fprintf(stderr, "Error opening PLUTOSDR TX device: %s\n", buf);
        goto pluto_thread_exit;
    }

    // Additional IQ kernel buffers, default is 4
    iio_device_set_kernel_buffers_count(tx, 12);

    phydev = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", true);
    iio_channel_attr_write(phy_chn, "rf_port_select", plutotx.rfport);
    iio_channel_attr_write_longlong(phy_chn, "rf_bandwidth", plutotx.bw_hz);
    iio_channel_attr_write_longlong(phy_chn, "sampling_frequency", plutotx.fs_hz);
    iio_channel_attr_write_double(phy_chn, "hardwaregain", plutotx.gain_db);

    iio_channel_attr_write_bool(
            iio_device_find_channel(phydev, "altvoltage0", true)
            , "powerdown", true); // Turn OFF RX LO

    iio_channel_attr_write_longlong(
            iio_device_find_channel(phydev, "altvoltage1", true)
            , "frequency", plutotx.lo_hz); // Set TX LO frequency

    tx0_i = iio_device_find_channel(tx, "voltage0", true);
    if (!tx0_i)
        tx0_i = iio_device_find_channel(tx, "altvoltage0", true);

    tx0_q = iio_device_find_channel(tx, "voltage1", true);
    if (!tx0_q)
        tx0_q = iio_device_find_channel(tx, "altvoltage1", true);

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    ad9361_set_bb_rate(iio_context_find_device(ctx, "ad9361-phy"), plutotx.fs_hz);

    tx_buffer = iio_device_create_buffer(tx, NUM_SAMPLES, false);
    if (!tx_buffer) {
        fprintf(stderr, "Could not create TX buffer.\n");
        goto pluto_thread_exit;
    }

    iio_channel_attr_write_bool(
            iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
            , "powerdown", false); // Turn ON TX LO

    int32_t ntx = 0;
    char *ptx_buffer = (char *) iio_buffer_start(tx_buffer);

    while (!plutotx.exit) {
        pthread_mutex_lock(&plutotx.data_mutex);
        memcpy(ptx_buffer, iq_buff, BUFFER_SIZE);
        pthread_cond_signal(&plutotx.data_cond);
        pthread_mutex_unlock(&plutotx.data_mutex);
        // Schedule TX buffer
        ntx = iio_buffer_push(tx_buffer);
        if (ntx < 0) {
            fprintf(stderr, "Error pushing buf %d\n", (int) ntx);
            break;
            ;
        }
    }

pluto_thread_exit:
    if (ctx) {
        iio_channel_attr_write_bool(
                iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
                , "powerdown", true); // Turn OFF TX LO
    }

    if (tx_buffer) {
        iio_buffer_destroy(tx_buffer);
    }
    if (tx0_i) {
        iio_channel_disable(tx0_i);
    }
    if (tx0_q) {
        iio_channel_disable(tx0_q);
    }
    if (ctx) {
        iio_context_destroy(ctx);
    }

    // Wake the main thread (if it's still waiting)
    pthread_mutex_lock(&plutotx.data_mutex);
    plutotx.exit = true; // just in case
    pthread_cond_signal(&plutotx.data_cond);
    pthread_mutex_unlock(&plutotx.data_mutex);
#ifndef _WIN32
    pthread_exit(NULL);
#else
    return NULL;
#endif
}

static size_t fwrite_rinex(void *buffer, size_t size, size_t nmemb, void *stream) {
    struct ftp_file *out = (struct ftp_file *) stream;
    if (out && !out->stream) {
        /* open file for writing */
        out->stream = fopen(out->filename, "wb");
        if (!out->stream)
            return -1; /* failure, can't open file to write */
    }
    return fwrite(buffer, size, nmemb, out->stream);
}

int main(int argc, char *argv[]) {
    int sv;
    int neph, ieph;
    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
    gpstime_t g0;

    double llh[3];

    int i;
    channel_t chan[MAX_CHAN];
    double elvmask = 0.0; // in degree

    int ip, qp;
    int iTable;

    gpstime_t grx;
    double delt;
    int isamp;

    int numd = 0, iumd = 0;
    // Allocate user motion array
    double xyz[USER_MOTION_SIZE][3];   
    
    int staticLocationMode = true;

    bool use_rinex3 = false;
    bool use_ftp = false;
    CURL *curl;
    CURLcode res = CURLE_GOT_NOTHING;
    struct ftp_file ftp = {
        RINEX2_FILE_NAME,
        NULL
    };

    const char* navfile = NULL;
    const char* umfile = NULL;

    int result;
    double gain[MAX_CHAN];
    double path_loss;
    double ant_gain;
    double ant_pat[37];
    int ibs; // boresight angle index

    datetime_t t0, tmin, tmax;
    gpstime_t gmin, gmax;
    double dt;
    int igrx;

    bool verb;
    bool timeoverwrite = false; // Overwirte the TOC and TOE in the RINEX file
    ionoutc_t ionoutc;

    ////////////////////////////////////////////////////////////
    // Read options
    ////////////////////////////////////////////////////////////

    // Default options
    g0.week = -1; // Invalid start time
    verb = false;
    ionoutc.enable = true;

    // Default static location; Tokyo
    llh[0] = 35.681298 / R2D;
    llh[1] = 139.766247 / R2D;
    llh[2] = 10.0;

    plutotx.bw_hz = MHZ(3.0); // 3.0 MHz RF bandwidth
    plutotx.fs_hz = MHZ(2.6); // 2.6 MS/s TX sample rate
    plutotx.lo_hz = GHZ(1.575420); // 1.57542 GHz RF frequency
    plutotx.rfport = "A";
    plutotx.gain_db = -20.0;
    plutotx.hostname = NULL;
    plutotx.uri = NULL;

    pthread_mutex_init(&plutotx.data_mutex, NULL);
    pthread_cond_init(&plutotx.data_cond, NULL);

    // signal handlers:
    signal(SIGINT, handle_sig);
    signal(SIGTERM, handle_sig);
    signal(SIGQUIT, handle_sig);

    /* On a multi-core CPU we run the main thread and reader thread on different cores.
     * Try sticking the main thread to core 1
     */
    thread_to_core(1);

    if (argc < 3) {
        usage();
        exit(1);
    }

    while ((result = getopt(argc, argv, "e:3:u:g:c:l:s:T:t:A:B:U:N:vfi?")) != -1) {
        switch (result) {
            case 'e':
                navfile = optarg;
                break;
            case 'u':
                umfile = optarg;
                staticLocationMode = false;
                break;
            case '3':
                use_rinex3 = true;
                ftp.filename = RINEX3_FILE_NAME;
                break;
            case 'f':
                use_ftp = true;
                break;
            case 'c':
                // Static ECEF coordinates input mode
                sscanf(optarg, "%lf,%lf,%lf", &xyz[0][0], &xyz[0][1], &xyz[0][2]);
                break;
            case 'l':
                // Static geodetic coordinates input mode
                // Added by scateu@gmail.com
                sscanf(optarg, "%lf,%lf,%lf", &llh[0], &llh[1], &llh[2]);
                llh[0] = llh[0] / R2D; // convert to RAD
                llh[1] = llh[1] / R2D; // convert to RAD
                llh2xyz(llh, xyz[0]); // Convert llh to xyz
                break;
            case 's':
                plutotx.fs_hz = (long long) atoi(optarg);
                if (plutotx.fs_hz < MHZ(1.0)) {
                    fprintf(stderr, "ERROR: Invalid sampling frequency.\n");
                    exit(1);
                }
                break;
            case 'T':
                timeoverwrite = true;
                if (strncmp(optarg, "now", 3) == 0) {
                    time_t timer;
                    struct tm *gmt;

                    time(&timer);
                    gmt = gmtime(&timer);

                    t0.y = gmt->tm_year + 1900;
                    t0.m = gmt->tm_mon + 1;
                    t0.d = gmt->tm_mday;
                    t0.hh = gmt->tm_hour;
                    t0.mm = gmt->tm_min;
                    t0.sec = (double) gmt->tm_sec;

                    date2gps(&t0, &g0);

                }
                break;
            case 't':
                sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh, &t0.mm, &t0.sec);
                if (t0.y <= 1980 || t0.m < 1 || t0.m > 12 || t0.d < 1 || t0.d > 31 ||
                        t0.hh < 0 || t0.hh > 23 || t0.mm < 0 || t0.mm > 59 || t0.sec < 0.0 || t0.sec >= 60.0) {
                    fprintf(stderr, "ERROR: Invalid date and time.\n");
                    exit(1);
                }
                t0.sec = floor(t0.sec);
                date2gps(&t0, &g0);
                break;
            case 'i':
                ionoutc.enable = false; // Disable ionospheric correction
                break;
            case 'v':
                verb = true;
                break;
            case 'A':
                plutotx.gain_db = atof(optarg);
                if (plutotx.gain_db > 0.0) plutotx.gain_db = 0.0;
                if (plutotx.gain_db < -80.0) plutotx.gain_db = -80.0;
                break;
            case 'B':
                plutotx.bw_hz = MHZ(atof(optarg));
                if (plutotx.bw_hz > MHZ(5.0)) plutotx.bw_hz = MHZ(5.0);
                if (plutotx.bw_hz < MHZ(1.0)) plutotx.bw_hz = MHZ(1.0);
                break;
            case 'U':
                plutotx.uri = optarg;
                break;
            case 'N':
                plutotx.hostname = optarg;
                break;
            case ':':
            case '?':
                usage();
                exit(1);
            default:
                break;
        }
    }

    if ((navfile == NULL) && (use_ftp == false)) {
        fprintf(stderr, "ERROR: GPS ephemeris file is not specified.\n");
        exit(1);
    }

    delt = 1.0 / plutotx.fs_hz;

    ////////////////////////////////////////////////////////////
    // Receiver position
    ////////////////////////////////////////////////////////////

    if (!staticLocationMode) {
        // Read user motion file
        numd = readUserMotion(xyz, umfile);

        if (numd == -1) {
            fprintf(stderr, "ERROR: Failed to open user motion file.\n");
            exit(1);
        } else if (numd == 0) {
            fprintf(stderr, "ERROR: Failed to read user motion data.\n");
            exit(1);
        }

        fprintf(stderr, "Using user motion mode.\n");
    } else {
        // Static geodetic coordinates input mode: "-l"
        fprintf(stderr, "Using static location mode.\n");
    }

    /*
    fprintf(stderr, "xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1], xyz[0][2]);
    fprintf(stderr, "llh = %11.6f, %11.6f, %11.1f\n", llh[0]*R2D, llh[1]*R2D, llh[2]);
     */
    ////////////////////////////////////////////////////////////
    // Read ephemeris
    ////////////////////////////////////////////////////////////
    if (use_ftp) {
        time_t t = time(NULL);
        struct tm *tm = gmtime(&t);
        char* url = malloc(NAME_MAX);
        const char *station = stations_v2[25].id_v2;
        // We fetch data from previous hour because the actual hour is still in progress
        tm->tm_hour -= 1;
        if (tm->tm_hour < 0) {
            tm->tm_hour = 23;
        }

        if (use_rinex3) {
            station = stations_v3[0].id_v2;
        }

        // Compose FTP URL
        snprintf(url, NAME_MAX, RINEX_FTP_URL RINEX_FTP_FILE, (use_rinex3) ? RINEX3_SUBFOLDER : RINEX2_SUBFOLDER,
                tm->tm_yday + 1, tm->tm_hour, station, tm->tm_yday + 1, 'a' + tm->tm_hour, tm->tm_year - 100);

        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fwrite_rinex);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ftp);
            curl_easy_setopt(curl, CURLOPT_USE_SSL, CURLUSESSL_NONE);
            if (verb) {
                curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
            } else {
                curl_easy_setopt(curl, CURLOPT_VERBOSE, 0L);
            }
            curl_easy_setopt(curl, CURLOPT_USERPWD, "anonymous:anonymous");
            res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);
        }

        if (ftp.stream)
            fclose(ftp.stream);

        free(url);
        curl_global_cleanup();

        if (res != CURLE_OK) {
            fprintf(stderr, "Curl error: %d\n", res);
            exit(1);
        }
    }

    if (use_rinex3) {
        neph = readRinex3(eph, &ionoutc, navfile);
    } else {
        neph = readRinex2(eph, &ionoutc, navfile);
    }

    if (neph == 0) {
        fprintf(stderr, "ERROR: No ephemeris available.\n");
        exit(1);
    }

    if ((verb == true)&&(ionoutc.vflg == true)) {
        fprintf(stderr, "  %12.3e %12.3e %12.3e %12.3e\n",
                ionoutc.alpha0, ionoutc.alpha1, ionoutc.alpha2, ionoutc.alpha3);
        fprintf(stderr, "  %12.3e %12.3e %12.3e %12.3e\n",
                ionoutc.beta0, ionoutc.beta1, ionoutc.beta2, ionoutc.beta3);
        fprintf(stderr, "   %19.11e %19.11e  %9d %9d\n",
                ionoutc.A0, ionoutc.A1, ionoutc.tot, ionoutc.wnt);
        fprintf(stderr, "%6d\n", ionoutc.dtls);
    }

    for (sv = 0; sv < MAX_SAT; sv++) {
        if (eph[0][sv].vflg == true) {
            gmin = eph[0][sv].toc;
            tmin = eph[0][sv].t;
            break;
        }
    }

    gmax.sec = 0;
    gmax.week = 0;
    tmax.sec = 0;
    tmax.mm = 0;
    tmax.hh = 0;
    tmax.d = 0;
    tmax.m = 0;
    tmax.y = 0;
    for (sv = 0; sv < MAX_SAT; sv++) {
        if (eph[neph - 1][sv].vflg == true) {
            gmax = eph[neph - 1][sv].toc;
            tmax = eph[neph - 1][sv].t;
            break;
        }
    }

    if (g0.week >= 0) // Scenario start time has been set.
    {
        if (timeoverwrite == true) {
            gpstime_t gtmp;
            datetime_t ttmp;
            double dsec;

            gtmp.week = g0.week;
            gtmp.sec = (double) (((int) (g0.sec)) / 7200)*7200.0;

            dsec = subGpsTime(gtmp, gmin);

            // Overwrite the UTC reference week number
            ionoutc.wnt = gtmp.week;
            ionoutc.tot = (int) gtmp.sec;

            // Iono/UTC parameters may no longer valid
            //ionoutc.vflg = FALSE;

            // Overwrite the TOC and TOE to the scenario start time
            for (sv = 0; sv < MAX_SAT; sv++) {
                for (i = 0; i < neph; i++) {
                    if (eph[i][sv].vflg == true) {
                        gtmp = incGpsTime(eph[i][sv].toc, dsec);
                        gps2date(&gtmp, &ttmp);
                        eph[i][sv].toc = gtmp;
                        eph[i][sv].t = ttmp;

                        gtmp = incGpsTime(eph[i][sv].toe, dsec);
                        eph[i][sv].toe = gtmp;
                    }
                }
            }
        } else {
            if (subGpsTime(g0, gmin) < 0.0 || subGpsTime(gmax, g0) < 0.0) {
                fprintf(stderr, "ERROR: Invalid start time.\n");
                fprintf(stderr, "tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                        tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec,
                        gmin.week, gmin.sec);
                fprintf(stderr, "tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                        tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec,
                        gmax.week, gmax.sec);
                exit(1);
            }
        }
    } else {
        g0 = gmin;
        t0 = tmin;
    }

    fprintf(stderr, "Gain: %.1fdB\n", plutotx.gain_db);
    fprintf(stderr, "RINEX date = %s\n", rinex_date);
    fprintf(stderr, "Start time = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
            t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);

    // Select the current set of ephemerides
    ieph = -1;

    for (i = 0; i < neph; i++) {
        for (sv = 0; sv < MAX_SAT; sv++) {
            if (eph[i][sv].vflg == true) {
                dt = subGpsTime(g0, eph[i][sv].toc);
                if (dt >= -SECONDS_IN_HOUR && dt < SECONDS_IN_HOUR) {
                    ieph = i;
                    break;
                }
            }
        }

        if (ieph >= 0) // ieph has been set
            break;
    }

    if (ieph == -1) {
        fprintf(stderr, "ERROR: No current set of ephemerides has been found.\n");
        exit(1);
    }

    ////////////////////////////////////////////////////////////
    // Baseband signal buffer and output file
    ////////////////////////////////////////////////////////////

    // Allocate I/Q buffer
    iq_buff = calloc(NUM_SAMPLES, 4);

    if (iq_buff == NULL) {
        fprintf(stderr, "ERROR: Faild to allocate 16-bit I/Q buffer.\n");
        goto exit_main_thread;
    }

    ////////////////////////////////////////////////////////////
    // Start ADALM-Pluto TX thread
    ////////////////////////////////////////////////////////////
    pthread_create(&pluto_thread, NULL, pluto_tx_thread_ep, NULL);

    ////////////////////////////////////////////////////////////
    // Initialize channels
    ////////////////////////////////////////////////////////////

    // Clear all channels
    for (i = 0; i < MAX_CHAN; i++)
        chan[i].prn = 0;

    // Clear satellite allocation flag
    for (sv = 0; sv < MAX_SAT; sv++)
        allocatedSat[sv] = -1;

    // Initial reception time
    grx = incGpsTime(g0, 0.0);

    // Allocate visible satellites
    allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);

    fprintf(stderr, "PRN   Az    El     Range     Iono\n");
    for (i = 0; i < MAX_CHAN; i++) {
        if (chan[i].prn > 0)
            fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
                chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
    }

    ////////////////////////////////////////////////////////////
    // Receiver antenna gain pattern
    ////////////////////////////////////////////////////////////

    for (i = 0; i < 37; i++)
        ant_pat[i] = pow(10.0, -ant_pat_db[i] / 20.0);

    ////////////////////////////////////////////////////////////
    // Generate baseband signals
    ////////////////////////////////////////////////////////////

    // Update receiver time
    grx = incGpsTime(grx, 0.1);

    while (!plutotx.exit) {
        for (i = 0; i < MAX_CHAN; i++) {
            if (chan[i].prn > 0) {
                // Refresh code phase and data bit counters
                range_t rho;
                sv = chan[i].prn - 1;

                // Current pseudorange
                if (!staticLocationMode) {
                    computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[iumd]);
                } else {
                    computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[0]);
                }

                chan[i].azel[0] = rho.azel[0];
                chan[i].azel[1] = rho.azel[1];

                // Update code phase and data bit counters
                computeCodePhase(&chan[i], rho, 0.1);
#ifndef FLOAT_CARR_PHASE
                chan[i].carr_phasestep = (int) round(512.0 * 65536.0 * chan[i].f_carr * delt);
#endif
                // Path loss
                path_loss = 20200000.0 / rho.d;

                // Receiver antenna gain
                ibs = (int) ((90.0 - rho.azel[1] * R2D) / 5.0); // covert elevation to boresight
                ant_gain = ant_pat[ibs];

                // Signal gain
                gain[i] = (double) (path_loss * ant_gain * iq_gain);
            }
        }

        pthread_mutex_lock(&plutotx.data_mutex);
        for (isamp = 0; isamp < NUM_SAMPLES; isamp++) {
            int64_t i_acc = 0;
            int64_t q_acc = 0;

            for (i = 0; i < MAX_CHAN; i++) {
                if (chan[i].prn > 0) {
#ifdef FLOAT_CARR_PHASE
                    iTable = (int) floor(chan[i].carr_phase * 512.0);
#else
                    iTable = (chan[i].carr_phase >> 16) & 0x1ff; // 9-bit index
#endif
                    ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
                    qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];

                    // Accumulate for all visible satellites
                    i_acc += ip;
                    q_acc += qp;

                    // Update code phase
                    chan[i].code_phase += chan[i].f_code * delt;

                    if (chan[i].code_phase >= CA_SEQ_LEN) {
                        chan[i].code_phase -= CA_SEQ_LEN;

                        chan[i].icode++;

                        if (chan[i].icode >= 20) // 20 C/A codes = 1 navigation data bit
                        {
                            chan[i].icode = 0;
                            chan[i].ibit++;

                            if (chan[i].ibit >= 30) // 30 navigation data bits = 1 word
                            {
                                chan[i].ibit = 0;
                                chan[i].iword++;
                                /*
                                if (chan[i].iword>=N_DWRD)
                                        fprintf(stderr, "\nWARNING: Subframe word buffer overflow.\n");
                                 */
                            }

                            // Set new navigation data bit
                            chan[i].dataBit = (int) ((chan[i].dwrd[chan[i].iword]>>(29 - chan[i].ibit)) & 0x1UL)*2 - 1;
                        }
                    }

                    // Set current code chip
                    chan[i].codeCA = chan[i].ca[(int) chan[i].code_phase]*2 - 1;

                    // Update carrier phase
#ifdef FLOAT_CARR_PHASE
                    chan[i].carr_phase += chan[i].f_carr * delt;

                    if (chan[i].carr_phase >= 1.0)
                        chan[i].carr_phase -= 1.0;
                    else if (chan[i].carr_phase < 0.0)
                        chan[i].carr_phase += 1.0;
#else
                    chan[i].carr_phase += chan[i].carr_phasestep;
#endif
                }
            }

            i_acc = (i_acc + iq_offset) >> iq_shift;
            q_acc = (q_acc + iq_offset) >> iq_shift;

            // Store I/Q samples into buffer
            iq_buff[isamp * 2] = (short) i_acc;
            iq_buff[isamp * 2 + 1] = (short) q_acc;
        }
        pthread_cond_signal(&plutotx.data_cond);
        pthread_cond_wait(&plutotx.data_cond, &plutotx.data_mutex);
        pthread_mutex_unlock(&plutotx.data_mutex);

        //
        // Update navigation message and channel allocation every 30 seconds
        //
        igrx = (int) (grx.sec * 10.0 + 0.5);

        if (igrx % 300 == 0) // Every 30 seconds
        {
            // Update navigation message
            for (i = 0; i < MAX_CHAN; i++) {
                if (chan[i].prn > 0)
                    generateNavMsg(grx, &chan[i], 0);
            }

            // Refresh ephemeris and subframes
            // Quick and dirty fix. Need more elegant way.
            for (sv = 0; sv < MAX_SAT; sv++) {
                if (eph[ieph + 1][sv].vflg == true) {
                    dt = subGpsTime(eph[ieph + 1][sv].toc, grx);
                    if (dt < SECONDS_IN_HOUR) {
                        ieph++;

                        for (i = 0; i < MAX_CHAN; i++) {
                            // Generate new subframes if allocated
                            if (chan[i].prn != 0)
                                eph2sbf(eph[ieph][chan[i].prn - 1], ionoutc, chan[i].sbf);
                        }
                    }
                    break;
                }
            }

            // Update channel allocation
            if (!staticLocationMode) {
                allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[iumd], elvmask);
            } else {
                allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);
            }
        }
        // Update receiver time
        grx = incGpsTime(grx, 0.1);
        // update postition index
        iumd++;
        if (iumd >= numd) {
            iumd = 0;
        }
    }

exit_main_thread:
    pthread_mutex_unlock(&plutotx.data_mutex);
    plutotx.exit = true;
    pthread_join(pluto_thread, NULL); /* Wait on Pluto TX thread exit */
    pthread_mutex_destroy(&plutotx.data_mutex);

    // Free I/Q buffers
    if (iq_buff) {
        free(iq_buff);
    }
    return (0);
}
