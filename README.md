# PLUTO-GPS-SIM

PLUTO-GPS-SIM generates a GPS baseband signal IQ data stream, which is then transmitted by the 
software-defined radio (SDR) platform [ADALM-Pluto](https://wiki.analog.com/university/tools/pluto).

### Generating the GPS signal file

A user-defined trajectory can be specified in either a CSV file, which contains 
the Earth-centered Earth-fixed (ECEF) user positions, or an NMEA GGA stream.
The sampling rate of the user motion has to be 10Hz.
The user is also able to assign a static location directly through the command line.

The user specifies the GPS satellite constellation through a GPS broadcast 
ephemeris file. The daily GPS broadcast ephemeris file (brdc) is a merge of the
individual site navigation files into one. The archive for the daily file is:

[ftp://cddis.gsfc.nasa.gov/gnss/data/daily/](ftp://cddis.gsfc.nasa.gov/gnss/data/daily/)

These files are then used to generate the simulated pseudorange and
Doppler for the GPS satellites in view. This simulated range data is 
then used to generate the digitized I/Q samples for the GPS signal
which are then feed into ADALM-Pluto SDR to transmit the GPS L1 frequency.

The simulation start time can be specified if the corresponding set of ephemerides
is available. Otherwise the first time of ephemeris in the RINEX navigation file
is selected.

### Build instructions
#### Dependencies

You will need the latest build and install of libad9361-dev and libiio-dev. The Debian packages 
libad9361-dev that is available up to Debian 9 (stretch) is outdated and missing a required function.
So you have to build packages from source:
```
$ git clone https://github.com/analogdevicesinc/libad9361-iio.git
$ cd libad9361-iio
$ cmake ./
$ make
$ sudo make install
```

```
$ git clone https://github.com/analogdevicesinc/libiio.git
$ cd libiio
$ cmake ./
$ make
$ sudo make install
```

#### Building under Linux with GCC
```
$ git clone https://github.com/mictronics/pluto-gps-sim
$ cd pluto-gps-sim
$ make all
```

### Usage
```
pluto-gps-sim [options]
Options:
  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)
  -u <user_motion> User motion file (dynamic mode)
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -c <location>    ECEF X,Y,Z in meters (static mode) e.g. 3967283.15,1022538.18,4872414.48
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 30.286502,120.032669,100
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -T <date,time>   Overwrite TOC and TOE to scenario start time (use ```now``` for actual time)
  -s <frequency>   Sampling frequency [Hz] (default: 2600000)
  -i               Disable ionospheric delay for spacecraft scenario
  -v               Show details about simulated channels
  -A <attenuation> Set TX attenuation [dB] (default -20.0)
  -B <bw>          Set RF bandwidth [MHz] (default 5.0)
  -U <uri>         ADALM-Pluto URI (eg. usb:1.2.5)
  -N <network>     ADALM-Pluto network IP or hostname (default pluto.local)
```

The user motion can be specified in either dynamic or static mode:

```
> pluto-gps-sim -e brdc3540.14n -u circle.csv
```

```
> pluto-gps-sim -e brdc3540.14n -g triumphv3.txt
```

```
> pluto-gps-sim -e brdc3540.14n -l 30.286502,120.032669,100
```

Set TX attenuation:
```
> pluto-gps-sim -e brdc3540.14n -A -30.0
```
Default -20.0dB. Applicable range 0.0dB to -80.0dB in 0.25dB steps.

Set RF bandwidth:
```
> pluto-gps-sim -e brdc3540.14n -B 3.0
```
Default 3.0MHz. Applicable range 1.0MHz to 5.0MHz

### License

Copyright &copy; 2018 Mictronics
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
