# How to build gr-gn3s

Source maintainer: Javier Arribas (jarribas at cttc.es)

This document describes how to build the GN3S V2 GPS Sampler GNU Radio Source USB 2.0 driver. 

More information on the device (not available anymore) can be found at http://www.sparkfun.com/products/8238

The driver core is based on Gregory W. Hecker driver available at http://github.com/gps-sdr/gps-sdr.

GR-GN3S is a GNU Radio's compliant signal source block intended to be used either with GNSS-SDR as a signal source, or as standalone signal source block instantiated from a GNU Radio flow graph from C++ or using Python (also includes a gnuradio-companion interface).

## Install GNU Radio:

You can install GNU Radio through a .deb package *or* by using pybombs. Please choose only **one** of these two procedures.

- In Ubuntu 12.10 and later, or Debian Jessie or later, install GNU Radio and its dependencies through a .deb package:

~~~~~~ 
$ sudo apt-get install gnuradio-dev 
~~~~~~


- Semi-automatic installation of GNU Radio using PyBOMBS:

Downloading, building and installing [GNU Radio](http://gnuradio.org/redmine/projects/gnuradio/wiki "GNU Radio's Homepage") and all its dependencies is not a simple task. We recommend to use [PyBOMBS](http://gnuradio.org/redmine/projects/pybombs/wiki "Python Build Overlay Managed Bundle System wiki") (Python Build Overlay Managed Bundle System), the GNU Radio install management system that automatically does all the work for you. In a terminal, type:

~~~~~~ 
$ git clone git://github.com/pybombs/pybombs 
$ cd pybombs
~~~~~~

Configure PyBOMBS:

~~~~~~
$ ./pybombs config 
~~~~~~

You can safely accept the default options but for ```prefix```. We recommend to put ```/usr/local``` there. After the configuration, you should get something similar to:

~~~~~~
gituser = username
prefix = /usr/local
satisfy_order = deb,src  # For Debian/Ubuntu/LinuxMint
satisfy_order = rpm,src  # For Fedora/CentOS/RHEL/openSUSE
forcepkgs =
forcebuild = gnuradio,uhd,gr-osmosdr,rtl-sdr,...
timeout = 30
cmakebuildtype = RelWithDebInfo
builddocs = OFF
cc = gcc
cxx = g++
makewidth = 4
~~~~~~

Then, you are ready to download and install GNU Radio and all their required dependencies by doing:

~~~~~~
$ sudo ./pybombs install gnuradio
~~~~~~

This can take some time (up to two hours to complete, depending on your system), and downloads, builds and installs the latest versions of the Universal Hardware Driver (UHD) and GNU Radio in your system, including all their dependencies. 
In case you do not want to use PyBOMBS and prefer to build and install GNU Radio step by step, follow instructions at the [GNU Radio Build Guide](http://gnuradio.org/redmine/projects/gnuradio/wiki/BuildGuide).


## Get the latest version of GNSS-SDR:

~~~~~~
$ git clone git://github.com/gnss-sdr/gnss-sdr
$ cd gnss-sdr
$ git checkout next
~~~~~~

## Build GR-GN3S:

- Go to GR-GN3S root directory and compile the driver:

~~~~~~
$ cd drivers/gr-gn3s
$ cd build
$ cmake ../
$ make
~~~~~~

NOTE: If you have installed GNU Radio via the gnuradio-dev package, you might need to use ```cmake -DCMAKE_INSTALL_PREFIX=/usr ../``` instead of ```cmake ../``` in order to make the module visible from gnuradio-companion once installed.


- If everything went fine, install the driver as root

~~~~~~
$ sudo make install
$ sudo ldconfig
~~~~~~

## Check that the module is usable by gnuradio-companion
 
Open gnuradio-companion and check the gn3s_source module under the GN3S tab. In order to gain access to USB ports, gnuradio-companion should be used as root. In addition, the driver requires access to the GN3S firmware binary file. It should be available in the same path where the application is called. GNSS-SDR comes with a pre-compiled custom GN3S firmware available at gnss-sdr/firmware/GN3S_v2/bin/gn3s_firmware.ihx. Please copy this file to the application path.

## Build gnss-sdr with the GN3S option enabled:

~~~~~~
$ cd gnss-sdr/build
$ cmake -DENABLE_GN3S=ON ../
$ make
$ sudo make install
~~~~~~

This will enable the *GN3S_Signal_Source* implementation, which is able to read from the GN3S V2 GPS Sampler in real-time. 


# Using the GN3S V2 GPS Sampler as a signal source with GNSS-SDR

GN3S V2's sampling frequency is 8.1838 Msps, delivering a signal with an intermediate frequency of 38400 Hz. This is an example of a gnss-sdr configuration file for a GPS L1 C/A receiver using the *GN3S_Signal_Source* implementation:

~~~~~~
GNSS-SDR.internal_fs_hz=2727933.33  ; 8183800/3 

;######### SIGNAL_SOURCE CONFIG ############
SignalSource.implementation=GN3S_Signal_Source
SignalSource.item_type=gr_complex
SignalSource.sampling_frequency=8183800
SignalSource.dump=false
SignalSource.dump_filename=../signal_source.dat

;######### SIGNAL_CONDITIONER CONFIG ############
SignalConditioner.implementation=Signal_Conditioner

;######### DATA_TYPE_ADAPTER CONFIG ############
DataTypeAdapter.implementation=Pass_Through

;######### INPUT_FILTER CONFIG ############
InputFilter.implementation=Freq_Xlating_Fir_Filter
InputFilter.dump=false
InputFilter.dump_filename=../data/input_filter.dat
InputFilter.input_item_type=gr_complex
InputFilter.output_item_type=gr_complex
InputFilter.taps_item_type=float
InputFilter.number_of_taps=5
InputFilter.number_of_bands=2
InputFilter.band1_begin=0.0
InputFilter.band1_end=0.45
InputFilter.band2_begin=0.55
InputFilter.band2_end=1.0
InputFilter.ampl1_begin=1.0
InputFilter.ampl1_end=1.0
InputFilter.ampl2_begin=0.0
InputFilter.ampl2_end=0.0
InputFilter.band1_error=1.0
InputFilter.band2_error=1.0
InputFilter.filter_type=bandpass
InputFilter.grid_density=16
InputFilter.sampling_frequency=8183800
InputFilter.IF=38400
InputFilter.decimation_factor=3

;######### RESAMPLER CONFIG ############
Resampler.implementation=Pass_Through
Resampler.dump=false
Resampler.dump_filename=../data/resampler.dat


;######### CHANNELS GLOBAL CONFIG ############
Channels_1C.count=8
Channels.in_acquisition=1
Channel.signal=1C

;######### ACQUISITION GLOBAL CONFIG ############
Acquisition_1C.dump=false
Acquisition_1C.dump_filename=./acq_dump.dat
Acquisition_1C.item_type=gr_complex
Acquisition_1C.if=0
Acquisition_1C.sampled_ms=1
Acquisition_1C.implementation=GPS_L1_CA_PCPS_Acquisition
Acquisition_1C.threshold=0.008
Acquisition_1C.doppler_max=10000
Acquisition_1C.doppler_step=500

;######### TRACKING GLOBAL CONFIG ############
Tracking_1C.implementation=GPS_L1_CA_DLL_PLL_Tracking
Tracking_1C.item_type=gr_complex
Tracking_1C.if=0 
Tracking_1C.dump=false
Tracking_1C.dump_filename=../data/epl_tracking_ch_
Tracking_1C.pll_bw_hz=45.0;
Tracking_1C.dll_bw_hz=2.0;
Tracking_1C.order=3;

;######### TELEMETRY DECODER GPS CONFIG ############
TelemetryDecoder_1C.implementation=GPS_L1_CA_Telemetry_Decoder
TelemetryDecoder_1C.dump=false
TelemetryDecoder_1C.decimation_factor=1;

;######### OBSERVABLES CONFIG ############
Observables.implementation=GPS_L1_CA_Observables
Observables.dump=false.
Observables.dump_filename=./observables.dat

;######### PVT CONFIG ############
PVT.implementation=GPS_L1_CA_PVT
PVT.averaging_depth=100
PVT.flag_averaging=false
PVT.output_rate_ms=10
PVT.display_rate_ms=500
PVT.dump_filename=./PVT
PVT.nmea_dump_filename=./gnss_sdr_pvt.nmea
PVT.flag_nmea_tty_port=false;
PVT.nmea_dump_devname=/dev/pts/4
PVT.dump=false

;######### OUTPUT_FILTER CONFIG ############
OutputFilter.implementation=Null_Sink_Output_Filter
OutputFilter.filename=data/gnss-sdr.dat
OutputFilter.item_type=gr_complex
~~~~~~

Save this configuration in a file, for instance ```my_GN3S_receiver.conf```,  copy the file located at ```install/gn3s_firmware.ihx``` to your working directory, and instantiate gnss-sdr by doing:

~~~~~~
$ gnss-sdr --config_file=./my_GN3S_receiver.conf
~~~~~~





