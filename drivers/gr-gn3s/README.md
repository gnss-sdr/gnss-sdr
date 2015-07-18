**How to build gr-gn3s**

Source maintainer: Javier Arribas (jarribas at cttc.es)

This document describes how to build the GN3S V2 GPS Sampler GNU Radio Source USB 2.0 driver. 

More information on the device (not available anymore) can be found in http://www.sparkfun.com/products/8238

The driver core is based on Gregory W. Hecker driver available in http://github.com/gps-sdr/gps-sdr.

GR-GN3S contains a GNU Radio fully compliant gr-block signal source intended to be used either with GNSS-SDR as a signal source or as 
standalone signal source block instantiated from a GNU Radio flowgraph from C++ or using Python (Also includes a gnuradio-companion interface).

1. **Install GNU Radio:**

You can install GNU Radio through a .deb package *or* by using pybombs. Please choose only **one** procedure.

- In Ubuntu 12.10 and later:

~~~~~~ 
$ sudo apt-get install gnuradio-dev 
~~~~~~



- Semi-automatic installation of GNU Radio:

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


2. **Get the latest version of GNSS-SDR:**

~~~~~~
$ git clone git://github.com/gnss-sdr/gnss-sdr
$ git checkout next
~~~~~~

3. **Build GR-GN3S:**

- Go to GR-GN3S root directory and compile the driver:

~~~~~~
$ cd gnss-sdr/drivers/gr-gn3s
$ cd build
$ cmake ../
$ make
~~~~~~

- If everything went fine, install the driver as root

~~~~~~
$ sudo make install
$ sudo ldconfig
~~~~~~

4. **Check that the module is usable by gnuradio-companion**
 
Open gnuradio-companion and check the gn3s_source module under the GN3S tab. 
In order to gain access to USB ports, gnuradio-companion should be used as root.
In addition, the driver requires access to the GN3S firmware binary file. 
It should be available in the same path where the application is called.
GNSS-SDR comes with a pre-compiled custom GN3S firmware available at gnss-sdr/firmware/GN3S_v2/bin/gn3s_firmware.ihx. 
Please copy this file to the application path.

5. **Build gnss-sdr with the GN3S option enabled:**

~~~~~~
$ cmake -DENABLE_GN3S=ON ../
$ make
$ sudo make install
~~~~~~

