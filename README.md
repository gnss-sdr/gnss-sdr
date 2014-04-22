**Welcome to GNSS-SDR!**

Check [gnss-sdr.org](http://gnss-sdr.org "GNSS-SDR's Homepage") for more information about this open source GNSS software defined receiver.

If you have questions about GNSS-SDR, please [subscribe to the gnss-sdr-developers mailing list](http://lists.sourceforge.net/lists/listinfo/gnss-sdr-developers "Subscribe to the gnss-sdr-developers mailing list" ) and post your questions there.



# How to build GNSS-SDR


This section describes how to set up the compilation environment in GNU/Linux or Mac OS X, and to build GNSS-SDR. See also [our Building Guide](http://gnss-sdr.org/documentation/building-guide "GNSS-SDR's Building Guide").

GNU/Linux 
----------

Tested distributions: Ubuntu 12.04, 12.10, 13.04, 13.10 and 14.04 (32 and 64 bits), Debian 6.0.6 and 7.2, Fedora 18, 19 and 20 and openSUSE 13.1 (newer versions should work, too).

### Install GNU Radio:

- Downloading, building and installing [GNU Radio](http://gnuradio.org/redmine/projects/gnuradio/wiki "GNU Radio's Homepage") and all its dependencies is not a simple task. We recommend to use [PyBOMBS](http://gnuradio.org/redmine/projects/pybombs/wiki) (Python Build Overlay Managed Bundle System), the GNU Radio install management system that automatically does all the work for you. In a terminal, type:
``` 
$ git clone git://github.com/pybombs/pybombs 
$ cd pybombs
```
Configure PyBOMBS:
```
$ ./pybombs config 
```

You can safely accept the default options but for ```prefix```. We recommend to put ```/usr/local``` there. After the configuration, you should get something similar to:
```
gituser = username
prefix = /usr/local
satisfy_order = deb,src  # For Debian/Ubuntu/LinuxMint
satisfy_order = rpm,src  # For Fedora/CentOS/RHEL/openSUSE
forcepkgs =
timeout = 30
cmakebuildtype = RelWithDebInfo
builddocs = OFF
```
Then, you are ready to download and install [UHD](http://files.ettus.com/uhd_docs/manual/html/) (the Universal Hardware Driver), GNU Radio and all their required dependencies by doing:
```
$ sudo ./pybombs install uhd gnuradio
```
This can take some time (up to two hours to complete, depending on your system), and installs the latest versions of the Universal Hardware Driver (UHD) and GNU Radio in your system, including all their dependencies. 
In case you do not want to use PyBOMBS and prefer to build and install GNU Radio manually from source, follow instructions at the [GNU Radio Build Guide](http://gnuradio.org/redmine/projects/gnuradio/wiki/BuildGuide).


### Install other libraries used by GNSS-SDR:

#### Install the [Armadillo](http://arma.sourceforge.net/ "Armdillo's Homepage") C++ linear algebra library:
```
$ sudo apt-get install libblas-dev liblapack-dev gfortran   # For Debian/Ubuntu/LinuxMint
$ sudo yum install lapack-devel blas-devel gcc-fortran      # For Fedora/CentOS/RHEL
$ sudo zypper install lapack-devel blas-devel gcc-fortran   # For OpenSUSE

$ wget http://sourceforge.net/projects/arma/files/armadillo-4.100.2.tar.gz
$ tar xvfz armadillo-4.100.2.tar.gz
$ cd armadillo-4.100.2
$ cmake .
$ make
$ sudo make install
```

The full stop separated from "cmake" by a space is important. CMake will figure out what other libraries are currently installed and will modify Armadillo's configuration correspondingly. CMake will also generate a run-time armadillo library, which is a combined alias for all the relevant libraries present on your system (eg. BLAS, LAPACK and ATLAS).

#### Install [Gflags](http://code.google.com/p/gflags/ "gflags' Homepage"), a commandline flags processing module for C++:
``` 
$ wget http://gflags.googlecode.com/files/gflags-2.0.zip
$ unzip gflags-2.0.zip
$ cd gflags-2.0
$ ./configure
$ make
$ sudo make install
$ sudo ldconfig
``` 
#### Install [glog](http://code.google.com/p/google-glog/ "Glog's Homepage"), a library that implements application-level logging:
``` 
$ wget http://google-glog.googlecode.com/files/glog-0.3.3.tar.gz 
$ tar xvfz glog-0.3.3.tar.gz 
$ cd glog-0.3.3
$ ./configure
$ make
$ sudo make install
$ sudo ldconfig
``` 

#### Build the [Google C++ Testing Framework](http://code.google.com/p/googletest/ "Googletest Homepage"),  (also known as googletest):
``` 
$ wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
$ unzip gtest-1.7.0.zip
$ cd gtest-1.7.0
$ ./configure
$ make
``` 

Please DO NOT install gtest (do not do "sudo make install"). Every user needs to compile his tests using the same compiler flags used to compile the installed Google Test libraries; otherwise he may run into undefined behaviors (i.e. the tests can behave strangely and may even crash for no obvious reasons). The reason is that C++ has this thing called the One-Definition Rule: if two C++ source files contain different definitions of the same class/function/variable, and you link them together, you violate the rule. The linker may or may not catch the error (in many cases it is not required by the C++ standard to catch the violation). If it does not, you get strange run-time behaviors that are unexpected and hard to debug. If you compile Google Test and your test code using different compiler flags, they may see different definitions of the same class/function/variable (e.g. due to the use of ```#if``` in Google Test). Therefore, for your sanity, we recommend to avoid installing pre-compiled Google Test libraries. Instead, each project should compile Google Test itself such that it can be sure that the same flags are used for both Google Test and the tests. The building system of GNSS-SDR does the compilation and linking of gtest its own tests; it is only required that you tell the system where the gtest folder that you downloaded resides. Just add to your ```$HOME/.bashrc``` file the following line:
``` 
$ export GTEST_DIR=/home/username/gtest-1.7.0
``` 
changing /home/username/gtest-1.7.0 by the actual directory where you downloaded gtest. Again, it is recommended to add this line to your ```$HOME/.bashrc``` file.

#### Install the [SSL development libraries](https://www.openssl.org/ "OpenSSL's Homepage"):
``` 
$ sudo apt-get install libssl-dev    # For Debian/Ubuntu/LinuxMint
$ sudo yum install openssl-devel     # For Fedora/CentOS/RHEL
``` 
### Clone GNSS-SDR's Git repository:
``` 
$ git clone git://git.code.sf.net/p/gnss-sdr/cttc gnss-sdr
``` 
Cloning the GNSS-SDR repository as in the line above will create a folder named gnss-sdr with the following structure:
``` 
 |-gnss-sdr
 |---build      <- where gnss-sdr is built
 |---cmake      <- CMake-related files
 |---conf       <- Configuration files. Each file represents one receiver.
 |---data       <- Populate this folder with your captured data.
 |---docs       <- Contains documentation-related files
 |---drivers    <- Drivers for some RF front-ends
 |---firmware   <- Firmware for some front-ends
 |---install    <- Executables 
 |---src        <- Source code folder
 |-----algorithms
 |-----core
 |-----main
 |-----tests
 |-----utils     <- some utilities (e.g. Matlab scripts)
``` 


###### Build GN3S V2 Custom firmware and driver (OPTIONAL):

- Go to GR-GN3S root directory, compile and install the driver
  (read the drivers/gr-gn3s/README for more information):
```   
$ cd gnss-sdr/drivers/gr-gn3s
$ cd build
$ cmake ../
$ make
$ sudo make install
$ sudo ldconfig
``` 
- Set the environment variable ```GN3S_DRIVER=1``` in order to enable the GN3S_Signal_Source in GNSS-SDR:
``` 
$ export GN3S_DRIVER=1
``` 
In order to gain access to USB ports, gnss-sdr should be used as root.
In addition, the driver requires access to the GN3S firmware binary file. 
It should be available in the same path where the application is called.
GNSS-SDR comes with a pre-compiled custom GN3S firmware available at gnss-sdr/firmware/GN3S_v2/bin/gn3s_firmware.ihx. 
Please copy this file to the application path. The GNSS-SDR default path is gnss-sdr/install

(in order to disable the GN3S_Signal_Source compilation, you should remove the GN3S_DRIVER variable and build again GNSS-SDR).

###### Build RTL-SDR support (OPTIONAL)

- Install the [OsmoSDR](http://sdr.osmocom.org/trac/ "OsmoSDR's Homepage") library and GNU Radio's source block: 
``` 
$ git clone git://git.osmocom.org/osmo-sdr.git
$ cd osmo-sdr/software/libosmosdr
$ mkdir build
$ cd build/
$ cmake ../
$ make
$ sudo make install
$ sudo ldconfig
$ cd ../../
$ git clone git://git.osmocom.org/gr-osmosdr
$ cd gr-osmosdr
$ mkdir build
$ cd build
$ cmake ../ -Wno-dev
$ make
$ sudo make install
$ sudo ldconfig
``` 
- Set the environment variable ```RTLSDR_DRIVER=1``` in order to enable the Rtlsdr_Signal_Source in GNSS-SDR:
``` 
$ export RTLSDR_DRIVER=1
``` 
- In order to compile the RTLSDR adapter you should also provide the path to the gr-osmosdr source code using:
``` 
$ export OSMOSDR_ROOT=/path/to/gr-osmosdr
``` 
The default will be ```OSMOSDR_ROOT=/usr/local```

(in order to disable the Rtlsdr_Signal_Source compilation, you should remove the RTLSDR_DRIVER variable and build again GNSS-SDR).

### Build GNSS-SDR

- Go to GNSS-SDR's build directory:
``` 
$ cd gnss-sdr/build
``` 
- Configure and build the application:
``` 
$ cmake ../
$ make
``` 
By default, cmake is configured to build the release version. If you want to build the debug version, please use:
``` 
cmake ../ -DCMAKE_BUILD_TYPE=Debug
``` 
- Move the executables to the install folder:
``` 
$ make install
``` 
- If everything goes well, two new executables will be created at gnss-sdr/install, namely ```gnss-sdr``` and ```run_tests```. 

- You can create the documentation by doing:
``` 
$ make doc
``` 
from the gnss-sdr/build folder. This will generate HTML documentation that can be retrieved pointing your browser of preference to gnss-sdr/docs/html/index.html.
If a LaTeX installation is detected in your system,
``` 
$ make pdfmanual
``` 
will create a PDF manual at gnss-sdr/docs/GNSS-SDR_manual.pdf. Please note that the PDF generation requires some fonts to be installed on the host system. In Ubuntu 12.10, those fonts do not come by default. You can install them by doing:
``` 
$ sudo apt-get install texlive-fonts-recommended
``` 
and then run ```cmake ../``` and make pdfmanual again. Finally,
``` 
$ make doc-clean
``` 
will remove the content of previously-generated documentation.

- By default, CMake will build the Release version, meaning that the compiler will generate a faster, optimized executable. This is the recommended build type when using a RF front-end and you need to attain real time. Ifyou are working with a file (and thus without real-time constraints), you may want to obtain more information about the internals of the receiver, as well as more fine-grained logging. This can be done by building the Debug version, by doing:
``` 
$ cd gnss-sdr/build
$ cmake -DCMAKE_BUILD_TYPE=Debug ../
$ make
$ make install
``` 
- If you are using Eclipse as your development environment, CMake can create the project for you. Type:
``` 
$ cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DECLIPSE_CDT4_GENERATE_SOURCE_PROJECT=TRUE -DCMAKE_ECLIPSE_VERSION=3.7 -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 ../
``` 
and then import the created project file into Eclipse:

1. Import project using Menu File -> Import.
2. Select General -> Existing projects into workspace.
3. Browse where your build tree is and select the root build tree directory. Keep "Copy projects into workspace" unchecked.
4. You get a fully functional Eclipse project.


Mac OS X 
---------

Tested versions: 10.8 (Mountain Lion) and 10.9 (Mavericks).



### MAC OS X 10.9 Mavericks

If you still have not installed [Xcode](http://developer.apple.com/xcode/), do it now from the App Store (it's free).

Then, [install Macports](http://www.macports.org/install.php). If you are upgrading from a previous installation, please follow the [migration rules](http://trac.macports.org/wiki/Migration).

In a terminal, type:
``` 
$ sudo port selfupdate
$ sudo port upgrade outdated
$ sudo port install doxygen +latex
$ sudo port install uhd gnuradio
$ sudo port install armadillo
``` 
Install GFlags manually from the trunk:
``` 
$ svn checkout http://gflags.googlecode.com/svn/trunk gflags-trunk
$ cd gflags-trunk
$ CXXFLAGS="-stdlib=libc++" CC=clang CXX=clang++ ./configure  
$ make
$ sudo make install
``` 
Install Glog manually from the subversion repository. Revision 142 is known to work well:
``` 
$ svn checkout -r142 http://google-glog.googlecode.com/svn/trunk/ google-glog
$ cd google-glog
$ CXXFLAGS="-stdlib=libc++" CC=clang CXX=clang++ ./configure 
$ make
$ sudo make install
``` 
Finally, you are ready to checkout the GNSS-SDR repository and build the software:
``` 
$ git clone git://git.code.sf.net/p/gnss-sdr/cttc gnss-sdr
$ cd gnss-sdr/build
$ cmake ../ -DCMAKE_CXX_COMPILER=/usr/bin/clang++
$ make
$ make install
``` 

This will create two executables at gnss-sdr/install, namely ```gnss-sdr``` and ```run_tests```. The documentation can be built by:
``` 
$ make doc
``` 
and can be viewed doing:
``` 
$ open ../docs/html/index.html
``` 


### MAC OS X 10.8 Mountain Lion 


If you still have not installed [Xcode](http://developer.apple.com/xcode/), do it now from the App Store (it's free). Once installed, download and install the command line tools:

Xcode -> Preferences -> Downloads -> Components -> Command Line Tools

Then, install Macports via [Mac OS X Package (.pkg) installer](http://www.macports.org/install.php).

Once MacPorts is properly installed on your system, open a terminal and type:
``` 
$ sudo port selfupdate
$ sudo port install gcc48
$ sudo port select --set gcc mp-gcc48
``` 
Install [X11 via XQuartz-2.7.4.dmg](http://xquartz.macosforge.org/landing/) (needed by gnuradio-companion but not by GNSS-SDR).

Install GNU Radio:
``` 
$ sudo port install gnuradio
``` 
Install other dependencies:
``` 
$ sudo port install armadillo
``` 
The libraries gflags and glog should be installed manually, and in that particular order (same steps as above). If they are not already installed
when building GNSS-SDR, cmake will download, build and link them statically but they will not remain installed in the system.

Finally, you are ready to checkout the GNSS-SDR repository and build the software:
``` 
$ git clone git://git.code.sf.net/p/gnss-sdr/cttc gnss-sdr
$ cd gnss-sdr/build
$ cmake ../ -DCMAKE_CXX_COMPILER=g++
$ make
$ make install
``` 
This will create two executables at gnss-sdr/install, namely ```gnss-sdr``` and ```run_tests```. The documentation can be built by:
``` 
$ make doc
``` 
and can be viewed doing:
``` 
$ open ../docs/html/index.html
``` 


Updating GNSS-SDR
-----------------

If you cloned GNSS-SDR some days ago, it is possible that some developer has updated files at the Git repository. You can update your working copy by doing:
``` 
$ git checkout master      # Switch to branch you want to update
$ git pull origin master   # Download the newest code from our repository
``` 
or, if you want to test the lastest developments:
``` 
$ git checkout next 
$ git pull origin next 
``` 
Before rebuilding the source code, it is safe (and recommended) to remove the remainders of old compilations:
``` 
$ rm -rf gnss-sdr/build/*
``` 




Getting started
---------------


1. After building the code, you will find the ```gnss-sdr``` executable file at gnss-sdr/install.
2. In post-processing mode, you have to provide a captured GNSS signal file.
  1. The signal file can be easily recorded using the GNU Radio file sink in ```gr_complex<float>``` mode.
  2. You will need a GPS active antenna, a [USRP](http://www.ettus.com/product) and a suitable USRP daughter board to receive GPS L1 C/A signals. GNSS-SDR require to have at least 2 MHz of bandwidth in 1.57542 GHz. (remember to enable the DC bias with the daughter board jumper).
We use a [DBSRX2](https://www.ettus.com/product/details/DBSRX2) to do the task, but you can try the newer Ettus' daughter boards as well. 
  3. The easiest way to capture a signal file is to use the GNU Radio Companion GUI. Only two blocks are needed: an USRP signal source connected to complex float file sink. You need to tune the USRP central frequency and decimation factor using USRP signal source properties box. We suggest using a decimation factor of 20 if you use the USRP2. This will give you 100/20 = 5 MSPS which will be enough to receive GPS L1 C/A signals. The front-end gain should also be configured. In our test with the DBSRX2 we obtained good results with ```G=50```.
  4. Capture at least 80 seconds of signal in open sky conditions. During the process, be aware of USRP driver buffer underuns messages. If your hard disk is not fast enough to write data at this speed you can capture to a virtual RAM drive. 80 seconds of signal at 5 MSPS occupies less than 3 Gbytes using ```gr_complex<float>```.
  5. If you have no access to a RF front-end, you can download a sample raw data file (that contains GPS and Galileo signals) from [here](http://sourceforge.net/projects/gnss-sdr/files/data/).
3. You are ready to configure the receiver to use your captured file among other parameters:
  1. The default configuration file resides at [./conf/gnss-sdr.conf](./conf/gnss-sdr.conf).
  2. You need to modify at least the following settings:
    * ```SignalSource.filename=``` (absolute or relative route to your GNSS signal captured file)
    * ```GNSS-SDR.internal_fs_hz=``` (captured file sampling rate in Hz)
    * ```SignalSource.sampling_frequency=``` (captured file sampling rate in Hz)
    * ```SignalConditioner.sample_freq_in=``` (captured file sampling rate in Hz)
    * ```SignalConditioner.sample_freq_out=``` (captured file sampling rate in Hz)
    * ```TelemetryDecoder.fs_in=``` (captured file sampling rate in Hz)
  3. The configuration file has in-line documentation, you can try to tune the number of channels and several receiver parameters.
4. Run the receiver from the install directory. The program reports the current status in text mode, directly to the terminal window. If all goes well, and GNSS-SDR is able to successfully track an decode at least 4 satellites, you will get PVT fixes. The program will write a .kml file and RINEX (yet experimental) files in the install directory. In addition to the console output, GNSS-SDR also writes log files at /tmp/ (configurable with the commandline flag ```./gnss-sdr --log_dir=/path/to/log```).

