# GNSS-SDR GUI for generating Configuration files for Receivers

This page gives information about GUI developed for GNSS-SDR as part of GSoC 2017. The goal of this project was to develop a friendly Graphical User Interface (GUI) for the generation of configuration files for user defined receivers. The application reads the reference files of all the different implementations available in GNSS-SDR for different blocks and allows user to select implementations as desired by receiver.  All the options for a selected implementation will show up automatically in GUI so there is no need to remember the variable names. Developed GUI allow users to configure supl, signal sources, signal conditioner, acquisition, tacking, telemetry decoder, observable and PVT blocks. This will allow a friendly, and intuitive receiver definition. The GUI is easy to use and populate itself dynamically based on user input. After the user finish giving the inputs the application generates the configuration file that is compatible to be used with GNSS-SDR. Qt, a very popular and widely used graphical toolkit for creating GUI applications, was used to develop GUI.


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Installation from source on Linux (Tested on Ubuntu 16.04.3 LTS).

If you have a fresh installation of Ubuntu follow the following steps

```
$ sudo apt-get update
$ sudo apt-get install qtbase5-dev
$ sudo apt-get install qt5-default
$ qmake gnss_sdr_gui.pro DESTDIR=/path/to/executable/ GUI_FILES_LOCATION="/path/to/gui_ini_files/"
$ make
```

If you want to place the executable in /home/username/ and you have placed "gui_ini_files" folder in /home/username/Documents/. Use following qmake command 

```
$ qmake gnss_sdr_gui.pro DESTDIR=/home/username/ GUI_FILES_LOCATION="/home/username/Documents/"
$ make
```

### Running on Linux with Qt Creator 4.2.1 (Community):

Install Qt Qt Creator 4.2.1 community edition. Available free [here](https://info.qt.io/download-qt-for-application-development)

*	Just open the project (.pro) file in Qt creator.
*	Click configure project.
*	Clean and build.
*	Set DESTDIR and GUI_FILES_LOCATION from Qt creator project configuration.
*	Run

### Running on Windows with Qt Creator 4.2.1 (Community):

Install Qt Qt Creator 4.2.1 community edition. Available free [here](https://info.qt.io/download-qt-for-application-development)

*	Just open the project (.pro) file in Qt creator.
*	Click configure project.
*	Clean and build.
*	Set DESTDIR and GUI_FILES_LOCATION from Qt creator project configuration.
*	Run


## Generating the configuration file.

### Video 

Please click the link below to watch a video showing the working of the developed GUI. The video also shows how to generate the example configuration file for the online tutorial “My first position fix”. The output of GNSS-SDR with the generated configuration file is also sown.

[Video](https://streamable.com/casws)


## Contributing

Please read [CONTRIBUTING.md](https://github.com/gnss-sdr/gnss-sdr/blob/master/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.


## License

Please see following link for license

[License](https://github.com/gnss-sdr/gnss-sdr#about-the-software-license)

## Acknowledgments

* Luis Esteve Elfau 
* Carlos Fernández
ing

