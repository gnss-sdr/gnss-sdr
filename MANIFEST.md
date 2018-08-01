title: gnss-sdr
brief: An open source global navigation satellite systems software defined receiver
tags:
  - sdr
  - gnss
  - gps
  - Galileo
  - Glonass
author:
  - Carles Fernandez-Prades <carles.fernandez@cttc.es>
  - Javier Arribas <javier.arribas@cttc.es>
  - et altri (see AUTHORS file for a list of contributors)
copyright_owner:
  - The Authors
dependencies: gnuradio (>= 3.7.3), armadillo, gflags, glog, gnutls, matio
license: GPLv3+
repo: https://github.com/gnss-sdr/gnss-sdr
website: https://gnss-sdr.org
icon: https://raw.githubusercontent.com/gnss-sdr/gnss-sdr/master/docs/doxygen/images/gnss-sdr_logo.png
---
Global Navigation Satellite Systems receiver defined by software. It performs all the signal
processing from raw signal samples up to the computation of the Position-Velocity-Time solution,
including code and phase observables. It is able to work with raw data files or, if there is
computational power enough, in real time with suitable radiofrequency front-ends. This software
is mainly developed at [CTTC](http://www.cttc.es "Centre Tecnologic de Telecomunicacions de Catalunya")
with contributions from around the world. More info at [gnss-sdr.org](https://gnss-sdr.org "GNSS-SDR's Homepage").
