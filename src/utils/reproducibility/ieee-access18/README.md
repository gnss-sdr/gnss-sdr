Continuous Reproducibility in GNSS Signal Processing
----------------------------------------------------


This folder contains files required for the reproduction of the experiment proposed in:

C. Fern&aacute;ndez-Prades, J. Vil&agrave;-Valls, J. Arribas and A. Ramos, [*Continuous Reproducibility in GNSS Signal Processing*](https://ieeexplore.ieee.org/document/8331069/), IEEE Access, Vol. 6, No. 1, pp. 20451-20463, April 2018. DOI: [10.1109/ACCESS.2018.2822835](https://doi.org/10.1109/ACCESS.2018.2822835)

The data set used in this paper is available at https://zenodo.org/record/1184601

The sample format is `ibyte`: Interleaved (I&Q) stream of samples of type signed integer, 8-bit two’s complement number ranging from -128 to 127. The sampling rate is 3 MSps.

The figure appearing in that paper can be automatically generated with the pipeline available at https://gitlab.com/gnss-sdr/gnss-sdr/pipelines

After the **Build** stage, which compiles the source code in several versions of the most popular GNU/Linux distributions, and the **Test** stage, which executes GNSS-SDR’s QA code, the **Deploy** stage creates and publishes an image of a software container ready to execute the experiment. This container is available by doing:

```
$ docker pull carlesfernandez/docker-gnsssdr:access18
```

Then, in the **Experiment** stage, a job installs the image created in the previous step, grabs the data file, executes the experiment and produces a figure with the obtained results.

The steps to reproduce the experiment in your own machine (with [Docker](https://www.docker.com) already installed and running) are:

```
$ docker pull carlesfernandez/docker-gnsssdr:access18
$ docker run -it -v $PWD/access18:/home/access18 carlesfernandez/docker-gnsssdr:access18
$ git clone https://github.com/gnss-sdr/gnss-sdr
$ cd gnss-sdr
$ git checkout next
$ mkdir -p exp-access18/data
$ cd exp-access18/data
$ curl https://zenodo.org/record/1184601/files/L2_signal_samples.tar.xz --output L2_signal_samples.tar.xz
$ tar xvfJ L2_signal_samples.tar.xz
$ echo "3a04c1eeb970776bb77f5e3b7eaff2df  L2_signal_samples.tar.xz" > data.md5
$ md5sum -c data.md5
$ cd ..
$ cp ../src/utils/reproducibility/ieee-access18/L2-access18.conf .
$ cp ../src/utils/reproducibility/ieee-access18/plot_dump.m .
$ cp -r ../src/utils/matlab/libs/geoFunctions .
$ gnss-sdr --c=L2-access18.conf
$ octave --no-gui plot_dump.m
$ epspdf Figure2.eps Figure2.pdf
$ cp Figure2.pdf /home/access18/
$ exit
```

You will find the file `Figure2.pdf` in a newly created folder called `access18`.
