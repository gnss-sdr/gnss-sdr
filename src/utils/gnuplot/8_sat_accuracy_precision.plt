#to load the file digit from terminal:
#> gnuplot 8_sat_IFEN_accuracy_precision.plt

#set terminal pdf color font "Bold,14"
#set output "IFEN_solutions_pdf"
set terminal jpeg font "Helvetica, 14"
set output "8_sat_accuracy_precision.jpeg"

set grid 
set xrange [-15:15]
set yrange [-10:20]
set ylabel "North [m]" 
set xlabel "East [m]"

set key Left left
set title "Accuracy-Precision (with respect to CORRECT coordinates)- 2DRMS"
file1="4_GPS_3_GAL_GNSS_SDR_solutions.txt"
file2="8_GAL_GNSS_SDR_solutions.txt"
file3="8_GPS_GNSS_SDR_solutions.txt"

#values to copy from statistic file
DRMS_1=2*3.077 	 #it is 2*DRMS  combined
DRMS_2=2*1.87	 # gal
DRMS_3=2*2.034   # gps

#difference with respect to the reference position

#values to copy from statistic file
delta_E_1=-1.812 #combined
delta_N_1= 3.596 #combined

delta_E_2= 1.191 #gal
delta_N_2= 1.923 #gal

delta_E_3= -0.560 #gps
delta_N_3= 1.323  #gps

set parametric
#dummy variable is t for curves, u/v for surfaces
set size square
set angle degree
set trange [0:360]

plot file1 u 9:10 with points pointsize 0.3 lc rgb "green" title "4 GPS-3 GAL",\
file3 u 9:10 with points pointsize 0.3 lc rgb "red" title "8 GPS",\
file2 u 9:10 with points pointsize 0.3 lc rgb "blue" title "8 GAL",\
DRMS_1*sin(t)+delta_E_1,DRMS_1*cos(t)+delta_N_1 lw 2 lc rgb "green" notitle,\
DRMS_3*sin(t)+delta_E_3,DRMS_3*cos(t)+delta_N_3 lw 2 lc rgb "red" notitle,\
DRMS_2*sin(t)+delta_E_2,DRMS_2*cos(t)+delta_N_2 lw 2 lc rgb "blue" notitle

