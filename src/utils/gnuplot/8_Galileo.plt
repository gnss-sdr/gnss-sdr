#set terminal pdf color font "Bold,14"
#set output "IFEN_accuracy.pdf"

set terminal jpeg font "Helvetica, 14"
set output "8_GALILEO_accuracy_precision.jpeg"

set grid 
set xrange [-8:8]
set yrange [-8:8]
set ylabel "North [m]" 
set xlabel "East [m]"

set key Left left
set title "IFEN simulated data, 8 Galileo - Accuracy and Precision"
#file1="8_GPS_GNSS_SDR_solutions.txt"
file2="8_GAL_GNSS_SDR_solutions.txt"
#file3="8_GPS_GNSS_SDR_solutions.txt"

#values to copy from statistic file
DRMS= 1.870121081
DUE_DRMS= 3.740242162
CEP= 1.556390643

#difference with respect to the reference position
#values to copy from statistic file
delta_E=1.191 #galileo
delta_N=1.923 #galileo

set parametric
#dummy variable is t for curves, u/v for surfaces
set size square
set angle degree
set trange [0:360]
#radius_6_GPS=6

plot file2 u 9:10 with points pointsize 0.3 lc rgb "blue" notitle,\
DRMS*sin(t)+delta_E,DRMS*cos(t)+delta_N lw 3 lc rgb "black" title "DRMS",\
DUE_DRMS*sin(t)+delta_E,DUE_DRMS*cos(t)+delta_N lw 2 lc rgb "gray" title "2DRMS",\
CEP*sin(t)+delta_E,CEP*cos(t)+delta_N lw 1 lc rgb "black"  title "CEP"

