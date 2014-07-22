#set terminal pdf color font "Bold,14"
#set output "IFEN_accuracy.pdf"

set terminal jpeg font "Helvetica, 14"
set output "4_GPS_3_GAL_accuracy_precision.jpeg"

set grid 
set xrange [-10:10]
set yrange [-5:15]
set ylabel "North [m]" 
set xlabel "East [m]"

set key Left left
set title "IFEN simulated data, 4 GPS, 8 Gal - Accuracy and Precision"
#file1="8_GPS_GNSS_SDR_solutions.txt"
#file2="8_GAL_GNSS_SDR_solutions.txt"
file3="4_GPS_3_GAL_GNSS_SDR_solutions.txt"

#values to copy from statistic file
DRMS= 3.077806456
DUE_DRMS= 6.155612912
CEP= 2.565164055


#difference with respect to the reference position
#values to copy from statistic file
delta_E= -1.812 # combined
delta_N= 3.596  # combined

set parametric
#dummy variable is t for curves, u/v for surfaces
set size square
set angle degree
set trange [0:360]
#radius_6_GPS=6

plot file3 u 9:10 with points pointsize 0.3 lc rgb "green" notitle,\
DRMS*sin(t)+delta_E,DRMS*cos(t)+delta_N lw 3 lc rgb "black" title "DRMS",\
DUE_DRMS*sin(t)+delta_E,DUE_DRMS*cos(t)+delta_N lw 2 lc rgb "gray" title "2DRMS",\
CEP*sin(t)+delta_E,CEP*cos(t)+delta_N lw 1 lc rgb "black"  title "CEP"

