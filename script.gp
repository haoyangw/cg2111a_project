plot [0:320] [0:320] "lidar_reading.dat" using 1:2 with points pointtype 4

while (1) {
    replot
    pause 1
}
