set terminal png size 1920,1080
set datafile separator ','
set key autotitle columnhead
plot 'testi.csv' using 0:8 with lines, 'testi.csv' using 0:9 with lines, 'testi.csv' using 0:10 with lines, 'testi.csv' using 0:11 with lines
