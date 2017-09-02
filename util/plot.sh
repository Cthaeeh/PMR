echo "Start plot script"

TITLE=" example title"
XLABEL=" example x label"
YLABEL=" example y label"
OUTPUT="output.jpg"

gnuplot <<PLOT

set terminal jpeg
set output "$OUTPUT"
set title "$TITLE"
set xlabel "$XLABEL"
set ylabel "$YLABEL"
set offset graph 0.1, graph 0.2, graph 0.1, graph 0.1
set style line 1 lt rgb "red"  lw 2 pt 0
set style line 2 lt rgb "blue" lw 2 pt 0

plot "positions_odo.csv" with linespoints ls 1 ,"positions_amcl.csv" with linespoints ls 2

quit
PLOT
