cd ../build/
cmake ../src/
make
./ekf_observer > data.txt
mv data.txt ../src/plot/Sample/
cd ../src/plot/Sample/
python plotter.py
