import pylab as plt
import sys

f = open('data.txt', 'r')
x_s = []
y_s = []
x_k = []
y_k = []
counter = 0;

cnt = int(sys.argv[1])

for line in f:
    line = line.split(" ")
    if len(x_k) < cnt:
	x_k.append(float(line[0]))
	y_k.append(float(line[1]))
    elif len(x_s) < cnt:
        x_s.append(float(line[0]))
        y_s.append(float(line[1]))
    	
plt.plot(x_s, y_s, 'b-', label = 'Simple')
plt.plot(x_k, y_k, 'r-', label = 'Kinematic')
plt.legend(loc='upper right')
plt.show()
