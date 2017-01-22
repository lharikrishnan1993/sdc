#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
import re

fk = open('car.txt', 'r')
fs = open('bicycle.txt', 'r')

k_x = []
k_y = []

s_x = []
s_y = []

for line in fk:
    line = line.split(' ')
    s_x.append(line[0])
    s_y.append(line[1])

for line in fs:
    line = line.split(' ')
    k_x.append(line[0])
    k_y.append(line[1])

#Kinematic
a, = plt.plot(s_x, s_y, 'b-', label='Bicycle Model True')

#Simple
c, = plt.plot(k_x, k_y, 'g-', label='Dubins Model True')

plt.legend([a, c], ['Dubins Car Model', 'Bicycle Model'])
plt.show()
