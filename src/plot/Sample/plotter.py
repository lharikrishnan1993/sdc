#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
import re

fk = open('data.txt', 'r')
#fs = open('data_s.txt', 'r')

kekf_x = []
kekf_y = []
ktrue_x = []
ktrue_y = []
kmeas_x = []
kmeas_y = []

sekf_x = []
sekf_y = []
strue_x = []
strue_y = []
smeas_x = []
smeas_y = []

for line in fk:
    line = line.split(':')
    if line[0] == 'Measurement(K)':
	line = line[1].replace('((', ':').replace('))',':').replace(')',':').replace('(',':').replace(':,:',':').replace(':,:',':')
	line = line.split(":")
	kmeas_x.append(line[1])
	kmeas_y.append(line[2])
    elif line[0] == 'EKF State(K)':
        line = line[1].replace('((', ':').replace('))',':').replace(')',':').replace('(',':').replace(':,:',':').replace(':,:',':')
        line = line.split(":")
        kekf_x.append(line[1]) 
        kekf_y.append(line[2])
    elif line[0] == 'True_State(K)':
        line = line[1].replace('((', ':').replace('))',':').replace(')',':').replace('(',':').replace(':,:',':').replace(':,:',':')
        line = line.split(":")
        ktrue_x.append(line[1]) 
        ktrue_y.append(line[2])

#for line in fs:
#    line = line.split(':')
    elif line[0] == 'Measurement(S)':
        line = line[1].replace('((', ':').replace('))',':').replace(')',':').replace('(',':').replace(':,:',':').replace(':,:',':')
        line = line.split(":")
        smeas_x.append(line[1])
        smeas_y.append(line[2])
    elif line[0] == 'EKF State(S)':
        line = line[1].replace('((', ':').replace('))',':').replace(')',':').replace('(',':').replace(':,:',':').replace(':,:',':')
        line = line.split(":")
        sekf_x.append(line[1])
        sekf_y.append(line[2])
    elif line[0] == 'True_State(S)':
        line = line[1].replace('((', ':').replace('))',':').replace(')',':').replace('(',':').replace(':,:',':').replace(':,:',':')
        line = line.split(":")
        strue_x.append(line[1])
        strue_y.append(line[2])


#Kinematic
a, = plt.plot(strue_x, strue_y, 'b-', label='Bicycle Model True')
#plt.plot(smeas_x, smeas_y, 'ko')
b, = plt.plot(sekf_x, sekf_y, 'k-', label='Bicycle Model EKF')

#Simple
c, = plt.plot(ktrue_x, ktrue_y, 'g-', label='Dubins Model True')
#plt.plot(kmeas_x, kmeas_y, 'r*')
d, = plt.plot(kekf_x, kekf_y, 'r-', label='Dubins Model EKF')
plt.legend([a, b, c, d], ['Bicycle Model True', 'Bicycle Model EKF', 'Dubins Model True', 'Dubins Model EKF'])
plt.show()
