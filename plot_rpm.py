import numpy as np
import matplotlib.pyplot as plt

time = []
pos = []
f = open('rpm_data.txt')
for line in f.readlines():
    dat = line.split(',')
    time.append(float(dat[0]))
    pos.append(int(dat[1][:-1]))

time = np.array(time)
pos = np.array(pos)

rpm = np.diff(pos)/(46.85*48)/np.diff(time)*60

plt.figure(0)
plt.plot(time[:-1],-rpm)
plt.title('pos v time')
plt.xlabel('Time [s]')
plt.ylabel('Position [counts]')
#plt.legend(['Measured','Theoretical','Theoretical+\pi'])
#plt.legend(['Meas','Theo'])
plt.grid(True)
plt.show()
