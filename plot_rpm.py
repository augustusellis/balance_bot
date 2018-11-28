import numpy as np
import matplotlib.pyplot as plt

time = []
pos = []
f = open('rpm_data.txt')
for line in f.readlines():
    dat = line.split(',')
    time.append(float(dat[0]))
    pos.append(int(dat[1][:-1]))

time = np.array(time)[::60]
pos = np.array(pos)[::60]
print(time)
print(pos)

rpm = np.diff(pos)/(46.85*48)/np.diff(time)*60

plt.figure(0)
plt.plot(time[:-1],-rpm)
plt.title('rpm v time')
plt.xlabel('Time [s]')
plt.ylabel('RPM')
#plt.legend(['Measured','Theoretical','Theoretical+\pi'])
#plt.legend(['Meas','Theo'])
plt.grid(True)
plt.show()
