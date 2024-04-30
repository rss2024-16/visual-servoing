import numpy as np
import matplotlib.pyplot as plt

data = np.load('stats3.npy',allow_pickle=True)
# print(data)
# x = data.get('x')
y = data.item().get('y')
x = data.item().get('x')
dist = data.item().get('dist')

plt.figure()
plt.xlabel('Timestep')
plt.ylabel('Differences')
plt.plot([x for x in range(len(x))],x)
plt.plot([x for x in range(len(y))],y)
plt.plot([x for x in range(len(dist))],dist)
plt.legend(['X','Y','Dist'])
plt.show()

