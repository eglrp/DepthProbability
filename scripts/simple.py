import sys
sys.path.append("../cmake-build-debug")
import libdepthProb as dp
import pylab as pl
import numpy as np
import math

import seaborn as sns
sns.set(color_codes=True)


print "gaussion random"
print dp.RandomGaussion(2.0, 1.0, 10)

print "uniform random"
print dp.RandomUniform(-1.0, 5.0, 10)

print "generate gaussion random with uniform distribution"
data = dp.RandomGaussion(2.5, 1.0, 50) + dp.RandomUniform(-10.0, 40.0, 100)

### estimate mu and sigma suppose gaussion
sum = 0
for item in data:
    sum+=item

mu0 = sum/len(data)
sigma0 = 0
for item in data:
    sigma0 += (item-mu0)*(item-mu0)

sigma0 = math.sqrt(sigma0/len(data))

[pi,mu1,sigma1] = dp.solveParams(data)


#pl.hist(data, np.arange(-1., 10, 0.5))
#pl.show()
sns.distplot(data, bins=60, kde=False,)

xs = np.linspace(-10, 60, 400).tolist()
ys0 = dp.GetGaussionSamples(mu0,sigma0,xs)
for i in range(len(ys0)):
    ys0[i] *= pi*len(data)
pl.plot(xs,ys0)


ys1 = dp.GetGaussionSamples(mu1,sigma1,xs)
for i in range(len(ys1)):
    ys1[i] *= pi*len(data)
pl.plot(xs,ys1)

pl.show()


