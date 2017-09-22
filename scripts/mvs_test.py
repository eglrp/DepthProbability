import sys
sys.path.append("../cmake-build-release")
import libdepthProb as dp
import numpy as np
import matplotlib.pyplot as plt

mvs = dp.mvs(6)
mvs.readScene("/home/jianping/workspace/datas/depthtest/sparse.mvs")


# data = mvs.getImage(1)
# width, height = data[0:2]
# imageValues = data[2:]
# image = np.array(imageValues)
# image.resize([height, width, 3])
# image=image*1/255.0
# lum_img = image[:,:,0]

#mvs.estimateDepthMap(0)

data = mvs.getDepth(0)
width, height = data[0:2]
print [width, height]
imageValues = data[2:]
image = np.array(imageValues)
print image
# image=image*1/255.0
image.resize([height, width])


plt.imshow(image)
plt.show()