from skimage import io
from skimage import morphology
from skimage.morphology import medial_axis
from skimage.morphology import skeletonize
from scipy import ndimage
import imageio
import matplotlib.pyplot as plt
import cv2
import numpy as np
image = io.imread('OGM_empty.pgm')
[rows, cols] = image.shape
image[image<255] = 0
image[image==255] = 1

# Compute the medial axis (similar to voronoi) and the distance transform, and is refered to [page](https://scikit-image.org/docs/0.4/auto_examples/plot_medial_transform.html)
skel, distance = medial_axis(image, return_distance=True)
dist_on_skel = distance * skel
# Compute the skeleton which is much more smoother than the medial_axis method, and is refered to [page](https://scikit-image.org/docs/dev/auto_examples/edges/plot_skeleton.html)
skeleton = skeletonize(image)
skeleton_lee = skeletonize(image, method='lee') # small branches are less

fig, axes = plt.subplots(2, 2, figsize=(8, 4), sharex=True, sharey=True)
ax = axes.ravel()

dist_on_skel = 1.0 * (dist_on_skel > 1) # invert the image

ax[0].imshow(np.invert(image), cmap=plt.cm.binary)
ax[0].set_title('original')
ax[0].axis('off')

ax[1].contour(image, [0.5], colors='w')
ax[1].imshow(skeleton, cmap=plt.cm.gray)
ax[1].set_title('skeletonize')
ax[1].axis('off')

ax[2].contour(image, [0.5], colors='w')
ax[2].imshow(skeleton_lee, cmap=plt.cm.gray)
ax[2].set_title('skeletonize (Lee 94)')
ax[2].axis('off')

ax[3].contour(image, [0.5], colors='w')
ax[3].imshow(dist_on_skel, cmap=plt.cm.gray) #, interpolation='nearest')
ax[3].set_title('medial axis (similar to voronoi diagram)')
ax[3].axis('off')

fig.tight_layout()
#plt.show()

plt.clf()
plt.imsave("skeleton.png", skeleton, cmap=plt.cm.gray)

import cv2
img = cv2.imread('skeleton.png',0)
kernel = np.ones((2,2), np.uint8)
img_dilation = cv2.dilate(img, kernel, iterations=1)
cv2.imwrite('dilated.png',img_dilation)
