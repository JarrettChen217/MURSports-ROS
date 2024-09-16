import cv2
import numpy as np

image = np.zeros((480, 640, 2), dtype=np.uint8)

cv2.imshow("Frame", image)
cv2.waitKey(0)
cv2.destroyAllWindows()