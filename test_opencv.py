import cv2
import numpy as np
import matplotlib.pyplot as plt

print("OpenCV version:", cv2.__version__)

# Test 1: Just show version and a dummy image
img = np.zeros((300, 400, 3), dtype=np.uint8)
img[:] = (0, 255, 0)  # green background
cv2.putText(img, "It works!", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

cv2.imshow("Test Window - press any key", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Test 2: matplotlib version (no window popup)
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.title("Matplotlib way")
plt.axis('off')
plt.show()

print("All good! You can start learning OpenCV now.")
