from cv2 import cv2
import numpy as np
src1 = np.array([[x, y, 1] for x in range(10) for y in range(10)], np.float64)
src2 = np.array([[x * 2 + 1, y * 3 + 2] for x in range(10) for y in range(10)], np.float64)
ret, A = cv2.solve(src1, src2, flags=cv2.DECOMP_SVD)
print(A)
print(np.matmul(src1[0:10], A))