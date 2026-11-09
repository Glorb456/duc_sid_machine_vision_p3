""" Calibrates the camera from the chessboard images in images/ and saves the
    camera matrix and distortion coefficients to calibration.npz. """

import os
import numpy as np
import cv2 as cv

HERE = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(HERE, 'images')
OUT_PATH = os.path.join(HERE, 'calibration.npz')
ROWS, COLS = 7, 10  # inner corners of the chessboard

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# chessboard corner positions in board units, like (0,0,0), (1,0,0), ...
objp = np.zeros((ROWS*COLS,3), np.float32)
objp[:,:2] = np.mgrid[0:ROWS,0:COLS].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = sorted(os.path.join(IMAGE_DIR, f) for f in os.listdir(IMAGE_DIR) if f.endswith('.jpg'))

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, (ROWS,COLS), None)
    print(ret)

    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv.drawChessboardCorners(img, (ROWS,COLS), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

if len(objpoints) < 10:
    raise SystemExit(f'only {len(objpoints)} boards detected in {len(images)} images; need 10+')

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

h,  w = img.shape[:2]
np.savez(OUT_PATH, mtx=mtx, dist=dist, width=w, height=h)
print(f'saved {OUT_PATH}')

# reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "total error: {}".format(mean_error/len(objpoints)) )