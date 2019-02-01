import numpy as np
import cv2
from glob import glob
import matplotlib.pyplot as plt 
from tqdm import tqdm 

class Line:
    # n frame computing average
    color = None
    detected = False
    current_xfitted = None # xs
    current_fit = None # coeff
    radius_of_curvature = 0.0
    def __init__(self, col):
        self.color = col

class Camera:
    foo = 0
    cameraMat = None
    distCoeff = None
    rvecs = []
    tvecs = []


class LaneDetector:
    imgH = 0
    imgW = 0
    distToCtr = 0
    cam = Camera
    Mtrans = None
    Minv = None
    leftLine = Line((0, 0, 255))
    rightLine = Line((255, 0, 0))
    src = np.array([[250, 720], [595, 450], [687, 450], [1170, 720]]).astype(np.float32)
    dst = np.array([[320, 720], [320, 0],   [960, 0],   [960, 720]]).astype(np.float32)

    def __init__(self, h, w):
        self.imgH = h
        self.imgW = w

    def cameraCalib(self, chessboards, nx=9, ny=6):
        grid = np.meshgrid(np.arange(nx), np.arange(ny))
        
        obj_points = []
        img_points = []
    
        objpts = np.zeros((nx*ny, 3), np.float32)
        objpts[:, :2] = np.hstack([grid[0].reshape((-1, 1)), grid[1].reshape((-1, 1))])

        for x in tqdm(chessboards):
          img = cv2.imread(x, 0)
          if img is None:
            raise Exception("cannot load image")
          try:
              ret, imgpts = cv2.findChessboardCorners(img, (nx, ny), None)
              if not ret:
                  raise Exception("failed to extract corners")
          except:
              print("skipped.")
              continue    

          img_points.append(imgpts)
          obj_points.append(objpts)

        flags = 0 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
        ret, camM, distCoeff, rvecs, tvecs =\
            cv2.calibrateCamera(obj_points, img_points, img.shape[::-1], None, None,
                                flags=flags)
        self.cam.cameraMat = camM
        self.cam.distCoeff = distCoeff
        self.cam.rvecs = rvecs
        self.cam.tvecs = tvecs


    def undistImage(self, img):
        camM = self.cam.cameraMat
        distCoeff = self.cam.distCoeff

        undist = cv2.undistort(img, camM, distCoeff, camM)
        undist_col = undist.copy()
        cv2.polylines(undist_col, [np.int32(self.src)], True, (0, 0, 255))
        return undist, undist_col

    def thresholding(self, img):
        R = img[..., 2]
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        S = hls[..., 2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        scld_sobelx = 255*np.abs(sobelx)/(np.max(sobelx))

        R_bin = cv2.threshold(R, 200, 1, cv2.THRESH_BINARY)[1]
        S_bin = cv2.threshold(S, 170, 1, cv2.THRESH_BINARY)[1]
        sobelx_bin = np.zeros_like(gray)
        sobelx_bin[(scld_sobelx < 100) & (scld_sobelx > 20)] = 1
        combined = (R_bin + S_bin + sobelx_bin).astype(np.bool)
        combinedCol = np.stack([sobelx_bin, S_bin, R_bin], -1)*255

        # cv2.imshow("combinedcol", combinedCol)
        # cv2.waitKey()
        return combined, combinedCol

    def warper(self, thrsd, undist_col):
        thrsd_size = (thrsd.shape[1], thrsd.shape[0])
        self.Mtrans = cv2.getPerspectiveTransform(self.src, self.dst)
        warped = cv2.warpPerspective(thrsd.astype(np.uint8), self.Mtrans, thrsd_size,
                                     flags=cv2.INTER_NEAREST)
        warped_undist_col = cv2.warpPerspective(undist_col.astype(np.uint8),
                                                self.Mtrans, thrsd_size,
                                                flags=cv2.INTER_NEAREST)
        # plt.figure(figsize=(20, 10))
        # plt.subplot(1, 2, 1)
        # plt.imshow(cv2.cvtColor(undist_col, cv2.COLOR_BGR2RGB))
        # plt.subplot(1, 2, 2)
        # plt.imshow(cv2.cvtColor(warped_undist_col, cv2.COLOR_BGR2RGB))
        # plt.show()
        return warped


    def noSlidingWind(self, thrsd, thrsd_col, line):
        h, w = thrsd.shape[:2]
        margin = 100
        bestx = line.current_xfitted
        x_inds = []
        y_inds = []
        for i in range(720):
            win_x_low = int(min(w - 1, max(0, bestx[i] - margin)))
            win_x_high = int(min(w - 1, max(0, bestx[i] + margin)))
            # thrsd_col[i, win_x_low] = (0, 255, 0)
            # thrsd_col[i, win_x_high] = (0, 255, 0)
            for j in range(win_x_low, win_x_high):
                if thrsd[i, j] > 0:
                    thrsd_col[i, j] = line.color
                    x_inds.append(j)
                    y_inds.append(i)
        return x_inds, y_inds



    def slidingWindow(self, thrsd, base, thrsd_col, color):
        h, w = thrsd.shape[:2]
        nwindows = 9
        margin = 100
        minpix = 50
        window_height = h/nwindows + (1 if h%nwindows == 0 else 0)
        x_inds = []
        y_inds = []
        for i in range(nwindows):
            good_xinds = []
            good_yinds = []
            win_y_low = int(h - (i+1) * window_height)
            win_y_hight = int(h - i * window_height)

            win_x_low = int(min(max(base - margin, 0), w - 1))
            win_x_hight = int(min(max(base + margin, 0), w - 1))

            # cv2.rectangle(thrsd_col, (win_x_low, win_y_low), (win_x_hight, win_y_hight),
            #               (0, 255, 0), 2);

            for row in range(win_y_low, win_y_hight):
                for col in range(win_x_low, win_x_hight):
                    if thrsd[row, col] > 0:
                        thrsd_col[row, col] = color
                        good_xinds.append(col)
                        good_yinds.append(row)

            x_inds.extend(good_xinds)
            y_inds.extend(good_yinds)

            if len(good_xinds) > minpix:
                base = np.mean(good_xinds)

        return x_inds, y_inds


    def detectLine(self, thrsd):
        thrsd_col = np.zeros(thrsd.shape + (3,), np.uint8)
        H = thrsd.shape[0]
        mid_point = thrsd.shape[1] / 2
        bottomHalf = thrsd[H/2:, :]
        histogram = np.sum(bottomHalf, axis=0)
        left_base = np.argmax(histogram[:mid_point])
        right_base = np.argmax(histogram[mid_point:]) + mid_point
        if not self.leftLine.detected:
            l_xinds, l_yinds = self.slidingWindow(thrsd, left_base, thrsd_col, self.leftLine.color)
        else:
            l_xinds, l_yinds = self.noSlidingWind(thrsd, thrsd_col, self.leftLine)

        if not self.rightLine.detected:
            r_xinds, r_yinds = self.slidingWindow(thrsd, right_base, thrsd_col, self.rightLine.color)
        else:
            r_xinds, r_yinds = self.noSlidingWind(thrsd, thrsd_col, self.rightLine)

        l_coeff = np.polyfit(l_yinds, l_xinds, 2)
        r_coeff = np.polyfit(r_yinds, r_xinds, 2)
        l_bestx = np.poly1d(l_coeff)(np.arange(0, 720)).astype(np.int32)
        r_bestx = np.poly1d(r_coeff)(np.arange(0, 720)).astype(np.int32)

        # for i in range(720):
        #     thrsd_col[i, l_bestx[i]] = (0, 255, 0)
        #     thrsd_col[i, r_bestx[i]] = (0, 255, 0)

        for i in range(720):
            for j in range(l_bestx[i], r_bestx[i]):
                thrsd_col[i, j] = (0, 255, 0)

        self.leftLine.detected = True
        self.rightLine.detected = True
        self.leftLine.current_xfitted = l_bestx
        self.rightLine.current_xfitted = r_bestx
        self.leftLine.current_fit = l_coeff
        self.rightLine.current_fit = r_coeff
        # cv2.imshow("thrsd_col", thrsd_col)
        # cv2.waitKey()
        return thrsd_col


    def warpBack(self, thrsd):
        thrsd_size = (thrsd.shape[1], thrsd.shape[0])
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)
        warpedBack = cv2.warpPerspective(thrsd.astype(np.uint8), self.Minv, thrsd_size,
                                     flags=cv2.INTER_NEAREST)
        return warpedBack

    def computeCurvature(self):
        ym_per_pix = float(30 / 720)

        def computeCurvatureHelper(coeff):
            return (1+(2*coeff[0]*719*ym_per_pix + coeff[1])**2)**1.5/abs(2*coeff[0])

        l_coeff = self.leftLine.current_fit
        self.leftLine.radius_of_curvature = computeCurvatureHelper(l_coeff)
        r_coeff = self.rightLine.current_fit
        self.rightLine.radius_of_curvature = computeCurvatureHelper(r_coeff)

    def computeDistToCtr(self):
        xm_per_pix = float(3.7/700)
        left_bottom = self.leftLine.current_fit[-1]
        right_bottom = self.rightLine.current_fit[-1]
        lane_ctr_warped = np.array([[[left_bottom + float(right_bottom - left_bottom),
                           (719)]]], np.float32)
        lane_ctr_real = cv2.perspectiveTransform(lane_ctr_warped, self.Minv)
        return (lane_ctr_real[0][0][0] - 1280/2) * xm_per_pix


    def overlay(self, wpbk, img):
        self.computeCurvature()
        distToCtr = self.computeDistToCtr()
        avg_radius = int((self.leftLine.radius_of_curvature +\
                          self.rightLine.radius_of_curvature) / 2.0)
        cv2.putText(img, "Radius of curvature: {}".format(avg_radius), (10, 30), 0, 1,
                    (255, 255, 255), 1, 1)
        if distToCtr > 0:
            cv2.putText(img, "Vehicle is {0:.2f} (m) left of center".format(distToCtr),
                        (10, 60), 0, 1, (255, 255, 255), 1, 1)
        else:
            cv2.putText(img, "Vehicle is {0:.2f} (m) right of center".format(distToCtr),
                        (10, 60), 0, 1, (255, 255, 255), 1, 1)
        return cv2.addWeighted(wpbk, 0.3, img, 0.7, 0.0)


    def detect(self, img):
        undist, undist_col = self.undistImage(img)
        thrsd, thrsd_col = self.thresholding(undist)
        warped = self.warper(thrsd, undist_col)
        thrsd_col = self.detectLine(warped)
        warpedBack = self.warpBack(thrsd_col)
        ovrly = self.overlay(warpedBack, img)
        # cv2.imshow("ovrly", ovrly)
        # cv2.waitKey()
        return ovrly


def main():
    # calibration and undist chessboard
    chssbds = glob("camera_cal/*jpg")
    if len(chssbds) == 0: raise Exception("error in load image filenames")
    ld = LaneDetector(0, 0)
    ld.cameraCalib(chssbds, 9, 6)
    # chssbd = cv2.imread("camera_cal/calibration1.jpg")
    # chssbd_undist = ld.undistImage(chssbd)
    # plt.subplot(1, 2, 1)
    # plt.imshow(chssbd)
    # plt.subplot(1, 2, 2)
    # plt.imshow(`chssbd_undist)
    # plt.show()

    # img = cv2.imread("test_images/test2.jpg")
    # img = cv2.imread("test_images/straight_lines1.jpg")
    # ovrly = ld.detect(img)
    # cv2.imshow("ovrly", ovrly)
    # cv2.waitKey()

    # process video
    cap = cv2.VideoCapture("project_video.mp4")
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    if not ret: raise Exception("error loading video")
    ld.imgH, ld.imgW = frame.shape[:2]


    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (w, h))
    # out_m = cv2.VideoWriter('output_monitor.avi', fourcc, 30.0, (w, h))

    indx = 0
    while (cap.isOpened()):
        indx+=1
        print(indx)
        overlay = ld.detect(frame)
        out.write(overlay)
        # out_m.write(monitor)
        ret, frame = cap.read()
    out.release()
    # out_m.release()


if __name__=="__main__":
    main()
