import numpy as np
import cv2
from glob import glob
import matplotlib.pyplot as plt 
from tqdm import tqdm 

class Line:
    # n frame computing average
    detected = False
    current_xfitted = [] # xs
    current_fit = [] # coeff
    radius_of_curvature = 0.0
    line_base_pos = 0.0

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
    Mtrains = None
    Minv = None
    leftLine = Line()
    rightLine = Line() 

    def __init__(self, h, w):
        self.imgH = h
        self.imgW = w

    def cameraCalib(self, chessboards, nx=9, ny=6):
        num_imgs = len(chessboards)
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
              continue    

          img_points.append(imgpts)
          obj_points.append(objpts)

        flag = 0 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
        ret, camM, distCoeff, rvecs, tvecs =\
            cv2.calibrateCamera(obj_points, img_points, img.shape[::-1], None, None)
        self.cam.cameraMat = camM
        self.cam.distCoeff = distCoeff
        self.cam.rvecs = rvecs
        self.cam.tvecs = tvecs


    def undistImage(self, img):
        camM = self.cam.cameraMat
        distCoeff = self.cam.distCoeff
        rvecs = self.cam.rvecs
        tvecs = self.cam.tvecs

        undist = cv2.undistort(img, camM, distCoeff, camM)     

        return undist

    def thresholding(self, img):
        R = img[..., 2]
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        S = img[..., 2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        scld_sobelx = 255*np.abs(sobelx)/(np.max(sobelx))

        R_bin = cv2.threshold(R, 200, 1, cv2.THRESH_BINARY)[1]
        S_bin = cv2.threshold(S, 170, 1, cv2.THRESH_BINARY)[1]
        sobelx_bin = np.zeros_like(gray)
        sobelx_bin[(sobelx < 100) & (sobelx > 20)] = 1 

        combined = (R_bin + S_bin + sobelx_bin).astype(np.bool)
        combinedCol = np.stack([sobelx_bin, S_bin, R_bin], -1)*255

        return combined, combinedCol

    def warper(self, img, src, dst):
        img_size = (img.shape[1], img.shape[0])
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_NEAREST) 
        return warped


    def slidingWindowHelper(self, thrsd):
     


    def slidingWindow(self, thrsd):
        H = thrsd.shape[0]
        nwindwos = 9
        margin = 100
        minpix = 50
        window_height = H/nwindows +\
            (1 if H%nwindows == 0 else 0)
        bottomHalf = img[:, H/2:]
        histogram = np.sum(bottomHalf, axis=1)
        if (not leftLine.detected) {

        } else {
            lefLine.current
        }

        if (not rightLine.detected) {
           
        } else {
            
        }
        

    def detect(self, img):
        undist = self.undistImage(img)
        thrshd, thrshd_col = self.thresholding(undist)
        return None, None

def main():
    # calibration and undist chessboard
    chssbds = glob("../camera_cal/*jpg")
    if len(chssbds) == 0: raise Exception("error in load image filenames")
    ld = LaneDetector(0, 0)
    ld.cameraCalib(chssbds, 9, 6)
    #chssbd = cv2.imread("../camera_cal/calibration1.jpg")
    #chssbd_undist = ld.undistImage(chssbd)
    #plt.imshow(chssbd_undist)
    #plt.show()

    # process video
    cap = cv2.VideoCapture("../project_video.mp4")
    ret, frame = cap.read()
    if not ret: raise Exception("error loading video")
    h, w = frame.shape[:2]

    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('output.avi', fourcc, 30.0, (w, h))
    #out_m = cv2.VideoWriter('output_monitor.avi', fourcc, 30.0, (w, h))
    
    while (cap.isOpened()):
      overlay, monitor = ld.detect(frame)
      #out.write(overlay)
      #out_m.write(monitor)
      ret, rame = cap.read()
      break
        

if __name__=="__main__":
    main()
