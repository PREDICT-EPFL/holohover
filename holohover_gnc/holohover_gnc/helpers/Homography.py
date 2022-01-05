import cv2
import numpy as np
from numpy.lib.type_check import imag
import time 
import pandas as pd

def project_to_floor(image_coordinates, H): 
  """
  This method takes the Homography matrix and the 2d image cartesian coordinates. It returns the (x, y)
  cartesian coordinates in 3d cartesian world coordinates on floor plane(at z=0). Notice that z coordinate is omitted
  here and added inside the tracking funtion. 
  
  Parameters
  ----------
  image_coordinates: 2d pixel coordinates (x,y)
  h: 3x3 Homography matrix np.array[3x3]
  Returns
  ----------
  floor_coordinates: List of x, y coordinates in 3d world of same pixel on floor plane i.e. (x,y,z) Considering z=0 and 
  ommitted here.
  """
  #adding 1 for homogenous coordinate system
  x, y, w = H @ np.array([[*image_coordinates, 1]]).T
  return [x/w, y/w]


def extract_edge(event, x, y, flags, param):
    if(event == cv2.EVENT_LBUTTONDOWN):
            refPt.append((x, y))


def order_points(pts):
	# initialzie a list of coordinates that will be ordered
	# such that the first entry in the list is the top-left,
	# the second entry is the top-right, the third is the
	# bottom-right, and the fourth is the bottom-left
	rect = np.zeros((4, 2), dtype = "float32")
	# the top-left point will have the smallest sum, whereas
	# the bottom-right point will have the largest sum
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
	# now, compute the difference between the points, the
	# top-right point will have the smallest difference,
	# whereas the bottom-left will have the largest difference
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]
	# return the ordered coordinates
	return rect

def four_point_transform(image, pts):
	# obtain a consistent order of the points and unpack them
	# individually
	rect = order_points(pts)
	(tl, tr, br, bl) = rect
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))
	# now that we have the dimensions of the new image, construct
	# the set of destination points to obtain a "birds eye view",
	# (i.e. top-down view) of the image, again specifying points
	# in the top-left, top-right, bottom-right, and bottom-left
	# order
	dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")
	# compute the perspective transform matrix and then apply it
	M = cv2.getPerspectiveTransform(rect, dst)
	warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
	# return the warped image
	return M, warped

def get_metric_position(point,s,M):
    return ((M @ point))[0:2]  

def get_Transformation(img=[], id=0, save=True):
    # returns the transformation matrix M[3x3] which maps a homogeneous pixel in the image frame [u,v,1] to air
    # hockey points [x,y,1]
    global refPt
    refPt = []
    warp = False

    if img is None:
        cap = cv2.VideoCapture(2)
        time.sleep(2)
        ret,img = cap.read()
        time.sleep(0.5)
        cap.release() 
        

    # keep looping until the 'q' key is pressed
    while True:
        # display the image and wait for a keypress
        cv2.imshow("image", img)
        cv2.setMouseCallback("image", extract_edge)
        key = cv2.waitKey(1) & 0xFF
        # if the 'c' key is pressed, break from the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            break

        if len(refPt) == 4:    
                # draw a polygon around the region of interest
                tmp = np.array(refPt,np.int32)
                cv2.polylines(img, [tmp], 1, (0, 255, 0), 2)
                # write point coordinates in a file
                warp = True


        if warp == True:
                # apply the four point tranform to obtain a "birds eye view" of
                # the image
                M, warped = four_point_transform(img, np.array(refPt))
                np.savetxt('warp.csv', M, delimiter=",")
                # show the original and warped images
                cv2.imshow("Warped", warped)
                
                # get scaling factor
                pix_height = warped.shape[0]
                pix_width = warped.shape[1]
                metric_height = 214             # Height of the table in cm 
                metric_width = 104           # Width of the table in cm

                s1 = metric_height/pix_height
                s2 = metric_width/pix_width
                s = np.mean(([s1,s2]))

                M = M*s
                if save:
                    np.savetxt('M.csv', M, delimiter=",")
                    np.savetxt('refPt.csv', np.array(refPt), delimiter=",")
                else:
                    continue



    return M, refPt


if __name__ == "__main__":
    #M = get_Transformation(img=None) 
    CV_CAP_PROP_FRAME_WIDTH = 3
    CV_CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_FPS = 5  
    cap = cv2.VideoCapture(0)      
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))               #Change if needed
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1920)
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(CAP_PROP_FPS, 60)
    ret, frame = cap.read()
    M = get_Transformation(frame)