import cv2
import numpy as np
from cv2 import aruco
from numpy.lib.type_check import imag
import time

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
        cap = cv2.VideoCapture(0)
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
                metric_width = 106.5            # Width of the table in cm

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
    
def get_arukos(img):
    """
    return: 
        corners: [N,4,2]
        ids: [N,1]
    """
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None:
        corners = np.array(corners)
        corners = corners[:,0,:,:]
        return corners, ids 
    else:
        return None, None


def get_yaw(corners, Y=[1,0], X=[0,1], unit='radians'):
    #get the local frame
    y = corners[1] - corners[0]
    x = corners[3] - corners[0]

    y = np.flip(y)
    x = np.flip(x)
    theta = np.dot(x,X)/(np.linalg.norm(x)*np.linalg.norm(X)) #rad

    if unit == 'degrees':
        return theta*180/np.pi #deg
    else:
        return theta

def get_transformation(corners):
    theta = get_yaw(corners)

    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)],[0,0]])
    t = np.array([corners[0][0],corners[0][1],1])

    T = np.concatenate((R,np.reshape(t,(3,1))), axis=1)
    return T

def get_position(corners, img, unit='metric'):
    pix_height = img.shape[0]
    pix_width = img.shape[1]
    metric_height = 2.14             # Height of the table in m 
    metric_width = 1.065            # Width of the table in m
    s1 = metric_height/pix_height
    s2 = metric_width/pix_width
    s = np.mean(([s1,s2])) 

    if unit == 'metric':
        pos = np.flip(corners[0][:])*s
    else:
        pos = np.flip(corners[0])
    return pos

def get_pose(img,ID=[]):
    pose = np.empty(shape=(len(ID),4))
    
    # get aruko corners
    corners, ids = get_arukos(img)

    if corners is None:
        pose[:,:] = -999
        for idx, id in enumerate(ID):
            pose[idx,0] = id

    else:
        for idx,id in enumerate(ID):
            try:
                corner_idx = np.where(ids==id)[0][0] #Check if id is detected
                pos = get_position(corners[corner_idx],img,unit='metric')
                yaw = get_yaw(corners[corner_idx], unit='radians')
                pose[idx,:] = [id,pos[0],pos[1],yaw]
            except:
                pose[idx,:] = [id,-999,-999,-999]

    return pose


if __name__ == "__main__":
    img = cv2.imread('/home/karim/holohover/src/holohover_gnc/holohover_gnc/helpers/imgs/aruko_test_3.jpg')
    robot_id = 1
    puck_id = 2 
    ids = [robot_id,puck_id]
    pose = get_pose(img, ids)

    print("The pose of the robot is : ", pose[0])


