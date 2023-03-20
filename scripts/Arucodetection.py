import cv2
import numpy as np

# Load the dictionary and calibration parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(dictionary,parameters)

def getCord(markerCorners,markerIds):
	out ={}
	for i in range(len(markerIds) if len(markerIds) < 4 else 4):
		x = markerCorners[0][0][i][0]
		y = markerCorners[0][0][i][1]
		id = markerIds[i][0]
		out[id]=(x,y)
	if len(out) == 4:
		x0= out[3][0]
		y0= out[3][1]
		x1= out[4][0]
		y1= out[4][1]
		return (x1-x0,y1-y0)
	return None

# Initialize the video capture device
cap = cv2.VideoCapture(2)

while True:
    # Capture a frame from the video stream
	ret, frame = cap.read()

    # Convert the frame to grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

    # Draw the detected markers on the frame
	if markerIds is not None:
		cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
		print(getCord(markerCorners,markerIds))
   
	
  
	

    # Wait for a key press
	cv2.imshow('frame', frame)
	key = cv2.waitKey(1)

    # Exit the loop if the 'q' key is pressed
	if key == ord('q'):
		break
print(getCord(markerCorners,markerIds))
# Releas
# e the video capture device and close the window
cap.release()
cv2.destroyAllWindows()