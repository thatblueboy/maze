import cv2
import numpy as np

# Load the dictionary and calibration parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(dictionary,parameters)

def getCord(markerCorners,markerIds):
	out =[]
	for i in range(0,len(markerIds)):
		x = markerCorners[0][i][0][0]
		y = markerCorners[0][i][0][1]
		id = markerIds[i][0]
		out+=[(x,y,id)]
	return out
def calcCord(cord):
	x = cord[0]
	y = cord[1]
	id = cord[2]
	if id == 0:
		x = x - 0.5
		y = y - 0.5
	elif id == 1:
		x = x + 0.5
		y = y - 0.5
	elif id == 2:
		x = x + 0.5
		y = y + 0.5
	elif id == 3:
		x = x - 0.5
		y = y + 0.5
	return (x,y,id)
# Initialize the video capture device
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the video stream
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

    # Draw the detected markers on the frame
    if markerIds is not None:
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
   
	
    cv2.imshow('frame', frame)
    print(markerIds)

    # Wait for a key press
    key = cv2.waitKey(1)

    # Exit the loop if the 'q' key is pressed
    if key == ord('q'):
        break
print(type(markerCorners))
# Release the video capture device and close the window
cap.release()
cv2.destroyAllWindows()