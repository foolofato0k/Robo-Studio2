import cv2 as cv
import numpy as np

# Read image and convert it to grayscale for processing _______________
image = cv.imread('webcam_unprocessed_img.jpg')
img_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# Face detection _____________
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
faces = face_cascade.detectMultiScale(img_gray, 1.3, 5)

# Draw rectangle around the face _____
for (x,y,w,h) in faces:
        roi_gray = img_gray[y:y+h, x:x+w]

# Seperate face image _______
img_face = roi_gray.copy()
cv.imshow('img_face', img_face) # show processed image

# Contour Detection__________________

# Add a blur to simplify for contour processing __________________
img_gray = cv.GaussianBlur(img_gray, (3,3), 0) 

# convert to binary image for contour detection
ret, thresh = cv.threshold(img_face, 150, 255, cv.THRESH_BINARY)
cv.imshow('img_r_binary', thresh) # show binary image

contours, hierarchy = cv.findContours(image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)

new_img = np.zeros(img_face.shape) 

cv.drawContours(image=new_img, contours=contours, contourIdx=-1, color=(255, 255, 255), thickness=1)
cv.imshow('Contours image', new_img) # show contours overlayed image

print("number of contours: " + str(len(contours)))

'''
# looking at contour hierarchy
for i in range(len(contours)):

        cv.drawContours(image=new_img, contours=contours, contourIdx=i, color=(255, 255, 255), thickness=1)
        
        cv.waitKey(0)

        print("points in contour " + str(i) + ":")
        print(str(contours[i]))
        cv.imshow('contour hierarch', new_img) # show contours overlayed image
        print(str(len(contours[i])))
'''
cv.waitKey(0)
cv.destroyAllWindows()



