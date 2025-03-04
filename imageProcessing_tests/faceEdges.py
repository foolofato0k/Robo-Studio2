import cv2 as cv

# Read image and convert it to grayscale for processing _______________________________
image = cv.imread('webcam_unprocessed_img.jpg')
img_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# Face detection ____________________________________
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
faces = face_cascade.detectMultiScale(img_gray, 1.3, 5)

# Get Region of interest
for (x,y,w,h) in faces:
        roi_gray = img_gray[y:y+h, x:x+w]
img_face = roi_gray.copy() # focus only on the face

# Add a blur to simplify for edge processing ____________________
img_face = cv.GaussianBlur(img_face, (5,5), 0)
cv.imshow('blurred image', img_face)

# Edge detection _________________________________
edge_img = cv.Canny(image=img_face, threshold1=20, threshold2=120) # Canny Edge Detection
cv.imshow('Canny Edge Detection', edge_img)

'''
# Can resize the edge image
edge_resize = cv.resize(edge_img, (0, 0), fx = 2, fy = 2)
cv.imshow('resized edge image', edge_resize)
print('original size: ' + str(len(edge_img)))
print('new size:' + str(len(edge_resize)))
'''

cv.waitKey(0)
cv.destroyAllWindows()