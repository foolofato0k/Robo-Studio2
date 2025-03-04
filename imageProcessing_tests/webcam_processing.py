import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

switchToContourDetection = False # if you want to test with contour detection (but I feel like we should stick with edge detection)
frameCount = 0

## If you want to save the video
#fps = cap.get(cv.CAP_PROP_FPS)
#width = int(cap.get(3))
#height = int(cap.get(4))
#save_vid = cv.VideoWriter("saved_video2.mp4", cv.VideoWriter_fourcc('m','p','4','v'), fps=fps, frameSize=(width,height))
## to save the video uncomment the save_vid.write() function in the loop

while True:
    # Capture frame-by-frame - if frame is read correctly ret is True
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # convert frame to grayscale for processing
    
    # Face detection
    face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml') # load model
    faces = face_cascade.detectMultiScale(frame_gray, 1.3, 5) # run

    # To mask out everything but the face
    mask = np.zeros_like(frame_gray) # make a mask

    # Get Region of interest
    for (x,y,w,h) in faces:
            roi_gray = frame_gray[y:y+h, x:x+w]
            mask[y:y+h, x:x+w] = 255
    
    # Apply mask to the frame (it makes a new frame that is a copy of the old one and then applys the mask to it)
    frame_face = frame_gray.copy()
    frame_face = cv.bitwise_and(frame_face, frame_face, mask=mask)

    img_gray = cv.GaussianBlur(frame_face, (5,5), 0) # apply blur for processing (nesscery for edge detection but not contour)
    
    # Default uses edge detection
    if switchToContourDetection:
        ret, thresh = cv.threshold(frame_face, 60, 255, cv.THRESH_BINARY)
        contours, hierarchy = cv.findContours(image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)
        out_frame = np.zeros(frame_face.shape) # create a new image to draw the contours on
        cv.drawContours(image=out_frame, contours=contours, contourIdx=-1, color=(255, 255, 255), thickness=1)
        cv.imshow('Contours image', out_frame) # show contours overlayed image

    else:
        out_frame= cv.Canny(image=frame_face, threshold1=85, threshold2=100) # Canny Edge Detection
        cv.imshow('Canny Edge Detection', out_frame)

    frameCount += 1
    print('frame count: ' + str(frameCount))

    # Saving
    # #save_vid.write(frame) # save unprocessed video - just change frame to out_frame to save the processed video
    # cv.imshow('frame', frame) # show unprocessed frame if u want
    
    # Exit key is 's' - can save images here as well if u want
    if cv.waitKey(1) == ord('s'):
        cv.imwrite('webcam_unprocessed_img.jpg',frame) # save unprocessed frame if u want
        #cv.imwrtie('webcam_processed_img.jpg', out_frame) # save procssed frame
        print('Image Saved!')
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()