import cv2 as cv
import numpy as np
import svgwrite # need to pip install it once
import svgpathtools as spt

def FaceDetector(img):

    # FACE DETECTION ________________
    # convert to grayscale for face detection and processing
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(img_gray, 1.3, 5)

    # get Region of Interest
    roi = None
    for (x,y,w,h) in faces:
        roi = img[y:y+h, x:x+w]
    
    # Mask out everything but the face
    mask = np.zeros_like(img_gray)
    mask[y:y+h, x:x+w] = 255
    img_face = img_gray.copy()
    img_mask = cv.bitwise_and(img_face, img_face, mask=mask)

    img_roi = roi.copy()
    
    return img_mask, img_roi

def EdgeDetector(img, blur, thresh1, thresh2):
    # EDGE DETECTION _________________________________
    # Add a blur to simplify for edge processing
    img = cv.GaussianBlur(img, (blur, blur), 0)
    edge_img = cv.Canny(image=img, threshold1=thresh1, threshold2=thresh2) # Canny Edge Detection\
    return edge_img
     
def CreatePoster(img):

    img = cv.resize(img, (200, 200))

    # CREATE POSTER - based on face image shape
    poster = np.zeros((400,400),dtype=np.uint8) # create poster

    w, h = img.shape
    poster[100:100+w,100:100+h] = img # trying to put the face image in the center
    pw, ph = poster.shape

    # ADD TEXT
    font_large = 5 # Scale text relative to canvas size
    font_small = font_large * 0.5  # Smaller text

    wanted_pos = (int(0.12 * pw), int(0.2 * ph))  # Top part
    dead_or_alive_pos = (int(0.12 * pw), int(0.85 * ph))  # Bottom part
    reward_pos = ( int(0.35 * pw), int(0.95 * ph))  # Near the bottom center

    poster = cv.putText(poster, 'WANTED', org=wanted_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_large, color=255, thickness=1)
    poster = cv.putText(poster, 'DEAD OR ALIVE', org=dead_or_alive_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1)
    poster = cv.putText(poster, '$100000', org=reward_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1)

    return poster

def CreateSVG(img):
    # Converting to SVG file format - or can just give the points where there are edges
    edge_pts = [] # can save edge points if want
    w, h = img.shape
    dwg = svgwrite.Drawing("face2.svg",profile='tiny')
    prev_pt = None

    for w_pt in range(w):
            for h_pt in range(h):
                    if img[w_pt,h_pt] == 255:
                            if prev_pt != None:
                                    line = dwg.line(prev_pt, (h_pt,w_pt), stroke=svgwrite.rgb(0,0,0,'%'))
                                    dwg.add(line)
                                    edge_pts.append([h_pt,w_pt])
                    
                    prev_pt = [h_pt,w_pt]
    
    return dwg

if __name__ == '__main__':
    
    # get webcam image
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        if cv.waitKey(5) == ord('q'):
            break

        # Process Image
        img_mask, img_roi = FaceDetector(frame) # masked image for web cam out put, roi image for further processing
        cv.imshow('face_img', img_mask)

    cap.release()

    # get image - if not webcam
    #frame = cv.imread('webcam_img2.jpg')

    # Face Detection
    img_mask, img_roi = FaceDetector(frame)
    cv.imshow('face_ image', img_mask)

    # Edge detector
    edge_img = EdgeDetector(img_roi, 3, 20, 120) # image, guassian blur, thresh 1, thresh 2

    # Create Poster
    poster = CreatePoster(edge_img)
    #cv.imwrite('poster2.jpg', poster)
    cv.imshow('poster.jpg', poster)

    # Create SVG for processing
    #svg = CreateSVG(poster)
    #svg.save()
    
    # Get Paths from svg
    #file_path = 'face_1.svg'
    #paths, attributes = spt.svg2paths(file_path)

    #for i, path in enumerate(paths):
    #    print(f"Path {i+1}: {path}")
    

    cv.waitKey(0)
    cv.destroyAllWindows()
