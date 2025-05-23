import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import potrace

def detectFaceEdges(img):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # convert to grayscale for processing

    # FACE DETECTION ______________________
    # Detect Face
    face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(img, 1.3, 5)

    # Get Region of Interest
    roi = img.copy()
    for (x,y,w,h) in faces:
        roi = roi[y:y+h, x:x+w]
    roi = cv.resize(roi, (200, 200)) # for canvas creation - can comment out if needed
    

    # EDGE DETECTION ______________________
    # Add Gussian Blur
    img_blur = cv.GaussianBlur(roi, (3,3), 0)
    # Detect Edges
    edge_img = cv.Canny(image=img_blur, threshold1=20, threshold2=120) # Canny Edge Detection

    poster = createPoster(edge_img)

    return poster

def createPoster(img): # private function
        
        # CREATE POSTER - based on face image shape ___________________________
        buffer = 2
        w, h = img.shape
        canvas = np.zeros((w*buffer,h*buffer),dtype=np.uint8) # create poster
        cw, ch = canvas.shape

        # Casting the original image into the center of the canvas
        o_w = int(cw/buffer) - int(w/buffer)
        o_h = int(ch/buffer) - int(h/buffer)
        
        canvas[o_w:w+o_w, o_h:h+o_h] = img
        canvas = cv.rectangle(canvas,(o_w,o_h),(w+o_w,h+o_h),color=255,thickness=1)

        # ADD TEXT_______________________________
        font_large = 5 # Scale text relative to canvas size
        font_small = font_large * 0.5  # Smaller text

        wanted_pos = (o_w-50, o_h-10)  # Top part
        dead_or_alive_pos = (o_w-50, ch-50)  # Bottom part
        reward_pos = (o_w+50, ch-10)  # Near the bottom center

        canvas = cv.putText(canvas, 'WANTED', org=wanted_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_large, color=255, thickness=1)
        canvas = cv.putText(canvas, 'DEAD OR ALIVE', org=dead_or_alive_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1)
        canvas = cv.putText(canvas, '$10', org=reward_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1)

        return canvas

def getPaths(img):

    img = cv.threshold(img, 127, 255, cv.THRESH_BINARY)[1] # convert to black and white - with one chanel
    num_labels, labels_img = cv.connectedComponents(img) # get the groups - mess with the parameters later
    h, w = labels_img.shape

    paths = []
    pt_type = None

    for label in range(num_labels):

        mask = (labels_img == label).astype(np.uint8) 
        bitmap = potrace.Bitmap(mask.astype(bool))
        path = bitmap.trace()

        paths.append(path)

    return paths

def WebcamImg(image):

    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame. Exiting ...")
            break

        cv.imshow('frame', frame)
        if cv.waitKey(1) == ord('s'):
            image = frame
            break
    cap.release()

    return image


if __name__ == '__main__':
     
    image = None
     
    webcam = False
    if webcam:
        image = WebcamImg(image)
        if image == None:
            exit()
            
    else:
        image = cv.imread('test_images/webcam_img.jpg')
    
    if image is None:
        print("Failed to load image. Check the path: test_images/webcam_img.jpg")
        exit()

    # PROCESSING EDGES ________________________________________
    poster = detectFaceEdges(image)
    paths = getPaths(poster)

    # SHOWING OUTPUTS _________________________________________
    # Image
    cv.imshow('poster', poster)
    cv.imwrite('test_images/poster.png',poster)

    cv.waitKey(0)
    cv.destroyAllWindows()

    # Paths
    fig, ax = plt.subplots(figsize=(6, 6))
    i = 0
    for path in paths:
        if i == 0:
            i += 1
            continue
        for curve in path:
            curve_verts = curve.tesselate()
            x, y = zip(*curve_verts)
            ax.scatter(x, y, marker='o', s=2, label=f'Group {i}')

        i += 1
    plt.show()