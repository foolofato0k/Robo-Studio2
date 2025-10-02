import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import potrace
import os

def detectFaceEdges(img):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # convert to grayscale for processing

    # FACE DETECTION ______________________
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.realpath(__file__))
    cascade_path = os.path.join(script_dir, 'haarcascade_frontalface_default.xml')

    # Load Haar Cascade from the same directory as the script
    face_cascade = cv.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        raise ValueError("Haar cascade file could not be loaded.")

    # Face detection
    faces = face_cascade.detectMultiScale(img, 1.3, 5)

    if len(faces) == 0:
        print("No Face Detected!")
        return None, None
    #elif len(faces) > 3:
    #    print("More than 3 Faces Detected!")
    #    return None, None        

    # Get Region of Interest
    roi = img.copy()
    for (x,y,w,h) in faces:
        roi = roi[y:y+h, x:x+w]
    roi = cv.resize(roi, (250, 250)) # resized to a standard size for canvas creation 
    
    # EDGE DETECTION ______________________
    # Add Gussian Blur and Detect Edges
    img_blur = cv.GaussianBlur(roi, (3,3), 0)
    edge_img = cv.Canny(image=img_blur, threshold1=50, threshold2=120) # Canny Edge Detection

    clean_img = reduceNoise(edge_img)
    return clean_img

def createPoster(img): # private function
        
        canvas = np.zeros((400,400),dtype=np.uint8) # create poster

        # ADD TEXT_______________________________
        Heading = 'WANTED'
        subtext1 = 'DEAD OR ALIVE'
        subtext2 = '$10'
        font_large = 4 # Scale text relative to canvas size
        font_small = font_large * 0.5  # Smaller text
#
        heading_pos = (80, 60)  # Top part
        subtext1_pos = (80, 360)  # Bottom part
        subtext2_pos = (160, 395)  # Near the bottom center
#
        canvas = cv.putText(canvas, Heading, org=heading_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_large, color=255, thickness=1, lineType=cv.LINE_AA)
        canvas = cv.putText(canvas, subtext1, org=subtext1_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1, lineType=cv.LINE_AA)
        canvas = cv.putText(canvas, subtext2, org=subtext2_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1, lineType=cv.LINE_AA)

        canvas = cv.flip(canvas, 1)
        canvas[75:325, 75:325] = img


        return canvas

def groupEdges(img):

    _, img = cv.threshold(img, 127, 255, cv.THRESH_BINARY)
    num_labels, labels_img = cv.connectedComponents(img) # get the groups

    masks = []
    
    for label in range(1, num_labels):  # skip background
        mask = (labels_img == label).astype(np.uint8)
        masks.append(mask) # add group masks to array

    return num_labels, masks

def reduceNoise(img):

    _, masks = groupEdges(img)
    clean_img = np.zeros(img.shape, dtype=np.uint8)
    

    for mask in masks:
        if cv.countNonZero(mask) >= 40:
            clean_img[mask != 0] = 255
            
    # Apply Morphology kernal
    kernel = np.ones((3, 3), np.uint8)
    cleaned_img = cv.morphologyEx(clean_img, cv.MORPH_CLOSE, kernel)

    return cleaned_img

def getPaths(img):

    ### OLD VERSION ---- 2172 strokes
    _, masks = groupEdges(img)
    paths = []
    for mask in masks:
        bitmap = potrace.Bitmap(mask.astype(bool))
        path = bitmap.trace()
        paths.append(path)
    
    ### NEW VERSION ---- 2082 strokes (but the eyes are missing)
    #contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    #paths = []
    #for cnt in contours:
    #    mask = np.zeros_like(img)
    #    cv.drawContours(mask, [cnt], -1, color=255, thickness=cv.FILLED)
    #    bitmap = potrace.Bitmap(mask.astype(bool))
    #    path = bitmap.trace()
    #    paths.append(path)

    return paths

def tesselate(curves, distance_threshold=40.0):

    stroke_plan = []
    for path in curves:
        for curve in path:
            curve_verts = curve.tesselate()
            if len(curve_verts) < 10:
                continue

            current_stroke = [curve_verts[0]]

            for i in range(1, len(curve_verts)-2):
                if i % 2 != 0:
                    prev = np.array(curve_verts[i-1])
                    curr = np.array(curve_verts[i+2])
                    if np.linalg.norm(curr - prev) > distance_threshold:
                        stroke_plan.append(current_stroke)
                        current_stroke = []
                    current_stroke.append(tuple(curr))
            if current_stroke:
                stroke_plan.append(current_stroke)
    return stroke_plan

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

#####################################################################
if __name__ == '__main__':
     
    image = None
     
    webcam = False
    if webcam:
        image = WebcamImg(image)
        if image == None:
            exit()
            
    else:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        image_path = os.path.abspath(os.path.join(
                    script_dir,
                    '..', '..','..',                       # go up into src
                    'gui',
                    'gui',
                    'capture_image',
                    'webcam_img.jpg'
                ))
        image = cv.imread(image_path)
    
    if image is None:
        print(f"Failed to load image. Check the path: {image_path}")
        exit()

    # PROCESSING EDGES ________________________________________
    poster = detectFaceEdges(image)
    paths = getPaths(poster)

    # SHOWING OUTPUTS _________________________________________
    # Image
    cv.imshow('poster', poster)
    #cv.imwrite('test_images/poster.png',poster)

    cv.waitKey(0)
    cv.destroyAllWindows()


    # Paths
    fig, ax = plt.subplots(figsize=(6, 6))
    i = 0
    for path in paths:
        for curve in path:
            curve_verts = curve.tesselate()
            if len(curve_verts) < 25:
                continue

            x, y = zip(*curve_verts)
            ax.scatter(x, y, marker='o', s=2, label=f'Group {i}')
        i += 1
    
    print(f"paths: {i}")
    plt.show()