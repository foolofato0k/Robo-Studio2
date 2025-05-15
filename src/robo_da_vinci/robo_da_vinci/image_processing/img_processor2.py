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
    # Get Region of Interest
    roi = img.copy()
    for (x,y,w,h) in faces:
        roi = roi[y:y+h, x:x+w]
    
    roi = cv.resize(roi, (250, 250)) # resized to a standard size for canvas creation 
    

    # EDGE DETECTION ______________________
    # Add Gussian Blur
    img_blur = cv.GaussianBlur(roi, (3,3), 0)
    # Detect Edges
    edge_img = cv.Canny(image=img_blur, threshold1=20, threshold2=120) # Canny Edge Detection

    # Try to clean image
    kernel = np.ones((3, 3), np.uint8)
    edge_img = cv.morphologyEx(edge_img, cv.MORPH_CLOSE, kernel)

    edge_img = createPoster(edge_img) # without text

    return edge_img

def createPoster(img): # private function
        
        canvas = np.zeros((400,400),dtype=np.uint8) # create poster
        canvas[75:325, 75:325] = img
        cv.rectangle(canvas,(75,75),(325,325),color=255,thickness=1, lineType=cv.LINE_AA)

        # ADD TEXT_______________________________
        #Heading = 'WANTED'
        #subtext1 = 'DEAD OR ALIVE'
        #subtext2 = '$10'
        #font_large = 4 # Scale text relative to canvas size
        #font_small = font_large * 0.5  # Smaller text
#
        #heading_pos = (80, 60)  # Top part
        #subtext1_pos = (80, 360)  # Bottom part
        #subtext2_pos = (160, 395)  # Near the bottom center
#
        #canvas = cv.putText(canvas, Heading, org=heading_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_large, color=255, thickness=1, lineType=cv.LINE_AA)
        #canvas = cv.putText(canvas, subtext1, org=subtext1_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1, lineType=cv.LINE_AA)
        #canvas = cv.putText(canvas, subtext2, org=subtext2_pos, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=font_small, color=255, thickness=1, lineType=cv.LINE_AA)

        return canvas

def getPaths(img):

    ### OLD VERSION of contour generation - publishes 1872 strokes
    img = cv.threshold(img, 127, 255, cv.THRESH_BINARY)[1] # convert to black and white - with one chanel
    num_labels, labels_img = cv.connectedComponents(img) # get the groups
    paths = []
    for label in range(num_labels):
        if label == 0: # first label is the background
            continue
        else:
            mask = (labels_img == label).astype(np.uint8) 
            bitmap = potrace.Bitmap(mask.astype(bool))
            path = bitmap.trace()
            paths.append(path)

    ### NEW VERSION of contour generation - publishes 1762 strokes
    #contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    #paths = []
    #for cnt in contours:
    #    mask = np.zeros_like(img)
    #    cv.drawContours(mask, [cnt], -1, color=255, thickness=cv.FILLED)
    #    bitmap = potrace.Bitmap(mask.astype(bool))
    #    path = bitmap.trace()
    #    paths.append(path)
    
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


    
def tesselate(curves, distance_threshold=40.0):
    stroke_plan = []
    for path in curves:
        for curve in path:
            curve_verts = curve.tesselate()
            if len(curve_verts) < 25:
                continue

            current_stroke = [curve_verts[0]]

            for i in range(1, len(curve_verts)):
                prev = np.array(curve_verts[i-1])
                curr = np.array(curve_verts[i])
                if np.linalg.norm(curr - prev) > distance_threshold:
                    stroke_plan.append(current_stroke)
                    current_stroke = []
                current_stroke.append(tuple(curr))
            if current_stroke:
                stroke_plan.append(current_stroke)
    return stroke_plan


if __name__ == '__main__':
     
    image = None
     
    webcam = False
    if webcam:
        image = WebcamImg(image)
        if image == None:
            exit()
            
    else:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        image_path = os.path.join(script_dir, 'test_images', 'webcam_img.jpg')
        image = cv.imread(image_path)
    
    if image is None:
        print("Failed to load image. Check the path: test_images/webcam_img.jpg")
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
            x, y = zip(*curve_verts)
            ax.scatter(x, y, marker='o', s=2, label=f'Group {i}')
        i += 1
    plt.show()