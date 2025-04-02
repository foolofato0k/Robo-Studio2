from pixels2svg import pixels2svg

import svgwrite
import svgpathtools as spt

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev


def convertSVG(img):
        file = "poster.svg"
        svg = svgwrite.Drawing("o.svg",profile='tiny')
        w, h = img.shape
        
        prev_pt = None
        for w_pt in range(w):
                for h_pt in range(h):
                        if img[w_pt,h_pt] == 255:
                                if prev_pt != None:
                                        segment = svg.line(prev_pt, (h_pt,w_pt), stroke=svgwrite.rgb(0,0,0,'%'))
                                        svg.add(segment)
                        prev_pt = [h_pt,w_pt]
        
        svg.save()
        return file

def getPath(svg):
        paths, attributes = spt.svg2paths(svg) # get paths

        # convert polar to cartesian coordinatates
        cart_paths = []
        for path in paths:
            start_pt = pol2cart(path.start)
            end_pt = pol2cart(path.end)
            
            cart_paths.append([start_pt, end_pt])

        return cart_paths
    
def pol2cart(polar): # private function
        x = polar.real
        y = polar.imag
        return (x,y)

if __name__ == '__main__':
    
    # manual output
    input_path = 'poster.jpg'
    img = cv.imread(input_path)
    svg = convertSVG(img)

    ## pixel2svg output
    #input_path = 'test_images/poster.png'
    #svg = 'test_images/svg_out.svg'
    #pixels2svg(input_path, svg, remove_background=True)'

    # try pypotrace next

    # get cartesian paths
    paths = getPath(svg)
    print(paths)