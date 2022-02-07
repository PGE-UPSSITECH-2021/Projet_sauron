from qualite_analyse_contour import * 
from qualite_pretraitement import * 
import cv2


def fonction_qualite(rayon_attendu,image,debug):

    #image = cv2.resize(image,(520,388))

    if(debug):
        cv2.imshow('RAW',image)


    image=filtrage.preprocess(image)

    if(debug):
        cv2.imshow('FILTERED',image)


    isdefective, defect, contours = analyseContour.caracterization(image,rayon_attendu)
    if(is_defective):
        cv2.drawContours(image, contours, -1, (0,0,255), 15)
    else:
        cv2.drawContours(image, contours, -1, (0,255,0), 15)
    
    
    if(debug):
        cv2.imshow('RESULT',image)

    return isdefective, defect, image

