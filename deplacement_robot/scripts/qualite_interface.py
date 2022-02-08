from qualite_analyse_contour import * 
from qualite_pretraitement import * 
import cv2


def fonction_qualite(rayon_attendu,image_raw,debug):
    #calcul a la mainm pas fiable.
    px_to_mm = 0.04624277456
    mm_to_px = 21.625
    rayon_attendu = rayon_attendu*mm_to_px  #ici on bosse en px
    #image = cv2.resize(image,(520,388))

    if(debug):
        cv2.imshow('RAW',cv2.resize(image_raw,(520,388)))
	cv2.waitKey(0)


    image_processed,offset_,nohole=filtrage.preprocess(image_raw)
	
    if(nohole):
	return True,"Trou non detecte",image_raw
	

    if(debug):
        cv2.imshow('FILTERED',image_processed)
	cv2.waitKey(0)



    isdefective, defect, contours = analyseContour.caracterization(cv2.cvtColor(image_processed,cv2.COLOR_GRAY2RGB),rayon_attendu,px_to_mm,affichage=False)

    print("image raw")
    if(isdefective):
        image_result = cv2.drawContours(image_raw, contours, -1, (0,0,255), 15,offset=offset_)
    else:
        image_result= cv2.drawContours(image_raw, contours, -1, (0,255,0), 15,offset=offset_)
    
    
    if(debug):
        cv2.imshow('RESULT',cv2.resize(image_result,(520,388)))
	cv2.waitKey(0)

    return isdefective, defect, image_result

