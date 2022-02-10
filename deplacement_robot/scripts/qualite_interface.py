from qualite_analyse_contour import * 
from qualite_pretraitement import * 
import cv2
import time
from matplotlib import pyplot as plt

def fonction_qualite(rayon_attendu,image_raw,debug,showResult = True,fast_algo= True):
    #calcul a la mainm pas fiable.
    start_time = time.time()

    px_to_mm = 0.04624277456
    mm_to_px = 21
    rayon_attendu_px = rayon_attendu*mm_to_px*2  #ici on bosse en px
    #image = cv2.resize(image,(520,388))

    if(fast_algo):
        #if fast we scale down the img
        height, width, _ = image_raw.shape 
        scale = 5
        dsize = (int(width/scale), int(height/scale))
        image_raw = cv2.resize(image_raw, dsize)


    if(debug):
        cv2.imshow('RAW',cv2.resize(image_raw,(520,388)))
        cv2.waitKey(0)


    image_processed,offset_,nohole=filtrage.preprocess(image_raw,fast_algo)
	
    if(nohole):
        return True,"Trou non detecte",image_raw
	

    if(debug):
        cv2.imshow('FILTERED',image_processed)
        cv2.waitKey(0)



    isdefective, defect, contours = analyseContour.caracterization(cv2.cvtColor(image_processed,cv2.COLOR_GRAY2RGB),rayon_attendu_px,px_to_mm,affichage=False)
    
    px_size = 15
    if(fast_algo):
        px_size = 5

    print("image raw")
    if(isdefective):
        image_result = cv2.drawContours(image_raw, contours, -1, (0,0,255), px_size,offset=offset_)
    else:
        image_result= cv2.drawContours(image_raw, contours, -1, (0,255,0), px_size,offset=offset_)
    
    
    if(showResult):
        print("["+str((time.time() - start_time))+" seconds]")
        plt.imshow(cv2.resize(image_result,(520,388)))
        plt.draw()
        plt.pause(0.001)

    return not isdefective, defect, image_result

