#!/usr/bin/python
import sys
import numpy as np
import cv2
assert sys.version_info.major == 2
assert sys.version_info.minor == 7
assert sys.version_info.micro == 17

class Contour:

    def __init__(self,im_bgr):
        """
        Constructeur
        Parametre : image BGR d'une unique forme, binarisee (noir = hors forme, blanc = forme)
        """
        assert im_bgr is not None, "empty image"
        assert (len(im_bgr.shape) == 3 and im_bgr.shape[2] == 3),"probleme dimensions, image BGR ?"
        self.__im_bgr = im_bgr
        self.__im_grey = cv2.cvtColor(self.__im_bgr,cv2.COLOR_BGR2GRAY)
        ret,im_thresh = cv2.threshold(self.__im_grey,127,255,cv2.THRESH_BINARY)
        self.__im_thresh = im_thresh
        
        #selon documentation : moments sur contours, ou sur image gris... faire un choix
        #self.__moments = cv2.moments(self.getContour())
        self.__moments = cv2.moments(self.__im_thresh,binaryImage=True)

        contours,hierarchy = cv2.findContours(self.__im_thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        assert (len(contours)==1),"plusieurs contours detectes"
        self.__contour = contours[0] 
        

    def getContour(self):
        return self.__contour

    def getImGrey(self):
        return self.__im_grey

    def getImThresh(self):
        return self.__im_thresh

    def getMoments(self):
        return self.__moments

    def getAire(self):
        """aire de la surface definie par le contour, en pixels"""
        return np.round(self.getMoments()['m00']).astype("int")

    def getCentre(self):
        """tuple (ligne, colonne) de la position du centre de la surface definie par le contour, en pixels"""
        return (self.__getLigneCentre(),self.__getColonneCentre())

    def getLigneCentre(self):
        return np.round((self.getMoments()['m10'])/(self.getMoments()['m00'])).astype("int")

    def getColonneCentre(self):
        return np.round((self.getMoments()['m01'])/(self.getMoments()['m00'])).astype("int")

class Cercle(Contour):

    def __init__(self,im_bgr):
        Contour.__init__(self,im_bgr)
        self.__x_circle,self.__y_circle,self.__r_circle = self.findCircle()

    def findCircle(self):
        """
        out : [x,y,rayon] cercle obtenu par transformation de hough
        """
        method = cv2.HOUGH_GRADIENT
        dp = 1 #0.5 : step 2 fois plus grand que resolution image. 2 : step 2 fois plus petit que resolution image
        minDist = 100000 #distance minimale entre cercles detectes (detecter cercle unique si assez grand, voir aucun si trop grand) mais le premier est conserve...
        param1 = 1#	seuil superieur donne a canny (seuil inf= moitie seuil sup)
        param2 = 1# seuil inferieur utilise pour mecanisme de vote (si bas, des faux cercles sont detectes)
        minRadius = 0#seuil inf des cercles recherches
        maxRadius = 0#seuil sup des cercles recherches (si <=0 utilise +gde dimension de l'image, si <0, retourne centres sans donner de rayons)
        circles = cv2.HoughCircles(self.getImGrey(),method,dp,minDist,param2 = param2) #liste de tuples contenant coordonnees cercles et votes
        assert circles is not None,"Pas de cercle trouve"
        assert len(circles.shape) == 3 and circles.shape[0] == 1 and circles.shape[1] == 1 and circles.shape[2] == 3,"Plusieurs cercles trouves"
        x_circle = int(np.round(circles[:1,:1,0:1]))
        y_circle = int(np.round(circles[:1,:1,1:2]))
        r_circle = int(np.round(circles[:1,:1,2:3]))
        return x_circle,y_circle,r_circle
        
    def getXCircle(self):
        return self.__x_circle

    def getYCircle(self):
        return self.__y_circle

    def getRCircle(self):
        return self.__r_circle

    def getCircle(self):
        return [self.__x_circle,self.__y_circle,self.__r_circle]

class analyseContour:

    @staticmethod
    def __contour(im_bgr):
        """
        in : image RGB de la forme 0/255
        out : liste des points formant le contour, abscisse centre contour, ordonnee centre contour, aire du contour
        """
        contour = Contour(im_bgr)
        print("m00 : {}".format(contour.getMoments()['m00']))
        print("mu20 : {}".format(contour.getMoments()['mu20']))
        print("mu02 : {}".format(contour.getMoments()['mu02']))
        print("mu11 : {}".format(contour.getMoments()['mu11']))
        print("nu20 : {}".format(contour.getMoments()['nu20']))
        print("nu02 : {}".format(contour.getMoments()['nu02']))
        print("nu11 : {}".format(contour.getMoments()['nu11']))
        print("nu30 : {}".format(contour.getMoments()['nu30']))
        print("nu03 : {}".format(contour.getMoments()['nu03']))
        print("nu21 : {}".format(contour.getMoments()['nu21']))
        print("nu12 : {}".format(contour.getMoments()['nu12']))
        
        #print("moments : {}".format(contour.getMoments()))
        return contour.getContour(),contour.getLigneCentre(),contour.getColonneCentre(),contour.getAire(),contour.getMoments()

    @staticmethod
    def __cercle(im_bgr):
        """
        in : image rgb binaire
        out : [x,y,rayon] cercle obtenu par hough
        """
        cercle = Cercle(im_bgr)
        return cercle.getCircle()
    
    @staticmethod
    def caracterization(image,rayon,affichage=False):
        """
        fonction caracterization
        in :
        -image (numpy array order 3) binaire 0/255 du contour UNIQUE du trou (trou a 1 valeur, reste a une autre valeur)
        -rayon suppose du trou dans image
        -affichage : booleen pour creer fenetre affichage des differentes etapes
        out :
        - isdefective : booleen (vrai si defaut present)
        - defect : string contenant une forme/defaut reconnue
        - contours : ensemble des pixels formant le contour
        """
        isdefective = False
        defect = ""
        circle = analyseContour.__cercle(image)
        contours,xbar,ybar,aire,moments = analyseContour.__contour(image)
        #caracterisation :

        #entre hough et les contours (la forme est-elle obstruee ou fissuree ?)
        if (aire < 0.94*np.pi*((circle[2])**2)):
            isdefective = True
            defect += "obstruction"# (area of circle > area of real hole) ({}>{})\n".format(float(np.pi*((circle[2])**2)),aire)
        if (aire > 1.06*np.pi*((circle[2])**2)):
            isdefective=True
            defect += "tear "# (area of real hole > area of circle) ({}>{})\n".format(aire,float(np.pi*((circle[2])**2)))
        
        if(moments['nu20']< 0.96*1/(4*np.pi) or moments['nu20']>1.04*1/(4*np.pi)):
            isdefective = True
            #defect += "issue with nu20 moment\n"

        if(moments['nu02']< 0.96*1/(4*np.pi) or moments['nu02']>1.04*1/(4*np.pi)):
            isdefective = True
            #defect += "issue with nu02 moment\n"   

        if(abs(moments['nu11'])>10**-2):
            isdefective=True
            #defect += "issue with nu11 moment\n"

        if(abs(moments['nu21'])>10**-3):
            isdefective=True
            #defect += "issue with nu21 moment\n"

        if(abs(moments['nu12'])>10**-3):
            isdefective=True
            #defect += "issue with nu12 moment\n"

        if(abs(moments['nu30'])>10**-3):
            isdefective=True
            #defect += "issue with nu30 moment\n"

        if(abs(moments['nu03'])>10**-3):
            isdefective=True
            #defect += "issue with nu03 moment\n"

        #entre hough et le parametre de la fonction (le rayon en parametre est-il correct ?)
        if (circle[2]*0.9<=rayon<=circle[2]*1.1):
            pass
        else:
            isdefective=True
            defect += "radius mismatch {}/{}".format(rayon,float(circle[2]))  # function parameter/cercle ({}/{})\n".format(rayon,float(circle[2]))
        

        if affichage:
            cv2.imshow('original image',image)
            cv2.waitKey(0)
            output = image.copy()
            cv2.circle(output,(circle[0],circle[1]),circle[2],(0,0,255),1) #dessine cercle
            cv2.imshow('transfo hough',output)
            cv2.waitKey(0)
            output = np.zeros(image.shape)
            output = cv2.drawContours(image=output,contours=Contour(image).getContour(),contourIdx=-1,color=(0,0,255))
            cv2.imshow('contours',output)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return isdefective, defect, contours

if __name__ == "__main__":
    image = cv2.imread('analyse_contour/81radius.jpg')
    defect,resultat = analyseContour.caracterization(image,81,True)
    print(defect)
    print(resultat)