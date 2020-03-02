#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
from math import pi
import matplotlib.cm as cm


# Parameters to use when opening the webcam.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1

magentaMin = np.array([139,  50,  100], dtype=np.uint8)
magentaMax = np.array([189, 255, 200], dtype=np.uint8)
cianoMin = np.array([85,  50,  100], dtype=np.uint8)
cianoMax = np.array([135, 255, 255], dtype=np.uint8)

#fonte para letra
font = cv2.FONT_HERSHEY_SIMPLEX

# Returns an image containing the borders of the image
# sigma is how far from the median we are setting the thresholds
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged
# Cria o detector BRISK
brisk = cv2.BRISK_create()


# Configura o algoritmo de casamento de features que vê *como* o objeto que deve ser encontrado aparece na imagem
bf = cv2.BFMatcher(cv2.NORM_HAMMING)

# Define o mínimo de pontos similares
MINIMO_SEMELHANCAS = 10
# Essa função vai ser usada abaixo. Ela encontra a matriz (homografia) 
# que quando multiplicada pela imagem de entrada gera a imagem de 


def find_homography_draw_box(kp1, kp2, img_cena):
    
    out = img_cena.copy()
    
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)


    # Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
    # Esta transformação é chamada de homografia 
    # Para saber mais veja 
    # https://docs.opencv.org/3.4/d9/dab/tutorial_homography.html
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()


    
    h,w = img_original.shape
    # Um retângulo com as dimensões da imagem original
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

    # Transforma os pontos do retângulo para onde estao na imagem destino usando a homografia encontrada
    dst = cv2.perspectiveTransform(pts,M)


    # Desenha um contorno em vermelho ao redor de onde o objeto foi encontrado
    img2b = cv2.polylines(out,[np.int32(dst)],True,(255,255,0),5, cv2.LINE_AA)
    
    return img2b
    

def find_good_matches(descriptor_image1, frame_gray):
    """
        Recebe o descritor da imagem a procurar e um frame da cena, e devolve os keypoints e os good matches
    """
    des1 = descriptor_image1
    kp2, des2 = brisk.detectAndCompute(frame_gray,None)

    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = bf.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    return kp2, good

original_bgr = cv2.imread("insper_logo.png")  # Imagem a procurar
img_original = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2GRAY)
original_rgb = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2RGB)


# Encontra os pontos únicos (keypoints) nas duas imagems
kp1, des1 = brisk.detectAndCompute(img_original ,None)

while(True):
    # Capture frame-by-frame
    print("New frame")
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(gray,(5,5),0)

    # Detect the edges present in the image
    bordas = auto_canny(blur)

    #Faz um map em HSV
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    circles = []

    # reseta o valor da distancia do papel
    papel = None

    # reseta o angulo e vetor para novo frame
    angulo = None
    vetor = ()

    # reset das coordenadas dos centros
    magenta = ()
    ciano = ()

    # Obtains a version of the edges image where we can draw in color
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)

    # HoughCircles - detects circles using the Hough Method. For an explanation of
    # param1 and param2 please see an explanation here http://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    circles = None
    circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=100,minRadius=5,maxRadius=60)
    
    kp2, good_matches = find_good_matches(des1, gray)
    if len(good_matches) > MINIMO_SEMELHANCAS:
        print("Matches found")    
        bordas_color = find_homography_draw_box(kp1, kp2, bordas_color)
    else:
        print("Not enough matches are found - %d/%d" % (len(good_matches),MINIMO_SEMELHANCAS))

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            #print(i)
            
            mask = cv2.inRange(hsvFrame, magentaMin, magentaMax)
            if mask[i[1]][i[0]] > 200:
                cv2.circle(bordas_color,(i[0],i[1]),i[2],(int(frame[i[1],i[0]][0]), int(frame[i[1],i[0]][1]), int(frame[i[1],i[0]][2])),-1)
                # Salva as coordenadas do circulo magenta
                magenta = (i[0],i[1])           
                cv2.circle(bordas_color,(i[0],i[1]),2,(0,0,255),3)
            mask = cv2.inRange(hsvFrame, cianoMin, cianoMax)
            if mask[i[1]][i[0]] > 200:
                cv2.circle(bordas_color,(i[0],i[1]),i[2],(int(frame[i[1],i[0]][0]), int(frame[i[1],i[0]][1]), int(frame[i[1],i[0]][2])),-1)
                # Salva as coordenadas do círculo ciano
                ciano = (i[0],i[1])
                cv2.circle(bordas_color,(i[0],i[1]),2,(0,0,255),3)
            
    
    #Desenha a reta entre os centros dos círculos
    try:
        cv2.line(bordas_color,magenta,ciano,(0,255,0),5)
        if magenta [0] > ciano[0] and magenta[1] > ciano[1]:
            dist = ((magenta[0]-ciano[0])**2+(magenta[1]-ciano[1])**2)**(1/2)
            papel = (777 * 14 / dist)
            angulo = (180-abs(np.arctan((magenta[1]-ciano[1])/(magenta[0]-ciano[0]))*180/np.pi))
            
        elif magenta[0] > ciano[0] and ciano[1] > magenta[1]:
            dist = ((magenta[0]-ciano[0])**2+(ciano[1]-magenta[1])**2)**(1/2)
            papel = (777 * 14 / dist)
            angulo = (abs(np.arctan((ciano[1]-magenta[1])/(magenta[0]-ciano[0]))*180/np.pi))

        elif ciano[0] > magenta[0] and ciano[1] > magenta[1]:
            dist = ((ciano[0]-magenta[0])**2+(ciano[1]-magenta[1])**2)**(1/2)
            papel = (777 * 14 / dist)
            angulo = (180-abs(np.arctan((ciano[1]-magenta[1])/(ciano[0]-magenta[0]))*180/np.pi))

        elif ciano[0] > magenta[0] and magenta[1] > ciano[1]:
            dist = ((ciano[0]-magenta[0])**2+(magenta[1]-ciano[1])**2)**(1/2)
            papel = (777 * 14 / dist)
            angulo = (abs(np.arctan((magenta[1]-ciano[1])/(ciano[0]-magenta[0]))*180/np.pi))
            
        elif ciano[1] == magenta[1]:
            angulo = "pi/2"

        else:
            angulo = "0"   

        cv2.putText(bordas_color,"%.2f cm" % papel,(0,130), font, 2,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(bordas_color,"%.2f graus" % angulo,(0,50), font, 2,(255,255,255),2,cv2.LINE_AA)
    except:
        print("cade circulo")

    # Display the resulting frame
    cv2.imshow('Detector de circulos',bordas_color)
    print("No circles were found")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
