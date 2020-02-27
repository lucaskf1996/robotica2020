#!/usr/bin/python
# -*- coding: utf-8 -*-


import numpy as np
import cv2

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


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)

    original_bgr = cv2.imread("insper_logo.png")  # Imagem a procurar
    img_original = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2GRAY)
    original_rgb = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2RGB)


    # Encontra os pontos únicos (keypoints) nas duas imagems
    kp1, des1 = brisk.detectAndCompute(img_original ,None)


    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read(0)
        
        if ret == False:
            print("Problema para capturar o frame da câmera")
            continue

        # Our operations on the frame come here
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp2, good_matches = find_good_matches(des1, gray)

        if len(good_matches) > MINIMO_SEMELHANCAS:
            print("Matches found")    
            frame = find_homography_draw_box(kp1, kp2, frame)
        else:
            print("Not enough matches are found - %d/%d" % (len(good_matches),MINIMO_SEMELHANCAS))
         
        
        cv2.imshow("teste brisk", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

