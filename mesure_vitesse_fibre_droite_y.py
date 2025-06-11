# -*- coding: utf-8 -*-
"""
Created on Wed Jun 11 13:42:22 2025

@author: Alok
"""

import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
import fitutils as fu

cap = cv2.VideoCapture(0)

fgbg = cv2.createBackgroundSubtractorMOG2()
t = []
pos_milieu = []

while(1):
    ret, frame = cap.read()

    fgmask = fgbg.apply(frame)
    column = fgmask[:,202]
    
    fgmask2 = fgmask
    fgmask2[:,::100] = 255

    cv2.imshow('frame',fgmask2)
    indexWhites = np.where(column==127)
    pos_milieu.append(np.mean(indexWhites))
    print(column)
    print(indexWhites)
    t.append(time.time())
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()

t = np.array(t)
pos_milieu = np.array(pos_milieu)

#%%

plt.figure()
plt.plot(t, pos_milieu)

# slow
plt.plot(t[530:780], pos_milieu[530:780]) # JogPositive
plt.plot(t[860:1180], pos_milieu[860:1180]) # JogNegative

# medium
plt.plot(t[1277:1334], pos_milieu[1277:1334]) # JogPositive
plt.plot(t[1414:1480], pos_milieu[1414:1480]) # JogNegative

# fast
plt.plot(t[1610:1640], pos_milieu[1610:1640]) # JogPositive
plt.plot(t[1727:1761], pos_milieu[1727:1761]) # JogNegative
plt.show()

res_pos_slow = fu.linfitxy(t[550:780], pos_milieu[550:780], dx=1e-6, dy=2, marker='.', plot=True)
res_neg_slow = fu.linfitxy(t[860:1180], pos_milieu[860:1180], dx=1e-6, dy=2, marker='.', plot=True)
res_pos_medi = fu.linfitxy(t[1277:1334], pos_milieu[1277:1334], dx=1e-6, dy=2, marker='.', plot=True)
res_neg_medi = fu.linfitxy(t[1414:1480], pos_milieu[1414:1480], dx=1e-6, dy=2, marker='.', plot=True)
res_pos_fast = fu.linfitxy(t[1610:1640], pos_milieu[1610:1640], dx=1e-6, dy=2, marker='.', plot=True)
res_neg_fast = fu.linfitxy(t[1727:1761], pos_milieu[1727:1761], dx=1e-6, dy=2, marker='.', plot=True)

print(np.abs(res_neg_slow[0]/res_pos_slow[0]))
print(np.abs(res_neg_medi[0]/res_pos_medi[0]))
print(np.abs(res_neg_fast[0]/res_pos_fast[0]))

pourcentage = np.array([np.abs(res_neg_slow[0]/res_pos_slow[0]), np.abs(res_neg_medi[0]/res_pos_medi[0]), np.abs(res_neg_fast[0]/res_pos_fast[0])])
vitesse = np.array([100, 500, 1000])

plt.figure()
plt.plot(vitesse, pourcentage, '+')
plt.show()

#%%

# Software Velocity (steps/s)
softVelocity = np.array([100, 500, 1000])

# Screen Velocity (pixels/s)
screenVelocity_pos = np.array((res_pos_slow[0], res_pos_medi[0], res_pos_fast[0]))
screenVelocity_neg = np.array((res_neg_slow[0], res_neg_medi[0], res_neg_fast[0]))

# True Velocity (m/s)
trueVelocity_pos = screenVelocity_pos * 36e-6 / 490  # car 490px = 36µm
trueVelocity_neg = screenVelocity_neg * 36e-6 / 490


print(trueVelocity_pos)
print(trueVelocity_neg)


# Trucs à faire:
    # Implementer un choix de vitesse "en ligne" pour utiliser le bouton 'hat'
    # Fusionner le code de controle de manette et le code d'aquisition de la vidéo pour faciliter la prise de mesure
    # Mesurer la vraie vitesse pour les deux fibres, sur les deux axes pour plus de vitesses:
        # Exemple : faire varier la vitesse entre 10 et 2000 par pas de 10 sur [10, 100], puis 100 sur [100, 2000]
        # Pour l'axe x, rien besoin de changer, il y a une partie immobile donc qui n'apparait pas à la caméra
    # Reconfectionner le mode "configuration" afin qu'il n'interfere pas avec le texte
    # Redimmensionner l'écran et voir si il y a un moyen de l'afficher à un endroit précis de l'écran
