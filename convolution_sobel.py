import cv2
import numpy as np

# Charge l'image en niveaux de gris
image = cv2.imread('test_convo.png', cv2.IMREAD_GRAYSCALE)

# Définis ton noyau de convolution (exemple : flou gaussien 3x3)
kernel = np.array([[-1, 0, 1],
                   [-2, 0, 2],
                   [-1, 0, 1]], dtype=np.float32)

# kernel = kernel / kernel.sum()  # Normalise

# Applique la convolution
convolved = cv2.filter2D(image, -1, kernel)

cv2.imshow('convolved.png', convolved)
cv2.waitKey(0)           # Attend que tu appuies une touche
cv2.destroyAllWindows()  # Ferme la fenêtre
