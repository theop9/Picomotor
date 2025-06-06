import numpy as np
import pygame


pygame.init()
pygame.joystick.init()
screen = pygame.display.set_mode((600, 600))
pygame.display.set_caption("Contrôle clavier")
clock = pygame.time.Clock()
running = True
dt = 0


class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 25)

    def tprint(self, screen, text):
        text_bitmap = self.font.render(text, True, (0, 0, 0))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

text_print = TextPrint()

# Détection de la manette
if pygame.joystick.get_count() == 0:
    print("Aucune manette détectée")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Manette détectée : {joystick.get_name()} \n")

# Définition des home positions (récupérer directement avec a query DH? ou avec une fonction de la DLL)
home_1_x = 200
home_1_y = 200
home_1_z = 0
home_2_x = 400
home_2_y = 200
home_2_z = 0


# Initialisation du mode de déplacement
indexMode = 0
Mode = np.array(['independent fibers', 'simultaneous'])
actualMode = Mode[indexMode]
isModeChanged = False
print(f"Mode choisi par défault : {actualMode}")

# Initialisation du mode de vitesse
slow_velocity = 100 #steps/s
medium_velocity = 500 #steps/s
fast_velocity = 1000 #steps/s
velocity = np.array([slow_velocity, medium_velocity, fast_velocity])
ModeVitesse = np.array(['slow', 'medium', 'fast'])

indexModeVitesse = 0
#o.set_all_velocity(int(velocity[indexModeVitesse]))
actualModeVitesse = ModeVitesse[indexModeVitesse]
isModeVitesseChanged = False
print(f"Mode de vitesse choisi par défaut : {actualModeVitesse}")


# Initialisation du mouvement des moteurs (immobiles au démarrage)
isMoving = np.array([[False, False, False], # right : x, y, z
                     [False, False, False]]) # left : x, y, z


fibre1_pos = pygame.Vector3(2*screen.get_width() / 3, screen.get_height() / 2, 0)
fibre2_pos = pygame.Vector3(screen.get_width() / 3, screen.get_height() / 2, 0)

def get_direction(buttons):
    direction = np.zeros_like(buttons, str)
    for i in range(len(buttons)):
        #print(f"i = {i}")
        for j in range(len(buttons[i])):
            #print(f"j = {j}, signe = {np.sign(buttons[i,j])}")
            if np.sign(buttons[i,j]) == 1:
                direction[i,j] = '+'
            elif np.sign(buttons[i,j]) == -1:
                direction[i,j] = '-'
            else:
                direction[i,j] = '0'
    return direction

# Main code
try:
    while running:
        
        # Affichage des modes de déplacement et de vitesse
        screen.fill((255, 255, 255))
        text_print.reset()
        text_print.tprint(screen, f"Mode : {actualMode}")
        text_print.tprint(screen, f"Velocity : {actualModeVitesse} ({velocity[indexModeVitesse]} steps/sec)")
        
        # Affichage des home positions
        pygame.draw.line(screen, "red", (home_1_x - 2, home_1_y - 2), ((home_1_x + 2, home_1_y + 2)), width=1)
        pygame.draw.line(screen, "red", (home_1_x - 2, home_1_y + 2), ((home_1_x + 2, home_1_y - 2)), width=1)
        pygame.draw.line(screen, "blue", (home_2_x - 2, home_2_y - 2), ((home_2_x + 2, home_2_y + 2)), width=1)
        pygame.draw.line(screen, "blue", (home_2_x - 2, home_2_y + 2), ((home_2_x + 2, home_2_y - 2)), width=1)
        pygame.display.flip()
        
        # Manières d'arreter le "jeu" : fermer la fenêtre, appuyer sur espace, interrompre le noyau
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    print("Espace pressée, arrêt du programme.")
                    running = False
                    
        # # Stop all motion immediately
        # btnAbort = joystick.get_button(10)
        # if btnAbort == 1:
        #     o.abort_motion()
            
        
        
        # Changement de mode de déplacement
        btnMode = joystick.get_button(6)
        
        if btnMode == 1:
            if not isModeChanged:
                indexMode += 1
                indexMode = indexMode%len(Mode)
                actualMode = Mode[indexMode]
                print(f"Déplacement : {actualMode}")
                isModeChanged = True
        if btnMode == 0:
            if isModeChanged:
                isModeChanged = False
                
        # Changement de mode de vitesse   
        btnModeVitesse = joystick.get_button(7)             
        if btnModeVitesse == 1:
            if not isModeVitesseChanged:
                indexModeVitesse += 1
                indexModeVitesse = indexModeVitesse%len(ModeVitesse)
                actualModeVitesse = ModeVitesse[indexModeVitesse]
                #o.set_all_velocity(int(velocity[indexModeVitesse]))
                print(f"Vitesse : {actualModeVitesse}")
                # print(cmdModeVitesse[indexModeVitesse])
                isModeVitesseChanged = True
        if btnModeVitesse == 0:
            if isModeVitesseChanged:
                isModeVitesseChanged = False
                
        
        # Déplacement des fibres
        left_joystick_x = joystick.get_axis(0)
        left_joystick_y = - joystick.get_axis(1) # - car défini avec le positif vers le bas (en regardant du haut)
        left_joystick_z_pos = joystick.get_button(4)
        left_joystick_z_neg = joystick.get_axis(4)
        left_joystick_z = left_joystick_z_pos - (left_joystick_z_neg + 1)/2
        
        right_joystick_x = joystick.get_axis(2)
        right_joystick_y = - joystick.get_axis(3) # - car défini avec le positif vers le bas (en regardant du haut)
        right_joystick_z_pos = joystick.get_button(5)
        right_joystick_z_neg = joystick.get_axis(5)
        right_joystick_z = right_joystick_z_pos - (right_joystick_z_neg + 1)/2

        
        buttons = np.array([[right_joystick_x, right_joystick_y, right_joystick_z], 
                            [left_joystick_x, left_joystick_y, left_joystick_z]])
        
        isTouched_list = np.array([[(np.abs(right_joystick_x) > 0.1), (np.abs(right_joystick_y) > 0.1), right_joystick_z_pos != 0 or right_joystick_z_neg != -1],
                                   [(np.abs(left_joystick_x) > 0.1), (np.abs(left_joystick_y) > 0.1), left_joystick_z_pos != 0 or left_joystick_z_neg != -1]])
        

                        
        direction = get_direction(buttons)
        
        # Mode indépendant 
        if indexMode == 0:              
            cmd = ''
            for addr in [1,2]:
                for axis in [1,2,3]:
                    isTouched = isTouched_list[addr-1, axis-1]
                    if isTouched:
                        # Commande Picomotor
                        if not isMoving[addr-1, axis-1]:
                            cmd += f"{addr}>{axis}MV{direction[addr-1, axis-1]};"
                            isMoving[addr-1, axis-1] = True
                            
                    else:
                        if isMoving[addr-1, axis-1]:
                            cmd += f"{addr}>{axis}ST;"
                            isMoving[addr-1, axis-1] = False
                            
    
            if cmd !='':
                cmd = cmd[:-1]
                print(cmd)
                #o.query(cmd)

                
        # Mode pincettes
        elif indexMode == 1:
            cmd = ''
            for axis in [1,2,3]:
                
                # Déplacement en bloc : joystick droit
                isTouchedRight = isTouched_list[0, axis-1] 
                if isTouchedRight:
                    if not isMoving[0, axis-1]:
                        cmd += f"1>{axis}MV{direction[0, axis-1]};2>{axis}MV{direction[0, axis-1]};"
                        isMoving[0, axis-1] = True
                else:
                    if isMoving[0, axis-1]:
                        cmd += f"1>{axis}ST;2>{axis}ST;"
                        isMoving[0, axis-1] = False
                        
                # Déplacement opposé (pour pincer) : joystick gauche
                isTouchedLeft = isTouched_list[1, axis-1] 
                if isTouchedLeft and joystick.get_button(1)==1:
                    # Commande Picomotor
                    if not isMoving[1, axis-1]:
                        if direction[1, axis-1] == '+':
                            cmd += f"1>{axis}MV+;2>{axis}MV-;"
                        elif direction[1, axis-1] == '-':
                            cmd += f"1>{axis}MV-;2>{axis}MV+;"
                        isMoving[1, axis-1] = True

                else:
                    if isMoving[1, axis-1]:
                        cmd += f"1>{axis}ST;2>{axis}ST;"
                        isMoving[1, axis-1] = False
                        
            if cmd != '':
                cmd = cmd[:-1]
                print(cmd)
                #o.query(cmd)
                
        # positions = o.get_all_positions()
        # fibre1_pos.x, fibre1_pos.y, fibre1_pos.z = positions[0]
        # fibre2_pos.x, fibre2_pos.y, fibre2_pos.z = positions[1]
        
        # pygame.draw.line(screen, "red", (screen.get_width(), fibre1_pos.y), (fibre1_pos.x, fibre1_pos.y), width=1)
        # pygame.draw.line(screen, "blue", (0, fibre2_pos.y), (fibre2_pos.x, fibre2_pos.y), width=1)
                
        # text_print.tprint(screen, '')
        # text_print.tprint(screen, f"Fiber 1 : x = {fibre1_pos.x:.0f}, y = {fibre1_pos.y:.0f}, z = {fibre1_pos.z:.0f}")
        # text_print.tprint(screen, f"Fiber 2 : x = {fibre2_pos.x:.0f}, y = {fibre2_pos.y:.0f}, z = {fibre2_pos.z:.0f}")
                
        pygame.display.flip()
        
        dt = clock.tick(60) / 1000


except KeyboardInterrupt:
    print("Fin du programme")

finally:
    pygame.quit()
