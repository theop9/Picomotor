# -*- coding: utf-8 -*-
"""
Created on Thu Jun  5 11:43:32 2025

@author: Alok
"""

import sys
import os
import inspect
# Import the .NET Common Language Runtime (CLR) to allow interaction with .NET
import clr
import numpy as np
import pygame
import time

print ("Python %s\n\n" % (sys.version,))

strCurrFile = os.path.abspath (inspect.stack()[0][1])
print ("Executing File = %s\n" % strCurrFile)

# Initialize the DLL folder path to where the DLLs are located
strPathDllFolder = r"C:\Program Files\New Focus\New Focus Picomotor Application\Samples"
print ("Executing Dir  = %s\n" % strPathDllFolder)

# Add the DLL folder path to the system search path (before adding references)
sys.path.append (strPathDllFolder)

# Add a reference to each .NET assembly required
clr.AddReference ("DeviceIOLib")
clr.AddReference ("CmdLib8742")

# Import a class from a namespace
from Newport.DeviceIOLib import *
from NewFocus.PicomotorApp import CmdLib8742
from System.Text import StringBuilder

print ("Waiting for device discovery...")
# Call the class constructor to create an object
deviceIO = DeviceIOLib (True)
cmdLib8742 = CmdLib8742 (deviceIO)

# Set up USB to only discover picomotors
deviceIO.SetUSBProductID (0x4000);

# Discover USB and Ethernet devices - delay 5 seconds
deviceIO.DiscoverDevices (1, 100)

# Get the list of discovered devices
strDeviceKeys = np.array ([])
strDeviceKeys = deviceIO.GetDeviceKeys ()
nDeviceCount = deviceIO.GetDeviceCount ()
#nSlaveCount = cmdLib8742.GetSlaveCount(strDeviceKeys[0])
print ("Device Count = %d\n" % nDeviceCount)
#print ("Slave Count = %d\n" % nSlaveCount)

#%%
class picomotor:
    
    def __init__(self):
        #self.name = name
        self.DeviceCount = deviceIO.GetDeviceCount ()
        self.DeviceKeys = deviceIO.GetDeviceKeys ()
        
        
    def system_info(self):
        for i in range(self.DeviceCount):
            strDeviceKey = str(self.DeviceKeys[i])
            if deviceIO.Open(strDeviceKey):
                nDeviceAddress = cmdLib8742.GetMasterDeviceAddress(strDeviceKey)
                strModelSerial = cmdLib8742.GetModelSerial(strDeviceKey, nDeviceAddress)
                nSlaveCount = cmdLib8742.GetSlaveCount(strDeviceKey)
                nDeviceAddressList = cmdLib8742.GetDeviceAddresses(strDeviceKey)
                print(f"device {i+1} : {nSlaveCount} slaves")
                print(f"address : {nDeviceAddress}, Model Serial : {strModelSerial}\n")
                for n in range(nSlaveCount):
                    nDeviceAddressSlave = nDeviceAddressList[n]
                    strModelSerialSlave = cmdLib8742.GetModelSerial(strDeviceKey, nDeviceAddressSlave)
                    print(f"slave {n+1} : ")
                    print(f"address : {nDeviceAddressSlave}, Model Serial : {strModelSerialSlave}")
                    
    def get_slave_count(self):
        #for each device
        for i in range(self.DeviceCount):
            strDeviceKey = str(self.DeviceKeys[i])
            if deviceIO.Open(strDeviceKey):
                nDeviceAddress = cmdLib8742.GetMasterDeviceAddress(strDeviceKey)
                strModelSerial = cmdLib8742.GetModelSerial(strDeviceKey, nDeviceAddress)
                nSlaveCount = cmdLib8742.GetSlaveCount(strDeviceKey)
                print(f"device {i} : {nSlaveCount} slaves")
        
    def get_position(self, addr, axis):
        strBldr = StringBuilder(64)
        for i in range(self.DeviceCount):
            strDeviceKey = str(self.DeviceKeys[i])
            if deviceIO.Open(strDeviceKey):
                nDeviceAddress = cmdLib8742.GetMasterDeviceAddress(strDeviceKey)
                strBldr.Remove(0, strBldr.Length)
                cmd = f"{addr}>{axis}TP?"
                nReturn = cmdLib8742.Query(strDeviceKey, cmd, strBldr)
                if nReturn:
                    print(strBldr.ToString())
                else:
                    print('error')
                    # errMsg = StrongBox[str]("")
                    # errNum = StrongBox[str]("")
                    # nMsg = cmdLib8742.GetErrorMsg(strDeviceKey, nDeviceAddress, errMsg)
                    # nNum = cmdLib8742.GetErrorNum(strDeviceKey, nDeviceAddress, errNum)
                    # print(nMsg, nNum)
                    # print(errNum.Value, ".", errMsg.Value)

            
    def get_all_info_0(self):
        for i in range(self.DeviceCount):
            strDeviceKey = str(self.DeviceKeys[i])
            if deviceIO.Open(strDeviceKey):
                nDeviceAddress = cmdLib8742.GetMasterDeviceAddress(strDeviceKey)
                strModelSerial = cmdLib8742.GetModelSerial(strDeviceKey, nDeviceAddress)
                print(f"Device {i+1} : {strModelSerial}    (address : {nDeviceAddress})\n")
                for nMotor in range(1, 4):
                    nPosition = 0
                    cmdLib8742.GetPosition(strDeviceKey, nDeviceAddress, nMotor, nPosition)
                    stepsPerSec = 0
                    cmdLib8742.GetVelocity(strDeviceKey, nDeviceAddress, nMotor, stepsPerSec)
                    stepsPerSec2 = 0
                    cmdLib8742.GetAcceleration(strDeviceKey, nDeviceAddress, nMotor, stepsPerSec2)
                    print(f" moteur {nMotor} : position {nPosition} \t vitesse : {stepsPerSec} \t acceleration : {stepsPerSec2}")
                    
                
                nSlaveCount = cmdLib8742.GetSlaveCount(strDeviceKey)
                nDeviceAddressList = cmdLib8742.GetDeviceAddresses(strDeviceKey)
                for n in range(nSlaveCount):
                    nDeviceAddressSlave = nDeviceAddressList[n]
                    strModelSerialSlave = cmdLib8742.GetModelSerial(strDeviceKey, nDeviceAddressSlave)
                    print(f"\nSlave {n+1} : {strModelSerialSlave}    (address : {nDeviceAddressSlave})\n")
                    for nMotor in range(1, 4):
                        nPosition = 0
                        cmdLib8742.GetPosition(strDeviceKey, nDeviceAddressSlave, nMotor, nPosition)
                        stepsPerSec = 0
                        cmdLib8742.GetVelocity(strDeviceKey, nDeviceAddressSlave, nMotor, stepsPerSec)
                        stepsPerSec2 = 0
                        cmdLib8742.GetAcceleration(strDeviceKey, nDeviceAddressSlave, nMotor, stepsPerSec2)
                        print(f" moteur {nMotor} : position {nPosition} \t vitesse : {stepsPerSec} \t acceleration : {stepsPerSec2}")
    
    def get_all_info(self):
        strBldr = StringBuilder(64)
        for i in range(self.DeviceCount):
            strDeviceKey = str(self.DeviceKeys[i])
            if deviceIO.Open(strDeviceKey):
                nDeviceAddress = cmdLib8742.GetMasterDeviceAddress(strDeviceKey)
                strModelSerial = cmdLib8742.GetModelSerial(strDeviceKey, nDeviceAddress)
                print(f"Device {i+1} : {strModelSerial}    (address : {nDeviceAddress})\n")
                for axis in range(1, 4):
                    cmd = f"{nDeviceAddress}>{axis}TP?;{nDeviceAddress}>{axis}VA?;{nDeviceAddress}>{axis}AC?"
                    strBldr.Remove(0, strBldr.Length)
                    nReturn = cmdLib8742.Query(strDeviceKey, cmd, strBldr)
                    if nReturn:
                        print(strBldr.ToString())
                    else:
                        print('error')
                    
                
                nSlaveCount = cmdLib8742.GetSlaveCount(strDeviceKey)
                nDeviceAddressList = cmdLib8742.GetDeviceAddresses(strDeviceKey)
                for n in range(nSlaveCount):
                    nDeviceAddressSlave = nDeviceAddressList[n]
                    strModelSerialSlave = cmdLib8742.GetModelSerial(strDeviceKey, nDeviceAddressSlave)
                    print(f"\nSlave {n+1} : {strModelSerialSlave}    (address : {nDeviceAddressSlave})\n")
                    for axis in range(1, 4):
                        cmd = f"{nDeviceAddressSlave}>{axis}TP?;{nDeviceAddressSlave}>{axis}VA?;{nDeviceAddressSlave}>{axis}AC?"
                        strBldr.Remove(0, strBldr.Length)
                        nReturn = cmdLib8742.Query(strDeviceKey, cmd, strBldr)
                        if nReturn:
                            print(strBldr.ToString())
                        else:
                            print('error')
    
    
    
    
    def step(self, addr, axis, direction):
        strDeviceKey = self.DeviceKeys[0]
        if direction=='+':
            nSteps = 1
        elif direction=='-':
            nSteps = -1
        else:
            nSteps = None
        print(f"{nSteps} steps effectués par le moteur {axis} du controlleur {addr}")
        nReturn = cmdLib8742.RelativeMove(strDeviceKey, addr, axis, nSteps)
        
    def steps(self, addr, axis, nSteps) :
        strDeviceKey = self.DeviceKeys[0]
        nReturn = cmdLib8742.RelativeMove(strDeviceKey, addr, axis, nSteps)

    def abort_motion(self, addr):
        strDeviceKey = self.DeviceKeys[0]
        nReturn = cmdLib8742.AbortMotion(strDeviceKey, addr)
        
    def stop_motion(self, addr, axis):
        strDeviceKey = self.DeviceKeys[0]
        nReturn = cmdLib8742.StopMotion(strDeviceKey, addr, axis)
        
    def shutdown(self):
        cmdLib8742.Shutdown ()
        deviceIO.Shutdown ()
        
    def query(self, cmd):
        strDeviceKey = self.DeviceKeys[0]
        strBldr = StringBuilder(64)
        strBldr.Remove (0, strBldr.Length)
        nReturn = deviceIO.Query(strDeviceKey, cmd, strBldr)
        #print(nReturn)
        if nReturn:
            print(strBldr.ToString())
        else:
            print('error')
            
    def get_all_positions(self):
        strDeviceKey = self.DeviceKeys[0]
        positions = np.zeros((2,3))
        for addr in [1,2]:
            for axis in [1,2,3]:
                refPosition = 0
                nReturn, position = cmdLib8742.GetPosition(strDeviceKey, addr, axis, refPosition)
                if nReturn:
                    positions[addr-1, axis-1] = position
        return positions
    
    def get_position2(self, addr, axis):
        strDeviceKey = self.DeviceKeys[0]
        refPosition = 0
        nReturn, position = cmdLib8742.GetPosition(strDeviceKey, addr, axis, refPosition)
        if nReturn:
            return position
        else:
            print(f"erreur : nReturn={nReturn} & position={position}")
            
    def set_all_velocity(self, stepsPerSec):
        strDeviceKey = self.DeviceKeys[0]
        for addr in [1,2]:
            for axis in [1,2,3]:
                nReturn = cmdLib8742.SetVelocity(strDeviceKey, addr, axis, stepsPerSec)
                
    def go_home(self):
        strDeviceKey = self.DeviceKeys[0]
        for addr in [1,2]:
            for axis in [1,2,3]:
                nReturn = cmdLib8742.MoveToHome(strDeviceKey, addr, axis)
    
    
    # def query(self, key, cmd):
    # sb = StringBuilder(64)
    # ret = self.deviceIO.Query(key, cmd, sb)
    # result = sb.ToString().strip() if ret else None
    # return result

#%%

o = picomotor()
o.system_info()


o.query('1>3MV+')
o.query('1>3ST')

o.get_position2(1,3)
o.get_all_positions()

o.set_all_velocity(1001)

o.query('1>1DH?')
o.query('1>2DH?')
o.query('1>3DH?')
o.query('2>1DH?')
o.query('2>2DH?')
o.query('2>3DH?')

o.go_home()

#o.shutdown()
t1 = time.time()
cmdLib8742.JogPositive(strDeviceKey, 1, 1)
t2 = time.time()
cmdLib8742.StopMotion(strDeviceKey, 1, 1)
t3 = time.time()
print(t3-t1)
print(t3-t2)
print(t2-t1)

#%%



pygame.init()
pygame.joystick.init()
screen = pygame.display.set_mode((600, 600))
center = (screen.get_width()/2, screen.get_height()/2)
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
        self.x += 20

    def unindent(self):
        self.x -= 20

text_print = TextPrint()

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

def coord_to_screen(pos):
    return (pos[0] + center[0], -pos[1] + center[1])

def positionOnScreen(pos, pos_G):
    return(screen.get_width()/2 + alpha*(pos[1] - pos_G[1]), screen.get_height()/2 + alpha*(pos[0] - pos_G[0]))

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
isModeChangedLastFrame = False
print(f"Mode choisi par défault : {actualMode}")

# Initialisation du mode de vitesse
slow_velocity = 100 #steps/s
medium_velocity = 500 #steps/s
fast_velocity = 1000 #steps/s
velocity = np.array([slow_velocity, medium_velocity, fast_velocity])
ModeVitesse = np.array(['slow', 'medium', 'fast'])

indexModeVitesse = 0
o.set_all_velocity(int(velocity[indexModeVitesse]))
actualModeVitesse = ModeVitesse[indexModeVitesse]
isModeVitesseChangedLastFrame = False
print(f"Mode de vitesse choisi par défaut : {actualModeVitesse}")


# Initialisation du mouvement des moteurs (immobiles au démarrage)
isMoving = np.array([[False, False, False], # right : x, y, z
                     [False, False, False]]) # left : x, y, z

# Initialisation du mode de calibration coordonnées x,y des coins du sample
toggleCalibration = False
isCalibrationChangedLastFrame = False
Calibration_Step1 = False
Calibration_Step2 = False
corners = [] 
isCorner = [ 'No', 'No', 'No', 'No']


fibre1_pos = pygame.Vector3(0, 0, 0)
fibre2_pos = pygame.Vector3(0, 0, 0)



# Main code
try:
    while running:
        
        # Affichage des modes de déplacement et de vitesse
        screen.fill((255, 255, 255))
        text_print.reset()
        text_print.tprint(screen, f"CALIBRATION : {toggleCalibration}")
        text_print.indent()
        text_print.tprint(screen, f"STEP 1 : {Calibration_Step1}")
        text_print.indent()
        text_print.tprint(screen, f"corners : {isCorner[0]}, {isCorner[1]}, {isCorner[2]}, {isCorner[3]}")
        text_print.unindent()
        text_print.tprint(screen, f"STEP 2 : {Calibration_Step2}")
        text_print.unindent()
        text_print.tprint(screen, f"Mode : {actualMode}")
        text_print.tprint(screen, f"Velocity : {actualModeVitesse} ({velocity[indexModeVitesse]} steps/sec)")
        
        # Affichage des home positions
        # pygame.draw.line(screen, "red", (home_1_x - 2, home_1_y - 2), ((home_1_x + 2, home_1_y + 2)), width=1)
        # pygame.draw.line(screen, "red", (home_1_x - 2, home_1_y + 2), ((home_1_x + 2, home_1_y - 2)), width=1)
        # pygame.draw.line(screen, "blue", (home_2_x - 2, home_2_y - 2), ((home_2_x + 2, home_2_y + 2)), width=1)
        # pygame.draw.line(screen, "blue", (home_2_x - 2, home_2_y + 2), ((home_2_x + 2, home_2_y - 2)), width=1)
        
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
        if btnMode and not isModeChangedLastFrame:
            indexMode += 1
            indexMode = indexMode%len(Mode)
            actualMode = Mode[indexMode]
            print(f"Déplacement : {actualMode}")
        isModeChangedLastFrame = btnMode
                
        # Changement de mode de vitesse   
        btnModeVitesse = joystick.get_button(7)             
        if btnModeVitesse and not isModeVitesseChangedLastFrame:
            indexModeVitesse += 1
            indexModeVitesse = indexModeVitesse%len(ModeVitesse)
            actualModeVitesse = ModeVitesse[indexModeVitesse]
            o.set_all_velocity(int(velocity[indexModeVitesse]))
            print(f"Vitesse : {actualModeVitesse}")
            # print(cmdModeVitesse[indexModeVitesse])
        isModeVitesseChangedLastFrame = btnModeVitesse
                 
        
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

        # Tableau répertoriant pour chaque axe (colonne) comment on souhaite le modifier
        # /!\ les axes du moteur sont différents des axes usuels : on a X, Y, Z = y, +/-x, z
        buttons = np.array([[-right_joystick_y, -right_joystick_x, right_joystick_z], # y avant le x car l'horizontale (en regardant du haut) correspond à l'axe y du moteur
                            [-left_joystick_y, left_joystick_x, left_joystick_z]])
        
        isTouched_list = np.array([[(np.abs(right_joystick_y) > 0.1), (np.abs(right_joystick_x) > 0.1), right_joystick_z_pos != 0 or right_joystick_z_neg != -1],
                                   [(np.abs(left_joystick_y) > 0.1), (np.abs(left_joystick_x) > 0.1), left_joystick_z_pos != 0 or left_joystick_z_neg != -1]])
        

                        
        direction = get_direction(buttons)
        
        # Mode création de la map du sample
        btnCalibration = joystick.get_button(10)
        if btnCalibration and not isCalibrationChangedLastFrame:
            toggleCalibration = not toggleCalibration
        isCalibrationChangedLastFrame = btnCalibration
                
                
        if toggleCalibration:
            # Step 1 : pointage des coins de l'échantillon avec la fibre de gauche
            if not Calibration_Step1:
                cmd = ''
                for axis in [1,2,3]:
                    isTouched = isTouched_list[1, axis-1]
                    if isTouched:
                        # Commande Picomotor
                        if not isMoving[1, axis-1]:
                            cmd += f"2>{axis}MV{direction[1, axis-1]};"
                            isMoving[1, axis-1] = True
                            
                    else:
                        if isMoving[1, axis-1]:
                            cmd += f"2>{axis}ST;"
                            isMoving[1, axis-1] = False
                                
        
                if cmd !='':
                    cmd = cmd[:-1]
                    print(cmd)
                    o.query(cmd)
                btnConfirm = joystick.get_button(1)
                position_corner = o.get_all_positions()[1] # on pointe avec la fibre de gauche
                
                if (len(corners) == 0 and btnConfirm == 1) or (btnConfirm == 1 and len(corners) < 4 and not any(np.array_equal(position_corner, c) for c in corners)):
                    corners.append(position_corner)
                    isCorner[len(corners)-1] = 'Ok'
                    print(f"point ajouté : position = {position_corner}")  
                    
            # Step 2 : Coordination avec la fibre gauche
            elif Calibration_Step1 and not Calibration_Step2:
                cmd = ''
                for axis in [1,2,3]:
                    isTouched = isTouched_list[0, axis-1]
                    if isTouched:
                        # Commande Picomotor
                        if not isMoving[0, axis-1]:
                            cmd += f"1>{axis}MV{direction[0, axis-1]};"
                            isMoving[0, axis-1] = True
                            
                    else:
                        if isMoving[0, axis-1]:
                            cmd += f"1>{axis}ST;"
                            isMoving[0, axis-1] = False
                if cmd !='':
                    cmd = cmd[:-1]
                    print(cmd)
                    o.query(cmd)

                btnConfirm = joystick.get_button(1)

                if btnConfirm == 1:
                    position_pointage = o.get_all_positions()[0] # fibre de droite cette fois ci
                    Calibration_Step2 = True
                
                
        else:
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
                    t1 = time.time()
                    o.query(cmd)
                    t2 = time.time()
                    print(t2-t1)
    
                    
            # Mode pincettes
            elif indexMode == 1:
                cmd = ''
                for axis in [1,2,3]:
                    
                    # Déplacement en bloc : joystick droit
                    isTouchedRight = isTouched_list[0, axis-1] 
                    if isTouchedRight:
                        if not isMoving[0, axis-1]:
                            if axis == 2:
                                print('axis = 2')
                                if direction[0,1] == '+':
                                    print('+')
                                    cmd += "1>2MV+;2>2MV-;"
                                elif direction[0,1] == '-':
                                    print('-')
                                    cmd += "1>2MV-;2>2MV+;"
                            else:
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
                            if axis == 2:
                                if direction[1, 1] == '+':
                                    cmd += "1>2MV+;2>2MV+;"
                                elif direction[1, 1] == '-':
                                    cmd += "1>2MV-;2>2MV-;"
                            else:                            
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
                    o.query(cmd)
                    
        CalibrationDone = Calibration_Step1 and Calibration_Step2
                
        
        if len(corners) == 4 and not Calibration_Step1 :
            corners = np.array(corners)
            # maxLength_Steps = max([max(corners[:,0]), max(corners[:,1])]) - min([min(corners[:,0]), min(corners[:,1])]) 
            maxHeight_Steps = max(corners[:,1]) - min(corners[:,1])
            maxWidth_Steps = max(corners[:,0]) - min(corners[:,0])
            maxLength_Steps = max([maxHeight_Steps, maxWidth_Steps])
            maxHeight_Screen = 3*screen.get_height()/4
            alpha = maxHeight_Screen/maxLength_Steps #on prend max height car c'est un carré donc peut importe la dimension qu'on prend
            corners_screen = []
            pos_G = (sum(corners[:,0])/4, sum(corners[:,1])/4)
            print(alpha)
            print(pos_G)
            for position_corner in corners:
                print(position_corner[:-1])
                corners_screen.append(positionOnScreen(position_corner[:-1], pos_G))
            Calibration_Step1 = True
            time.sleep(0.5)
            print(corners_screen)
            
        
                
        elif Calibration_Step1 and not CalibrationDone:
            pygame.draw.polygon(screen, "grey", corners_screen)
            
        elif CalibrationDone:
            pygame.draw.polygon(screen, "grey", corners_screen)
            positions = o.get_all_positions()
            fibre1_pos.x, fibre1_pos.y = positionOnScreen(positions[0,:-1] - position_pointage[:-1] + corners[-1,:-1], pos_G)
            fibre2_pos.x, fibre2_pos.y = positionOnScreen(positions[1,:-1], pos_G)
            fibre1_pos.z, fibre2_pos.z = positions[:,2]
            # fibre1_pos.y, fibre1_pos.x, fibre1_pos.z = positions[0]
            # fibre2_pos.y, fibre2_pos.x, fibre2_pos.z = positions[1]
            
            # pygame.draw.line(screen, "red", (screen.get_width(), fibre1_pos.y/10 +center[1]), (-fibre1_pos.x/10 +center[0], fibre1_pos.y/10 +center[1]), width=1)
            # pygame.draw.line(screen, "blue", (0, fibre2_pos.y/10 +center[1]), (fibre2_pos.x/10, fibre2_pos.y/10 +center[1]), width=1)
            
            pygame.draw.line(screen, "red", (screen.get_width(), fibre1_pos.y), (fibre1_pos.x, fibre1_pos.y), width=1)
            pygame.draw.line(screen, "blue", (0, fibre2_pos.y), (fibre2_pos.x, fibre2_pos.y), width=1)
            
            print(fibre2_pos.x, fibre2_pos.y)
            print(fibre1_pos.x, fibre1_pos.y)
                    
            text_print.tprint(screen, '')
            # text_print.tprint(screen, f"Motor 1 : x (axis 2) = {fibre1_pos.x:.0f}, y (axis 1) = {fibre1_pos.y:.0f}, z (axis 3) = {fibre1_pos.z:.0f}")
            # text_print.tprint(screen, f"Fiber 2 : x (axis 2) = {fibre2_pos.x:.0f}, y (axis 1) = {fibre2_pos.y:.0f}, z (axis 3) = {fibre2_pos.z:.0f}")
            
            text_print.tprint(screen, f"Motor 1 : x (axis 2) = {positions[0,1]:.0f}, y (axis 1) = {positions[0,0]:.0f}, z (axis 3) = {fibre1_pos.z:.0f}")
            text_print.tprint(screen, f"Fiber 2 : x (axis 2) = {positions[1,1]:.0f}, y (axis 1) = {positions[1,0]:.0f}, z (axis 3) = {fibre2_pos.z:.0f}")
        
        #print(CalibrationDone)
        pygame.display.flip()
        
        dt = clock.tick(60) / 1000



except KeyboardInterrupt:
    print("Fin du programme")

finally:
    pygame.quit()
    
