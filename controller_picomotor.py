import sys
import os
import inspect
# Import the .NET Common Language Runtime (CLR) to allow interaction with .NET
import clr
import numpy as np
import time
import pygame

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
        nReturn = deviceIO.Query(strDeviceKey, cmd, strBldr)
        if nReturn:
            print(strBldr.ToString())
        else:
            print('error')
    
    
    # def query(self, key, cmd):
    # sb = StringBuilder(64)
    # ret = self.deviceIO.Query(key, cmd, sb)
    # result = sb.ToString().strip() if ret else None
    # return result

#%%

o = picomotor()
o.system_info()
o.query('2>3TP?')


o.query('1>3MV+')
o.query('1>3ST')
1>3ST




#o.shutdown()

#%%



pygame.init()
pygame.joystick.init()
screen = pygame.display.set_mode((400, 400))
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

# Définition des modes de vitesses 
slow_velocity = 200 #steps/s
medium_velocity = 1000 #steps/s
fast_velocity = 2000 #steps/s
velocity = np.array([slow_velocity, medium_velocity, fast_velocity])

# Initialisation du mode de déplacement
indexMode = 0
Mode = np.array(['mode1 (fibres indépendentes)', 'mode2 (pincettes)'])
actualMode = Mode[indexMode]
isModeChanged = False
print(f"Mode choisi par défault : {actualMode}")

# Initialisation du mode de vitesse
indexModeVitesse = 0
ModeVitesse = np.array(['mode1 (slow)', 'mode2 (medium)', 'mode3 (fast)'])
cmdModeVitesse = np.array([f'1>1VA{slow_velocity};1>2VA{slow_velocity};1>3VA{slow_velocity};2>1VA{slow_velocity};2>2VA{slow_velocity};2>3VA{slow_velocity}', 
                    f'1>1VA{medium_velocity};1>2VA{medium_velocity};1>3VA{medium_velocity};2>1VA{medium_velocity};2>2VA{medium_velocity};2>3VA{medium_velocity}', 
                    f'1>1VA{fast_velocity};1>2VA{fast_velocity};1>3VA{fast_velocity};2>1VA{fast_velocity};2>2VA{fast_velocity};2>3VA{fast_velocity}'])
actualModeVitesse = ModeVitesse[indexModeVitesse]
isModeVitesseChanged = False
print(f"Mode de vitesse choisi par défaut : {actualModeVitesse}")

# Initialisation du mouvement des moteurs (immobiles au démarrage)
isMoving = np.array([[False, False, False], # right : x, y, z
                     [False, False, False]]) # left : x, y, z


fibre1_pos = pygame.Vector2(2*screen.get_width() / 3, screen.get_height() / 2)
fibre2_pos = pygame.Vector2(screen.get_width() / 3, screen.get_height() / 2)

# Main code
try:
    while running:
        
        # Affichage des modes de déplacement et de vitesse
        screen.fill((255, 255, 255))
        text_print.reset()
        text_print.tprint(screen, "Mode de déplacement:")
        text_print.indent()
        text_print.tprint(screen, actualMode)
        text_print.unindent()
        text_print.tprint(screen, "Mode de vitesse:")
        text_print.indent()
        text_print.tprint(screen, actualModeVitesse)
        text_print.unindent()
        
        pygame.draw.circle(screen, "red", fibre1_pos, 4)
        pygame.draw.circle(screen, "blue", fibre2_pos, 4)
        
        # Manières d'arreter le "jeu" : fermer la fenêtre, appuyer sur espace, interrompre le noyau
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    print("Espace pressée, arrêt du programme.")
                    running = False
        
        # Changement de mode de déplacement
        btnMode = joystick.get_button(6)
        
        if btnMode == 1:
            if not isModeChanged:
                indexMode += 1
                indexMode = indexMode%2
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
                indexModeVitesse = indexModeVitesse%3
                actualModeVitesse = ModeVitesse[indexModeVitesse]
                print(f"Vitesse : {actualModeVitesse}")
                print(cmdModeVitesse[indexModeVitesse])
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
                        
        direction = get_direction(buttons)
        
        # Mode indépendant 
        if indexMode == 0:              
            cmd = ''
            for addr in [1,2]:
                for axis in [1,2,3]:
                    isTouched = isTouched_list[addr-1, axis-1]
                    if isTouched:
                        # Mouvement visuel
                        if addr==1:
                            if axis == 1:
                                if direction[addr-1, axis-1] == '+':
                                    fibre1_pos.x += velocity[indexModeVitesse]*dt/10
                                elif direction[addr-1, axis-1] == '-':
                                    fibre1_pos.x -= velocity[indexModeVitesse]*dt/10
                            elif axis == 2:
                                if direction[addr-1, axis-1] == '+':
                                    fibre1_pos.y -= velocity[indexModeVitesse]*dt/10
                                elif direction[addr-1, axis-1] == '-':
                                    fibre1_pos.y += velocity[indexModeVitesse]*dt/10
                        elif addr==2:
                            if axis == 1:
                                if direction[addr-1, axis-1] == '+':
                                    fibre2_pos.x += velocity[indexModeVitesse]*dt/10
                                elif direction[addr-1, axis-1] == '-':
                                    fibre2_pos.x -= velocity[indexModeVitesse]*dt/10
                            elif axis == 2:
                                if direction[addr-1, axis-1] == '+':
                                    fibre2_pos.y -= velocity[indexModeVitesse]*dt/10
                                elif direction[addr-1, axis-1] == '-':
                                    fibre2_pos.y += velocity[indexModeVitesse]*dt/10
                                    
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
                o.query(cmd)
                
        # Mode pincettes
        elif indexMode == 1:
            cmd = ''
            for axis in [1,2,3]:
                
                # Déplacement en bloc : joystick droit
                isTouchedRight = isTouched_list[0, axis-1] 
                if isTouchedRight:
                    # Mouvement visuel
                    if axis == 1:
                        if direction[0,0] == '+':
                            fibre1_pos.x += velocity[indexModeVitesse]*dt/10
                            fibre2_pos.x += velocity[indexModeVitesse]*dt/10
                        elif direction[0,0] == '-':
                            fibre1_pos.x -= velocity[indexModeVitesse]*dt/10
                            fibre2_pos.x -= velocity[indexModeVitesse]*dt/10
                    elif axis == 2:
                        if direction[0,1] == '+':
                            fibre1_pos.y -= velocity[indexModeVitesse]*dt/10
                            fibre2_pos.y -= velocity[indexModeVitesse]*dt/10
                        elif direction[0,1] == '-':
                            fibre1_pos.y += velocity[indexModeVitesse]*dt/10
                            fibre2_pos.y += velocity[indexModeVitesse]*dt/10

                                
                    # Commande Picomotor
                    if not isMoving[0, axis-1]:
                        cmd += f"1>{axis}MV{direction[0, axis-1]};2>{axis}MV{direction[0, axis-1]};"
                        isMoving[0, axis-1] = True
                else:
                    if isMoving[0, axis-1]:
                        cmd += f"1>{axis}ST;2>{axis}ST;"
                        isMoving[0, axis-1] = False
                        
                # Déplacement opposé (pour pincer) : joystick gauche
                isTouchedLeft = isTouched_list[1, axis-1] 
                if isTouchedLeft:
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
                
        pygame.display.flip()
        
        dt = clock.tick(60) / 1000


except KeyboardInterrupt:
    print("Fin du programme")

finally:
    pygame.quit()