# Creation of the path planner using
# python python open cv ad numpy
# The code is developed for 2D path planning
# because it is faster for raspberry to calculate the path in this way, 
# and because I consider that on the z axis the altidue must be constant during the flight
# to avoid excessive oscillations
# Jan 23
# Riccardo Tessarin
import numpy as np
from math import dist
import cv2
import math
import matplotlib.pyplot as plt

class Astar():
    def __init__(self, m): #m is the only input require while definig class, it is the map of the environment
        self.map = m
        self.initialize()

    def initialize(self): #It is necessary when I want to recalculate a new path from 2 points on the map
        self.open = []
        self.close = {}
        self.g = {} #distance from start to the node
        self.h = {} #distance from node to the goal
        node_goal = None

    # estimation
    def distance(self, a, b):
        # Diagonal distance
        dMax = np.max([np.abs(a[0]-b[0]), np.abs(a[1]-b[1])])
        dMin = np.min([np.abs(a[0]-b[0]), np.abs(a[1]-b[1])])
        d = math.sqrt(2.) * dMin + (dMax - dMin)
        return d

    def planning(self, start, goal, inter, img=None):
        self.initialize() # Initialize the calculator
        self.open.append(start) #first step of the alghorithm is put node start in open list
        self.close[start] = None
        self.g[start] = 0 # coordinate of start are at distance 0 from itself
        self.h[start] = self.distance(start, goal) # distance from start point to goal
        goal_node = None
        
        while(1): #Here start the real alghorithm
            min_distance = 9999999. #this value mast be maxed at the beginning
            min_id = -1 #not exist in reality this index
            for i, node in reversed(list(enumerate(self.open))): #mia modifica, prendo prima gli ultimi perchè sono tendenzialmente quelli con h minore
            #for i, node in enumerate(self.open): # controllo tutti i nodi in open
                # 1:(loc1, loc2) come sto chiamando con enumerate i vari nodi in open
                f = self.g[node] # Calcolo il costo in termini di distanza di tutti
                y = self.h[node] # i vicini del nodo che sto considerando

                if f+y < min_distance: # Alla fine tengo il nodo che minimizza la somma dei due valori
                    min_distance = f+y
                    min_id = i

            # Alla fine questo nodo migliore è quello che mi rimane e su cui faccio i calcoli
            p = self.open.pop(min_id)  # Tolgo quello che vado a considerare da open e tengo però le sue coordinate

            #Controllo se in p vi è un ostacolo
            if self.map[p[1], p[0]] < 0.5: #Qui il check si basa sul colore della mappa, nero = ostacolo
                continue # In questo modo il punto con ostacolo viene tolto da open e non si sviluppano i conti su di esso.

            # Controllo se sono arrivato alla destinazione
            if self.distance(p, goal) < inter:
                self.goal_node = p # Salvo p per non perderlo in goal node
                break

            # eight directions, definisco le 8 direzioni cui posso muovermi dal punto in cui sono (loc1, loc2)
            pts_next1 = [(p[0]+inter, p[1]), (p[0], p[1]+inter),
                         (p[0]-inter, p[1]), (p[0], p[1]-inter)]
            pts_next2 = [(p[0]+inter, p[1]+inter), (p[0]-inter, p[1]+inter),
                         (p[0]-inter, p[1]-inter), (p[0]+inter, p[1]-inter)]
            pts_next = pts_next1 + pts_next2

            for pn in pts_next: #Itero sui punti
                # Ci sono 2 possibilità, il punto è nuovo, oppure è già stato considerato
                # per i puntinuovi
                if pn not in self.close:
                    # metto pn in open
                    self.open.append(pn)
                    self.close[pn] = p # Associo pn al punto cui lo sto collegando
                    # Calcolo per quel punto il valore di h e g
                    self.g[pn] = self.g[p] + inter #La distanza da start che ha
                    self.h[pn] = self.distance(pn, goal) #quanto  distante ancora da goal
                
                # Se è gia in close devo controllare se la distanza g diminuisce
                elif self.g[pn] > self.g[p] + inter:
                    # In caso affermativo sostituisco con la distanza minore e cambio il parente
                    self.close[pn] = p
                    self.g[pn] = self.g[p] + inter 
                    # Non serve però ricalcolare la h, quella non cambia

            '''if img is not None:
                cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
                cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)
                cv2.circle(img, p, 2, (0, 0, 1), 1)
                img_ = cv2.flip(img, 0)
                cv2.imshow("A* Test", img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break'''


        # Extract path
        path = []
        p = self.goal_node
        while(True):
            path.insert(0, p)
            if self.close[p] == None:
                break
            p = self.close[p] # Il valore precedente in close, chiama il suo parente che diventa il punto successivo del path e ricorsivamente ricostruisco il percorso
        if path[-1] != goal:
            path.append(goal)
        return path

import timeit
smooth = True
if __name__ == "__main__":
    #img = cv2.flip(cv2.imread("C:\Volume_D\Programming\Blimp_git\Blimp\povo2_provaPathPlanning.png"), 0)
    img = cv2.flip(cv2.imread("C:\Volume_D\Programming\Blimp_git\Blimp\lab_meccatronica.png"),0)
    #flip = img[::-1,:,:] # revise height in (height, width, channel)
    img[img > 128] = 255
    img[img <= 128] = 0
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255.
    m = 1-cv2.dilate(1-m, np.ones((20, 20))) #inflate for avoid the obstacle 
    img = img.astype(float)/255.

    

    start = (150,150) ## MY case
    goal = (360, 760)

    

    a = timeit.default_timer()
    astar = Astar(m)
    path = astar.planning(start=start, goal=goal, img=img, inter=10) #100 è 1 metro nella realtà
    path_array = np.array(path)/100.0
    print(path_array) # use path[0][0] per access specific indexon x ==> [0][i], y ==> [1][i] 
    b = timeit.default_timer()
    print("Time: ", b-a)

    cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
    cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)

    path_rasp = [(150, 150), (160, 160), (160, 170), (160, 180), (160, 190), (170, 200), (180, 210), (190, 220), (200, 230), (210, 240), (220, 250), (230, 260), (240, 260), (250, 270), (260, 280), (270, 290), (280, 300), (270, 310), (260, 320), (250, 330), (240, 340), (240, 350), (240, 360), (230, 370), (220, 380), (210, 390), (200, 400), (190, 410), (190, 420), (190, 430), (200, 440), (200, 450), (190, 460), (180, 470), (180, 480), (180, 490), (180, 500), (180, 510), (180, 520), (180, 530), (180, 540), (190, 550), (200, 560), (200, 570), (210, 580), (220, 590), (230, 590), (240, 600), (250, 610), (260, 620), (270, 630), (280, 640), (290, 650), (300, 660), (310, 670), (320, 680), (330, 690), (340, 700), (350, 710), (360, 720), (360, 730), (360, 740), (360, 750), (360, 760)]


   


   
    for i in range(len(path)-1):
        cv2.line(img, path[i], path[i+1], (1,0,0,),1)
        
        #cv2.line(img, path_rasp[i], path_rasp[i+1], (0,1,0), 1)
    img_ = cv2.flip(img, 0)
    #cv2.imshow("A* Test", img_)
    plt.imshow(img_), plt.title('Path Planning in Mechatronics Lab')
    plt.ion()
    plt.show()
    plt.pause(0.1)
    #k = cv2.waitKey(0)

    l = 0
    for l in range (100):
        img = cv2.flip(cv2.imread("C:\Volume_D\Programming\Blimp_git\Blimp\lab_meccatronica.png"),0)
        #flip = img[::-1,:,:] # revise height in (height, width, channel)
        img[img > 128] = 255
        img[img <= 128] = 0
        m = np.asarray(img)
        m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
        m = m.astype(float) / 255.
        m = 1-cv2.dilate(1-m, np.ones((20, 20))) #inflate for avoid the obstacle 
        img = img.astype(float)/255.

        for i in range(len(path)-1):
            cv2.line(img, path[i], path[i+1], (1,0,0,),1)

        cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
        cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)
        
        cv2.circle(img, (path[l][0], path[l][1]), 5, (0, 1, 0), 3)

        img_ = cv2.flip(img, 0)
        plt.imshow(img_), plt.title('Path Planning in Mechatronics Lab')
        plt.show()
        plt.pause(0.1)
        l = l+1

