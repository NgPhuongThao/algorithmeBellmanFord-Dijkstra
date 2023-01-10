# -*- coding: utf-8 -*-
"""
Created on Mon Jun 13 09:21:19 2022

@author: lucil
"""

import numpy.random as rd
from random import randrange
import random
import numpy as np
import math
import time
from matplotlib.pyplot import *

##################################################################################################################
################################################ Exercice 3 ######################################################
##################################################################################################################

print("#######################################################################")
print("######################### Exercice 3 ##################################")
print("#######################################################################\n")

def MatriceAleatoire(n,a,b): 
    '''fonction prenant en entrée 3 paramètres :
      - n est la taille de la matrice
      - a et b sont les poids du coefficient représentant 50%
    La fonction génère aléatoirement une matrice de taille n×n présentant
    50% de coefficients ∞ et 50% de coefficients avec des poids compris entre a et b.
    Pour cela on crée une matrice de 0 et de 1. Ensuite on multiplie partout par un nombre aléatoire compris entre a et b
    Ce qui va xmultipliermultiplier que les endroits ou il y a des 1.
    '''
    M=rd.randint(0,2,(n,n)) # crée une matrice de 0 et de 1 de taille n*n. il y a 50% de 0 et 50% de 1
    M = M.astype(float)
    for i in range (0,n): # parcourt la matrice en long
        for j in range (0,n): #parcourt la matrice en large
            if M[i,j]==0:
                M[i,j]=float('inf') #initialise tous les poids 0 à l'infini
            else:
                M[i,j]=randrange(a,b+1)
                #initialise la valeur de la matrice(0 ou 1) avec un nombre aléatoire entre a et b 
    #(on note b+1 car python prend les bornes inférieures). Donc si il y avait un 0, on multiplie 
    #ce qui donne toujours 0. Si c'est un 1, on obtient donc un nombre aléatoire entre a et b.
    return M

print("Exemple 1: on veut une matrice de taille 4*4 avec 50% de valeurs entre 1 et 10:\n",MatriceAleatoire(4,1,10),'\n') 
print("Exemple 2: on veut une matrice de taille 10*10 avec 50% de valeurs entre 10 et 100:\n",MatriceAleatoire(10,10,100),'\n')

##################################################################################################################
################################################ Exercice 4 ######################################################
##################################################################################################################

print("#######################################################################")
print("######################### Exercice 4 ##################################")
print("#######################################################################\n")

def Dijkstra(M, s0):
    """
    Algorithme de Dijkstra
    Prend en entrée la matrice d'un graphe pondéré à poids positifs (M) et un sommet (s0)
    Retourne pour chacun des sommets la longueur et l'itinéraire du plus court chemin de s0 à s
    La mention 'Sommet t non joignable à s0 par un chemin dans le graphe G'
    """
    #Initialisation
    sommets = {s0 : (0,s0)} #initialise la distance de s0 à 0 et son prédécesseur à s0    
    n = np.shape(M)[0] #récupère la largeur de la matrice
    for i in range(0,n): #initialise la distance et le prédécesseur de chaque sommet différent de s0
        if i != s0 : 
            if M[s0][i]<math.inf: #si le sommet i est un successeur de s0 (donc si leur poids n'est pas infini)
                sommets[i] = (M[s0][i],s0) #initialise la distance au poids entre s0 et i et le prédécesseur de i à s0
            else:
                sommets[i] = (float('inf'), None) #sinon initiliase la distance à infini et aucun prédécesseur
    A = [s0] #Initialise la liste A de sommets rencontrésà s0
    
    #Itération
    listeValeurs = [i[0] for i in list(sommets.values())] #Récupère les distances de chaque sommet
    maj = True #Initialise la mise à jour à Vrai
    while (A != [] or math.inf in listeValeurs) and maj : #Tant que la liste A est remplie et qu'on fait des maj
        min = float('inf') #Initialise le minimum à infini
        s=s0 #Initialise le sommet par défaut à s0
        for i in sommets.keys(): #Pour tous les sommets
            if sommets[i][0] < min and i not in A: #Si la distance du sommet i est inférieur au min trouvé et qu'il n'est pas dans A
                min = sommets[i][0] #Initialise le minimum à sa distance
                s = i #et le sommet s à i
    
        if s not in A: #Si s n'est pas déjà dans A
            A.append(s) #Ajoute s à A
        else:
            maj= False #Sinon il n'y a pas de màj et on sortira de la boucle
            
        for t in range(0,n): #Pour tous les sommets t successeurs de s
            if t not in A:  #Si t n'est pas dans A
                if sommets[s][0] + M[s][t] < sommets[t][0]: #modifier les variables en t par la règle 
                    sommets[t] = (sommets[s][0]+M[s][t],s)
        listeValeurs = [i[0] for i in list(sommets.values())] #Mise à jour des valeurs dans le dictionnaire
        
    #Affichage
    print("--- RESULTAT DE DIJKSTRA ---")
    for i in sommets.keys(): #Pour tous les sommets de la matrice
        if i != s0: #différent de s0
            if sommets[i][0] == float('inf'): #Si la distance de i est infinie alors, i n'est pas joignable
                print("Sommet ", i, "non joignable à ",s0," par un chemin dans le graphe G.")
            else: #Sinon affiche le plus court chemin de s0 à i et sa longueur
                longueurCourtChemin = 0
                courtChemin = [i]
                s = i
                while s != s0:
                    longueurCourtChemin += sommets[s][0] #la longueur du plus court chemin est la somme des distances entre chaque sommet rencontré
                    s = sommets[s][1] #récupère le successeur de s
                    courtChemin.append(s) #ajoute s au plus court chemin
                courtChemin.reverse() #inverse le chemin pour retomber sur s0 à s et non s à s0
                print("Plus court chemin de ",s0," à ",i," : ",courtChemin," de longueur ",longueurCourtChemin)
    #renvoie le dictionnaire des poids
    return sommets

M = MatriceAleatoire(8,1,10)
print(Dijkstra(M,0))

##################################################################################################################
################################################ Exercice 5 ######################################################
##################################################################################################################

print("#######################################################################")
print("######################### Exercice 5 ##################################")
print("#######################################################################\n")

def BellmanFord(M, s0):
    """
    Algorithme de Bellman-Ford
    Prend en entrée la matrice d'un graphe pondéré à poids relatifs (M) et un sommet (s0)
    Retourne pour chacun des sommets la longueur et l'itinéraire du plus court chemin de s0 à s
    La mention 'Sommet t non joignable à s0 par un chemin dans le graphe G'
    Si le nombre de tours est égal à la largeur de la matrice M alors affiche :
        "Pas de plus court chemin : présence d'un cycle négatif"
    """
    #Initialisation
    n = np.shape(M)[0] #Récupère la largeur de la matrice
    listeDistance = {} #Initialise la liste des distances et des prédécesseurs des sommets
    listeFleches = [] #Initialise la liste des flèches
    for i in range(n):
        for j in range(n):
            if M[i][j]<float("inf"):
                listeFleches.append((i,j)) #On récupère toutes les flèches du graphe

    #Initialise le dictionnaire des sommets avec la distance et le prédecesseur des sommets
    for t in range(n):
        if (t!=s0):
            if M[s0][t]<float('inf'):
                listeDistance[t]=[M[s0][t],s0] #Si le sommet est un successeur de s0, on l'initialise au poids entre s0 et le sommet et le prédécesseur à S0
            else:
                listeDistance[t]=[float('inf'),None] #Sinon, il est initialisé à infini sans prédécesseur
        else:
            listeDistance[s0]=[0,s0] #initialise la distance de s0 à 0 et son prédécesseur à lui-même

    tours = 0 #Initialise les tours à 0
    maj = True #Initialise la maj à vrai pour rentrer dans la boucle
    while maj and tours <= n-1: #Tant qu'il y a des mises à jours et que le nombre de tours ne dépasse pas la largeur de la matrice
        maj = False #Il n'y a pas eu de mise à jour
        for i in listeFleches: #Pour toutes les flèches
            if listeDistance[i[0]][0] + M[i[0]][i[1]] < listeDistance[i[1]][0]:
                listeDistance[i[1]][0] = listeDistance[i[0]][0] + M[i[0]][i[1]]
                listeDistance[abs(i[1])][1] = i[0]
                maj = True #Il y a eu une mise à jour
        tours += 1 #Un tour est passé
        
    print("--- RESULTAT DE BELLMAN-FORD ---")
    print("Nombre de tours : ",tours)
    if tours >= n : #Si le nombre de tours est égale ou dépasse la largeur de la matrice
        print("Pas de plus court chemin : présence d'un cycle négatif") #Alors elle possède un cycle négatif et il n'existe aucun plus court chemin
    else:
        for i in range(n):
            if i != s0:
                if listeDistance[i][0] == float('inf'): #Sinon si la distance de i est infini on renvoie :
                    print("Sommet ", i, "non joignable à ",s0," par un chemin dans le graphe G.")
                else:
                    courtChemin = [i] #Initialise la liste du plus court chemin au sommet courant i
                    longueurCourtChemin = 0.0 #Initialise la longueur à 0
                    s = i #Récupère le sommet courant 
                    while s != s0: #Tant que le sommet de parcours n'est pas s0
                        longueurCourtChemin += listeDistance[s][0] #On somme la distance de chaque sommet pour obtenir le total
                        s = listeDistance[s][1] #Récupère le sommet précédant de s
                        courtChemin.append(s) #Rajoute le prédécesseur au plus court chemin
                    courtChemin.reverse() #Inverse le plus court chemin pour avoir s0 à s 
                    print("Plus court chemin de ",s0," à ",i," : ",courtChemin," de longueur ",longueurCourtChemin)
    return listeDistance
        
M = MatriceAleatoire(8,1,10)
print(BellmanFord(M,0),'\n')

M = [[float('inf'),-2,float('inf'),float('inf')],
     [float('inf'),float('inf'),1,3],
     [-2,float('inf'),float('inf'),-4],
     [float('inf'),float('inf'),float('inf'),float('inf')]]
print(BellmanFord(M,0))

##################################################################################################################
################################################ Exercice 6 ######################################################
##################################################################################################################

#Parcours aléatoire
def bellmanFordAleatoire(M, s0):
   """
    Algorithme de Bellman-Ford
    Prend en entrée la matrice d'un graphe pondéré à poids relatifs (M) et un sommet (s0)
    Retourne pour chacun des sommets la longueur et l'itinéraire du plus court chemin de s0 à s
    La mention 'Sommet t non joignable à s0 par un chemin dans le graphe G'
    Si le nombre de tours est égal à la largeur de la matrice M alors affiche :
        "Pas de plus court chemin : présence d'un cycle négatif"
    """
    #Initialisation
   n = np.shape(M)[0] #Récupère la largeur de la matrice
   listeDistance = {} #Initialise la liste des distances et des prédécesseurs des sommets
   listeFleches = [] #Initialise la liste des flèches
   for i in range(n):
       for j in range(n):
            if M[i][j]<float("inf"):
                listeFleches.append((i,j)) #On récupère toutes les flèches du graphe
   random.shuffle(listeFleches) #Mélange la liste des flèches
    
    #Initialise le dictionnaire des sommets avec la distance et le prédecesseur des sommets
   for t in range(n):
        if (t!=s0):
            if M[s0][t]<float('inf'):
                listeDistance[t]=[M[s0][t],s0] #Si le sommet est un successeur de s0, on l'initialise au poids entre s0 et le sommet et le prédécesseur à S0
            else:
                listeDistance[t]=[float('inf'),None] #Sinon, il est initialisé à infini sans prédécesseur
        else:
            listeDistance[s0]=[0,s0] #initialise la distance de s0 à 0 et son prédécesseur à lui-même

   tours = 0 #Initialise les tours à 0
   maj = True #Initialise la maj à vrai pour rentrer dans la boucle
   while maj and tours <= n-1: #Tant qu'il y a des mises à jours et que le nombre de tours ne dépasse pas la largeur de la matrice
       maj = False #Il n'y a pas eu de mise à jour
       for i in listeFleches: #Pour toutes les flèches
            if listeDistance[i[0]][0] + M[i[0]][i[1]] < listeDistance[i[1]][0]:
                listeDistance[i[1]][0] = listeDistance[i[0]][0] + M[i[0]][i[1]]
                listeDistance[i[1]][1] = i[0]
                maj = True #Il y a eu une mise à jour
       tours += 1 #Un tour est passé
        
   print("--- RESULTAT DE BELLMAN-FORD ---")
   print("Nombre de tours : ",tours)
   if tours >= n : #Si le nombre de tours est égale ou dépasse la largeur de la matrice
        print("Pas de plus court chemin : présence d'un cycle négatif") #Alors elle possède un cycle négatif et il n'existe aucun plus court chemin
   else:
        for i in range(n):
            if i != s0:
                if listeDistance[i][0] == float('inf'): #Sinon si la distance de i est infini on renvoie :
                    print("Sommet ", i, "non joignable à ",s0," par un chemin dans le graphe G.")
                else:
                    courtChemin = [i] #Initialise la liste du plus court chemin au sommet courant i
                    longueurCourtChemin = 0.0 #Initialise la longueur à 0
                    s = i #Récupère le sommet courant 
                    while s != s0: #Tant que le sommet de parcours n'est pas s0
                        longueurCourtChemin += listeDistance[s][0] #On somme la distance de chaque sommet pour obtenir le total
                        s = listeDistance[s][1] #Récupère le sommet précédant de s
                        courtChemin.append(s) #Rajoute le prédécesseur au plus court chemin
                    courtChemin.reverse() #Inverse le plus court chemin pour avoir s0 à s 
                    print("Plus court chemin de ",s0," à ",i," : ",courtChemin," de longueur ",longueurCourtChemin)
   return listeDistance, tours

#Parcours en largeur
def bellmanFordLargeur(M, s0):
    """
    Algorithme de Bellman-Ford
    Prend en entrée la matrice d'un graphe pondéré à poids relatifs (M) et un sommet (s0)
    Retourne pour chacun des sommets la longueur et l'itinéraire du plus court chemin de s0 à s
    La mention 'Sommet t non joignable à s0 par un chemin dans le graphe G'
    Si le nombre de tours est égal à la largeur de la matrice M alors affiche :
        "Pas de plus court chemin : présence d'un cycle négatif"
    """
    #Initialisation
    n = np.shape(M)[0] #Récupère la largeur de la matrice
    listeDistance = {} #Initialise la liste des distances et des prédécesseurs des sommets
    listeFleches = [] #Initialise la liste des flèches
    for i in range(n):
        for j in range(n):
            if M[i][j]<float("inf"):
                listeFleches.append((i,j)) #On récupère toutes les flèches du graphe

    #Initialise le dictionnaire des sommets avec la distance et le prédecesseur des sommets
    for t in range(n):
        if (t!=s0):
            if M[s0][t]<float('inf'):
                listeDistance[t]=[M[s0][t],s0] #Si le sommet est un successeur de s0, on l'initialise au poids entre s0 et le sommet et le prédécesseur à S0
            else:
                listeDistance[t]=[float('inf'),None] #Sinon, il est initialisé à infini sans prédécesseur
        else:
            listeDistance[s0]=[0,s0] #initialise la distance de s0 à 0 et son prédécesseur à lui-même

    tours = 0 #Initialise les tours à 0
    maj = True #Initialise la maj à vrai pour rentrer dans la boucle
    while maj and tours <= n-1: #Tant qu'il y a des mises à jours et que le nombre de tours ne dépasse pas la largeur de la matrice
        maj = False #Il n'y a pas eu de mise à jour
        for i in listeFleches: #Pour toutes les flèches
            if listeDistance[i[0]][0] + M[i[0]][i[1]] < listeDistance[i[1]][0]:
                listeDistance[i[1]][0] = listeDistance[i[0]][0] + M[i[0]][i[1]]
                listeDistance[i[1]][1] = i[0]
                maj = True #Il y a eu une mise à jour
        tours += 1 #Un tour est passé
        
    print("--- RESULTAT DE BELLMAN-FORD ---")
    print("Nombre de tours : ",tours)
    if tours >= n : #Si le nombre de tours est égale ou dépasse la largeur de la matrice
        print("Pas de plus court chemin : présence d'un cycle négatif") #Alors elle possède un cycle négatif et il n'existe aucun plus court chemin
    else:
        for i in range(n):
            if i != s0:
                if listeDistance[i][0] == float('inf'): #Sinon si la distance de i est infini on renvoie :
                    print("Sommet ", i, "non joignable à ",s0," par un chemin dans le graphe G.")
                else:
                    courtChemin = [i] #Initialise la liste du plus court chemin au sommet courant i
                    longueurCourtChemin = 0.0 #Initialise la longueur à 0
                    s = i #Récupère le sommet courant 
                    while s != s0: #Tant que le sommet de parcours n'est pas s0
                        longueurCourtChemin += listeDistance[s][0] #On somme la distance de chaque sommet pour obtenir le total
                        s = listeDistance[s][1] #Récupère le sommet précédant de s
                        courtChemin.append(s) #Rajoute le prédécesseur au plus court chemin
                    courtChemin.reverse() #Inverse le plus court chemin pour avoir s0 à s 
                    print("Plus court chemin de ",s0," à ",i," : ",courtChemin," de longueur ",longueurCourtChemin)
    return listeDistance, tours

#Parcours en profondeur
def bellmanFordProfondeur(M, s0):
    """
    Algorithme de Bellman-Ford
    Prend en entrée la matrice d'un graphe pondéré à poids relatifs (M) et un sommet (s0)
    Retourne pour chacun des sommets la longueur et l'itinéraire du plus court chemin de s0 à s
    La mention 'Sommet t non joignable à s0 par un chemin dans le graphe G'
    Si le nombre de tours est égal à la largeur de la matrice M alors affiche :
        "Pas de plus court chemin : présence d'un cycle négatif"
    """
    #Initialisation
    n = np.shape(M)[0] #Récupère la largeur de la matrice
    listeDistance = {} #Initialise la liste des distances et des prédécesseurs des sommets
    listeFleches = [] #Initialise la liste des flèches
    for i in range(n):
        listeFleches += pp(M,i)
    
    #Initialise le dictionnaire des sommets avec la distance et le prédecesseur des sommets
    for t in range(n):
        if (t!=s0):
            if M[s0][t]<float('inf'):
                listeDistance[t]=[M[s0][t],s0] #Si le sommet est un successeur de s0, on l'initialise au poids entre s0 et le sommet et le prédécesseur à S0
            else:
                listeDistance[t]=[float('inf'),None] #Sinon, il est initialisé à infini sans prédécesseur
        else:
            listeDistance[s0]=[0,s0] #initialise la distance de s0 à 0 et son prédécesseur à lui-même

    tours = 0 #Initialise les tours à 0
    maj = True #Initialise la maj à vrai pour rentrer dans la boucle
    while maj and tours <= n-1: #Tant qu'il y a des mises à jours et que le nombre de tours ne dépasse pas la largeur de la matrice
        maj = False #Il n'y a pas eu de mise à jour
        for i in listeFleches: #Pour toutes les flèches
            if listeDistance[i[0]][0] + M[i[0]][i[1]] < listeDistance[i[1]][0]:
                listeDistance[i[1]][0] = listeDistance[i[0]][0] + M[i[0]][i[1]]
                listeDistance[i[1]][1] = i[0]
                maj = True #Il y a eu une mise à jour
        tours += 1 #Un tour est passé
        
    print("--- RESULTAT DE BELLMAN-FORD ---")
    print("Nombre de tours : ",tours)
    if tours >= n : #Si le nombre de tours est égale ou dépasse la largeur de la matrice
        print("Pas de plus court chemin : présence d'un cycle négatif") #Alors elle possède un cycle négatif et il n'existe aucun plus court chemin
    else:
        for i in range(n):
            if i != s0:
                if listeDistance[i][0] == float('inf'): #Sinon si la distance de i est infini on renvoie :
                    print("Sommet ", i, "non joignable à ",s0," par un chemin dans le graphe G.")
                else:
                    courtChemin = [i] #Initialise la liste du plus court chemin au sommet courant i
                    longueurCourtChemin = 0.0 #Initialise la longueur à 0
                    s = i #Récupère le sommet courant 
                    while s != s0: #Tant que le sommet de parcours n'est pas s0
                        longueurCourtChemin += listeDistance[s][0] #On somme la distance de chaque sommet pour obtenir le total
                        s = listeDistance[s][1] #Récupère le sommet précédant de s
                        courtChemin.append(s) #Rajoute le prédécesseur au plus court chemin
                    courtChemin.reverse() #Inverse le plus court chemin pour avoir s0 à s 
                    print("Plus court chemin de ",s0," à ",i," : ",courtChemin," de longueur ",longueurCourtChemin)
    return listeDistance, tours


def pp(M,s):
    n=len(M)       # taille du tableau = nombre de sommets
    couleur={}     # On colorie tous les sommets en blanc et s en vert
    for i  in range(n):
        couleur[i]='blanc'
    couleur[s]='vert'
    pile=[s]       # on initialise la pile à s
    Resultat=[s] # on initialise la liste des résultats à s
    
    while pile !=[]: # tant que la pile n'est pas vide,
        i=pile[-1]          # on prend le dernier sommet i de la pile
        Succ_blanc=[]       # on crée la liste de ses successeurs non déjà visités (blancs)
        for j in range(n):
            if (M[i][j]==1 and couleur[j]=='blanc'):
                Succ_blanc.append(j)
        if Succ_blanc!=[]:  # s'il y en a,
            v= Succ_blanc[0]    # on prend le premier (si on veut l'ordre alphabétique)
            couleur[v]='vert'   # on le colorie en vert, 
            pile.append(v)      # on l'empile
            Resultat.append(v)  # on le met en liste rsultat
        else:               # sinon:
            pile.pop()          # on sort i de la pile
    
    listeFleches = []
    for i in Resultat:
        listeFleches.append((s,i))
    return(listeFleches)

#Exercice 6.3
def plusefficace(M):
    """effectue tout les algo de Bellman ford et compare le nombre de tours effectuer
    """
    print("\nTest de tout les choix de tri de la liste de flèches\n")
    V1=bellmanFordAleatoire(M,0)
    #Fais avec une liste de manière arbitraire
    V1=V1[0:]
    #Récupere le nombre de tours (et pas la liste des plus court chemins)
    V2=bellmanFordLargeur(M,0)
    #Fais avec une liste en largeur
    V2=V2[0:]
    #Récupere le nombre de tours (et pas la liste des plus court chemins)
    V3=bellmanFordProfondeur(M,0)
    #Fais avec une liste en profondeur
    V3=V3[0:]
    #Récupere le nombre de tours (et pas la liste des plus court chemins)
    if V1[1] >= V2[1] and V1[1] >= V3[1]:
        print("\nBellmanFord plus efficace lors d'une liste de flèche de manière arbitraire")
    if V2[1] >= V1[1] and V2[1] >= V3[1]:
        print("\nBellmanFord plus efficace lors d'une liste de flèche d'ordre choisi en largeur")
    if V3[1] >= V1[1] and V3[1] >= V2[1]:
        print("\nBellmanFord plus efficace lors d'une liste de flèche d'ordre choisi en profondeur")

plusefficace(M)

##################################################################################################################
################################################ Exercice 7 ######################################################
##################################################################################################################


def Dij(n):
    """
    Algorithme qui calcule le temps d'exécution de l'algorithme Dijkstra pour n sommets
    Prend un entier n en entrée
    Renvoie le temps d'exécution en float
    """
    M = MatriceAleatoire(n, 1, 50)
    start = time.perf_counter()
    Dijkstra(M,0)
    end = time.perf_counter()
    elapsed = end - start
    print("\n--- TEST DE COMPLEXITE DE DIJKSTRA ---")
    print(f'Temps d\'exécution : {elapsed:.2}ms')
    return elapsed
    
def BF(n):
    """
    Algorithme qui calcule le temps d'exécution de l'algorithme le plus efficace de Bellman-Ford pour n sommets
    Prend un entier n en entrée
    Renvoie le temps d'exécution en float
    """
    M = MatriceAleatoire(n, 1, 50)
    start = time.perf_counter()
    bellmanFordLargeur(M,0)
    end = time.perf_counter()
    elapsed = end - start
    print("\n--- TEST DE COMPLEXITE DE BELLMAN-FORD ---")
    print(f'Temps d\'exécution du Bellman-Ford à parcours aléatoire: {elapsed:.2}ms')
    return elapsed

def rep():
    """Fonction qui permet de tracer le graphe des temps d'exécution de x de 1 à 100
    des algorithmes Dijkstra et Bellman-Ford"""
    x = np.linspace(1,100,100) #Crée la liste des abscisses de 1 à 100 contenant 100 points
    listePointsDij = [] #Initialise la liste des points de l'algorithme Dijkstra à vide
    listePointsBel = [] #Initialise la liste des points de l'algorithme de Bellman-Ford à vide
    for i in range(len(x)): #Remplissage des listes
        listePointsDij.append(Dij(i))
        listePointsBel.append(BF(i))
    plot(x,listePointsDij, label="dijkstra") #Trace la courbe de dijkstra
    plot(x,listePointsBel, label="bellman-ford") #Trace la courbe de bellman-ford
    legend()
    show()
    
def repLog():
    """Fonction qui permet de tracer le graphe semilogarithmique sur les ordonnées des temps d'exécution de x de 1 à 100
    des algorithmes Dijkstra et Bellman-Ford"""
    x=np.linspace(1,100,100) #Crée la liste des abscisses de 1 à 100 contenant 100 points
    listePointsDij = [] #Initialise la liste des points de l'algorithme Dijkstra à vide
    listePointsBel = [] #Initialise la liste des points de l'algorithme de Bellman-Ford à vide
    for i in range(len(x)): #Remplissage des listes
        listePointsDij.append(Dij(i))
        listePointsBel.append(BF(i))
    semilogy(x,listePointsDij,label="dijkstra") #Trace la courbe semi-logarithmique de dijkstra
    semilogy(x,listePointsBel, label="bellman-ford") #Trace la courbe semi-logarithmique de bellman-ford
    legend()
    show()