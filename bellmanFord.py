# -*- coding: utf-8 -*-
"""
Created on Mon Jun 13 01:00:30 2022

@author: bibota
"""
import numpy as np

def bellmanFort(M, s0):
    n = np.shape(M)[0]
    listeDistance = {}
    listeFleches = []
    for i in range(n):
        for j in range(n):
            if M[i][j]<float("inf"):
                listeFleches.append((i,j))
                
    

    for t in range(n):
        if (t!=s0):
            if M[s0][t]<float('inf'):
                listeDistance[t]=[M[s0][t],s0]
            else:
                listeDistance[t]=[float('inf'),None]
        else:
            listeDistance[s0]=(0,s0)

    tours = 0
    maj = True
    while maj and tours <= n-1:
        for i in listeFleches:
            maj = False
            if listeDistance[i[0]][0] + M[i[0]][i[1]] < listeDistance[i[1]][0]:
                listeDistance[i[1]][0] = listeDistance[i[0]][0] + M[i[0]][i[1]]
                listeDistance[i[1]][1] = i[0]
                maj = True
        tours += 1
        
    if tours >= n :
        print("Pas de plus court chemin : présence d'un cycle négatif")
    else:
        for i in range(n):
            if i != s0:
                if listeDistance[i][0] == float('inf'):
                    print("Sommet ", i, "non joignable à ",s0," par un chemin dans le graphe G.")
                else:
                    courtChemin = [i]
                    longueurCourtChemin = 0.0
                    s = i
                    while s != s0:
                        longueurCourtChemin += listeDistance[s][0]
                        s = listeDistance[s][1]
                        courtChemin.append(s)
                    courtChemin.reverse()
                    print("Plus court chemin de ",s0," à ",i," : ",courtChemin," de longueur ",longueurCourtChemin)
    return listeDistance
        
M = [[float('inf'),4.0,6.0,float('inf'),8.0,float('inf'),float('inf'),float('inf')],
     [float('inf'),float('inf'),1.0,float('inf'),2.0,float('inf'),float('inf'),float('inf')],
     [float('inf'),float('inf'),float('inf'),4.0,float('inf'),4.0,6.0,float('inf')],
     [float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),3.0,float('inf')],
     [float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),1.0,float('inf'),float('inf')],
     [float('inf'),2.0,float('inf'),4.0,1.0,2.0,2.0,float('inf')],
     [float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf')],
     [float('inf'),float('inf'),float('inf'),3.0,float('inf'),float('inf'),2.0,float('inf')]]
print(bellmanFort(M,0))

print()
M = [[float('inf'),86,34,134,float('inf'),float('inf')],
     [86,float('inf'),float('inf'),float('inf'),93,float('inf')],
     [34,float('inf'),float('inf'),float('inf'),47,float('inf')],
     [134,float('inf'),float('inf'),float('inf'),float('inf'),62],
     [float('inf'),93,47,float('inf'),float('inf'),56],
     [float('inf'),float('inf'),float('inf'),62,56,float('inf')]]
print(bellmanFort(M,0))

M = [[float('inf'),-2,float('inf'),float('inf')],
     [float('inf'),float('inf'),1,3],
     [-2,float('inf'),float('inf'),-4],
     [float('inf'),float('inf'),float('inf'),float('inf')]]
print(bellmanFort(M,0))