import numpy as np
import math

def Dijkstra(M, s0):
    """
    Algorithme de Dijkstra
    Prend en entrée la matrice d'un graphe pondéré à poids positifs (M) et un sommet (s0)
    Retourne pour chacun des sommets la longueur et l'itinéraire du plus court chemin de s0 à s
    La mention 'Sommet t non joignable à s0 par un chemin dans le graphe G'
    """
    sommets = {s0 : (0,s0)}
    
    n = np.shape(M)[0]
    for i in range(0,n):
        if i != s0 :
            if M[s0][i]<math.inf:
                sommets[i] = (M[s0][i],s0)
            else:
                sommets[i] = (float('inf'), None)

    A = [s0]
    listeValeurs = [i[0] for i in list(sommets.values())]
    maj = True
    while (A != [] or math.inf in listeValeurs) and maj :
        min = float('inf')
        for i in sommets.keys():
            if sommets[i][0] < min and i not in A:
                min = sommets[i][0]
                s = i 
    
        if s not in A:
            A.append(s)
        else:
            maj= False
            
        for t in range(0,n):
            if t not in A:
                if sommets[s][0] + M[s][t] < sommets[t][0]:
                    sommets[t] = (sommets[s][0]+M[s][t],s)
        listeValeurs = [i[0] for i in list(sommets.values())]
        
    for i in sommets.keys():
        if i != s0:
            if sommets[i][0] == float('inf'):
                print("Sommet ", i, "non joignable à ",s0," par un chemin dans le graphe G.")
            else:
                longueurCourtChemin = 0
                courtChemin = [i]
                s = i
                while s != s0:
                    longueurCourtChemin += sommets[s][0]
                    s = sommets[s][1]
                    courtChemin.append(s)
                courtChemin.reverse()
                print("Plus court chemin de ",s0," à ",i," : ",courtChemin," de longueur ",longueurCourtChemin)
    print()
    return sommets

M = [[float('inf'),4.0,6.0,float('inf'),8.0,float('inf'),float('inf'),float('inf')],
     [float('inf'),float('inf'),1.0,float('inf'),2.0,float('inf'),float('inf'),float('inf')],
     [float('inf'),float('inf'),float('inf'),4.0,float('inf'),4.0,6.0,float('inf')],
     [float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),3.0,float('inf')],
     [float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),1.0,float('inf'),float('inf')],
     [float('inf'),2.0,float('inf'),4.0,1.0,2.0,2.0,float('inf')],
     [float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf'),float('inf')],
     [float('inf'),float('inf'),float('inf'),3.0,float('inf'),float('inf'),2.0,float('inf')]]
print(Dijkstra(M,0))