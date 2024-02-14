import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import pyqtgraph as pg
import numpy as np
from scipy.ndimage import median_filter
import os
from scipy.stats import linregress
import colorama
from colorama import Fore, Style
from tool import *
from filtre import *
colorama.init(autoreset=True)
import matplotlib.pyplot as plt

def afficher_graphiques(masses_descente, vitesses_moyennes_descente, masses_montee,vitesses_moyennes_montee):
    fig, ax = plt.subplots()

    # Traitement et affichage pour les données de montée
    ax.scatter(masses_montee, vitesses_moyennes_montee, color='b', label='Vitesses moyennes (montée)')
    slope_montee, intercept_montee, _, _, _ = linregress(masses_montee, vitesses_moyennes_montee)
    x_montee = np.array([0, max(masses_montee)])
    y_montee = intercept_montee + slope_montee * x_montee
    ax.plot(x_montee, y_montee, color='b', linestyle='-', label='Régression linéaire (montée)')

    # Traitement et affichage pour les données de descente
    ax.scatter(masses_descente, vitesses_moyennes_descente, color='r', label='Vitesses moyennes (descente)')
    slope_descente, intercept_descente, _, _, _ = linregress(masses_descente, vitesses_moyennes_descente)
    x_descente = np.array([0, max(masses_descente)])
    y_descente = intercept_descente + slope_descente * x_descente
    ax.plot(x_descente, y_descente, color='r', linestyle='-', label='Régression linéaire (descente)')

    # Configuration du graphique
    ax.set_title("Vitesses en montée et descente sur verrin 2.3")
    ax.set_xlabel("Masses (kg)")
    ax.set_ylabel("Vitesses moyennes (m/s)")
    ax.legend()
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=min(min(vitesses_moyennes_montee), min(vitesses_moyennes_descente)), top=max(max(vitesses_moyennes_montee), max(vitesses_moyennes_descente)))
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)

    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.set_ylim(top=0.5)
    
    plt.show()

def stat(base, nom_fichier):
    verify = False
    counter_values_pos_reel_1 = []
    counter_values_pwm_cible = []
    vit_positive = []
    vit_negative = []
    affichage = False

    with open(base + nom_fichier, 'r', encoding="ISO-8859-1") as fichier:
        compteur_lignes = 0
        for ligne in fichier:
            if compteur_lignes < 3:
                compteur_lignes += 1
                continue
            if ligne.startswith("$SOPE"):
                continue 
            ligne_sans_prefixe = ligne.strip().replace("L23#", "")
            tab_valeurs = ligne_sans_prefixe.split('#')

            if len(tab_valeurs) < 5:
                continue

            if verify == False:
                temp_0 = float(tab_valeurs[4])
                verify = True

            flt_pos_reel = float(tab_valeurs[0])
            flt_pwm_cible = float(tab_valeurs[3])
            flt_temp = float(tab_valeurs[4]) - temp_0

            counter_values_pos_reel_1.append((flt_temp, flt_pos_reel))
            counter_values_pwm_cible.append((flt_temp, flt_pwm_cible))

        data_array_values_pos_reel = np.array(counter_values_pos_reel_1)
        temps = data_array_values_pos_reel[:, 0]
        positions = data_array_values_pos_reel[:, 1]

        position_t = []
        temp_t = [] 

        acq = int(extraire_numero_acquisition(nom_fichier))
        masse = extraire_poids(nom_fichier)

        f_bas = 20
        f_haut = 280
        intervalle = 20
 
        f_len = f_haut - f_bas - 1

        for i in range (f_bas, f_haut, intervalle):
            position_t.append(positions[i])
            temp_t.append(temps[i])

        with np.errstate(divide='ignore', invalid='ignore'):# gere le 0 divide error
            vitesses = np.gradient(position_t, temp_t)

        m_p = len(position_t) - 1
        m_t = len(temp_t) - 1
        d_p = position_t[m_p]  - position_t[0]
        d_t = temp_t[m_t] - temp_t[0]
        v_moy = d_p / d_t
        print(Fore.GREEN + " " + 
            str(acq)+ " " + 
            str(masse)+ " " + 
            str(d_p)+ " " + 
            str(d_t)+ " " + 
            str(v_moy))

        #print(acq, masse, d_p, d_t, v_moy)
        return masse, d_p, d_t, v_moy
    
def down_23():
    ### executions sur plusieurs fichiers ###
    # Chemin du répertoire
    repertoire_specifie = "./"
    tab_donnees = []
    if os.path.exists(repertoire_specifie) and os.path.isdir(repertoire_specifie):
        for nom_fichier in os.listdir(repertoire_specifie):
            if nom_fichier == "__pycache__" or nom_fichier == "down_23.py" or nom_fichier == "filtre.py" or nom_fichier == "up_23.py" or nom_fichier == "tool.py" or nom_fichier == "vitesses_23.py":
                continue
            chemin_fichier = os.path.join(repertoire_specifie, nom_fichier)
            print(Fore.RED + nom_fichier)
            t = stat(repertoire_specifie, nom_fichier)
            tab_donnees.append(t)
    else:
        print("Le répertoire spécifié n'existe pas ou n'est pas un répertoire.")


    masses = [data[0] for data in tab_donnees]
    positions = [data[1] for data in tab_donnees]
    temps = [data[2] for data in tab_donnees]
    vitesses_moyennes = [data[3] for data in tab_donnees]

    # # Appel de la fonction d'affichage avec les données extraites
    # afficher_graphiques(masses, vitesses_moyennes)
    return masses,vitesses_moyennes

def stat_up23(base, nom_fichier):
    verify = False
    counter_values_pos_reel_1 = []
    counter_values_pwm_cible = []
    vit_positive = []
    vit_negative = []
    affichage = False

    with open(base + nom_fichier, 'r', encoding="ISO-8859-1") as fichier:
        compteur_lignes = 0
        for ligne in fichier:
            if compteur_lignes < 3:
                compteur_lignes += 1
                continue
            if ligne.startswith("$SOPE"):
                continue 
            ligne_sans_prefixe = ligne.strip().replace("L23#", "")
            tab_valeurs = ligne_sans_prefixe.split('#')

            if len(tab_valeurs) < 5:
                continue

            if verify == False:
                temp_0 = float(tab_valeurs[4])
                verify = True

            flt_pos_reel = float(tab_valeurs[0])
            flt_pwm_cible = float(tab_valeurs[3])
            flt_temp = float(tab_valeurs[4]) - temp_0

            counter_values_pos_reel_1.append((flt_temp, flt_pos_reel))
            counter_values_pwm_cible.append((flt_temp, flt_pwm_cible))

        data_array_values_pos_reel = np.array(counter_values_pos_reel_1)
        temps = data_array_values_pos_reel[:, 0]
        positions = data_array_values_pos_reel[:, 1]

        position_t = []
        temp_t = [] 

        acq = int(extraire_numero_acquisition(nom_fichier))
        masse = extraire_poids(nom_fichier)

        if masse == 1:
            f_bas = 320
            f_haut = 620
            intervalle = 20
        if masse == 21:
            f_bas = 300
            f_haut = 620
            intervalle = 20
        if masse == 41:
            f_bas = 300
            f_haut = 640
            intervalle = 20
        if masse == 61:
            f_bas = 300
            f_haut = 640
            intervalle = 20
        if masse == 81:
            f_bas = 280
            f_haut = 680
            intervalle = 20
        if masse == 91:
            f_bas = 280
            f_haut = 700
            intervalle = 20
        if masse == 101:
            f_bas = 280
            f_haut = 700
            intervalle = 20
        if masse == 111:
            f_bas = 280
            f_haut = 740
            intervalle = 20
        if masse == 121:
            f_bas = 940
            f_haut = 1340
            intervalle = 20
        if masse == 146:
            f_bas = 300
            f_haut = 740
            intervalle = 20
        if masse == 150:
            f_bas = 960
            f_haut = 1400
            intervalle = 20
        # f_bas = 40
        # f_haut = 300
        # intervalle = 20

        f_len = f_haut - f_bas - 1

        for i in range (f_bas, f_haut, intervalle):
            position_t.append(positions[i])
            temp_t.append(temps[i])
            # print(Fore.RED + str(i))
            # print(Fore.GREEN + str(positions[i]) )
            # print(Fore.YELLOW + str(temps[i]))

        with np.errstate(divide='ignore', invalid='ignore'):# gere le 0 divide error
            vitesses = np.gradient(position_t, temp_t)

        # print("bas : ", f_bas)
        # print("haut : ", f_haut)
        #print("echelle : ", f_len)

        # print("Temps_0 : ", temp_0)
        # print("Nombres de valeurs echantilllone : ",len(data_array_values_pos_reel))
        # print("V max : ", max(vitesses))
        m_p = len(position_t) - 1
        m_t = len(temp_t) - 1
        d_p = position_t[m_p]  - position_t[0]
        d_t = temp_t[m_t] - temp_t[0]
        v_moy = d_p / d_t
        print(Fore.GREEN + " " + 
            str(acq)+ " " + 
            str(masse)+ " " + 
            str(d_p)+ " " + 
            str(d_t)+ " " + 
            str(v_moy))
        # print("distance : ", d_p)
        # print(Fore.RED + "Vitesse moyenne : " , v_moy)
        #print("V moyenne : ", np.sum(np.absolute(vitesses)) / len(vitesses) )
        #return acq, masse, d_p, d_t, v_moy
        #print(acq, masse, d_p, d_t, v_moy)
        return masse, d_p, d_t, v_moy

def up_23():
    ### executions sur plusieurs fichiers ###
    # Chemin du répertoire
    repertoire_specifie = "./"
    tab_donnees = []
    if os.path.exists(repertoire_specifie) and os.path.isdir(repertoire_specifie):
        for nom_fichier in os.listdir(repertoire_specifie):
            if nom_fichier == "__pycache__" or nom_fichier == "down_23.py" or nom_fichier == "filtre.py" or nom_fichier == "up_23.py" or nom_fichier == "tool.py" or nom_fichier == "vitesses_23.py":
                continue
            chemin_fichier = os.path.join(repertoire_specifie, nom_fichier)
            print(Fore.RED + nom_fichier)
            t = stat_up23(repertoire_specifie, nom_fichier)
            tab_donnees.append(t)
            #print(t)
    else:
        print("Le répertoire spécifié n'existe pas ou n'est pas un répertoire.")


    masses = [data[0] for data in tab_donnees]
    positions = [data[1] for data in tab_donnees]
    temps = [data[2] for data in tab_donnees]
    vitesses_moyennes = [-1*data[3] for data in tab_donnees]

    # # Appel de la fonction d'affichage avec les données extraites
    # afficher_graphiques(masses, vitesses_moyennes)
    return masses,vitesses_moyennes

afficher_graphiques(down_23()[0],down_23()[1],up_23()[0],up_23()[1])
