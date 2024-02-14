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
colorama.init(autoreset=True)

def afficher_graphiques(masses, positions, temps, vitesses_moyennes):
    app = QApplication(sys.argv)
    main_window = QMainWindow()
    main_window.setWindowTitle("Affichage de delta P en m et des vitesses moyennes en descente sur verrin 2.3")  
    central_widget = QWidget()
    main_window.setCentralWidget(central_widget)
    layout = QVBoxLayout(central_widget)

    plot_widget_vitesses = pg.PlotWidget()
    layout.addWidget(plot_widget_vitesses)
    curve_vitesses = plot_widget_vitesses.plot(pen='b')
    plot_widget_vitesses.plotItem.showGrid(True, True, alpha=0.5)

    # Affichage des vitesses
    vitesses_moyennes = [abs(element) for element in vitesses_moyennes]
    plot_widget_vitesses.plot(masses, vitesses_moyennes, pen=None, symbol='o', symbolPen='r', symbolBrush='r')

    # Calcul de la régression linéaire pour les vitesses
    vitesses_regression = np.polyfit(masses, vitesses_moyennes, 1)
    vitesses_regression_line = np.polyval(vitesses_regression, masses)
    plot_widget_vitesses.plot(masses, vitesses_regression_line, pen='r')

    plot_widget_vitesses.setYRange(0, 0.5)

    main_window.show()
    sys.exit(app.exec_())

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

        print(acq, masse, d_p, d_t, v_moy)
        return masse, d_p, d_t, v_moy

### executions sur plusieurs fichiers ###
# Chemin du répertoire
repertoire_specifie = "./"
tab_donnees = []
if os.path.exists(repertoire_specifie) and os.path.isdir(repertoire_specifie):
    for nom_fichier in os.listdir(repertoire_specifie):
        if nom_fichier == "__pycache__" or nom_fichier == "down_23.py" or nom_fichier == "filtre.py" or nom_fichier == "up_23.py" or nom_fichier == "tool.py" or nom_fichier == "Vitesses_23.py":
            continue
        chemin_fichier = os.path.join(repertoire_specifie, nom_fichier)       
        t = stat(repertoire_specifie, nom_fichier)
        tab_donnees.append(t)
else:
    print("Le répertoire spécifié n'existe pas ou n'est pas un répertoire.")


masses = [data[0] for data in tab_donnees]
positions = [data[1] for data in tab_donnees]
temps = [data[2] for data in tab_donnees]
vitesses_moyennes = [data[3] for data in tab_donnees]

afficher_graphiques(masses, positions, temps, vitesses_moyennes)
