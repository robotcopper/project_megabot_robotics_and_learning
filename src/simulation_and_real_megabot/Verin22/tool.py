def tracer_courbe(tableau_de_tuples):
    # Extraction des données
    abscisses = [t[0] for t in tableau_de_tuples]
    ordonnees = [t[1:] for t in tableau_de_tuples]

    # Afficher les points
    plt.scatter(abscisses, [y[0] for y in ordonnees], label='delta P (m)')
    plt.scatter(abscisses, [y[1] for y in ordonnees], label='delta T (s)')
    plt.scatter(abscisses, [y[2] for y in ordonnees], label='Vitesses m/s ')

    # Ajouter des labels et une légende
    plt.xlabel('Première valeur (abscisses)')
    plt.ylabel('Valeurs suivantes (ordonnées)')
    plt.legend()

    # Afficher le graphique
    plt.show()
import re
import matplotlib.pyplot as plt

def extraire_poids(chaine):
    match = re.search(r'(\d+(\.\d+)?)kg', chaine)
    
    if match:
        poids_str = match.group(1)
        poids_entier = int(float(poids_str))
        #print(Fore.GREEN + "Poids de : " + str(poids_entier) + " kgs.")
        return poids_entier
    else:
        print(Fore.RED + "Aucun poids trouvé dans la chaîne.")
        return None

def moyenne_glissante_np(tableau, taille):
    if taille <= 0 or not isinstance(taille, int):
        raise ValueError("La taille de la moyenne glissante doit être un entier positif.")

    if not isinstance(tableau, np.ndarray):
        raise ValueError("Le tableau doit être de type numpy.ndarray.")

    if taille > len(tableau):
        raise ValueError("La taille de la fenêtre glissante ne peut pas être plus grande que la taille du tableau d'entrée.")

    resultats = np.zeros_like(tableau, dtype=float)
    
    # Remplir les premières positions avec les données du tableau d'entrée
    resultats[:taille-1] = tableau[:taille-1]

    for i in range(taille-1, len(tableau)):
        moyenne = np.mean(tableau[i - taille + 1:i + 1])
        resultats[i] = moyenne

    return resultats

def extraire_numero_acquisition(nom_fichier):
    # Expression régulière pour extraire le numéro d'acquisition
    expression_reguliere = r"_acq(\d+)_"

    # Utilisation de la fonction findall pour trouver toutes les occurrences correspondantes
    resultats = re.findall(expression_reguliere, nom_fichier)

    # Si des correspondances sont trouvées, le numéro d'acquisition sera dans resultats[0]
    if resultats:
        numero_acquisition = resultats[0]
        return numero_acquisition
    else:
        return None  # Retourne None si aucun numéro d'acquisition n'est trouvé dans le nom du fichier

def moyenne(tab):
    tab_sans_nan = [x for x in tab if not np.isnan(x)]
    somme = sum(tab_sans_nan)
    moyenne = somme / len(tab_sans_nan)
    return moyenne
