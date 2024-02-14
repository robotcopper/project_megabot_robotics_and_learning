def filtre(masse, acq):
    Data_1kg  = [[1600,4060,3800,40,40],#f_bas monte
    [2420,4860,4580,780,600],#f_haut monte
    [660,3360,3140,860,640],#f_bas descen
    [1580,4040,3740,1700,1360]]#f_haut descen

    Data_6kg  = [[1240,1940,1500,1820,1620],#f_bas monte
    [1940,2380,2000,2240,2480],#f_haut monte
    [660,1480,2040,2260,2480],#f_bas descen
    [1220,1900,2460,2700,3400]]#f_haut descen

    Data_21kg  = [[40,40,40,40],#f_bas monte
    [620,620,660,840],#f_haut monte
    [660,660,700,900],#f_bas descen
    [1380,1280,1560,1800]]#f_haut descen

    Data_41kg  = [[1540,2100,2300,2140,1140],#f_bas monte
    [2060,2660,2780,2560,1600],#f_haut monte
    [800,1600,660,580,600],#f_bas descen
    [1500,2060,1320,1140,1100]]#f_haut descen

    Data_61kg  = [[20,20,40,20,],#f_bas monte
    [900,900,860,900],#f_haut monte
    [940,940,940,940],#f_bas descen
    [1880,1880,1880,1880]]#f_haut descen

    Data_81kg  = [[40,40,40,40,40],#f_bas monte
    [900,900,900,900,900],#f_haut monte
    [940,940,940,940,940],#f_bas descen
    [1900,1900,1900,1900,1900]]#f_haut descen

    Data_101kg  = [[40,40,40,40,40],#f_bas monte
    [900,900,900,900,900],#f_haut monte
    [940,940,940,940,940],#f_bas descen
    [1900,1900,1900,1900,1900]]#f_haut descen

    Data_121kg  = [[40,40,40,40,40],#f_bas monte
    [900,900,900,900,900],#f_haut monte
    [940,940,940,940,940],#f_bas descen
    [1900,1900,1900,1900,1900]]#f_haut descen

    Data_141kg  = [[40,40,40,40,40],#f_bas monte
    [920,920,920,920,920],#f_haut monte
    [960,960,960,960,960],#f_bas descen
    [1940,1940,1940,1940,1940]]#f_haut descen

    Data_150kg  = [[40,40,40,40,40],#f_bas monte
    [880,900,780,880,880],#f_haut monte
    [980,980,880,940,960],#f_bas descen
    [1900,1920,1860,1800,1940]]#f_haut descen

    Data_170kg  = [[40,40,40,40,40],#f_bas monte
    [900,840,880,880,880],#f_haut monte
    [960,900,940,940,2920],#f_bas descen
    [2000,1940,2000,2000,3940]]#f_haut descen

    dico_masse = {}
    dico_masse[0] = Data_1kg
    dico_masse[1] = Data_6kg
    dico_masse[2] = Data_21kg
    dico_masse[3] = Data_41kg
    dico_masse[4] = Data_61kg
    dico_masse[5] = Data_81kg
    dico_masse[6] = Data_101kg
    dico_masse[7] = Data_121kg
    dico_masse[8] = Data_141kg
    dico_masse[9] = Data_150kg
    dico_masse[10] = Data_170kg

    # fichiers ou il n'y a pas 5 acquisitions
    if masse == 21 and acq == 4 or masse == 21 and acq == 5 or masse == 61:
        acq = acq - 1

    tab_masse = [1,6,21,41,61,81,101,121,141,150,170]
    tab_travail = []
    
    for i in range (0, len(tab_masse)):
        
        if masse == tab_masse[i]:
            tab_travail = list(zip(*dico_masse[i]))[acq - 1]
            break

    if tab_travail == []:
        return "pas de correspondance trouvé"
    else :
        return tab_travail


