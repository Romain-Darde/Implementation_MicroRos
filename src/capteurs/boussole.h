#ifndef BOUSSOLE_H
#define BOUSSOLE_H

// Initialise le magnétomètre et vérifie qu'il répond correctement
int boussole_init(void);

// Lit les données et renvoie l'angle en degrés
float boussole_get_angle(void);

#endif