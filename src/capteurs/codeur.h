#ifndef CODEUR_H
#define CODEUR_H

// Initialise les GPIO et les interruptions
void codeur_init(void);

// Renvoie la valeur actuelle du compteur
int codeur_get_ticks(void);

// Remise à zéro si besoin
void codeur_reset(void);

#endif