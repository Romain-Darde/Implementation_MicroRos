#include "pid.h"

void pid_init(PID_Controller *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // On remet la mémoire à zéro au démarrage
    pid->somme_erreurs = 0.0f;
    pid->erreur_precedente = 0.0f;
}

int pid_calculer_commande(PID_Controller *pid, float consigne, float position_actuelle, float dt) {
    // Calcul de l'erreur brute
    float erreur = consigne - position_actuelle;
    
    // Correction pour le chemin le plus court entre -180 degrés et +180 degrés
    while (erreur > 180.0f)  erreur -= 360.0f;
    while (erreur < -180.0f) erreur += 360.0f;

    // Terme proportionnel
    float P = pid->kp * erreur;

    // Terme intégral avec Anti-Windup
    pid->somme_erreurs += erreur * dt;
    
    // Bridage de l'accumulation pour éviter l'emballement si le moteur se bloque
    if (pid->somme_erreurs > 500.0f) pid->somme_erreurs = 500.0f;
    if (pid->somme_erreurs < -500.0f) pid->somme_erreurs = -500.0f;
    
    float I = pid->ki * pid->somme_erreurs;

    // terme dérivé
    float vitesse_erreur = (erreur - pid->erreur_precedente) / dt;
    float D = pid->kd * vitesse_erreur;

    // Erreur gardé en mémoire pour la prochaine itération
    pid->erreur_precedente = erreur;

    // Calcul de la commande totale
    float commande_flottante = P + I + D;
    int commande = (int)commande_flottante;

    // Bridage de sécurité
    if (commande > 100) commande = 100;
    if (commande < -100) commande = -100;

    return commande;
}