#ifndef PID_H
#define PID_H

// Structure : état du correcteur
typedef struct {
    float kp;
    float ki;
    float kd;
    float somme_erreurs;
    float erreur_precedente;
} PID_Controller;

// Fonction pour régler les gains au démarrage
void pid_init(PID_Controller *pid, float kp, float ki, float kd);

// Fonction pour calculer la commande à envoyer au moteur
int pid_calculer_commande(PID_Controller *pid, float consigne, float position_actuelle, float dt);

#endif