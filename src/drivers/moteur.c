#include "moteur.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

extern struct k_mutex i2c_mutex;

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
#define MOTEUR_ADDR 0x14

#define CMD_WAKEUP  0x06
#define CMD_TRIGO   0x02
#define CMD_HORAIRE 0x03

// Moteur connécté sur le canal B du driver motor : B1 et B2
#define MOTOR_CHB 0x01 

void moteur_init(void) {
    if (!device_is_ready(i2c_dev)) {
        printk("ERREUR: I2C non pret pour le moteur\r\n");
        return;
    }
    
    uint8_t wakeup = CMD_WAKEUP;
    k_mutex_lock(&i2c_mutex, K_FOREVER);
    i2c_write(i2c_dev, &wakeup, 1, MOTEUR_ADDR);
    k_mutex_unlock(&i2c_mutex);
    k_msleep(50); 

    moteur_set_vitesse(0);
    //printk("Motor driver I2C (0x14) initialise (Moteur canal B)\r\n");
}

void moteur_set_vitesse(int vitesse) {
    uint8_t sens;
    
    if (vitesse > 60) vitesse = 60;
    if (vitesse < -60) vitesse = -60;

    if (vitesse > 0) {
        sens = CMD_TRIGO;
    } else {
        sens = CMD_HORAIRE;
        vitesse = -vitesse; 
    }

    // Trame I2C : {Sens, Moteur B (0x01), Vitesse}

    // On essaie de prendre le Mutex pendant max 10 ms
    uint8_t cmd[] = {sens, MOTOR_CHB, (uint8_t)vitesse};
    if (k_mutex_lock(&i2c_mutex, K_MSEC(10)) == 0) {
        // Si succès, on envoie la commande et on libère le Mutex
        i2c_write(i2c_dev, cmd, 3, MOTEUR_ADDR);
        k_mutex_unlock(&i2c_mutex);
    } 
}