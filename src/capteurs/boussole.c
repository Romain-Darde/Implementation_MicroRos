#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include "boussole.h"

extern struct k_mutex i2c_mutex;

#define AK09918_ADDR 0x0C
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

int boussole_init(void) {
    if (!device_is_ready(i2c_dev)) {
        printk("Boussole I2C pas prête\n");
        return -1;
    }

    k_mutex_lock(&i2c_mutex, K_FOREVER);
    
    // On sort l'IMU du mode sommeil
    i2c_reg_write_byte(i2c_dev, 0x69, 0x6A, 0x00);
    i2c_reg_write_byte(i2c_dev, 0x69, 0x6B, 0x00);
    k_msleep(10); // Tempo pour être sûr que l'IMU est bien réveillée
    
    // Bypass registre INT_PIN_CFG (0x37), on met le bit 1 à 1 (0x02)
    i2c_reg_write_byte(i2c_dev, 0x69, 0x37, 0x02);
    k_msleep(10); // Tempo pour être sûr que le bypass est bien activé
    
    // Démarrage boussole
    int ret = i2c_reg_write_byte(i2c_dev, 0x0C, 0x31, 0x08);
    
    k_mutex_unlock(&i2c_mutex);
    
    if (ret != 0) {
        printk("Erreur Boussole I2C (0x0C)\n");
    } else {
        printk("Succès Boussole initialisee \n");
    }
    
    k_msleep(50);
    return ret;
}

float boussole_get_angle(void) {
    static float derniere_valeur = 0.0f; // Garde la valeur en mémoire
    uint8_t data[9]; // de 0x10 à 0x18 pour les axes X, Y, Z et le statut

    if (!device_is_ready(i2c_dev)) {
        return derniere_valeur;
    }

    // On essaie de prendre le Mutex pendant max 10 ms
    if (k_mutex_lock(&i2c_mutex, K_MSEC(10)) != 0) {
        // Mutex bloqué, on abandonne pour ce cycle et on renvoie l'ancienne valeur
        return derniere_valeur; 
    }
    
    int ret = i2c_burst_read(i2c_dev, AK09918_ADDR, 0x10, data, 9);
    k_mutex_unlock(&i2c_mutex);
    
    // Si la lecture échoue, on renvoie la dernière valeur connue
    if (ret != 0) return derniere_valeur;

    int16_t x = (int16_t)(data[2] << 8 | data[1]);
    int16_t y = (int16_t)(data[4] << 8 | data[3]);

    float angle = atan2f((float)y, (float)x) * 180.0f / 3.14159f;
    if (angle < 0.0f) {
        angle += 360.0f;
    }

    derniere_valeur = angle; // Maj de la dernière valeur connue
    return angle;
}
