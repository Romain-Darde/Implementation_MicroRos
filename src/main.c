#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <math.h>

// Include Micro-ROS 
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "drivers/moteur.h"
#include "capteurs/codeur.h"
#include "capteurs/boussole.h"
#include "controle/pid.h"

struct RobotState {
    float angle_actuel;   // Position actuelle de l'aiguille
    float consigne;       // Où l'on veut que l'aiguille pointe (Le Nord = 0.0)
};

// Initialisation de l'état du robot à 0
struct RobotState etat_robot = {0.0, 0.0};

// Création d'un PID global pour le contrôle du moteur
PID_Controller mon_pid;

K_MUTEX_DEFINE(robot_mutex);
K_MUTEX_DEFINE(i2c_mutex);

// Sémaphore pour synchroniser le démarrage du PID
K_SEM_DEFINE(sem_boussole, 0, 1);

// Définitions des piles pour les threads
// K_THREAD_STACK_DEFINE(thread_capteurs_stack, 1024);
// K_THREAD_STACK_DEFINE(thread_pid_stack, 1024);

// Micro-ROS : ile et données pour le thread ROS
// K_THREAD_STACK_DEFINE(thread_ros_stack, 4096);
// Réduction de la pile de 4096 à 2560
// K_THREAD_STACK_DEFINE(thread_ros_stack, 2560);

// // Définitions des piles ultra-compressées
// K_THREAD_STACK_DEFINE(thread_capteurs_stack, 512);
// K_THREAD_STACK_DEFINE(thread_pid_stack, 512);

// // Pile ROS réduite au minimum vital
// K_THREAD_STACK_DEFINE(thread_ros_stack, 2048);

// MARCHE POUR LA COMPILE
// Définitions des piles au minimum vital absolu
K_THREAD_STACK_DEFINE(thread_capteurs_stack, 384);
K_THREAD_STACK_DEFINE(thread_pid_stack, 384);

// Pile ROS légèrement compressée (1.5 Ko)
K_THREAD_STACK_DEFINE(thread_ros_stack, 1536);

// // On redonne de l'oxygène aux tâches
// K_THREAD_STACK_DEFINE(thread_capteurs_stack, 512);
// K_THREAD_STACK_DEFINE(thread_pid_stack, 512);

// // Pile ROS sécurisée
// K_THREAD_STACK_DEFINE(thread_ros_stack, 2048);

struct k_thread thread_ros_data;
k_tid_t thread_ros_id;

// Variables pour les structures des threads
struct k_thread thread_capteurs_data;
struct k_thread thread_pid_data;

// Variables pour les IDs des threads
k_tid_t thread_capteurs_id;
k_tid_t thread_pid_id;

// Thread de lecture des capteurs qui met à jour l'état du robot (angle actuel) en continu

void thread_capteurs_fn(void *arg1, void *arg2, void *arg3) {
    float angle_filtre = boussole_get_angle();
    int compteur_stabilisation = 0;

    while (1) {
        float nouvel_angle = boussole_get_angle();

        // Filtre pour lisser les lectures de la boussole

        // On calcule la différence entre la nouvelle mesure et notre valeur filtrée
        float erreur_angle = nouvel_angle - angle_filtre;

        // Normalisation : on s'assure d'emprunter le chemin le plus court.
        // Ex : si le filtre est à 350° et le capteur lit 10°, l'erreur est de +20° (et non -340°)
        if (erreur_angle > 180.0f) {
            erreur_angle -= 360.0f;
        } else if (erreur_angle < -180.0f) {
            erreur_angle += 360.0f;
        }

        // Application du filtre 
        // On intègre seulement 20% de la nouvelle mesure pour lisser les variations brusques de la boussole
        angle_filtre += 0.2f * erreur_angle;

        // On contraint le résultat final pour qu'il reste strictement dans la plage [0, 360[ degrés
        if (angle_filtre < 0.0f) {
            angle_filtre += 360.0f;
        } else if (angle_filtre >= 360.0f) {
            angle_filtre -= 360.0f;
        }

        // On verrouille le Mutex car le thread PID lit ces valeurs en même temps
        k_mutex_lock(&robot_mutex, K_FOREVER);
        etat_robot.angle_actuel = angle_filtre;
        etat_robot.consigne = 0.0f; // Nord fixe
        k_mutex_unlock(&robot_mutex);

        if (compteur_stabilisation < 40) {
            compteur_stabilisation++;
            if (compteur_stabilisation == 40) {
                // Au bout de 2 secondes, on réveille le thread PID 
                k_sem_give(&sem_boussole);
            }
        }

        k_msleep(50); // Tempo pour ne pas saturer le bus I2C et laisser le temps au PID de réagir
    }
}

// Thread de contrôle PID qui lit l'état du robot, calcule la commande et l'envoie au moteur

void thread_pid_fn(void *arg1, void *arg2, void *arg3) {
    // Fréquence d'exécution du PID (10 ms)
    float dt = 0.01f;
    // Constante de conversion : 207 ticks d'encodeur correspondent à un tour complet (360°)
    const float DEGREES_PER_TICK = 360.0f / 207.0f; // 207 ticks valeur trouvée expérimentalement

    //k_msleep(2000);
    k_sem_take(&sem_boussole, K_FOREVER);  // On attend que la boussole soit stable avant de démarrer le PID
    // Lecture sécurisée de l'angle initial du robot et de la position initiale de l'aiguille pour calculer l'offset nord
    k_mutex_lock(&robot_mutex, K_FOREVER);
    float angle_robot_initial = etat_robot.angle_actuel;
    k_mutex_unlock(&robot_mutex);

    // Récupération de la position de départ de l'aiguille via le codeur
    float angle_aiguille_initial = (float)codeur_get_ticks() * DEGREES_PER_TICK;

    // Ramène l'angle de l'aiguille strictement entre [0, 360[ degrés
    angle_aiguille_initial = fmodf(angle_aiguille_initial, 360.0f);
    if (angle_aiguille_initial < 0.0f) {
        angle_aiguille_initial += 360.0f;
    }

    // Calcul de l'Offset : On fige la relation entre l'aiguille et le Nord au démarrage.
    // (Idéalement l'aiguille pointe vers le Nord avant d'allumer)
    float offset_nord = angle_robot_initial + angle_aiguille_initial;
    offset_nord = fmodf(offset_nord, 360.0f);
    if (offset_nord < 0.0f) {
        offset_nord += 360.0f;
    }

    while (1) {
        // Lecture de la position actuelle du robot (boussole)
        k_mutex_lock(&robot_mutex, K_FOREVER);
        float angle_robot = etat_robot.angle_actuel;
        k_mutex_unlock(&robot_mutex);

        // Récupération de la position actuelle de l'aiguille via le codeur
        float angle_aiguille = (float)codeur_get_ticks() * DEGREES_PER_TICK;
        angle_aiguille = fmodf(angle_aiguille, 360.0f);
        if (angle_aiguille < 0.0f) {
            angle_aiguille += 360.0f;
        }

        // Calcul dynamique
        // L'aiguille doit compenser le mouvement du robot pour rester pointée vers le Nord
        float cible_aiguille = offset_nord - angle_robot;

        // Le correcteur calcule la commande nécessaire pour réduire l'erreur entre la cible et la position actuelle
        int commande = pid_calculer_commande(&mon_pid, cible_aiguille, angle_aiguille, dt);
        
        // Envoi de la commande au driver moteur
        moteur_set_vitesse(commande);

        k_msleep(10); // Tempo
    }
}

// Ancienne version du main pour tester le codeur et le moteur séparément

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

// // Signal A du codeur sur D2 (pin 10) pour capter les impulsions du moteur et compter les tours. Moteur connecté sur B1 et B2 du driver motor (Adresse I2C 0x14)
// // Signal B du codeur sur D3 (pin 3) pour détecter le sens de rotation du moteur (non utilisé dans ce code, mais peut être utile pour des améliorations futures)
// static const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));
// static const struct device *gpiob = DEVICE_DT_GET(DT_NODELABEL(gpiob));

// #define PIN_A 10 // D2 -> PA10
// #define PIN_B 3  // D3 -> PB3

// static struct gpio_callback codeur_cb;
// volatile int compte = 0;

// void codeur_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
//     compte++; 
// }

// Thread Micro-ROS
void thread_ros_fn(void *arg1, void *arg2, void *arg3) {
    // Initialisation de la mémoire ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t publisher;
    std_msgs__msg__Float32 msg;

    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        k_msleep(100); 
    }

    // Création infra ROS (Support et Nœud)
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "noeud_nucleo_boussole", "", &support);

    // Création du Publisher (sur le Topic "/robot/angle_actuel")
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/robot/angle_actuel");

    // Boucle infinie de publication (Toutes les 100 ms)
    while (1) {
        // On lit l'angle avec le Mutex
        k_mutex_lock(&robot_mutex, K_FOREVER);
        float angle_a_envoyer = etat_robot.angle_actuel;
        k_mutex_unlock(&robot_mutex);

        // On met la valeur dans le message ROS
        msg.data = angle_a_envoyer;

        // On publie le message sur le Topic
        rcl_publish(&publisher, &msg, NULL);

        k_msleep(100); 
    }
}

int main(void) {
    // Test Phase 1
    // while (1) {

    //     // Test GPIO (Nouveau Codeur sur D2 / PA10)
    //     if (device_is_ready(gpioa)) {
    //         // On utilise gpioa partout au lieu de gpiob
    //         gpio_pin_configure(gpioa, PIN_A, GPIO_INPUT | GPIO_PULL_UP);
    //         gpio_pin_interrupt_configure(gpioa, PIN_A, GPIO_INT_EDGE_RISING);
    //         gpio_init_callback(&codeur_cb, codeur_handler, BIT(PIN_A));
    //         gpio_add_callback(gpioa, &codeur_cb);
            
    //         printk("OK: Codeur pret sur D2 (PA10)\n");
    //         k_msleep(2000);
    //     }

    //     // Test I2C (IMU 0x69 et Magneto 0x0C)
    //     if (device_is_ready(i2c_dev)) {
    //         uint8_t id_imu = 0, id_mag = 0;
    //         i2c_reg_read_byte(i2c_dev, 0x69, 0x75, &id_imu);
    //         i2c_reg_read_byte(i2c_dev, 0x0C, 0x00, &id_mag);
            
    //         printk("OK: IMU(0x69) ID=0x%x | Mag(0x0C) ID=0x%x\n", id_imu, id_mag);
    //         k_msleep(2000);
    //     } else {
    //         printk("ERREUR: Bus I2C non pret\n");
    //         k_msleep(2000);
    //     }

    //     // Test du codeur
    //     printk("Compteur codeur: %d\n", compte);
    //     k_msleep(2000);
    // }

    // Ancien code test moteur et codeur séparément avant de passer à la phase 2

    // k_msleep(2000);
    // printk("--- TEST MOTEUR ET CODEUR---\n");

    // // Initialisation Codeur
    // if (device_is_ready(gpioa)) {
    //     gpio_pin_configure(gpioa, PIN_A, GPIO_INPUT | GPIO_PULL_UP);
    //     gpio_pin_interrupt_configure(gpioa, PIN_A, GPIO_INT_EDGE_RISING);
    //     gpio_init_callback(&codeur_cb, codeur_handler, BIT(PIN_A));
    //     gpio_add_callback(gpioa, &codeur_cb);
    //     printk("Codeur ok sur D2\n");
    // }

    // // Initialisation Moteur
    // moteur_init();
    // k_msleep(3000);

    // while (1) {
    //     printk("\n[1] Ordre: Vitesse +80. Compteur actuel: %d\r\n", compte);
    //     moteur_set_vitesse(80);
    //     k_msleep(500);
    // }
    // return 0;

    //printk("Démarrage\n");

    // Test debug I2C pour vérifier que la boussole répond avant de lancer le reste du code
    // printk("TEST I2C\n");
    // uint8_t data;
    // for (uint8_t i = 1; i < 127; i++) {
    //     if (i2c_reg_read_byte(i2c_dev, i, 0x00, &data) == 0) {
    //         printk("Appareil trouve a l'adresse: 0x%02x\n", i);
    //     }
    // }

    // Initialisation hardware (I2C, GPIO, Moteur, Codeur, Boussole)
    // k_msleep(3000);
    // printk("Initialisation hardware\n");

    k_msleep(3000); // Laisser le temps d'ouvrir le terminal après le reset
    moteur_init();
    codeur_init();
    boussole_init();

    k_msleep(500);
    k_mutex_lock(&robot_mutex, K_FOREVER);
    float angle_depart = etat_robot.angle_actuel;
    k_mutex_unlock(&robot_mutex);

    // Test de la boussole pour vérifier qu'elle répond avant de lancer le reste du code
    // if (boussole_init() != 0) {
    //     printk("ERREUR BOUSSOLE\n");
    // } else {
    //     printk("Boussole OK !\n");
    // }

    // Valeurs du PID ordre : Kp=1.5, Ki=0.1, Kd=0.5
    pid_init(&mon_pid, 1.5, 0.0, 0.0);

    thread_capteurs_id = k_thread_create(&thread_capteurs_data, thread_capteurs_stack, 
                                         K_THREAD_STACK_SIZEOF(thread_capteurs_stack), 
                                         thread_capteurs_fn, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
    k_thread_start(thread_capteurs_id);

    thread_pid_id = k_thread_create(&thread_pid_data, thread_pid_stack, 
                                    K_THREAD_STACK_SIZEOF(thread_pid_stack), 
                                    thread_pid_fn, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    k_thread_start(thread_pid_id);

    thread_ros_id = k_thread_create(&thread_ros_data, thread_ros_stack, 
                                    K_THREAD_STACK_SIZEOF(thread_ros_stack), 
                                    thread_ros_fn, NULL, NULL, NULL, 8, 0, K_NO_WAIT);
    k_thread_start(thread_ros_id);

    while (1) {
        // float a, c;
        // k_mutex_lock(&robot_mutex, K_FOREVER);
        // a = etat_robot.angle_actuel;
        // c = etat_robot.consigne;
        // k_mutex_unlock(&robot_mutex);

        // printk("Cible: %.1f | Actuel: %.1f | Erreur: %.1f\r\n", c, a, c - a);
        k_msleep(2000); // Tempo pour avoir le temps de lire les valeurs dans le terminal
    }

    return 0;
}