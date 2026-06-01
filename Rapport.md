 
 # Rapport Implementation Micro-ROS

 ## Phase 3 : Preuve de Concept (PoC) Micro-ROS

 ### 1. Introduction et Objectif de la PoC

 L'objectif de cette troisième phase est de faire évoluer notre système embarqué (développé en Phase 2) vers un système configuré avec Micro-ROS.

 Le but est de réaliser une Preuve de Concept (PoC) démontrant que le microcontrôleur STM32 Nucleo-F103RB est capable de devenir un nœud ROS 2. Le but final est de transmettre les données d'asservissement en temps réel au PC via une communication série, afin de créer un "Jumeau Numérique" de la boussole visualisable dans l'outil de simulation RViz.

 ### 2. Choix Architecturaux et Méthodologie d'Intégration

 Afin de ne pas compromettre le code de production existant pour la phase 2, j’ai préféré créer ce nouveau dépôt Git (Implementation_MicroRos) pour cette dernière phase.

 Au lieu d'importer les  bibliothèques C++ de ROS 2 dans le projet Zephyr (ce qui a été mon premier essai infructueux). J’ai utilisé le module officiel micro_ros_zephyr_module (branche Humble, pour correspondre à la distribution ROS 2 de mon ordinateur) comme base de travail. Le code de la phase 2 (Capteurs, Moteurs, PID) y a ensuite été copié.

 Gestion de la mémoire sous Zephyr (prj.conf) : La pile Micro-ROS (RCL, RCLC, Micro XRCE-DDS) nécessitant des allocations dynamiques importantes, la configuration de Zephyr a été modifiée pour activer le support C++, les APIs POSIX, et allouer un Heap mémoire suffisant tout en respectant les limites de la RAM de la STM32 (20 Ko).

 ### 3. Implémentation du Nœud ROS sous Zephyr

 ### Tableau de Bord d’implémentation et erreurs rencontrées :

 Le fichier app.overlay par défaut du module Micro-ROS forçait l'utilisation d'un périphérique USB natif inexistant sur notre configuration. Je l’ai donc supprimé pour forcer Zephyr à utiliser la communication série standard via le ST-Link.

 Il m’a aussi fallu installer aussi cette librairie qui me manquait :

 ```bash
 pip install catkin_pkg empy lark
 ```
 ### Désactivation de ROS2 
 
 Il m’a aussi fallu désactiver temporairement ROS2 de mon ordinateur car j’avais un conflit d'environnement et cross-compilation:

 Lors des premières tentatives de compilation, le processus échouait systématiquement (erreurs sur builtin_interfaces ou conflits d'architecture 64-bit/32-bit).
 L'origine du problème provenait de mon environnement de développement : l'outil de construction de ROS 2 (colcon), exécuté par Zephyr en arrière-plan, scannait les variables d'environnement de mon ordinateur (Ubuntu). Il incluait alors les bibliothèques natives du PC (définies dans ~/.bashrc via /opt/ros/humble), corrompant ainsi la compilation croisée (cross-compilation) destinée au processeur ARM de la STM32.
 Les variables systèmes liées à ROS (AMENT_PREFIX_PATH, PYTHONPATH, LD_LIBRARY_PATH) ont dû être effacées du terminal avant la compilation pour forcer l'outil à générer des binaires exclusivement dédiés à la cible embarquée.

 Alors j’ai exécuté ces commandes:

 ```bash
 unset AMENT_PREFIX_PATH
 unset CMAKE_PREFIX_PATH
 unset ROS_DISTRO
 unset ROS_VERSION
 ```

 Cela n’a pas suffi alors temporairement, j’ai modifié le fichier bash de mon terminal et commenté la ligne suivante:

 ```bash
 gedit ~/.bashrc

 export TURTLEBOT3_MODEL=waffle
 # source /opt/ros/humble/setup.bash
 export LINOROBOT2_BASE=2wd
 export PATH=~/.local/bin:$PATH

 ```

 Dans un tout nouveau terminal cette fois-ci:

 ```bash
 rm -rf ~/Documents/Implementation_MicroRos/build ~/Documents/Implementation_MicroRos/modules/libmicroros/micro_ros_dev ~/Documents/Implementation_MicroRos/modules/libmicroros/micro_ros_src

 cd ~/Documents/Implementation_MicroRos
 source ~/zephyrproject/.venv/bin/activate
 source ~/zephyrproject/zephyr/zephyr-env.sh
 west build -p always -b nucleo_f103rb
 ```
 ### Problème de RAM disponible
 
 Après avoir passé cette erreur un autre point bloquant a été la RAM de la carte Nucleo, voici les erreurs que j'obtenais dans le terminal:

 ```text
 region 'RAM' overflowed by 20936 bytes
 section 'bss' will not fit in region 'RAM'
 ```

 Car la carte Nucleo STM32F103RB ne dispose que de 20 Ko de RAM. Avec ses paramètres par défaut, Micro-ROS provisionne d'importants buffers de fiabilité et d'historique, exigeant près de 40 Ko de RAM, ce qui provoque un dépassement critique ("overflow") de la section .bss.

 Puisque le robot n'a besoin que d'un seul Nœud et d'un seul Publisher (pour envoyer l'angle nécessaire à la boussole), nous allons désactiver tout le reste pour pouvoir mettre Micro-ROS dans les 20 Ko de la STM32.

 J’ai modifié la ligne de définition du thread ROS et réduit sa pile à 2560.

 Mais encore :

 ```text
 section 'noinit' will not fit in region 'RAM'
 region 'RAM' overflowed by 6272 bytes
 ```

 J’ai dû avoir recours à un bridage structurel de Micro-ROS (prj.conf) : j’ai strictement limitée au besoin de la PoC : 1 seul Nœud, 1 seul Publisher, et 0 Subscriber/Service. De plus, les tampons d'historique réseau (RMW_HISTORIC et XRCE_DDS_HISTORIC) ont été abaissés à 1 seul message, supprimant toute allocation mémoire superflue.

 Compression du Heap et des Threads Zephyr : L'allocation dynamique globale (Heap Mem Pool) a été drastiquement abaissée à 2 Ko. En parallèle, les piles d'exécution (stacks) des threads applicatifs ont été ajustées au plus juste : 512 octets pour la scrutation des capteurs et le calcul du PID, et 2048 octets pour le thread de communication ROS.

 Même après avoir fait rentrer le système dans la RAM, je me suis heurté à un crash silencieux de la carte et à une erreur de compilation persistante sur l'API POSIX (_SC_ADVISORY_INFO). Le crash silencieux était dû à un Buffer Overflow : lors de mes réductions de mémoire, j'avais diminué la taille réservée pour la pile du thread ROS, mais j'avais laissé l'ancienne taille "en dur" (4096 octets) dans l'appel de la fonction k_thread_create. J'ai sécurisé cela en remplaçant la valeur numérique par la macro Zephyr K_THREAD_STACK_SIZEOF().
 Concernant l'erreur de compilation, elle provenait d'un conflit connu sous Zephyr 3.7 : la librairie C par défaut (Picolibc) entre en conflit avec la norme POSIX requise par Micro-ROS. Pour résoudre ce problème définitivement, j'ai forcé Zephyr à utiliser la librairie classique et légère "Newlib Nano" dans mon fichier de configuration (CONFIG_NEWLIB_LIBC_NANO=y), suivi d'une destruction totale du cache de compilation (rm -rf build ...) pour appliquer le changement.

  ### Compilation réussie et test agent Micro-Ros

 Une fois la compilation réussie dans un autre terminal :

 ```bash
 sudo micro-ros-agent serial --dev /dev/ttyACM0 -v6
 ```

 Exemple de log obtenu :

 ```text
 [1779137832.079541] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
 [1779137832.080237] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 6
 ```

Bien que l'agent soit sur écoute sur le PC, la carte Nucleo reste complètement muette. Même en appuyant plusieurs fois sur le bouton RESET de la carte, aucune communication ne s'établit.

J'ai tenté plusieurs pistes pour débloquer la situation :

Désactivation de la console Zephyr : J'ai complètement coupé les logs textes (printk) de l'OS dans le prj.conf pour m'assurer qu'ils ne venaient pas parasiter et corrompre les paquets de Micro-ROS sur le port série.

Ajustement de la RAM au runtime : J'ai supposé que la compression très importante de la mémoire provoquait un crash de la carte au moment d'exécuter la fonction de ping. J'ai donc essayé de redonner un peu d'espace dynamique au thread ROS et au Heap (autour de 2048 octets), mais en essayant cette solution, il m'a été impossible de réussir à faire passer la compilation.

Malgré ces ajustements et de nombreux resets, la carte n'a jamais pu établir la session avec l'agent.

 ### 4. Conclusion de la PoC
   
La limite physique des 20 Ko de RAM de la Nucleo est trop restrictive pour l'exécution d'un noeud Micro-Ros. Le middleware de communication réseau a besoin de plus de mémoire dynamique pour pouvoir écrire ses paquets et établir une connexion.
