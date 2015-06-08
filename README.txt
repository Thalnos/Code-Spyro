De map Image bevat de complete Linux-image voor de Galileo. De inhoud van deze map wordt in de root map van een micro-SD kaart gekopieerd.
Bij opstarten van de Galileo wordt het besturingssysteem gestart. Een script loopt bij start en stelt een Adhoc netwerk op. Dit netwerk heeft SSID RobotNetwerk.
Het statische IP-adres van de Galileo wordt ingesteld op 192.168.1.2 (gateway 192.168.1.0). Connectie via SSH is mogelijk (bv met Putty).
Het python-programma van de Galileo wordt ook gestart. Indien op de Galileo geprogrammeerd wordt, moet best eerst dit proces gestopt worden, aangezien dit maximale prioriteit heeft.
Dit kan via het commando top om het PID te kennen, vervolgens het commando kill PID om het proces te stoppen.
Het opstartscript bevindt zich onder /pythonprogrammas. Het script heet startup.sh.
De code die uitgevoerd wordt op de Galileo heet robotsocketsplinenewRLS.py eveneens in dezelfde map.
Deze code en het opstartscript Zijn nog eens apart terug te vinden in de map Code/Galileo.
De map Code/PC bevat de pyton code die draait op de PC. Deze geeft een interface met de robot voor experimenten en de loopbeweging in te stellen.
De PC connecteert aan de hand van het IP adres van de Galileo (192.168.1.2).