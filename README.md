# Blimp
A simple autonomous blimp project on Raspberry
![alt text](http://url/to/blimp.png)
![alt text](http://url/to/blimp_2.png)
![alt text](http://url/to/support.png)

##  Funzione blimp_to_world_rf

La funzione blimp_to_world_rf crea un programma che stima con il madgwick per 10 secondi l'orientazione in cui si trova il dirigibile. In contemporanea con l'uwb viene misurata la posizione del blimp mentre si muove in linea retta nel laboratorio. La differenza delle 2 orientazioni calcolate è usata come off set per ruotare il sistema di riferimento del blimp nel absolute reference frame rappresentato dagli UWB. In questo modo posso pilotare poi per il resto del tempo il dirigibile usando angoli calcolati con il path planning nel global reference frame.

## PID
Usati per gestione pwm motori. Due motori laterali realizzano orientazione e navigazione, motore su asse Z mantiene quota impostata. 
## Trilateration e LLS e WLS

## Comunicazione e guida con tastiera


## Path planner e coordinate UWB in lab di meccatronica
Implementato path planner, prende in input mappa dell'ambiente disegnata e riscalata correttamente (1 pixel * 1 cm) e restituisce il percorso corrispondente, in step di 10 cm, che poi converto in metri per comodità del mio algoritmo

![Alt text](https://github.com/rktessa/Blimp/blob/main/phat_plannig_solution.png?raw=true "Scheme of the  Alghorithm ")

## Tester per spinta motori
In motor_test_final.py  implementato un semplice scipt per variare da terminale la pwm dei motori ed eseguire così un loro test molto semplice. 
Inoltre la prova con una prima elica toroidale ha dato degli ottimi risultati, è molto più silenziosa rispetto al modello di elica precendente.


## Programma per la misurazione
Protocollo di comunicazione molto basilare con tutte le funzioni che abbiamo per il rilevamento dei sensori del dirigibile. Usayo]]] **zeroMQ** per questo scopo. 



## MADGWICK
 Prove con diversi beta, non 1 che rende la misura troppo instabile, set a 0.35 ok, i valori oscillano in un intorno di 4 gradi circa. 
    Se si riesce ad aumentare sopra i 10 Hz la frequenza di campionamento questo estimatore diventa molto migliore. 



# LISTA DEI CODICI IN USO
## Aggiornato  18 Maggio 2023

Stiamo usando cose nella cartella V3:

- **calibrazione_madgwick.py** Stampa risultati di Madgwick e Mad mappato, per calibrare a esattamente 90 gradi le rotazioni.

- **calibrate_magnetometer.py** Acquisisce i dati grezzi del magnetometro per 60 secondi e fa Hard e Soft Iron calibration

- **blimp_class.py** Contiene tutte le classi in uso: Astar, PID,  Madgwick, Reject Outliers, RTDoA, calibration, print calibration, trilateration, PSI mapped ==> Da inserire Kalman
-  **lazy_client.py** codice che effettua tutte le misurazioni e le invia al computer

In Blimp_muletto_V6

- **lazy_server.py** contiene il codice di main_blimp.py, ma effettua i calcoli con i dati ricevuti dal blimp.

- **global.py** contiene i valori che vanno salvati globalmente per essere rimandati al blimp, le PWM dei motori alla fine

In Blimp class abbiamo ruotato gli assi dei dati ricevuti dalla imu in modo da passare da (x,yz)_IMU a (y,x,-z)_stanza e poi usando i valori stimati dal madgwick si calcola la rotazione giusta per le accelerazioni correggendo abbastanza bene i valori calcolati e riferendoli così al corretto sistema di riferimento.

- **quaternio.py** c'è una modifica nella conversione da quat a euler per evitare errori su arcsin

IN muletto_V6 
- c'è lazy server che computa i dati e c'è una versione anche con il kalman filter dentro
- nella versione con kalman sto acquisendo alla massima frequenza i dati, sono leggermente rallentati invece in lazyserver.py 


## Aggiornato 13 Aprile 2023

Cartella corrente Blimp_V4 cin dentro:

- **calibrazione_madgwick.py** Stampa risultati di Madgwick e Mad mappato, per calibrare a esattamente 90 gradi le rotazioni.

- **calibrate_magnetometer.py** Acquisisce i dati grezzi del magnetometro per 60 secondi e fa Hard e Soft Iron calibration

- **blimp_class.py** Contiene tutte le classi in uso: Astar, PID, Kalman, Madgwick, Reject Outliers, RTDoA, calibration, print calibration, trilateration, PSI mapped

- **main_blimp.py** In cui cerchiamo di far andare tutto il codice assieme. 

- **kalman_blimp.py** per testare l'uso di Kalman: Inizialmente stando fermo acquisisce orientazione rispetto al global RF e stima con una media la posizione di partenza. 
Poi 

Frequenze di campionamento dei sensori: 

Sonar = 2 Hz
Madgwick = real time dt del codice
UWB = il serial va a 916000 Hz
TdOA = 2Hz
IMU = al massimo della frequenza possibile


In Blimp_V2 è dove è rimasta la funzione blimp_to_world_RF **ORIGINALE**, anche nella stessa cartella trovi un Astar dentro che funzionava in teoria.


