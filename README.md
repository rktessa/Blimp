# Blimp
A simple autonomous blimp project on Raspberry

##  Funzione blimp_to_world_rf
In prova.py e in main_blimp_01.py è visibile la routine inziale del codice ora funzionante. 

La funzione blimp_to_world_rf crea un programma che stima con il madgwick per 10 secondi l'orientazione in cui si trova il dirigibile. In contemporanea con l'uwb viene misurata la posizione del blimp mentre si muove in linea retta nel laboratorio. La differenza delle 2 orientazioni calcolate è usata come off set per ruotare il sistema di riferimento del blimp nel absolute reference frame rappresentato dagli UWB. In questo modo posso pilotare poi per il resto del tempo il dirigibile usando angoli calcolati con il path planning nel global reference frame

 


## Lavori in corso
In the blimp_class file a repo of all the functions that might be used during blimp flight is under construction.

In file "madgwick.py" it can be found a working
implementation of the madgwick function.

In "kalman_blimp_test_01.py" I am developing the kalman filter for heading estimation of the vehicle during its motion. 

The main program then function in this way: 

![Alt text](https://github.com/rktessa/Blimp/blob/main/alg_flow.jpg?raw=true "Scheme of the  Alghorithm ") 

Important aspects to undestand for Kalman filter implementation:

- orientation of IMU (in particular due to gravity) impact Kalman? 

- calculation of sensor accuracy value for determining the variance of sensors. 

- smooth the value of Kalman filter, see RTS. 

- capire se la rootazione come la ho fatta va bene

- chiarire come salvare su csv/txt al meglio lo stream con i dati calcolati di posizione/sensori e valori del kalman ==> ispirarsi al codice visto e implementato in "path_planning.py" con le python cv per plottare il path del blimp

- implementare che il risultato del programma per il calcolo della traiettoria sia un csv/txt che viene letto e convertito in un data frame durante la navigazione. 

-si può lanciare un programma mentre sta andando il primo?, devo usare delle exception (?)


- Sistemare R e Q ==> R usando la varianza delle misure dei sensori, mentre Q capendo quanta affidabilità vogliamo dare al nostro modello

- Inserire le accelerazioni misurate ruotate nel sistema di riferimento global per il kalman filter

-


## PID

Da capire i valori massimi di forza generati dai motori, rivedi sto passaggio

Di conseguenza da definire il max signal da passare, oltre al quale il pid satura 

convertire target distance in metri  da centimetri

capire dove inserire i time counter per avere l'effettivo sample rate del codice che serve in:
    - Kalman
    - PID
    - Madgwick


definire il dist_init

Abbiamo spostato nel file **PID** la classe pid control che stiamo cercando di debuggare adesso. 
Il file corrente in uso è **main_blimp_02**



## Trilateration e LLS e WLS
Su suggerimento di Santoro una cosa interessante da fare  è quella di andare ad analizzare quanto inficiano i sensori con la loro precisione e con il loro numero in questa misura della posizione fatta
con il LS alghorithm. Lui mi diceva di partire da 4 ancore, il minimo per una misura in 3d e fare tot prove con una certa varianza e salvare tutti i risultati, poi cambiare la varianza e farne altre.
E con queste prove andare a vedere con un istogramma dove si poszionano i risultati. 

Poi di aggiungere sensori (tipo da 4 a 10 ancore) e ripetere così e tirare fuori una curva con sulla x
il numero di ancore e sulla y l'errore medio? e così vedere l'andamento. 
Penso possa essere utile anche per la relazione di fontanelli. 

## Comunicazione e guida con tastiera
Messo in funzione il codice implementato da Filippo, sarebbe interessante svilupparlo per sostiuire joycon con tastiera e poi per ottene le  misure ogni decimo di secondo tipo e fare un plot che si aggiorna sul pc con i dati live. 

==> Capire come implementare protocollo per trasmissione dei dati dal blimp al pc ogni secondo (?)


## Path planner e coordinate UWB in lab di meccatronica
Implementato path planner, prende in input mappa dell'ambiente disegnata e riscalata correttamente (1 pixel * 1 cm) e restituisce il percorso corrispondente, in step di 10 cm, che poi converto in metri per comodità del mio algoritmo

![Alt text](https://github.com/rktessa/Blimp/blob/main/phat_plannig_solution.png?raw=true "Scheme of the  Alghorithm ")

## Tester per spinta motori
In motor_test_final.py ho implementato un semplice scipt per variare da terminale la pwm dei motori ed eseguire così un loro test molto semplice. 
Inoltre la prova con una prima elica toroidale ha dato degli ottimi risultati, è molto più silenziosa rispetto al modello di elica precendente.


## Programma per la misurazione
Un'idea interessante per gestire tutti i dati dei sensori potrebbe essere quello di scrivere un programma che integra un protocollo di comunicazione molto basilare con tutte le funzioni che abbiamo per il rilevamento dei sensori del dirigibile. Potrei provare a usare **zeroMQ** per questo scopo. 

In Blimp_V3

## MADGWICK

    prove con diversi beta, non 1 che rende la misura troppo instabile, set a 0.35 ok, i valori oscillano in un intorno di 4 gradi circa. 
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


