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


- chiedere come fare tuning di Q a Santoro

- come inserire madgwick nel z per kalman, capire e inserire. ==> più che altro per il fatto che la misura del ultrasuonin è relativa al fondo. Possiamo anche non fare sensor fusion per forza. 

- scrivere sim blimp

- provare a vedere se kalman funziona


## PID

Da capire i valori massimi di forza generati dai motori, rivedi sto passaggio

Di conseguenza da definire il max signal da passare, oltre al quale il pid satura 

convertire target distance in metri  da centimetri

capire dove inserire i time counter per avere l'effettivo sample rate del codice che serve in:
    - Kalman
    - PID
    - Madgwick


definire il dist_init



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

