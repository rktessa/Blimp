# Blimp
A simple autonomous blimp project on Raspberry

## Cose importanti da modificare
La funzione Orientation initial devi modificarla per fare in modo che nei primi 10 secondi ti dia il valore di orientazione del blimp rispetto al reference frame assoluto degli UWB. Poi la fai andare all'inizio del codice e in questo modo ti ricavi una terna di valori di angoli di cui ruotare la lettura trovata poi nel while 1 del madgwick mentre fa andare la navigazione. 


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

- chiarire come salvare su csv/txt al meglio lo stream con i dati calcolati di posizione/sensori e valori del kalman

- implementare che il risultato del programma per il calcolo della traiettoria sia un csv/txt che viene letto e convertito in un data frame durante la navigazione. 

-si può lanciare un programma mentre sta andando il primo?, devo usare delle exception (?)

- come programmare input da joycon

- chiedere come fare tuning di Q a Santoro

- come inserire madgwick nel z per kalman, capire e inserire.

- scrivere sim blimp

- provare a vedere se kalman funziona


# PID

Da capire i valori massimi di forza generati dai motori, rivedi sto passaggio

Di conseguenza da definire il max signal da passare, oltre al quale il pid satura 

convertire target distance in metri  da centimetri

capire dove inserire i time counter per avere l'effettivosample rate del codice che serve in:
    - Kalman
    - PID
    - Madgwick


definire il dist_init

