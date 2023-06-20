import sys
import select

def input_with_timeout(prompt, timeout):
    print(prompt, end='', flush=True)
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.readline().rstrip()
    else:
        return None

while True:
    comando = input("Inserisci un comando (o 'stop' per terminare): ")

    if comando == 'stop':
        break  # Esci dal ciclo while esterno

    if comando == 'start':
        while True:
            # Esegui le azioni nel loop interno
            # ...

            nuovo_input = input_with_timeout("Inserisci un nuovo input (o 'stop' per terminare): ", timeout=0.1)

            if nuovo_input is None:
                # Nessun input ricevuto entro il timeout
                # Continua l'esecuzione del loop interno
                pass
            elif nuovo_input == 'stop':
                break  # Esci dal ciclo while interno
            else:
                # Esegui le azioni corrispondenti al nuovo input
                # ...

                # Stampa un messaggio o esegui altre azioni
                print("Nuovo input eseguito:", nuovo_input)
