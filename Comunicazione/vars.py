import time



if __name__ == "__main__":

    field = 0
    game = 0
    screen = 0
    ai = 0

    while True:
        field = field +1
        game = (game +1)*2
        screen = screen + 2
        ai = ai + 5
        time.sleep(2)