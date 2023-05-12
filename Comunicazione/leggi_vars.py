
from vars import field, game, screen, ai

from copy import copy, deepycopy

while True:

    my_field = copy(field)  # Use deepcopy instead if needed
    my_game = copy(game)
    my_screen = copy(screen)
    my_ai = copy(ai)

    print(ai, field, game,screen )