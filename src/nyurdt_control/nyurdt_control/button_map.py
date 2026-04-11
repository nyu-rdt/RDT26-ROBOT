'''
button_map.py

Contains enum mapping from button name to pygame button value constant
'''

from enum import Enum

class ButtonMap(Enum):
    A = 0
    B = 1
    Y = 2
    X = 3
    LB = 4
    RB = 5
    WINDOW = 6
    MENU = 7
    XBOX = 8
    LEFT_ANALOG = 9
    RIGHT_ANALOG = 10
    EXIT = 11