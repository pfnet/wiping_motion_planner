#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import atexit
import os
import sys
import termios
import time
from select import select

import numpy as np
try:
    import cv2
except:
    cv2 = []



class KBHit:

    def __init__(self):
        '''Creates a KBHit object that you can call to do various keyboard things.
        '''

        if os.name == 'nt':
            pass

        else:

            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)

            # New terminal setting unbuffered
            self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)


    def set_normal_term(self):
        ''' Resets to normal terminal.  On Windows this is a no-op.
        '''

        if os.name == 'nt':
            pass

        else:
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)


    def getch(self):
        ''' Returns a keyboard character after kbhit() has been called.
            Should not be called in the same program as getarrow().
        '''

        s = ''
        return sys.stdin.read(1)


    def getarrow(self):
        ''' Returns an arrow-key code after kbhit() has been called. Codes are
        0 : up
        1 : right
        2 : down
        3 : left
        Should not be called in the same program as getch().
        '''

        if os.name == 'nt':
            msvcrt.getch() # skip 0xE0
            c = msvcrt.getch()
            vals = [72, 77, 80, 75]

        else:
            c = sys.stdin.read(3)[2]
            vals = [65, 67, 66, 68]

        return vals.index(ord(c.decode('utf-8')))


    def kbhit(self):
        ''' Returns True if keyboard character was hit, False otherwise.
        '''
        if os.name == 'nt':
            return msvcrt.kbhit()

        else:
            dr,dw,de = select([sys.stdin], [], [], 0)
            return dr != []


class KeyboardInput():
    '''
    Mimic a kbhit in python. The closest would be using the
    type "terminal", but this mode needs a terminal, and does not work with an IDE.
    This means that you cannot debug the code.

    If you want to debug the code, you can still use a replacement, by setting
    type "opencv".

    '''

    def __init__(self, type):

        self.type = type
        print("Keyboard input activated. Press ESC to stop.")

        if type == 'terminal':
            # this method only runs on the terminal/console. Cannot use an IDE for this
            # thus, you cannot debug the code with IDE.
            # this should be used for deployment, as it does not require graphics and can be
            # run as a docker container without any problems
            self.kb = KBHit()

        elif type == 'ide':
            # use opencv method such that IDE can be used to debug

            # import here and not at the start of the file because you dont want dockerized code to complain unless
            # you really need cv2 libraries.
            if cv2 == []:
                ValueError("You cannot use the option ide as the cv2 was not found. Use terminal option instead.")

            self.fake_img = np.zeros(shape=(40, 40))
            self.kb = []

        else:
            raise ValueError("\n\n******Invalid specification. Chose between terminal or ide******\n\n")


    def get_keyboard_with_cv2(self, v=[], kill_at_ESC = True):

        cv2.imshow('fake_window', self.fake_img)
        #cv2.moveWindow('fake_window', 200, 200)
        key = cv2.waitKey(1) & 0xFF

        if key != 255:
            key_char = chr(key)

            if ord(key_char) == 27:  # ESC
                a = 2
                if kill_at_ESC:
                    raise ValueError("\n\nEscape (Esc) key detected!!!")
                else:
                    key_char = 'ESC'

            return key_char


        return []

    def get_val(self, v=[], kill_at_ESC = True):

        c = []
        if self.type == 'terminal':
            if self.kb.kbhit():
                c = self.kb.getch()
                termios.tcflush(sys.stdin, termios.TCIOFLUSH)

                if ord(c) == 27:  # ESC
                    if kill_at_ESC:
                        raise ValueError("\n\nEscape (Esc) key detected!!!")
                    else:
                        c = 'ESC'

        if self.type == 'ide':
            c = self.get_keyboard_with_cv2(v=v, kill_at_ESC =  kill_at_ESC)


        return c


if __name__ == "__main__":

    if 0:
        type = 'terminal'
    else:
        type = 'ide'

    ky = KeyboardInput(type)

    t_lastprint = time.time()

    while 1:
        time0 = time.time()
        c = ky.get_val()

        if c != []:
            print(c)

        if time.time() - t_lastprint > 0.5:
            t_lastprint = time.time()
            print("Freq %.1f (Hz)" % (1.0 / (time.time() - time0)))