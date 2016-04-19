#!/usr/bin/python

import curses
import time

# start curses
stdscr = curses.initscr()
curses.cbreak()
curses.noecho()
stdscr.keypad(1)

def draw_debug(chDebug):
    height,width = stdscr.getmaxyx()
    # place this box between x = 0.2, 0.8, y = 0.1, 0.4
    yl = int(0.1*height)
    yh = int(0.4*height)
    xl = int(0.05*width)
    xh = int(0.95*width)
    stdscr.addstr(yl,xl,"x")
    stdscr.addstr(yl,xh,"x")
    stdscr.addstr(yh,xl,"x")
    stdscr.addstr(yh,xh,"x")
    stdscr.refresh()

def draw_gps_recv(stringIn=None):
    height,width = stdscr.getmaxyx()
    # place on the last line
    y = height
    x = width
    if stringIn is not None:
        # draw the line
        try:
            stdscr.addstr(y,x,".")
            stdscr.refresh()
        except curses.error:
            pass
    return y

try:
    height,width = stdscr.getmaxyx()
    '''num = min(height,width)
    for x in range(num):
        stdscr.addch(x,x,'X')
    '''
    draw_debug("")
    draw_gps_recv("COMM GPS: x")
    # Print Hello World along the top
    stdscr.addstr(0,10,"Hello World")
    while True:
        stdscr.refresh()
        pass
except KeyboardInterrupt:
    # cleanup curses
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()
