/**
 * The code in this file has been referenced from: https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux
 */ 
#ifndef TERMINAL_GETCH_H
#define TERMINAL_GETCH_H

#include <termios.h>
#include <stdio.h>

static struct termios old_, current_;

void initTermios(void);
void resetTermios(void);
char getch(void);

#endif