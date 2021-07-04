#include "terminal_getch.h"

// Functions to receive user input from console without echoing the input
void initTermios(void)
{
  tcgetattr(0, &old_);              /* grab old_ terminal i/o settings */
  current_ = old_;                   /* make new settings same as old_ settings */
  current_.c_lflag &= ~ICANON;      /* disable buffered i/o */
  current_.c_lflag &= ~ECHO;        /* set no echo mode */
  tcsetattr(0, TCSANOW, &current_); /* use these new terminal i/o settings now */
}

void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old_);
}

char getch(void)
{
  char ch;
  initTermios();
  ch = getchar();
  resetTermios();
  return ch;
}