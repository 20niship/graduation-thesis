#pragma once

#include <ctime>
#include <iomanip>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

inline bool isKeyPressed() {
  struct termios oldSettings, newSettings;
  tcgetattr(STDIN_FILENO, &oldSettings);
  newSettings = oldSettings;
  newSettings.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

  bool keyPressed = false;
  fd_set readSet;
  FD_ZERO(&readSet);
  FD_SET(STDIN_FILENO, &readSet);
  struct timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 0;

  if(select(STDIN_FILENO + 1, &readSet, NULL, NULL, &timeout) > 0) {
    char c;
    read(STDIN_FILENO, &c, 1);
    keyPressed = true;
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
  return keyPressed;
}
