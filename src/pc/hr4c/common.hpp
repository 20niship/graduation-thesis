#pragma once

#include <sys/time.h>
#include <sys/types.h>
#include <iostream>

inline bool kbhit() {
  struct timeval tv;
  fd_set read_fd;

  tv.tv_sec  = 0;
  tv.tv_usec = 0;
  FD_ZERO(&read_fd);
  FD_SET(0, &read_fd);

  if(select(1, &read_fd, NULL, NULL, &tv) == -1) return 0;

  if(FD_ISSET(0, &read_fd)) return 1;

  return 0;
}

