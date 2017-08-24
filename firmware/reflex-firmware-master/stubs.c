#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include "console.h"

/* UART DEBUGGING
*     true if you want to debug via UART, using the 4 pins on the brainboard, implies in higher latency
*     false, if you want a faster normal functioning
*/
#define DEBUG_VIA_UART false

extern int _end;

caddr_t _sbrk(int incr)
{
  static unsigned char *heap = NULL ;
  unsigned char *prev_heap ;
  if ( heap == NULL )
    heap = (unsigned char *)&_end ;
  prev_heap = heap;
  heap += incr ;
  return (caddr_t) prev_heap ;
}

int _kill(__attribute__((unused)) int pid, 
          __attribute__((unused)) int sig) { return -1; }
void _exit(__attribute__((unused)) int status) { while (1) {} } // spin...
int _getpid() { return 1; }

int _write(__attribute__((unused)) int fd, const void *buf, size_t count)
{
  #if DEBUG_VIA_UART == true
    consolePrint((uint8_t *)buf, count);
  #endif
  return count;
}
int _close(__attribute__((unused)) int fd) { return -1; }
int _fstat(__attribute__((unused)) int fd, 
           __attribute__((unused)) struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}
int _isatty(__attribute__((unused)) int fd) { return 1; }
off_t _lseek(__attribute__((unused)) int fd, 
             __attribute__((unused)) off_t offset, 
             __attribute__((unused)) int whence) { return 0; }
ssize_t _read(__attribute__((unused)) int fd, 
              __attribute__((unused)) void *buf, 
              __attribute__((unused)) size_t count) { return 0; }

struct __FILE { int handle; };
FILE __stdout;
FILE __stderr;
int fputc(__attribute__((unused)) int ch, __attribute__((unused)) FILE *f)
{
  return 0;
}
void _ttywrch(__attribute__((unused)) int ch) { }

