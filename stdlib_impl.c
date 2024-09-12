#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>

#undef errno
extern int  errno;

// https://www.embecosm.com/appnotes/ean9/ean9-howto-newlib-1.0.html#id2719973
int
_write (int   file,
        char *buf,
        int   nbytes)
{
  int i;

  /* Output character at at time */
  for (i = 0; i < nbytes; i++)
    {
      // UART1_SendString (&buf[i], 1);
    }
        
  return nbytes;

} 

void *
_sbrk (int nbytes)
{
  static uint8_t heap[512];
  static uint16_t last = 0;
  if(last+nbytes < 512)
  {
    void* pointer = &heap[last];
    last+=nbytes;
    return pointer;
  }
  else
  {
    return  (void *) -1;
  }
}

int
_close (int   file)
{
  errno = EBADF;
  
  return -1;                    /* Always fails */
}

int
_lseek (int   file,
        int   offset,
        int   whence)
{
  return  0;

} 

int
_read (int   file,
       char *ptr,
       int   len)
{
  return  0;                            /* EOF */

} 

int
_fstat (int          file,
        struct stat *st)
{
  st->st_mode = S_IFCHR;
  return  0;

}

int
_isatty (int   file)
{
  return  1;

}

void Default_Handler(void);
void 
_exit (int  rc)
{
 Default_Handler();
 while(1){}
} 

int
_kill (int  pid,
       int  sig)
{
  errno = EINVAL;
  return -1;                    /* Always fails */

} 

int
_getpid ()
{
  return  1;                            /* Success */

}

void _init (void) {}
