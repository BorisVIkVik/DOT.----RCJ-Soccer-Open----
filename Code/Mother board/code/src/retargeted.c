#include <stdio.h>
#include <rt_misc.h>

#pragma import(__use_no_semihosting_swi)

#include <rt_sys.h>

extern void $Super$$_sys_open(void);

FILEHANDLE $Sub$$_sys_open(const char *name, int openmode)
{
 return 1; /* everything goes to the same output */
}

extern void $Super$$_sys_close(void);
int $Sub$$_sys_close(FILEHANDLE fh)
{
 return 0;
}

extern void $Super$$_sys_write(void);
int $Sub$$_sys_write(FILEHANDLE fh, const unsigned char *buf,
              unsigned len, int mode)
{
 //your_device_write(buf, len);
 return 0;
}

extern void $Super$$_sys_read(void);
int $Sub$$_sys_read(FILEHANDLE fh, unsigned char *buf,
             unsigned len, int mode)
{
 return -1; /* not supported */
}

//extern void $Super$$_ttywrch(void);
//void $Sub$$_ttywrch(int ch)
//{
// char c = ch;
// //your_device_write(&c, 1);
//}

extern void $Super$$_sys_istty(void);
int $Sub$$_sys_istty(FILEHANDLE fh)
{
 return 0; /* buffered output */
}

extern void $Super$$_sys_seek(void);
int $Sub$$_sys_seek(FILEHANDLE fh, long pos)
{
 return -1; /* not supported */
}

extern void $Super$$_sys_flen(void);
long $Sub$$_sys_flen(FILEHANDLE fh)
{
 return -1; /* not supported */
}

extern void $Super$$_sys_exit(void);
long $Sub$$_sys_exit(FILEHANDLE fh)
{
 return -1; /* not supported */
}
