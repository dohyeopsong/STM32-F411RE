/*
 * syscalls.c
 *
 *  Created on: Apr 2, 2026
 *      Author: dohyeopsong
 */


#include <sys/stat.h>

__attribute__((weak)) int _close(int fd) { return -1; }
__attribute__((weak)) int _lseek(int fd, int offset, int whence) { return -1; }
__attribute__((weak)) int _read(int fd, char *buf, int count) { return -1; }
__attribute__((weak)) int _write(int fd, char *buf, int count) { return count; }
__attribute__((weak)) int _fstat(int fd, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
__attribute__((weak)) int _isatty(int fd) { return 1; }
__attribute__((weak)) int _getpid(void) { return 1; }
__attribute__((weak)) int _kill(int pid, int sig) { return -1; }
