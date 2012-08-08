#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([Parallel RRT*], [1.0], [jeffi@cs.unc.edu])
AC_CONFIG_SRCDIR([src/prrts.c])
AC_CONFIG_HEADERS([config.h])

CFLAGS="-Wall -O3"

dnl AC_ARG_ENABLE(openmp, AS_HELP_STRING([--disable-openmp], [Do not use OpenMP for multi-threaded parallelism]), [ac_cv_use_omp=$enableval], [ac_cv_use_omp=yes])
dnl if test "x$ac_cv_use_omp" = "xyes" ; then
dnl    AC_DEFINE(USE_OMP, 1, [Enable OpenMP support])
dnl    AX_OPENMP(,AC_MSG_ERROR([Compiler does not support OpenMP]))
dnl    CFLAGS="$CFLAGS $OPENMP_CFLAGS"
dnl fi

dnl AC_ARG_ENABLE(pthreads, AS_HELP_STRING([--enabled-pthreads], [Use pthreads for multi-threaded parallelism], [USE_PTHREADS=1]))

# AC_ARG_ENABLE([pthreads], AS_HELP_STRING([--enable-pthreads], [Use pthreads for parallelism]))
# AC_ARG_ENABLE([openmp], AS_HELP_STRING([--enable-openmp], [Use openmp for parallelism (preferred)]))
dnl AC_ARG_WITH([openmp], [AS_HELP_STRING([--without-openmp],
dnl                       [Do not use openmp for parallel sections])],
dnl   [],
dnl   [with_openmp=yes])
                  
dnl AS_IF([test "x$with_openmp" != xno],
dnl   [AC_CHECK_LIB([openmp], [main],
dnl     [AC_DEFINE([USE_OPENMP], [1], [Define if you have openmp])],
dnl     [AC_MSG_FAILURE([openmp test failed (--without-openmp to disable)])],
dnl     [-lncurses])])

AC_ARG_ENABLE([crc], AS_HELP_STRING([--enable-crc], [Enable runtime CRC checks of data-structures (useful only for debugging)]))
AS_IF([test "x$enable_crc" = "xyes"], [AC_DEFINE([CHECK_CRCS], [1], [Check CRCs of data-structures])])

# Checks for programs.
AC_PROG_CC

# Checks for libraries.
AC_OPENMP

# Checks for header files.
AC_FUNC_ALLOCA
AC_CHECK_HEADERS([float.h limits.h stddef.h stdint.h stdlib.h string.h sys/param.h sys/time.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT64_T
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_CHECK_FUNCS([clock_gettime gettimeofday memset pow sqrt strtol])

AC_CONFIG_FILES([src/Makefile Makefile])
AC_OUTPUT
