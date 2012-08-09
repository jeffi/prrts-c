#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([Parallel RRT*], [1.0], [jeffi@cs.unc.edu])
AC_CONFIG_SRCDIR([src/prrts.c])
AC_CONFIG_HEADERS([config.h])

# LIBS="$LIBS -lm -lc -lpthread"

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
AC_CHECK_LIB([rt], [clock_gettime], [LIBS="$LIBS -lrt"])
AC_CHECK_FUNCS([gettimeofday memset pow sqrt strtol])

AC_CONFIG_FILES([src/Makefile Makefile])
AC_OUTPUT
