include $(top_srcdir)/Makefile.common

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/../common $(SOAPYSDR_CFLAGS)
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(SOAPYSDR_CFLAGS)


noinst_HEADERS = soapy_trx.h

noinst_LTLIBRARIES = libdevice.la

libdevice_la_SOURCES = soapy_trx.cpp
libdevice_la_LIBADD = $(top_builddir)/Transceiver52M/device/common/libdevice_common.la
