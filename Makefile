#Makefile for gcc template
PROJECT = Level_Set
DEBUG=
 
R = ar
ARFLAGS = rv
GFLAGS =
GET = get
ASFLAGS =
MAS = mas
AS = as
FC = f77
CFLAGS = -Wall -g

CC = g++
#CC = cc
LDFLAGS = 
LD = ld
LFLAGS =
LEX = lex
YFLAGS =
YACC = yacc
LOADLIBS =
MAKE = make
MAKEARGS = 'SHELL=/bin/sh'
SHELL = /bin/sh
MAKEFLAGS = b
RANLIB      =   ranlib

IncludeDir =    -I. \
  -I/usr/include \
  -I/usr/local/include 
LibDir = -L. \
 -L/usr/lib\
  -L/usr/local/lib
LIBS = -lm

Objs =  RANGE.o \
 TRIANGLE.o \
 GRID.o \
 TRIANGULATED_SURFACE.o \
 tri2phi.o \
 FLOOD_FILL.o \
 LEVELSET_MAKER.o \
 LEVELSET.o

.cpp.o:
	$(CC) $(DEBUG) $(CFLAGS) -c $(IncludeDir) $< -o $@

.cc.o:
	$(CC) $(DEBUG) $(CFLAGS) -c $(IncludeDir) $< -o $@
 
$(PROJECT): $(Objs) 
	$(CC) $(DEBUG) -o   $@ $(LibDir) $(Objs) $(LIBS)

clean: 
	rm -f $(Objs)  $(PROJECT)
