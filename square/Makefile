#
# The Compiler
#
CC      = gcc
LD      = ${CC}
#SMR     = /usr/local/smr
SMR     = $(shell if [ -f "/etc/redhat-release" ]; then echo "/home/thieden/Desktop/autonomous-robot-systems/mobotware-3.1125/build/"; elif [ -f "/etc/debian_version" ]; then echo "/usr/local/smr"; fi)
CFLAGS  = -Wall -O2 -I${SMR}/include
LDFLAGS = -L${SMR}/lib

#
# Our program files
#
PROG   = square
HDRS   =
OBJS   = square.o serverif.o
#LIBS   = -lm /usr/local/smr/lib/librhd.a -lrobot
LIBS   = $(shell if [ -f "/etc/redhat-release" ]; then echo "-lm /home/thieden/Desktop/autonomous-robot-systems/mobotware-3.1125/build/lib/librhd.a -lrobot"; elif [ -f "/etc/debian_version" ]; then echo "-lm /usr/local/smr/lib/librhd.a -lrobot"; fi)


all:	${PROG}

${PROG}: ${OBJS}
	${LD} -o ${@} ${LDFLAGS} ${OBJS} ${LIBS}

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
