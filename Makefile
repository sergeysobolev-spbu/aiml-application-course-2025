CFLAGS = -g
LIBS = -lm -g
APP = demo
CC = gcc -c
LD = gcc 

%.o: %.c
	$(CC) $< $(CFLAGS)


all: main.o
	$(LD) -o $(APP) $< $(LIBS) $(CFLAGS)

clean: 
	rm -rf *.o $(APP)