CFLAGS = -g
LIBS = -lm -g -lcunit
APP = demo
TEST_APP = demo_test
CC = gcc -c
LD = gcc 

%.o: %.c
	$(CC) $< $(CFLAGS)


all: main.o ant_colony.o
	$(LD) -o $(APP) $^ $(LIBS) $(CFLAGS)

test: test_ant_colony.o ant_colony.o
	$(LD) -o $(TEST_APP) $^ $(LIBS) $(CFLAGS)

clean: 
	rm -rf *.o $(APP)