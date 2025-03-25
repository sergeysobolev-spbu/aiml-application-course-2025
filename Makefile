# Определяем компилятор и флаги компиляции
CC = gcc
HEADERS = -I ./includes

CFLAGS = -Wall -Wextra -g $(HEADERS)
LDFLAGS = -lcunit -lm

# Определяем имена файлов
SRC = src/pathfinding.c 
APP_SRC = src/application.c
TESTS = tests/test_pathfinding.c
OBJ1 = $(SRC:.c=.o)
OBJ = $(patsubst src/%,build/%,$(OBJ1))
APP_OBJ1 = $(APP_SRC:.c=.o)
APP_OBJ = $(patsubst src/%,build/%,$(APP_OBJ1))
TEST_OBJ1 = $(TESTS:.c=.o)
TEST_OBJ = $(patsubst tests/%,build/%,$(TEST_OBJ1))
TEST_TARGET = build/pathfinding_test
APP_TARGET = build/pathfinding_app

# Правило для сборки исполняемого файла
all: $(APP_TARGET)

$(APP_TARGET): $(OBJ) $(APP_OBJ)
	$(CC) $(OBJ) $(APP_OBJ) -o $@ $(LDFLAGS)

$(TEST_TARGET): $(OBJ) $(TEST_OBJ)
	$(CC) $(OBJ) $(TEST_OBJ) -o $@ $(LDFLAGS)

# Правило для компиляции .c файлов в .o
%.o: %.c	
	$(CC) $(CFLAGS) -c $< -o build/$(notdir $@)

# Правило для запуска тестов
test: $(TEST_TARGET)
	./$(TEST_TARGET)

app: $(APP_TARGET)
	./$(APP_TARGET)

# Правило для очистки временных файлов
clean:
	rm -f $(OBJ) $(APP_OBJ) $(TEST_OBJ) $(APP_TARGET) $(TEST_TARGET)

.PHONY: all test clean
