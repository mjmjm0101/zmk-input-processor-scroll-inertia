CC      ?= cc
CFLAGS  ?= -Wall -Wextra -Wpedantic -std=c11 -O2

.PHONY: test clean

test: test_math
	./test_math

test_math: test_math.c ../src/scroll_inertia_math.h
	$(CC) $(CFLAGS) -o $@ test_math.c

clean:
	rm -f test_math
