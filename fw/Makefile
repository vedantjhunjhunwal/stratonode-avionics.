CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion -Iinclude
SRC = src/drivers.c src/node.c src/app.c

all: test sim m4

build:
	mkdir -p build

test: build                      ## 136 unit checks, AddressSanitizer + UBSan
	$(CC) $(CFLAGS) -fsanitize=address,undefined $(SRC) tests/test_main.c -lm -o build/test_fw
	./build/test_fw

sim: build                       ## closed-loop 10.5 h flight, 5 seeds, with and without load shedding
	$(CC) -std=gnu11 -O2 -Wall -Wextra -Iinclude $(SRC) sim/flight_sim.c -lm -o build/flight_sim
	mkdir -p ../results/sim
	for s in 1 2 3 4 5; do \
	  ./build/flight_sim --seed $$s --csv ../results/sim/seed$$s.csv --json ../results/sim/seed$$s.json > /dev/null & \
	  ./build/flight_sim --seed $$s --no-shed --json ../results/sim/seed$${s}_noshed.json > /dev/null & \
	done; wait

m4:                              ## STM32G474 (Cortex-M4F) cross-compile, size report, heap check
	./target/build.sh

.PHONY: all test sim m4
