# Host-side tests of the firmware logic:   make -C firmware/test
CXX ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -O2 -include cstdint
SRC = ../pm3rrr_firmware/kinematics.cpp ../pm3rrr_firmware/protocol.cpp

test: test_host expected_ik.csv
	./test_host expected_ik.csv

expected_ik.csv: generate_expected.py
	python3 generate_expected.py > $@

test_host: test_host.cpp $(SRC) ../pm3rrr_firmware/*.h
	$(CXX) $(CXXFLAGS) -o $@ test_host.cpp $(SRC) -lm

clean:
	rm -f test_host expected_ik.csv

.PHONY: test clean
