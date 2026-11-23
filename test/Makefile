CXX ?= g++
LOGS ?= ../../stream/frames.log ../../laptop/boiler-capture/frames.log ../../laptop/boiler-capture/frames-1119.log
test_core: test_core.cpp ../components/boiler_bus/boiler_core.cpp ../components/boiler_bus/boiler_core.h
	$(CXX) -std=c++17 -O2 -Wall -Wextra -o $@ test_core.cpp ../components/boiler_bus/boiler_core.cpp
test: test_core
	./test_core $(LOGS)
clean:
	rm -f test_core
