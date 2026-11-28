CXX ?= g++
LOGS ?= ../../stream/frames.log ../../laptop/boiler-capture/frames.log ../../laptop/boiler-capture/frames-1119.log
test_core: test_core.cpp ../components/boiler_bus/boiler_core.cpp ../components/boiler_bus/boiler_poller.cpp ../components/boiler_bus/boiler_errors.cpp ../components/boiler_bus/boiler_core.h ../components/boiler_bus/boiler_poller.h ../components/boiler_bus/boiler_probe.h
	$(CXX) -std=c++17 -O2 -Wall -Wextra -o $@ test_core.cpp ../components/boiler_bus/boiler_core.cpp ../components/boiler_bus/boiler_poller.cpp ../components/boiler_bus/boiler_errors.cpp
test: test_core
	./test_core $(LOGS)
clean:
	rm -f test_core
