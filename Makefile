.PHONY: test clean

build/protocol_test: components/alpha2/protocol.cpp components/alpha2/protocol.h tests/protocol_test.cpp
	mkdir -p build
	$(CXX) -std=c++11 -Wall -Wextra -Werror components/alpha2/protocol.cpp tests/protocol_test.cpp -o $@

test: build/protocol_test
	./build/protocol_test

clean:
	rm -rf build
