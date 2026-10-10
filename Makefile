.PHONY: all configure test clean

all: configure
	cmake --build build

configure:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

test: configure
	cmake --build build
	ctest --test-dir build --output-on-failure

clean:
	rm -rf build
