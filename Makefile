#use $(MAKE) to forward the make jobs
all : build
	cd build && $(MAKE)

build:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
	
clean :
	cd build && $(MAKE) clean
	rm -rf build
