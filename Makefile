#use $(MAKE) to forward the make jobs
all : build doc
	cd build && $(MAKE)
	
doc : build
	cd build && $(MAKE) doc

build:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
	
clean :
	cd build && $(MAKE) clean
