build:
	mkdir build

build/%.o : src/%.cpp build
	gcc -Wall -Iinclude -c $< 

% : build/%.o build/name_of.o
	@rm -f $@
	gcc -Wall -Iinclude -o $@ $< -lstdc++
	@echo ------------------------------------------------
	./$@
