all: main

main: main.o
	g++ main.o -o quadcopter-control -I ./

main.o: main.cpp
	g++ -c -std=c++11 -Wall main.cpp -I ./

clean:
	rm -rf *o quadcopter-control
