.PHONY: track clean-track

track: track.cpp
	g++ -std=c++17 -O2 -Wall -Wextra track.cpp -o track $$(pkg-config --cflags --libs opencv4)

clean-track:
	rm -f track
