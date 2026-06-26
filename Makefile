.PHONY: track clean-track

track: tools/track.cpp
	g++ -std=c++17 -O2 -Wall -Wextra tools/track.cpp -o track $$(pkg-config --cflags --libs opencv4)

clean-track:
	rm -f track
