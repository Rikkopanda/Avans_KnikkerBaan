CXX ?= g++
PKG_CONFIG ?= pkg-config

QT_PACKAGE := $(firstword $(foreach package,Qt6Widgets Qt5Widgets QtGui,$(if $(shell $(PKG_CONFIG) --exists $(package) && echo yes),$(package))))
MOC ?= $(shell command -v moc-qt6 2>/dev/null || command -v moc-qt5 2>/dev/null || command -v moc 2>/dev/null || command -v moc-qt4 2>/dev/null || printf '%s' "$(firstword $(wildcard /usr/lib/qt6/libexec/moc /usr/lib/qt6/bin/moc /usr/lib/qt5/bin/moc /usr/lib/qt4/bin/moc))")
QT_CFLAGS := $(shell test -n "$(QT_PACKAGE)" && $(PKG_CONFIG) --cflags $(QT_PACKAGE))
QT_LIBS := $(shell test -n "$(QT_PACKAGE)" && $(PKG_CONFIG) --libs $(QT_PACKAGE))

CXXFLAGS += -Wall -std=c++11 -fPIC $(QT_CFLAGS)
MOCFLAGS += $(QT_CFLAGS)
LDLIBS += $(QT_LIBS)

SOURCES = mainWindow.cpp main.cpp
MOC_HEADERS = mainWindow.h
EXECUTABLE = serial_plot_gui
MOC_SOURCES = $(MOC_HEADERS:.h=.moc.cc)
OBJECTS = $(SOURCES:.cpp=.o) $(MOC_SOURCES:.cc=.o)

all: $(EXECUTABLE)
	@echo Done!

$(EXECUTABLE): $(OBJECTS) | check-qt
	$(CXX) $^ $(LDLIBS) -o $@

# Generate object files, rule to change postfix
%.o: %.cpp | check-qt
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.moc.cc | check-qt
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Generate cc from h via Qt's Meta Object Compiler, rule to change postfix
%.moc.cc: %.h | check-qt
	$(MOC) $(MOCFLAGS) $< -o $@

check-qt:
	@test -n "$(QT_PACKAGE)" || (echo "Qt development package not found by pkg-config. Install qtbase5-dev/qtbase6-dev or set CXXFLAGS and LDLIBS manually." >&2; exit 1)
	@test -n "$(MOC)" || (echo "Qt moc not found. Install Qt development tools or run: make MOC=/path/to/moc" >&2; exit 1)

.PHONY: tags clean run
.PHONY: check-qt

run: $(EXECUTABLE)
	./run_serial_plot_gui.sh $(ARGS)

clean:
	rm -f *.o *.moc.cc ui_mainwindow.h hello $(EXECUTABLE)

# Generate ctags file for all included files (autocomplete and jump to source)
tags:
	$(CXX) -M $(CXXFLAGS) $(SOURCES) | \
	sed -e 's/[\\ ]/\n/g' | \
	sed -e '/^$$/d' -e '/\.o:[ \t]*$$/d' | \
	ctags -L - --c++-kinds=+p --fields=+iaS --extra=+q
