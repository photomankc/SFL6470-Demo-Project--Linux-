# external dependencies
# These can be set by specifying them on the command line
# i.e. make 
OBJDIR=build

# A list of paths to search for header files in
INCLUDES   = ./include ./motors ./bus_protocol     

# The output folder for intermediate build files (object files)
OBJDIR     = build

# Compiler directives and flags
# Ensure our include paths are all prefixed with -I
CC_INCLUDES= $(addprefix -I,$(INCLUDES))

# specify our desired compiler
CC         = g++

# specify our compiler flags, including all of the include folders
CFLAGS     = -Wall -std=c++0x -c $(CC_INCLUDES)

# specify our linker
LD         = g++

# specify our linker flags
LDFLAGS    = 

# recursively find all C++ files from the Makefile directory
SOURCES    = $(wildcard *.cpp) $(wildcard */*.cpp)

# for each C++ file found change cpp to o and prefix the OBJDIR
OBJECTS   := $(addprefix $(OBJDIR)/,$(patsubst %.cpp,%.o,$(SOURCES)))

# the final binary output. can be overriden at the command line
BIN        = ./bin/dspin-test



all: $(OBJECTS)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $(BIN)

$(OBJDIR)/%.o : %.cpp | $(OBJDIR)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR):        
	@mkdir -p $(OBJDIR)


clean:        
	@rm -rf build
