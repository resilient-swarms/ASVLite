# A simple makefile to compile the tex files to a pdf document.
# Build targets provided:
# -----------------------
# make        : default compile option is make quiet.
# make quiet  : compile the tex files to pdf with minimal output message.
# make verbose: compile with detailed output messages.
# make view   : compile, if required, and view pdf file. 
# make clean  : clean all build files.

# Name of output pdf file
TARGET := documentation

# Name of main tex file which includes all the other tex files.
MAIN := main

# Set the source and build directory
SRC_DIRS := ./
BUILD_DIR := ./build

# Collect all tex files and bib files.
SRCS := $(wildcard *.tex) \
        $(wildcard *.bib)

# Get versions of pdflatex and bibtex. This will be used to check if these applications exist.
PDFLATEX_VERSION := $(shell pdflatex --version 2>/dev/null)
BIBTEX_VERSION := $(shell bibtex --version 2>/dev/null)
# Check if all dependencies installed
ifdef PDFLATEX_VERSION
	ifdef BIBTEX_VERSION
		DEPENDENCIES_INSTALLED := 'true'
	endif
endif

# Compile the tex files
# ---------------------
# Each line of the build script has the following structure:
# -@command > /dev/null 2>&1 || true
# -           : to ignore the exit status of the command that is executed
#               and continue even if error encountered.
# @           : to suppresses the 'echo' of the command that is executed.
# > /dev/null : to redirect output (1) to /dev/null
# 2>&1        : to redirect error (2) to /dev/null 
# || true     : to suppress the "make: error ....(ignored)" output message.
#
# The last part '> /dev/null 2>&1 || true' is to make the output quiet

# Quiet compile
quiet: QUIET := > /dev/null 2>&1 || true
quiet: $(BUILD_DIR)/$(TARGET).pdf

# Verbous compile
verbose: QUIET := 
verbose: $(BUILD_DIR)/$(TARGET).pdf

# View the pdf 
# quiet is the build option for view
view: QUIET := > /dev/null 2>&1 || true
view: $(BUILD_DIR)/$(TARGET).pdf
	-@xdg-open $(BUILD_DIR)/$(TARGET).pdf > /dev/null 2>&1 || true &

# Compile pdf 
$(BUILD_DIR)/$(TARGET).pdf: $(SRCS)
	-@echo "--> Checking if pdflatex and bibtex are installed"
ifdef PDFLATEX_VERSION 
	-@echo "    --> pdflatex found."
else
	-@echo "    --> pdflatex not found."
endif
ifdef BIBTEX_VERSION 
	-@echo "    --> bibtex found."
else
	-@echo "    --> bibtex not found."
endif
# Proceed with compiling only if pdflatex and bibtex are found
ifdef DEPENDENCIES_INSTALLED
	-@echo "--> Making build directory if it does not exist"
	-@mkdir $(BUILD_DIR) $(QUIET)

	-@echo "--> pdflatex first run"
	-@pdflatex -interaction=nonstopmode -output-directory=$(BUILD_DIR) $(MAIN).tex $(QUIET)
	
	-@echo "--> run bibtex"
	-@bibtex $(BUILD_DIR)/$(MAIN) $(QUIET)
	
	-@echo "--> pdflatex second run"
	-@pdflatex -interaction=nonstopmode -output-directory=$(BUILD_DIR) $(MAIN).tex $(QUIET)
	
	-@echo "--> pdflatex final run"
	-@pdflatex -interaction=nonstopmode -output-directory=$(BUILD_DIR) $(MAIN).tex $(QUIET)
# Rename MAIN.pdf to TARGET.pdf if MAIN not equal to TARGET
ifneq ($(MAIN), $(TARGET)) 
	-@mv $(BUILD_DIR)/$(MAIN).pdf $(BUILD_DIR)/$(TARGET).pdf $(QUIET)
endif
# Check if the pdf file created. 
ifneq ("$(BUILD_DIR)/$(wildcard *.pdf)","")
	-@echo "--> Created $(BUILD_DIR)/$(TARGET).pdf"
else
	-@echo "--> Build failed possibly due to missing latex packages."
endif
# Else statement for: ifdef DEPENDENCIES_INSTALLED
else
	-@echo "--> Missing dependencies. Cannot compile."
endif

# Clean the build directory
.PHONY: clean
clean:
	-@$(RM) -r $(BUILD_DIR)
	-@echo "--> Build directory removed"