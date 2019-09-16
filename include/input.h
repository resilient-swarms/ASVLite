#ifndef INPUT_H
#define INPUT_H

#include "asv.h"

/**
 * Function to read the input file and set the ASV's input values. 
 * @param file is the path to the input toml file. 
 * @param asv is the asv object for which the input values are to be set. 
 * @return 1 if the operation was successfull else return 0. 
 */
int set_input(char* file, struct Asv* asv);

#endif