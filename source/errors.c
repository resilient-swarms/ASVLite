#include <malloc.h>
#include <stddef.h>
#include <string.h>
#include"errors.h"

const char* error_null_pointer  = "Argument cannot be a null pointer.";
const char* error_negative_time = "Time cannot be negative.";
const char* error_time_not_incremented = "Time not incremented.";
const char* error_invalid_index = "Invalid index.";
const char* error_malloc_failed = "Memory allocation failed.";
const char* error_incorrect_rudder_angle = "Incorrect rudder angle.";

void set_error_msg(char* p_error_msg, const char* msg)
{
  if(p_error_msg)
  {
    clear_error_msg(p_error_msg);
  }
  p_error_msg = (char*)malloc(sizeof(char) * strlen(msg));
  strcpy(p_error_msg, msg);
}

void clear_error_msg(char* p_error_msg)
{
  if(p_error_msg)
  {
    free(p_error_msg);
    p_error_msg = NULL;
  }
}