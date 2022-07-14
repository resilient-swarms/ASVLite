#ifndef ERRORS_H
#define ERRORS_H

extern const char* error_null_pointer;
extern const char* error_negative_time;
extern const char* error_time_not_incremented;
extern const char* error_invalid_index;
extern const char* error_malloc_failed;
extern const char* error_incorrect_rudder_angle;

void set_error_msg(char* p_error_msg, const char* msg);
void clear_error_msg(char* p_error_msg);

#endif // ERRORS_H