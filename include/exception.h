#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <string>

class ValueError : public std::exception
{
  std::string errorMessage;
 public:
  virtual const char* what() const throw()
  {
    return errorMessage.c_str();
  }
 ValueError(std::string message = "Incorrect value passed.") : exception() 
 {errorMessage = message;}
};

#endif // EXCEPTION_H
