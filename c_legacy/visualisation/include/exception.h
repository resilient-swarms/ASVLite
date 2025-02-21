#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <string>

namespace asv_swarm
{
namespace Exception
{
class ValueError : public std::exception
{
public:
  ValueError(std::string message = "Incorrect value passed.") : exception() 
  {
    errorMessage = message;
  }

  virtual const char* what() const throw()
  {
    return errorMessage.c_str();
  }

private:
  std::string errorMessage;
}; //class ValueError

} // namespace Exception
} // namespace asv_swarm

#endif // EXCEPTION_H
