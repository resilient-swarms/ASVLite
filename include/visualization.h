#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "sea_surface_actor.h"
#include <vtkCommand.h>

namespace asv_swarm
{
/**
 * Class to coordinate visualization. 
 */
class Visualization : public vtkCommand
{
public:
  Visualization();

  void set_sea_condition(Quantity<Units::velocity> wind_speed,
                         Quantity<Units::length> wind_fetch,
                         Quantity<Units::plane_angle> wind_direction);

  void start();

  void increment_time();

  void Execute(vtkObject *caller, 
               unsigned long vtkNotUsed(eventId),
               void *vtkNotUsed(callData)) override;

private:
  unsigned int timer_step_size;
  Sea_surface_actor* sea_surface_actor{nullptr};
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor;
}; //class Visualization

} //namespace asv_swarm

#endif // VISUALIZATION_H