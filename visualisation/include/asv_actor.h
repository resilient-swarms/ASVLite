#ifndef ASV_ACTOR_H
#define ASV_ACTOR_H

extern "C" {
#include "asv.h"
}
#include "vtkCamera.h"
#include <vtkActor.h>
#include <vtkNamedColors.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include "vtkCylinderSource.h"

namespace asv_swarm
{
namespace Visualisation
{
/**
 * This class creates an actor for an ASV.
 */
class Asv_actor : public vtkCommand
{
public:
  /**
   * Constructor.
   */
  Asv_actor(struct Asv* asv);

  /**
   * Set the step size for time increment.
   */
  void set_timer_step_size(double timer_step_size){
    this->timer_step_size = timer_step_size;
  }

  /**
   * Returns pointer to vtkActor object for asv.
   */
  vtkSmartPointer<vtkActor> get_actor(){return asv_actor;}

  /**
   * Increment time count.
   */
  void increment_time();

  /**
   * This method is called by vtk pipeline and it sets the 
   * position and attitude of the ASV for the current time step.
   */
  virtual void Execute(vtkObject* caller, unsigned long eventId,
                       void* vtkNotUsed(callData));

private:
  /** 
   * Method to set the compute the sea surface elevations at the grid points.
   * @param time in seconds from the start of simulation.
   */
  void set_asv_position_attitude(double time);

private:
  unsigned long timer_count;
  double timer_step_size; // sec
  double current_time; // sec
  // ASV is represented using a cylinder geometry.
  vtkSmartPointer<vtkCylinderSource> cylinder {nullptr}; 
  vtkSmartPointer<vtkPolyDataMapper> cylinderMapper {nullptr};
  vtkSmartPointer<vtkActor> asv_actor {nullptr};
  
  struct Asv* asv;
}; // class Asv_actor

} // namespace Visualisation
} // namespace asv_swarm

#endif // ASV_ACTOR_H
