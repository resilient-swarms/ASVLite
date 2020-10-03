#ifndef ASV_ACTOR_H
#define ASV_ACTOR_H

extern "C" {
#include "asv.h"
}
#include "vtkCylinderSource.h"
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>

namespace asv_swarm
{
namespace Visualisation
{
/**
 * This class creates an actor for an ASV.
 */
class Asv_actor : 
  public vtkPolyDataAlgorithm
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
  void increment_time(){++timer_count;}

protected:
  /**
   * This method is called by vtk pipeline and it sets the z values for the
   * control points in the mesh representing sea surface for the current time 
   * step.
   */
  virtual int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector) override;

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
  vtkCylinderSource* cylinder {nullptr}; 
  vtkPolyDataMapper* cylinderMapper {nullptr};
  vtkActor* asv_actor {nullptr};
  
  struct Asv* asv;
}; // class Asv_actor

} // namespace Visualisation
} // namespace asv_swarm

#endif // ASV_ACTOR_H
