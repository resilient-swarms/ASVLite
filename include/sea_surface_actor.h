#ifndef SEA_SURFACE_ACTOR_H
#define SEA_SURFACE_ACTOR_H

#include "sea_surface_dynamics.h"
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
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

/**
 * This class provides visualization for the data in the class
 * Sea_surface_dynamics.
 */
class Sea_surface_actor : 
  public Sea_surface_dynamics,
  public vtkPolyDataAlgorithm
{
public:
  /**
   * Constructor.
   */
  Sea_surface_actor(Quantity<Units::velocity> wind_speed,
                    Quantity<Units::length> fetch,
                    Quantity<Units::plane_angle> wind_direction);

  /**
   * Increment time.
   */
  void increment_time(){++timer_count;}

  /**
   * Set the step size for time increment.
   */
  void set_timer_step_size(unsigned int timer_step_size){
    this->timer_step_size = timer_step_size;
  }

  /**
   * Returns pointer to sea surface actor.
   */
  vtkSmartPointer<vtkActor> get_actor(){return sea_surface_actor;}

protected:
  /**
   * Method to set the z values of the control points for the time passed as
   * argument.
   */
  virtual int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector) override;
  
private:
  unsigned long timer_count;
  unsigned int timer_step_size;
  vtkSmartPointer<vtkPoints> sea_surface_mesh_points {nullptr};
  vtkSmartPointer<vtkCellArray> sea_surface_mesh_cells {nullptr}; 
  vtkSmartPointer<vtkPolyDataMapper> sea_surface_mapper {nullptr};
  vtkSmartPointer<vtkActor> sea_surface_actor {nullptr};
}; // class Sea_surface_actor
} // namespace asv_swarm

#endif // SEA_SURFACE_ACTOR_H
