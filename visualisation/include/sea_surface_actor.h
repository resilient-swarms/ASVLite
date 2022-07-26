#ifndef SEA_SURFACE_ACTOR_H
#define SEA_SURFACE_ACTOR_H

extern "C" {
#include "wave.h"
#include "geometry.h"
}
#include <vector>
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
namespace Visualisation
{
/**
 * This class creates an actor for sea surface.
 */
class Sea_surface_actor : 
  public vtkPolyDataAlgorithm
{
public:
  /**
   * Constructor.
   */
  Sea_surface_actor(struct Wave* wave);

  /**
   * Set the step size for time increment.
   */
  void set_timer_step_size(double timer_step_size){
    this->timer_step_size = timer_step_size;
  }

  /**
   * Override the default edge length of the square sea surface. Also resets the 
   * control points on the surface.
   * @param field_length is the edge length in meter. Value of length should be 
   * a non-zero positive value.
   */
  void set_field_length(double field_length);

  /**
   * Method to set he number of points along both x and y directions
   * of the square field. The default value for the number of points is
   * provided by the constructor asv_swarm::Visualisation::Sea_surface_actor. 
   * A higher number for the count will result in a more dense mesh representing the sea 
   * surface. After updating the count the method resets all the points 
   * as per the new count value.
   * @param grid_size the number of points along one edge of the sea
   * surface. The value should be greater than 1.
   */
  void set_sea_surface_grid_count(unsigned int grid_size);

  /**
   * Overide the default position of the simulated sea surface.
   */
  void set_sea_surface_position(union Coordinates_3D sea_surface_position);

  /**
   * Increment time count.
   */
  void increment_time();

  /**
   * Returns pointer to vtkActor object for sea surface.
   */
  vtkSmartPointer<vtkActor> get_actor(){return sea_surface_actor;}

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
   */
  void set_sea_surface_elevations();

  /**
   * Method to set uniformly spaced points for the sea surface.
   */
  void set_sea_surface_points();
  
private:
  unsigned long timer_count;
  double timer_step_size; // sec
  double current_time; // sec
  vtkSmartPointer<vtkPoints> sea_surface_mesh_points {nullptr};
  vtkSmartPointer<vtkCellArray> sea_surface_mesh_cells {nullptr}; 
  vtkSmartPointer<vtkPolyDataMapper> sea_surface_mapper {nullptr};
  vtkSmartPointer<vtkActor> sea_surface_actor {nullptr};
  
  struct Wave* wave;
  std::vector<std::vector<union Coordinates_3D>> sea_surface_points; // A grid of NxN points to represent the square sea surface. 
  unsigned int sea_surface_grid_size; // sea_surface_grid_size = N. Value must be greater than 1.
  double field_length; // Length in meter of one edge of the square sea surface.
  union Coordinates_3D sea_surface_position; // Position of the bottom left corner of the simulated sea surface.
}; // class Sea_surface_actor

} // namespace Visualisation
} // namespace asv_swarm

#endif // SEA_SURFACE_ACTOR_H
