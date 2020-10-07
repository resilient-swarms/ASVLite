#ifndef SCENE_H
#define SCENE_H

extern "C" {
#include "simulation.h"
}
#include "sea_surface_actor.h"
#include "asv_actor.h"
#include <vtkCommand.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

namespace asv_swarm
{
namespace Visualisation
{
/**
 * Class to coordinate visualisation. This class contains all actors. It also 
 * contains the vtk objects for rendering and animation.  
 */
class Scene : public vtkCommand
{
public:
  /**
   * Constructor. Also initialises the renderer, window and interactor.
   * @param node is the first node in the linked list Simulatation_data.
   */ 
  Scene(struct Simulation* node);

  /**
   * Clean the heap.
   */
  ~Scene();

  /**
   * Override the default frame rate for animation.
   * @param time_step_size time step size in seconds
   */
  void set_timer_step_size(double time_step_size);

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
  void set_sea_surface_grid_size(unsigned int grid_size);

  /**
   * Starts the animation. 
   */
  void start();

protected:
  /**
   * Synchronise time update for all actors. Method calls all actors and update 
   * their time by incrementing it by one time step.
   */
  void increment_time();

  /**
   * Call back function for timer. Overrides virtual method vtkCommand::Execute.
   */
  void Execute(vtkObject *caller, 
               unsigned long vtkNotUsed(eventId),
               void *vtkNotUsed(callData)) override;

private:
  Simulation* first_node;
  long timer_count; 
  double timer_step_size; // sec
  vtkSmartPointer<vtkAxesActor> axes_actor;
  vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget;
  Sea_surface_actor* sea_surface_actor;
  std::vector<Asv_actor*> asv_actors;
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor;
  // Record simulation time.
  struct timespec start_time;
  struct timespec finish_time;
}; //class Scene

} //namespace Visualisation
} //namespace asv_swarm

#endif // SCENE_H
