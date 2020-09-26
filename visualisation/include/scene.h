#ifndef SCENE_H
#define SCENE_H

#include "sea_surface_actor.h"
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
   */ 
  Scene();

  /**
   * Override the default frame rate for animation.
   * @param time_step_size time step size in seconds
   */
  void set_timer_step_size(double time_step_size);

  /**
   * Add a sea surface actor to the scene. 
   */
  void add_actor(Sea_surface_actor* sea_surface_actor);

  /**
   * Add an ASV actor to the scene.
   */
  /*void add_actor(ASV_actor* asv_actor);*/

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
  double timer_step_size; // sec
  vtkSmartPointer<vtkAxesActor> axes_actor;
  vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget;
  Sea_surface_actor* sea_surface_actor;
  // Place holder for std::vector<asv_actor*> asv_actors;
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor;
}; //class Scene

} //namespace Visualisation
} //namespace asv_swarm

#endif // SCENE_H
