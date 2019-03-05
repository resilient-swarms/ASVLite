#ifndef SCENE_H
#define SCENE_H

#include "sea_surface_actor.h"
#include <vtkCommand.h>

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
  unsigned int timer_step_size;
  Sea_surface_actor* sea_surface_actor;
  // Place holder for std::vector<asv_actor*> asv_actors;
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor;
}; //class Scene

} //namespace Visualisation
} //namespace asv_swarm

#endif // SCENE_H
