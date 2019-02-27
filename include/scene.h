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
 * contains the VTK objects for rendering and animation.  
 */
class Scene : public vtkCommand
{
public:
  /**
   * Constructor. Also initialises the renderer, window and interactor.
   */ 
  Scene();

  /**
   * Set the sea condition and initialise the sea actor. The method also sets
   * some default values for field size, number of grid points on the field
   * surface, number of wave frequencies considered in the wave spectrum and the 
   * number of directions considered in the wave spectrum. The default values
   * can be found in the constructors Sea_surface_dynamics::Sea_surface_dynamics
   * and Wave_spectrum::Wave_spectrum.
   * @param wind_speed is the wind speed in m/s.
   * @param wind_fetch is the length of sea, in m, over which the wind blows.
   * @param wind_direction is the direction in which the wind blows measured in
   * radians with respect to North direction and direction of measurement such
   * that East is at PI/2 with North.
   */
  void initialise_sea_surface_actor(Quantity<Units::velocity> wind_speed,
                            Quantity<Units::length> wind_fetch,
                            Quantity<Units::plane_angle> wind_direction);

  /**
   * Returns pointer to actor sea.
   */
  Sea_surface_actor* get_sea_surface_actor(){return sea_surface_actor;}

  /**
   * Starts the animation. 
   */
  void start();

protected:
  /**
   * Method to call all actors and update their time by incrementing it by one
   * time step.
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
