#include "scene.h"

using namespace asv_swarm;
using namespace asv_swarm::Visualisation;

Scene::Scene(): vtkCommand{}
{
  unsigned int frame_rate = 25; /* Required fps for animation */
  timer_step_size = static_cast<int>(1000/frame_rate); // units in milliseconds. 

  /* Actors initialised to nullptr. Actors must be initialised by calling the
   * corresponding initialise_actor method. 
   */
  sea_surface_actor = nullptr;

  /* Create the renderer, window and interactor */
  renderer = vtkSmartPointer<vtkRenderer>::New();
  window = vtkSmartPointer<vtkRenderWindow>::New();
  window->AddRenderer(renderer);
  interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(window);
}

void Scene::initialise_sea_surface_actor(
    Quantity<Units::velocity> wind_speed,
    Quantity<Units::length> wind_fetch,
    Quantity<Units::plane_angle> wind_direction)
{
  sea_surface_actor = new Sea_surface_actor(wind_speed, 
                                            wind_fetch, 
                                            wind_direction);
  /* set timer in sea surface actor*/
  sea_surface_actor->set_timer_step_size(timer_step_size);
}

void Scene::start()
{
  /* Initialize must be called prior to creating timer events */
  interactor->Initialize();
  interactor->CreateRepeatingTimer(timer_step_size);/* Repeating timer events */
  interactor->AddObserver(vtkCommand::TimerEvent, this);
  
  /* Add actors */
  renderer->AddActor(sea_surface_actor->get_vtk_actor());
  
  /* Render and interact */
  window->SetSize(window->GetScreenSize());
  window->Render();
  interactor->Start();
}

void Scene::increment_time()
{
  sea_surface_actor->increment_time();
}

void Scene::Execute(vtkObject *caller, 
                            unsigned long vtkNotUsed(eventId),
                            void *vtkNotUsed(callData))
{
  increment_time();
  sea_surface_actor->Modified();
  
  vtkRenderWindowInteractor *interactor =
    static_cast<vtkRenderWindowInteractor*>(caller);
  interactor->Render();
}
