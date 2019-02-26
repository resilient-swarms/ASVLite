#include "visualization.h"

using namespace asv_swarm;

Visualization::Visualization(): vtkCommand{}
{
  timer_step_size = 10; //units in milliseconds
  /* Create the renderer, window and interactor */
  renderer = vtkSmartPointer<vtkRenderer>::New();
  window = vtkSmartPointer<vtkRenderWindow>::New();
  window->AddRenderer(renderer);
  interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(window);
}

void Visualization::set_sea_condition(Quantity<Units::velocity> wind_speed,
                                    Quantity<Units::length> wind_fetch,
                                    Quantity<Units::plane_angle> wind_direction)
{
  sea_surface_actor = new Sea_surface_actor(wind_speed, 
                                            wind_fetch, 
                                            wind_direction);
  /* set timer in sea surface actor*/
  sea_surface_actor->set_timer_step_size(timer_step_size);
}

void Visualization::start()
{
  /* Initialize must be called prior to creating timer events */
  interactor->Initialize();
  interactor->CreateRepeatingTimer(10); /* Repeating timer event at every 
                                           10 milliseconds */
  interactor->AddObserver(vtkCommand::TimerEvent, this);
  /* Render and interact */
  renderer->AddActor(sea_surface_actor->get_actor());
  window->SetSize(window->GetScreenSize());
  window->Render();
  interactor->Start();
}

void Visualization::increment_time()
{
  sea_surface_actor->increment_time();
}

void Visualization::Execute(vtkObject *caller, 
                            unsigned long vtkNotUsed(eventId),
                            void *vtkNotUsed(callData))
{
  increment_time();
  sea_surface_actor->Modified();
  
  vtkRenderWindowInteractor *interactor =
    static_cast<vtkRenderWindowInteractor*>(caller);
  interactor->Render();
}