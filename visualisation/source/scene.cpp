#include "exception.h"
#include "scene.h"
#include <vtkNamedColors.h>
#include <vtkCamera.h>

using namespace asv_swarm;
using namespace asv_swarm::Visualisation;

Scene::Scene(): vtkCommand{}
{
  timer_step_size = 0.04; // seconds. Default timer step size corresponding to frame rate of 25fps. 

  // Actors initialised to nullptr. Actors must be initialised by calling the
  // corresponding initialise_actor method. 
  sea_surface_actor = nullptr;

  // Create the renderer, window and interactor 
  renderer = vtkSmartPointer<vtkRenderer>::New();
  window = vtkSmartPointer<vtkRenderWindow>::New();
  window->AddRenderer(renderer);
  interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(window);

  // Create an actor for displaying the coordinate axes 
  axes_actor = vtkSmartPointer<vtkAxesActor>::New();
  axes_widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  double rgba[4]{0.0, 0.0, 0.0, 0.0};
  vtkSmartPointer<vtkNamedColors> colors = 
    vtkSmartPointer<vtkNamedColors>::New();
  colors->GetColor("Carrot",rgba);
  axes_widget->SetOutlineColor(rgba[0], rgba[1], rgba[2]);
  axes_widget->SetOrientationMarker( axes_actor );
  axes_widget->SetInteractor( interactor );
  axes_widget->SetViewport( 0.0, 0.0, 0.3, 0.3 );
  axes_widget->SetEnabled( 1 );
  axes_widget->InteractiveOff();
}

void Scene::set_timer_step_size(double time_step_size)
{
  timer_step_size = time_step_size; 
  // set timer step size in all actors
  sea_surface_actor->set_timer_step_size(timer_step_size);
  // TODO: set time for asv actor
}


void Scene::add_actor(Sea_surface_actor* sea_surface_actor)
{
  if( !sea_surface_actor )
  {
    throw asv_swarm::Exception::ValueError("Scene::add_actor. Parameter sea_surface_actor" 
                                           "should not be nullptr.");
  }
  //TODO: throw exception for null ASV actors.
  
  this->sea_surface_actor = sea_surface_actor;
}

void Scene::start()
{
  // Initialize must be called prior to creating timer events 
  interactor->Initialize();
  interactor->CreateRepeatingTimer(timer_step_size);// Repeating timer events 
  interactor->AddObserver(vtkCommand::TimerEvent, this);
  
  // Add actors 
  renderer->AddActor(sea_surface_actor->get_actor());

  // Inform all actors of the timer step size 
  sea_surface_actor->set_timer_step_size(timer_step_size);
  //TODO: set_timer_step_size for asv actor.

  // Render and interact 
  renderer->ResetCamera();
  window->SetSize(window->GetScreenSize());
  window->Render();
  interactor->Start();
}

void Scene::increment_time()
{
  sea_surface_actor->increment_time();
  //TODO: increment time for asv actor.
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
