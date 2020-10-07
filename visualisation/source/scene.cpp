#include "exception.h"
#include "scene.h"
#include <vtkNamedColors.h>
#include <vtkCamera.h>

using namespace asv_swarm;
using namespace asv_swarm::Visualisation;

Scene::Scene(struct Simulation* first_node): vtkCommand{}
{
  timer_count = 0;
  timer_step_size = first_node->asv->dynamics.time_step_size;
  this->first_node = first_node;

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

  // Create actor for sea surface
  this->sea_surface_actor = new Sea_surface_actor(first_node->wave);

  // Create actor for ASV 
  for(struct Simulation* node = first_node; node != NULL; node = node->next)
  {
    auto asv_actor = new Asv_actor(node->asv);
    this->asv_actors.push_back(asv_actor);
  }

  // Add actors and set the time step size
  renderer->AddActor(sea_surface_actor->get_actor());
  sea_surface_actor->set_timer_step_size(timer_step_size);
  for(auto asv_actor : asv_actors)
  {
    renderer->AddActor(asv_actor->get_actor());
    asv_actor->set_timer_step_size(timer_step_size);
  }
}

Scene::~Scene()
{
  delete this->sea_surface_actor;
  for(auto asv_actor : asv_actors)
  {
    delete asv_actor;
  }
}

void Scene::set_field_length(double field_length)
{
  sea_surface_actor->set_field_length(field_length);
}

void Scene::set_sea_surface_grid_size(unsigned int grid_size)
{
  sea_surface_actor->set_sea_surface_grid_size(grid_size);
}


void Scene::start()
{
  // Initialize must be called prior to creating timer events 
  interactor->Initialize();
  interactor->CreateRepeatingTimer(timer_step_size * 1000.0);// Repeating timer events. Arguement in sec converted to milli-sec.
  
  // Add observers.
  // When events are invoked, the observers are called in the order they were added.
  // Scene should be invoked first as it increments time and updates simulation data.
  // Call all actors after excuting scene.
  // Add scene as an observer.
  interactor->AddObserver(vtkCommand::TimerEvent, this);
  // Add asv actors as observer.
  for(auto asv_actor : asv_actors)
  {
    interactor->AddObserver(vtkCommand::TimerEvent, asv_actor);
  }
  
  // Render and interact 
  renderer->ResetCamera();
  window->SetSize(window->GetScreenSize());
  window->Render();
  interactor->Start();
}

void Scene::increment_time()
{
  ++timer_count;
  sea_surface_actor->increment_time();
  for(auto asv_actor : asv_actors)
  {
    asv_actor->increment_time();
  }
}

void Scene::Execute(vtkObject *caller, 
                    unsigned long vtkNotUsed(eventId),
                    void *vtkNotUsed(callData))
{
  increment_time();

  // Compute for current time step.
  bool buffer_exceeded = false;
  bool has_all_reached_final_waypoint = true;
  simulation_with_time_sync_for_time_step(first_node, timer_count, &buffer_exceeded, &has_all_reached_final_waypoint);
  // stop if all reached the destination or if buffer exceeded.
  if(has_all_reached_final_waypoint || buffer_exceeded)
  {
    // stop simulation
    exit(0);
  }

  // Update sea surface visualisation
  sea_surface_actor->Modified();
  
  vtkRenderWindowInteractor *interactor =
    static_cast<vtkRenderWindowInteractor*>(caller);
  interactor->Render();
}
