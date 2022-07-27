#include "exception.h"
#include "scene.h"
#include <vtkNamedColors.h>
#include <vtkCamera.h>
#include <math.h>

using namespace asv_swarm;
using namespace asv_swarm::Visualisation;

Scene::Scene(struct Simulation* first_node): vtkCommand{}
{
  int count_asvs = simulation_get_count_asvs(first_node);
  struct Asv** asvs = new struct Asv*[count_asvs];
  simulation_get_asvs(first_node, asvs);
  timer_count = 0;
  timer_step_size = 40.0/1000.0; // sec
  this->first_node = first_node;

  // Create the renderer, window and interactor 
  renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(255, 255, 255);
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
  struct Wave* wave = asv_get_wave(asvs[0]);
  this->sea_surface_actor = new Sea_surface_actor(wave);

  // Create actor for ASV 
  for(int i = 0; i < count_asvs; ++i)
  {
    auto asv_actor = new Asv_actor(asvs[i]);
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

  // Set the sea surface mesh 
  double field_length = get_sea_surface_edge_length();
  int grid_count = get_count_mesh_cells_along_edge();
  union Coordinates_3D sea_surface_position = get_sea_surface_position();
  sea_surface_actor->set_field_length(field_length);
  sea_surface_actor->set_sea_surface_grid_count(grid_count);
  sea_surface_actor->set_sea_surface_position(sea_surface_position);

  delete[] asvs;
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
  simulation_run_a_timestep(first_node);
  bool has_any_reached_final_waypoint = false;
  bool buffer_exceeded = false;
  int count_asvs = simulation_get_count_asvs(first_node);
  struct Asv** asvs = new struct Asv*[count_asvs];
  simulation_get_asvs(first_node, asvs);
  for(int i = 0; i < count_asvs; ++i)
  {
    union Coordinates_3D p1 = asv_get_position_cog(asvs[i]);
    union Coordinates_3D p2 = simulation_get_waypoint(first_node, asvs[i]);
    double distance = sqrt((p1.keys.x-p2.keys.x)*(p1.keys.x-p2.keys.x) + (p1.keys.y-p2.keys.y)*(p1.keys.y-p2.keys.y));
    if(distance < 5.0)
    {
      has_any_reached_final_waypoint = true;
    }
    long buffer_size = simulation_get_buffer_size();
    long buffer_length = simulation_get_buffer_length(first_node, asvs[i]);
    if(buffer_length >= buffer_size)
    {
      buffer_exceeded = true;
    }
  }  

  // stop if all reached the destination or if buffer exceeded.
  if(has_any_reached_final_waypoint || buffer_exceeded)
  {
    // stop simulation
    interactor->ExitCallback();
  }

  // Update sea surface visualisation
  sea_surface_actor->Modified();
  
  //vtkRenderWindowInteractor *interactor =
  //  static_cast<vtkRenderWindowInteractor*>(caller);
  interactor->Render();
}
