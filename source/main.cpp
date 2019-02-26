#include "sea_surface_actor.h"
#include <vtkCommand.h>

using namespace asv_swarm;

unsigned int timer_count{0u};
Sea_surface_actor* p_sea_surface_actor{nullptr};

class CommandSubclass : public vtkCommand
{
  public:
  vtkTypeMacro(CommandSubclass, vtkCommand);
  static CommandSubclass *New()
  {
    return new CommandSubclass;
  }

  void Execute(vtkObject *caller, 
               unsigned long vtkNotUsed(eventId),
               void *vtkNotUsed(callData)) override
  {
    ++timer_count;
    p_sea_surface_actor->Modified();
    
    vtkRenderWindowInteractor *interactor =
      static_cast<vtkRenderWindowInteractor*>(caller);
    interactor->Render();
  }
};

int main()
{
  /* Define sea condition */
  Quantity<Units::length> fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {15*Units::meter_per_second};
  /* wind direction is 30deg east of north*/
  Quantity<Units::plane_angle> wind_direction {Const::PI/6 * Units::radian};

  /* Initialize visualization for sea surface */
  Sea_surface_actor sea_surface_actor {
    fetch, wind_speed, wind_direction};
  p_sea_surface_actor = &sea_surface_actor;

  /* Create the renderer, window and interactor */
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> window = 
    vtkSmartPointer<vtkRenderWindow>::New();
  window->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(window);

  /* Initialize must be called prior to creating timer events */
  interactor->Initialize();
  interactor->CreateRepeatingTimer(10); /* Repeating timer event at every 
                                           10 milliseconds */

  /* Add observer to timer event */
  vtkSmartPointer<CommandSubclass> timerCallback =
  vtkSmartPointer<CommandSubclass>::New();
  interactor->AddObserver(vtkCommand::TimerEvent, timerCallback);

  /* Render and interact */
  renderer->AddActor(sea_surface_actor.get_actor());
  window->SetSize(window->GetScreenSize());
  window->Render();
  interactor->Start();

  return EXIT_SUCCESS;
}
