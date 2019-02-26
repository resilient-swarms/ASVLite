#include "sea_surface_actor.h"
#include <vtkCommand.h>

using namespace asv_swarm;

class Visualization : public vtkCommand
{
public:
  Visualization(): vtkCommand{}
  {
    timer_step_size = 10; //units in milliseconds
    /* Create the renderer, window and interactor */
    renderer = vtkSmartPointer<vtkRenderer>::New();
    window = vtkSmartPointer<vtkRenderWindow>::New();
    window->AddRenderer(renderer);
    interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(window);

  }

  void set_sea_condition(Quantity<Units::velocity> wind_speed,
                         Quantity<Units::length> wind_fetch,
                         Quantity<Units::plane_angle> wind_direction)
  {
    sea_surface_actor = new Sea_surface_actor(wind_speed, 
                                              wind_fetch, 
                                              wind_direction);
    /* set timer in sea surface actor*/
    sea_surface_actor->set_timer_step_size(timer_step_size);
  }

  void start()
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

  void increment_time()
  {
    sea_surface_actor->increment_time();
  }

  void Execute(vtkObject *caller, 
               unsigned long vtkNotUsed(eventId),
               void *vtkNotUsed(callData)) override
  {
    increment_time();
    sea_surface_actor->Modified();
    
    vtkRenderWindowInteractor *interactor =
      static_cast<vtkRenderWindowInteractor*>(caller);
    interactor->Render();
  }

private:
  unsigned int timer_step_size;
  Sea_surface_actor* sea_surface_actor{nullptr};
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor;
};

int main()
{
  /* Define sea condition */
  Quantity<Units::length> wind_fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {15*Units::meter_per_second};
  /* wind direction is 30deg east of north*/
  Quantity<Units::plane_angle> wind_direction {Const::PI/6 * Units::radian};

  /* Add observer to timer event */
  Visualization visualization;
  visualization.set_sea_condition(wind_speed, wind_fetch, wind_direction);
  visualization.start();

  return EXIT_SUCCESS;
}
