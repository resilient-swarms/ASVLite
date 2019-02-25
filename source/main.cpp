#include"sea_surface_visualization.h"

using namespace asv_swarm;

int main()
{
  /* Define sea condition */
  Quantity<Units::length> fetch {100*Units::kilo*Units::meter};
  Quantity<Units::velocity> wind_speed {15*Units::meter_per_second};
  /* wind direction is 30deg east of north*/
  Quantity<Units::plane_angle> wind_direction {Const::PI/6 * Units::radian};

  /* Initialize visualization for sea surface */
  Sea_surface_visualization sea_surface_visualization {
    fetch, wind_speed, wind_direction};

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

  /* Render and interact */
  sea_surface_visualization.set_gui(renderer, window, interactor);
  window->Render();
  interactor->Start();

  return EXIT_SUCCESS;
}
