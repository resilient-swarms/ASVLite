#ifndef SEA_SURFACE_VISUALIZATION_H
#define SEA_SURFACE_VISUALIZATION_H

#include "sea_surface_dynamics.h"
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCommand.h>

namespace asv_swarm
{

/**
 * This class provides visualization for the data in the class
 * Sea_surface_dynamics.
 */
class Sea_surface_visualization : 
  public Sea_surface_dynamics,
  private vtkCommand
{
public:
  /**
   * Constructor.
   */
  Sea_surface_visualization(Quantity<Units::length> fetch,
                            Quantity<Units::velocity> wind_speed,
                            Quantity<Units::plane_angle> wind_direction);
  
  /**
   * Method to run the visualization.
   */
  void start_visualization();

protected:
  /**
   * Method to get the sea surface profile based on the frame to be rendered.
   */
  virtual void Execute(vtkObject* caller, 
                       unsigned long enentId,
                       void* vtkNotUsed(callData)) override;

private:
  vtkSmartPointer<vtkPoints> sea_surface_mesh_points;
  vtkSmartPointer<vtkCellArray> sea_surface_mesh_cells; 
  vtkSmartPointer<vtkPolyData> sea_surface_mesh;
  vtkSmartPointer<vtkPolyDataMapper> sea_surface_mesh_mapper;
  vtkSmartPointer<vtkActor> sea_surface_actor;
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> window;
  vtkSmartPointer<vtkRenderWindowInteractor> interactor;
  unsigned int time_count;

}; // class Sea_surface_visualization
} // namespace asv_swarm

#endif // SEA_SURFACE_VISUALIZATION_H
