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
#include <vtkPolyDataAlgorithm.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>

namespace asv_swarm
{

/**
 * This class provides visualization for the data in the class
 * Sea_surface_dynamics.
 */
class Sea_surface_visualization : 
  public Sea_surface_dynamics,
  public vtkPolyDataAlgorithm
{
public:
  /**
   * Constructor.
   */
  Sea_surface_visualization(Quantity<Units::length> fetch,
                            Quantity<Units::velocity> wind_speed,
                            Quantity<Units::plane_angle> wind_direction);

  /**
   * Sets renderer, window and interactor and adds the sea surface actor to the
   * renderer.
   */
  void set_gui(vtkRenderer* renderer, 
               vtkRenderWindow* window, 
               vtkRenderWindowInteractor* interactor);

protected:
  /**
   * Method to set the z values of the control points for the time passed as
   * argument.
   */
  virtual int RequestData(vtkInformation* request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector) override;
  
private:
  vtkSmartPointer<vtkPoints> sea_surface_mesh_points {nullptr};
  vtkSmartPointer<vtkCellArray> sea_surface_mesh_cells {nullptr}; 
  vtkSmartPointer<vtkPolyDataMapper> sea_surface_mesh_mapper {nullptr};
  vtkSmartPointer<vtkActor> sea_surface_actor {nullptr};
  vtkSmartPointer<vtkRenderer> renderer {nullptr};
  vtkSmartPointer<vtkRenderWindow> window {nullptr};
  vtkSmartPointer<vtkRenderWindowInteractor> interactor {nullptr};
}; // class Sea_surface_visualization
} // namespace asv_swarm

#endif // SEA_SURFACE_VISUALIZATION_H
