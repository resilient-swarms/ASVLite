#include "sea_surface_visualization.h"

using namespace asv_swarm;

Sea_surface_visualization::Sea_surface_visualization(
    Quantity<Units::length> fetch,
    Quantity<Units::velocity> wind_speed,
    Quantity<Units::plane_angle> wind_direction) :
  vtkPolyDataAlgorithm{},
  vtkCommand{},
  Sea_surface_dynamics{fetch, wind_speed, wind_direction},
  timer_count{0}
{
  /* This filter does not need an input port */
  SetNumberOfInputPorts(0);

  /* Initialize the mapper and actor */
  sea_surface_mesh_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  sea_surface_mesh_mapper->SetInputConnection(this->GetOutputPort());
  sea_surface_actor = vtkSmartPointer<vtkActor>::New();
  sea_surface_actor->SetMapper(sea_surface_mesh_mapper);
  sea_surface_actor->GetProperty()->SetRepresentationToWireframe();
  sea_surface_actor->GetProperty()->SetColor(0,0,255); // blue waves
}

void Sea_surface_visualization::set_gui(vtkRenderer* renderer,
                                        vtkRenderWindow* window,
                                        vtkRenderWindowInteractor* interactor)
{
  this->renderer = renderer;
  this->window = window;
  this->interactor = interactor;
  
  /* Add the actor to the scene */
  renderer->AddActor(sea_surface_actor);
}

int Sea_surface_visualization::RequestData(vtkInformation* request,
                                            vtkInformationVector** inputVector,
                                            vtkInformationVector* outputVector)
{
  
  /* Get output*/
  vtkPolyData* output = vtkPolyData::GetData(outputVector,0);
  
  /* Get the timer count */ 
  Quantity<Units::time> time {
    interactor->GetTimerEventId() * Units::milli * Units::seconds};

  /* Set the sea surface profile for the current time step */
  this->set_sea_surface_profile(time);

  /* Create the points, cells and mesh */
  if(sea_surface_mesh_points)
  { 
    /* Points already made no need to create again. Only need to modify the 
     * z coordinates.
     */
    for(unsigned int i{0u}; i<sea_surface_mesh_points->GetNumberOfPoints(); ++i)
    {
      /*Get the row and col for the control point from point id*/
      unsigned int col = int(i / control_points_count);
      unsigned int row = int(i % control_points_count);
 
      double* p;
      p = sea_surface_mesh_points->GetPoint(i);
      /* p[0] is the x coordinate
       * p[1] is the y coordinate
       * p[2] is the z coordinate
       * Set p[2] it the z coordinate of the corresponding control point.
       */
      double z = control_points[row][col].z.value();
      p[2] = z;
    }

    /* Create the mesh */
    output->SetPoints(sea_surface_mesh_points);
    output->SetPolys(sea_surface_mesh_cells);
  }
  else
  {
    /* Initialize the sea surface mesh */
  
    /* The first step is to create the mesh to represent the sea surface. 
     * Creating the mesh contains two steps:
     * 1. Create all the control points on the mesh
     * 2. Connect the control points and create each cell of the mesh
     * Then use the points and the connection details to create the full mesh
     */
    /* Create the control points for the mesh */
    sea_surface_mesh_points = vtkSmartPointer<vtkPoints>::New();
    sea_surface_mesh_points->SetNumberOfPoints( control_points_count * 
                                                control_points_count);
    /* set the control points for vtk mesh */
    unsigned int sea_surface_mesh_point_id = 0u;
    for(auto control_points_row : control_points)
    {
      for(auto control_point : control_points_row)
      {
        sea_surface_mesh_points->SetPoint(sea_surface_mesh_point_id, 
                                          control_point.x.value(), 
                                          control_point.y.value(),
                                          control_point.z.value()*1000);
        ++sea_surface_mesh_point_id;
      }
    }

    /* Create the cells of the mesh. The cell array can be thought of as a 
     * connectivity list. Here we specify the number of points followed by that 
     * number of point ids. This can be repeated as many times as there are 
     * primitives in the list. 
     */
    sea_surface_mesh_cells = vtkSmartPointer<vtkCellArray>::New(); /* This can 
    be considered as an array of arrays where each array represents a square 
    mesh that is part of the large mesh.*/

    /* Now that the container for holding each mesh primitive has been created,
     * enter each primitive into the container.*/
    for(unsigned int i{0u}; i<control_points_count-1; ++i)
    {
      for(unsigned int j{0u}; j<control_points_count-1; ++j)
      {
        unsigned int count_nodes_in_cell {3};
        sea_surface_mesh_cells->InsertNextCell(count_nodes_in_cell);
        sea_surface_mesh_cells->InsertCellPoint(i*control_points_count+j);
        sea_surface_mesh_cells->InsertCellPoint(i*control_points_count+j+1);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*control_points_count+j+1);

        sea_surface_mesh_cells->InsertNextCell(count_nodes_in_cell);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*control_points_count+j+1);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*control_points_count+j);
        sea_surface_mesh_cells->InsertCellPoint(i*control_points_count+j);
      }
    }

    /* Create the mesh */
    output->SetPoints(sea_surface_mesh_points);
    output->SetPolys(sea_surface_mesh_cells);
  }

  return 1;
}

