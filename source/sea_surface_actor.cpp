#include "sea_surface_actor.h"

using namespace asv_swarm;
extern unsigned int timer_count;

Sea_surface_actor::Sea_surface_actor(
    Quantity<Units::velocity> wind_speed,
    Quantity<Units::length> fetch,
    Quantity<Units::plane_angle> wind_direction) :
  vtkPolyDataAlgorithm{},
  Sea_surface_dynamics{fetch, wind_speed, wind_direction}
{
  /* This filter does not need an input port */
  SetNumberOfInputPorts(0);

  /* Initialize the mapper and actor */
  sea_surface_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  sea_surface_mapper->SetInputConnection(this->GetOutputPort());
  sea_surface_actor = vtkSmartPointer<vtkActor>::New();
  sea_surface_actor->SetMapper(sea_surface_mapper);
  sea_surface_actor->GetProperty()->SetRepresentationToWireframe();
  sea_surface_actor->GetProperty()->SetColor(0,0,255); // blue waves
}

int Sea_surface_actor::RequestData(vtkInformation* request,
                                   vtkInformationVector** inputVector,
                                   vtkInformationVector* outputVector)
{
  
  /* Get output*/
  vtkPolyData* output = vtkPolyData::GetData(outputVector,0);

  /* Get the timer count */ 
  // TODO: Correct the time. It should get the repeat timer interval and
  // multiply when calculating time. Maybe a good way would be the
  // callback::Execute() set the time in sea_surface_viz and then call
  // requestdata.
  Quantity<Units::time> time {timer_count *10 * Units::milli*  Units::seconds};

  /* Set the sea surface profile for the current time step */
  this->set_sea_surface_profile(time);

  /* Create the points, cells and mesh */
  if(sea_surface_mesh_points)
  { 
    /* Points already made no need to create again. Only need to modify the 
     * z coordinates.
     */
    unsigned int sea_surface_mesh_point_id = 0u;
    for(auto control_points_row : control_points)
    {
      for(auto control_point : control_points_row)
      {
        double x = control_point.x.value();
        double y = control_point.y.value();
        //TODO: Correct the formula for z by removing the scaling factor.
        double z = control_point.z.value()*100;
        sea_surface_mesh_points->SetPoint(sea_surface_mesh_point_id,x,y,z); 
        ++sea_surface_mesh_point_id;
      }
    }
    sea_surface_mesh_points->Modified();
    /* Update the cells */
    sea_surface_mesh_cells->Modified();
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
        double x = control_point.x.value();
        double y = control_point.y.value();
        //TODO: Correct the formula for z by removing the scaling factor.
        double z = control_point.z.value()*100;
        sea_surface_mesh_points->InsertPoint(sea_surface_mesh_point_id,x,y,z); 
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
  }
  
  /* Create the mesh */
  output->SetPoints(sea_surface_mesh_points);
  output->SetPolys(sea_surface_mesh_cells);
  output->Modified();
  return 1;
}

