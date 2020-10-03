#include "asv_actor.h"
#include "exception.h"
#include <iostream>
#include <iomanip>

using namespace asv_swarm;
using namespace Visualisation;

Asv_actor::Asv_actor(struct Asv* asv):
  vtkPolyDataAlgorithm{},
  asv{asv},
  timer_count{0},
  timer_step_size{0.04},
  current_time{0}
{
  // Initialise the cylinder geometry.
  cylinder = vtkCylinderSource::New();
  cylinder->SetResolution(8);
  cylinder->SetRadius(asv->spec.B_wl/2.0);
  cylinder->SetHeight(asv->spec.D);

  // Set the position and attitude at time step 0
  double dist_cg_centroid = asv->spec.D/2.0 - asv->spec.cog.z;
  asv_compute_dynamics(asv, 0.0);
//   double x_centroid = asv->cog_position.x;
//   double y_centroid = asv->cog_position.y;
//   double z_centroid = asv->cog_position.z + dist_cg_centroid;
//   cylinder->SetCenter(x_centroid, y_centroid, z_centroid);

  // This filter does not need an input port
  SetNumberOfInputPorts(0);

  // Initialize the mapper and actor
  cylinderMapper = vtkPolyDataMapper::New();
  cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
  asv_actor = vtkActor::New();
  asv_actor->SetMapper(cylinderMapper);
  asv_actor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
  asv_actor->RotateX(30.0);
  asv_actor->RotateY(-45.0);
}

int Asv_actor::RequestData(vtkInformation* request,
                                   vtkInformationVector** inputVector,
                                   vtkInformationVector* outputVector)
{
  /*
  // Get output
  vtkPolyData* output = vtkPolyData::GetData(outputVector,0);

  // Get the timer count
  // TODO: Correct the time. It should get the repeat timer interval and
  // multiply when calculating time. Maybe a good way would be the
  // callback::Execute() set the time in sea_surface_viz and then call
  // requestdata.
  double time = static_cast<double>(timer_count) * timer_step_size; // seconds

  // Set the sea surface profile for the current time step
  set_sea_surface_elevations(time);

  // Create the points, cells and mesh
  if(sea_surface_mesh_points)
  { 
    // Points already made no need to create again. Only need to modify the 
    // z coordinates.
    unsigned int sea_surface_mesh_point_id = 0u;
    for(auto& points_row : sea_surface_points)
    {
      for(auto& point : points_row)
      {
        double x = point.x;
        double y = point.y;
        //TODO: Correct the formula for z by removing the scaling factor.
        double z = point.z;
        sea_surface_mesh_points->SetPoint(sea_surface_mesh_point_id,x,y,z); 
        ++sea_surface_mesh_point_id;
      }
    }
    sea_surface_mesh_points->Modified();
    // Update the cells
    sea_surface_mesh_cells->Modified();
  }
  else
  {
    // Initialize the sea surface mesh.

    // The first step is to create the mesh to represent the sea surface. 
    // Creating the mesh contains two steps:
    // 1. Create all the control points on the mesh
    // 2. Connect the control points and create each cell of the mesh
    // Then use the points and the connection details to create the full mesh

    // Create the control points for the mesh 
    sea_surface_mesh_points = vtkSmartPointer<vtkPoints>::New();
    sea_surface_mesh_points->SetNumberOfPoints( sea_surface_grid_size * 
                                                sea_surface_grid_size);
    // set the control points for vtk mesh 
    unsigned int sea_surface_mesh_point_id = 0u;
    for(auto& points_row : sea_surface_points)
    {
      for(auto& point : points_row)
      {
        double x = point.x;
        double y = point.y;
        //TODO: Correct the formula for z by removing the scaling factor.
        double z = point.z;
        sea_surface_mesh_points->InsertPoint(sea_surface_mesh_point_id,x,y,z); 
        ++sea_surface_mesh_point_id;
      }
    }

    // Create the cells of the mesh. The cell array can be thought of as a 
    // connectivity list. Here we specify the number of points followed by that 
    // number of point ids. This can be repeated as many times as there are 
    // primitives in the list. 
    sea_surface_mesh_cells = vtkSmartPointer<vtkCellArray>::New(); // This can 
    // be considered as an array of arrays where each array represents a square 
    // mesh that is part of the large mesh.

    // Now that the container for holding each mesh primitive has been created,
    // enter each primitive into the container.
    for(unsigned int i{0u}; i<sea_surface_grid_size-1; ++i)
    {
      for(unsigned int j{0u}; j<sea_surface_grid_size-1; ++j)
      {
        unsigned int count_nodes_in_cell {3};
        sea_surface_mesh_cells->InsertNextCell(count_nodes_in_cell);
        sea_surface_mesh_cells->InsertCellPoint(i*sea_surface_grid_size+j);
        sea_surface_mesh_cells->InsertCellPoint(i*sea_surface_grid_size+j+1);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*sea_surface_grid_size+j+1);

        sea_surface_mesh_cells->InsertNextCell(count_nodes_in_cell);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*sea_surface_grid_size+j+1);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*sea_surface_grid_size+j);
        sea_surface_mesh_cells->InsertCellPoint(i*sea_surface_grid_size+j);
      }
    }
  }
  
  // Create the mesh
  output->SetPoints(sea_surface_mesh_points);
  output->SetPolys(sea_surface_mesh_cells);
  output->Modified();
  */
  return 1;
}
