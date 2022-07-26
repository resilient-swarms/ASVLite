#include "asv_actor.h"
#include "exception.h"
#include "constants.h"
#include <iostream>
#include <iomanip>

using namespace asv_swarm;
using namespace Visualisation;

Asv_actor::Asv_actor(struct Asv* asv):
  asv{asv},
  timer_count{0},
  timer_step_size{0.0},
  current_time{0.0}
{
  // Initialise the cylinder geometry.
  cylinder = vtkSmartPointer<vtkCylinderSource>::New();
  cylinder->SetResolution(8);
  struct Asv_specification spec = asv_get_spec(asv);
  cylinder->SetRadius(spec.B_wl/2.0);
  cylinder->SetHeight(spec.D);
  cylinder->Update();

  // Initialize the mapper and actor
  cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
  asv_actor = vtkSmartPointer<vtkActor>::New();
  asv_actor->SetMapper(cylinderMapper);
  asv_actor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);

  // Set the position at time step 0
  union Coordinates_3D origin_position = asv_get_position_origin(asv);
  double x = origin_position.keys.x;
  double y = origin_position.keys.y;
  double z = origin_position.keys.z + spec.D/2.0; // The SetPosition() takes the coordinates of the centre of the ASV.
  asv_actor->SetPosition(x, y, z);
  //asv_actor->GetProperty()->SetRepresentationToWireframe();

  // Set attitude at time step 0
  // The cylinder at angle of (0,0,0) has its vertical axis along paraller to the waterline. 
  // Rotate it in so that the waterline is a circle.
  asv_actor->RotateX(90.0);
  // Now, a +ve theta rotation of the cylinder in its:
  // y-axis changes ASV heading by -theta deg,
  // x-axis changes ASV roll by theta deg towards SB side.
  // z-axis changes ASV pitch by theta deg towards aft.

  // Set the orientation of the vehicle.
  union Coordinates_3D attitude = asv_get_attitude(asv);
  yaw   = attitude.keys.z * 360.0/PI;
  roll  = attitude.keys.x * 360.0/PI;
  pitch = attitude.keys.y * 360.0/PI; 
  asv_actor->RotateY(-yaw);
  asv_actor->RotateX(roll);
  asv_actor->RotateZ(pitch);
}

void Asv_actor::increment_time() 
{
  ++timer_count; 
  current_time = static_cast<double>(timer_count) * timer_step_size; // sec
}

void Asv_actor::Execute(vtkObject* caller, unsigned long eventId,
                       void* vtkNotUsed(callData))
{
  // Set the ASV position for current time step
  struct Asv_specification spec = asv_get_spec(asv);
  union Coordinates_3D origin_position = asv_get_position_origin(asv);
  double x = origin_position.keys.x;
  double y = origin_position.keys.y;
  double z = origin_position.keys.z + spec.D/2.0; // The SetPosition() takes the coordinates of the centre of the ASV.
  asv_actor->SetPosition(x, y, z);

  // Set the ASV attitude for current time step
  union Coordinates_3D attitude = asv_get_attitude(asv);
  double new_yaw   = attitude.keys.z * 360.0/PI;
  double new_roll  = attitude.keys.x * 360.0/PI;
  double new_pitch = attitude.keys.y * 360.0/PI; 
  asv_actor->RotateY(new_yaw - yaw);
  asv_actor->RotateX(new_roll - roll);
  asv_actor->RotateZ(new_pitch - pitch);
  yaw = new_yaw;
  roll = new_roll;
  pitch = new_pitch;
}