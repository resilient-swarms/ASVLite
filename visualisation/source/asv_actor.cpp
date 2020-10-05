#include "asv_actor.h"
#include "exception.h"
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
  cylinder = vtkCylinderSource::New();
  cylinder->SetResolution(8);
  cylinder->SetRadius(asv->spec.B_wl/2.0);
  cylinder->SetHeight(asv->spec.D);
  cylinder->Update();

  // Initialize the mapper and actor
  cylinderMapper = vtkPolyDataMapper::New();
  cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
  asv_actor = vtkActor::New();
  asv_actor->SetMapper(cylinderMapper);
  asv_actor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);

  // Set the position at time step 0
  asv_compute_dynamics(asv, 0.0);
  double x_centroid = asv->origin_position.x;
  double y_centroid = asv->origin_position.y;
  double z_centroid = asv->origin_position.z;
  asv_actor->SetPosition(x_centroid, y_centroid, z_centroid);
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
  double heading = asv->attitude.z * 360.0/PI;
  double roll = asv->attitude.x * 360.0/PI;
  double pitch = asv->attitude.y * 360.0/PI; 
  asv_actor->RotateY(-heading);
  asv_actor->RotateX(roll);
  asv_actor->RotateZ(pitch);
}

void Asv_actor::Execute(vtkObject* caller, unsigned long eventId,
                       void* vtkNotUsed(callData))
{
  double time = static_cast<double>(timer_count) * timer_step_size; // seconds
  asv_compute_dynamics(asv, time);
  // Set the orientation of the vehicle.
  double heading = asv->attitude.z * 360.0/PI;
  double roll = asv->attitude.x * 360.0/PI;
  double pitch = asv->attitude.y * 360.0/PI; 
  asv_actor->RotateY(-heading);
  asv_actor->RotateX(roll);
  asv_actor->RotateZ(pitch);
}