#ifndef VTKMDIY_MPI_MPICAST_HPP
#define VTKMDIY_MPI_MPICAST_HPP

/// This header provides convinience functions to cast from diy's type erased MPI objects
/// to thier correct types.

#ifndef VTKMDIY_HAS_MPI
#  include <mpi.h>
#endif

namespace diy
{
namespace mpi
{

#define DEFINE_MPI_CAST(mpitype)                                                                              \
inline mpitype& mpi_cast(DIY_##mpitype& obj) { return *reinterpret_cast<mpitype*>(&obj); }                    \
inline const mpitype& mpi_cast(const DIY_##mpitype& obj) { return *reinterpret_cast<const mpitype*>(&obj); }  \
inline DIY_##mpitype make_DIY_##mpitype(const mpitype& obj) { DIY_##mpitype ret; mpi_cast(ret) = obj; return ret; }

DEFINE_MPI_CAST(MPI_Comm)
DEFINE_MPI_CAST(MPI_Datatype)
DEFINE_MPI_CAST(MPI_Status)
DEFINE_MPI_CAST(MPI_Request)
DEFINE_MPI_CAST(MPI_Op)
DEFINE_MPI_CAST(MPI_File)
DEFINE_MPI_CAST(MPI_Win)

#undef DEFINE_MPI_CAST

}
} // diy::mpi

#endif // VTKMDIY_MPI_MPICAST_HPP
