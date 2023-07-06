#ifdef VTKMDIY_MPI_AS_LIB
#include "window.hpp"
#endif

#include <algorithm>

namespace diy
{
namespace mpi
{

#ifdef VTKMDIY_MPI_AS_LIB
#  ifdef _MSC_VER
#    define EXPORT_MACRO VTKMDIY_MPI_EXPORT
#  else
#    define EXPORT_MACRO
#  endif
EXPORT_MACRO const int nocheck  = MPI_MODE_NOCHECK;
#  undef EXPORT_MACRO
#endif

namespace detail
{

DIY_MPI_Win win_create(const communicator& comm, void* base, unsigned size, int disp)
{
#if VTKMDIY_HAS_MPI
  DIY_MPI_Win win;
  MPI_Win_create(base, size, disp, MPI_INFO_NULL, mpi_cast(comm.handle()), &mpi_cast(win));
  return win;
#else
  (void)comm; (void)size; (void)disp;
  auto win = make_DIY_MPI_Win(base);
  return win;
#endif
}

void win_free(DIY_MPI_Win& win)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_free(&mpi_cast(win));
#else
  (void)win;
#endif
}

void put(const DIY_MPI_Win& win, const void* data, int count, const datatype& type, int rank, unsigned offset)
{
#if VTKMDIY_HAS_MPI
  MPI_Put(data, count, mpi_cast(type.handle), rank, offset, count, mpi_cast(type.handle), mpi_cast(win));
#else
  void* buffer = mpi_cast(win);
  size_t size = mpi_cast(type.handle);
  std::copy_n(static_cast<const int8_t*>(data),
              size * static_cast<size_t>(count),
              static_cast<int8_t*>(buffer) + (offset * size));
  (void)rank;
#endif
}

void get(const DIY_MPI_Win& win, void* data, int count, const datatype& type, int rank, unsigned offset)
{
#if VTKMDIY_HAS_MPI
  MPI_Get(data, count, mpi_cast(type.handle), rank, offset, count, mpi_cast(type.handle), mpi_cast(win));
#else
  const void* buffer = mpi_cast(win);
  size_t size = mpi_cast(type.handle);
  std::copy_n(static_cast<const int8_t*>(buffer) + (offset * size),
              size * static_cast<size_t>(count),
              static_cast<int8_t*>(data));
  (void)rank;
#endif
}

void fence(const DIY_MPI_Win& win, int assert)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_fence(assert, mpi_cast(win));
#else
  (void) win; (void) assert;
#endif
}

void lock(const DIY_MPI_Win& win, int lock_type, int rank, int assert)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_lock(lock_type, rank, assert, mpi_cast(win));
#else
  (void) win; (void) lock_type; (void) rank; (void) assert;
#endif
}

void unlock(const DIY_MPI_Win& win, int rank)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_unlock(rank, mpi_cast(win));
#else
  (void) win; (void) rank;
#endif
}

void lock_all(const DIY_MPI_Win& win, int assert)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_lock_all(assert, mpi_cast(win));
#else
  (void) win; (void) assert;
#endif
}

void unlock_all(const DIY_MPI_Win& win)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_unlock_all(mpi_cast(win));
#else
  (void) win;
#endif
}

void fetch_and_op(const DIY_MPI_Win& win,
                  const void* origin, void* result, const datatype& type,
                  int rank, unsigned offset,
                  const operation& op)
{
#if VTKMDIY_HAS_MPI
  MPI_Fetch_and_op(origin, result, mpi_cast(type.handle), rank, offset, mpi_cast(op.handle), mpi_cast(win));
#else
  (void) win; (void) origin; (void) result; (void) type; (void) rank; (void) offset; (void) op;
  VTKMDIY_UNSUPPORTED_MPI_CALL(MPI_Fetch_and_op);
#endif
}

void fetch(const DIY_MPI_Win& win, void* result, const datatype& type, int rank, unsigned offset)
{
#if VTKMDIY_HAS_MPI
  MPI_Fetch_and_op(nullptr, result, mpi_cast(type.handle), rank, offset, MPI_NO_OP, mpi_cast(win));
#else
  (void) rank;
  const void* buffer = mpi_cast(win);
  size_t size = mpi_cast(type.handle);
  std::copy_n(static_cast<const int8_t*>(buffer) + (offset * size),
              size,
              static_cast<int8_t*>(result));
#endif
}

void replace(const DIY_MPI_Win& win, const void* value, const datatype& type, int rank, unsigned offset)
{
#if VTKMDIY_HAS_MPI
  MPI_Fetch_and_op(value, nullptr, mpi_cast(type.handle), rank, offset, MPI_REPLACE, mpi_cast(win));
#else
  (void) rank;
  void* buffer = mpi_cast(win);
  size_t size = mpi_cast(type.handle);
  std::copy_n(static_cast<const int8_t*>(value),
              size,
              static_cast<int8_t*>(buffer) + (offset * size));
#endif
}

void sync(const DIY_MPI_Win& win)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_sync(mpi_cast(win));
#else
  (void) win;
#endif
}

void flush(const DIY_MPI_Win& win, int rank)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_flush(rank, mpi_cast(win));
#else
  (void) win; (void) rank;
#endif
}

void flush_all(const DIY_MPI_Win& win)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_flush_all(mpi_cast(win));
#else
  (void) win;
#endif
}

void flush_local(const DIY_MPI_Win& win, int rank)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_flush_local(rank, mpi_cast(win));
#else
  (void) win; (void) rank;
#endif
}

void flush_local_all(const DIY_MPI_Win& win)
{
#if VTKMDIY_HAS_MPI
  MPI_Win_flush_local_all(mpi_cast(win));
#else
  (void) win;
#endif
}

}
}
} // diy::mpi::detail
