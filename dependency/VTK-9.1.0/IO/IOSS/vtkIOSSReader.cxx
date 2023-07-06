/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkIOSSReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkIOSSReader.h"
#include "vtkIOSSFilesScanner.h"
#include "vtkIOSSUtilities.h"

#include "vtkCellData.h"
#include "vtkDataArraySelection.h"
#include "vtkDataAssembly.h"
#include "vtkDataSet.h"
#include "vtkExtractGrid.h"
#include "vtkIdList.h"
#include "vtkInformation.h"
#include "vtkInformationIntegerKey.h"
#include "vtkInformationVector.h"
#include "vtkIntArray.h"
#include "vtkLogger.h"
#include "vtkMultiProcessController.h"
#include "vtkMultiProcessStream.h"
#include "vtkMultiProcessStreamSerialization.h"
#include "vtkObjectFactory.h"
#include "vtkPartitionedDataSet.h"
#include "vtkPartitionedDataSetCollection.h"
#include "vtkPointData.h"
#include "vtkRemoveUnusedPoints.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkStringArray.h"
#include "vtkStructuredData.h"
#include "vtkStructuredGrid.h"
#include "vtkUnsignedCharArray.h"
#include "vtkUnstructuredGrid.h"
#include "vtkVector.h"
#include "vtkVectorOperators.h"
#include "vtksys/RegularExpression.hxx"
#include "vtksys/SystemTools.hxx"

// Ioss includes
#include <vtk_ioss.h>
// clang-format off
#include VTK_IOSS(Ionit_Initializer.h)
#include VTK_IOSS(Ioss_Assembly.h)
#include VTK_IOSS(Ioss_DatabaseIO.h)
#include VTK_IOSS(Ioss_EdgeBlock.h)
#include VTK_IOSS(Ioss_EdgeSet.h)
#include VTK_IOSS(Ioss_ElementBlock.h)
#include VTK_IOSS(Ioss_ElementSet.h)
#include VTK_IOSS(Ioss_FaceBlock.h)
#include VTK_IOSS(Ioss_FaceSet.h)
#include VTK_IOSS(Ioss_IOFactory.h)
#include VTK_IOSS(Ioss_NodeBlock.h)
#include VTK_IOSS(Ioss_NodeSet.h)
#include VTK_IOSS(Ioss_Region.h)
#include VTK_IOSS(Ioss_SideBlock.h)
#include VTK_IOSS(Ioss_SideSet.h)
#include VTK_IOSS(Ioss_StructuredBlock.h)
// clang-format on

#include <array>
#include <cassert>
#include <iterator>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>

struct DatabaseParitionInfo
{
  int ProcessCount = 0;
  std::set<int> Ranks;

  bool operator==(const DatabaseParitionInfo& other) const
  {
    return this->ProcessCount == other.ProcessCount && this->Ranks == other.Ranks;
  }
};

// Opaque handle used to identify a specific Region
using DatabaseHandle = std::pair<std::string, int>;

namespace
{

template <typename T>
bool Synchronize(vtkMultiProcessController* controller, T& data, T& result)
{
  if (controller == nullptr || controller->GetNumberOfProcesses() <= 1)
  {
    return true;
  }

  vtkMultiProcessStream stream;
  stream << data;

  std::vector<vtkMultiProcessStream> all_streams;
  if (controller->AllGather(stream, all_streams))
  {
    for (auto& s : all_streams)
    {
      s >> result;
    }
    return true;
  }

  return false;
}

template <typename T>
bool Broadcast(vtkMultiProcessController* controller, T& data, int root)
{
  if (controller == nullptr || controller->GetNumberOfProcesses() <= 1)
  {
    return true;
  }
  if (controller->GetLocalProcessId() == root)
  {
    vtkMultiProcessStream stream;
    stream << data;
    return controller->Broadcast(stream, root) != 0;
  }
  else
  {
    data = T();
    vtkMultiProcessStream stream;
    if (controller->Broadcast(stream, root))
    {
      stream >> data;
      return true;
    }
    return false;
  }
}

vtkSmartPointer<vtkAbstractArray> JoinArrays(
  const std::vector<vtkSmartPointer<vtkAbstractArray>>& arrays)
{
  if (arrays.empty())
  {
    return nullptr;
  }
  else if (arrays.size() == 1)
  {
    return arrays[0];
  }

  vtkIdType numTuples = 0;
  for (auto& array : arrays)
  {
    numTuples += array->GetNumberOfTuples();
  }

  vtkSmartPointer<vtkAbstractArray> result;
  result.TakeReference(arrays[0]->NewInstance());
  result->CopyInformation(arrays[0]->GetInformation());
  result->SetName(arrays[0]->GetName());
  result->SetNumberOfComponents(arrays[0]->GetNumberOfComponents());
  result->SetNumberOfTuples(numTuples);
  vtkIdType offset = 0;
  for (auto& array : arrays)
  {
    const auto count = array->GetNumberOfTuples();
    result->InsertTuples(offset, count, 0, array);
    offset += count;
  }
  result->Modified();
  assert(offset == numTuples);
  return result;
}
}

class vtkIOSSReader::vtkInternals
{
  // it's okay to instantiate this multiple times.
  Ioss::Init::Initializer io;

  using DatabaseNamesType = std::map<std::string, DatabaseParitionInfo>;
  DatabaseNamesType UnfilteredDatabaseNames;
  DatabaseNamesType DatabaseNames;
  vtkTimeStamp DatabaseNamesMTime;

  std::map<std::string, std::set<double>> DatabaseTimes;
  std::vector<double> TimestepValues;
  vtkTimeStamp TimestepValuesMTime;

  // a collection of names for blocks and sets in the file(s).
  std::array<std::set<vtkIOSSUtilities::EntityNameType>, vtkIOSSReader::NUMBER_OF_ENTITY_TYPES>
    EntityNames;
  vtkTimeStamp SelectionsMTime;

  // Keeps track of idx of a paritioned dataset in the output.
  std::map<std::pair<Ioss::EntityType, std::string>, unsigned int> DatasetIndexMap;

  std::map<DatabaseHandle, std::shared_ptr<Ioss::Region>> RegionMap;

  vtkIOSSUtilities::Cache Cache;

  vtkIOSSUtilities::DatabaseFormatType Format = vtkIOSSUtilities::DatabaseFormatType::UNKNOWN;
  vtkIOSSReader* IOSSReader = nullptr;

  vtkSmartPointer<vtkDataAssembly> Assembly;
  vtkTimeStamp AssemblyMTime;

public:
  vtkInternals(vtkIOSSReader* reader)
    : IOSSReader(reader)
  {
  }

  Ioss::PropertyManager DatabaseProperties;
  std::set<std::string> FileNames;
  vtkTimeStamp FileNamesMTime;

  std::set<std::string> Selectors;

  const std::vector<double>& GetTimeSteps() const { return this->TimestepValues; }
  vtkIOSSUtilities::DatabaseFormatType GetFormat() const { return this->Format; }

  //@{
  /**
   * Cache related API.
   */
  void ClearCache() { this->Cache.Clear(); }
  void ResetCacheAccessCounts() { this->Cache.ResetAccessCounts(); }
  void ClearCacheUnused()
  {
    switch (this->Format)
    {
      case vtkIOSSUtilities::DatabaseFormatType::CATALYST:
        // For Catalyst, we don't want to hold on to the cache for longer than
        // the RequestData pass. For we clear it entirely here.
        this->Cache.Clear();
        break;
      default:
        this->Cache.ClearUnused();
        break;
    }
  }
  //@}

  /**
   * Processes filenames to populate names for Ioss databases to read.
   *
   * A file collection representing files partitioned across ranks where each
   * rank generate a separate file (spatial partitioning) are all represented
   * by a single Ioss database.
   *
   * Multiple Ioss databases are generated when the files are a temporal
   * in nature or represent restarts.
   *
   * This method simply uses the filenames to determine what type of files we
   * are encountering. For spatial partitions, the filenames must end with
   * '{processor-count}.{rank}'.
   *
   * @returns `false` to indicate failure.
   */
  bool UpdateDatabaseNames(vtkIOSSReader* self);

  /**
   * Read Ioss databases to generate information about timesteps / times
   * in the databases.
   *
   * This is called after successful call to `UpdateDatabaseNames` which should
   * populate the list of Ioss databases. This method iterates over all
   * databases and gathers informations about timesteps available in those
   * databases. When running in parallel, only the root node opens the Ioss
   * databases and reads the time information. That information is then
   * exchanged with all ranks thus at the end of this method all ranks should
   * have their time information updated.
   *
   * @returns `false` on failure.
   */
  bool UpdateTimeInformation(vtkIOSSReader* self);

  /**
   * Populates various `vtkDataArraySelection` objects on the vtkIOSSReader with
   * names for entity-blocks, -sets, and fields defined on them.
   */
  bool UpdateEntityAndFieldSelections(vtkIOSSReader* self);

  /**
   * Populates the vtkDataAssembly used for block/set selection.
   */
  bool UpdateAssembly(vtkIOSSReader* self, int* tag);

  vtkDataAssembly* GetAssembly() const { return this->Assembly; }

  /**
   * Fills up the output data-structure based on the entity blocks/sets chosen
   * and those available.
   */
  bool GenerateOutput(vtkPartitionedDataSetCollection* output, vtkIOSSReader* self);

  /**
   * Fills up the vtkDataAssembly with ioss-assemblies, if present.
   */
  bool ReadAssemblies(vtkPartitionedDataSetCollection* output, const DatabaseHandle& handle);

  /**
   * Reads datasets (meshes and fields) for the given block.
   */
  std::vector<vtkSmartPointer<vtkDataSet>> GetDataSets(const std::string& blockname,
    vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle, int timestep,
    vtkIOSSReader* self);

  /**
   * Read quality assurance and information data from the file.
   */
  bool GetQAAndInformationRecords(vtkFieldData* fd, const DatabaseHandle& handle);

  /**
   * Read global fields.
   */
  bool GetGlobalFields(vtkFieldData* fd, const DatabaseHandle& handle, int timestep);

  /**
   * Returns the list of fileids, if any to be read for a given "piece" for the
   * chosen timestep.
   */
  std::vector<DatabaseHandle> GetDatabaseHandles(int piece, int npieces, int timestep) const;

  /**
   * Useful for printing error messages etc.
   */
  std::string GetRawFileName(const DatabaseHandle& handle, bool shortname = false) const
  {
    auto iter = this->DatabaseNames.find(handle.first);
    if (iter == this->DatabaseNames.end())
    {
      throw std::runtime_error("bad database handle!");
    }

    const int& fileid = handle.second;
    auto dbasename = shortname ? vtksys::SystemTools::GetFilenameName(handle.first) : handle.first;

    auto& dinfo = iter->second;
    if (dinfo.ProcessCount > 0)
    {
      return Ioss::Utils::decode_filename(
        dbasename, dinfo.ProcessCount, *std::next(dinfo.Ranks.begin(), fileid));
    }
    return dbasename;
  }

  /**
   * For spatially partitioned files, this returns the partition identifier for
   * the file identified by the handle.
   */
  int GetFileProcessor(const DatabaseHandle& handle) const
  {
    auto iter = this->DatabaseNames.find(handle.first);
    if (iter == this->DatabaseNames.end())
    {
      throw std::runtime_error("bad database handle!");
    }
    const int& fileid = handle.second;
    auto& dinfo = iter->second;
    if (dinfo.ProcessCount > 0)
    {
      return *std::next(dinfo.Ranks.begin(), fileid);
    }

    // this is not a spatially partitioned file; just return 0.
    return 0;
  }

  /**
   * Releases any open file handles.
   */
  void ReleaseHandles()
  {
    // RegionMap is where all the handles are kept. All we need to do is release
    // them.
    for (const auto& pair : this->RegionMap)
    {
      pair.second->get_database()->closeDatabase();
    }
  }

  /**
   * Clear all regions, databases etc.
   */
  void Reset()
  {
    this->Cache.Clear();
    this->RegionMap.clear();
    this->TimestepValuesMTime = vtkTimeStamp();
  }

private:
  std::vector<int> GetFileIds(const std::string& dbasename, int myrank, int numRanks) const;
  Ioss::Region* GetRegion(const std::string& dbasename, int fileid);
  Ioss::Region* GetRegion(const DatabaseHandle& handle)
  {
    return this->GetRegion(handle.first, handle.second);
  }

  //@{
  /**
   * Reads a field with name `fieldname` from entity block or set with chosen name
   * (`blockname`) and type (`vtk_entity_type`). Field may be a result
   * field which can be time-varying. In that case, `timestep` is used to
   * identify the timestep to read.
   *
   * Returns non-null array on success. Returns nullptr if block or field is
   * missing (which is not an error condition).
   *
   * On error, `std::runtime_error` is thrown.
   */
  vtkSmartPointer<vtkAbstractArray> GetField(const std::string& fieldname, Ioss::Region* region,
    Ioss::GroupingEntity* group_entity, const DatabaseHandle& handle, int timestep,
    vtkIdTypeArray* ids_to_extract = nullptr, const std::string& cache_key_suffix = std::string());
  //@}

  /**
   * Fill up the `grid` with connectivity information for the entity block (or
   * set) with the given name (`blockname`) and type (vtk_entity_type).
   *
   * `handle` is the database / file handle for the current piece / rank
   * obtained by calling `GetDatabaseHandles`.
   *
   * Returns true on success. `false` will be returned when the handle doesn't
   * have the chosen blockname/entity.
   *
   * On file reading error, `std::runtime_error` is thrown.
   */
  bool GetTopology(vtkUnstructuredGrid* grid, const std::string& blockname,
    vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle);

  /**
   * Fill up `grid` with point coordinates aka geometry read from the block
   * with the given name (`blockname`). The point coordinates are always
   * read from a block of type NODEBLOCK.
   *
   * `handle` is the database / file handle for the current piece / rank
   * obtained by calling `GetDatabaseHandles`.
   *
   * Returns true on success.
   *
   * On file reading error, `std::runtime_error` is thrown.
   */
  bool GetGeometry(
    vtkUnstructuredGrid* grid, const std::string& blockname, const DatabaseHandle& handle);

  /**
   * GetGeometry for vtkStructuredGrid i.e. CGNS.
   */
  bool GetGeometry(vtkStructuredGrid* grid, const Ioss::StructuredBlock* groupEntity);

  /**
   * Adds geometry (points) and topology (cell) information to the grid for the
   * entity block or set chosen using the name (`blockname`) and type
   * (`vtk_entity_type`).
   *
   * `handle` is the database / file handle for the current piece / rank
   * obtained by calling `GetDatabaseHandles`.
   *
   * If `remove_unused_points` is true, any points that are not used by the
   * cells are removed. When that is done, an array called
   * `__vtk_mesh_original_pt_ids__` is added to the cache for the entity
   * which can be used to identify which points were passed through.
   */
  bool GetMesh(vtkUnstructuredGrid* grid, const std::string& blockname,
    vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle,
    bool remove_unused_points);

  /**
   * Reads a structured block. vtk_entity_type must be
   * `vtkIOSSReader::STRUCTUREDBLOCK`.
   */
  bool GetMesh(vtkStructuredGrid* grid, const std::string& blockname,
    vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle);

  /**
   * Add "id" array to the dataset using the id for the groupping entity, if
   * any. The array named "object_id" is added as a cell-data array to follow
   * the pattern used by vtkExodusIIReader.
   */
  bool GenerateEntityIdArray(vtkDataSet* grid, const std::string& blockname,
    vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle);

  /**
   * Reads selected field arrays for the given entity block or set.
   * If `read_ioss_ids` is true, then element ids are read as applicable.
   *
   * `ids_to_extract`, when specified, is a `vtkIdTypeArray` identifying the
   * subset of indices to produce in the output. This is used for point data fields
   * when the mesh was generated with `remove_unused_points` on. This ensures
   * that point data arrays match the points. When `ids_to_extract` is provided,
   * for the caching to work correctly, the `cache_key_suffix` must be set to
   * the name of the entity block (or set) which provided the cells to determine
   * which points to extract.
   *
   * Returns true on success.
   *
   * On error, `std::runtime_error` is thrown.
   */
  bool GetFields(vtkDataSetAttributes* dsa, vtkDataArraySelection* selection, Ioss::Region* region,
    Ioss::GroupingEntity* group_entity, const DatabaseHandle& handle, int timestep,
    bool read_ioss_ids, vtkIdTypeArray* ids_to_extract = nullptr,
    const std::string& cache_key_suffix = std::string());

  /**
   * This reads node fields for an entity block or set.
   *
   * Internally calls `GetFields()` with correct values for `ids_to_extract` and
   * `cache_key_suffix`.
   *
   */
  bool GetNodeFields(vtkDataSetAttributes* dsa, vtkDataArraySelection* selection,
    Ioss::Region* region, Ioss::GroupingEntity* group_entity, const DatabaseHandle& handle,
    int timestep, bool read_ioss_ids);

  /**
   * Reads node block array with displacements and then transforms
   * the points in the grid using those displacements.
   */
  bool ApplyDisplacements(vtkPointSet* grid, Ioss::Region* region,
    Ioss::GroupingEntity* group_entity, const DatabaseHandle& handle, int timestep);

  /**
   * Adds 'file_id' array to indicate which file the dataset was read from.
   */
  bool GenerateFileId(
    vtkDataSet* grid, Ioss::GroupingEntity* group_entity, const DatabaseHandle& handle);

  /**
   * Fields like "ids" have to be vtkIdTypeArray in VTK. This method does the
   * conversion if needed.
   */
  vtkSmartPointer<vtkAbstractArray> ConvertFieldForVTK(vtkAbstractArray* array)
  {
    if (array == nullptr || array->GetName() == nullptr || strcmp(array->GetName(), "ids") != 0)
    {
      return array;
    }

    if (vtkIdTypeArray::SafeDownCast(array))
    {
      return array;
    }

    vtkNew<vtkIdTypeArray> ids;
    ids->DeepCopy(array);
    return ids;
  }

  unsigned int GetDataSetIndexForEntity(const Ioss::GroupingEntity* entity) const
  {
    return this->DatasetIndexMap.at(std::make_pair(entity->type(), entity->name()));
  }

  ///@{
  /**
   * Called by `GetDataSets` to process each type of dataset.
   * There's slight difference in how they are handled and hence these separate methods.
   */
  std::vector<vtkSmartPointer<vtkDataSet>> GetExodusDataSets(const std::string& blockname,
    vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle, int timestep,
    vtkIOSSReader* self);

  std::vector<vtkSmartPointer<vtkDataSet>> GetCGNSDataSets(const std::string& blockname,
    vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle, int timestep,
    vtkIOSSReader* self);
  ///@}

  bool BuildAssembly(Ioss::Region* region, vtkDataAssembly* assembly, int root, bool add_leaves);

  /**
   * Generate a subset based the readers current settings for FileRange and
   * FileStride.
   */
  DatabaseNamesType GenerateSubset(const DatabaseNamesType& databases, vtkIOSSReader* self);
};

//----------------------------------------------------------------------------
std::vector<int> vtkIOSSReader::vtkInternals::GetFileIds(
  const std::string& dbasename, int myrank, int numRanks) const
{
  auto iter = this->DatabaseNames.find(dbasename);
  if ((iter == this->DatabaseNames.end()) || (myrank < 0) ||
    (iter->second.ProcessCount == 0 && myrank != 0) ||
    (iter->second.ProcessCount != 0 && myrank >= iter->second.ProcessCount))
  {
    return std::vector<int>();
  }

  // note, number of files may be less than the number of ranks the partitioned
  // file was written out on. that happens when user only chooses a smaller
  // subset.
  int nfiles = iter->second.ProcessCount > 0 ? static_cast<int>(iter->second.Ranks.size()) : 1;

  // this logic is same as diy::ContiguousAssigner::local_gids(..)
  // the goal is split the available set of files into number of ranks in
  // continguous chunks.
  const int div = nfiles / numRanks;
  const int mod = nfiles % numRanks;

  int from, to;
  if (myrank < mod)
  {
    from = myrank * (div + 1);
  }
  else
  {
    from = mod * (div + 1) + (myrank - mod) * div;
  }

  if (myrank + 1 < mod)
  {
    to = (myrank + 1) * (div + 1);
  }
  else
  {
    to = mod * (div + 1) + (myrank + 1 - mod) * div;
  }

  std::vector<int> fileids;
  for (int fileid = from; fileid < to; ++fileid)
  {
    fileids.push_back(fileid);
  }
  return fileids;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::UpdateDatabaseNames(vtkIOSSReader* self)
{
  if (this->DatabaseNamesMTime > this->FileNamesMTime)
  {
    // we may still need filtering if MTime changed, so check that.
    if (self->GetMTime() > this->DatabaseNamesMTime)
    {
      auto subset = this->GenerateSubset(this->UnfilteredDatabaseNames, self);
      if (this->DatabaseNames != subset)
      {
        this->DatabaseNames = std::move(subset);
        this->DatabaseNamesMTime.Modified();
      }
    }
    return (!this->DatabaseNames.empty());
  }

  // Clear cache since we're updating the databases, old caches no longer makes
  // sense.
  this->Cache.Clear();

  // Clear old Ioss::Region's since they may not be correct anymore.
  this->RegionMap.clear();

  auto filenames = this->FileNames;
  auto controller = self->GetController();
  const int myrank = controller ? controller->GetLocalProcessId() : 0;

  if (myrank == 0)
  {
    if (filenames.size() == 1 && vtkIOSSFilesScanner::IsMetaFile(*filenames.begin()))
    {
      filenames = vtkIOSSFilesScanner::GetFilesFromMetaFile(*filenames.begin());
    }
    else if (self->GetScanForRelatedFiles())
    {
      filenames = vtkIOSSFilesScanner::GetRelatedFiles(filenames);
    }
  }

  if (!::Broadcast(controller, filenames, 0))
  {
    return false;
  }

  if (filenames.empty())
  {
    vtkErrorWithObjectMacro(self, "No filename specified.");
    return false;
  }

  // process filename to determine the base-name and the `processor_count`, and
  // `my_processor` values.
  // clang-format off
  vtksys::RegularExpression regEx(R"(^(.*)\.([0-9]+)\.([0-9]+)$)");
  // clang-format on

  DatabaseNamesType databases;
  for (auto& fname : filenames)
  {
    if (regEx.find(fname))
    {
      auto dbasename = regEx.match(1);
      auto processor_count = std::atoi(regEx.match(2).c_str());
      auto my_processor = std::atoi(regEx.match(3).c_str());

      auto& info = databases[dbasename];
      if (info.ProcessCount == 0 || info.ProcessCount == processor_count)
      {
        info.ProcessCount = processor_count;
        info.Ranks.insert(my_processor);
      }
      else
      {
        auto fname_name = vtksys::SystemTools::GetFilenameName(fname);
        vtkErrorWithObjectMacro(self,
          "Filenames specified use inconsistent naming schemes. '"
            << fname_name << "' has incorrect processor-count (" << processor_count << "), '"
            << info.ProcessCount << "' was expected.");
        return false;
      }
    }
    else
    {
      databases.insert(std::make_pair(fname, DatabaseParitionInfo()));
    }
  }

  this->UnfilteredDatabaseNames.swap(databases);

  if (vtkLogger::GetCurrentVerbosityCutoff() >= vtkLogger::VERBOSITY_TRACE)
  {
    // let's log.
    vtkLogF(
      TRACE, "Found Ioss databases (%d)", static_cast<int>(this->UnfilteredDatabaseNames.size()));
    std::ostringstream str;
    for (const auto& pair : this->UnfilteredDatabaseNames)
    {
      if (pair.second.ProcessCount > 0)
      {
        // reset ostringstream.
        str.str("");
        str.clear();
        for (auto& rank : pair.second.Ranks)
        {
          str << " " << rank;
        }
        vtkLogF(TRACE, "'%s' [processor_count = %d][ranks = %s]",
          vtksys::SystemTools::GetFilenameName(pair.first).c_str(), pair.second.ProcessCount,
          str.str().c_str());
      }
      else
      {
        vtkLogF(TRACE, "'%s'", vtksys::SystemTools::GetFilenameName(pair.first).c_str());
      }
    }
  }

  this->DatabaseNames = this->GenerateSubset(this->UnfilteredDatabaseNames, self);
  this->DatabaseNamesMTime.Modified();
  return !this->DatabaseNames.empty();
}

//----------------------------------------------------------------------------
vtkIOSSReader::vtkInternals::DatabaseNamesType vtkIOSSReader::vtkInternals::GenerateSubset(
  const vtkIOSSReader::vtkInternals::DatabaseNamesType& databases, vtkIOSSReader* self)
{
  int fileRange[2];
  self->GetFileRange(fileRange);
  const int stride = self->GetFileStride();
  if (fileRange[0] >= fileRange[1] || stride < 1 || databases.empty())
  {
    return databases;
  }

  // We need to filter filenames.
  DatabaseNamesType result = databases;
  for (auto& pair : result)
  {
    auto& dbaseInfo = pair.second;
    if (dbaseInfo.ProcessCount <= 0)
    {
      continue;
    }

    // remove all "ranks" not fitting the requested range.
    for (auto iter = dbaseInfo.Ranks.begin(); iter != dbaseInfo.Ranks.end();)
    {
      const int rank = (*iter);
      if ((rank < fileRange[0] || rank >= fileRange[1] || (rank - fileRange[0]) % stride != 0))
      {
        iter = dbaseInfo.Ranks.erase(iter);
      }
      else
      {
        ++iter;
      }
    }
  }

  // remove any databases which have no ranks to be read in.
  for (auto iter = result.begin(); iter != result.end();)
  {
    auto& dbaseInfo = iter->second;
    if (dbaseInfo.ProcessCount > 0 && dbaseInfo.Ranks.empty())
    {
      iter = result.erase(iter);
    }
    else
    {
      ++iter;
    }
  }
  return result;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::UpdateTimeInformation(vtkIOSSReader* self)
{
  if (this->TimestepValuesMTime > this->DatabaseNamesMTime)
  {
    return true;
  }

  vtkLogScopeF(TRACE, "UpdateTimeInformation");
  auto controller = self->GetController();
  const auto rank = controller ? controller->GetLocalProcessId() : 0;
  const auto numRanks = controller ? controller->GetNumberOfProcesses() : 1;

  int success = 1;
  if (rank == 0)
  {
    // time values for each database.
    std::map<std::string, std::set<double>> dbase_times;

    // read all databases to collect timestep information.
    for (const auto& pair : this->DatabaseNames)
    {
      assert(pair.second.ProcessCount == 0 || !pair.second.Ranks.empty());
      const auto fileids = this->GetFileIds(pair.first, rank, numRanks);
      if (fileids.empty())
      {
        continue;
      }
      try
      {
        auto region = this->GetRegion(pair.first, fileids.front());
        dbase_times[pair.first] = vtkIOSSUtilities::GetTimeValues(region);
      }
      catch (std::runtime_error& e)
      {
        vtkErrorWithObjectMacro(self, "Error in UpdateTimeInformation: \n" << e.what());
        success = 0;
        dbase_times.clear();
        break;
      }
    }

    this->DatabaseTimes.swap(dbase_times);
  }

  if (numRanks > 1)
  {
    auto& dbase_times = this->DatabaseTimes;
    int msg[2] = { success, static_cast<int>(dbase_times.size()) };
    controller->Broadcast(msg, 2, 0);
    success = msg[0];
    if (success && msg[1] > 0)
    {
      success = ::Broadcast(controller, dbase_times, 0);
    }
    else
    {
      dbase_times.clear();
    }

    // this is a good place for us to sync up format too.
    int iFormat = static_cast<int>(this->Format);
    controller->Broadcast(&iFormat, 1, 0);
    this->Format = static_cast<vtkIOSSUtilities::DatabaseFormatType>(iFormat);
  }

  // Fillup TimestepValues for ease of use later.
  std::set<double> times_set;
  for (auto& pair : this->DatabaseTimes)
  {
    std::copy(pair.second.begin(), pair.second.end(), std::inserter(times_set, times_set.end()));
  }
  this->TimestepValues.resize(times_set.size());
  std::copy(times_set.begin(), times_set.end(), this->TimestepValues.begin());
  this->TimestepValuesMTime.Modified();
  return (success == 1);
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::UpdateEntityAndFieldSelections(vtkIOSSReader* self)
{
  if (this->SelectionsMTime > this->DatabaseNamesMTime)
  {
    return true;
  }

  vtkLogScopeF(TRACE, "UpdateEntityAndFieldSelections");
  auto controller = self->GetController();
  const auto rank = controller ? controller->GetLocalProcessId() : 0;
  const auto numRanks = controller ? controller->GetNumberOfProcesses() : 1;

  // This has to be done all all ranks since not all files in a database have
  // all the blocks consequently need not have all the fields.
  std::array<std::set<vtkIOSSUtilities::EntityNameType>, vtkIOSSReader::NUMBER_OF_ENTITY_TYPES>
    entity_names;
  std::array<std::set<std::string>, vtkIOSSReader::NUMBER_OF_ENTITY_TYPES> field_names;
  std::set<vtkIOSSUtilities::EntityNameType> bc_names;

  // format should have been set (and synced) across all ranks by now.
  assert(this->Format != vtkIOSSUtilities::UNKNOWN);

  // When each rank is reading multiple files, reading all those files for
  // gathering meta-data can be slow. However, with CGNS, that is required
  // since the file doesn't have information about all blocks in all files.
  // see paraview/paraview#20873.
  const bool readAllFilesForMetaData = (this->Format == vtkIOSSUtilities::DatabaseFormatType::CGNS);

  for (const auto& pair : this->DatabaseNames)
  {
    auto fileids = this->GetFileIds(pair.first, rank, numRanks);
    if (!readAllFilesForMetaData && fileids.size() > 1)
    {
      // reading 1 file is adequate, and that too on rank 0 alone.
      fileids.resize(rank == 0 ? 1 : 0);
    }

    for (const auto& fileid : fileids)
    {
      if (auto region = this->GetRegion(pair.first, fileid))
      {
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_node_blocks(),
          entity_names[vtkIOSSReader::NODEBLOCK], field_names[vtkIOSSReader::NODEBLOCK]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_edge_blocks(),
          entity_names[vtkIOSSReader::EDGEBLOCK], field_names[vtkIOSSReader::EDGEBLOCK]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_face_blocks(),
          entity_names[vtkIOSSReader::FACEBLOCK], field_names[vtkIOSSReader::FACEBLOCK]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_element_blocks(),
          entity_names[vtkIOSSReader::ELEMENTBLOCK], field_names[vtkIOSSReader::ELEMENTBLOCK]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_structured_blocks(),
          entity_names[vtkIOSSReader::STRUCTUREDBLOCK],
          field_names[vtkIOSSReader::STRUCTUREDBLOCK]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_nodesets(),
          entity_names[vtkIOSSReader::NODESET], field_names[vtkIOSSReader::NODESET]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_edgesets(),
          entity_names[vtkIOSSReader::EDGESET], field_names[vtkIOSSReader::EDGESET]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_facesets(),
          entity_names[vtkIOSSReader::FACESET], field_names[vtkIOSSReader::FACESET]);
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_elementsets(),
          entity_names[vtkIOSSReader::ELEMENTSET], field_names[vtkIOSSReader::ELEMENTSET]);

        // note: for CGNS, the sidesets contain family names for BC. They need to
        // be handled differently from exodus side sets.
        vtkIOSSUtilities::GetEntityAndFieldNames(region, region->get_sidesets(),
          entity_names[vtkIOSSReader::SIDESET], field_names[vtkIOSSReader::SIDESET]);

        // note: for CGNS, the structuredblock elements have nested BC patches. These patches
        // are named as well. Let's collect those names too.
        for (const auto& sb : region->get_structured_blocks())
        {
          const int64_t id = sb->property_exists("id") ? sb->get_property("id").get_int() : 0;
          for (auto& bc : sb->m_boundaryConditions)
          {
            if (!bc.m_bcName.empty())
            {
              bc_names.emplace(static_cast<vtkTypeUInt64>(id), bc.m_bcName);
            }
          }
        }

        // another CGNS idiosyncrasy, we need to read node fields from
        // node_blocks nested under the structured_blocks.
        for (auto& sb : region->get_structured_blocks())
        {
          std::set<vtkIOSSUtilities::EntityNameType> unused;
          vtkIOSSUtilities::GetEntityAndFieldNames(region,
            Ioss::NodeBlockContainer({ &sb->get_node_block() }), unused,
            field_names[vtkIOSSReader::NODEBLOCK]);
        }
      }
      // necessary to avoid errors from IO libraries, e.g. CGNS, about
      // too many files open.
      this->ReleaseHandles();
    }
  }

  if (numRanks > 1)
  {
    //// sync selections across all ranks.
    ::Synchronize(controller, entity_names, entity_names);
    ::Synchronize(controller, field_names, field_names);

    // Sync format. Needed since all ranks may not have read entity information
    // thus may not have format setup correctly.
    int iFormat = static_cast<int>(this->Format);
    controller->Broadcast(&iFormat, 1, 0);
    this->Format = static_cast<vtkIOSSUtilities::DatabaseFormatType>(iFormat);
  }

  // update known block/set names.
  this->EntityNames = entity_names;
  for (int cc = ENTITY_START; cc < ENTITY_END; ++cc)
  {
    auto entitySelection = self->GetEntitySelection(cc);
    for (auto& name : entity_names[cc])
    {
      entitySelection->AddArray(name.second.c_str(), vtkIOSSReader::GetEntityTypeIsBlock(cc));
    }

    auto fieldSelection = self->GetFieldSelection(cc);
    for (auto& name : field_names[cc])
    {
      fieldSelection->AddArray(name.c_str(), vtkIOSSReader::GetEntityTypeIsBlock(cc));
    }
  }

  // Populate DatasetIndexMap.
  unsigned int pdsIdx = 0;
  for (int etype = vtkIOSSReader::NODEBLOCK + 1; etype < vtkIOSSReader::ENTITY_END; ++etype)
  {
    // for sidesets when reading CGNS, use the patch names.
    const auto& namesSet = this->EntityNames[etype];

    // EntityNames are sorted by their exodus "id".
    for (const auto& ename : namesSet)
    {
      auto ioss_etype =
        vtkIOSSUtilities::GetIOSSEntityType(static_cast<vtkIOSSReader::EntityType>(etype));
      this->DatasetIndexMap[std::make_pair(ioss_etype, ename.second)] = pdsIdx++;
    }
  }

  this->SelectionsMTime.Modified();
  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::BuildAssembly(
  Ioss::Region* region, vtkDataAssembly* assembly, int root, bool add_leaves)
{
  if (region == nullptr || assembly == nullptr)
  {
    return false;
  }

  // assemblies in Ioss are simply stored as a vector. we need to build graph
  // from that vector of assemblies.
  std::set<const Ioss::GroupingEntity*> root_assemblies;
  for (auto& ioss_assembly : region->get_assemblies())
  {
    assert(ioss_assembly != nullptr);
    root_assemblies.insert(ioss_assembly);

    for (auto child : ioss_assembly->get_members())
    {
      // a child cannot be a root, so remove it.
      root_assemblies.erase(child);
    }
  }

  if (root_assemblies.empty())
  {
    return false;
  }

  std::function<void(const Ioss::Assembly*, int)> processAssembly;
  processAssembly = [&assembly, &processAssembly, &add_leaves, this](
                      const Ioss::Assembly* ioss_assembly, int parent) {
    auto node = assembly->AddNode(
      vtkDataAssembly::MakeValidNodeName(ioss_assembly->name().c_str()).c_str(), parent);
    assembly->SetAttribute(node, "label", ioss_assembly->name().c_str());
    if (ioss_assembly->get_member_type() == Ioss::ASSEMBLY)
    {
      for (auto& child : ioss_assembly->get_members())
      {
        processAssembly(dynamic_cast<const Ioss::Assembly*>(child), node);
      }
    }
    else
    {
      for (auto& child : ioss_assembly->get_members())
      {
        int dsnode = node;
        if (add_leaves)
        {
          dsnode = assembly->AddNode(
            vtkDataAssembly::MakeValidNodeName(child->name().c_str()).c_str(), node);
          assembly->SetAttribute(dsnode, "label", child->name().c_str());
        }
        assembly->AddDataSetIndex(dsnode, this->GetDataSetIndexForEntity(child));
      }
    }
  };

  // to preserve order of assemblies, we iterate over region assemblies.
  for (auto& ioss_assembly : region->get_assemblies())
  {
    if (root_assemblies.find(ioss_assembly) != root_assemblies.end())
    {
      processAssembly(ioss_assembly, root);
    }
  }

  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::UpdateAssembly(vtkIOSSReader* self, int* tag)
{
  if (this->AssemblyMTime > this->DatabaseNamesMTime)
  {
    return true;
  }

  vtkLogScopeF(TRACE, "UpdateAssembly");
  this->AssemblyMTime.Modified();

  auto controller = self->GetController();
  const auto rank = controller ? controller->GetLocalProcessId() : 0;
  const auto numRanks = controller ? controller->GetNumberOfProcesses() : 1;

  if (rank == 0)
  {
    // it's unclear how assemblies in Ioss are distributed across partitioned
    // files. so we assume they are duplicated on all only read it from root node.
    const auto handle = this->GetDatabaseHandles(rank, numRanks, 0).front();
    auto region = this->GetRegion(handle);

    this->Assembly = vtk::TakeSmartPointer(vtkDataAssembly::New());
    this->Assembly->SetRootNodeName("Assemblies");
    const auto status = this->BuildAssembly(region, this->Assembly, 0, /*add_leaves=*/true);
    *tag = status ? static_cast<int>(this->AssemblyMTime.GetMTime()) : 0;
    if (numRanks > 1)
    {
      vtkMultiProcessStream stream;

      stream << (*tag);
      stream << this->Assembly->SerializeToXML(vtkIndent());
      controller->Broadcast(stream, 0);
    }
    if (!status)
    {
      this->Assembly = nullptr;
    }
  }
  else
  {
    vtkMultiProcessStream stream;
    controller->Broadcast(stream, 0);

    std::string data;
    stream >> (*tag) >> data;

    if ((*tag) != 0)
    {
      this->Assembly = vtk::TakeSmartPointer(vtkDataAssembly::New());
      this->Assembly->InitializeFromXML(data.c_str());
    }
    else
    {
      this->Assembly = nullptr;
    }
  }

  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GenerateOutput(
  vtkPartitionedDataSetCollection* output, vtkIOSSReader*)
{
  // we skip NODEBLOCK since we never put out NODEBLOCK in the output by itself.
  vtkNew<vtkDataAssembly> assembly;
  assembly->SetRootNodeName("IOSS");
  output->SetDataAssembly(assembly);

  for (int etype = vtkIOSSReader::NODEBLOCK + 1; etype < vtkIOSSReader::ENTITY_END; ++etype)
  {
    // for sidesets when reading CGNS, use the patch names.
    const auto& namesSet = this->EntityNames[etype];

    if (namesSet.empty())
    {
      // skip 0-count entity types; keeps output assembly simpler to read.
      continue;
    }

    const int entity_node =
      assembly->AddNode(vtkIOSSReader::GetDataAssemblyNodeNameForEntityType(etype));

    // EntityNames are sorted by their exodus "id".
    for (const auto& ename : namesSet)
    {
      const auto pdsIdx = output->GetNumberOfPartitionedDataSets();
      vtkNew<vtkPartitionedDataSet> parts;
      output->SetPartitionedDataSet(pdsIdx, parts);
      output->GetMetaData(pdsIdx)->Set(vtkCompositeDataSet::NAME(), ename.second.c_str());
      output->GetMetaData(pdsIdx)->Set(
        vtkIOSSReader::ENTITY_TYPE(), etype); // save for vtkIOSSReader use.
      auto node = assembly->AddNode(
        vtkDataAssembly::MakeValidNodeName(ename.second.c_str()).c_str(), entity_node);
      assembly->SetAttribute(node, "label", ename.second.c_str());
      assembly->AddDataSetIndex(node, pdsIdx);
    }
  }

  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::ReadAssemblies(
  vtkPartitionedDataSetCollection* output, const DatabaseHandle& handle)
{
  /**
   * It's not entirely clear how IOSS-assemblies should be made available in the data
   * model. For now, we'll add them under the default vtkDataAssembly associated
   * with the output
   **/
  auto assembly = output->GetDataAssembly();
  assert(assembly != nullptr);

  auto region = this->GetRegion(handle);
  if (!region)
  {
    return false;
  }

  const auto node_assemblies = assembly->AddNode("assemblies");
  if (!this->BuildAssembly(region, assembly, node_assemblies, /*add_leaves=*/true))
  {
    assembly->RemoveNode(node_assemblies);
  }

  return true;
}

//----------------------------------------------------------------------------
Ioss::Region* vtkIOSSReader::vtkInternals::GetRegion(const std::string& dbasename, int fileid)
{
  assert(fileid >= 0);
  auto iter = this->DatabaseNames.find(dbasename);
  assert(iter != this->DatabaseNames.end());

  const bool has_multiple_files = (iter->second.ProcessCount > 0);
  assert(has_multiple_files == false || (fileid < static_cast<int>(iter->second.Ranks.size())));

  auto processor = has_multiple_files ? *std::next(iter->second.Ranks.begin(), fileid) : 0;

  auto riter = this->RegionMap.find(std::make_pair(dbasename, processor));
  if (riter == this->RegionMap.end())
  {
    Ioss::PropertyManager properties;
    if (has_multiple_files)
    {
      properties.add(Ioss::Property("my_processor", processor));
      properties.add(Ioss::Property("processor_count", iter->second.ProcessCount));
    }

    // fixme: should this be configurable? it won't really work if we made it
    // configurable since our vtkDataArraySelection object would need to purged
    // and refilled.
    properties.add(Ioss::Property("FIELD_SUFFIX_SEPARATOR", ""));

    // tell the reader to read all blocks, even if empty. necessary to avoid
    // having to read all files to gather metadata, if possible
    // see paraview/paraview#20873.
    properties.add(Ioss::Property("RETAIN_EMPTY_BLOCKS", "on"));

    // Fillup with user-specified properties.
    Ioss::NameList names;
    this->DatabaseProperties.describe(&names);

    for (const auto& name : names)
    {
      properties.add(this->DatabaseProperties.get(name));
    }

    // If MPI is enabled in the build, Ioss can call MPI routines. We need to
    // make sure that MPI is initialized before calling
    // Ioss::IOFactory::create.
    vtkIOSSUtilities::InitializeEnvironmentForIOSS();
    std::string dtype;
    switch (vtkIOSSUtilities::DetectType(dbasename))
    {
      case vtkIOSSUtilities::DatabaseFormatType::CGNS:
        dtype = "cgns";
        break;
      case vtkIOSSUtilities::DatabaseFormatType::CATALYST:
        dtype = "catalyst";
        break;
      case vtkIOSSUtilities::DatabaseFormatType::EXODUS:
      default:
        dtype = "exodusII";
        break;
    }

    auto dbase = std::unique_ptr<Ioss::DatabaseIO>(Ioss::IOFactory::create(
      this->IOSSReader->DatabaseTypeOverride ? std::string(this->IOSSReader->DatabaseTypeOverride)
                                             : dtype,
      dbasename, Ioss::READ_RESTART, MPI_COMM_WORLD, properties));
    if (dbase == nullptr || !dbase->ok(/*write_message=*/true))
    {
      throw std::runtime_error(
        "Failed to open database " + this->GetRawFileName(DatabaseHandle{ dbasename, fileid }));
    }
    dbase->set_surface_split_type(Ioss::SPLIT_BY_TOPOLOGIES);

    // note: `Ioss::Region` constructor may throw exception.
    auto region = std::make_shared<Ioss::Region>(dbase.get());

    // release the dbase ptr since region (if created successfully) takes over
    // the ownership and calls delete on it when done.
    (void)dbase.release();

    riter =
      this->RegionMap.insert(std::make_pair(std::make_pair(dbasename, processor), region)).first;

    if (this->Format != vtkIOSSUtilities::DatabaseFormatType::UNKNOWN &&
      this->Format != vtkIOSSUtilities::GetFormat(region.get()))
    {
      throw std::runtime_error("Format mismatch! This is unexpected and indicate an error "
                               "in the reader implementation.");
    }
    this->Format = vtkIOSSUtilities::GetFormat(region.get());
  }
  return riter->second.get();
}

//----------------------------------------------------------------------------
std::vector<DatabaseHandle> vtkIOSSReader::vtkInternals::GetDatabaseHandles(
  int piece, int npieces, int timestep) const
{
  std::string dbasename;
  if (timestep >= 0 && timestep < static_cast<int>(this->TimestepValues.size()))
  {
    const double time = this->TimestepValues[timestep];

    // find the right database in a set of restarts;
    for (const auto& pair : this->DatabaseTimes)
    {
      if (pair.second.find(time) != pair.second.end())
      {
        // if multiple databases provide the same timestep, we opt to choose
        // the one with a newer end timestep. this follows from the fact that
        // often a restart may be started after "rewinding" a bit to overcome
        // some bad timesteps.
        if (dbasename.empty() ||
          (*this->DatabaseTimes.at(dbasename).rbegin() < *pair.second.rbegin()))
        {
          dbasename = pair.first;
        }
      }
    }
  }
  else if (timestep <= 0 && this->TimestepValues.empty())
  {
    dbasename = this->DatabaseNames.begin()->first;
  }
  else
  {
    vtkLogF(ERROR, "time stuff is busted!");
    return std::vector<DatabaseHandle>();
  }

  assert(!dbasename.empty());
  const auto fileids = this->GetFileIds(dbasename, piece, npieces);
  std::vector<DatabaseHandle> handles(fileids.size());
  std::transform(fileids.begin(), fileids.end(), handles.begin(),
    [&dbasename](int fileid) { return DatabaseHandle(dbasename, fileid); });
  return handles;
}

//----------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkDataSet>> vtkIOSSReader::vtkInternals::GetDataSets(
  const std::string& blockname, vtkIOSSReader::EntityType vtk_entity_type,
  const DatabaseHandle& handle, int timestep, vtkIOSSReader* self)
{
  // TODO: ideally, this method shouldn't depend on format but entity type.
  switch (this->Format)
  {
    case vtkIOSSUtilities::DatabaseFormatType::CGNS:
      switch (vtk_entity_type)
      {
        case STRUCTUREDBLOCK:
        case SIDESET:
          return this->GetCGNSDataSets(blockname, vtk_entity_type, handle, timestep, self);

        default:
          // not supported for CGNS (AFAIK)
          return {};
      }

    case vtkIOSSUtilities::DatabaseFormatType::EXODUS:
    case vtkIOSSUtilities::DatabaseFormatType::CATALYST:
      switch (vtk_entity_type)
      {
        case STRUCTUREDBLOCK:
          return {};
        default:
          return this->GetExodusDataSets(blockname, vtk_entity_type, handle, timestep, self);
      }

    default:
      vtkLogF(
        ERROR, "Format not setup correctly or unknown format (%d)", static_cast<int>(this->Format));
      return {};
  }
}

//----------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkDataSet>> vtkIOSSReader::vtkInternals::GetExodusDataSets(
  const std::string& blockname, vtkIOSSReader::EntityType vtk_entity_type,
  const DatabaseHandle& handle, int timestep, vtkIOSSReader* self)
{
  const auto ioss_entity_type = vtkIOSSUtilities::GetIOSSEntityType(vtk_entity_type);
  auto region = this->GetRegion(handle.first, handle.second);
  if (!region)
  {
    return {};
  }

  auto group_entity = region->get_entity(blockname, ioss_entity_type);
  if (!group_entity)
  {
    return {};
  }

  vtkNew<vtkUnstructuredGrid> dataset;
  if (!this->GetMesh(dataset, blockname, vtk_entity_type, handle, self->GetRemoveUnusedPoints()))
  {
    return {};
  }

  // let's read arrays.
  auto fieldSelection = self->GetFieldSelection(vtk_entity_type);
  assert(fieldSelection != nullptr);
  this->GetFields(dataset->GetCellData(), fieldSelection, region, group_entity, handle, timestep,
    self->GetReadIds());

  auto nodeFieldSelection = self->GetNodeBlockFieldSelection();
  assert(nodeFieldSelection != nullptr);
  this->GetNodeFields(dataset->GetPointData(), nodeFieldSelection, region, group_entity, handle,
    timestep, self->GetReadIds());

  if (self->GetApplyDisplacements())
  {
    this->ApplyDisplacements(dataset, region, group_entity, handle, timestep);
  }

  if (self->GetGenerateFileId())
  {
    this->GenerateFileId(dataset, group_entity, handle);
  }

  if (self->GetReadIds())
  {
    this->GenerateEntityIdArray(dataset, blockname, vtk_entity_type, handle);
  }

  return { dataset.GetPointer() };
}

//----------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkDataSet>> vtkIOSSReader::vtkInternals::GetCGNSDataSets(
  const std::string& blockname, vtkIOSSReader::EntityType vtk_entity_type,
  const DatabaseHandle& handle, int timestep, vtkIOSSReader* self)
{
  const auto ioss_entity_type = vtkIOSSUtilities::GetIOSSEntityType(vtk_entity_type);
  auto region = this->GetRegion(handle.first, handle.second);
  if (!region)
  {
    return {};
  }

  if (vtk_entity_type == vtkIOSSReader::STRUCTUREDBLOCK)
  {
    auto groups = vtkIOSSUtilities::GetMatchingStructuredBlocks(region, blockname);
    std::vector<vtkSmartPointer<vtkDataSet>> grids;
    for (auto group_entity : groups)
    {
      vtkNew<vtkStructuredGrid> grid;
      if (!this->GetGeometry(grid, group_entity))
      {
        return {};
      }

      auto fieldSelection = self->GetFieldSelection(vtk_entity_type);
      assert(fieldSelection != nullptr);
      this->GetFields(grid->GetCellData(), fieldSelection, region, group_entity, handle, timestep,
        self->GetReadIds());

      // Next, read node fields from nested node-block
      auto nodeFieldSelection = self->GetNodeBlockFieldSelection();
      assert(nodeFieldSelection != nullptr);
      this->GetNodeFields(grid->GetPointData(), nodeFieldSelection, region, group_entity, handle,
        timestep, self->GetReadIds());

      if (self->GetApplyDisplacements())
      {
        this->ApplyDisplacements(grid, region, group_entity, handle, timestep);
      }

      if (self->GetGenerateFileId())
      {
        this->GenerateFileId(grid, group_entity, handle);
      }

      if (self->GetReadIds())
      {
        this->GenerateEntityIdArray(grid, blockname, vtk_entity_type, handle);
      }

      grids.emplace_back(grid.GetPointer());
    }
    return grids;
  }
  else if (vtk_entity_type == vtkIOSSReader::SIDESET)
  {
    std::vector<vtkSmartPointer<vtkDataSet>> result;

    // need to read each side-block.
    auto sideSet = dynamic_cast<Ioss::SideSet*>(region->get_entity(blockname, ioss_entity_type));
    if (!sideSet)
    {
      return {};
    }

    // this is the family name for this side set.
    const auto family = sideSet->name();

    std::map<const Ioss::StructuredBlock*, vtkSmartPointer<vtkDataSet>> fullGridMap;

    // for each side block, find the BC matching the family name and then do extract
    // VOI.
    for (const auto& sideBlock : sideSet->get_side_blocks())
    {
      // for each side block, go to the parent block
      auto parentBlock = dynamic_cast<const Ioss::StructuredBlock*>(sideBlock->parent_block());
      assert(parentBlock != nullptr);
      for (auto& bc : parentBlock->m_boundaryConditions)
      {
        if (bc.m_famName == family)
        {
          // read full grid with fields.
          auto iter = fullGridMap.find(parentBlock);
          if (iter == fullGridMap.end())
          {
            auto grids = this->GetCGNSDataSets(
              parentBlock->name(), vtkIOSSReader::STRUCTUREDBLOCK, handle, timestep, self);
            if (grids.empty())
            {
              continue;
            }
            assert(grids.size() == 1);
            iter = fullGridMap.insert(std::make_pair(parentBlock, grids.front())).first;
          }
          assert(iter != fullGridMap.end() && iter->second != nullptr);

          vtkNew<vtkExtractGrid> extractor;
          extractor->SetInputDataObject(iter->second);

          // extents in bc are starting with 1.
          // so adjust them for VTK
          // clang-format off
          int extents[6] = {
            bc.m_rangeBeg[0] - 1, bc.m_rangeEnd[0]  - 1,
            bc.m_rangeBeg[1] - 1, bc.m_rangeEnd[1]  - 1,
            bc.m_rangeBeg[2] - 1, bc.m_rangeEnd[2]  - 1
          };
          // clang-format on

          extractor->SetVOI(extents);
          extractor->Update();

          auto piece = vtkDataSet::SafeDownCast(extractor->GetOutputDataObject(0));

          vtkNew<vtkStringArray> sideBlockInfo;
          sideBlockInfo->SetName("SideBlock Information");
          sideBlockInfo->SetNumberOfComponents(3);
          sideBlockInfo->SetComponentName(0, "Name");
          sideBlockInfo->SetComponentName(1, "Family");
          sideBlockInfo->SetComponentName(2, "ParentBlock");
          sideBlockInfo->InsertNextValue(sideBlock->name());
          sideBlockInfo->InsertNextValue(family);
          sideBlockInfo->InsertNextValue(parentBlock->name());
          piece->GetFieldData()->AddArray(sideBlockInfo);
          result.emplace_back(piece);
        }
      }
    }

    return result;
  }

  return {};
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetMesh(vtkUnstructuredGrid* dataset,
  const std::string& blockname, vtkIOSSReader::EntityType vtk_entity_type,
  const DatabaseHandle& handle, bool remove_unused_points)
{
  auto ioss_entity_type = vtkIOSSUtilities::GetIOSSEntityType(vtk_entity_type);
  auto region = this->GetRegion(handle);
  auto group_entity = region->get_entity(blockname, ioss_entity_type);
  if (!group_entity)
  {
    return false;
  }

  auto& cache = this->Cache;
  const std::string cacheKey{ "__vtk_mesh__" };
  if (auto cachedDataset = vtkDataSet::SafeDownCast(cache.Find(group_entity, cacheKey)))
  {
    dataset->CopyStructure(cachedDataset);
    return true;
  }

  if (!this->GetTopology(dataset, blockname, vtk_entity_type, handle) ||
    !this->GetGeometry(dataset, "nodeblock_1", handle))
  {
    return false;
  }

  if (remove_unused_points)
  {
    // let's prune unused points.
    vtkNew<vtkRemoveUnusedPoints> pruner;
    pruner->SetOriginalPointIdsArrayName("__vtk_mesh_original_pt_ids__");
    pruner->SetInputDataObject(dataset);
    pruner->Update();

    auto pruned = pruner->GetOutput();
    // cache original pt ids;  this is used in `GetNodeFields`.
    if (auto originalIds = pruned->GetPointData()->GetArray("__vtk_mesh_original_pt_ids__"))
    {
      cache.Insert(group_entity, "__vtk_mesh_original_pt_ids__", originalIds);
      // cache mesh
      dataset->CopyStructure(pruned);
      cache.Insert(group_entity, cacheKey, pruned);
      return true;
    }

    return false;
  }
  else
  {
    vtkNew<vtkUnstructuredGrid> clone;
    clone->CopyStructure(dataset);
    cache.Insert(group_entity, cacheKey, clone);
    return true;
  }
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetMesh(vtkStructuredGrid* grid, const std::string& blockname,
  vtkIOSSReader::EntityType vtk_entity_type, const DatabaseHandle& handle)
{
  vtkLogScopeF(TRACE, "GetMesh(%s)", blockname.c_str());
  assert(
    vtk_entity_type == vtkIOSSReader::STRUCTUREDBLOCK || vtk_entity_type == vtkIOSSReader::SIDESET);

  if (vtk_entity_type == vtkIOSSReader::STRUCTUREDBLOCK)
  {
    auto ioss_entity_type = vtkIOSSUtilities::GetIOSSEntityType(vtk_entity_type);
    auto region = this->GetRegion(handle);
    auto group_entity =
      dynamic_cast<Ioss::StructuredBlock*>(region->get_entity(blockname, ioss_entity_type));
    if (!group_entity)
    {
      return false;
    }

    return this->GetGeometry(grid, group_entity);
  }
  else if (vtk_entity_type == vtkIOSSReader::SIDESET)
  {
    auto ioss_entity_type = vtkIOSSUtilities::GetIOSSEntityType(vtk_entity_type);
    auto region = this->GetRegion(handle);
    auto sideSet = dynamic_cast<Ioss::SideSet*>(region->get_entity(blockname, ioss_entity_type));
    if (!sideSet)
    {
      return false;
    }

    // this is the family name for this side set.
    const auto family = sideSet->name();

    // for each side block, find the BC matching the family name and then do extract
    // VOI.
    for (const auto& sideBlock : sideSet->get_side_blocks())
    {
      // for each side block, go to the parent block
      auto parentBlock = dynamic_cast<const Ioss::StructuredBlock*>(sideBlock->parent_block());
      assert(parentBlock != nullptr);
      for (auto& bc : parentBlock->m_boundaryConditions)
      {
        if (bc.m_famName == family)
        {
          vtkNew<vtkStructuredGrid> fullGrid;
          this->GetGeometry(fullGrid, parentBlock);
          break;
        }
      }
    }

    abort();
  }
  else
  {
    throw std::runtime_error("Unsupported 'GetMesh' call for entity type.");
  }
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GenerateEntityIdArray(vtkDataSet* dataset,
  const std::string& blockname, vtkIOSSReader::EntityType vtk_entity_type,
  const DatabaseHandle& handle)
{
  auto ioss_entity_type = vtkIOSSUtilities::GetIOSSEntityType(vtk_entity_type);
  auto region = this->GetRegion(handle);
  auto group_entity = region->get_entity(blockname, ioss_entity_type);
  if (!group_entity || !group_entity->property_exists("id"))
  {
    return false;
  }

  auto& cache = this->Cache;
  const std::string cacheKey{ "__vtk_entity_id__" };

  if (auto cachedArray = vtkIdTypeArray::SafeDownCast(cache.Find(group_entity, cacheKey)))
  {
    dataset->GetCellData()->AddArray(cachedArray);
  }
  else
  {
    vtkNew<vtkIdTypeArray> objectId;
    objectId->SetNumberOfTuples(dataset->GetNumberOfCells());
    objectId->FillValue(static_cast<vtkIdType>(group_entity->get_property("id").get_int()));
    objectId->SetName("object_id");
    cache.Insert(group_entity, cacheKey, objectId);
    dataset->GetCellData()->AddArray(objectId);
  }

  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetTopology(vtkUnstructuredGrid* grid,
  const std::string& blockname, vtkIOSSReader::EntityType vtk_entity_type,
  const DatabaseHandle& handle)
{
  auto ioss_entity_type = vtkIOSSUtilities::GetIOSSEntityType(vtk_entity_type);
  auto region = this->GetRegion(handle);
  auto group_entity = region->get_entity(blockname, ioss_entity_type);
  if (!group_entity)
  {
    return false;
  }

  vtkLogScopeF(TRACE, "GetTopology (%s)[file=%s]", blockname.c_str(),
    this->GetRawFileName(handle, true).c_str());
  if (ioss_entity_type == Ioss::EntityType::SIDESET)
  {
    // for side set, the topology is stored in nested elements called
    // SideBlocks. Since we split side sets by topologies, each sideblock can be
    // treated as a regular entity block.
    assert(group_entity->get_database()->get_surface_split_type() == Ioss::SPLIT_BY_TOPOLOGIES);
    std::vector<std::pair<int, vtkSmartPointer<vtkCellArray>>> sideblock_cells;
    auto sideSet = static_cast<Ioss::SideSet*>(group_entity);
    vtkIdType numCells = 0, connectivitySize = 0;
    for (auto sideBlock : sideSet->get_side_blocks())
    {
      int cell_type = VTK_EMPTY_CELL;
      auto cellarray = vtkIOSSUtilities::GetConnectivity(sideBlock, cell_type, &this->Cache);
      if (cellarray != nullptr && cell_type != VTK_EMPTY_CELL)
      {
        numCells += cellarray->GetNumberOfCells();
        sideblock_cells.emplace_back(cell_type, cellarray);
      }
    }
    if (sideblock_cells.size() == 1)
    {
      grid->SetCells(sideblock_cells.front().first, sideblock_cells.front().second);
      return true;
    }
    else if (sideblock_cells.size() > 1)
    {
      // this happens when side block has mixed topological elements.
      vtkNew<vtkCellArray> appendedCellArray;
      appendedCellArray->AllocateExact(numCells, connectivitySize);
      vtkNew<vtkUnsignedCharArray> cellTypesArray;
      cellTypesArray->SetNumberOfTuples(numCells);
      auto ptr = cellTypesArray->GetPointer(0);
      for (auto& pair : sideblock_cells)
      {
        appendedCellArray->Append(pair.second);
        ptr =
          std::fill_n(ptr, pair.second->GetNumberOfCells(), static_cast<unsigned char>(pair.first));
      }
      grid->SetCells(cellTypesArray, appendedCellArray);
      return true;
    }
  }
  else
  {
    int cell_type = VTK_EMPTY_CELL;
    auto cellarray = vtkIOSSUtilities::GetConnectivity(group_entity, cell_type, &this->Cache);
    if (cell_type != VTK_EMPTY_CELL && cellarray != nullptr)
    {
      grid->SetCells(cell_type, cellarray);
      return true;
    }
  }
  return false;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetGeometry(
  vtkUnstructuredGrid* grid, const std::string& blockname, const DatabaseHandle& handle)
{
  auto region = this->GetRegion(handle);
  auto group_entity = region->get_entity(blockname, Ioss::EntityType::NODEBLOCK);
  if (!group_entity)
  {
    return false;
  }

  vtkLogScopeF(TRACE, "GetGeometry(%s)[file=%s]", blockname.c_str(),
    this->GetRawFileName(handle, true).c_str());
  auto pts = vtkIOSSUtilities::GetMeshModelCoordinates(group_entity, &this->Cache);
  grid->SetPoints(pts);
  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetGeometry(
  vtkStructuredGrid* grid, const Ioss::StructuredBlock* groupEntity)
{
  auto& sblock = (*groupEntity);

  int extents[6];
  extents[0] = static_cast<int>(sblock.get_property("offset_i").get_int());
  extents[1] = extents[0] + static_cast<int>(sblock.get_property("ni").get_int());
  extents[2] = static_cast<int>(sblock.get_property("offset_j").get_int());
  extents[3] = extents[2] + static_cast<int>(sblock.get_property("nj").get_int());
  extents[4] = static_cast<int>(sblock.get_property("offset_k").get_int());
  extents[5] = extents[4] + static_cast<int>(sblock.get_property("nk").get_int());

  assert(
    sblock.get_property("node_count").get_int() == vtkStructuredData::GetNumberOfPoints(extents));
  assert(
    sblock.get_property("cell_count").get_int() == vtkStructuredData::GetNumberOfCells(extents));

  // set extents on grid.
  grid->SetExtent(extents);

  // now read the points.
  auto points = vtkIOSSUtilities::GetMeshModelCoordinates(&sblock, &this->Cache);
  grid->SetPoints(points);
  assert(points->GetNumberOfPoints() == vtkStructuredData::GetNumberOfPoints(extents));
  return true;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkAbstractArray> vtkIOSSReader::vtkInternals::GetField(
  const std::string& fieldname, Ioss::Region* region, Ioss::GroupingEntity* group_entity,
  const DatabaseHandle& vtkNotUsed(handle), int timestep, vtkIdTypeArray* ids_to_extract,
  const std::string& cache_key_suffix)
{
  const auto get_field = [&fieldname, &region, &timestep, this](
                           Ioss::GroupingEntity* entity) -> vtkSmartPointer<vtkAbstractArray> {
    if (!entity->field_exists(fieldname))
    {
      return nullptr;
    }

    if (!vtkIOSSUtilities::IsFieldTransient(entity, fieldname))
    {
      // non-time dependent field.
      return vtkIOSSUtilities::GetData(entity, fieldname, /*transform=*/nullptr, &this->Cache);
    }

    // determine state for transient data.
    const auto max = region->get_max_time();
    if (max.first <= 0)
    {
      // see paraview/paraview#20658 for why this is needed.
      return nullptr;
    }

    const auto min = region->get_min_time();
    int state = -1;
    for (int cc = min.first; cc <= max.first; ++cc)
    {
      if (region->get_state_time(cc) == this->TimestepValues[timestep])
      {
        state = cc;
        break;
      }
    }
    if (state == -1)
    {
      throw std::runtime_error("Invalid timestep chosen: " + std::to_string(timestep));
    }
    region->begin_state(state);
    try
    {
      const std::string key = "__vtk_transient_" + fieldname + "_" + std::to_string(state) + "__";
      auto f =
        vtkIOSSUtilities::GetData(entity, fieldname, /*transform=*/nullptr, &this->Cache, key);
      region->end_state(state);
      return f;
    }
    catch (...)
    {
      region->end_state(state);
      std::rethrow_exception(std::current_exception());
    }
  };

  const auto get_field_for_entity = [&]() {
    if (group_entity->type() == Ioss::EntityType::SIDESET)
    {
      // sidesets need to be handled specially. For sidesets, the fields are
      // available on nested sideblocks.
      std::vector<vtkSmartPointer<vtkAbstractArray>> arrays;
      auto sideSet = static_cast<Ioss::SideSet*>(group_entity);
      for (auto sideBlock : sideSet->get_side_blocks())
      {
        if (auto array = get_field(sideBlock))
        {
          arrays.push_back(array);
        }
      }
      return ::JoinArrays(arrays);
    }
    else
    {
      return get_field(group_entity);
    }
  };

  auto& cache = this->Cache;
  const std::string cacheKey =
    (vtkIOSSUtilities::IsFieldTransient(group_entity, fieldname)
        ? "__vtk_transientfield_" + fieldname + std::to_string(timestep) + "__"
        : "__vtk_field_" + fieldname + "__") +
    cache_key_suffix;
  if (auto cached = vtkAbstractArray::SafeDownCast(cache.Find(group_entity, cacheKey)))
  {
    return cached;
  }

  auto full_field = get_field_for_entity();
  if (full_field != nullptr && ids_to_extract != nullptr)
  {
    // subset the field.
    vtkNew<vtkIdList> list;
    // this is a shallow copy.
    list->SetArray(ids_to_extract->GetPointer(0), ids_to_extract->GetNumberOfTuples());

    vtkSmartPointer<vtkAbstractArray> clone;
    clone.TakeReference(full_field->NewInstance());
    clone->SetName(full_field->GetName());
    clone->SetNumberOfComponents(full_field->GetNumberOfComponents());
    clone->SetNumberOfTuples(list->GetNumberOfIds());
    full_field->GetTuples(list, clone);

    // get back the data pointer from the idlist
    list->Release();

    // convert field if needed for VTK e.g. ids have to be `vtkIdTypeArray`.
    clone = this->ConvertFieldForVTK(clone);

    cache.Insert(group_entity, cacheKey, clone);
    return clone;
  }
  else
  {
    // convert field if needed for VTK e.g. ids have to be `vtkIdTypeArray`.
    full_field = this->ConvertFieldForVTK(full_field);

    cache.Insert(group_entity, cacheKey, full_field);
    return full_field;
  }
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetFields(vtkDataSetAttributes* dsa,
  vtkDataArraySelection* selection, Ioss::Region* region, Ioss::GroupingEntity* group_entity,
  const DatabaseHandle& handle, int timestep, bool read_ioss_ids,
  vtkIdTypeArray* ids_to_extract /*=nullptr*/,
  const std::string& cache_key_suffix /*= std::string()*/)
{
  std::vector<std::string> fieldnames;
  std::string globalIdsFieldName;
  if (read_ioss_ids)
  {
    switch (group_entity->type())
    {
      case Ioss::EntityType::NODEBLOCK:
      case Ioss::EntityType::EDGEBLOCK:
      case Ioss::EntityType::FACEBLOCK:
      case Ioss::EntityType::ELEMENTBLOCK:
      case Ioss::EntityType::NODESET:
        fieldnames.emplace_back("ids");
        globalIdsFieldName = "ids";
        break;

      case Ioss::EntityType::STRUCTUREDBLOCK:
        if (vtkPointData::SafeDownCast(dsa))
        {
          fieldnames.emplace_back("cell_node_ids");
        }
        else
        {
          fieldnames.emplace_back("cell_ids");
        }
        // note: unlike for Exodus, there ids are not unique
        // across blocks and hence are not flagged as global ids.
        break;

      case Ioss::EntityType::EDGESET:
      case Ioss::EntityType::FACESET:
      case Ioss::EntityType::ELEMENTSET:
      case Ioss::EntityType::SIDESET:
        fieldnames.emplace_back("element_side");
        break;

      default:
        break;
    }
  }
  for (int cc = 0; selection != nullptr && cc < selection->GetNumberOfArrays(); ++cc)
  {
    if (selection->GetArraySetting(cc))
    {
      fieldnames.emplace_back(selection->GetArrayName(cc));
    }
  }
  for (const auto& fieldname : fieldnames)
  {
    if (auto array = this->GetField(
          fieldname, region, group_entity, handle, timestep, ids_to_extract, cache_key_suffix))
    {
      if (fieldname == globalIdsFieldName)
      {
        dsa->SetGlobalIds(vtkDataArray::SafeDownCast(array));
      }
      else
      {
        dsa->AddArray(array);
      }
    }
  }

  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetNodeFields(vtkDataSetAttributes* dsa,
  vtkDataArraySelection* selection, Ioss::Region* region, Ioss::GroupingEntity* group_entity,
  const DatabaseHandle& handle, int timestep, bool read_ioss_ids)
{
  if (group_entity->type() == Ioss::EntityType::STRUCTUREDBLOCK)
  {
    // CGNS
    // node fields are stored under nested node block. So use that.
    auto sb = dynamic_cast<Ioss::StructuredBlock*>(group_entity);
    auto& nodeBlock = sb->get_node_block();
    if (!this->GetFields(
          dsa, selection, region, &nodeBlock, handle, timestep, /*read_ioss_ids=*/false))
    {
      return false;
    }

    // for STRUCTUREDBLOCK, the node ids are read from the SB itself, and not
    // the nested nodeBlock.
    return read_ioss_ids
      ? this->GetFields(dsa, nullptr, region, sb, handle, timestep, /*read_ioss_ids=*/true)
      : true;
  }
  else
  {
    // Exodus
    const auto blockname = group_entity->name();
    auto& cache = this->Cache;
    auto vtk_raw_ids_array =
      vtkIdTypeArray::SafeDownCast(cache.Find(group_entity, "__vtk_mesh_original_pt_ids__"));
    const std::string cache_key_suffix = vtk_raw_ids_array != nullptr ? blockname : std::string();

    auto nodeblock = region->get_entity("nodeblock_1", Ioss::EntityType::NODEBLOCK);
    return this->GetFields(dsa, selection, region, nodeblock, handle, timestep, read_ioss_ids,
      vtk_raw_ids_array, cache_key_suffix);
  }
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GenerateFileId(
  vtkDataSet* grid, Ioss::GroupingEntity* group_entity, const DatabaseHandle& handle)
{
  if (!group_entity)
  {
    return false;
  }

  auto& cache = this->Cache;
  if (auto file_ids = vtkDataArray::SafeDownCast(cache.Find(group_entity, "__vtk_file_ids__")))
  {
    assert(grid->GetNumberOfCells() == file_ids->GetNumberOfTuples());
    grid->GetCellData()->AddArray(file_ids);
    return true;
  }

  vtkNew<vtkIntArray> file_ids;
  file_ids->SetName("file_id");
  file_ids->SetNumberOfTuples(grid->GetNumberOfCells());

  int fileId = handle.second;

  // from index get original file rank number, if possible and use that.
  try
  {
    const auto& dbaseInfo = this->DatabaseNames.at(handle.first);
    if (dbaseInfo.ProcessCount != 0)
    {
      assert(fileId >= 0 && fileId < static_cast<decltype(fileId)>(dbaseInfo.Ranks.size()));
      fileId = *std::next(dbaseInfo.Ranks.begin(), fileId);
    }
  }
  catch (std::out_of_range&)
  {
  }

  std::fill(file_ids->GetPointer(0), file_ids->GetPointer(0) + grid->GetNumberOfCells(), fileId);
  cache.Insert(group_entity, "__vtk_file_ids__", file_ids.GetPointer());
  grid->GetCellData()->AddArray(file_ids);
  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::ApplyDisplacements(vtkPointSet* grid, Ioss::Region* region,
  Ioss::GroupingEntity* group_entity, const DatabaseHandle& handle, int timestep)
{
  if (!group_entity)
  {
    return false;
  }

  auto& cache = this->Cache;
  const auto xformPtsCacheKey = "__vtk_xformed_pts_" + std::to_string(timestep);
  if (auto xformedPts = vtkPoints::SafeDownCast(cache.Find(group_entity, xformPtsCacheKey)))
  {
    assert(xformedPts->GetNumberOfPoints() == grid->GetNumberOfPoints());
    grid->SetPoints(xformedPts);
    return true;
  }

  vtkSmartPointer<vtkDataArray> array;

  if (group_entity->type() == Ioss::EntityType::STRUCTUREDBLOCK)
  {
    // CGNS
    // node fields are stored under nested node block. So use that.
    auto sb = dynamic_cast<Ioss::StructuredBlock*>(group_entity);
    auto& nodeBlock = sb->get_node_block();
    auto displ_array_name = vtkIOSSUtilities::GetDisplacementFieldName(&nodeBlock);
    if (displ_array_name.empty())
    {
      return false;
    }

    array = vtkDataArray::SafeDownCast(
      this->GetField(displ_array_name, region, &nodeBlock, handle, timestep));
  }
  else
  {
    // EXODUS
    // node fields are stored in global node-block from which we need to subset based on the "ids"
    // for those current block.
    auto nodeBlock = region->get_entity("nodeblock_1", Ioss::EntityType::NODEBLOCK);
    auto displ_array_name = vtkIOSSUtilities::GetDisplacementFieldName(nodeBlock);
    if (displ_array_name.empty())
    {
      return false;
    }

    auto vtk_raw_ids_array =
      vtkIdTypeArray::SafeDownCast(cache.Find(group_entity, "__vtk_mesh_original_pt_ids__"));
    const std::string cache_key_suffix =
      vtk_raw_ids_array != nullptr ? group_entity->name() : std::string();
    array = vtkDataArray::SafeDownCast(this->GetField(
      displ_array_name, region, nodeBlock, handle, timestep, vtk_raw_ids_array, cache_key_suffix));
  }

  if (array)
  {
    // NOTE: array maybe 2 component for 2d dataset; but our points are always 3D.
    auto pts = grid->GetPoints();
    auto numPts = pts->GetNumberOfPoints();

    assert(array->GetNumberOfTuples() == numPts && array->GetNumberOfComponents() <= 3);

    vtkNew<vtkPoints> xformedPts;
    xformedPts->SetDataType(pts->GetDataType());
    xformedPts->SetNumberOfPoints(pts->GetNumberOfPoints());
    vtkVector3d coords{ 0.0 }, displ{ 0.0 };
    for (vtkIdType cc = 0; cc < numPts; ++cc)
    {
      pts->GetPoint(cc, coords.GetData());
      array->GetTuple(cc, displ.GetData());
      xformedPts->SetPoint(cc, (coords + displ).GetData());
    }

    grid->SetPoints(xformedPts);
    cache.Insert(group_entity, xformPtsCacheKey, xformedPts);
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetQAAndInformationRecords(
  vtkFieldData* fd, const DatabaseHandle& handle)
{
  auto region = this->GetRegion(handle);
  if (!region)
  {
    return false;
  }

  const auto& qa = region->get_qa_records();
  vtkNew<vtkStringArray> qa_records;
  qa_records->SetName("QA Records");
  qa_records->SetNumberOfComponents(4);
  qa_records->Allocate(static_cast<vtkIdType>(qa.size()));
  qa_records->SetComponentName(0, "Code Name");
  qa_records->SetComponentName(1, "QA Descriptor");
  qa_records->SetComponentName(2, "Date");
  qa_records->SetComponentName(3, "Time");
  for (auto& name : qa)
  {
    qa_records->InsertNextValue(name);
  }

  const auto& info = region->get_information_records();
  vtkNew<vtkStringArray> info_records;
  info_records->SetName("Information Records");
  info_records->SetNumberOfComponents(1);
  info_records->Allocate(static_cast<vtkIdType>(info.size()));
  for (auto& n : info)
  {
    info_records->InsertNextValue(n);
  }

  fd->AddArray(info_records);
  fd->AddArray(qa_records);
  return true;
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::vtkInternals::GetGlobalFields(
  vtkFieldData* fd, const DatabaseHandle& handle, int timestep)
{
  auto region = this->GetRegion(handle);
  if (!region)
  {
    return false;
  }

  Ioss::NameList fieldNames;
  region->field_describe(&fieldNames);
  for (const auto& name : fieldNames)
  {
    switch (region->get_fieldref(name).get_role())
    {
      case Ioss::Field::ATTRIBUTE:
      case Ioss::Field::REDUCTION:
        if (auto array = this->GetField(name, region, region, handle, timestep))
        {
          fd->AddArray(array);
        }
        break;
      default:
        break;
    }
  }
  return true;
}

//============================================================================
vtkStandardNewMacro(vtkIOSSReader);
vtkCxxSetObjectMacro(vtkIOSSReader, Controller, vtkMultiProcessController);
vtkInformationKeyMacro(vtkIOSSReader, ENTITY_TYPE, Integer);
//----------------------------------------------------------------------------
vtkIOSSReader::vtkIOSSReader()
  : Controller(nullptr)
  , GenerateFileId(false)
  , ScanForRelatedFiles(true)
  , ReadIds(true)
  , RemoveUnusedPoints(true)
  , ApplyDisplacements(true)
  , ReadGlobalFields(true)
  , ReadQAAndInformationRecords(true)
  , DatabaseTypeOverride(nullptr)
  , AssemblyTag(0)
  , FileRange{ 0, -1 }
  , FileStride{ 1 }
  , Internals(new vtkIOSSReader::vtkInternals(this))
{
  this->SetController(vtkMultiProcessController::GetGlobalController());
}

//----------------------------------------------------------------------------
vtkIOSSReader::~vtkIOSSReader()
{
  this->SetDatabaseTypeOverride(nullptr);
  this->SetController(nullptr);
  delete this->Internals;
}

//----------------------------------------------------------------------------
int vtkIOSSReader::FillOutputPortInformation(int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPartitionedDataSetCollection");
  return 1;
}

//----------------------------------------------------------------------------
void vtkIOSSReader::SetScanForRelatedFiles(bool val)
{
  if (this->ScanForRelatedFiles != val)
  {
    this->ScanForRelatedFiles = val;
    auto& internals = (*this->Internals);
    internals.FileNamesMTime.Modified();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::SetFileName(const char* fname)
{
  auto& internals = (*this->Internals);
  if (fname == nullptr)
  {
    if (!internals.FileNames.empty())
    {
      internals.FileNames.clear();
      internals.FileNamesMTime.Modified();
      this->Modified();
    }
    return;
  }

  if (internals.FileNames.size() == 1 && *internals.FileNames.begin() == fname)
  {
    return;
  }

  internals.FileNames.clear();
  internals.FileNames.insert(fname);
  internals.FileNamesMTime.Modified();
  this->Modified();
}

//----------------------------------------------------------------------------
void vtkIOSSReader::AddFileName(const char* fname)
{
  auto& internals = (*this->Internals);
  if (fname != nullptr && !internals.FileNames.insert(fname).second)
  {
    internals.FileNamesMTime.Modified();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::ClearFileNames()
{
  auto& internals = (*this->Internals);
  if (!internals.FileNames.empty())
  {
    internals.FileNames.clear();
    internals.FileNamesMTime.Modified();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
const char* vtkIOSSReader::GetFileName(int index) const
{
  auto& internals = (*this->Internals);
  if (static_cast<int>(internals.FileNames.size()) > index)
  {
    auto iter = std::next(internals.FileNames.begin(), index);
    return iter->c_str();
  }
  return nullptr;
}

//----------------------------------------------------------------------------
int vtkIOSSReader::GetNumberOfFileNames() const
{
  auto& internals = (*this->Internals);
  return static_cast<int>(internals.FileNames.size());
}

//----------------------------------------------------------------------------
int vtkIOSSReader::ReadMetaData(vtkInformation* metadata)
{
  vtkLogScopeF(TRACE, "ReadMetaData");
  auto& internals = (*this->Internals);
  if (!internals.UpdateDatabaseNames(this))
  {
    return 0;
  }

  // read time information and generate that.
  if (!internals.UpdateTimeInformation(this))
  {
    return 0;
  }
  else
  {
    // add timesteps to metadata
    const auto& timesteps = internals.GetTimeSteps();
    if (!timesteps.empty())
    {
      metadata->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps[0],
        static_cast<int>(timesteps.size()));
      double time_range[2] = { timesteps.front(), timesteps.back() };
      metadata->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), time_range, 2);
    }
    else
    {
      metadata->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
      metadata->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
    }
  }

  // read field/entity selection meta-data. i.e. update vtkDataArraySelection
  // instances for all available entity-blocks, entity-sets, and their
  // corresponding data arrays.
  if (!internals.UpdateEntityAndFieldSelections(this))
  {
    return 0;
  }

  // read assembly information.
  if (!internals.UpdateAssembly(this, &this->AssemblyTag))
  {
    return 0;
  }

  metadata->Set(vtkAlgorithm::CAN_HANDLE_PIECE_REQUEST(), 1);
  return 1;
}

//----------------------------------------------------------------------------
int vtkIOSSReader::ReadMesh(
  int piece, int npieces, int vtkNotUsed(nghosts), int timestep, vtkDataObject* output)
{
  auto& internals = (*this->Internals);

  if (!internals.UpdateDatabaseNames(this))
  {
    // this should not be necessary. ReadMetaData returns false when
    // `UpdateDatabaseNames` fails. At which point vtkReaderAlgorithm should
    // never call `RequestData` leading to a call to this method. However, it
    // does, for some reason. Hence adding this check here.
    // ref: paraview/paraview#19951.
    return 0;
  }

  // This is the first method that gets called when generating data.
  // Reset internal cache counters so we can flush fields not accessed.
  internals.ResetCacheAccessCounts();

  auto collection = vtkPartitionedDataSetCollection::SafeDownCast(output);

  // setup output based on the block/set selections (and those available in the
  // database).
  if (!internals.GenerateOutput(collection, this))
  {
    vtkErrorMacro("Failed to generate output.");
    return 0;
  }

  std::set<unsigned int> selectedAssemblyIndices;
  if (!internals.Selectors.empty() && internals.GetAssembly() != nullptr)
  {
    std::vector<std::string> selectors(internals.Selectors.size());
    std::copy(internals.Selectors.begin(), internals.Selectors.end(), selectors.begin());
    auto assembly = internals.GetAssembly();
    auto nodes = assembly->SelectNodes(selectors);
    auto dsindices = assembly->GetDataSetIndices(nodes);
    selectedAssemblyIndices.insert(dsindices.begin(), dsindices.end());
  }

  // dbaseHandles are handles for individual files this instance will to read to
  // satisfy the request. Can be >= 0.
  const auto dbaseHandles = internals.GetDatabaseHandles(piece, npieces, timestep);
  for (unsigned int pdsIdx = 0; pdsIdx < collection->GetNumberOfPartitionedDataSets(); ++pdsIdx)
  {
    const std::string blockname(collection->GetMetaData(pdsIdx)->Get(vtkCompositeDataSet::NAME()));
    const auto vtk_entity_type =
      static_cast<vtkIOSSReader::EntityType>(collection->GetMetaData(pdsIdx)->Get(ENTITY_TYPE()));

    auto selection = this->GetEntitySelection(vtk_entity_type);
    if (!selection->ArrayIsEnabled(blockname.c_str()) &&
      selectedAssemblyIndices.find(pdsIdx) == selectedAssemblyIndices.end())
    {
      // skip disabled blocks.
      continue;
    }

    auto pds = collection->GetPartitionedDataSet(pdsIdx);
    assert(pds != nullptr);
    for (unsigned int cc = 0; cc < static_cast<unsigned int>(dbaseHandles.size()); ++cc)
    {
      const auto& handle = dbaseHandles[cc];
      try
      {
        auto datasets = internals.GetDataSets(blockname, vtk_entity_type, handle, timestep, this);
        for (auto& ds : datasets)
        {
          pds->SetPartition(pds->GetNumberOfPartitions(), ds);
        }
      }
      catch (const std::runtime_error& e)
      {
        vtkLogF(ERROR,
          "Error reading entity block (or set) named '%s' from '%s'; skipping. Details: %s",
          blockname.c_str(), internals.GetRawFileName(handle).c_str(), e.what());
      }

      internals.ReleaseHandles();
    }
  }

  // Read global data. Since this should be same on all ranks, we only read on
  // root node and broadcast it to all. This helps us easily handle the case
  // where the number of reading-ranks is more than writing-ranks.
  auto controller = this->GetController();
  const auto rank = controller ? controller->GetLocalProcessId() : 0;
  const auto numRanks = controller ? controller->GetNumberOfProcesses() : 1;
  if (!dbaseHandles.empty() && rank == 0)
  {
    // Read global data. Since global data is expected to be identical on all
    // files in a partitioned collection, we can read it from the first
    // dbaseHandle alone.
    if (this->ReadGlobalFields)
    {
      internals.GetGlobalFields(collection->GetFieldData(), dbaseHandles[0], timestep);
    }

    if (this->ReadQAAndInformationRecords)
    {
      internals.GetQAAndInformationRecords(collection->GetFieldData(), dbaseHandles[0]);
    }

    // Handle assemblies.
    internals.ReadAssemblies(collection, dbaseHandles[0]);
  }

  if (numRanks > 1)
  {
    vtkNew<vtkUnstructuredGrid> temp;
    vtkMultiProcessStream stream;
    if (rank == 0)
    {
      temp->GetFieldData()->ShallowCopy(collection->GetFieldData());
      stream << collection->GetDataAssembly()->SerializeToXML(vtkIndent());
    }
    controller->Broadcast(temp, 0);
    controller->Broadcast(stream, 0);
    if (rank > 0)
    {
      collection->GetFieldData()->ShallowCopy(temp->GetFieldData());

      std::string xml;
      stream >> xml;
      collection->GetDataAssembly()->InitializeFromXML(xml.c_str());
    }
  }

  internals.ClearCacheUnused();
  return 1;
}

//----------------------------------------------------------------------------
vtkDataArraySelection* vtkIOSSReader::GetEntitySelection(int type)
{
  if (type < 0 || type >= NUMBER_OF_ENTITY_TYPES)
  {
    vtkErrorMacro("Invalid type '" << type
                                   << "'. Supported values are "
                                      "vtkIOSSReader::NODEBLOCK (0), ... vtkIOSSReader::SIDESET ("
                                   << vtkIOSSReader::SIDESET << ").");
    return nullptr;
  }
  return this->EntitySelection[type];
}

//----------------------------------------------------------------------------
vtkDataArraySelection* vtkIOSSReader::GetFieldSelection(int type)
{
  if (type < 0 || type >= NUMBER_OF_ENTITY_TYPES)
  {
    vtkErrorMacro("Invalid type '" << type
                                   << "'. Supported values are "
                                      "vtkIOSSReader::NODEBLOCK (0), ... vtkIOSSReader::SIDESET ("
                                   << vtkIOSSReader::SIDESET << ").");
    return nullptr;
  }
  return this->EntityFieldSelection[type];
}

//----------------------------------------------------------------------------
vtkMTimeType vtkIOSSReader::GetMTime()
{
  auto mtime = this->Superclass::GetMTime();
  for (int cc = ENTITY_START; cc < ENTITY_END; ++cc)
  {
    mtime = std::max(mtime, this->EntitySelection[cc]->GetMTime());
    mtime = std::max(mtime, this->EntityFieldSelection[cc]->GetMTime());
  }
  return mtime;
}

//----------------------------------------------------------------------------
void vtkIOSSReader::RemoveAllEntitySelections()
{
  for (int cc = ENTITY_START; cc < ENTITY_END; ++cc)
  {
    this->GetEntitySelection(cc)->RemoveAllArrays();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::RemoveAllFieldSelections()
{
  for (int cc = ENTITY_START; cc < ENTITY_END; ++cc)
  {
    this->GetFieldSelection(cc)->RemoveAllArrays();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::SetRemoveUnusedPoints(bool val)
{
  if (this->RemoveUnusedPoints != val)
  {
    // clear cache to ensure we read appropriate points/point data.
    this->Internals->ClearCache();
    this->RemoveUnusedPoints = val;
    this->Modified();
  }
}

//----------------------------------------------------------------------------
const char* vtkIOSSReader::GetDataAssemblyNodeNameForEntityType(int type)
{
  switch (type)
  {
    case NODEBLOCK:
      return "node_blocks";
    case EDGEBLOCK:
      return "edge_blocks";
    case FACEBLOCK:
      return "face_blocks";
    case ELEMENTBLOCK:
      return "element_blocks";
    case STRUCTUREDBLOCK:
      return "structured_blocks";
    case NODESET:
      return "node_sets";
    case EDGESET:
      return "edge_sets";
    case FACESET:
      return "face_sets";
    case ELEMENTSET:
      return "element_sets";
    case SIDESET:
      return "side_sets";
    default:
      vtkLogF(ERROR, "Invalid type '%d'", type);
      return nullptr;
  }
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::DoTestFilePatternMatching()
{
  return vtkIOSSFilesScanner::DoTestFilePatternMatching();
}

//----------------------------------------------------------------------------
vtkTypeBool vtkIOSSReader::ProcessRequest(
  vtkInformation* request, vtkInformationVector** inInfo, vtkInformationVector* outInfo)
{
  const auto status = this->Superclass::ProcessRequest(request, inInfo, outInfo);

  auto& internals = (*this->Internals);
  internals.ReleaseHandles();
  return status;
}

//----------------------------------------------------------------------------
template <typename T>
bool updateProperty(Ioss::PropertyManager& pm, const std::string& name, const T& value,
  Ioss::Property::BasicType type, T (Ioss::Property::*getter)() const)
{
  if (!pm.exists(name) || !pm.get(name).is_valid() || pm.get(name).get_type() != type ||
    (pm.get(name).*getter)() != value)
  {
    pm.add(Ioss::Property(name, value));
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------
void vtkIOSSReader::AddProperty(const char* name, int value)
{
  auto& internals = (*this->Internals);
  auto& pm = internals.DatabaseProperties;
  if (updateProperty<int64_t>(pm, name, value, Ioss::Property::INTEGER, &Ioss::Property::get_int))
  {
    internals.Reset();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::AddProperty(const char* name, double value)
{
  auto& internals = (*this->Internals);
  auto& pm = internals.DatabaseProperties;
  if (updateProperty<double>(pm, name, value, Ioss::Property::REAL, &Ioss::Property::get_real))
  {
    internals.Reset();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::AddProperty(const char* name, void* value)
{
  auto& internals = (*this->Internals);
  auto& pm = internals.DatabaseProperties;
  if (updateProperty<void*>(pm, name, value, Ioss::Property::POINTER, &Ioss::Property::get_pointer))
  {
    internals.Reset();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::AddProperty(const char* name, const char* value)
{
  auto& internals = (*this->Internals);
  auto& pm = internals.DatabaseProperties;
  if (updateProperty<std::string>(
        pm, name, value, Ioss::Property::STRING, &Ioss::Property::get_string))
  {
    internals.Reset();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::RemoveProperty(const char* name)
{
  auto& internals = (*this->Internals);
  auto& pm = internals.DatabaseProperties;
  if (pm.exists(name))
  {
    pm.erase(name);
    internals.Reset();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::ClearProperties()
{
  auto& internals = (*this->Internals);
  auto& pm = internals.DatabaseProperties;
  if (pm.count() > 0)
  {
    Ioss::NameList names;
    pm.describe(&names);
    for (const auto& name : names)
    {
      pm.erase(name);
    }
    internals.Reset();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
vtkDataAssembly* vtkIOSSReader::GetAssembly()
{
  auto& internals = (*this->Internals);
  return internals.GetAssembly();
}

//----------------------------------------------------------------------------
bool vtkIOSSReader::AddSelector(const char* selector)
{
  auto& internals = (*this->Internals);
  if (selector != nullptr && internals.Selectors.insert(selector).second)
  {
    this->Modified();
    return true;
  }

  return false;
}

//----------------------------------------------------------------------------
void vtkIOSSReader::ClearSelectors()
{
  auto& internals = (*this->Internals);
  if (!internals.Selectors.empty())
  {
    internals.Selectors.clear();
    this->Modified();
  }
}

//----------------------------------------------------------------------------
void vtkIOSSReader::SetSelector(const char* selector)
{
  this->ClearSelectors();
  this->AddSelector(selector);
}

//----------------------------------------------------------------------------
int vtkIOSSReader::GetNumberOfSelectors() const
{
  auto& internals = (*this->Internals);
  return static_cast<int>(internals.Selectors.size());
}

//----------------------------------------------------------------------------
const char* vtkIOSSReader::GetSelector(int index) const
{
  auto& internals = (*this->Internals);
  if (index >= 0 && index < this->GetNumberOfSelectors())
  {
    auto iter = std::next(internals.Selectors.begin(), index);
    return iter->c_str();
  }
  return nullptr;
}

//----------------------------------------------------------------------------
void vtkIOSSReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "GenerateFileId: " << this->GenerateFileId << endl;
  os << indent << "ScanForRelatedFiles: " << this->ScanForRelatedFiles << endl;
  os << indent << "FileRange: " << this->FileRange[0] << ", " << this->FileRange[1] << endl;
  os << indent << "FileStride: " << this->FileStride << endl;
  os << indent << "ReadIds: " << this->ReadIds << endl;
  os << indent << "RemoveUnusedPoints: " << this->RemoveUnusedPoints << endl;
  os << indent << "ApplyDisplacements: " << this->ApplyDisplacements << endl;
  os << indent << "ReadGlobalFields: " << this->ReadGlobalFields << endl;
  os << indent << "ReadQAAndInformationRecords: " << this->ReadQAAndInformationRecords << endl;
  os << indent << "DatabaseTypeOverride: " << this->DatabaseTypeOverride << endl;

  os << indent << "NodeBlockSelection: " << endl;
  this->GetNodeBlockSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "EdgeBlockSelection: " << endl;
  this->GetEdgeBlockSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "FaceBlockSelection: " << endl;
  this->GetFaceBlockSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "ElementBlockSelection: " << endl;
  this->GetElementBlockSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "StructuredBlockSelection: " << endl;
  this->GetStructuredBlockSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "NodeSetSelection: " << endl;
  this->GetNodeSetSelection()->PrintSelf(os, indent.GetNextIndent());

  os << indent << "NodeBlockFieldSelection: " << endl;
  this->GetNodeBlockFieldSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "EdgeBlockFieldSelection: " << endl;
  this->GetEdgeBlockFieldSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "FaceBlockFieldSelection: " << endl;
  this->GetFaceBlockFieldSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "ElementBlockFieldSelection: " << endl;
  this->GetElementBlockFieldSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "StructuredBlockFieldSelection: " << endl;
  this->GetStructuredBlockFieldSelection()->PrintSelf(os, indent.GetNextIndent());
  os << indent << "NodeSetFieldSelection: " << endl;
  this->GetNodeSetFieldSelection()->PrintSelf(os, indent.GetNextIndent());
}
