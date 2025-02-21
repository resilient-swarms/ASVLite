/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkIOSSReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*----------------------------------------------------------------------------
 Copyright (c) Sandia Corporation
 See Copyright.txt or http://www.paraview.org/HTML/Copyright.html for details.
----------------------------------------------------------------------------*/

/**
 * @class vtkIOSSReader
 * @brief Reader for IOSS (Sierra IO System)
 *
 * vtkIOSSReader is reader that uses the IOSS (Sierra IO System) library to
 * read files. Currently, this reader support Exodus and CGNS file formats. IOSS
 * imposes certain restrictions on these file formats and hence it may not be
 * possible to open every Exodus or CGNS file using this reader. This is
 * true especially for CGNS, more so than Exodus. In that case `vtkCGNSReader`
 * may be more appropriate.
 *
 * @section SpecifyingFiles Specifying Files
 *
 * One can select a single file to read using `vtkIOSSReader::SetFileName`.
 * With IOSS, however, it is not uncommon to have a collection of files named
 * using standard patterns (described in Section @ref IossNamingConventions).
 * To support this use-case, the reader automatically scans for additionally
 * files internally. To disable this behaviour, call
 * `vtkIOSSReader::ScanForRelatedFilesOff`.
 *
 * Alternatively, the list of files to be read can be explicitly specified using
 * `vtkIOSSReader::AddFileName`. Then too, if `ScanForRelatedFiles` is `true`,
 * the reader will search for related files for each of the files specified.
 *
 * Additionally, `FileRange` and `FileStride` may be used to limit to reading a
 * subset of files.
 *
 * @section SelectingBlocksSets Selecting blocks and sets to read
 *
 * An IOSS file comprises of blocks and sets of various types. These are
 * described by the enum `vtkIOSSReader::EntityType`.
 *
 * `vtkIOSSReader::GetEntitySelection` returns a `vtkDataArraySelection`
 * instance for each of the entity types. This `vtkDataArraySelection` can be
 * used to query the names for available blocks or sets and also select which
 * ones to read.
 *
 * Typical usage is as follows:
 *
 * @code{.cpp}
 *
 * vtkNew<vtkIOSSReader> reader;
 * reader->SetFileName(...);
 * reader->UpdateInformation();
 * reader->GetElementBlockSelection()->EnableArray("Block0");
 * reader->GetEntitySelection(vtkIOSSReader::SIDESET)->DisableAllArrays();
 * @endcode
 *
 * By default, all blocks are enabled, while all sets are disabled.
 *
 * In additional to selecting blocks and sets by name, if the file defines assemblies
 * that organize these blocks and sets, then one can use selector expressions
 * to enable blocks/sets as defined in the assemblies.
 *
 * A block (or set) is treated as enabled if it is either explicitly enabled using the
 * block selection or implicitly enabled due to a selector specified on over the assemblies.
 *
 * Typical usage to select blocks by assembly alone is as follows:
 *
 * @code{.cpp}
 * vtkNew<vtkIOSSReader> reader;
 * reader->SetFileName(...);
 * reader->UpdateInformation();
 * reader->GetElementBlockSelection()->DisableAllArrays();
 * ...
 * reader->AddSelector("//Low");
 * reader->AddSelector("//High");
 * @endcode
 *
 * @section SelectingArrays Selecting arrays to read
 *
 * Similar to the block and set selection, arrays (or fields as IOSS refers to
 * them) to read from each of the blocks or sets can be specified using the
 * `vtkDataArraySelection` instance returned using
 * `vtkIOSSReader::GetFieldSelection` (or one of its convenience variants).
 *
 * By default all arrays are enabled.
 *
 * @section IossNamingConventions IOSS Naming Conventions
 *
 * An IOSS complete dataset is referred to as a database. There can be multiple
 * multiple timesteps in a single database. A single database may
 * split among multiple files. When a database is split among multiple files,
 * this is strictly spatial partitioning with each file storing part of the data
 * for a specific partition. In this case, the files are named with suffix
 * `.{NP}.{RANK}` where `{NP}` is the total number of partitions  and `{RANK}`
 * is the partition number. For example, if database named `can.e` is split among four
 * files representing 4 partitions, it will be named as follows:
 *
 * @verbatim
 *  can.e.4.0
 *  can.e.4.1
 *  can.e.4.2
 *  can.e.4.3
 * @endverbatim
 *
 * In this example, the database name is `can.e` while the `.4.[0-4]` suffix
 * provides the partition information.
 *
 * Note, the database need not be split into multiple files. Thus, a writer may
 * generate a single `can.e` file that has all the timesteps and paritions and
 * still provide all information available when the database is split among
 * multiple files.
 *
 * Multiple databases (with each stored in a single file or spatially split among files)
 * can form a temporal sequence. This done by using another file naming
 * convention. If the database name is followed by `-s.{RS}`, where `{RS}` is
 * some number sequence), then the databases are treated as a temporal sequence
 * with `{RS}` (called restart numbers) representing the temporal sequence
 * order.
 *
 * The follow represents a temporal sequence:
 *
 * @verbatim
 *  mysimoutput.e-s.000
 *  mysimoutput.e-s.001
 *  mysimoutput.e-s.002
 * @endverbatim
 *
 * You can use any number of digits for the restart number, but by convention
 * the number used should be the same for all files. Also by convention, you can
 * leave off the `-s.{RS}` suffix for the first file. The following
 * sequence is internally the same as that above:
 *
 * @verbatim
 *  mysimoutput.e-s
 *  mysimoutput.e-s.001
 *  mysimoutput.e-s.002
 * @endverbatim
 *
 * When a database in the temporal sequence is spatially split in multiple
 * files, the corresponding filename is suffixed by the partition information.
 * For example:
 *
 * @verbatim
 *  mysimoutput.e-s.2.0
 *  mysimoutput.e-s.2.1
 *  mysimoutput.e-s.001.2.0
 *  mysimoutput.e-s.001.2.1
 *  mysimoutput.e-s.002.2.0
 *  mysimoutput.e-s.002.2.1
 * @endverbatim
 *
 * In this case, the filenames take the form `{DBNAME}-s.{RS}.{NP}.{RANK}`,
 * where `{DBNAME}` is the database name, `{RS}` is the restart number,
 * `{NP}` is the number of spatial partitions and `{RANK}` is the spatial partition number.
 *
 * @section References References
 * * [Sierra IO System](http://gsjaardema.github.io/seacas/)
 */

#ifndef vtkIOSSReader_h
#define vtkIOSSReader_h

#include "vtkIOIOSSModule.h" // for export macros
#include "vtkNew.h"          // for vtkNew
#include "vtkReaderAlgorithm.h"

class vtkDataArraySelection;
class vtkDataAssembly;
class vtkMultiProcessController;

class VTKIOIOSS_EXPORT vtkIOSSReader : public vtkReaderAlgorithm
{
public:
  static vtkIOSSReader* New();
  vtkTypeMacro(vtkIOSSReader, vtkReaderAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * API to set the filenames.
   */
  void AddFileName(VTK_FILEPATH const char* fname);
  void ClearFileNames();
  VTK_FILEPATH const char* GetFileName(int index) const;
  int GetNumberOfFileNames() const;
  ///@}

  /**
   * Set a single filename. Note, this will clear all existing filenames.
   */
  void SetFileName(VTK_FILEPATH const char* fname);

  ///@{
  /**
   * Get/Set the IOSS database name to use for reading the file. If not
   * specified (default), the reader will determine based on the file extension.
   */
  vtkSetStringMacro(DatabaseTypeOverride);
  vtkGetStringMacro(DatabaseTypeOverride);
  ///@}

  ///@{
  /**
   * When set to true, the reader can automatically locate and load additional
   * files that are part of the collection.
   */
  void SetScanForRelatedFiles(bool value);
  vtkGetMacro(ScanForRelatedFiles, bool);
  vtkBooleanMacro(ScanForRelatedFiles, bool);
  ///@}

  ///@{
  /**
   * This provides a mechanism to limit to reading to certain files in a
   * spatially partitioned file-series. To just specific subset of files, one can
   * always simply specify those files using `AddFileName` and then set
   * `ScanForRelatedFiles` to false. Another way is to let the reader scan for all
   * related files and then use `FileRange` and `FileStride` to limit which
   * files are read.
   *
   * If the range is invalid, i.e. `FileRange[0] >= FileRange[1]`, it's assumed
   * that no file-range overrides have been specified and both FileRange and
   * FileStride will be ignored. When valid, only the chosen subset of files
   * will be processed.
   */
  vtkSetVector2Macro(FileRange, int);
  vtkGetVector2Macro(FileRange, int);
  vtkSetClampMacro(FileStride, int, 1, VTK_INT_MAX);
  vtkGetMacro(FileStride, int);
  ///@}

  ///@{
  /**
   * When set to true, the reader will add a cell-data array for cells named
   * 'file_id' which identifies the file number when reading spatially
   * partitioned files.
   *
   * Default is false.
   */
  vtkSetMacro(GenerateFileId, bool);
  vtkGetMacro(GenerateFileId, bool);
  vtkBooleanMacro(GenerateFileId, bool);
  ///@}

  ///@{
  /**
   * When set to true (default), the reader will read ids associated with
   * elements.
   */
  vtkSetMacro(ReadIds, bool);
  vtkGetMacro(ReadIds, bool);
  vtkBooleanMacro(ReadIds, bool);
  ///@}

  ///@{
  /**
   * Node related data, including point coordinates, point field data etc. is
   * typically shared between all blocks and sets. By default, the reader will
   * remove unused points for each block or set. To avoid this, set this flag to
   * false.
   *
   * Default is true, unused points are removed.
   */
  void SetRemoveUnusedPoints(bool);
  vtkGetMacro(RemoveUnusedPoints, bool);
  vtkBooleanMacro(RemoveUnusedPoints, bool);
  ///@}

  ///@{
  /**
   * When set to true (default), if an array named 'displacement' is present in
   * the node field arrays, it will be used to transform the point coordinates.
   */
  vtkSetMacro(ApplyDisplacements, bool);
  vtkGetMacro(ApplyDisplacements, bool);
  vtkBooleanMacro(ApplyDisplacements, bool);
  ///@}

  ///@{
  /**
   * When set to true (default), the reader will read global fields.
   */
  vtkSetMacro(ReadGlobalFields, bool);
  vtkGetMacro(ReadGlobalFields, bool);
  vtkBooleanMacro(ReadGlobalFields, bool);
  ///@}

  ///@{
  /**
   * When set to true (default), the reader will read quality assurance and
   * information fields.
   */
  vtkSetMacro(ReadQAAndInformationRecords, bool);
  vtkGetMacro(ReadQAAndInformationRecords, bool);
  vtkBooleanMacro(ReadQAAndInformationRecords, bool);
  ///@}

  ///@{
  /**
   * Get/Set the controller to use when working in parallel. Initialized to
   * `vtkMultiProcessController::GetGlobalController` in the constructor.
   *
   * The controller is used to using `ReadMetaData` stage to distribute the work
   * of gathering meta-data from multiple files, if any, across ranks and then
   * exchanging that information between all ranks.
   *
   * The actual reading of data is controlled by piece requests sent by the
   * pipeline e.g. using `vtkAlgorithm::UpdatePiece`.
   */
  void SetController(vtkMultiProcessController* controller);
  vtkGetObjectMacro(Controller, vtkMultiProcessController);
  ///@}

  ///@{
  /**
   * IOSS databases support various properties that affect how the database is
   * read. These properties can be set using this API. Note, it's best to call
   * this before the first update to the reader since any change and the reader
   * will flush all caches and close all open databases etc.
   */
  void AddProperty(const char* name, int value);
  void AddProperty(const char* name, double value);
  void AddProperty(const char* name, void* value);
  void AddProperty(const char* name, const char* value);
  void RemoveProperty(const char* name);
  void ClearProperties();
  ///@}

  enum EntityType
  {
    NODEBLOCK,
    EDGEBLOCK,
    FACEBLOCK,
    ELEMENTBLOCK,
    STRUCTUREDBLOCK,
    NODESET,
    EDGESET,
    FACESET,
    ELEMENTSET,
    SIDESET,
    NUMBER_OF_ENTITY_TYPES,

    BLOCK_START = NODEBLOCK,
    BLOCK_END = NODESET,
    SET_START = NODESET,
    SET_END = NUMBER_OF_ENTITY_TYPES,
    ENTITY_START = NODEBLOCK,
    ENTITY_END = NUMBER_OF_ENTITY_TYPES,
  };

  static bool GetEntityTypeIsBlock(int type) { return (type >= BLOCK_START && type < BLOCK_END); }
  static bool GetEntityTypeIsSet(int type) { return (type >= SET_START && type < SET_END); }
  static const char* GetDataAssemblyNodeNameForEntityType(int type);

  vtkDataArraySelection* GetEntitySelection(int type);
  vtkDataArraySelection* GetNodeBlockSelection() { return this->GetEntitySelection(NODEBLOCK); }
  vtkDataArraySelection* GetEdgeBlockSelection() { return this->GetEntitySelection(EDGEBLOCK); }
  vtkDataArraySelection* GetFaceBlockSelection() { return this->GetEntitySelection(FACEBLOCK); }
  vtkDataArraySelection* GetElementBlockSelection()
  {
    return this->GetEntitySelection(ELEMENTBLOCK);
  }
  vtkDataArraySelection* GetStructuredBlockSelection()
  {
    return this->GetEntitySelection(STRUCTUREDBLOCK);
  }
  vtkDataArraySelection* GetNodeSetSelection() { return this->GetEntitySelection(NODESET); }
  vtkDataArraySelection* GetEdgeSetSelection() { return this->GetEntitySelection(EDGESET); }
  vtkDataArraySelection* GetFaceSetSelection() { return this->GetEntitySelection(FACESET); }
  vtkDataArraySelection* GetElementSetSelection() { return this->GetEntitySelection(ELEMENTSET); }
  vtkDataArraySelection* GetSideSetSelection() { return this->GetEntitySelection(SIDESET); }

  vtkDataArraySelection* GetFieldSelection(int type);
  vtkDataArraySelection* GetNodeBlockFieldSelection() { return this->GetFieldSelection(NODEBLOCK); }
  vtkDataArraySelection* GetEdgeBlockFieldSelection() { return this->GetFieldSelection(EDGEBLOCK); }
  vtkDataArraySelection* GetFaceBlockFieldSelection() { return this->GetFieldSelection(FACEBLOCK); }
  vtkDataArraySelection* GetElementBlockFieldSelection()
  {
    return this->GetFieldSelection(ELEMENTBLOCK);
  }
  vtkDataArraySelection* GetStructuredBlockFieldSelection()
  {
    return this->GetFieldSelection(STRUCTUREDBLOCK);
  }
  vtkDataArraySelection* GetNodeSetFieldSelection() { return this->GetFieldSelection(NODESET); }
  vtkDataArraySelection* GetEdgeSetFieldSelection() { return this->GetFieldSelection(EDGESET); }
  vtkDataArraySelection* GetFaceSetFieldSelection() { return this->GetFieldSelection(FACESET); }
  vtkDataArraySelection* GetElementSetFieldSelection()
  {
    return this->GetFieldSelection(ELEMENTSET);
  }
  vtkDataArraySelection* GetSideSetFieldSelection() { return this->GetFieldSelection(SIDESET); }

  void RemoveAllEntitySelections();
  void RemoveAllFieldSelections();
  void RemoveAllSelections()
  {
    this->RemoveAllEntitySelections();
    this->RemoveAllFieldSelections();
  }

  ///@{
  /**
   * Assemblies provide yet another way of selection blocks/sets to load, if
   * available in the dataset. If a block (or set) is enabled either in the
   * block (or set) selection or using assembly selector then it is treated as
   * enabled and will be read.
   *
   * This method returns the vtkDataAssembly. Since IOSS can have multiple
   * assemblies, all are nested under the root "Assemblies" node.
   *
   * If the file has no assemblies, this will return nullptr.
   */
  vtkDataAssembly* GetAssembly();
  ///@}

  /**
   * Whenever the assembly is changed, this tag gets changed. Note, users should
   * not assume that this is monotonically increasing but instead simply rely on
   * its value to determine if the assembly may have changed since last time.
   *
   * It is set to 0 whenever there's no valid assembly available.
   */
  vtkGetMacro(AssemblyTag, int);

  ///@{
  /**
   * API to specify selectors that indicate which branches on the assembly are
   * chosen.
   */
  bool AddSelector(const char* selector);
  void ClearSelectors();
  void SetSelector(const char* selector);
  ///@}

  ///@{
  /**
   * API to access selectors.
   */
  int GetNumberOfSelectors() const;
  const char* GetSelector(int index) const;
  ///@}

  ///@{
  /**
   * Implementation for vtkReaderAlgorithm API
   */
  int ReadMetaData(vtkInformation* metadata) override;
  int ReadMesh(int piece, int npieces, int nghosts, int timestep, vtkDataObject* output) override;
  int ReadPoints(int, int, int, int, vtkDataObject*) override { return 1; }
  int ReadArrays(int, int, int, int, vtkDataObject*) override { return 1; }
  ///@}

  /**
   * Overridden to take into account mtimes for vtkDataArraySelection instances.
   */
  vtkMTimeType GetMTime() override;

  /**
   * Runs a bunch of tests for file pattern matching.
   */
  static bool DoTestFilePatternMatching();

  /**
   * Overridden to release handles at the end of each pass.
   */
  vtkTypeBool ProcessRequest(
    vtkInformation* request, vtkInformationVector** inInfo, vtkInformationVector* outInfo) override;

protected:
  vtkIOSSReader();
  ~vtkIOSSReader() override;

  int FillOutputPortInformation(int port, vtkInformation* info) override;

private:
  vtkIOSSReader(const vtkIOSSReader&) = delete;
  void operator=(const vtkIOSSReader&) = delete;
  vtkNew<vtkDataArraySelection> EntitySelection[NUMBER_OF_ENTITY_TYPES];
  vtkNew<vtkDataArraySelection> EntityFieldSelection[NUMBER_OF_ENTITY_TYPES];
  vtkMultiProcessController* Controller;
  bool GenerateFileId;
  bool ScanForRelatedFiles;
  bool ReadIds;
  bool RemoveUnusedPoints;
  bool ApplyDisplacements;
  bool ReadGlobalFields;
  bool ReadQAAndInformationRecords;
  char* DatabaseTypeOverride;
  int AssemblyTag;
  int FileRange[2];
  int FileStride;

  class vtkInternals;
  vtkInternals* Internals;

  static vtkInformationIntegerKey* ENTITY_TYPE();
};

#endif
