/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOSPRayMaterialLibrary.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkOSPRayMaterialLibrary
 * @brief   a collection of materials for vtk apps to draw from
 *
 * A singleton instance of this class manages a collection of materials.
 * The materials can be read in from disk or created programmatically.
 *
 * @sa vtkOSPRayMaterialHelpers
 */

#ifndef vtkOSPRayMaterialLibrary_h
#define vtkOSPRayMaterialLibrary_h

#include "vtkObject.h"
#include "vtkRenderingRayTracingModule.h" // For export macro

#include <initializer_list> //for initializer_list!
#include <map>              //for map!
#include <set>              //for set!
#include <string>           //for string!
#include <vector>           //for vector!

class vtkOSPRayMaterialLibraryInternals;
class vtkTexture;
struct TextureInfo;

class VTKRENDERINGRAYTRACING_EXPORT vtkOSPRayMaterialLibrary : public vtkObject
{
public:
  static vtkOSPRayMaterialLibrary* New();
  vtkTypeMacro(vtkOSPRayMaterialLibrary, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Called to kick off events in all followers.
   */
  void Fire();

  /**
   * Reads the given file of materials and creates the in memory data
   * structures needed to display objects with them. Returns false only if
   * file could not be meaningfully interpreted.
   */
  bool ReadFile(const char* FileName);

  /**
   * Serialize contents to an in memory buffer.
   * If writeImageInline, all textures are stored in a XML format.
   * Else, store the texture using its filename stored in its TextureInfo struct.
   * Warning: you must free the return value using delete.
   */
  const char* WriteBuffer(bool writeImageInline = true);

  /**
   * Serialize contents to a file specified by \p filename.
   * If writeImageInline, all textures are stored in a XML format.
   * Else, store the texture using its filename stored in its TextureInfo struct
   * Warning: if \p filename exists, its content is discarded.
   */
  void WriteFile(const std::string& filename, bool writeImageInline = false);

  /**
   * DeSerialize contents from an in memory buffer as ReadFile does from a
   * file or set of files. Returns false only if buffer could not be
   * meaningfully interpreted.
   */
  bool ReadBuffer(const char* Buffer);

  /**
   * Returns the set of material nicknames.
   */
  std::set<std::string> GetMaterialNames();

  /**
   * Return an implementation name for the given material nickname.
   */
  std::string LookupImplName(const std::string& nickname);

  /**
   * Returns list of variable names set for a specific material.
   */
  std::vector<std::string> GetDoubleShaderVariableList(const std::string& nickname);

  /**
   * Returns a uniform variable.
   */
  std::vector<double> GetDoubleShaderVariable(
    const std::string& nickname, const std::string& varname);

  /**
   * Returns list of texture names set for a specific material.
   */
  std::vector<std::string> GetTextureList(const std::string& nickname);

  /**
   * Returns a texture.
   */
  vtkTexture* GetTexture(const std::string& nickname, const std::string& varname);

  /**
   * Returns the texture information (name, texture and filename) of this \p varname.
   * If not found, return nullptr;
   */
  const TextureInfo* GetTextureInfo(const std::string& nickname, const std::string& varname);

  /**
   * Returns the name (and not the shader variable name) associated to a
   * texture.
   */
  std::string GetTextureName(const std::string& nickname, const std::string& varname);

  /**
   * Returns the filename associated ti a texture (if any).
   */
  std::string GetTextureFilename(const std::string& nickname, const std::string& varname);

  /**
   * Add Material
   * Adds a new material nickname to the set of known materials.
   * If the name is a repeat, we replace the old one.
   **/
  void AddMaterial(const std::string& nickname, const std::string& implname);

  /**
   * Remove Material
   * Removes a material nickname from the set of known materials.
   * Do nothing if material does not exist.
   **/
  void RemoveMaterial(const std::string& nickname);

  /**
   * Add Texture
   * Given a material @c nickname and a shader variable @c varname, set its data
   * to a specific texture @c tex named @c texturename. If not specified the texture
   * is called "unnamedTexture".
   * The last parameter, filename, is defaulted to empty. If specified, the absolut path
   * to find the texture will be stored. Useful when writing the library into a file.
   *
   * Replaces any previous content.
   **/
  void AddTexture(const std::string& nickname, const std::string& varname, vtkTexture* tex,
    const std::string& texturename = "unnamedTexture", const std::string& filename = "");

  /**
   * Remove Texture
   * Removes a texture for a specific materal @c nickname and shader variable @c varname.
   * Do nothing if texture does not exist.
   **/
  void RemoveTexture(const std::string& nickname, const std::string& varname);

  /**
   * Remove all textures of a specific material
   **/
  void RemoveAllTextures(const std::string& nickname);

  /**
   * Add control variable
   * Adds a new control variable. Replaces any previous content.
   * @{
   **/
  void AddShaderVariable(
    const std::string& nickname, const std::string& variablename, int numVars, const double* x);
  void AddShaderVariable(const std::string& nickname, const std::string& variablename,
    const std::initializer_list<double>& data)
  {
    this->AddShaderVariable(nickname, variablename, static_cast<int>(data.size()), data.begin());
  }
  /**@}*/

  /**
   * Remove control variable
   * Removes a new control variable.
   * Do nothing if variable does not exist.
   **/
  void RemoveShaderVariable(const std::string& nickname, const std::string& variablename);

  /**
   * Remove all control variables of a specific material
   **/
  void RemoveAllShaderVariables(const std::string& nickname);

  /**
   * Lists all different parameter types
   */
  enum class ParameterType : unsigned char
  {
    FLOAT,
    NORMALIZED_FLOAT,
    FLOAT_DATA,
    VEC3,
    COLOR_RGB,
    BOOLEAN,
    TEXTURE,
    VEC2,
    VEC4
  };

  using ParametersMap = std::map<std::string, ParameterType>;

  /**
   * Get the dictionary of all possible materials based on OSPRay documentation.
   */
  static const std::map<std::string, ParametersMap>& GetParametersDictionary();

protected:
  vtkOSPRayMaterialLibrary();
  ~vtkOSPRayMaterialLibrary() override;

  bool InternalParse(const char* name, bool IsFile);
  bool InternalParseJSON(const char* name, bool IsFile, std::istream* doc);
  bool InternalParseMTL(const char* name, bool IsFile, std::istream* doc);
  bool ReadTextureFileOrData(const std::string& texFilenameOrData, bool fromfile,
    const std::string& parentDir, vtkTexture* textr, std::string& textureName,
    std::string& textureFilename);

private:
  vtkOSPRayMaterialLibrary(const vtkOSPRayMaterialLibrary&) = delete;
  void operator=(const vtkOSPRayMaterialLibrary&) = delete;

  vtkOSPRayMaterialLibraryInternals* Internal;
};

#endif
