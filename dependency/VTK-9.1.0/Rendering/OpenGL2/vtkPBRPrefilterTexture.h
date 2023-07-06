/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPBRPrefilterTexture.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkPBRPrefilterTexture
 * @brief   precompute prefilter texture used in physically based rendering
 *
 * Prefilter texture is a cubemap result of integration of the input cubemap contribution in
 * BRDF equation. The result depends on roughness coefficient so several levels of mipmap are
 * used to store results of different roughness coefficients.
 * It is used in Image Base Lighting to compute the specular part.
 */

#ifndef vtkPBRPrefilterTexture_h
#define vtkPBRPrefilterTexture_h

#include "vtkOpenGLTexture.h"
#include "vtkRenderingOpenGL2Module.h" // For export macro

class vtkOpenGLFramebufferObject;
class vtkOpenGLRenderWindow;
class vtkOpenGLTexture;
class vtkRenderWindow;

class VTKRENDERINGOPENGL2_EXPORT vtkPBRPrefilterTexture : public vtkOpenGLTexture
{
public:
  static vtkPBRPrefilterTexture* New();
  vtkTypeMacro(vtkPBRPrefilterTexture, vtkOpenGLTexture);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Get/Set the input texture.
   */
  void SetInputTexture(vtkOpenGLTexture*);
  vtkGetObjectMacro(InputTexture, vtkOpenGLTexture);
  ///@}

  /**
   * Implement base class method.
   */
  void Load(vtkRenderer*) override;

  /**
   * Implement base class method.
   */
  void Render(vtkRenderer* ren) override { this->Load(ren); }

  ///@{
  /**
   * Get size of texture (input texture height).
   */
  vtkGetMacro(PrefilterSize, unsigned int);
  ///@}

  ///@{
  /**
   * Set/Get the number of mip-map levels.
   * Default is 5.
   */
  vtkGetMacro(PrefilterLevels, unsigned int);
  vtkSetMacro(PrefilterLevels, unsigned int);
  ///@}

  ///@{
  /**
   * Set/Get the maximum number of samples.
   * The number of samples for each roughness is between 1
   * at roughness = 0 and PrefilterMaxSamples at roughness = 1
   * Default is 512.
   */
  vtkGetMacro(PrefilterMaxSamples, unsigned int);
  vtkSetMacro(PrefilterMaxSamples, unsigned int);
  ///@}

  ///@{
  /**
   * Set/Get the conversion to linear color space.
   * If the input texture is in sRGB color space and the conversion is not done by OpenGL
   * directly with the texture format, the conversion can be done in the shader with this flag.
   */
  vtkGetMacro(ConvertToLinear, bool);
  vtkSetMacro(ConvertToLinear, bool);
  vtkBooleanMacro(ConvertToLinear, bool);
  ///@}

  /**
   * Release any graphics resources that are being consumed by this texture.
   * The parameter window could be used to determine which graphic
   * resources to release. Using the same texture object in multiple
   * render windows is NOT currently supported.
   */
  void ReleaseGraphicsResources(vtkWindow*) override;

protected:
  vtkPBRPrefilterTexture() = default;
  ~vtkPBRPrefilterTexture() override;

  unsigned int PrefilterSize;
  unsigned int PrefilterLevels = 5;
  unsigned int PrefilterMaxSamples = 512;
  vtkOpenGLTexture* InputTexture = nullptr;
  bool ConvertToLinear = false;

private:
  vtkPBRPrefilterTexture(const vtkPBRPrefilterTexture&) = delete;
  void operator=(const vtkPBRPrefilterTexture&) = delete;
};

#endif
