/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOSPRayLightNode.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkOSPRayLightNode
 * @brief   links vtkLights to OSPRay
 *
 * Translates vtkLight state into OSPRay rendering calls
 */

#ifndef vtkOSPRayLightNode_h
#define vtkOSPRayLightNode_h

#include "vtkLightNode.h"
#include "vtkRenderingRayTracingModule.h" // For export macro

#include "RTWrapper/RTWrapper.h" // for handle types

#include <string> // for std::string

class vtkInformationDoubleKey;
class vtkInformationIntegerKey;
class vtkLight;
class vtkOSPRayRendererNode;

class VTKRENDERINGRAYTRACING_EXPORT vtkOSPRayLightNode : public vtkLightNode
{
public:
  static vtkOSPRayLightNode* New();
  vtkTypeMacro(vtkOSPRayLightNode, vtkLightNode);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Make ospray calls to render me.
   */
  void Render(bool prepass) override;

  ///@{
  /**
   * A global multiplier to all ospray lights.
   * default is 1.0
   */
  static void SetLightScale(double s);
  static double GetLightScale();
  ///@}

  // state beyond rendering core...

  /**
   * When present on light, the light acts as an ambient source.
   * An AmbientLight is one that has no specific position in space and for
   * which only the ambient color term affects the result.
   */
  static vtkInformationIntegerKey* IS_AMBIENT();

  ///@{
  /**
   * Convenience method to set/get IS_AMBIENT on a vtkLight.
   */
  static void SetIsAmbient(int, vtkLight*);
  static int GetIsAmbient(vtkLight*);
  ///@}

  /**
   * The radius setting, when > 0.0, produces soft shadows in the
   * path tracer.
   */
  static vtkInformationDoubleKey* RADIUS();

  ///@{
  /**
   * Convenience method to set/get RADIUS on a vtkLight.
   */
  static void SetRadius(double, vtkLight*);
  static double GetRadius(vtkLight*);
  ///@}

protected:
  vtkOSPRayLightNode();
  ~vtkOSPRayLightNode() override;

private:
  vtkOSPRayLightNode(const vtkOSPRayLightNode&) = delete;
  void operator=(const vtkOSPRayLightNode&) = delete;

  static double LightScale;
  void* OLight;
};

#endif
