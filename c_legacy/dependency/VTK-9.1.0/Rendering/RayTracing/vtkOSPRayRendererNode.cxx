/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOSPRayRendererNode.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif

#include "vtkOSPRayRendererNode.h"

#include "vtkAbstractVolumeMapper.h"
#include "vtkBoundingBox.h"
#include "vtkCamera.h"
#include "vtkCollectionIterator.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkInformationDoubleKey.h"
#include "vtkInformationDoubleVectorKey.h"
#include "vtkInformationIntegerKey.h"
#include "vtkInformationObjectBaseKey.h"
#include "vtkInformationStringKey.h"
#include "vtkLight.h"
#include "vtkMapper.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkOSPRayActorNode.h"
#include "vtkOSPRayCameraNode.h"
#include "vtkOSPRayLightNode.h"
#include "vtkOSPRayMaterialHelpers.h"
#include "vtkOSPRayMaterialLibrary.h"
#include "vtkOSPRayVolumeNode.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkTexture.h"
#include "vtkTransform.h"
#include "vtkVolume.h"
#include "vtkVolumeCollection.h"
#include "vtkWeakPointer.h"

#include "RTWrapper/RTWrapper.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace ospray
{
namespace opengl
{

// code borrowed from ospray::modules::opengl to facilitate updating
// and linking
// todo: use ospray's copy instead of this
inline osp::vec3f operator*(const osp::vec3f& a, const osp::vec3f& b)
{
  return osp::vec3f{ a.x * b.x, a.y * b.y, a.z * b.z };
}
inline osp::vec3f operator*(const osp::vec3f& a, float b)
{
  return osp::vec3f{ a.x * b, a.y * b, a.z * b };
}
inline osp::vec3f operator/(const osp::vec3f& a, float b)
{
  return osp::vec3f{ a.x / b, a.y / b, a.z / b };
}
inline osp::vec3f operator*(float b, const osp::vec3f& a)
{
  return osp::vec3f{ a.x * b, a.y * b, a.z * b };
}
inline osp::vec3f operator*=(osp::vec3f a, float b)
{
  return osp::vec3f{ a.x * b, a.y * b, a.z * b };
}
inline osp::vec3f operator-(const osp::vec3f& a, const osp::vec3f& b)
{
  return osp::vec3f{ a.x - b.x, a.y - b.y, a.z - b.z };
}
inline osp::vec3f operator+(const osp::vec3f& a, const osp::vec3f& b)
{
  return osp::vec3f{ a.x + b.x, a.y + b.y, a.z + b.z };
}
inline osp::vec3f cross(const osp::vec3f& a, const osp::vec3f& b)
{
  return osp::vec3f{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

inline float dot(const osp::vec3f& a, const osp::vec3f& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline osp::vec3f normalize(const osp::vec3f& v)
{
  return v / sqrtf(dot(v, v));
}

/*! \brief Compute and return OpenGL depth values from the depth component of the given
  OSPRay framebuffer, using parameters of the current OpenGL context and assuming a
  perspective projection.

  This function automatically determines the parameters of the OpenGL perspective
  projection and camera direction / up vectors. It assumes these values match those
  provided to OSPRay (fovy, aspect, camera direction / up vectors). It then maps the
  OSPRay depth buffer and transforms it to OpenGL depth values according to the OpenGL
  perspective projection.

  The OSPRay frame buffer object must have been constructed with the OSP_FB_DEPTH flag.
*/
OSPTexture getOSPDepthTextureFromOpenGLPerspective(const double& fovy, const double& aspect,
  const double& zNear, const double& zFar, const osp::vec3f& _cameraDir,
  const osp::vec3f& _cameraUp, const float* glDepthBuffer, float* ospDepthBuffer,
  const size_t& glDepthBufferWidth, const size_t& glDepthBufferHeight, RTW::Backend* backend)
{
  osp::vec3f cameraDir = (osp::vec3f&)_cameraDir;
  osp::vec3f cameraUp = (osp::vec3f&)_cameraUp;
  // this should later be done in ISPC...

  // transform OpenGL depth to linear depth
  for (size_t i = 0; i < glDepthBufferWidth * glDepthBufferHeight; i++)
  {
    const double z_n = 2.0 * glDepthBuffer[i] - 1.0;
    ospDepthBuffer[i] = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));
    if (vtkMath::IsNan(ospDepthBuffer[i]))
    {
      ospDepthBuffer[i] = FLT_MAX;
    }
  }

  // transform from orthogonal Z depth to ray distance t
  osp::vec3f dir_du = normalize(cross(cameraDir, cameraUp));
  osp::vec3f dir_dv = normalize(cross(dir_du, cameraDir));

  const float imagePlaneSizeY = 2.f * tanf(fovy / 2.f * M_PI / 180.f);
  const float imagePlaneSizeX = imagePlaneSizeY * aspect;

  dir_du *= imagePlaneSizeX;
  dir_dv *= imagePlaneSizeY;

  const osp::vec3f dir_00 = cameraDir - .5f * dir_du - .5f * dir_dv;

  for (size_t j = 0; j < glDepthBufferHeight; j++)
  {
    for (size_t i = 0; i < glDepthBufferWidth; i++)
    {
      const osp::vec3f dir_ij =
        normalize(dir_00 + float(i) / float(glDepthBufferWidth - 1) * dir_du +
          float(j) / float(glDepthBufferHeight - 1) * dir_dv);

      const float t = ospDepthBuffer[j * glDepthBufferWidth + i] / dot(cameraDir, dir_ij);
      ospDepthBuffer[j * glDepthBufferWidth + i] = t;
    }
  }

  // nearest texture filtering required for depth textures -- we don't want interpolation of depth
  // values...
  osp::vec2i texSize = { static_cast<int>(glDepthBufferWidth),
    static_cast<int>(glDepthBufferHeight) };
  OSPTexture depthTexture = vtkOSPRayMaterialHelpers::NewTexture2D(
    backend, (osp::vec2i&)texSize, OSP_TEXTURE_R32F, ospDepthBuffer, OSP_TEXTURE_FILTER_NEAREST);

  return depthTexture;
}
}
}

vtkInformationKeyMacro(vtkOSPRayRendererNode, SAMPLES_PER_PIXEL, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, MAX_CONTRIBUTION, Double);
vtkInformationKeyMacro(vtkOSPRayRendererNode, MAX_DEPTH, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, MIN_CONTRIBUTION, Double);
vtkInformationKeyMacro(vtkOSPRayRendererNode, ROULETTE_DEPTH, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, VOLUME_ANISOTROPY, Double);
vtkInformationKeyMacro(vtkOSPRayRendererNode, VARIANCE_THRESHOLD, Double);
vtkInformationKeyMacro(vtkOSPRayRendererNode, MAX_FRAMES, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, AMBIENT_SAMPLES, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, COMPOSITE_ON_GL, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, RENDERER_TYPE, String);
vtkInformationKeyMacro(vtkOSPRayRendererNode, NORTH_POLE, DoubleVector);
vtkInformationKeyMacro(vtkOSPRayRendererNode, EAST_POLE, DoubleVector);
vtkInformationKeyMacro(vtkOSPRayRendererNode, MATERIAL_LIBRARY, ObjectBase);
vtkInformationKeyMacro(vtkOSPRayRendererNode, VIEW_TIME, Double);
vtkInformationKeyMacro(vtkOSPRayRendererNode, TIME_CACHE_SIZE, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, DENOISER_THRESHOLD, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, ENABLE_DENOISER, Integer);
vtkInformationKeyMacro(vtkOSPRayRendererNode, BACKGROUND_MODE, Integer);

class vtkOSPRayRendererNodeInternals
{
  // todo: move the rest of the internal data here too
public:
  vtkOSPRayRendererNodeInternals(vtkOSPRayRendererNode* _owner)
    : Owner(_owner){};

  ~vtkOSPRayRendererNodeInternals() = default;

  bool CanReuseBG(bool forbackplate)
  {
    bool retval = true;
    int index = (forbackplate ? 0 : 1);
    vtkRenderer* ren = vtkRenderer::SafeDownCast(this->Owner->GetRenderable());
    bool useTexture =
      (forbackplate ? ren->GetTexturedBackground() : ren->GetUseImageBasedLighting());
    if (this->lUseTexture[index] != useTexture)
    {
      this->lUseTexture[index] = useTexture;
      retval = false;
    }
    vtkTexture* envTexture =
      (forbackplate ? ren->GetBackgroundTexture() : ren->GetEnvironmentTexture());
    vtkMTimeType envTextureTime = 0;
    if (envTexture)
    {
      envTextureTime = envTexture->GetMTime();
    }
    if (this->lTexture[index] != envTexture || envTextureTime > this->lTextureTime[index])
    {
      this->lTexture[index] = envTexture;
      this->lTextureTime[index] = envTextureTime;
      retval = false;
    }
    bool useGradient =
      (forbackplate ? ren->GetGradientBackground() : ren->GetGradientEnvironmentalBG());
    if (this->lUseGradient[index] != useGradient)
    {
      this->lUseGradient[index] = useGradient;
      retval = false;
    }
    double* color1 = (forbackplate ? ren->GetBackground() : ren->GetEnvironmentalBG());
    double* color2 = (forbackplate ? ren->GetBackground2() : ren->GetEnvironmentalBG2());
    if (this->lColor1[index][0] != color1[0] || this->lColor1[index][1] != color1[1] ||
      this->lColor1[index][2] != color1[2] || this->lColor2[index][0] != color2[0] ||
      this->lColor2[index][1] != color2[1] || this->lColor2[index][2] != color2[2])
    {
      this->lColor1[index][0] = color1[0];
      this->lColor1[index][1] = color1[1];
      this->lColor1[index][2] = color1[2];
      this->lColor2[index][0] = color2[0];
      this->lColor2[index][1] = color2[1];
      this->lColor2[index][2] = color2[2];
      retval = false;
    }
    if (!forbackplate)
    {
      double* up = vtkOSPRayRendererNode::GetNorthPole(ren);
      if (!up)
      {
        up = ren->GetEnvironmentUp();
      }
      if (this->lup[0] != up[0] || this->lup[1] != up[1] || this->lup[2] != up[2])
      {
        this->lup[0] = up[0];
        this->lup[1] = up[1];
        this->lup[2] = up[2];
        retval = false;
      }

      double* east = vtkOSPRayRendererNode::GetEastPole(ren);
      if (!east)
      {
        east = ren->GetEnvironmentRight();
      }
      if (this->least[0] != east[0] || this->least[1] != east[1] || this->least[2] != east[2])
      {
        this->least[0] = east[0];
        this->least[1] = east[1];
        this->least[2] = east[2];
        retval = false;
      }
    }
    return retval;
  }

  bool SetupPathTraceBackground(bool forbackplate, RTW::Backend* backend)
  {
    vtkRenderer* ren = vtkRenderer::SafeDownCast(this->Owner->GetRenderable());

    auto bgMode = vtkOSPRayRendererNode::GetBackgroundMode(ren);
    bool sameMode = (bgMode == this->lBackgroundMode);
    bool forpathtracer = true;
    if (std::string(this->Owner->GetRendererType(ren)).find(std::string("pathtracer")) ==
      std::string::npos)
    {
      forpathtracer = false;
    }

    if (!forpathtracer)
    {
      // scivis is backplate only
      if (!forbackplate)
      {
        return true;
      }
      bgMode = vtkOSPRayRendererNode::Backplate; // ignore user setting
    }

    OSPTexture t2d = nullptr;
    bool reuseable = sameMode && this->CanReuseBG(forbackplate);
    if (!reuseable)
    {
      vtkTexture* text =
        (forbackplate ? ren->GetBackgroundTexture() : ren->GetEnvironmentTexture());
      if (text && (forbackplate ? ren->GetTexturedBackground() : ren->GetUseImageBasedLighting()))
      {
        // user provided 2D background texture
        vtkImageData* vColorTextureMap = text->GetInput();
        // todo: if the imageData is empty, we should download the texture from the GPU
        if (vColorTextureMap)
        {
          bool isSRGB = text->GetUseSRGBColorSpace() ||
            (forpathtracer && vColorTextureMap->GetScalarType() == VTK_UNSIGNED_CHAR);
          t2d = vtkOSPRayMaterialHelpers::VTKToOSPTexture(backend, vColorTextureMap, isSRGB);
        }
      }

      double bgAlpha = ren->GetBackgroundAlpha();
      if (t2d == nullptr)
      {
        // simple color or gradient
        std::vector<unsigned char> ochars;
        double bg1[3];
        if (forbackplate)
        {
          ren->GetBackground(bg1);
        }
        else
        {
          ren->GetEnvironmentalBG(bg1);
          bgAlpha = 1.0;
        }

        int isize = 1;
        int jsize = 1;
        if (forbackplate ? ren->GetGradientBackground() : ren->GetGradientEnvironmentalBG())
        {
          double bg2[3];
          if (forbackplate)
          {
            ren->GetBackground2(bg2);
          }
          else
          {
            ren->GetEnvironmentalBG2(bg2);
          }

          isize = 256; // todo: configurable
          jsize = 2;
          ochars.resize(isize * jsize * 4);
          unsigned char* oc = ochars.data();
          for (int i = 0; i < isize; i++)
          {
            double frac = i / static_cast<double>(isize - 1);
            *(oc + 0) = (bg1[0] * (1.0 - frac) + bg2[0] * frac) * 255;
            *(oc + 1) = (bg1[1] * (1.0 - frac) + bg2[1] * frac) * 255;
            *(oc + 2) = (bg1[2] * (1.0 - frac) + bg2[2] * frac) * 255;
            *(oc + 3) = bgAlpha * 255;
            *(oc + 4) = *(oc + 0);
            *(oc + 5) = *(oc + 1);
            *(oc + 6) = *(oc + 2);
            *(oc + 7) = *(oc + 3);
            oc += 8;
          }
        }
        else
        {
          ochars.resize(4);
          ochars[0] = bg1[0] * 255;
          ochars[1] = bg1[1] * 255;
          ochars[2] = bg1[2] * 255;
          ochars[3] = bgAlpha * 255;
        }

        // when using path tracer, the final image is gamma corrected so the background has to be
        // sampled in linear color space (using OSP_TEXTURE_SRGBA texture format)
        t2d = vtkOSPRayMaterialHelpers::NewTexture2D(backend, osp::vec2i{ jsize, isize },
          (forpathtracer ? OSP_TEXTURE_SRGBA : OSP_TEXTURE_RGBA8), ochars.data(), 0);
      }

      // now apply the texture we chose above to the right place
      if (forbackplate)
      {
        if (bgMode & vtkOSPRayRendererNode::Backplate)
        {
          ospSetObject(this->Owner->GetORenderer(), "map_backplate", t2d);
        }
        else
        {
          ospSetObject(this->Owner->GetORenderer(), "map_backplate", nullptr);
        }
      }
      else
      {
        OSPLight ospLight = ospNewLight("hdri");
        ospSetObject(ospLight, "map", t2d);
        double* up = vtkOSPRayRendererNode::GetNorthPole(ren);
        if (!up)
        {
          up = ren->GetEnvironmentUp();
        }
        ospSetVec3f(ospLight, "up", (float)up[0], (float)up[1], (float)up[2]);
        double* east = vtkOSPRayRendererNode::GetEastPole(ren);
        if (!east)
        {
          east = ren->GetEnvironmentRight();
        }
        ospSetVec3f(ospLight, "direction", (float)east[0], (float)east[1], (float)east[2]);
        if (bgMode == vtkOSPRayRendererNode::Environment)
        {
          ospSetBool(ospLight, "visible", true);
        }
        else
        {
          ospSetBool(ospLight, "visible", false); // prevents blending onto backplate in "both" mode
        }
        ospCommit(ospLight);
        this->BGLight = ospLight;
      }
    } //! reusable

    if (!forbackplate && (bgMode & vtkOSPRayRendererNode::Environment))
    {
      this->Owner->AddLight(this->BGLight); // lights cleared every frame, so always add
    }
    return reuseable;
  }

  std::map<vtkProp3D*, vtkAbstractMapper3D*> LastMapperFor;
  vtkOSPRayRendererNode* Owner;

  vtkOSPRayRendererNode::BackgroundMode lBackgroundMode = vtkOSPRayRendererNode::None;
  double lColor1[2][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
  bool lUseGradient[2] = { false, false };
  double lColor2[2][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
  bool lUseTexture[2] = { false, false };
  vtkWeakPointer<vtkTexture> lTexture[2] = { nullptr, nullptr };
  vtkMTimeType lTextureTime[2] = { 0, 0 };
  double lup[3];
  double least[3];

  double LastViewPort[2];
  double LastParallelScale = 0.0;
  double LastFocalDisk = -1.0;
  double LastFocalDistance = -1.0;
  double LastDTime = 0.0;

  OSPLight BGLight{ nullptr };
  RTW::Backend* Backend = nullptr;
};

//============================================================================
vtkStandardNewMacro(vtkOSPRayRendererNode);

//------------------------------------------------------------------------------
vtkOSPRayRendererNode::vtkOSPRayRendererNode()
{
  this->ColorBufferTex = 0;
  this->DepthBufferTex = 0;
  this->OWorld = nullptr;
  this->ORenderer = nullptr;
  this->NumActors = 0;
  this->ComputeDepth = true;
  this->OFrameBuffer = nullptr;
  this->ImageX = this->ImageY = -1;
  this->CompositeOnGL = false;
  this->Accumulate = true;
  this->AccumulateCount = 0;
  this->ActorCount = 0;
  this->AccumulateTime = 0;
  this->AccumulateMatrix = vtkMatrix4x4::New();
  this->Internal = new vtkOSPRayRendererNodeInternals(this);
  this->PreviousType = "none";
  this->Cache = new vtkOSPRayCache<vtkOSPRayCacheItemObject>;

#ifdef VTKOSPRAY_ENABLE_DENOISER
  this->DenoiserDevice = oidn::newDevice();
  this->DenoiserDevice.commit();
  this->DenoiserFilter = this->DenoiserDevice.newFilter("RT");
#endif
}

//------------------------------------------------------------------------------
vtkOSPRayRendererNode::~vtkOSPRayRendererNode()
{
  if (this->Internal->Backend != nullptr)
  {
    RTW::Backend* backend = this->Internal->Backend;
    ospRelease(this->ORenderer);
    ospRelease(this->OFrameBuffer);
    // DDM NO ospRelease(this->OCamera);
    this->CacheContents.clear();
    this->Cache->SetSize(0);
    this->Lights.clear();
  }
  this->AccumulateMatrix->Delete();
  delete this->Internal;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetSamplesPerPixel(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::SAMPLES_PER_PIXEL(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetSamplesPerPixel(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 1;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::SAMPLES_PER_PIXEL()))
  {
    return (info->Get(vtkOSPRayRendererNode::SAMPLES_PER_PIXEL()));
  }
  return 1;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetMaxContribution(double value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::MAX_CONTRIBUTION(), value);
}

//------------------------------------------------------------------------------
double vtkOSPRayRendererNode::GetMaxContribution(vtkRenderer* renderer)
{
  constexpr double DEFAULT_MAX_CONTRIBUTION = 2.0;
  if (!renderer)
  {
    return DEFAULT_MAX_CONTRIBUTION;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::MAX_CONTRIBUTION()))
  {
    return (info->Get(vtkOSPRayRendererNode::MAX_CONTRIBUTION()));
  }
  return DEFAULT_MAX_CONTRIBUTION;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetMaxDepth(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::MAX_DEPTH(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetMaxDepth(vtkRenderer* renderer)
{
  constexpr int DEFAULT_MAX_DEPTH = 20;
  if (!renderer)
  {
    return DEFAULT_MAX_DEPTH;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::MAX_DEPTH()))
  {
    return (info->Get(vtkOSPRayRendererNode::MAX_DEPTH()));
  }
  return DEFAULT_MAX_DEPTH;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetMinContribution(double value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::MIN_CONTRIBUTION(), value);
}

//------------------------------------------------------------------------------
double vtkOSPRayRendererNode::GetMinContribution(vtkRenderer* renderer)
{
  constexpr double DEFAULT_MIN_CONTRIBUTION = 0.01;
  if (!renderer)
  {
    return DEFAULT_MIN_CONTRIBUTION;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::MIN_CONTRIBUTION()))
  {
    return (info->Get(vtkOSPRayRendererNode::MIN_CONTRIBUTION()));
  }
  return DEFAULT_MIN_CONTRIBUTION;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetRouletteDepth(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::ROULETTE_DEPTH(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetRouletteDepth(vtkRenderer* renderer)
{
  constexpr int DEFAULT_ROULETTE_DEPTH = 5;
  if (!renderer)
  {
    return DEFAULT_ROULETTE_DEPTH;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::ROULETTE_DEPTH()))
  {
    return (info->Get(vtkOSPRayRendererNode::ROULETTE_DEPTH()));
  }
  return DEFAULT_ROULETTE_DEPTH;
}

//----------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetVolumeAnisotropy(double value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::VOLUME_ANISOTROPY(), value);
}

//----------------------------------------------------------------------------
double vtkOSPRayRendererNode::GetVolumeAnisotropy(vtkRenderer* renderer)
{
  constexpr double DEFAULT_VOLUME_ANISOTROPY = 0.0;
  if (!renderer)
  {
    return DEFAULT_VOLUME_ANISOTROPY;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::VOLUME_ANISOTROPY()))
  {
    return (info->Get(vtkOSPRayRendererNode::VOLUME_ANISOTROPY()));
  }
  return DEFAULT_VOLUME_ANISOTROPY;
}

//----------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetVarianceThreshold(double value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::VARIANCE_THRESHOLD(), value);
}

//------------------------------------------------------------------------------
double vtkOSPRayRendererNode::GetVarianceThreshold(vtkRenderer* renderer)
{
  constexpr double DEFAULT_VARIANCE_THRESHOLD = 0.3;
  if (!renderer)
  {
    return DEFAULT_VARIANCE_THRESHOLD;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::VARIANCE_THRESHOLD()))
  {
    return (info->Get(vtkOSPRayRendererNode::VARIANCE_THRESHOLD()));
  }
  return DEFAULT_VARIANCE_THRESHOLD;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetMaterialLibrary(
  vtkOSPRayMaterialLibrary* value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::MATERIAL_LIBRARY(), value);
}

//------------------------------------------------------------------------------
vtkOSPRayMaterialLibrary* vtkOSPRayRendererNode::GetMaterialLibrary(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return nullptr;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::MATERIAL_LIBRARY()))
  {
    vtkObjectBase* obj = info->Get(vtkOSPRayRendererNode::MATERIAL_LIBRARY());
    return (vtkOSPRayMaterialLibrary::SafeDownCast(obj));
  }
  return nullptr;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetMaxFrames(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::MAX_FRAMES(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetMaxFrames(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 1;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::MAX_FRAMES()))
  {
    return (info->Get(vtkOSPRayRendererNode::MAX_FRAMES()));
  }
  return 1;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetRendererType(std::string name, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();

#ifdef VTK_ENABLE_OSPRAY
  if ("scivis" == name || "OSPRay raycaster" == name)
  {
    info->Set(vtkOSPRayRendererNode::RENDERER_TYPE(), "scivis");
  }
  if ("pathtracer" == name || "OSPRay pathtracer" == name)
  {
    info->Set(vtkOSPRayRendererNode::RENDERER_TYPE(), "pathtracer");
  }
#endif

#ifdef VTK_ENABLE_VISRTX
  if ("optix pathtracer" == name || "OptiX pathtracer" == name)
  {
    info->Set(vtkOSPRayRendererNode::RENDERER_TYPE(), "optix pathtracer");
  }
#endif
}

//------------------------------------------------------------------------------
std::string vtkOSPRayRendererNode::GetRendererType(vtkRenderer* renderer)
{
  if (!renderer)
  {
#ifdef VTK_ENABLE_OSPRAY
    return std::string("scivis");
#else
    return std::string("optix pathtracer");
#endif
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::RENDERER_TYPE()))
  {
    return (info->Get(vtkOSPRayRendererNode::RENDERER_TYPE()));
  }
#ifdef VTK_ENABLE_OSPRAY
  return std::string("scivis");
#else
  return std::string("optix pathtracer");
#endif
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetAmbientSamples(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::AMBIENT_SAMPLES(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetAmbientSamples(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 0;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::AMBIENT_SAMPLES()))
  {
    return (info->Get(vtkOSPRayRendererNode::AMBIENT_SAMPLES()));
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetCompositeOnGL(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::COMPOSITE_ON_GL(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetCompositeOnGL(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 0;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::COMPOSITE_ON_GL()))
  {
    return (info->Get(vtkOSPRayRendererNode::COMPOSITE_ON_GL()));
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetNorthPole(double* value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::NORTH_POLE(), value, 3);
}

//------------------------------------------------------------------------------
double* vtkOSPRayRendererNode::GetNorthPole(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return nullptr;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::NORTH_POLE()))
  {
    return (info->Get(vtkOSPRayRendererNode::NORTH_POLE()));
  }
  return nullptr;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetEastPole(double* value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::EAST_POLE(), value, 3);
}

//------------------------------------------------------------------------------
double* vtkOSPRayRendererNode::GetEastPole(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return nullptr;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::EAST_POLE()))
  {
    return (info->Get(vtkOSPRayRendererNode::EAST_POLE()));
  }
  return nullptr;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetViewTime(double value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::VIEW_TIME(), value);
}

//------------------------------------------------------------------------------
double vtkOSPRayRendererNode::GetViewTime(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 0;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::VIEW_TIME()))
  {
    return (info->Get(vtkOSPRayRendererNode::VIEW_TIME()));
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetTimeCacheSize(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::TIME_CACHE_SIZE(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetTimeCacheSize(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 0;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::TIME_CACHE_SIZE()))
  {
    return (info->Get(vtkOSPRayRendererNode::TIME_CACHE_SIZE()));
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetDenoiserThreshold(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::DENOISER_THRESHOLD(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetDenoiserThreshold(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 4;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::DENOISER_THRESHOLD()))
  {
    return (info->Get(vtkOSPRayRendererNode::DENOISER_THRESHOLD()));
  }
  return 4;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetEnableDenoiser(int value, vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::ENABLE_DENOISER(), value);
}

//------------------------------------------------------------------------------
int vtkOSPRayRendererNode::GetEnableDenoiser(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return 0;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::ENABLE_DENOISER()))
  {
    return (info->Get(vtkOSPRayRendererNode::ENABLE_DENOISER()));
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::SetBackgroundMode(
  vtkOSPRayRendererNode::BackgroundMode value, vtkRenderer* renderer)
{
  if (!renderer || value < 0 || value >= vtkOSPRayRendererNode::NumberOfMode)
  {
    return;
  }
  vtkInformation* info = renderer->GetInformation();
  info->Set(vtkOSPRayRendererNode::BACKGROUND_MODE(), value);
}

//------------------------------------------------------------------------------
vtkOSPRayRendererNode::BackgroundMode vtkOSPRayRendererNode::GetBackgroundMode(
  vtkRenderer* renderer)
{
  if (!renderer)
  {
    return vtkOSPRayRendererNode::Environment;
  }
  vtkInformation* info = renderer->GetInformation();
  if (info && info->Has(vtkOSPRayRendererNode::BACKGROUND_MODE()))
  {
    return (vtkOSPRayRendererNode::BackgroundMode)(
      info->Get(vtkOSPRayRendererNode::BACKGROUND_MODE()));
  }
  return vtkOSPRayRendererNode::Environment;
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::Traverse(int operation)
{
  vtkRenderer* ren = vtkRenderer::SafeDownCast(this->GetRenderable());
  if (!ren)
  {
    return;
  }

  // do not override other passes
  if (operation != render)
  {
    this->Superclass::Traverse(operation);
    return;
  }

  this->Apply(operation, true);

  // camera
  // TODO: this repeated traversal to find things of particular types
  // is bad, find something smarter
  auto const& nodes = this->GetChildren();
  for (auto node : nodes)
  {
    vtkOSPRayCameraNode* child = vtkOSPRayCameraNode::SafeDownCast(node);
    if (child)
    {
      child->Traverse(operation);
      this->OCamera = child->GetOCamera();
      break;
    }
  }

  RTW::Backend* backend = this->Internal->Backend;
  if (backend == nullptr)
  {
    return;
  }

  // lights
  this->Lights.clear();
  bool hasAmbient = false;
  for (auto node : nodes)
  {
    vtkOSPRayLightNode* child = vtkOSPRayLightNode::SafeDownCast(node);
    if (child)
    {
      child->Traverse(operation);
      if (child->GetIsAmbient(vtkLight::SafeDownCast(child->GetRenderable())))
      {
        hasAmbient = true;
      }
    }
  }

  if (!hasAmbient && (this->GetAmbientSamples(static_cast<vtkRenderer*>(this->Renderable)) > 0))
  {
    // hardcode an ambient light for AO since OSP 1.2 stopped doing so.
    OSPLight ospAmbient = ospNewLight("ambient");
    ospSetString(ospAmbient, "name", "default_ambient");
    ospSetVec3f(ospAmbient, "color", 1.f, 1.f, 1.f);
    ospSetFloat(
      ospAmbient, "intensity", 0.13f * vtkOSPRayLightNode::GetLightScale() * vtkMath::Pi());
    ospCommit(ospAmbient);
    this->Lights.push_back(ospAmbient);
  }

  bool bpreused = this->Internal->SetupPathTraceBackground(true, backend);
  bool envreused = this->Internal->SetupPathTraceBackground(false, backend);
  this->Internal->lBackgroundMode = vtkOSPRayRendererNode::GetBackgroundMode(
    static_cast<vtkRenderer*>(this->Renderable)); // save it only once both of the above checks
  bool bgreused = envreused && bpreused;

  // skip every actor/volume when caching is on
  double tstep = vtkOSPRayRendererNode::GetViewTime(ren);
  bool tCacheHit = this->Cache->Contains(tstep);

  // actors
  // since we have to spatially sort everything
  // let's see if we can avoid that in the common case when
  // the objects have not changed. Note we also cache in actornodes
  // to reuse already created ospray meshes
  vtkMTimeType recent = 0;
  int numAct = 0; // catches removed actors
  for (auto node : nodes)
  {
    vtkOSPRayActorNode* child = vtkOSPRayActorNode::SafeDownCast(node);
    vtkOSPRayVolumeNode* vchild = vtkOSPRayVolumeNode::SafeDownCast(node);
    if (child)
    {
      numAct++;
      recent = std::max(recent, child->GetMTime());
    }
    if (vchild)
    {
      numAct++;
      recent = std::max(recent, vchild->GetMTime());
    }
  }

  bool enable_cache = true; // turn off to force rebuilds for debugging
  if (!tCacheHit && (!enable_cache || (recent > this->RenderTime) || (numAct != this->NumActors)))
  {
    this->Instances.clear();
    this->NumActors = numAct;
    for (auto node : nodes)
    {
      vtkOSPRayActorNode* child = vtkOSPRayActorNode::SafeDownCast(node);
      if (child)
      {
        child->Traverse(operation);
      }
      vtkOSPRayVolumeNode* vchild = vtkOSPRayVolumeNode::SafeDownCast(node);
      if (vchild)
      {
        vchild->Traverse(operation);
      }
    }
    this->RenderTime = recent;
  }

  if (!bgreused)
  {
    // hack to ensure progressive rendering resets when background changes
    this->AccumulateTime = 0;
  }
  this->Apply(operation, false);
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::Invalidate(bool prepass)
{
  if (prepass)
  {
    this->RenderTime = 0;
  }
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::Build(bool prepass)
{
  if (prepass)
  {
    vtkRenderer* aren = vtkRenderer::SafeDownCast(this->Renderable);
    // make sure we have a camera
    if (!aren->IsActiveCameraCreated())
    {
      aren->ResetCamera();
    }
  }
  this->Superclass::Build(prepass);
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::Render(bool prepass)
{
  vtkRenderer* ren = vtkRenderer::SafeDownCast(this->GetRenderable());
  if (!ren)
  {
    return;
  }

  RTW::Backend* backend = this->Internal->Backend;

  if (prepass)
  {
    OSPRenderer oRenderer = nullptr;

    std::string type = this->GetRendererType(static_cast<vtkRenderer*>(this->Renderable));
    if (!this->ORenderer || this->PreviousType != type)
    {
      this->Traverse(invalidate);
      this->Internal->Backend = rtwSwitch(type.c_str());
      if (this->Internal->Backend == nullptr)
      {
        return;
      }
      backend = this->Internal->Backend;

      oRenderer = ospNewRenderer(type.c_str());
      this->ORenderer = oRenderer;
      this->PreviousType = type;
    }
    else
    {
      oRenderer = this->ORenderer;
    }

    ospSetFloat(this->ORenderer, "maxContribution", this->GetMaxContribution(ren));
    ospSetFloat(this->ORenderer, "minContribution", this->GetMinContribution(ren));
    ospSetFloat(this->ORenderer, "maxPathLength", this->GetMaxDepth(ren));
    ospSetFloat(this->ORenderer, "rouletteDepth", this->GetRouletteDepth(ren));
    ospSetFloat(this->ORenderer, "varianceThreshold", this->GetVarianceThreshold(ren));
    // ospSetInt(oRenderer, "geometryLights", 0); //avoid a crash in ospray 2.1.0
    ospCommit(this->ORenderer);

    if (ren->GetUseShadows())
    {
      ospSetBool(oRenderer, "shadows", true);
    }
    else
    {
      ospSetBool(oRenderer, "shadows", false);
    }

    // todo: this can be expensive and should be cached
    // also the user might want to control
    vtkBoundingBox bbox(ren->ComputeVisiblePropBounds());
    if (bbox.IsValid())
    {
      float diam = static_cast<float>(bbox.GetDiagonalLength());
      float logDiam = log(diam);
      if (logDiam < 0.f)
      {
        logDiam = 1.f / (fabs(logDiam));
      }
      float epsilon = 1e-5 * logDiam;
      ospSetFloat(oRenderer, "epsilon", epsilon);
      ospSetFloat(oRenderer, "aoDistance", diam * 0.3);
      ospSetInt(oRenderer, "autoEpsilon", 0);
    }
    else
    {
      ospSetFloat(oRenderer, "epsilon", 0.001f);
    }

    vtkVolumeCollection* vc = ren->GetVolumes();
    if (vc->GetNumberOfItems())
    {
      ospSetInt(oRenderer, "aoTransparencyEnabled", 1);
    }

    ospSetInt(oRenderer, "aoSamples", this->GetAmbientSamples(ren));
    ospSetInt(oRenderer, "pixelSamples", this->GetSamplesPerPixel(ren));
    this->CompositeOnGL = (this->GetCompositeOnGL(ren) != 0);
  }
  else
  {

    int newCSize = vtkOSPRayRendererNode::GetTimeCacheSize(ren);
    int currentCSize = this->Cache->GetSize();
    if (newCSize < currentCSize)
    {
      this->CacheContents.clear();
    }
    this->Cache->SetSize(newCSize);
    double tstep = vtkOSPRayRendererNode::GetViewTime(ren);
    auto cached = this->Cache->Get(tstep);
    if (cached)
    {
      this->OWorld = static_cast<OSPWorld>(cached->object);
    }
    else
    {
      if (this->CacheContents.find(this->OWorld) == this->CacheContents.end())
      {
        ospRelease(this->OWorld);
      }
      this->OWorld = ospNewWorld();
      // put the model into a group (collection of models)
      OSPData lights = nullptr;
      if (this->Lights.size())
      {
        lights = ospNewCopyData1D(
          this->Lights.data(), OSP_LIGHT, static_cast<uint32_t>(this->Lights.size()));
        ospCommit(lights);
        ospSetObject(this->OWorld, "light", lights);
      }
      OSPData instances = nullptr;
      if (this->Instances.size())
      {
        instances = ospNewCopyData1D(
          this->Instances.data(), OSP_INSTANCE, static_cast<uint32_t>(this->Instances.size()));
        ospCommit(instances);
        ospSetObject(this->OWorld, "instance", instances);
      }
      ospCommit(this->OWorld);
      ospRelease(lights);
      ospRelease(instances);

      if (this->Cache->HasRoom())
      {
        auto cacheEntry = std::make_shared<vtkOSPRayCacheItemObject>(backend, this->OWorld);
        this->Cache->Set(tstep, std::move(cacheEntry));
        this->CacheContents.insert(this->OWorld);
      }
    }

    OSPRenderer oRenderer = this->ORenderer;
    ospCommit(oRenderer);

    osp::vec2i isize = { this->Size[0], this->Size[1] };
    if (this->ImageX != this->Size[0] || this->ImageY != this->Size[1])
    {
      this->ImageX = this->Size[0];
      this->ImageY = this->Size[1];
      const size_t size = this->ImageX * this->ImageY;
      ospRelease(this->OFrameBuffer);
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wextra"
#endif
      this->OFrameBuffer = ospNewFrameBuffer(isize, OSP_FB_RGBA32F,
        OSP_FB_COLOR | (this->ComputeDepth ? OSP_FB_DEPTH : 0) |
          (this->Accumulate ? OSP_FB_ACCUM : 0)
#ifdef VTKOSPRAY_ENABLE_DENOISER
          | OSP_FB_NORMAL | OSP_FB_ALBEDO
#endif
      );
      this->DenoisedBuffer.resize(size);
      this->ColorBuffer.resize(size);
      this->NormalBuffer.resize(size);
      this->AlbedoBuffer.resize(size);
      this->DenoiserDirty = true;
      ospSetFloat(this->OFrameBuffer, "gamma", 1.0f);
      ospCommit(this->OFrameBuffer);
      ospFrameBufferClear(this->OFrameBuffer);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
      this->Buffer.resize(this->Size[0] * this->Size[1] * 4);
      this->ZBuffer.resize(this->Size[0] * this->Size[1]);
      if (this->CompositeOnGL)
      {
        this->ODepthBuffer.resize(this->Size[0] * this->Size[1]);
      }
    }
    else if (this->Accumulate)
    {
      // check if something has changed
      // if so we clear and start over, otherwise we continue to accumulate
      bool canReuse = true;

      // TODO: these all need some work as checks are not necessarily fast
      // nor sufficient for all cases that matter

      // check for stereo and disable so don't get left in right
      vtkRenderWindow* rwin = vtkRenderWindow::SafeDownCast(ren->GetVTKWindow());
      if (rwin && rwin->GetStereoRender())
      {
        canReuse = false;
      }

      double dtime = vtkOSPRayRendererNode::GetViewTime(ren);
      if (this->Internal->LastDTime != dtime)
      {
        this->Internal->LastDTime = dtime;
        canReuse = false;
      }

      // check for tiling, ie typically putting together large images to save high res pictures
      double* vp = rwin->GetTileViewport();
      if (this->Internal->LastViewPort[0] != vp[0] || this->Internal->LastViewPort[1] != vp[1])
      {
        canReuse = false;
        this->Internal->LastViewPort[0] = vp[0];
        this->Internal->LastViewPort[1] = vp[1];
      }

      // check actors (and time)
      vtkMTimeType m = 0;
      vtkActorCollection* ac = ren->GetActors();
      int nitems = ac->GetNumberOfItems();
      if (nitems != this->ActorCount)
      {
        // TODO: need a hash or something to really check for added/deleted
        this->ActorCount = nitems;
        this->AccumulateCount = 0;
        canReuse = false;
      }
      if (canReuse)
      {
        ac->InitTraversal();
        vtkActor* nac = ac->GetNextActor();
        while (nac)
        {
          if (nac->GetRedrawMTime() > m)
          {
            m = nac->GetRedrawMTime();
          }
          if (this->Internal->LastMapperFor[nac] != nac->GetMapper())
          {
            // a check to ensure vtkPVLODActor restarts on LOD swap
            this->Internal->LastMapperFor[nac] = nac->GetMapper();
            canReuse = false;
          }
          nac = ac->GetNextActor();
        }
        if (this->AccumulateTime < m)
        {
          this->AccumulateTime = m;
          canReuse = false;
        }
      }

      if (canReuse)
      {
        m = 0;
        vtkVolumeCollection* vc = ren->GetVolumes();
        vc->InitTraversal();
        vtkVolume* nvol = vc->GetNextVolume();
        while (nvol)
        {
          if (nvol->GetRedrawMTime() > m)
          {
            m = nvol->GetRedrawMTime();
          }
          if (this->Internal->LastMapperFor[nvol] != nvol->GetMapper())
          {
            // a check to ensure vtkPVLODActor restarts on LOD swap
            this->Internal->LastMapperFor[nvol] = nvol->GetMapper();
            canReuse = false;
          }
          nvol = vc->GetNextVolume();
        };
        if (this->AccumulateTime < m)
        {
          this->AccumulateTime = m;
          canReuse = false;
        }
      }

      if (canReuse)
      {
        // check camera
        // Why not cam->mtime?
        // cam->mtime is bumped by synch after this in parallel so never reuses
        // Why not cam->MVTO->mtime?
        //  cam set's elements directly, so the mtime doesn't bump with motion
        vtkMatrix4x4* camnow = ren->GetActiveCamera()->GetModelViewTransformObject()->GetMatrix();
        for (int i = 0; i < 4; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            if (this->AccumulateMatrix->GetElement(i, j) != camnow->GetElement(i, j))
            {
              this->AccumulateMatrix->DeepCopy(camnow);
              canReuse = false;
              i = 4;
              j = 4;
            }
          }
        }
        if (this->Internal->LastParallelScale != ren->GetActiveCamera()->GetParallelScale())
        {
          this->Internal->LastParallelScale = ren->GetActiveCamera()->GetParallelScale();
          canReuse = false;
        }

        if (this->Internal->LastFocalDisk != ren->GetActiveCamera()->GetFocalDisk())
        {
          this->Internal->LastFocalDisk = ren->GetActiveCamera()->GetFocalDisk();
          canReuse = false;
        }

        if (this->Internal->LastFocalDistance != ren->GetActiveCamera()->GetFocalDistance())
        {
          this->Internal->LastFocalDistance = ren->GetActiveCamera()->GetFocalDistance();
          canReuse = false;
        }
      }
      if (!canReuse)
      {
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wextra"
#endif
        ospFrameBufferClear(this->OFrameBuffer);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
        this->AccumulateCount = 0;
      }
    }
    else if (!this->Accumulate)
    {
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wextra"
#endif
      ospFrameBufferClear(this->OFrameBuffer);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
    }

    vtkCamera* cam = vtkRenderer::SafeDownCast(this->Renderable)->GetActiveCamera();

    ospSetInt(oRenderer, "backgroundEnabled", ren->GetErase());
    if (this->CompositeOnGL && backend->IsSupported(RTW_DEPTH_COMPOSITING))
    {
      vtkRenderWindow* rwin = vtkRenderWindow::SafeDownCast(ren->GetVTKWindow());
      int viewportX, viewportY;
      int viewportWidth, viewportHeight;
      ren->GetTiledSizeAndOrigin(&viewportWidth, &viewportHeight, &viewportX, &viewportY);
      rwin->GetZbufferData(viewportX, viewportY, viewportX + viewportWidth - 1,
        viewportY + viewportHeight - 1, this->GetZBuffer());

      double zNear, zFar;
      double fovy, aspect;
      fovy = cam->GetViewAngle();
      aspect = double(viewportWidth) / double(viewportHeight);
      cam->GetClippingRange(zNear, zFar);
      double camUp[3];
      double camDir[3];
      cam->GetViewUp(camUp);
      cam->GetFocalPoint(camDir);
      osp::vec3f cameraUp = { static_cast<float>(camUp[0]), static_cast<float>(camUp[1]),
        static_cast<float>(camUp[2]) };
      osp::vec3f cameraDir = { static_cast<float>(camDir[0]), static_cast<float>(camDir[1]),
        static_cast<float>(camDir[2]) };
      double cameraPos[3];
      cam->GetPosition(cameraPos);
      cameraDir.x -= cameraPos[0];
      cameraDir.y -= cameraPos[1];
      cameraDir.z -= cameraPos[2];
      cameraDir = ospray::opengl::normalize(cameraDir);

      OSPTexture glDepthTex = ospray::opengl::getOSPDepthTextureFromOpenGLPerspective(fovy, aspect,
        zNear, zFar, (osp::vec3f&)cameraDir, (osp::vec3f&)cameraUp, this->GetZBuffer(),
        this->ODepthBuffer.data(), viewportWidth, viewportHeight, this->Internal->Backend);

      ospSetObject(oRenderer, "map_maxDepth", glDepthTex);
    }
    else
    {
      ospSetObject(oRenderer, "map_maxDepth", 0);
    }

    this->AccumulateCount += this->GetSamplesPerPixel(ren);
    bool useDenoiser =
      this->GetEnableDenoiser(ren) && (this->AccumulateCount >= this->GetDenoiserThreshold(ren));
    ospSetInt(oRenderer, "denoise", useDenoiser ? 1 : 0);
    ospCommit(oRenderer);

    const bool backendDepthNormalization = backend->IsSupported(RTW_DEPTH_NORMALIZATION);
    if (backendDepthNormalization)
    {
      const double* clipValues = cam->GetClippingRange();
      const double clipMin = clipValues[0];
      const double clipMax = clipValues[1];
      backend->SetDepthNormalizationGL(this->OFrameBuffer, clipMin, clipMax);
    }

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wextra"
#endif
    ospRenderFrame(this->OFrameBuffer, oRenderer, this->OCamera, this->OWorld);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

    // Check if backend can do direct OpenGL display using textures
    bool useOpenGLInterop = backend->IsSupported(RTW_OPENGL_INTEROP);

    // Only layer 0 can currently display using OpenGL
    if (ren->GetLayer() != 0)
      useOpenGLInterop = false;

    if (useOpenGLInterop)
    {
      // Check if we actually have an OpenGL window
      vtkRenderWindow* rwin = vtkRenderWindow::SafeDownCast(ren->GetVTKWindow());
      vtkOpenGLRenderWindow* windowOpenGL = vtkOpenGLRenderWindow::SafeDownCast(rwin);
      if (windowOpenGL != nullptr)
      {
        windowOpenGL->MakeCurrent();
        this->ColorBufferTex = backend->GetColorTextureGL(this->OFrameBuffer);
        this->DepthBufferTex = backend->GetDepthTextureGL(this->OFrameBuffer);

        useOpenGLInterop = (this->ColorBufferTex != 0 && this->DepthBufferTex != 0);
      }
      else
      {
        useOpenGLInterop = false;
      }
    }

    if (!useOpenGLInterop)
    {
      const void* rgba = ospMapFrameBuffer(this->OFrameBuffer, OSP_FB_COLOR);
#ifdef VTKOSPRAY_ENABLE_DENOISER
      const osp::vec4f* rgba4f = reinterpret_cast<const osp::vec4f*>(rgba);
      this->ColorBuffer.assign(rgba4f, rgba4f + this->Size[0] * this->Size[1]);
      if (useDenoiser)
      {
        this->Denoise();
      }
      const float* color = reinterpret_cast<const float*>(this->ColorBuffer.data());
      this->Buffer.assign(color, color + this->ImageX * this->ImageY * 4);
#else
      const float* rgbaf = reinterpret_cast<const float*>(rgba);
      this->Buffer.assign(rgbaf, rgbaf + this->Size[0] * this->Size[1] * 4);
#endif
      ospUnmapFrameBuffer(rgba, this->OFrameBuffer);

      if (this->ComputeDepth)
      {
        const float* Z =
          reinterpret_cast<const float*>(ospMapFrameBuffer(this->OFrameBuffer, OSP_FB_DEPTH));

        if (backendDepthNormalization)
        {
          this->ZBuffer.assign(Z, Z + this->Size[0] * this->Size[1]);
        }
        else
        {
          double* clipValues = cam->GetClippingRange();
          double clipMin = clipValues[0];
          double clipMax = clipValues[1];
          double clipDiv = 1.0 / (clipMax - clipMin);

          const float* s = Z;
          float* d = this->ZBuffer.data();
          for (int i = 0; i < (this->Size[0] * this->Size[1]); i++, s++, d++)
          {
            *d = (*s < clipMin ? 1.0 : (*s - clipMin) * clipDiv);
          }
        }

        ospUnmapFrameBuffer(Z, this->OFrameBuffer);
      }
    }
  }
}

void vtkOSPRayRendererNode::Denoise()
{
#ifdef VTKOSPRAY_ENABLE_DENOISER
  RTW::Backend* backend = this->Internal->Backend;
  this->DenoisedBuffer = this->ColorBuffer;
  if (this->DenoiserDirty)
  {
    this->DenoiserFilter.setImage("color", (void*)this->ColorBuffer.data(), oidn::Format::Float3,
      this->ImageX, this->ImageY, 0, sizeof(osp::vec4f));

    this->DenoiserFilter.setImage("normal", (void*)this->NormalBuffer.data(), oidn::Format::Float3,
      this->ImageX, this->ImageY, 0, sizeof(osp::vec3f));

    this->DenoiserFilter.setImage("albedo", (void*)this->AlbedoBuffer.data(), oidn::Format::Float3,
      this->ImageX, this->ImageY, 0, sizeof(osp::vec3f));

    this->DenoiserFilter.setImage("output", (void*)this->DenoisedBuffer.data(),
      oidn::Format::Float3, this->ImageX, this->ImageY, 0, sizeof(osp::vec4f));

    this->DenoiserFilter.commit();
    this->DenoiserDirty = false;
  }

  const auto size = this->ImageX * this->ImageY;
  const osp::vec4f* rgba =
    static_cast<const osp::vec4f*>(ospMapFrameBuffer(this->OFrameBuffer, OSP_FB_COLOR));
  std::copy(rgba, rgba + size, this->ColorBuffer.begin());
  ospUnmapFrameBuffer(rgba, this->OFrameBuffer);
  const osp::vec3f* normal =
    static_cast<const osp::vec3f*>(ospMapFrameBuffer(this->OFrameBuffer, OSP_FB_NORMAL));
  std::copy(normal, normal + size, this->NormalBuffer.begin());
  ospUnmapFrameBuffer(normal, this->OFrameBuffer);
  const osp::vec3f* albedo =
    static_cast<const osp::vec3f*>(ospMapFrameBuffer(this->OFrameBuffer, OSP_FB_ALBEDO));
  std::copy(albedo, albedo + size, this->AlbedoBuffer.begin());
  ospUnmapFrameBuffer(albedo, this->OFrameBuffer);

  this->DenoiserFilter.execute();
  // Carson: not sure we need two buffers
  this->ColorBuffer = this->DenoisedBuffer;
#endif
}

//------------------------------------------------------------------------------
void vtkOSPRayRendererNode::WriteLayer(
  unsigned char* buffer, float* Z, int buffx, int buffy, int layer)
{
  if (layer == 0)
  {
    for (int j = 0; j < buffy && j < this->Size[1]; j++)
    {
      float* iptr = this->Buffer.data() + j * this->Size[0] * 4;
      float* zptr = this->ZBuffer.data() + j * this->Size[0];
      unsigned char* optr = buffer + j * buffx * 4;
      float* ozptr = Z + j * buffx;
      for (int i = 0; i < buffx && i < this->Size[0]; i++)
      {
        *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
        *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
        *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
        *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
        *ozptr++ = *zptr;
        zptr++;
      }
    }
  }
  else
  {
    for (int j = 0; j < buffy && j < this->Size[1]; j++)
    {
      float* iptr = this->Buffer.data() + j * this->Size[0] * 4;
      float* zptr = this->ZBuffer.data() + j * this->Size[0];
      unsigned char* optr = buffer + j * buffx * 4;
      float* ozptr = Z + j * buffx;
      for (int i = 0; i < buffx && i < this->Size[0]; i++)
      {
        if (*zptr < 1.0)
        {
          if (this->CompositeOnGL)
          {
            // ospray is cooperating with GL (osprayvolumemapper)
            float A = iptr[3];
            for (int h = 0; h < 3; h++)
            {
              *optr = static_cast<unsigned char>(
                (*iptr * 255.f) * (1 - A) + static_cast<float>(*optr) * (A));
              optr++;
              iptr++;
            }
            optr++;
            iptr++;
          }
          else
          {
            // ospray owns all layers in window
            *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
            *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
            *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
            *optr++ = static_cast<unsigned char>(vtkMath::ClampValue(*iptr++, 0.f, 1.f) * 255.f);
          }
          *ozptr = *zptr;
        }
        else
        {
          optr += 4;
          iptr += 4;
        }
        ozptr++;
        zptr++;
      }
    }
  }
}

//------------------------------------------------------------------------------
vtkRenderer* vtkOSPRayRendererNode::GetRenderer()
{
  return vtkRenderer::SafeDownCast(this->GetRenderable());
}

//------------------------------------------------------------------------------
vtkOSPRayRendererNode* vtkOSPRayRendererNode::GetRendererNode(vtkViewNode* self)
{
  return static_cast<vtkOSPRayRendererNode*>(self->GetFirstAncestorOfType("vtkOSPRayRendererNode"));
}

//------------------------------------------------------------------------------
RTW::Backend* vtkOSPRayRendererNode::GetBackend()
{
  return this->Internal->Backend;
}
