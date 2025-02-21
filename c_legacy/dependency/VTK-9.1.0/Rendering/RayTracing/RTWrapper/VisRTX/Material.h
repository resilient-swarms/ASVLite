#pragma once

#include "vtkLogger.h"

#include "../Types.h"
#include "OSPRayMDL.h"
#include "Texture.h"

#include <VisRTX.h>
#include <cassert>
#include <set>
#include <sstream>
#include <string>

namespace RTW
{
    class Material : public Object
    {
        friend class Geometry;

    public:
        Material(const std::string& type) : Object(RTW_MATERIAL), type(type)
                                            
        {
            VisRTX::Context* rtx = VisRTX_GetContext();

            /*
             * Basic material
             */
            if (this->type == "obj" || this->type == "luminous")
            {
                this->material = rtx->CreateBasicMaterial();
            }

            /*
             * MDL material
             */
            else
            {
                //OSPRay 2.0 name backward compatibility.
                if (this->type == "alloy")
                    this->type = "Alloy";
                else if (this->type == "carPaint")
                    this->type = "CarPaint";
                else if (this->type == "glass")
                    this->type = "Glass";
                else if (this->type == "metal")
                    this->type = "Metal";
                else if (this->type == "metallicPaint")
                    this->type = "MetallicPaint";
                else if (this->type == "obj")
                    this->type = "OBJMaterial";
                else if (this->type == "principled")
                    this->type = "Principled";
                else if (this->type == "thinGlass")
                    this->type = "ThinGlass";

                const std::string materialname = "::ospray::" + this->type;
                try
                {
                    this->material = rtx->CreateMDLMaterial(materialname.c_str(), (char*)OSPRay_mdl, (uint32_t) sizeof(OSPRay_mdl), 0, nullptr, VisRTX::CompilationType::INSTANCE);
                }
                catch(const std::exception&)
                {
                    vtkLogF(ERROR, "VisRTX Error: CreateMDLMaterial failed! Falling back to BasicMaterial.");
                    this->material = nullptr;
                }
                if (!this->material)
                {
                    this->material = rtx->CreateBasicMaterial();
                }
            }
            assert(this->material != nullptr);
        }

        ~Material()
        {
            this->material->Release();
        }

        void Commit() override
        {
            assert(this->material != nullptr);

            /*
             * OBJMaterial (Basic material)
             */
            if (this->type == "obj" && this->material->GetType() == VisRTX::MaterialType::BASIC)
            {
                VisRTX::BasicMaterial* basicMaterial = dynamic_cast<VisRTX::BasicMaterial*>(this->material);
                assert(basicMaterial);
                if (!basicMaterial)
                {
                    return;
                }

                //this->PrintAllParameters();

                basicMaterial->SetDiffuse(this->GetVec3f({ "kd", "Kd" }, VisRTX::Vec3f(0.8f, 0.8f, 0.8f)));
                basicMaterial->SetSpecular(this->GetVec3f({ "ks", "Ks" }, VisRTX::Vec3f(0.0f, 0.0f, 0.0f)));
                basicMaterial->SetShininess(this->GetFloat({ "ns", "Ns" }, 10.0f));
                basicMaterial->SetOpacity(this->GetFloat({ "d", "alpha" }, 1.0f));
                basicMaterial->SetTransparencyFilter(this->GetVec3f({ "tf", "Tf" }, VisRTX::Vec3f(0.0f, 0.0f, 0.0f)));

                Texture* diffuseTex = this->GetObject<Texture>({ "map_Kd", "map_kd" });
                if (diffuseTex)
                    basicMaterial->SetDiffuseTexture(diffuseTex->texture);

                Texture* specularTex = this->GetObject<Texture>({ "map_Ks", "map_ks" });
                if (specularTex)
                    basicMaterial->SetSpecularTexture(specularTex->texture);

                Texture* shininessTex = this->GetObject<Texture>({ "map_Ns", "map_ns" });
                if (shininessTex)
                    basicMaterial->SetShininessTexture(shininessTex->texture);

                Texture* opacityTex = this->GetObject<Texture>({ "map_d", "map_alpha" });
                if (opacityTex)
                    basicMaterial->SetOpacityTexture(opacityTex->texture);

                Texture* bumpTex = this->GetObject<Texture>({ "map_Bump", "map_bump" });
                if (bumpTex)
                    basicMaterial->SetBumpMapTexture(bumpTex->texture);
            }

            /*
             * Luminous (Basic material)
             */
            else if (this->type == "luminous" && this->material->GetType() == VisRTX::MaterialType::BASIC)
            {
                VisRTX::BasicMaterial* basicMaterial = dynamic_cast<VisRTX::BasicMaterial*>(this->material);
                assert(basicMaterial);
                if (!basicMaterial)
                {
                    return;
                }
                basicMaterial->SetEmissive(this->GetVec3f({ "color" }, VisRTX::Vec3f(0.0f, 0.0f, 0.0f)));
                basicMaterial->SetLuminosity(this->GetFloat({ "intensity" }, 0.0f));
            }

            /*
             * Others (MDL material)
             */
            else if (this->material->GetType() == VisRTX::MaterialType::MDL)
            {
                VisRTX::MDLMaterial* mdlMaterial = dynamic_cast<VisRTX::MDLMaterial*>(this->material);
                assert(mdlMaterial);
                if (!mdlMaterial)
                {
                    return;
                }

                std::set<std::string> ospparams_current = this->GetAllParameters();

#define PRINT_MATERIAL_PARAMETERS 0
#if PRINT_MATERIAL_PARAMETERS
                static std::set<std::string> mdltypes_printed;
                if (mdltypes_printed.find(this->type) == mdltypes_printed.end())
                {
                    std::vector<std::string> availableParams;
                    for (uint32_t i = 0; i < mdlMaterial->GetParameterCount(); ++i)
                    {
                        availableParams.push_back(mdlMaterial->GetParameterName(i));
                    }

                    for (const auto &parameter : availableParams)
                    {
                        std::string parameterType;
                        switch (mdlMaterial->GetParameterType(parameter.c_str()))
                        {
                        case VisRTX::ParameterType::NONE:
                            parameterType = "none"; break;
                        case VisRTX::ParameterType::COLOR:
                            parameterType = "color"; break;
                        case VisRTX::ParameterType::DOUBLE:
                            parameterType = "double"; break;
                        case VisRTX::ParameterType::FLOAT:
                            parameterType = "float"; break;
                        case VisRTX::ParameterType::INT:
                            parameterType = "int"; break;
                        case VisRTX::ParameterType::BOOL:
                            parameterType = "bool"; break;
                        case VisRTX::ParameterType::TEXTURE:
                            parameterType = "texture"; break;
                        }
                        std::stringstream logStrBuf;
                        logStrBuf << "(mdl) " << this->type << ": " << parameterType << " " << parameter;
                        vtkLogF(INFO, "%s", logStrBuf.str().c_str());
                    }
                    mdltypes_printed.insert(this->type);
                }


                static std::set<std::string> ospparams_printed;

                for (auto param : ospparams_current)
                {
                    std::string complete = this->type + ": " + param;
                    if (ospparams_printed.find(complete) == ospparams_printed.end())
                    {
                        vtkLogF(INFO, "(osp) %s", complete.c_str());
                        ospparams_printed.insert(complete);
                    }
                }
#endif //PRINT_MATERIAL_PARAMETERS

#define WARN_NOT_IMPLEMENTED() vtkLogF(WARNING, "Warning: type \"%s\" not implemented (Material: %s, %s)", paramType.c_str(), this->type.c_str(), paramName.c_str());

                for (const std::string &param : ospparams_current)
                {
                    std::string paramType, paramName;
                    {
                        std::istringstream iss(param);
                        iss >> paramType;
                        iss >> paramName;
                    }

                    //getters for osp materials require a vector of names
                    std::vector<std::string> names;
                    names.push_back(paramName);

                    //rename parameters if needed (osp name -> mdl name)

                    static const std::map<std::pair<std::string, std::string>, std::string> renameMap
                    {
                        { { "obj", "map_kd" }, "map_Kd"},
                        { { "obj", "map_bump" }, "map_Bump"},
                        { { "Glass", "etaInside" }, "eta"},
                        { { "obj", "alpha" }, "d"},
                        { { "ThinGlass", "transmission" }, "attenuationColor"}
                    };

                    // explicit renames first
                    auto rename_it = renameMap.find(std::make_pair(this->type, paramName));
                    if (rename_it != renameMap.end())
                    {
                        paramName = rename_it->second;
                    }
                    else
                    {
                        //replace "...Map" with "map_..."
                        const std::string ospSuffix = "Map";
                        const std::string mdlPrefix = "map_";
                        if (paramName.length() >= ospSuffix.length()
                            && paramName.compare(paramName.length() - ospSuffix.length(),
                                ospSuffix.length(), ospSuffix) == 0)
                        {
                            std::string name =
                                paramName.substr(0, paramName.length() - ospSuffix.length());
                            paramName = mdlPrefix + name;
                        }
                    }

                    //exceptions first, e.g. spectra; then handle parameters by type
                    if (paramName == std::string("ior") && paramType == std::string("object"))
                    {
                        Data* iorData = this->GetObject<Data>(names);
                        assert(iorData->GetElementDataType() == RTW_VEC3F);

                        if (iorData->GetElementDataType() != RTW_VEC3F)
                        {
                            vtkLogF(ERROR, "Error: unexpected data type in ior object");
                            return;
                        }

                        const unsigned n_input = iorData->GetNumElements();

                        const VisRTX::Vec3f *input = (const VisRTX::Vec3f*)iorData->GetData();

                        static const unsigned spectrum_size = 8;
                        static const float wavelength_begin = 430.f;
                        static const float wavelength_spacing = 35.f;

                        float eta[spectrum_size], k[spectrum_size];

                        //subsample ior array
                        unsigned iinput = 0u, iprev = 0u;
                        for (unsigned iwl = 0u; iwl < spectrum_size; iwl++)
                        {
                            const float currentwl = wavelength_begin + (float)iwl * wavelength_spacing;
                            for (; iinput < n_input - 1 && input[iinput].x < currentwl; ++iinput)
                            {
                                iprev = iinput;
                            }
                            if (input[iprev].x == input[iinput].x)
                            {
                                eta[iwl] = input[iprev].y;
                                k[iwl] = input[iprev].z;
                            }
                            else
                            {
                                const float t = (currentwl - input[iprev].x) / (input[iinput].x - input[iprev].x);
                                eta[iwl] = (1.f - t) * input[iprev].y + t * input[iinput].y;
                                k[iwl] = (1.f - t) * input[iprev].z + t * input[iinput].z;
                            }
                        }

                        //response functions
                        static const float response_sRGB_r[spectrum_size] =
                        {
                            0.0598548,
                            -0.0234574,
                            -0.220138,
                            -0.238902,
                            0.316327,
                            0.738315,
                            0.323302,
                            0.0446981
                        };

                        static const float response_sRGB_g[spectrum_size] =
                        {
                            -0.0567346,
                            -0.0160361,
                            0.223861,
                            0.531185,
                            0.337221,
                            0.0149718,
                            -0.0296053,
                            -0.00486239
                        };

                        static const float response_sRGB_b[spectrum_size] =
                        {
                            0.420693,
                            0.616597,
                            0.0796766,
                            -0.0496266,
                            -0.0473149,
                            -0.0167536,
                            -0.00295686,
                            -0.000314818
                        };

                        //apply response functions to convert to RGB
                        VisRTX::Vec3f eta3(0.f, 0.f, 0.f), k3(0.f, 0.f, 0.f);
                        for (unsigned iwl = 0u; iwl < spectrum_size; ++iwl)
                        {
                            VisRTX::Vec3f response(response_sRGB_r[iwl], response_sRGB_g[iwl], response_sRGB_b[iwl]);

                            eta3.x += response.x * eta[iwl];
                            eta3.y += response.y * eta[iwl];
                            eta3.z += response.z * eta[iwl];
                            k3.x += response.x * k[iwl];
                            k3.y += response.y * k[iwl];
                            k3.z += response.z * k[iwl];
                        }

                        mdlMaterial->SetParameterColor("eta", eta3);
                        mdlMaterial->SetParameterColor("k", k3);
                    }
                    else if (paramType == std::string("string"))
                    {
                        std::string ospParam = this->GetString(names);
                        WARN_NOT_IMPLEMENTED();
                    }
                    else if (paramType == std::string("object"))
                    {
                        Texture* ospParam = this->GetObject<Texture>(names);
                        if (ospParam)
                        {
                            mdlMaterial->SetParameterTexture(paramName.c_str(), ospParam->texture);
                        }
                        else
                        {
                            vtkLogF(WARNING, "Object \"%s\" of material type \"%s\" is not a texture.", paramName.c_str(), this->type.c_str());
                        }
                    }
                    else if (paramType == std::string("int1"))
                    {
                        int ospParam = this->GetInt(names);

                        if (mdlMaterial->GetParameterType(paramName.c_str()) == VisRTX::ParameterType::BOOL)
                        {
                            mdlMaterial->SetParameterBool(paramName.c_str(), ospParam > 0);
                        }
                        else
                        {
                            mdlMaterial->SetParameterInt(paramName.c_str(), ospParam);
                        }
                    }
                    else if (paramType == std::string("float1"))
                    {
                        float ospParam = this->GetFloat(names);

                        if (mdlMaterial->GetParameterType(paramName.c_str()) == VisRTX::ParameterType::BOOL)
                        {
                            mdlMaterial->SetParameterBool(paramName.c_str(), ospParam > 0.0f);
                        }
                        else
                        {
                            mdlMaterial->SetParameterFloat(paramName.c_str(), ospParam);
                        }
                    }
                    else if (paramType == std::string("float2"))
                    {
                        VisRTX::Vec2f ospParam = this->GetVec2f(names);
                        WARN_NOT_IMPLEMENTED();
                    }
                    else if (paramType == std::string("int3"))
                    {
                        VisRTX::Vec3i ospParam = this->GetVec3i(names);
                        WARN_NOT_IMPLEMENTED();
                    }
                    else if (paramType == std::string("float3"))
                    {
                        VisRTX::Vec3f ospParam = this->GetVec3f(names);
                        mdlMaterial->SetParameterColor(paramName.c_str(), ospParam);
                    }
                    else if (paramType == std::string("float4"))
                    {
                        VisRTX::Vec4f ospParam = this->GetVec4f(names);
                        WARN_NOT_IMPLEMENTED();
                    }
                    else
                    {
                        WARN_NOT_IMPLEMENTED();
                    }
                }

                mdlMaterial->Compile();
                }
            }

    private:
        std::string type;
        VisRTX::Material* material = nullptr;
        };
    }
