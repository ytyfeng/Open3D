// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <backend/DriverEnums.h>
#include <backend/PixelBufferDescriptor.h>
#include <filament/Renderer.h>
#include <filament/SwapChain.h>
#include <memory>
#include "Open3D/GUI/Application.h"
#include "Open3D/Geometry/Geometry.h"
#include "Open3D/Geometry/BoundingVolume.h"
#include "Open3D/IO/ClassIO/FileFormatIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"
#include "Open3D/IO/ClassIO/PointCloudIO.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/Visualization/Rendering/Camera.h"
#include "Open3D/Visualization/Rendering/Filament/FilamentEngine.h"
#include "Open3D/Visualization/Rendering/Filament/FilamentRenderer.h"
#include "Open3D/Visualization/Rendering/Filament/FilamentResourceManager.h"
#include "Open3D/Visualization/Rendering/Filament/FilamentView.h"
#include "Open3D/Visualization/Rendering/Scene.h"
#include "Open3D/Visualization/Rendering/View.h"
#include "Open3D/Visualization/Visualizer/GuiVisualizer.h"

using namespace open3d;

std::shared_ptr<geometry::Geometry3D>
LoadGeometry(const std::string &path) {
    auto geometry = std::shared_ptr<geometry::Geometry3D>();

    auto geometry_type = io::ReadFileGeometryType(path);

    auto mesh = std::make_shared<geometry::TriangleMesh>();
    bool mesh_success = false;
    if (geometry_type & io::CONTAINS_TRIANGLES) {
        try {
            mesh_success = io::ReadTriangleMesh(path, *mesh);
        } catch (...) {
            mesh_success = false;
        }
    }
    if (mesh_success) {
        if (mesh->triangles_.size() == 0) {
            utility::LogWarning(
                    "Contains 0 triangles, will read as point cloud");
            mesh.reset();
        } else {
            mesh->ComputeVertexNormals();
            if (mesh->vertex_colors_.empty()) {
                mesh->PaintUniformColor({1, 1, 1});
            }
            geometry = mesh;
        }
        // Make sure the mesh has texture coordinates
        if (!mesh->HasTriangleUvs()) {
            mesh->triangle_uvs_.resize(mesh->triangles_.size() * 3, {0.0, 0.0});
        }
    } else {
        // LogError throws an exception, which we don't want, because this might
        // be a point cloud.
        utility::LogInfo("{} appears to be a point cloud", path.c_str());
        mesh.reset();
    }

    if (!geometry) {
        auto cloud = std::make_shared<geometry::PointCloud>();
        bool success = false;
        try {
            success = io::ReadPointCloud(path, *cloud);
        } catch (...) {
            success = false;
        }
        if (success) {
            utility::LogInfo("Successfully read {}", path.c_str());
            if (!cloud->HasNormals()) {
                cloud->EstimateNormals();
            }
            cloud->NormalizeNormals();
            geometry = cloud;
        } else {
            utility::LogWarning("Failed to read points {}", path.c_str());
            cloud.reset();
        }
    }

    return geometry;
}

struct TextureMaps {
    visualization::TextureHandle albedo_map =
            visualization::FilamentResourceManager::kDefaultTexture;
    visualization::TextureHandle normal_map =
            visualization::FilamentResourceManager::kDefaultNormalMap;
    visualization::TextureHandle ambient_occlusion_map =
            visualization::FilamentResourceManager::kDefaultTexture;
    visualization::TextureHandle roughness_map =
            visualization::FilamentResourceManager::kDefaultTexture;
    visualization::TextureHandle metallic_map =
            visualization::FilamentResourceManager::kDefaultTexture;
    visualization::TextureHandle reflectance_map =
            visualization::FilamentResourceManager::kDefaultTexture;
    visualization::TextureHandle clear_coat_map =
            visualization::FilamentResourceManager::kDefaultTexture;
    visualization::TextureHandle clear_coat_roughness_map =
            visualization::FilamentResourceManager::kDefaultTexture;
    visualization::TextureHandle anisotropy_map =
            visualization::FilamentResourceManager::kDefaultTexture;
};

struct MaterialProperties {
    Eigen::Vector3f base_color = {0.9f, 0.9f, 0.9f};
    float metallic = 0.f;
    float roughness = 0.7;
    float reflectance = 0.5f;
    float clear_coat = 0.2f;
    float clear_coat_roughness = 0.2f;
    float anisotropy = 0.f;
    float point_size = 5.f;
};

struct HeadlessMaterials {
    visualization::MaterialInstanceHandle lit_material;
    visualization::MaterialInstanceHandle unlit_material;
    MaterialProperties properties;
    TextureMaps maps;
} materials_;

void SetMaterialProperties(visualization::FilamentRenderer& renderer) {
    materials_.lit_material =
            renderer.ModifyMaterial(materials_.lit_material)
                    .SetColor("baseColor", materials_.properties.base_color)
                    .SetParameter("baseRoughness",
                                  materials_.properties.roughness)
                    .SetParameter("baseMetallic",
                                  materials_.properties.metallic)
                    .SetParameter("reflectance",
                                  materials_.properties.reflectance)
                    .SetParameter("clearCoat", materials_.properties.clear_coat)
                    .SetParameter("clearCoatRoughness",
                                  materials_.properties.clear_coat_roughness)
                    .SetParameter("anisotropy",
                                  materials_.properties.anisotropy)
                    .SetTexture(
                            "albedo", materials_.maps.albedo_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "normalMap", materials_.maps.normal_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "ambientOcclusionMap",
                            materials_.maps.ambient_occlusion_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "roughnessMap", materials_.maps.roughness_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "metallicMap", materials_.maps.metallic_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "reflectanceMap", materials_.maps.reflectance_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "clearCoatMap", materials_.maps.clear_coat_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "clearCoatRoughnessMap",
                            materials_.maps.clear_coat_roughness_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .SetTexture(
                            "anisotropyMap", materials_.maps.anisotropy_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .Finish();
    materials_.unlit_material =
            renderer.ModifyMaterial(materials_.unlit_material)
                    .SetColor("baseColor", materials_.properties.base_color)
                    .SetTexture(
                            "albedo", materials_.maps.albedo_map,
                            visualization::TextureSamplerParameters::Pretty())
                    .Finish();
}

void PrepareGeometry(std::shared_ptr<geometry::Geometry> geom, visualization::FilamentRenderer& renderer) {
    // Massage the geometry and get its material properties
    if (geom->GetGeometryType() ==  geometry::Geometry::GeometryType::TriangleMesh) {
        auto mesh =
                std::static_pointer_cast<const geometry::TriangleMesh>(geom);

        bool albedo_only = true;
        if (mesh->HasMaterials()) {
            auto mesh_material = mesh->materials_.begin()->second;
            materials_.properties.base_color.x() = mesh_material.baseColor.r();
            materials_.properties.base_color.y() = mesh_material.baseColor.g();
            materials_.properties.base_color.z() = mesh_material.baseColor.b();
            materials_.properties.roughness = mesh_material.baseRoughness;
            materials_.properties.reflectance = mesh_material.baseReflectance;
            materials_.properties.clear_coat = mesh_material.baseClearCoat;
            materials_.properties.clear_coat_roughness =
                    mesh_material.baseClearCoatRoughness;
            materials_.properties.anisotropy = mesh_material.baseAnisotropy;

            auto is_map_valid =
                    [](std::shared_ptr<geometry::Image> map) -> bool {
                        return map && map->HasData();
                    };

            if (is_map_valid(mesh_material.albedo)) {
                materials_.maps.albedo_map =
                        renderer.AddTexture(mesh_material.albedo);
            }
            if (is_map_valid(mesh_material.normalMap)) {
                materials_.maps.normal_map =
                        renderer.AddTexture(mesh_material.normalMap);
                albedo_only = false;
            }
            if (is_map_valid(mesh_material.ambientOcclusion)) {
                materials_.maps.ambient_occlusion_map =
                        renderer.AddTexture(mesh_material.ambientOcclusion);
                albedo_only = false;
            }
            if (is_map_valid(mesh_material.roughness)) {
                materials_.maps.roughness_map =
                        renderer.AddTexture(mesh_material.roughness);
                albedo_only = false;
            }
            if (is_map_valid(mesh_material.metallic)) {
                materials_.properties.metallic = 1.f;
                materials_.maps.metallic_map =
                        renderer.AddTexture(mesh_material.metallic);
                albedo_only = false;
            } else {
                materials_.properties.metallic = 0.f;
            }
            if (is_map_valid(mesh_material.reflectance)) {
                materials_.maps.reflectance_map =
                        renderer.AddTexture(mesh_material.reflectance);
                albedo_only = false;
            }
            if (is_map_valid(mesh_material.clearCoat)) {
                materials_.maps.clear_coat_map =
                        renderer.AddTexture(mesh_material.clearCoat);
                albedo_only = false;
            }
            if (is_map_valid(mesh_material.clearCoatRoughness)) {
                materials_.maps.clear_coat_roughness_map =
                        renderer.AddTexture(mesh_material.clearCoatRoughness);
                albedo_only = false;
            }
            if (is_map_valid(mesh_material.anisotropy)) {
                materials_.maps.anisotropy_map =
                        renderer.AddTexture(mesh_material.anisotropy);
                albedo_only = false;
            }
        }
    }
}

const int kBufferWidth = 512;
const int kBufferHeight = 512;

void ReadPixelsCallback(void* buffer, size_t buffer_size, void* user) {
    // Let main loop know we're done
    bool* fdone = static_cast<bool*>(user);
    *fdone = true;

    // Save image
    utility::LogWarning("in READPIXELS callback: {}", buffer_size);
    auto image = std::make_shared<geometry::Image>();
    image->width_ = kBufferWidth;
    image->height_ = kBufferHeight;
    image->num_of_channels_ = 3;
    image->bytes_per_channel_ = 1;
    image->data_ = std::vector<uint8_t>((uint8_t*)buffer, (uint8_t*)buffer + buffer_size);
    std::string opath("headless_lit.png");
    if (!io::WriteImage(opath, *image)) {
        utility::LogWarning("Could not write image to {}", opath);
    }
}

int main(int argc, const char* argv[]) {

    // Need a model name
    if(argc < 2) {
        utility::LogWarning("Usage: Open3DHeadless [meshfile|pointcloud]");
        return -1;
    }

    utility::LogInfo("Initializing rendering engine for headless...");
    
    // Initialize rendering engine
    // NOTE: An App object is currently required because FilamentResourceManager
    // uses it directly to get resource path.
    auto& app = gui::Application::GetInstance();
    app.Initialize(argc, argv);
    
    visualization::EngineInstance::SelectBackend(filament::backend::Backend::OPENGL);
    auto& engine = visualization::EngineInstance::GetInstance();
    auto& resource_mgr = visualization::EngineInstance::GetResourceManager();
    auto renderer = std::make_unique<visualization::FilamentRenderer>(engine, nullptr, resource_mgr);
    auto scene_id = renderer->CreateScene();
    auto scene = renderer->GetScene(scene_id);
    auto view_id = scene->AddView(0, 0, kBufferWidth, kBufferHeight);
    visualization::View* view = scene->GetView(view_id);
    auto swap_chain = engine.createSwapChain(kBufferWidth, kBufferHeight, filament::SwapChain::CONFIG_READABLE);
    
    renderer->SetClearColor(Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));

    utility::LogInfo("Initializing materials...");
    std::string resource_path = app.GetResourcePath();
    auto lit_handle = renderer->AddMaterial(visualization::ResourceLoadRequest(
            (resource_path + std::string("/defaultLit.filamat")).c_str()));
    auto unlit_handle =
            renderer->AddMaterial(visualization::ResourceLoadRequest(
                    (resource_path + std::string("/defaultUnlit.filamat"))
                            .c_str()));
    materials_.lit_material = renderer->AddMaterialInstance(lit_handle);
    materials_.unlit_material = renderer->AddMaterialInstance(unlit_handle);
    materials_.maps = TextureMaps();
    
    // NOTE: not using a gui since this is headless but all the essential
    // rendering work happens inside the gui so we have no choice but to
    // 'create' one
    // std::vector<std::shared_ptr<const geometry::Geometry>> empty;
    // auto vis = std::make_shared<visualization::GuiVisualizer>(empty, "", 32, 32, 0, 0);
    
    utility::LogInfo("Loading model {}", argv[1]);
    auto geom = LoadGeometry(argv[1]);
    if(!geom) return -1; // NOTE: LoadGeometry outputs the relevant warnings on failure
    utility::LogInfo("Model successfully loaded.");
    
    utility::LogInfo("Preparing geometry, materials and scenes...");
    PrepareGeometry(geom, *renderer);
    SetMaterialProperties(*renderer);
    geometry::AxisAlignedBoundingBox bounds;
    auto g3 = std::static_pointer_cast<const geometry::Geometry3D>(geom);
    auto geom_handle = scene->AddGeometry(*g3, materials_.lit_material);
    bounds += scene->GetEntityBoundingBox(geom_handle);
    auto cam = view->GetCamera();
    float max_dim = 1.25f * bounds.GetMaxExtent();
    Eigen::Vector3f center = bounds.GetCenter().cast<float>();
    Eigen::Vector3f eye, up;
    eye = Eigen::Vector3f(center.x(), center.y(), center.z() + max_dim);
    up = Eigen::Vector3f(0, 1, 0);
    cam->LookAt(center, eye, up);
    cam->SetProjection(60.0, 1.0, 1.0, 1000.0f, visualization::Camera::FovType::Vertical);
    
    utility::LogInfo("Prepare to render headless...");
    std::size_t buffer_size = kBufferHeight*kBufferWidth*3*sizeof(uint8_t);
    uint8_t* buffer = static_cast<std::uint8_t*>(malloc(buffer_size));
    memset(buffer, 0xff, buffer_size);
    bool frame_done = false;

    // auto read_pixels_cb = [&frame_done](void* buffer, std::size_t buffer_size, void* user) {
    //                           utility::LogWarning("We got a READ PIXELS CALLBACK!!!");
    //                           frame_done = true;
    //                       };

    utility::LogInfo("Render headless...");
    auto* downcast_view = dynamic_cast<visualization::FilamentView*>(view);
    bool read_pixels_started = false;
    while (!frame_done) {
        if(renderer->GetNative()->beginFrame(swap_chain)) {
            renderer->GetNative()->render(downcast_view->GetNativeView());
            
            if(!read_pixels_started) {
                read_pixels_started = true;

                filament::backend::PixelBufferDescriptor pd(
                        buffer, buffer_size,
                        filament::backend::PixelDataFormat::RGB,
                        filament::backend::PixelDataType::UBYTE, ReadPixelsCallback,
                        &frame_done);
                renderer->GetNative()->readPixels(0, 0, kBufferWidth,
                                                  kBufferHeight, std::move(pd));
            }
        }

        renderer->GetNative()->endFrame();
    }


    // Now render to buffer...
    // renderer->RenderToImage(512, 512, view, scene,
    //         [](std::shared_ptr<geometry::Image> image) mutable {
    //             utility::LogWarning("GOT CALLBACK!");
    //             std::string opath("headless_lit.png");
    //             if (!io::WriteImage(opath, *image)) {
    //                 utility::LogWarning("Could not write image to {}", opath);
    //             }
    //         });

    
    //renderer->BeginFrame();
    
    return 0;
}
