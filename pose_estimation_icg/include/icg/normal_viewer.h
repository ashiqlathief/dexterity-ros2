// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef ICG_INCLUDE_ICG_NORMAL_VIEWER_H_
#define ICG_INCLUDE_ICG_NORMAL_VIEWER_H_

#include <icg/camera.h>
#include <icg/common.h>
#include <icg/normal_renderer.h>
#include <icg/renderer_geometry.h>
#include <icg/viewer.h>

#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>

namespace icg
{

    /**
     * \brief \ref Viewer that overlays color images from a \ref ColorCamera with
     * normal renderings based on the geometry stored in the `RendererGeometry`.
     *
     * @param renderer_geometry_ptr referenced \ref RendererGeometry object that
     * defines the geometry that is displayed.
     * @param opacity defines the transparency of pixels from the normal rendering.
     */
    class NormalColorViewer : public ColorViewer
    {
    public:
        // Constructor and setup method
        NormalColorViewer (
            const std::string &name,
            const std::shared_ptr<ColorCamera> &color_camera_ptr,
            const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
            float opacity = 0.5f);
        NormalColorViewer (
            const std::string &name, const std::filesystem::path &metafile_path,
            const std::shared_ptr<ColorCamera> &color_camera_ptr,
            const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr);
        bool SetUp() override;

        // Setters
        void set_renderer_geometry_ptr (
            const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr);
        void set_opacity (float opacity);

        // Main methods
        bool UpdateViewer (int save_index) override;

        // Getters
        std::shared_ptr<RendererGeometry> renderer_geometry_ptr() const override;
        float opacity() const;
        // this getter function was added by object handling group in order to extract the final image that contains the tracking results in image format,
        // the added variable will be only an intermediate variable since the function that mixes (blendes) the image and the rendere is not available outside the class
        cv::Mat Image_renderer_mix() ;
        cv::Mat Image_and_render;
        std::mutex image_and_render_mutex_; // Image_and_render is read from the ROS image callback

    private:
        // Helper method
        bool LoadMetaData();

        // Data
        std::shared_ptr<RendererGeometry> renderer_geometry_ptr_ = nullptr;
        FullNormalRenderer renderer_;
        float opacity_ = 0.5f;
    };

    /**
     * \brief \ref Viewer that overlays normalized depth images from a \ref
     * DepthCamera with normal renderings based on the geometry stored in the
     * `RendererGeometry`.
     *
     * @param renderer_geometry_ptr referenced \ref RendererGeometry object that
     * defines the geometry that is displayed.
     * @param opacity defines the transparency of pixels from the normal rendering.
     */
    class NormalDepthViewer : public DepthViewer
    {
    public:
        // Constructor and setup method
        NormalDepthViewer (
            const std::string &name,
            const std::shared_ptr<DepthCamera> &depth_camera_ptr,
            const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
            float min_depth = 0.0f, float max_depth = 1.0f, float opacity = 0.5f);
        NormalDepthViewer (
            const std::string &name, const std::filesystem::path &metafile_path,
            const std::shared_ptr<DepthCamera> &depth_camera_ptr,
            const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr);
        bool SetUp() override;

        // Setters
        void set_renderer_geometry_ptr (
            const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr);
        void set_opacity (float opacity);

        // Main methods
        bool UpdateViewer (int save_index) override;

        // Getters
        std::shared_ptr<RendererGeometry> renderer_geometry_ptr() const override;
        float opacity() const;

    private:
        // Helper method
        bool LoadMetaData();

        // Data
        std::shared_ptr<RendererGeometry> renderer_geometry_ptr_ = nullptr;
        FullNormalRenderer renderer_;
        float opacity_ = 0.5f;
    };

} // namespace icg

#endif // ICG_INCLUDE_ICG_NORMAL_VIEWER_H_
