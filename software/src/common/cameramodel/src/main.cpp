#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-bayer-io.h"
#include "DASPi-fisheye-camera-model.h"
#include "DASPi-sphere-stitcher.h"
#include "DASPi-image-rotation.h"
#include "DASPi-camera-config.h"

namespace {

cv::Mat fullMask(int width, int height) {
    return cv::Mat(height, width, CV_8U, cv::Scalar(255));
}

cv::Mat applyImageRotation(const cv::Mat& image, ImageRotation rotation) {
    cv::Mat out;

    switch (rotation) {
    case ImageRotation::None:
        out = image.clone();
        break;
    case ImageRotation::Rotate90CW:
        cv::rotate(image, out, cv::ROTATE_90_CLOCKWISE);
        break;
    case ImageRotation::Rotate90CCW:
        cv::rotate(image, out, cv::ROTATE_90_COUNTERCLOCKWISE);
        break;
    case ImageRotation::Rotate180:
        cv::rotate(image, out, cv::ROTATE_180);
        break;
    }

    return out;
}

cv::Size rotatedSize(const cv::Size& size, ImageRotation rotation) {
    switch (rotation) {
    case ImageRotation::Rotate90CW:
    case ImageRotation::Rotate90CCW:
        return cv::Size(size.height, size.width);
    case ImageRotation::None:
    case ImageRotation::Rotate180:
    default:
        return size;
    }
}

std::shared_ptr<ICameraModel> makeFisheyeModelForRotation(ImageRotation rotation,
                                                          int width,
                                                          int height)
{
    const cv::Size size = rotatedSize(cv::Size(width, height), rotation);

    const double cx = static_cast<double>(size.width) / 2.0;
    const double cy = static_cast<double>(size.height) / 2.0;

    // Starting guess based on your 2.7 mm wide-angle lens specs.
    // Tune these later if needed.
    const double fx = 600.0;
    const double fy = 600.0;
    const double maxImageRadiusPx = 580.0;

    return std::make_shared<FisheyeCameraModel>(
        fx,
        fy,
        cx,
        cy,
        maxImageRadiusPx,
        size
    );
}

CameraView makeCameraView(const cv::Mat& image,
                          const std::shared_ptr<ICameraModel>& model,
                          const Eigen::Matrix3d& Rcw,
                          ImageRotation imageRotation)
{
    CameraView cam;

    cam.image = applyImageRotation(image, imageRotation);

    cv::Mat maskNonOverlap = fullMask(image.cols, image.rows);
    cv::Mat maskOverlap = cv::Mat::zeros(image.rows, image.cols, CV_8U);

    cam.maskNonOverlap = applyImageRotation(maskNonOverlap, imageRotation);
    cam.maskOverlap = applyImageRotation(maskOverlap, imageRotation);

    cam.model = model;
    cam.Rcw = Rcw;

    return cam;
}

void printCanonicalProjections(const std::string& name,
                               const std::shared_ptr<ICameraModel>& model)
{
    const auto printProj = [&](const char* label, const Eigen::Vector3d& ray) {
        const ProjectionResult p = model->project(ray.normalized());
        std::cout << name << " " << label
                  << ": valid=" << p.valid
                  << " uv=(" << p.uv.x << ", " << p.uv.y << ")\n";
    };

    printProj("forward", Eigen::Vector3d(0, 0, 1));
    printProj("right",   Eigen::Vector3d(1, 0, 0));
    printProj("left",    Eigen::Vector3d(-1, 0, 0));
    printProj("up",      Eigen::Vector3d(0, 1, 0));
    printProj("down",    Eigen::Vector3d(0, -1, 0));
}

void printCameraInfo(const std::string& name, const CameraView& cam) {
    std::cout << name << ".image: "
              << cam.image.cols << "x" << cam.image.rows << '\n';

    std::cout << name << ".maskNonOverlap: "
              << cam.maskNonOverlap.cols << "x" << cam.maskNonOverlap.rows << '\n';

    std::cout << name << ".model size: "
              << cam.model->imageSize().width << "x"
              << cam.model->imageSize().height << '\n';
}

} // namespace

int main() {
    try {
        const int width = 1456;
        const int height = 1088;

        const std::string home = std::getenv("HOME") ? std::getenv("HOME") : "";
        if (home.empty()) {
            std::cerr << "HOME environment variable is not set\n";
            return 1;
        }

        // Camera poses in world space.
        const Eigen::Matrix3d R_left = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d R_right;
        R_right = Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitY());

        // Easy-to-edit per-camera configuration.
        std::vector<CameraConfig> configs = {
            {
                "left",
                home + "/bayer_dumps/output-10.0.2.3_0.bayer",
                ImageRotation::Rotate90CCW,   // left camera: 90 CW
                R_left
            },
            {
                "right",
                home + "/bayer_dumps/output-10.0.3.3_0.bayer",
                ImageRotation::Rotate90CW,  // right camera: 90 CCW
                R_right
            }
        };

        std::vector<CameraView> cameras;
        cameras.reserve(configs.size());

        for (const auto& cfg : configs) {
            const cv::Mat rgb =
                DASPi::LoadBayer16AsBgr8(cfg.path, width, height);

            cv::imwrite("debug_" + cfg.name + ".png", rgb);

            auto model = makeFisheyeModelForRotation(cfg.imageRotation, width, height);

            printCanonicalProjections(cfg.name, model);

            CameraView cam = makeCameraView(rgb, model, cfg.Rcw, cfg.imageRotation);
            printCameraInfo(cfg.name, cam);

            cameras.push_back(std::move(cam));
        }

        SphereStitchConfig config;
        config.outputWidth = 1456;
        config.outputHeight = 1088;
        config.blendPower = 4.0;

        const Eigen::Vector3d centerRay =
            spherical::EquirectPixelToWorldRay(
                config.outputWidth / 2,
                config.outputHeight / 2,
                config.outputWidth,
                config.outputHeight);

        std::cout << "Panorama center ray = ("
                  << centerRay.x() << ", "
                  << centerRay.y() << ", "
                  << centerRay.z() << ")\n";

        if (!cameras.empty()) {
            const ProjectionResult centerProj = cameras.front().model->project(centerRay);
            std::cout << "Panorama center projection: valid=" << centerProj.valid
                      << " uv=(" << centerProj.uv.x << ", " << centerProj.uv.y << ")\n";
        }

        SphereStitcher stitcher(cameras, config);

        std::cout << "Stitching...\n";

        cv::Mat validMask;
        const cv::Mat pano = stitcher.stitchFisheye(&validMask);

        cv::imwrite("panorama.png", pano);
        cv::imwrite("valid_mask.png", validMask);

        std::cout << "Saved panorama.png\n";
        std::cout << "Saved valid_mask.png\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
