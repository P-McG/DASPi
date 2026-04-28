// DASPi-sphere-stitcher.cpp
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <mutex>

#include "DASPi-sphere-stitcher.h"
//#include "DASPi-spherical-math.h"
//#include "DASPi-rig-utils.h"

namespace {

//void LogCenterDebugIfNeeded(int x,
                            //int y,
                            //int centerX,
                            //int centerY,
                            //const Eigen::Vector3f& ray_world,
                            //const std::vector<Contribution>& contributions)
//{
    //if (x != centerX || y != centerY) {
        //return;
    //}

    //std::cout << "stitch center ray_world=("
              //<< ray_world.x() << ", "
              //<< ray_world.y() << ", "
              //<< ray_world.z() << ")\n";

    //std::cout << "center contributions=" << contributions.size() << '\n';

    //if (!contributions.empty()) {
        //std::cout << "center uv=("
                  //<< contributions[0].uv.x << ", "
                  //<< contributions[0].uv.y << ")\n";
    //}
//}



cv::Vec3d SampleBilinear(const cv::Mat& img, const cv::Point2d& uv) {
    const int x0 = static_cast<int>(std::floor(uv.x));
    const int y0 = static_cast<int>(std::floor(uv.y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if (x0 < 0 || y0 < 0 || x1 >= img.cols || y1 >= img.rows) {
        return cv::Vec3d(0.0, 0.0, 0.0);
    }

    const double dx = uv.x - static_cast<double>(x0);
    const double dy = uv.y - static_cast<double>(y0);

    const cv::Vec3d c00 = img.at<cv::Vec3b>(y0, x0);
    const cv::Vec3d c10 = img.at<cv::Vec3b>(y0, x1);
    const cv::Vec3d c01 = img.at<cv::Vec3b>(y1, x0);
    const cv::Vec3d c11 = img.at<cv::Vec3b>(y1, x1);

    const cv::Vec3d c0 = c00 * (1.0 - dx) + c10 * dx;
    const cv::Vec3d c1 = c01 * (1.0 - dx) + c11 * dx;

    return c0 * (1.0 - dy) + c1 * dy;
}

double OpticalWeight(const Eigen::Vector3d& ray_cam, double power) {
    const double z = ray_cam.z();
    if (z <= 0.0) {
        return 0.0;
    }
    return std::pow(z, power);
}

Eigen::Vector2d WorldRayToEquirectPixel(const Eigen::Vector3d& ray,
                                        int width,
                                        int height)
{
    const Eigen::Vector3d r = ray.normalized();

    const double lambda = std::atan2(r.x(), r.z());
    const double phi = std::asin(r.y());

    const double u = (lambda + M_PI) / (2.0 * M_PI);
    const double v = 0.5 - phi / M_PI;

    return Eigen::Vector2d(u * width, v * height);
}

//bool IsInsideImage(const cv::Mat& image, const cv::Point2d& uv)
//{
    //return uv.x >= 0.0 && uv.x < static_cast<double>(image.cols) &&
           //uv.y >= 0.0 && uv.y < static_cast<double>(image.rows);
//}

//cv::Vec3b SampleNearest3b(const cv::Mat& image, const cv::Point2d& uv)
//{
    //const int x = static_cast<int>(std::round(uv.x));
    //const int y = static_cast<int>(std::round(uv.y));

    //if (x < 0 || y < 0 || x >= image.cols || y >= image.rows) {
        //return cv::Vec3b(0, 0, 0);
    //}

    //return image.at<cv::Vec3b>(y, x);
//}

cv::Vec3b SampleBilinear8UC3Fast(const cv::Mat& img, float u, float v)
{
    if (img.empty() || img.type() != CV_8UC3) {
        return cv::Vec3b(0, 0, 0);
    }

    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if (x0 < 0 || y0 < 0 || x1 >= img.cols || y1 >= img.rows) {
        return cv::Vec3b(0, 0, 0);
    }

    const float dx = u - static_cast<float>(x0);
    const float dy = v - static_cast<float>(y0);

    const cv::Vec3b* row0 = img.ptr<cv::Vec3b>(y0);
    const cv::Vec3b* row1 = img.ptr<cv::Vec3b>(y1);

    const cv::Vec3b c00 = row0[x0];
    const cv::Vec3b c10 = row0[x1];
    const cv::Vec3b c01 = row1[x0];
    const cv::Vec3b c11 = row1[x1];

    cv::Vec3b out{};

    for (int c = 0; c < 3; ++c) {
        const float top =
            static_cast<float>(c00[c]) * (1.0f - dx) +
            static_cast<float>(c10[c]) * dx;

        const float bottom =
            static_cast<float>(c01[c]) * (1.0f - dx) +
            static_cast<float>(c11[c]) * dx;

        const float value = top * (1.0f - dy) + bottom * dy;

        out[c] = static_cast<std::uint8_t>(
            std::clamp(value, 0.0f, 255.0f));
    }

    return out;
}

} // namespace

SphereStitcher::SphereStitcher(std::vector<CameraView> cameras,
                               SphereStitchConfig config,
                               const RigData<3>& rig)
    : cameras_(std::move(cameras)),
      config_(config),
      projection_(config.outputWidth, config.outputHeight),
      rig_(rig)
{
    precomputeWorldRays();

    faceToCameraIndex_.assign(rig_.faces.size(), -1);

    for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        const auto& cam = cameras_[static_cast<std::size_t>(i)];
        const int face = cam.faceIndex;

        if (face < 0 || face >= static_cast<int>(faceToCameraIndex_.size())) {
            continue;
        }

        // Only the non-overlap stream is the owning stream for a face.
        if (cam.localStreamIndex != 0) {
            continue;
        }

        const int existingIndex =
            faceToCameraIndex_[static_cast<std::size_t>(face)];

        if (existingIndex != -1) {
            std::cerr << "[SphereStitcher] warning: duplicate non-overlap stream for face "
                      << face
                      << " existing=" << existingIndex
                      << " new=" << i
                      << " (keeping existing mapping)"
                      << '\n';
            continue;
        }

        faceToCameraIndex_[static_cast<std::size_t>(face)] = i;
    }

    precomputeProjectionOnlyMap();
}

bool SphereStitcher::IsInsideMask(const cv::Mat& mask, const cv::Point2d& uv)
{
    if (mask.empty()) {
        return false;
    }

    const int x = static_cast<int>(std::floor(uv.x));
    const int y = static_cast<int>(std::floor(uv.y));

    if (x < 0 || x >= mask.cols || y < 0 || y >= mask.rows) {
        return false;
    }

    return mask.at<std::uint8_t>(y, x) != 0;
}

std::size_t SphereStitcher::rayIndex(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(config_.outputWidth) +
           static_cast<std::size_t>(x);
}

void SphereStitcher::precomputeWorldRays() {
    worldRays_.resize(
        static_cast<std::size_t>(config_.outputWidth) *
        static_cast<std::size_t>(config_.outputHeight));

    for (int y = 0; y < config_.outputHeight; ++y) {
        for (int x = 0; x < config_.outputWidth; ++x) {
            worldRays_[rayIndex(x, y)] = projection_.pixelToRay(x, y).cast<float>();
        }
    }
}

void SphereStitcher::precomputeProjectionOnlyMap()
{
    projectionOnlyMap_.clear();

    const std::size_t pixelCount =
        static_cast<std::size_t>(config_.outputWidth) *
        static_cast<std::size_t>(config_.outputHeight);

    projectionOnlyMap_.resize(pixelCount);

    for (int y = 0; y < config_.outputHeight; ++y) {
        for (int x = 0; x < config_.outputWidth; ++x) {
            const std::size_t idx = rayIndex(x, y);

            ProjectionMapEntry entry{};
            entry.cameraIndex = -1;

            const Eigen::Vector3f& rayWorldF = worldRays_[idx];
            const int owningFace = FindOwningFace(rayWorldF);

            if (owningFace < 0 ||
                owningFace >= static_cast<int>(faceToCameraIndex_.size())) {
                projectionOnlyMap_[idx] = entry;
                continue;
            }

            const int camIndex =
                faceToCameraIndex_[static_cast<std::size_t>(owningFace)];

            if (camIndex < 0 ||
                camIndex >= static_cast<int>(cameras_.size())) {
                projectionOnlyMap_[idx] = entry;
                continue;
            }

            const auto& cam = cameras_[static_cast<std::size_t>(camIndex)];

            if (cam.localStreamIndex != 0) {
                projectionOnlyMap_[idx] = entry;
                continue;
            }

            const Eigen::Vector3d rayWorld =
                rayWorldF.cast<double>().normalized();

            const Eigen::Vector3d rayCam =
                cam.Rcw.transpose() * rayWorld;

            if (rayCam.z() <= 0.0) {
                projectionOnlyMap_[idx] = entry;
                continue;
            }

            const ProjectionResult proj = cam.model->project(rayCam);

            if (!proj.valid) {
                projectionOnlyMap_[idx] = entry;
                continue;
            }

            if (!cam.sensorValidMask.empty()) {
                const int sx = static_cast<int>(std::floor(proj.uv.x));
                const int sy = static_cast<int>(std::floor(proj.uv.y));

                if (sx < 0 || sy < 0 ||
                    sx >= cam.sensorValidMask.cols ||
                    sy >= cam.sensorValidMask.rows) {
                    projectionOnlyMap_[idx] = entry;
                    continue;
                }

                if (cam.sensorValidMask.at<std::uint8_t>(sy, sx) == 0) {
                    projectionOnlyMap_[idx] = entry;
                    continue;
                }
            }

            entry.cameraIndex = camIndex;
            entry.u = static_cast<float>(proj.uv.x);
            entry.v = static_cast<float>(proj.uv.y);

            projectionOnlyMap_[idx] = entry;
        }
    }
}

cv::Mat SphereStitcher::stitch() const {
    return stitch(nullptr);
}

int SphereStitcher::FindOwningFace(const Eigen::Vector3f& ray_world_f) const
{
    const Eigen::Vector3d ray_world = ray_world_f.cast<double>().normalized();

    for (int f = 0; f < static_cast<int>(rig_.faces.size()); ++f) {
        const RigFace<3>& face = rig_.faces[static_cast<std::size_t>(f)];
        if (spherical::IsRayInsideSphericalFace(ray_world, face, rig_.vertices)) {
            return f;
        }
    }

    return -1;
}



cv::Mat SphereStitcher::stitch(cv::Mat* validMask) const
{
    cv::Mat out(config_.outputHeight,
                config_.outputWidth,
                CV_8UC3,
                cv::Scalar(0, 0, 0));

    cv::Mat localValidMask;
    if (validMask != nullptr) {
        localValidMask = cv::Mat(config_.outputHeight,
                                 config_.outputWidth,
                                 CV_8UC1,
                                 cv::Scalar(0));
    }

    static bool wroteContributionDebug = false;
    cv::Mat contributionDebug;
    if (!wroteContributionDebug) {
        contributionDebug = cv::Mat(config_.outputHeight,
                                    config_.outputWidth,
                                    CV_8UC1,
                                    cv::Scalar(0));
    }

    for (int y = 0; y < config_.outputHeight; ++y) {
        for (int x = 0; x < config_.outputWidth; ++x) {
            const Eigen::Vector3f& ray_world = worldRays_[rayIndex(x, y)];
            const std::vector<Contribution> contributions =
                gatherContributions(ray_world);

            if (!wroteContributionDebug) {
                contributionDebug.at<std::uint8_t>(y, x) =
                    static_cast<std::uint8_t>(
                        std::min<int>(255,
                                      static_cast<int>(contributions.size()) * 80));
            }

            if (contributions.empty()) {
                out.at<cv::Vec3b>(y, x) = config_.backgroundColor;
                continue;
            }

            if (validMask != nullptr) {
                localValidMask.at<std::uint8_t>(y, x) = 255;
            }

            out.at<cv::Vec3b>(y, x) = resolvePixel(contributions);
        }
    }

    if (!wroteContributionDebug) {
        cv::imwrite("/tmp/contribution_debug.png", contributionDebug);
        wroteContributionDebug = true;
    }

    if (validMask != nullptr) {
        *validMask = std::move(localValidMask);
    }

    return out;
}

//cv::Mat SphereStitcher::stitch(cv::Mat* validMask) const
//{
    //cv::Mat out(config_.outputHeight,
                //config_.outputWidth,
                //CV_8UC3,
                //cv::Scalar(0, 0, 0));

    //cv::Mat localValidMask;
    //if (validMask != nullptr) {
        //localValidMask = cv::Mat(config_.outputHeight,
                                 //config_.outputWidth,
                                 //CV_8UC1,
                                 //cv::Scalar(0));
    //}

    //const int centerX = config_.outputWidth / 2;
    //const int centerY = config_.outputHeight / 2;

    //for (int y = 0; y < config_.outputHeight; ++y) {
        //for (int x = 0; x < config_.outputWidth; ++x) {
            //const Eigen::Vector3f& ray_world = worldRays_[rayIndex(x, y)];
            //const std::vector<Contribution> contributions = gatherContributions(ray_world);

            //if (!contributions.empty() && validMask != nullptr) {
                //localValidMask.at<std::uint8_t>(y, x) = 255;
            //}

            //if (x == centerX && y == centerY) {
                //std::cout << "stitch center ray_world=("
                          //<< ray_world.x() << ", "
                          //<< ray_world.y() << ", "
                          //<< ray_world.z() << ")\n";

                //std::cout << "center contributions=" << contributions.size() << '\n';

                //if (!contributions.empty()) {
                    //const auto& c = contributions[0];
                    //const auto& cam = cameras_[static_cast<std::size_t>(c.cameraIndex)];

                    //std::cout << "center cameraIndex=" << c.cameraIndex
                              //<< " faceIndex=" << cam.faceIndex
                              //<< " localStreamIndex=" << cam.localStreamIndex
                              //<< " neighborFaceIndex=" << cam.neighborFaceIndex
                              //<< " edgeIndex=" << cam.edgeIndex
                              //<< " uv=(" << c.uv.x << ", " << c.uv.y << ")\n";
                //}
            //}

            //if (contributions.empty()) {
                //out.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                //continue;
            //}

            //const auto& c = contributions[0];
            //const auto& cam = cameras_[static_cast<std::size_t>(c.cameraIndex)];

            //if (cam.moduleIndex == 0) {
                //out.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);   // red
            //} else if (cam.moduleIndex == 1) {
                //out.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);   // green
            //} else {
                //out.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);   // blue
            //}
        //}
    //}

    //if (validMask != nullptr) {
        //*validMask = std::move(localValidMask);
    //}

    //return out;
//}
//cv::Mat SphereStitcher::stitch() const {
    //cv::Mat out(config_.outputHeight,
                //config_.outputWidth,
                //CV_8UC3,
                //config_.backgroundColor);

////#pragma omp parallel for
    //for (int y = 0; y < config_.outputHeight; ++y) {
        //for (int x = 0; x < config_.outputWidth; ++x) {
            //const Eigen::Vector3f& ray_world = worldRays_[rayIndex(x, y)];
            //const auto contributions = gatherContributions(ray_world);
            //out.at<cv::Vec3b>(y, x) = resolvePixel(contributions);
        //}
    //}

    //return out;
//}

std::vector<Contribution>
SphereStitcher::gatherContributions(const Eigen::Vector3f& ray_world_f) const
{
    const Eigen::Vector3d ray_world = ray_world_f.cast<double>().normalized();

    std::vector<Contribution> out;
    out.reserve(cameras_.size());

    const int owningFace = FindOwningFace(ray_world_f);
    if (owningFace < 0) {
        return out;
    }

    auto projectIfValid = [&](const CameraView& cam,
                              cv::Point2d& uvOut,
                              Eigen::Vector3d& rayCamOut) -> bool
    {
        rayCamOut = cam.Rcw.transpose() * ray_world;

        if (rayCamOut.z() <= 0.0) {
            return false;
        }

        const ProjectionResult proj = cam.model->project(rayCamOut);
        if (!proj.valid) {
            return false;
        }

        if (!cam.sensorValidMask.empty()) {
            const int x = static_cast<int>(std::floor(proj.uv.x));
            const int y = static_cast<int>(std::floor(proj.uv.y));

            if (x < 0 || x >= cam.sensorValidMask.cols ||
                y < 0 || y >= cam.sensorValidMask.rows) {
                return false;
            }

            if (cam.sensorValidMask.at<std::uint8_t>(y, x) == 0) {
                return false;
            }
        }

        uvOut = proj.uv;
        return true;
    };

    auto appendContribution = [&](int camIndex,
                                  const cv::Point2d& uv,
                                  const Eigen::Vector3d& ray_cam,
                                  bool fromNonOverlap)
    {
        Contribution c;
        c.cameraIndex = camIndex;
        c.uv = uv;
        c.color = SampleBilinear(
            cameras_[static_cast<std::size_t>(camIndex)].image,
            uv);
        c.weight = OpticalWeight(ray_cam, config_.blendPower);
        c.fromNonOverlap = fromNonOverlap;

        out.push_back(std::move(c));
    };

    // Projection-only:
    // Use the owning face's mapped non-overlap stream directly.
    if (config_.mode == StitchMode::ProjectionOnly) {
        if (owningFace >= static_cast<int>(faceToCameraIndex_.size())) {
            return out;
        }

        const int camIndex =
            faceToCameraIndex_[static_cast<std::size_t>(owningFace)];

        if (camIndex < 0 ||
            camIndex >= static_cast<int>(cameras_.size())) {
            return out;
        }

        const CameraView& cam =
            cameras_[static_cast<std::size_t>(camIndex)];

        if (cam.localStreamIndex != 0) {
            return out;
        }

        cv::Point2d uv;
        Eigen::Vector3d ray_cam;

        if (!projectIfValid(cam, uv, ray_cam)) {
            return out;
        }

        appendContribution(camIndex, uv, ray_cam, true);
        return out;
    }

    // Blend mode:
    // 1) Use the owning face's non-overlap stream if the ray lands
    //    inside its non-overlap mask.
    for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        const CameraView& cam = cameras_[static_cast<std::size_t>(i)];

        if (cam.faceIndex != owningFace) {
            continue;
        }

        if (cam.localStreamIndex != 0) {
            continue;
        }

        cv::Point2d uv;
        Eigen::Vector3d ray_cam;

        if (!projectIfValid(cam, uv, ray_cam)) {
            continue;
        }

        const bool inNonOverlap =
            !cam.maskNonOverlap.empty() &&
            IsInsideMask(cam.maskNonOverlap, uv);

        if (!inNonOverlap) {
            continue;
        }

        appendContribution(i, uv, ray_cam, true);
        return out;
    }

    // Debug blend mode:
    // No non-overlap hit. Try every overlap stream connected to the owning face.
    //
    // This intentionally bypasses FindSeamLocalEdgeForRay() and edgeIndex pairing.
    // If this removes the black gaps in contribution_debug.png, the strict seam
    // edge selection is too restrictive or choosing the wrong edge.
    //
    // If the gaps remain, the overlap masks do not cover those projected UVs.
    for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        const CameraView& cam = cameras_[static_cast<std::size_t>(i)];

        if (cam.localStreamIndex <= 0) {
            continue;
        }

        const bool relatedToOwningFace =
            cam.faceIndex == owningFace ||
            cam.neighborFaceIndex == owningFace;

        if (!relatedToOwningFace) {
            continue;
        }

        cv::Point2d uv;
        Eigen::Vector3d ray_cam;

        if (!projectIfValid(cam, uv, ray_cam)) {
            continue;
        }

        const bool inOverlap =
            !cam.maskOverlap.empty() &&
            IsInsideMask(cam.maskOverlap, uv);

        if (!inOverlap) {
            continue;
        }

        appendContribution(i, uv, ray_cam, false);
    }

    return out;
}

//std::vector<Contribution>
//SphereStitcher::gatherContributions(const Eigen::Vector3f& ray_world) const {
    //std::vector<Contribution> out;
    //out.reserve(cameras_.size());

    //static std::atomic<int> printedRay{0};
    //static std::atomic<int> printedSummary{0};

    //int rejectedBehind = 0;
    //int rejectedProject = 0;
    //int rejectedMask = 0;
    //int accepted = 0;

    //for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        //const auto& cam = cameras_[i];

        //Eigen::Vector3f ray_cam =
            //cam.Rcw.cast<float>().transpose() * ray_world;

        //// Flip camera forward axis to match the pinhole model convention
        ////ray_cam.z() = -ray_cam.z();

        ////if (ray_cam.z() <= 0.0f) {
            ////++rejectedBehind;
            ////continue;
        ////}

        //const ProjectionResult proj = cam.model->project(ray_cam.cast<double>());

        ////if (!proj.valid) {
            ////++rejectedProject;
            ////continue;
        ////}
        
        //const double u = proj.uv.x;
        //const double v = proj.uv.y;
        
        //if (u < 0.0 || u >= static_cast<double>(cam.image.cols) ||
            //v < 0.0 || v >= static_cast<double>(cam.image.rows)) {
            //++rejectedProject;
            //continue;
        //}

        //const bool inNonOverlap = IsInsideMask(cam.maskNonOverlap, proj.uv);
        //const bool inOverlap    = IsInsideMask(cam.maskOverlap, proj.uv);

        //if (!inNonOverlap && !inOverlap) {
            //++rejectedMask;
            //continue;
        //}

        //Contribution c;
        //c.cameraIndex = i;
        //c.uv = proj.uv;
        //c.color = SampleBilinear(cam.image, proj.uv);
        //c.weight = OpticalWeight(ray_cam.cast<double>(), config_.blendPower);
        //c.fromNonOverlap = inNonOverlap;

        //out.push_back(c);
        //++accepted;
    //}

    //if (printedSummary.fetch_add(1) < 20) {
        //std::cout
            //<< "behind=" << rejectedBehind
            //<< " project=" << rejectedProject
            //<< " mask=" << rejectedMask
            //<< " accepted=" << accepted
            //<< '\n';
    //}

    //return out;
//}

//std::vector<Contribution>
//SphereStitcher::gatherContributions(const Eigen::Vector3f& ray_world) const {
    //std::vector<Contribution> out;
    //out.reserve(cameras_.size());

    //for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        //const auto& cam = cameras_[i];

		//const Eigen::Vector3f ray_cam =
		    //cam.Rcw.cast<float>().transpose() * ray_world;
		
		//Eigen::Vector3f ray_cam_fixed(
		    //ray_cam.x(),
		    //ray_cam.y(),
		    //-ray_cam.z()   // flip forward axis
		//);

        ////if (ray_cam.z() <= 0.0f) {
        //if (ray_cam.z() >= 0.0){
            //continue;
        //}

        //const ProjectionResult proj = cam.model->project(ray_cam.cast<double>());
        //if (!proj.valid) {
            //continue;
        //}

        //const bool inNonOverlap = IsInsideMask(cam.maskNonOverlap, proj.uv);
        //const bool inOverlap    = IsInsideMask(cam.maskOverlap, proj.uv);

        //if (!inNonOverlap && !inOverlap) {
            //continue;
        //}

        //Contribution c;
        //c.cameraIndex = i;
        //c.uv = proj.uv;
        //c.color = SampleBilinear(cam.image, proj.uv);
        //c.weight = OpticalWeight(ray_cam.cast<double>(), config_.blendPower);
        //c.fromNonOverlap = inNonOverlap;

        //out.push_back(c);
    //}

    //return out;
//}

cv::Vec3b SphereStitcher::resolvePixel(
    const std::vector<Contribution>& contributions) const
{
    if (contributions.empty()) {
        return config_.backgroundColor;
    }

    if (config_.mode == StitchMode::ProjectionOnly) {
        const cv::Vec3d& color = contributions.front().color;

        return cv::Vec3b(
            static_cast<std::uint8_t>(std::clamp(color[0], 0.0, 255.0)),
            static_cast<std::uint8_t>(std::clamp(color[1], 0.0, 255.0)),
            static_cast<std::uint8_t>(std::clamp(color[2], 0.0, 255.0))
        );
    }

    bool hasNonOverlap = false;
    for (const auto& c : contributions) {
        if (c.fromNonOverlap) {
            hasNonOverlap = true;
            break;
        }
    }

    cv::Vec3d accum(0.0, 0.0, 0.0);
    double weightSum = 0.0;

    for (const auto& c : contributions) {
        if (hasNonOverlap && !c.fromNonOverlap) {
            continue;
        }

        accum += c.weight * c.color;
        weightSum += c.weight;
    }

    if (weightSum <= 0.0) {
        return config_.backgroundColor;
    }

    const cv::Vec3d color = accum / weightSum;

    return cv::Vec3b(
        static_cast<std::uint8_t>(std::clamp(color[0], 0.0, 255.0)),
        static_cast<std::uint8_t>(std::clamp(color[1], 0.0, 255.0)),
        static_cast<std::uint8_t>(std::clamp(color[2], 0.0, 255.0))
    );
}

//cv::Vec3b SphereStitcher::resolvePixel(
    //const std::vector<Contribution>& contributions) const
//{
    //if (contributions.empty()) {
        //return config_.backgroundColor;
    //}

    //bool hasNonOverlap = false;
    //for (const auto& c : contributions) {
        //if (c.fromNonOverlap) {
            //hasNonOverlap = true;
            //break;
        //}
    //}

    //cv::Vec3d accum(0.0, 0.0, 0.0);
    //double weightSum = 0.0;

    //for (const auto& c : contributions) {
        //if (hasNonOverlap && !c.fromNonOverlap) {
            //continue;
        //}

        //accum += c.weight * c.color;
        //weightSum += c.weight;
    //}

    //if (weightSum <= 0.0) {
        //return config_.backgroundColor;
    //}

    //const cv::Vec3d color = accum / weightSum;

    //return cv::Vec3b(
        //static_cast<std::uint8_t>(std::clamp(color[0], 0.0, 255.0)),
        //static_cast<std::uint8_t>(std::clamp(color[1], 0.0, 255.0)),
        //static_cast<std::uint8_t>(std::clamp(color[2], 0.0, 255.0))
    //);
//}

cv::Vec3b SphereStitcher::renderPixel(int x, int y, std::uint8_t* valid) const {
    const Eigen::Vector3f& ray_world = worldRays_[rayIndex(x, y)];
    const std::vector<Contribution> contributions = gatherContributions(ray_world);

    if (valid != nullptr) {
        *valid = contributions.empty() ? 0 : 255;
    }

    return resolvePixel(contributions);
}

cv::Mat SphereStitcher::stitchFisheye(cv::Mat* validMask) const
{
    cv::Mat out(config_.outputHeight,
                config_.outputWidth,
                CV_8UC3,
                config_.backgroundColor);

    cv::Mat accum(config_.outputHeight,
                  config_.outputWidth,
                  CV_32FC3,
                  cv::Scalar(0.0f, 0.0f, 0.0f));

    cv::Mat weightSum(config_.outputHeight,
                      config_.outputWidth,
                      CV_32FC1,
                      cv::Scalar(0.0f));

    cv::Mat localValidMask;
    if (validMask != nullptr) {
        localValidMask = cv::Mat(config_.outputHeight,
                                 config_.outputWidth,
                                 CV_8UC1,
                                 cv::Scalar(0));
    }

    for (const auto& cam : cameras_) {
        for (int y = 0; y < cam.image.rows; ++y) {
            for (int x = 0; x < cam.image.cols; ++x) {
                const cv::Point2d uv(static_cast<double>(x),
                                     static_cast<double>(y));

                //const bool inNonOverlap = IsInsideMask(cam.maskNonOverlap, uv);
                //const bool inOverlap    = IsInsideMask(cam.maskOverlap, uv);

                //if (!inNonOverlap && !inOverlap) {
                    //continue;
                //}

				Eigen::Vector3d ray_cam;
				if (!cam.model->unproject(uv, ray_cam)) {
				    continue;
				}
                const Eigen::Vector3d ray_world = cam.Rcw * ray_cam;

                const Eigen::Vector2d pano =
                    WorldRayToEquirectPixel(ray_world,
                                            config_.outputWidth,
                                            config_.outputHeight);

                const int px = static_cast<int>(std::round(pano.x()));
                const int py = static_cast<int>(std::round(pano.y()));

                if (px < 0 || py < 0 ||
                    px >= config_.outputWidth || py >= config_.outputHeight) {
                    continue;
                }

                const cv::Vec3b color = cam.image.at<cv::Vec3b>(y, x);

                double weight = 1.0;
                //if (inOverlap) {
                    //weight = std::pow(std::max(0.0, ray_cam.z()), config_.blendPower);
                    //if (weight <= 0.0) {
                        //weight = 1.0;
                    //}
                //}

                cv::Vec3f& dst = accum.at<cv::Vec3f>(py, px);
                dst[0] += static_cast<float>(weight * color[0]);
                dst[1] += static_cast<float>(weight * color[1]);
                dst[2] += static_cast<float>(weight * color[2]);

                weightSum.at<float>(py, px) += static_cast<float>(weight);

                if (validMask != nullptr) {
                    localValidMask.at<std::uint8_t>(py, px) = 255;
                }
            }
        }
    }

    for (int y = 0; y < config_.outputHeight; ++y) {
        for (int x = 0; x < config_.outputWidth; ++x) {
            const float w = weightSum.at<float>(y, x);
            if (w <= 0.0f) {
                continue;
            }

            const cv::Vec3f sum = accum.at<cv::Vec3f>(y, x) / w;
            
            // Normalize to [0,1]
            const cv::Vec3f linear(
                std::clamp(sum[0] / 255.0f, 0.0f, 1.0f),
                std::clamp(sum[1] / 255.0f, 0.0f, 1.0f),
                std::clamp(sum[2] / 255.0f, 0.0f, 1.0f)
            );
            
            // Gamma correction
            constexpr float gamma = 1.0f / 2.2f;
            const cv::Vec3f corrected(
                std::pow(linear[0], gamma),
                std::pow(linear[1], gamma),
                std::pow(linear[2], gamma)
            );
            
            // Back to [0,255]
            out.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<std::uint8_t>(std::clamp(corrected[0] * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(std::clamp(corrected[1] * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(std::clamp(corrected[2] * 255.0f, 0.0f, 255.0f))
            );
        }
    }

    if (validMask != nullptr) {
        *validMask = std::move(localValidMask);
    }

    return out;
}

cv::Mat SphereStitcher::makePolygonMask(int width,
                        int height,
                        const std::vector<std::vector<cv::Point>>& polygons)
{
    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));
    cv::fillPoly(mask, polygons, cv::Scalar(255));
    return mask;
}

int SphereStitcher::FindSeamLocalEdgeForRay(const Eigen::Vector3d& ray_world,
                                            int faceIndex) const
{
    if (faceIndex < 0 ||
        faceIndex >= static_cast<int>(rig_.faces.size())) {
        return -1;
    }

    const RigFace<3>& face =
        rig_.faces[static_cast<std::size_t>(faceIndex)];

    const Eigen::Vector3d ray = ray_world.normalized();

    double bestAbsDot = std::numeric_limits<double>::infinity();
    int bestLocalEdge = -1;

    for (int localEdge = 0; localEdge < 3; ++localEdge) {
        const std::size_t i0 =
            face.indices[static_cast<std::size_t>(localEdge)];

        const std::size_t i1 =
            face.indices[static_cast<std::size_t>((localEdge + 1) % 3)];

        const Eigen::Vector3d a =
            rig_.vertices[static_cast<std::size_t>(i0)].normalized();

        const Eigen::Vector3d b =
            rig_.vertices[static_cast<std::size_t>(i1)].normalized();

        Eigen::Vector3d edgePlaneNormal = a.cross(b);

        const double n = edgePlaneNormal.norm();
        if (n <= 1e-12) {
            continue;
        }

        edgePlaneNormal /= n;

        const double absDot = std::abs(edgePlaneNormal.dot(ray));

        if (absDot < bestAbsDot) {
            bestAbsDot = absDot;
            bestLocalEdge = localEdge;
        }
    }

    return bestLocalEdge;
}

//int SphereStitcher::FindSeamLocalEdgeForRay(const Eigen::Vector3d& ray_world,
                                            //int faceIndex) const
//{
    //if (faceIndex < 0 ||
        //faceIndex >= static_cast<int>(rig_.faces.size())) {
        //return -1;
    //}

    //const RigFace<3>& face = rig_.faces[static_cast<std::size_t>(faceIndex)];

    //double bestAbsDot = std::numeric_limits<double>::infinity();
    //int bestLocalEdge = -1;

    //for (int i = 0; i < 3; ++i) {
        //const int j = (i + 1) % 3;

        //const Eigen::Vector3d& a =
            //rig_.vertices[face.indices[static_cast<std::size_t>(i)]];
        //const Eigen::Vector3d& b =
            //rig_.vertices[face.indices[static_cast<std::size_t>(j)]];

        //Eigen::Vector3d edgePlaneNormal = a.cross(b);
        //const double norm = edgePlaneNormal.norm();
        //if (norm < 1e-12) {
            //continue;
        //}
        //edgePlaneNormal /= norm;

        //if (edgePlaneNormal.dot(face.lookDir) < 0.0) {
            //edgePlaneNormal = -edgePlaneNormal;
        //}

        //const double absD = std::abs(edgePlaneNormal.dot(ray_world));
        //if (absD < bestAbsDot) {
            //bestAbsDot = absD;
            //bestLocalEdge = i;
        //}
    //}

    //return bestLocalEdge;
//}

void SphereStitcher::setCameras(const std::vector<CameraView>& cameras)
{
    if (cameras.size() != cameras_.size()) {
        throw std::runtime_error(
            "SphereStitcher::setCameras: camera count changed");
    }

    cameras_ = cameras;
}

cv::Mat SphereStitcher::stitchProjectionOnlyFast(cv::Mat* validMask) const
{
    /*
     * Fast path for StitchMode::ProjectionOnly.
     *
     * The expensive geometry has already been precomputed into
     * projectionOnlyMap_:
     *
     *   output pixel -> source camera index + source uv
     *
     * Per frame, this function only samples the current camera images.
     */

    static std::once_flag printedFastPathOnce;

    std::call_once(printedFastPathOnce, [&]() {
        size_t validEntries = 0;

        for (const auto& e : projectionOnlyMap_) {
            if (e.cameraIndex >= 0) {
                ++validEntries;
            }
        }

        std::cout << "[SphereStitcher] using ProjectionOnly fast path"
                  << " map_entries=" << projectionOnlyMap_.size()
                  << " valid_entries=" << validEntries
                  << " outputWidth=" << config_.outputWidth
                  << " outputHeight=" << config_.outputHeight
                  << '\n';
    });

    cv::Mat out(config_.outputHeight,
                config_.outputWidth,
                CV_8UC3,
                config_.backgroundColor);

    cv::Mat localValidMask;

    if (validMask != nullptr) {
        localValidMask = cv::Mat(config_.outputHeight,
                                 config_.outputWidth,
                                 CV_8UC1,
                                 cv::Scalar(0));
    }

    const std::size_t expectedMapSize =
        static_cast<std::size_t>(config_.outputWidth) *
        static_cast<std::size_t>(config_.outputHeight);

    if (projectionOnlyMap_.size() != expectedMapSize) {
        std::cerr << "[SphereStitcher::stitchProjectionOnlyFast] bad projection map size: "
                  << "projectionOnlyMap_.size()=" << projectionOnlyMap_.size()
                  << " expected=" << expectedMapSize
                  << '\n';

        if (validMask != nullptr) {
            *validMask = std::move(localValidMask);
        }

        return out;
    }

    if (cameras_.empty()) {
        if (validMask != nullptr) {
            *validMask = std::move(localValidMask);
        }

        return out;
    }

    /*
     * Parallelizing over rows is safe:
     * - each thread writes a different output row
     * - each thread writes a different valid-mask row
     * - camera images are read-only
     */
#pragma omp parallel for schedule(static)
    for (int y = 0; y < config_.outputHeight; ++y) {
        cv::Vec3b* dstRow = out.ptr<cv::Vec3b>(y);

        std::uint8_t* maskRow = nullptr;
        if (validMask != nullptr) {
            maskRow = localValidMask.ptr<std::uint8_t>(y);
        }

        const std::size_t rowBase =
            static_cast<std::size_t>(y) *
            static_cast<std::size_t>(config_.outputWidth);

        for (int x = 0; x < config_.outputWidth; ++x) {
            const std::size_t idx = rowBase + static_cast<std::size_t>(x);

            const ProjectionMapEntry& entry = projectionOnlyMap_[idx];

            if (entry.cameraIndex < 0 ||
                entry.cameraIndex >= static_cast<int>(cameras_.size())) {
                continue;
            }

            const CameraView& cam =
                cameras_[static_cast<std::size_t>(entry.cameraIndex)];

            if (cam.image.empty() || cam.image.type() != CV_8UC3) {
                continue;
            }

            const cv::Vec3b color =
                SampleBilinear8UC3Fast(cam.image, entry.u, entry.v);

            dstRow[x] = color;

            if (maskRow != nullptr) {
                /*
                 * Mark valid only if the sample is inside bounds.
                 * SampleBilinear8UC3Fast returns black on invalid bounds,
                 * so repeat a cheap bounds check here for the mask.
                 */
                const int sx = static_cast<int>(std::floor(entry.u));
                const int sy = static_cast<int>(std::floor(entry.v));

                if (sx >= 0 && sy >= 0 &&
                    sx + 1 < cam.image.cols &&
                    sy + 1 < cam.image.rows) {
                    maskRow[x] = 255;
                }
            }
        }
    }

    if (validMask != nullptr) {
        *validMask = std::move(localValidMask);
    }

    return out;
}
