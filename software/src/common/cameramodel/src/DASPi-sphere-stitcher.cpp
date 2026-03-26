// DASPi-sphere-stitcher.cpp
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "DASPi-sphere-stitcher.h"
#include "DASPi-spherical-math.h"

namespace {

void LogCenterDebugIfNeeded(int x,
                            int y,
                            int centerX,
                            int centerY,
                            const Eigen::Vector3f& ray_world,
                            const std::vector<Contribution>& contributions)
{
    if (x != centerX || y != centerY) {
        return;
    }

    std::cout << "stitch center ray_world=("
              << ray_world.x() << ", "
              << ray_world.y() << ", "
              << ray_world.z() << ")\n";

    std::cout << "center contributions=" << contributions.size() << '\n';

    if (!contributions.empty()) {
        std::cout << "center uv=("
                  << contributions[0].uv.x << ", "
                  << contributions[0].uv.y << ")\n";
    }
}

bool IsInsideMask(const cv::Mat& mask, const cv::Point2d& uv) {
    if (mask.empty()) {
        return false;
    }

    const int x = static_cast<int>(uv.x);
    const int y = static_cast<int>(uv.y);

    if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows) {
        return false;
    }

    return mask.at<std::uint8_t>(y, x) != 0;
}

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

} // namespace

SphereStitcher::SphereStitcher(std::vector<CameraView> cameras,
                               SphereStitchConfig config)
    : cameras_(std::move(cameras)),
      config_(config),
      projection_(config.outputWidth, config.outputHeight)
{
    precomputeWorldRays();
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
cv::Mat SphereStitcher::stitch() const {
    return stitch(nullptr);
}

cv::Mat SphereStitcher::stitch(cv::Mat* validMask) const {
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

    const int centerX = config_.outputWidth / 2;
    const int centerY = config_.outputHeight / 2;

    for (int y = 0; y < config_.outputHeight; ++y) {
        for (int x = 0; x < config_.outputWidth; ++x) {
            const Eigen::Vector3f& ray_world = worldRays_[rayIndex(x, y)];
            const std::vector<Contribution> contributions = gatherContributions(ray_world);

            if (!contributions.empty() && validMask != nullptr) {
                localValidMask.at<std::uint8_t>(y, x) = 255;
            }

			LogCenterDebugIfNeeded(x, y, centerX, centerY, ray_world, contributions);

            out.at<cv::Vec3b>(y, x) = resolvePixel(contributions);
        }
    }

    if (validMask != nullptr) {
        *validMask = std::move(localValidMask);
    }

    return out;
}
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
SphereStitcher::gatherContributions(const Eigen::Vector3f& ray_world) const {
    std::vector<Contribution> out;
    out.reserve(cameras_.size());

    static std::atomic<int> printedRay{0};
    static std::atomic<int> printedSummary{0};

    int rejectedBehind = 0;
    int rejectedProject = 0;
    int rejectedMask = 0;
    int accepted = 0;

    for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        const auto& cam = cameras_[i];

        Eigen::Vector3f ray_cam =
            cam.Rcw.cast<float>().transpose() * ray_world;

        // Flip camera forward axis to match the pinhole model convention
        //ray_cam.z() = -ray_cam.z();

        if (ray_cam.z() <= 0.0f) {
            ++rejectedBehind;
            continue;
        }

        const ProjectionResult proj = cam.model->project(ray_cam.cast<double>());

        if (!proj.valid) {
            ++rejectedProject;
            continue;
        }

        const bool inNonOverlap = IsInsideMask(cam.maskNonOverlap, proj.uv);
        const bool inOverlap    = IsInsideMask(cam.maskOverlap, proj.uv);

        if (!inNonOverlap && !inOverlap) {
            ++rejectedMask;
            continue;
        }

        Contribution c;
        c.cameraIndex = i;
        c.uv = proj.uv;
        c.color = SampleBilinear(cam.image, proj.uv);
        c.weight = OpticalWeight(ray_cam.cast<double>(), config_.blendPower);
        c.fromNonOverlap = inNonOverlap;

        out.push_back(c);
        ++accepted;
    }

    if (printedSummary.fetch_add(1) < 20) {
        std::cout
            << "behind=" << rejectedBehind
            << " project=" << rejectedProject
            << " mask=" << rejectedMask
            << " accepted=" << accepted
            << '\n';
    }

    return out;
}

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

                const bool inNonOverlap = IsInsideMask(cam.maskNonOverlap, uv);
                const bool inOverlap    = IsInsideMask(cam.maskOverlap, uv);

                if (!inNonOverlap && !inOverlap) {
                    continue;
                }

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
                if (inOverlap) {
                    weight = std::pow(std::max(0.0, ray_cam.z()), config_.blendPower);
                    if (weight <= 0.0) {
                        weight = 1.0;
                    }
                }

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

            out.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<std::uint8_t>(std::clamp(sum[0], 0.0f, 255.0f)),
                static_cast<std::uint8_t>(std::clamp(sum[1], 0.0f, 255.0f)),
                static_cast<std::uint8_t>(std::clamp(sum[2], 0.0f, 255.0f))
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
