#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <cmath>
#include <iostream>

#include "DASPi-i-camera-model.h"
#include "DASPi-projection-result.h"

namespace DASPi{
class FisheyeCameraModel : public ICameraModel {
public:
    FisheyeCameraModel(double fx,
                       double fy,
                       double cx,
                       double cy,
                       double maxImageRadiusPx,
                       cv::Size size)
        : fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          maxImageRadiusPx_(maxImageRadiusPx),
          size_(size) {}
          
    //void DebugCanonicalProjection(const ICameraModel& model)
    //{
        //const std::vector<std::pair<std::string, Eigen::Vector3d>> rays = {
            //{"forward", Eigen::Vector3d(0.0, 0.0, 1.0)},
            //{"right",   Eigen::Vector3d(1.0, 0.0, 0.0)},
            //{"left",    Eigen::Vector3d(-1.0, 0.0, 0.0)},
            //{"up",      Eigen::Vector3d(0.0, 1.0, 0.0)},
            //{"down",    Eigen::Vector3d(0.0,-1.0, 0.0)},
        //};
    
        //std::cout << "\n[DebugCanonicalProjection]\n";
    
        //for (const auto& [name, ray] : rays) {
            //const ProjectionResult proj = model.project(ray);
            //std::cout << name
                      //<< " valid=" << proj.valid
                      //<< " uv=(" << proj.uv.x << ", " << proj.uv.y << ")\n";
        //}
    
        //std::cout << std::endl;
    //}

    void DebugProjectUnprojectConsistency(const ICameraModel& model)
    {
        const std::vector<Eigen::Vector3d> rays = {
            Eigen::Vector3d(0.0, 0.0, 1.0).normalized(),
            Eigen::Vector3d(0.2, 0.0, 0.98).normalized(),
            Eigen::Vector3d(-0.2, 0.0, 0.98).normalized(),
            Eigen::Vector3d(0.0, 0.2, 0.98).normalized(),
            Eigen::Vector3d(0.0, -0.2, 0.98).normalized(),
        };
    
        for (const auto& ray0 : rays) {
            const ProjectionResult p = model.project(ray0);
            if (!p.valid) {
                std::cout << "project invalid for ray "
                          << ray0.transpose() << '\n';
                continue;
            }
    
            Eigen::Vector3d ray1;
            const bool ok = model.unproject(p.uv, ray1);
            if (!ok) {
                std::cout << "unproject failed for uv "
                          << p.uv.x << ", " << p.uv.y << '\n';
                continue;
            }
    
            std::cout << "ray0=" << ray0.transpose()
                      << " uv=(" << p.uv.x << "," << p.uv.y << ")"
                      << " ray1=" << ray1.transpose()
                      << " dot=" << ray0.dot(ray1)
                      << '\n';
        }
    }
    
    void DebugCanonicalProjection(const ICameraModel& model)
    {
        const std::vector<std::pair<std::string, Eigen::Vector3d>> rays = {
            {"forward", {0, 0, 1}},
            {"right",   {1, 0, 0}},
            {"left",    {-1, 0, 0}},
            {"up",      {0, 1, 0}},
            {"down",    {0, -1, 0}},
        };
    
        for (const auto& [name, ray] : rays) {
            const ProjectionResult p = model.project(ray);
            std::cout << name
                      << " valid=" << p.valid
                      << " uv=(" << p.uv.x << ", " << p.uv.y << ")\n";
        }
    }

    ProjectionResult project(const Eigen::Vector3d& ray) const override {
        ProjectionResult result{};
        const Eigen::Vector3d r = ray.normalized();
    
        const double theta = std::acos(std::clamp(r.z(), -1.0, 1.0));
        const double sinTheta = std::sqrt(r.x() * r.x() + r.y() * r.y());
    
        double radial = 0.0;
        if (theta > 1e-12) {
            radial = 2.0 * std::sin(theta / 2.0);
        }
    
        const double scale = (sinTheta > 1e-12) ? (radial / sinTheta) : 1.0;
    
        const double xn = r.x() * scale;
        const double yn = r.y() * scale;
    
        const double u = fx_ * xn + cx_;
        const double v = fy_ * yn + cy_;
    
        result.uv = cv::Point2d(u, v);
    
        //const double du = u - cx_;
        //const double dv = v - cy_;
        //const double imageRadius = std::sqrt(du * du + dv * dv);
    
        //result.valid =
            //(u >= 0.0 && u < static_cast<double>(size_.width) &&
             //v >= 0.0 && v < static_cast<double>(size_.height) &&
             //imageRadius <= maxImageRadiusPx_);
        result.valid =
            (u >= 0.0 && u < static_cast<double>(size_.width) &&
             v >= 0.0 && v < static_cast<double>(size_.height));
    
        result.valid = true;//troubleshooting
    
        return result;
    }
    
     bool unproject(const cv::Point2d& uv, Eigen::Vector3d& ray) const override {
        const double du = uv.x - cx_;
        const double dv = uv.y - cy_;
        const double imageRadius = std::sqrt(du * du + dv * dv);
    
        if (imageRadius > maxImageRadiusPx_) {
            return false;
        }
        
        const double x = (uv.x - cx_) / fx_;
        const double y = (uv.y - cy_) / fy_;
    
        const double r = std::sqrt(x * x + y * y);
    
        if (r < 1e-12) {
            ray = Eigen::Vector3d(0.0, 0.0, 1.0);
            return true;
        }
    
        const double theta = 2.0 * std::asin(std::clamp(r / 2.0, -1.0, 1.0));
        const double sinTheta = std::sin(theta);
        const double cosTheta = std::cos(theta);
        const double scale = sinTheta / r;
    
        ray = Eigen::Vector3d(
            x * scale,
            y * scale,
            cosTheta
        ).normalized();
    
        return true;
    }

    cv::Size imageSize() const override {
        return size_;
    }

private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double maxImageRadiusPx_;
    cv::Size size_;
};
}//DASPi
