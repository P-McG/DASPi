#include "DASPi-fisheye-camera-model.h"
#include <cmath>


//bool FisheyeCameraModel::unproject(const cv::Point2d& uv,
                                   //Eigen::Vector3d& ray) const
//{
    //const double x = (uv.x - cx_) / fx_;
    //const double y = (cy_ - uv.y) / fy_;

    //const double r = std::sqrt(x * x + y * y);

    //const double maxTheta = M_PI / 2.0;

    //if (r > maxTheta) {
        //return false; // 🔥 reject pixel
    //}

    //if (r < 1e-12) {
        //ray = Eigen::Vector3d(0,0,1);
        //return true;
    //}

    //const double theta = r;
    //const double sinTheta = std::sin(theta);
    //const double cosTheta = std::cos(theta);
    //const double scale = sinTheta / r;

    //ray = Eigen::Vector3d(
        //x * scale,
        //y * scale,
        //cosTheta
    //).normalized();

    //return true;
//}
        
//ProjectionResult FisheyeCameraModel::project(const Eigen::Vector3d& ray) const {
    //ProjectionResult result;

    //Eigen::Vector3d r = ray.normalized();

	//double theta = std::acos(r.z());
	
	//// limit FOV
	//const double maxTheta = M_PI / 2.0; // try 90°, then 120°, etc.
	
	//if (theta > maxTheta) {
	    //result.valid = false;
	    //return result;
	//}
	
	//double sinTheta = std::sqrt(r.x()*r.x() + r.y()*r.y());
	//double scale = (sinTheta > 1e-12) ? (theta / sinTheta) : 1.0;
	
	//double x = r.x() * scale;
	//double y = r.y() * scale;
	
	//double u = fx_ * x + cx_;
	//double v = cy_ - fy_ * y;
	
	//result.uv = {u, v};
	
	//result.valid =
	    //(u >= 0 && u < size_.width &&
	     //v >= 0 && v < size_.height);

    //return result;
//}
