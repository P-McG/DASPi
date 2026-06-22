// daspi-calibrate-charuco.cpp

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <array>
#include <iomanip>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "DASPi-runtime-camera-model.h"

//#include "DASPi-shape-config.h"
//#include "DASPi-overlapspace.h"
//#include "DASPi-icosahedronspherespace.h"
//#include "DASPi-module-spherical-basis.h"

namespace {

struct Options {
    std::vector<std::string> observationCsvPaths;

    std::string poseCsvPath{"charuco-pose-report.csv"};
    std::string relativePoseCsvPath{"relative-camera-pose-report.csv"};
    std::string calibrationOutputPath;
    std::string nominalRelativeRvecDegCsv;

    bool calibrateIntrinsics{false};
    int imageWidth{1456};
    int imageHeight{1088};
    std::string intrinsicsPrefix{"camera-intrinsics"};
    
    int squaresX{15};
    int squaresY{15};
    float squareLength{0.030f};
    float markerLength{0.023f};
    std::string dictionaryName{"DICT_4X4_250"};

    double fx{DASPi::RuntimeCameraIntrinsics::defaultFx};
    double fy{DASPi::RuntimeCameraIntrinsics::defaultFy};
    double cx{DASPi::RuntimeCameraIntrinsics::defaultCx};
    double cy{DASPi::RuntimeCameraIntrinsics::defaultCy};

    int minCorners{6};
};

struct FrameKey {
    std::string sessionId{"default"};
    int poseId{0};
    int module{-1};
    std::uint64_t frame{0};

    bool operator<(const FrameKey& other) const
    {
        if (sessionId != other.sessionId) {
            return sessionId < other.sessionId;
        }

        if (poseId != other.poseId) {
            return poseId < other.poseId;
        }

        if (module != other.module) {
            return module < other.module;
        }

        return frame < other.frame;
    }
};

struct FrameObservations {
    int facet{-1};
    std::int64_t timestampNs{0};
    std::map<int, cv::Point2f> cornersById;
};

struct CameraModel {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    bool calibrated{false};
    double calibrationRms{0.0};
};

struct SolvedPose {
    std::string sessionId{"default"};
    int poseId{0};
    int module{-1};
    int facet{-1};
    std::uint64_t frame{0};
    std::int64_t timestampNs{0};
    int corners{0};
    double rmsPx{0.0};

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat Rcb;   // board -> camera rotation
};

struct SharedPoseKey {
    std::string sessionId{"default"};
    int poseId{0};

    bool operator<(const SharedPoseKey& other) const
    {
        if (sessionId != other.sessionId) {
            return sessionId < other.sessionId;
        }

        return poseId < other.poseId;
    }
};

std::string Trim(std::string s)
{
    const auto notSpace = [](unsigned char c) {
        return !std::isspace(c);
    };

    s.erase(
        s.begin(),
        std::find_if(s.begin(), s.end(), notSpace)
    );

    s.erase(
        std::find_if(s.rbegin(), s.rend(), notSpace).base(),
        s.end()
    );

    return s;
}

std::vector<std::string> Split(const std::string& s, char delim)
{
    std::vector<std::string> out;
    std::string item;
    std::istringstream ss(s);

    while (std::getline(ss, item, delim)) {
        out.push_back(Trim(item));
    }

    return out;
}

bool StartsWith(const std::string& s, const std::string& prefix)
{
    return s.rfind(prefix, 0) == 0;
}

std::vector<std::string> SplitCsvList(const std::string& csv)
{
    std::vector<std::string> paths = Split(csv, ',');

    paths.erase(
        std::remove_if(
            paths.begin(),
            paths.end(),
            [](const std::string& s) { return s.empty(); }
        ),
        paths.end()
    );

    return paths;
}

void PrintUsage(const char* program)
{
    std::cerr
        << "Usage:\n"
        << "  " << program << " "
        << "--observations=<csv0,csv1,...> "
        << "[--poseCsv=charuco-pose-report.csv] "
        << "[--relativePoseCsv=relative-camera-pose-report.csv] "
        << "[--writeCalibration=camera-calibration.txt] "
        << "[--squaresX=15] [--squaresY=15] "
        << "[--squareLength=0.030] [--markerLength=0.023] "
        << "[--dictionary=DICT_4X4_250] "
        << "[--fx=<runtime>] [--fy=<runtime>] [--cx=<runtime>] [--cy=<runtime>] "
        << "[--calibrateIntrinsics] "
        << "[--imageWidth=1456] [--imageHeight=1088] "
        << "[--intrinsicsPrefix=camera-intrinsics] "
        << "[--nominalRelativeRvecDeg=rx,ry,rz] "
        << "[--minCorners=6]\n";
}

Options ParseArgs(int argc, char* argv[])
{
    Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};

        if (StartsWith(arg, "--observations=")) {
            options.observationCsvPaths =
                SplitCsvList(arg.substr(std::string("--observations=").size()));
        } else if (StartsWith(arg, "--poseCsv=")) {
            options.poseCsvPath =
                arg.substr(std::string("--poseCsv=").size());
        } else if (StartsWith(arg, "--squaresX=")) {
            options.squaresX =
                std::stoi(arg.substr(std::string("--squaresX=").size()));
        } else if (StartsWith(arg, "--squaresY=")) {
            options.squaresY =
                std::stoi(arg.substr(std::string("--squaresY=").size()));
        } else if (StartsWith(arg, "--squareLength=")) {
            options.squareLength =
                std::stof(arg.substr(std::string("--squareLength=").size()));
        } else if (StartsWith(arg, "--markerLength=")) {
            options.markerLength =
                std::stof(arg.substr(std::string("--markerLength=").size()));
        } else if (StartsWith(arg, "--dictionary=")) {
            options.dictionaryName =
                arg.substr(std::string("--dictionary=").size());
        } else if (StartsWith(arg, "--fx=")) {
            options.fx =
                std::stod(arg.substr(std::string("--fx=").size()));
        } else if (StartsWith(arg, "--fy=")) {
            options.fy =
                std::stod(arg.substr(std::string("--fy=").size()));
        } else if (StartsWith(arg, "--cx=")) {
            options.cx =
                std::stod(arg.substr(std::string("--cx=").size()));
        } else if (StartsWith(arg, "--cy=")) {
            options.cy =
                std::stod(arg.substr(std::string("--cy=").size()));
        } else if (StartsWith(arg, "--minCorners=")) {
            options.minCorners =
                std::stoi(arg.substr(std::string("--minCorners=").size()));
        } else if (arg == "--calibrateIntrinsics") {
            options.calibrateIntrinsics = true;
        } else if (StartsWith(arg, "--imageWidth=")) {
            options.imageWidth =
                std::stoi(arg.substr(std::string("--imageWidth=").size()));
        } else if (StartsWith(arg, "--imageHeight=")) {
            options.imageHeight =
                std::stoi(arg.substr(std::string("--imageHeight=").size()));
        } else if (StartsWith(arg, "--intrinsicsPrefix=")) {
            options.intrinsicsPrefix =
                arg.substr(std::string("--intrinsicsPrefix=").size());
        } else if (StartsWith(arg, "--relativePoseCsv=")) {
            options.relativePoseCsvPath =
                arg.substr(std::string("--relativePoseCsv=").size());
        } else if (StartsWith(arg, "--writeCalibration=")) {
            options.calibrationOutputPath =
                arg.substr(std::string("--writeCalibration=").size());
        } else if (StartsWith(arg, "--nominalRelativeRvecDeg=")) {
            options.nominalRelativeRvecDegCsv =
                arg.substr(std::string("--nominalRelativeRvecDeg=").size());
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else {
            PrintUsage(argv[0]);
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.observationCsvPaths.empty()) {
        PrintUsage(argv[0]);
        throw std::runtime_error("Missing --observations=<csv0,csv1,...>");
    }

    if (options.squaresX <= 1 || options.squaresY <= 1) {
        throw std::runtime_error("squaresX/squaresY must be > 1");
    }

    if (options.squareLength <= 0.0f || options.markerLength <= 0.0f) {
        throw std::runtime_error("squareLength/markerLength must be > 0");
    }

    if (options.markerLength >= options.squareLength) {
        throw std::runtime_error("markerLength must be smaller than squareLength");
    }

    if (options.minCorners < 4) {
        throw std::runtime_error("minCorners must be >= 4");
    }

    return options;
}
/*
 * ChArUco corner IDs are the inner chessboard-corner IDs.
 *
 * For a board with squaresX by squaresY squares, there are:
 *
 *     (squaresX - 1) * (squaresY - 1)
 *
 * ChArUco chessboard corners.
 *
 * OpenCV uses row-major ordering:
 *
 *     id = y * (squaresX - 1) + x
 *
 * where x/y are inner-corner indices.
 */
std::vector<cv::Point3f> MakeCharucoChessboardCorners(
    int squaresX,
    int squaresY,
    float squareLength)
{
    std::vector<cv::Point3f> corners;

    corners.reserve(
        static_cast<std::size_t>((squaresX - 1) * (squaresY - 1))
    );

    for (int y = 0; y < squaresY - 1; ++y) {
        for (int x = 0; x < squaresX - 1; ++x) {
            corners.emplace_back(
                static_cast<float>(x + 1) * squareLength,
                static_cast<float>(y + 1) * squareLength,
                0.0f
            );
        }
    }

    return corners;
}

bool AreObjectPointsCollinear(
    const std::vector<cv::Point3f>& objectPoints)
{
    if (objectPoints.size() < 3) {
        return true;
    }

    const cv::Point2d p0(
        objectPoints[0].x,
        objectPoints[0].y
    );

    constexpr double kEps = 1e-12;

    for (std::size_t i = 1; i < objectPoints.size(); ++i) {
        const cv::Point2d p1(
            objectPoints[i].x,
            objectPoints[i].y
        );

        const cv::Point2d v1 =
            p1 - p0;

        if (std::abs(v1.x) < kEps &&
            std::abs(v1.y) < kEps) {
            continue;
        }

        for (std::size_t j = i + 1; j < objectPoints.size(); ++j) {
            const cv::Point2d p2(
                objectPoints[j].x,
                objectPoints[j].y
            );

            const cv::Point2d v2 =
                p2 - p0;

            const double cross =
                v1.x * v2.y - v1.y * v2.x;

            if (std::abs(cross) > kEps) {
                return false;
            }
        }
    }

    return true;
}

bool ExtractFramePoints(
    const FrameObservations& frameObs,
    const std::vector<cv::Point3f>& boardCorners,
    int minCorners,
    std::vector<cv::Point3f>& objectPoints,
    std::vector<cv::Point2f>& imagePoints,
    bool& badId,
    bool& collinear)
{
    objectPoints.clear();
    imagePoints.clear();

    badId = false;
    collinear = false;

    for (const auto& [cornerId, imagePoint] : frameObs.cornersById) {
        if (cornerId < 0 ||
            static_cast<std::size_t>(cornerId) >= boardCorners.size()) {
            badId = true;
            return false;
        }

        objectPoints.push_back(
            boardCorners[static_cast<std::size_t>(cornerId)]
        );

        imagePoints.push_back(imagePoint);
    }

    if (static_cast<int>(objectPoints.size()) < minCorners) {
        return false;
    }

    if (AreObjectPointsCollinear(objectPoints)) {
        collinear = true;
        return false;
    }

    return true;
}

void LoadObservationCsv(
    const std::string& path,
    std::map<FrameKey, FrameObservations>& observations)
{
    std::ifstream in(path);

    if (!in) {
        throw std::runtime_error("Failed to open observations CSV: " + path);
    }

    std::string line;
    std::size_t lineNumber = 0;
    std::size_t loadedRows = 0;

    while (std::getline(in, line)) {
        ++lineNumber;

        line = Trim(line);

        if (line.empty() || StartsWith(line, "#")) {
            continue;
        }

        if (StartsWith(line, "frame,") ||
            StartsWith(line, "session_id,")) {
            continue;
        }

        const std::vector<std::string> fields =
            Split(line, ',');
        
        std::string sessionId{"default"};
        int poseId = 0;
        std::int64_t timestampNs = 0;
        std::uint64_t frame = 0;
        int module = -1;
        int facet = -1;
        int cornerId = -1;
        float x = 0.0f;
        float y = 0.0f;
        
        if (fields.size() >= 9) {
            /*
             * New format:
             *   session_id,pose_id,timestamp_ns,frame,module,facet,corner_id,x,y
             */
            sessionId = fields[0];
            poseId = std::stoi(fields[1]);
            timestampNs = static_cast<std::int64_t>(std::stoll(fields[2]));
            frame = static_cast<std::uint64_t>(std::stoull(fields[3]));
            module = std::stoi(fields[4]);
            facet = std::stoi(fields[5]);
            cornerId = std::stoi(fields[6]);
            x = std::stof(fields[7]);
            y = std::stof(fields[8]);
        } else if (fields.size() >= 6) {
            /*
             * Old format:
             *   frame,module,facet,corner_id,x,y
             */
            frame = static_cast<std::uint64_t>(std::stoull(fields[0]));
            module = std::stoi(fields[1]);
            facet = std::stoi(fields[2]);
            cornerId = std::stoi(fields[3]);
            x = std::stof(fields[4]);
            y = std::stof(fields[5]);
        } else {
            throw std::runtime_error(
                "Invalid CSV line " + std::to_string(lineNumber) +
                " in " + path +
                ". Expected either old format "
                "frame,module,facet,corner_id,x,y "
                "or new format "
                "session_id,pose_id,timestamp_ns,frame,module,facet,corner_id,x,y"
            );
        }
        
        FrameKey key{
            .sessionId = sessionId,
            .poseId = poseId,
            .module = module,
            .frame = frame
        };
        
        FrameObservations& frameObs =
            observations[key];
        
        frameObs.facet = facet;
        frameObs.timestampNs = timestampNs;

        /*
         * If the same corner appears twice for the same module/frame,
         * keep the last one. That handles accidental duplicate append rows.
         */
        frameObs.cornersById[cornerId] =
            cv::Point2f(x, y);

        ++loadedRows;
    }

    std::cout << "[load] " << path
              << " rows=" << loadedRows
              << '\n';
}

double ReprojectionRms(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const cv::Mat& rvec,
    const cv::Mat& tvec)
{
    std::vector<cv::Point2f> projected;

    cv::projectPoints(
        objectPoints,
        rvec,
        tvec,
        cameraMatrix,
        distCoeffs,
        projected
    );

    if (projected.size() != imagePoints.size() ||
        projected.empty()) {
        throw std::runtime_error("projectPoints returned invalid output");
    }

    double sumSq = 0.0;

    for (std::size_t i = 0; i < projected.size(); ++i) {
        const cv::Point2f d =
            projected[i] - imagePoints[i];

        sumSq += static_cast<double>(d.x) * d.x +
                 static_cast<double>(d.y) * d.y;
    }

    return std::sqrt(sumSq / static_cast<double>(projected.size()));
}

double Mean(const std::vector<double>& values)
{
    if (values.empty()) {
        return 0.0;
    }

    const double sum =
        std::accumulate(values.begin(), values.end(), 0.0);

    return sum / static_cast<double>(values.size());
}

void WriteRelativePoseReport(
    const std::vector<SolvedPose>& solvedPoses,
    const std::string& path)
{
    std::map<SharedPoseKey, std::map<int, SolvedPose>> bestPoseBySharedPose;

    for (const SolvedPose& pose : solvedPoses) {
        SharedPoseKey key{
            .sessionId = pose.sessionId,
            .poseId = pose.poseId
        };

        auto& moduleMap =
            bestPoseBySharedPose[key];

        auto it =
            moduleMap.find(pose.module);

        if (it == moduleMap.end() ||
            pose.rmsPx < it->second.rmsPx) {
            moduleMap[pose.module] = pose;
        }
    }

    std::ofstream out(path);

    if (!out) {
        throw std::runtime_error(
            "Failed to open relative pose CSV: " + path
        );
    }

    out
        << "session_id,pose_id,"
        << "module0_frame,module1_frame,"
        << "module0_rms_px,module1_rms_px,"
        << "relative_angle_deg,"
        << "relative_rvec_x,relative_rvec_y,relative_rvec_z,"
        << "relative_t_x,relative_t_y,relative_t_z\n";

    std::size_t sharedPoseCount = 0;
    std::vector<double> relativeAnglesDeg;

    constexpr double kRadToDeg =
        180.0 / 3.141592653589793238462643383279502884;

    for (const auto& [sharedKey, moduleMap] : bestPoseBySharedPose) {
        const auto module0It = moduleMap.find(0);
        const auto module1It = moduleMap.find(1);

        if (module0It == moduleMap.end() ||
            module1It == moduleMap.end()) {
            continue;
        }

        const SolvedPose& p0 =
            module0It->second;

        const SolvedPose& p1 =
            module1It->second;

        /*
         * solvePnP gives:
         *
         *   X_cam = R_cam_board * X_board + t_cam_board
         *
         * For the same physical board pose:
         *
         *   R_cam1_cam0 = R_cam1_board * inverse(R_cam0_board)
         *               = R1 * R0^T
         *
         *   t_cam1_cam0 = t1 - R_cam1_cam0 * t0
         */
        const cv::Mat R10 =
            p1.Rcb * p0.Rcb.t();

        const cv::Mat t10 =
            p1.tvec - R10 * p0.tvec;

        cv::Mat rvec10;
        cv::Rodrigues(R10, rvec10);

        const double angleRad =
            cv::norm(rvec10);

        const double angleDeg =
            angleRad * kRadToDeg;

        relativeAnglesDeg.push_back(angleDeg);
        ++sharedPoseCount;

        out
            << sharedKey.sessionId << ','
            << sharedKey.poseId << ','
            << p0.frame << ','
            << p1.frame << ','
            << std::fixed << std::setprecision(6)
            << p0.rmsPx << ','
            << p1.rmsPx << ','
            << angleDeg << ','
            << rvec10.at<double>(0, 0) << ','
            << rvec10.at<double>(1, 0) << ','
            << rvec10.at<double>(2, 0) << ','
            << t10.at<double>(0, 0) << ','
            << t10.at<double>(1, 0) << ','
            << t10.at<double>(2, 0) << '\n';
    }

    std::cout << "[relative pose]"
              << " shared_poses=" << sharedPoseCount
              << " output=" << path
              << '\n';

    if (!relativeAnglesDeg.empty()) {
        std::cout << "[relative pose]"
                  << " mean_relative_angle_deg="
                  << Mean(relativeAnglesDeg)
                  << '\n';
    }
}

struct RelativeAverage {
    bool valid{false};
    Eigen::Matrix3d R10Observed{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t10Observed{Eigen::Vector3d::Zero()};
    std::size_t sharedPoseCount{0};
};

Eigen::Matrix3d CvMat3x3ToEigen(const cv::Mat& m)
{
    Eigen::Matrix3d out;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out(r, c) = m.at<double>(r, c);
        }
    }

    return out;
}

Eigen::Vector3d CvVec3ToEigen(const cv::Mat& v)
{
    return Eigen::Vector3d(
        v.at<double>(0, 0),
        v.at<double>(1, 0),
        v.at<double>(2, 0)
    );
}

double Clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

double RadToDeg(double rad)
{
    return rad * 180.0 / 3.141592653589793238462643383279502884;
}

struct LocalYawPitchRollDeg {
    double yawDeg{0.0};
    double pitchDeg{0.0};
    double rollDeg{0.0};
};

/*
 * Inverse of:
 *
 *   Rdelta = Ryaw(local +Y) * Rpitch(local +X) * Rroll(local +Z)
 */
LocalYawPitchRollDeg DecomposeLocalYawPitchRollDeg(
    const Eigen::Matrix3d& R)
{
    const double pitchRad =
        std::asin(Clamp(-R(1, 2), -1.0, 1.0));

    const double cosPitch =
        std::cos(pitchRad);

    double yawRad = 0.0;
    double rollRad = 0.0;

    if (std::abs(cosPitch) > 1.0e-9) {
        yawRad =
            std::atan2(R(0, 2), R(2, 2));

        rollRad =
            std::atan2(R(1, 0), R(1, 1));
    } else {
        /*
         * Gimbal-lock fallback. This should not happen for small physical
         * mounting corrections, but keep the result finite.
         */
        yawRad =
            std::atan2(-R(2, 0), R(0, 0));

        rollRad = 0.0;
    }

    return LocalYawPitchRollDeg{
        .yawDeg = RadToDeg(yawRad),
        .pitchDeg = RadToDeg(pitchRad),
        .rollDeg = RadToDeg(rollRad)
    };
}

RelativeAverage ComputeAverageObservedRelativeTransform(
    const std::vector<SolvedPose>& solvedPoses)
{
    std::map<SharedPoseKey, std::map<int, SolvedPose>> bestPoseBySharedPose;

    for (const SolvedPose& pose : solvedPoses) {
        SharedPoseKey key{
            .sessionId = pose.sessionId,
            .poseId = pose.poseId
        };

        auto& moduleMap =
            bestPoseBySharedPose[key];

        auto it =
            moduleMap.find(pose.module);

        if (it == moduleMap.end() ||
            pose.rmsPx < it->second.rmsPx) {
            moduleMap[pose.module] = pose;
        }
    }

    bool haveReferenceQuaternion = false;
    Eigen::Quaterniond qReference;
    Eigen::Vector4d qAccum =
        Eigen::Vector4d::Zero();

    Eigen::Vector3d tAccum =
        Eigen::Vector3d::Zero();

    double weightAccum = 0.0;
    std::size_t sharedPoseCount = 0;

    for (const auto& [sharedKey, moduleMap] : bestPoseBySharedPose) {
        const auto module0It = moduleMap.find(0);
        const auto module1It = moduleMap.find(1);

        if (module0It == moduleMap.end() ||
            module1It == moduleMap.end()) {
            continue;
        }

        const SolvedPose& p0 =
            module0It->second;

        const SolvedPose& p1 =
            module1It->second;

        const Eigen::Matrix3d R0 =
            CvMat3x3ToEigen(p0.Rcb);

        const Eigen::Matrix3d R1 =
            CvMat3x3ToEigen(p1.Rcb);

        const Eigen::Vector3d t0 =
            CvVec3ToEigen(p0.tvec);

        const Eigen::Vector3d t1 =
            CvVec3ToEigen(p1.tvec);

        /*
         * solvePnP gives board -> camera:
         *
         *   X_cam = R_cam_board * X_board + t_cam_board
         *
         * Same physical board pose observed by module 0 and module 1:
         *
         *   R_cam1_cam0 = R1 * R0^T
         *   t_cam1_cam0 = t1 - R_cam1_cam0 * t0
         */
        const Eigen::Matrix3d R10 =
            R1 * R0.transpose();

        const Eigen::Vector3d t10 =
            t1 - R10 * t0;

        Eigen::Quaterniond q(R10);
        q.normalize();

        if (!haveReferenceQuaternion) {
            qReference = q;
            haveReferenceQuaternion = true;
        }

        if (qReference.coeffs().dot(q.coeffs()) < 0.0) {
            q.coeffs() *= -1.0;
        }

        const double weight =
            1.0 / std::max(1.0e-6, p0.rmsPx + p1.rmsPx);

        qAccum += weight * q.coeffs();
        tAccum += weight * t10;
        weightAccum += weight;

        ++sharedPoseCount;
    }

    if (sharedPoseCount == 0 || weightAccum <= 0.0) {
        return RelativeAverage{};
    }

    qAccum /= weightAccum;

    Eigen::Quaterniond qAvg(
        qAccum(3),
        qAccum(0),
        qAccum(1),
        qAccum(2)
    );

    qAvg.normalize();

    RelativeAverage out;
    out.valid = true;
    out.R10Observed = qAvg.toRotationMatrix();
    out.t10Observed = tAccum / weightAccum;
    out.sharedPoseCount = sharedPoseCount;

    return out;
}

double DegToRad(double deg)
{
    return deg * 3.141592653589793238462643383279502884 / 180.0;
}

std::array<double, 3> ParseTripleCsv(const std::string& csv)
{
    const std::vector<std::string> parts =
        Split(csv, ',');

    if (parts.size() != 3) {
        throw std::runtime_error(
            "Expected comma-separated triple: rx,ry,rz"
        );
    }

    return std::array<double, 3>{
        std::stod(parts[0]),
        std::stod(parts[1]),
        std::stod(parts[2])
    };
}

Eigen::Matrix3d RotationFromRvecDegCsv(const std::string& csv)
{
    const auto rvecDeg =
        ParseTripleCsv(csv);

    const Eigen::Vector3d rvecRad(
        DegToRad(rvecDeg[0]),
        DegToRad(rvecDeg[1]),
        DegToRad(rvecDeg[2])
    );

    const double angle =
        rvecRad.norm();

    if (angle < 1.0e-12) {
        return Eigen::Matrix3d::Identity();
    }

    return Eigen::AngleAxisd(
        angle,
        rvecRad / angle
    ).toRotationMatrix();
}

void WriteCalibrationCorrectionFile(
    const std::vector<SolvedPose>& solvedPoses,
    const std::string& path,
    const std::string& nominalRelativeRvecDegCsv)
{
    if (nominalRelativeRvecDegCsv.empty()) {
        std::cerr
            << "[calibration output] --writeCalibration was requested, "
            << "but --nominalRelativeRvecDeg=rx,ry,rz was not supplied. "
            << "Not writing " << path << '\n';

        std::cerr
            << "[calibration output] This is intentional: writing a "
            << "camera-calibration.txt requires comparing observed relative "
            << "pose against the nominal DASPi relative pose.\n";

        return;
    }

    const RelativeAverage avg =
        ComputeAverageObservedRelativeTransform(solvedPoses);

    if (!avg.valid) {
        std::cerr << "[calibration output] no shared poses; not writing "
                  << path << '\n';
        return;
    }

    const Eigen::Matrix3d R10Nominal =
        RotationFromRvecDegCsv(nominalRelativeRvecDegCsv);

    /*
     * Module 0 is the reference:
     *
     *   Rcw0_corrected = Rcw0_nominal
     *
     * Module 1 receives local correction:
     *
     *   Rcw1_corrected = Rcw1_nominal * D1
     *
     * Relative camera0 -> camera1:
     *
     *   R10 = Rcw1^T * Rcw0
     *
     * So:
     *
     *   D1 = R10_nominal * R10_observed^T
     */
    const Eigen::Matrix3d D1 =
        R10Nominal * avg.R10Observed.transpose();

    const LocalYawPitchRollDeg module1Correction =
        DecomposeLocalYawPitchRollDeg(D1);

    std::ofstream out(path);

    if (!out) {
        throw std::runtime_error(
            "Failed to open calibration output file: " + path
        );
    }

    out << "# DASPi generated camera calibration correction\n";
    out << "# module_index yaw_deg pitch_deg roll_deg\n";
    out << "# module 0 is kept as reference\n";
    out << "# shared_poses=" << avg.sharedPoseCount << '\n';
    out << "# nominal_relative_rvec_deg=" << nominalRelativeRvecDegCsv << '\n';
    out << "# observed_t10_m "
        << avg.t10Observed.x() << ' '
        << avg.t10Observed.y() << ' '
        << avg.t10Observed.z() << '\n';

    out << std::fixed << std::setprecision(9);
    out << "0 0.0 0.0 0.0\n";
    out << "1 "
        << module1Correction.yawDeg << ' '
        << module1Correction.pitchDeg << ' '
        << module1Correction.rollDeg << '\n';

    std::cout << "[calibration output]"
              << " shared_poses=" << avg.sharedPoseCount
              << " output=" << path
              << '\n';

    std::cout << "[calibration output]"
              << " module1 yaw_deg=" << module1Correction.yawDeg
              << " pitch_deg=" << module1Correction.pitchDeg
              << " roll_deg=" << module1Correction.rollDeg
              << '\n';
}



} // namespace

int main(int argc, char* argv[])
{
    try {
        const Options options =
            ParseArgs(argc, argv);

        std::cout << "[charuco calibrate]"
                  << " squares=" << options.squaresX << "x" << options.squaresY
                  << " squareLength=" << options.squareLength
                  << " markerLength=" << options.markerLength
                  << " dictionary=" << options.dictionaryName
                  << '\n';

        const std::vector<cv::Point3f> boardCorners =
            MakeCharucoChessboardCorners(
                options.squaresX,
                options.squaresY,
                options.squareLength
            );
        
        std::cout << "[charuco calibrate]"
                  << " board_corners=" << boardCorners.size()
                  << " note=dictionary_is_only_used_by_aperture_detector"
                  << '\n';

        if (options.calibrateIntrinsics) {
            std::cout
                << "[camera model]"
                << " solve=calibrated_intrinsics"
                << " initial_fx=" << options.fx
                << " initial_fy=" << options.fy
                << " initial_cx=" << options.cx
                << " initial_cy=" << options.cy
                << " note=runtime_ModuleSphericalMap_should_load_matching_camera_intrinsics_module_yml"
                << '\n';
        } else {
            std::cout
                << "[camera model]"
                << " solve=runtime_gnomonic"
                << " fx=" << options.fx
                << " fy=" << options.fy
                << " cx=" << options.cx
                << " cy=" << options.cy
                << " distortion=zero"
                << '\n';
        }

        std::map<FrameKey, FrameObservations> observations;

        for (const std::string& path : options.observationCsvPaths) {
            LoadObservationCsv(path, observations);
        }
        
        const cv::Mat defaultCameraMatrix =
            (cv::Mat_<double>(3, 3)
                << options.fx, 0.0, options.cx,
                   0.0, options.fy, options.cy,
                   0.0, 0.0, 1.0);
        
        const cv::Mat defaultDistCoeffs =
            cv::Mat::zeros(1, 5, CV_64F);
        
        CameraModel defaultModel{
            .cameraMatrix = defaultCameraMatrix.clone(),
            .distCoeffs = defaultDistCoeffs.clone(),
            .calibrated = false,
            .calibrationRms = 0.0
        };
        
        std::map<int, CameraModel> cameraModels;
        
        for (const auto& [key, frameObs] : observations) {
            cameraModels.emplace(key.module, defaultModel);
        }
        
        if (options.calibrateIntrinsics) {
            std::map<int, std::vector<std::vector<cv::Point3f>>> objectPointsByModule;
            std::map<int, std::vector<std::vector<cv::Point2f>>> imagePointsByModule;
        
            for (const auto& [key, frameObs] : observations) {
                std::vector<cv::Point3f> objectPoints;
                std::vector<cv::Point2f> imagePoints;
        
                bool badId = false;
                bool collinear = false;
        
                const bool ok =
                    ExtractFramePoints(
                        frameObs,
                        boardCorners,
                        options.minCorners,
                        objectPoints,
                        imagePoints,
                        badId,
                        collinear
                    );
        
                if (!ok) {
                    continue;
                }
        
                objectPointsByModule[key.module].push_back(objectPoints);
                imagePointsByModule[key.module].push_back(imagePoints);
            }
        
            for (const auto& [module, objectViews] : objectPointsByModule) {
                const auto& imageViews =
                    imagePointsByModule[module];
        
                if (objectViews.size() < 10) {
                    std::cerr << "[intrinsics] module=" << module
                              << " skipped; need at least 10 usable views, got "
                              << objectViews.size()
                              << '\n';
                    continue;
                }
        
                cv::Mat cameraMatrix =
                    defaultCameraMatrix.clone();
        
                cv::Mat distCoeffs =
                    cv::Mat::zeros(1, 5, CV_64F);
        
                std::vector<cv::Mat> rvecs;
                std::vector<cv::Mat> tvecs;
        
                const int flags =
                    cv::CALIB_USE_INTRINSIC_GUESS;
        
                const double rms =
                    cv::calibrateCamera(
                        objectViews,
                        imageViews,
                        cv::Size(options.imageWidth, options.imageHeight),
                        cameraMatrix,
                        distCoeffs,
                        rvecs,
                        tvecs,
                        flags
                    );
        
                 cameraModels[module] =
                    CameraModel{
                        .cameraMatrix = cameraMatrix.clone(),
                        .distCoeffs = distCoeffs.clone(),
                        .calibrated = true,
                        .calibrationRms = rms
                    };
        
                const std::string intrinsicsPath =
                    options.intrinsicsPrefix +
                    "-module" +
                    std::to_string(module) +
                    ".yml";
        
                cv::FileStorage fs(
                    intrinsicsPath,
                    cv::FileStorage::WRITE
                );
        
                if (!fs.isOpened()) {
                    throw std::runtime_error(
                        "Failed to open intrinsics output: " +
                        intrinsicsPath
                    );
                }
        
                fs << "module" << module;
                fs << "image_width" << options.imageWidth;
                fs << "image_height" << options.imageHeight;
                fs << "squares_x" << options.squaresX;
                fs << "squares_y" << options.squaresY;
                fs << "square_length" << options.squareLength;
                fs << "marker_length" << options.markerLength;
                fs << "calibration_rms_px" << rms;
                fs << "camera_matrix" << cameraMatrix;
                fs << "dist_coeffs" << distCoeffs;
        
                std::cout << "[intrinsics]"
                          << " module=" << module
                          << " views=" << objectViews.size()
                          << " rms_px=" << rms
                          << " fx=" << cameraMatrix.at<double>(0, 0)
                          << " fy=" << cameraMatrix.at<double>(1, 1)
                          << " cx=" << cameraMatrix.at<double>(0, 2)
                          << " cy=" << cameraMatrix.at<double>(1, 2)
                          << " output=" << intrinsicsPath
                          << '\n';
        
                std::cout << "[intrinsics]"
                          << " module=" << module
                          << " distCoeffs=" << distCoeffs
                          << '\n';
            }
        }

        std::ofstream poseCsv(options.poseCsvPath);

        if (!poseCsv) {
            throw std::runtime_error(
                "Failed to open pose CSV output: " +
                options.poseCsvPath
            );
        }

        poseCsv
            << "session_id,pose_id,module,frame,facet,timestamp_ns,corners,rms_px,"
            << "rvec_x,rvec_y,rvec_z,"
            << "tvec_x,tvec_y,tvec_z\n";

        std::map<int, std::vector<double>> rmsByModule;

        std::size_t attemptedFrames = 0;
        std::size_t solvedFrames = 0;
        std::size_t skippedTooFew = 0;
        std::size_t skippedBadId = 0;
        std::size_t skippedCollinear = 0;
        std::size_t failedPnP = 0;
        
        std::vector<SolvedPose> solvedPoses;

        for (const auto& [key, frameObs] : observations) {
            ++attemptedFrames;

            std::vector<cv::Point3f> objectPoints;
            std::vector<cv::Point2f> imagePoints;

            bool hasBadId = false;

            for (const auto& [cornerId, imagePoint] : frameObs.cornersById) {
                if (cornerId < 0 ||
                    static_cast<std::size_t>(cornerId) >= boardCorners.size()) {
                    hasBadId = true;
                    break;
                }

                objectPoints.push_back(
                    boardCorners[static_cast<std::size_t>(cornerId)]
                );

                imagePoints.push_back(imagePoint);
            }

            if (hasBadId) {
                ++skippedBadId;
                continue;
            }

            if (static_cast<int>(objectPoints.size()) < options.minCorners) {
                ++skippedTooFew;
                continue;
            }

            if (AreObjectPointsCollinear(objectPoints)) {
                ++skippedCollinear;
                continue;
            }

            cv::Mat rvec;
            cv::Mat tvec;

            const auto modelIt =
                cameraModels.find(key.module);
            
            const CameraModel& model =
                (modelIt != cameraModels.end())
                    ? modelIt->second
                    : defaultModel;
            
            const bool ok =
                cv::solvePnP(
                    objectPoints,
                    imagePoints,
                    model.cameraMatrix,
                    model.distCoeffs,
                    rvec,
                    tvec,
                    false,
                    cv::SOLVEPNP_ITERATIVE
                );

            if (!ok) {
                ++failedPnP;
                continue;
            }

            const double rms =
                ReprojectionRms(
                    objectPoints,
                    imagePoints,
                    model.cameraMatrix,
                    model.distCoeffs,
                    rvec,
                    tvec
                );
                
            cv::Mat Rcb;
            cv::Rodrigues(rvec, Rcb);
            
            solvedPoses.push_back(
                SolvedPose{
                    .sessionId = key.sessionId,
                    .poseId = key.poseId,
                    .module = key.module,
                    .facet = frameObs.facet,
                    .frame = key.frame,
                    .timestampNs = frameObs.timestampNs,
                    .corners = static_cast<int>(objectPoints.size()),
                    .rmsPx = rms,
                    .rvec = rvec.clone(),
                    .tvec = tvec.clone(),
                    .Rcb = Rcb.clone()
                }
            );  
                

            rmsByModule[key.module].push_back(rms);
            ++solvedFrames;

            poseCsv
                << key.sessionId << ','
                << key.poseId << ','
                << key.module << ','
                << key.frame << ','
                << frameObs.facet << ','
                << frameObs.timestampNs << ','
                << objectPoints.size() << ','
                << std::fixed << std::setprecision(6)
                << rms << ','
                << rvec.at<double>(0, 0) << ','
                << rvec.at<double>(1, 0) << ','
                << rvec.at<double>(2, 0) << ','
                << tvec.at<double>(0, 0) << ','
                << tvec.at<double>(1, 0) << ','
                << tvec.at<double>(2, 0) << '\n';
        }

        std::cout << "[summary]"
                  << " grouped_frames=" << observations.size()
                  << " attempted=" << attemptedFrames
                  << " solved=" << solvedFrames
                  << " skipped_too_few=" << skippedTooFew
                  << " skipped_bad_id=" << skippedBadId
                  << " skipped_collinear=" << skippedCollinear
                  << " failed_pnp=" << failedPnP
                  << '\n';

        for (const auto& [module, rmsValues] : rmsByModule) {
            std::cout << "[module summary]"
                      << " module=" << module
                      << " solved_frames=" << rmsValues.size()
                      << " mean_rms_px=" << Mean(rmsValues)
                      << '\n';
        }
        
        WriteRelativePoseReport(
            solvedPoses,
            options.relativePoseCsvPath
        );
        
        if (!options.calibrationOutputPath.empty()) {
            WriteCalibrationCorrectionFile(
                solvedPoses,
                options.calibrationOutputPath,
                options.nominalRelativeRvecDegCsv
            );
        }
        
        std::cout << "[output] wrote " << options.poseCsvPath << '\n';

        std::cout
            << "[next] This validates ChArUco observations and PnP quality.\n"
            << "[next] To generate yaw/pitch/roll, add either a known board-to-rig fixture pose\n"
            << "[next] or collect shared/multi-camera observations for bundle adjustment.\n";
            


        return EXIT_SUCCESS;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
