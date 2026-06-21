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

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace {

struct Options {
    std::vector<std::string> observationCsvPaths;

    std::string poseCsvPath{"charuco-pose-report.csv"};

    bool calibrateIntrinsics{false};
    int imageWidth{1456};
    int imageHeight{1088};
    std::string intrinsicsPrefix{"camera-intrinsics"};
    
    int squaresX{15};
    int squaresY{15};
    float squareLength{0.030f};
    float markerLength{0.023f};
    std::string dictionaryName{"DICT_4X4_250"};

    double fx{600.0};
    double fy{600.0};
    double cx{728.0};
    double cy{544.0};

    int minCorners{6};
};

struct FrameKey {
    int module{-1};
    std::uint64_t frame{0};

    bool operator<(const FrameKey& other) const
    {
        if (module != other.module) {
            return module < other.module;
        }

        return frame < other.frame;
    }
};

struct FrameObservations {
    int facet{-1};
    std::map<int, cv::Point2f> cornersById;
};

struct CameraModel {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    bool calibrated{false};
    double calibrationRms{0.0};
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
        << "[--squaresX=7] [--squaresY=5] "
        << "[--squareLength=0.040] [--markerLength=0.020] "
        << "[--dictionary=DICT_5X5_250] "
        << "[--fx=600] [--fy=600] [--cx=728] [--cy=544] "
        << "[--calibrateIntrinsics] "
        << "[--imageWidth=1456] [--imageHeight=1088] "
        << "[--intrinsicsPrefix=camera-intrinsics] "
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

        if (StartsWith(line, "frame,")) {
            continue;
        }

        const std::vector<std::string> fields =
            Split(line, ',');

        if (fields.size() < 6) {
            throw std::runtime_error(
                "Invalid CSV line " + std::to_string(lineNumber) +
                " in " + path +
                ". Expected: frame,module,facet,corner_id,x,y"
            );
        }

        const std::uint64_t frame =
            static_cast<std::uint64_t>(std::stoull(fields[0]));

        const int module =
            std::stoi(fields[1]);

        const int facet =
            std::stoi(fields[2]);

        const int cornerId =
            std::stoi(fields[3]);

        const float x =
            std::stof(fields[4]);

        const float y =
            std::stof(fields[5]);

        FrameKey key{
            .module = module,
            .frame = frame
        };

        FrameObservations& frameObs =
            observations[key];

        frameObs.facet = facet;

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

        std::cout << "[camera initial guess]"
                  << " fx=" << options.fx
                  << " fy=" << options.fy
                  << " cx=" << options.cx
                  << " cy=" << options.cy
                  << " distortion=calibrated_if_requested"
                  << '\n';

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
            << "module,frame,facet,corners,rms_px,"
            << "rvec_x,rvec_y,rvec_z,"
            << "tvec_x,tvec_y,tvec_z\n";

        std::map<int, std::vector<double>> rmsByModule;

        std::size_t attemptedFrames = 0;
        std::size_t solvedFrames = 0;
        std::size_t skippedTooFew = 0;
        std::size_t skippedBadId = 0;
        std::size_t skippedCollinear = 0;
        std::size_t failedPnP = 0;

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

            rmsByModule[key.module].push_back(rms);
            ++solvedFrames;

            poseCsv
                << key.module << ','
                << key.frame << ','
                << frameObs.facet << ','
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
