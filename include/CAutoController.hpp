//
// Created by Ronal on 5/7/2024.
//

#include <opencv2/opencv_modules.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include "thread"
#include "vector"

class CAutoController {
private:
    cv::Mat *_carImg, *_overheadImg, _above, _masked_img;

    std::vector<int> _autoInput;

    std::vector<bool> _threadExit;

    cv::Point _destination;
    int _target;
    int _speed;
    std::mutex _imgLock;

    static void autoTargetThread(CAutoController* ptr);
    static void runToPointThread(CAutoController* ptr);
    void autoTarget();
    void runToPoint();

    std::vector<int> _marker_ids;
    std::vector<std::vector<cv::Point2f>> _marker_corners, _rejected_candidates;
    cv::aruco::DetectorParameters _detector_params;
    cv::aruco::Dictionary _dictionary;
    cv::aruco::ArucoDetector _detector;

    cv::Scalar _hsv_threshold_low, _hsv_threshold_high;

public:
    enum controlType {
        MOVE_X,
        MOVE_Y,
        ROTATE,
    };

    struct waypoint {
        cv::Point coordinates;
        int speed;
        int rotation;
        bool turret;
    };

    CAutoController();
    ~CAutoController();

    bool init(cv::Mat *car, cv::Mat *above);
    void startAutoTarget(int id);
    void endAutoTarget();
    void startRunToPoint(cv::Point point, int speed);
    void endRunToPoint();
    int getAutoInput(int type);
    bool isRunning();

    cv::Mat get_masked_image();


    cv::Scalar_<int> get_hsv_threshold_low();
    cv::Scalar_<int> get_hsv_threshold_high();

    void set_hsv_threshold_low(cv::Scalar_<int> &hsv_low);
    void set_hsv_threshold_high(cv::Scalar_<int> &hsv_high);
};
