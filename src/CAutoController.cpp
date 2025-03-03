//
// Created by Ronal on 5/7/2024.
//
#include "../include/CAutoController.hpp"

#define MOVE_SPEED 255
#define HSV_L cv::Scalar(0, 130, 100)
#define HSV_H cv::Scalar(22, 255, 255)

CAutoController::CAutoController() = default;

CAutoController::~CAutoController() {
    _threadExit = std::vector<bool>(2,true);
}

bool CAutoController::init(cv::Mat *car, cv::Mat *above) {
    _threadExit = std::vector<bool>(2,true);
    _autoInput = std::vector<int>(4,0);
    _carImg = car;
    _overheadImg = above;
    _masked_img = cv::Mat::ones(cv::Size(600,600), CV_8UC3);
    _hsv_threshold_low = cv::Scalar_<int>(0,130,100);
    _hsv_threshold_high = cv::Scalar_<int>(22,255,255);
    return true;
}

void CAutoController::autoTargetThread(CAutoController* ptr) {
    while (!ptr->_threadExit[0]) {
        ptr -> autoTarget();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void CAutoController::runToPointThread(CAutoController* ptr) {
    while (!ptr->_threadExit[1]) {
        ptr -> runToPoint();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void CAutoController::autoTarget() {
    if (!_carImg->empty()) {
        _detector_params = cv::aruco::DetectorParameters();
        _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        _detector.setDetectorParameters(_detector_params);
        _detector.setDictionary(_dictionary);
        _detector.detectMarkers(*_carImg, _marker_corners, _marker_ids, _rejected_candidates);
    }
    if (!_carImg->empty()) {
        cv::aruco::drawDetectedMarkers(*_carImg, _marker_corners, _marker_ids);
        for (int i = 0; i < _marker_ids.size(); i++) {
            if (_marker_ids.at(i) == _target) {
                _autoInput[ROTATE] =
                        -((_carImg->size().width / 2) - ((_marker_corners[i][0].x - _marker_corners[i][1].x) / 2)) * 32768.0 / _carImg->size().width;
            }
        }
    }
}

void CAutoController::runToPoint() {
    if (!_overheadImg->empty()) {
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> contour;

//        _imgLock.lock();
        cv::cvtColor(*_overheadImg, _above, cv::COLOR_BGR2HSV);
        cv::dilate(_above, _above, cv::Mat());
        cv::inRange(_above, _hsv_threshold_low, _hsv_threshold_high, _above);
//        _above.convertTo(_above, CV_8UC1);
//        cv::findContours(_above, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//        _imgLock.unlock();

        _masked_img = _above;

//        int biggest = 0;
//        cv::Rect car;
//        for (int i = 0; i < contours.size(); i++) {
//            cv::Rect r = boundingRect(contours.at(i));
//            if ((r.width * r.height) > biggest) {
//                biggest = r.width * r.height;
//                car = r;
//            }
//        }

        spdlog::info("P2P ON");

//        _autoInput[MOVE_X] = MOVE_SPEED * (_destination.x - (car.x + car.width / 2)) * _speed / 32768.0;
//        _autoInput[MOVE_Y] = MOVE_SPEED * (_destination.y - (car.y + car.height / 2)) * _speed / 32768.0;
//
//        std::cout << cv::Point((car.x + car.width / 2), (car.y + car.height / 2)) << std::endl;
//        cv::circle(*_overheadImg, cv::Point((car.x + car.width / 2), (car.y + car.height / 2)), _overheadImg->cols / 60,
//                   cv::Scalar(0, 255, 0), -1);
//        cv::circle(*_overheadImg, _destination, _overheadImg->cols / 40, cv::Scalar(255, 0, 0), -1);
//
//        if (hypot(_destination.x - (car.x + car.width / 2), _destination.y - (car.y + car.height / 2)) <
//                ((_speed / 32768.0) * _overheadImg->cols / 5)) {
//            _threadExit[1] = true;
//        }
//        _threadExit[1] = true;
    }
}

void CAutoController::startAutoTarget(int id) {
    _target = id;
    _threadExit[0] = false;
    std::thread t1(&CAutoController::autoTargetThread, this);
    t1.detach();
}

void CAutoController::startRunToPoint(cv::Point point, int speed) {
    _threadExit[1] = false;
    _speed = speed;
    _destination = point;
    std::thread t2(&CAutoController::runToPointThread, this);
    t2.detach();
}

void CAutoController::endAutoTarget() {
    _threadExit[0] = true;
}

void CAutoController::endRunToPoint() {
    _threadExit[1] = true;
}

int CAutoController::getAutoInput(int type) {
    return _autoInput[type];
}

bool CAutoController::isRunning() {
    return !_threadExit[1];
}

cv::Mat CAutoController::get_masked_image() {
    return _masked_img;
}

cv::Scalar_<int> CAutoController::get_hsv_threshold_low() {
    return _hsv_threshold_low;
}

cv::Scalar_<int> CAutoController::get_hsv_threshold_high() {
    return _hsv_threshold_high;
}

void CAutoController::set_hsv_threshold_low(cv::Scalar_<int> &hsv_low) {
    _hsv_threshold_low = hsv_low;
}

void CAutoController::set_hsv_threshold_high(cv::Scalar_<int> &hsv_high) {
    _hsv_threshold_high = hsv_high;
}