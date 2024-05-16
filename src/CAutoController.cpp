//
// Created by Ronal on 5/7/2024.
//
#include "../include/CAutoController.hpp"

#define MOVE_SPEED 255
#define HSV_L cv::Scalar(0, 80, 180)
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