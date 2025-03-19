//
// Created by Ronal on 5/7/2024.
//
#include "../include/CAutoController.hpp"

#define MOVE_SPEED 255

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
    if (!_overheadImg->empty()) {
        cv::Mat working_copy = _overheadImg->clone();
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point>> contours;

        cv::findContours(working_copy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int biggest = 0;
        cv::Rect car;
        for (const auto & contour : contours) {
            cv::Rect r = boundingRect(contour);
            if ((r.width * r.height) > biggest) {
                biggest = r.width * r.height;
                car = r;
            }
        }

        spdlog::info("P2P ON");

        _autoInput[MOVE_X] = MOVE_SPEED * (_destination.x - (car.x + car.width / 2)) * _speed / 32768.0;
        _autoInput[MOVE_Y] = MOVE_SPEED * (_destination.y - (car.y + car.height / 2)) * _speed / 32768.0;
        _location = cv::Point(car.x + car.width / 2, car.y + car.height / 2);

        spdlog::info("Car location: {:d} {:d}", _location.x, _location.y);

        if (hypot(_destination.x - (car.x + car.width / 2), _destination.y - (car.y + car.height / 2)) <
                ((_speed / 32768.0) * _overheadImg->cols / 5)) {
            _threadExit[1] = true;
        }
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

cv::Point CAutoController::get_car() {
    return _location;
}

cv::Point CAutoController::get_destination() {
    return _destination;
}

void CAutoController::set_mask(cv::Mat arena_mask) {
    _masked_img = arena_mask;
}