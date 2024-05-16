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

bool CAutoController::CAutoControllerInit(cv::Mat *car, cv::Mat *above) {
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

}

void CAutoController::runToPoint() {

}

void CAutoController::startAutoTarget(int mark) {
    _threadExit[0] = false;
    std::thread t1(&CAutoController::autoTarget, this);
    t1.detach();
}

void CAutoController::startRunToPoint(cv::Point point) {
    _threadExit[1] = false;
    std::thread t2(&CAutoController::runToPoint, this);
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