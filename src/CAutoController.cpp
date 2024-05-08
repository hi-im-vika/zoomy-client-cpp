//
// Created by Ronal on 5/7/2024.
//
#include "../include/CAutoController.hpp"

CAutoController::CAutoController() = default;

CAutoController::~CAutoController() {
    _threadExit = true;
}

bool CAutoController::InitCAutoController(cv::Mat *car, cv::Mat *above) {
    _carImg = img;
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

}

void CAutoController::startRunToPoint(cv::Point point) {
    _threadExit[1] = false;

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