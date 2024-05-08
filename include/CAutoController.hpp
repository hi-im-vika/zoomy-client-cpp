//
// Created by Ronal on 5/7/2024.
//
#include "CZoomyClient.hpp"
#include "thread"
#include "vector"

enum valueType {
    GC_LEFTX,
    GC_LEFTY,
    GC_RIGHTX,
    GC_RIGHTY,
    GC_LTRIG,
    GC_RTRIG,
    GC_A,
    GC_B,
    GC_X,
    GC_Y,
};

class CAutoController {
private:
    cv::Mat *_carImg, *overheadImg;

    std::vector<int> _autoInput;

    std::vector<bool> _threadExit;

    static void autoTargetThread(CAutoController* ptr);

    static void runToPointThread(CAutoController* ptr);

    void autoTarget();

    void runToPoint();

public:
    CAutoController();

    ~CAutoController();

    bool CAutoControllerInit(cv::Mat *car, cv::Mat *above);

    void startAutoTarget(int mark);

    void endAutoTarget();

    void startRunToPoint(cv::Point point);

    void endRunToPoint();

    int getAutoInput(int type);
};
