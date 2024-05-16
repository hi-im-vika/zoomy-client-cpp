//
// Created by Ronal on 5/7/2024.
//

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include "thread"
#include "vector"

class CAutoController {
private:
    cv::Mat *_carImg, *_overheadImg;

    std::vector<int> _autoInput;

    std::vector<bool> _threadExit;

    static void autoTargetThread(CAutoController* ptr);

    static void runToPointThread(CAutoController* ptr);

    void autoTarget();

    void runToPoint();

public:
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

    CAutoController();

    ~CAutoController();

    bool init(cv::Mat *car, cv::Mat *above);

    void startAutoTarget(int id);

    void endAutoTarget();

    void startRunToPoint(cv::Point point, int speed);

    void endRunToPoint();

    int getAutoInput(int type);
};
