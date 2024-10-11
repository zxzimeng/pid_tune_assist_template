#ifndef PID_TUNE_H
#define PID_TUNE_H
#include "queue"
#include "../main.h"
#include "../lemlib/api.hpp"

namespace pid_tune_assist {
template <typename T>
class Cycle {
private:
    std::queue<T> values;

public:
    T current;
    explicit Cycle(T arr[]);
    T next();
};

struct PidTaskParams {
    pros::Controller* master; // Add master to the struct
    lemlib::PID* lateralPID;
    lemlib::PID* angularPID;
    Cycle<float>* headings;
    Cycle<float>* lengths;
    bool* isLateralPID;
};


class PidTune {
private:
    pros::Controller& master; // Add master controller reference
    lemlib::Chassis& chassis;
    Cycle<float> headings;
    Cycle<float> lengths;
    bool isLateralPID;

    static void handlePidTask(void* param);
    void handle_set_line_cols(int row, int col, const std::string &content); // Make this static

public:
    PidTune(pros::Controller& master, lemlib::Chassis& chassis, float headingsArr[], float lengthsArr[]); // Include master in constructor
    [[noreturn]] void run();
};
}
#endif //PID_TUNE_H