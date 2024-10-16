#include "pid_tune_assist/pid_tune_assist.h"
#include "lemlib/api.hpp"

namespace pid_tune_assist {
    // Cycle class template implementation
    template <typename T>
    Cycle<T>::Cycle(T arr[]) {
        // Calculate the size of the array
        int size = sizeof(arr) / sizeof(arr[0]);

        // Initialize the queue with the array elements
        for (int i = 0; i < size; ++i) {
            values.push(arr[i]);
        }
        current = arr[0];
    }

    template <typename T>
    T Cycle<T>::next() {
        T val = values.front();
        values.pop();
        values.push(val);
        current = val;
        return val;
    }

    // PidTune class implementation
    PidTune::PidTune(pros::Controller& master, lemlib::Chassis& chassis, float headingsArr[], float lengthsArr[]) :
        master(master), // Initialize master
        chassis(chassis),
        headings(headingsArr),
        lengths(lengthsArr),
        isLateralPID(true)
    {
    }

    [[noreturn]] void PidTune::run() {
        // Create the task and pass the `this` pointer
        PidTaskParams taskParams{};
        taskParams.master = &master;
        taskParams.lateralPID = &chassis.lateralPID;
        taskParams.angularPID = &chassis.angularPID;
        taskParams.isLateralPID = &isLateralPID;
        taskParams.headings = &headings;
        taskParams.lengths = &lengths;

        pros::Task debugTask(handlePidTask, &taskParams, "PID Controller Task");


        while (true) {
            // Display information in quadrants
            handle_set_line_cols(0, 0, "kP: " + std::to_string(isLateralPID ? chassis.lateralPID.kP : chassis.angularPID.kP));
            handle_set_line_cols(0, 1, "kD: " + std::to_string(isLateralPID ? chassis.lateralPID.kD : chassis.angularPID.kD));
            handle_set_line_cols(1, 0, "kI: " + std::to_string(isLateralPID ? chassis.lateralPID.kI : chassis.angularPID.kI));
            handle_set_line_cols(1, 1, isLateralPID?("L: " + std::to_string(lengths.current)):("H: " + std::to_string(headings.current)));

            // Display appropriate position value
            handle_set_line_cols(2, 0, isLateralPID ?  "Y: " + std::to_string(chassis.getPose().y) :
                                                             "Theta: " + std::to_string(chassis.getPose().theta));

            pros::delay(10);
        }
    }

    void PidTune::handlePidTask(void* param) {
        auto* pidTune = static_cast<PidTune*>(param);  // Access the PidTune instance

        pros::Controller& master = *(static_cast<PidTaskParams*>(param)->master);
        lemlib::PID& lateralPID = pidTune->chassis.lateralPID;
        lemlib::PID& angularPID = pidTune->chassis.angularPID;
        bool& isLateralPID_l = pidTune->isLateralPID;
        Cycle<float>& headings_l = pidTune->headings;
        Cycle<float>& lengths_l = pidTune->lengths;

        lemlib::PID active_pid = lateralPID;
        float dx = 1;

        while (true) {
            // Check for button B press
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                isLateralPID_l
                    ? pidTune->chassis.moveToPose({0, lengths_l.current, 0}, 5000, {}, true)
                    : pidTune->chassis.turnToHeading(headings_l.current, 5000, {}, true);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                isLateralPID_l
                    ? lengths_l.next()
                    : headings_l.next();
            }
            // Check for button A press
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                pidTune->chassis.cancelAllMotions();
                pidTune->chassis.cancelMotion();
            }

            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                isLateralPID_l = !isLateralPID_l;
                active_pid = isLateralPID_l ? lateralPID : angularPID;
            }

            // Handle other button presses
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                active_pid.kP += dx;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                active_pid.kP -= dx;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                active_pid.kI -= dx;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                active_pid.kI += dx;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
                active_pid.kD += dx;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                active_pid.kD -= dx;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                dx /= 10;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                dx *= 10;
            }

            pros::delay(10);
        }
    }

    void PidTune::handle_set_line_cols(int row, int col, const std::string &content) {
        // Calculate starting position for each quadrant
        int starting_pos = col * 7;

        master.clear_line(row); // Access the global master controller
        pros::delay(100);
        master.set_text(row, starting_pos, content);
        pros::delay(100);
    }
}