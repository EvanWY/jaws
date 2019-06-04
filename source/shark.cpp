#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>
#include <iostream>
#include <time.h>
#include <math.h>
#include <pthread.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

class SharkController {
    private:
        const int PWM_RANGE = 100;
        const int MAX_SAFE_PWM_VALUE = PWM_RANGE * 0.3;

        void safeSetPwm(int pin, int value) {
            if (value > MAX_SAFE_PWM_VALUE) {
                value = MAX_SAFE_PWM_VALUE;
                std::cout << "WARNING: trying to set pwm higher than safe level" << std::endl;
            }
            softPwmWrite(pin, value);
        }

        void setTailMotorSpeed(double v) {
            if (v > 0) {
                safeSetPwm(2, v * MAX_SAFE_PWM_VALUE);
                safeSetPwm(3, 0);
            }
            else {
                safeSetPwm(2, 0);
                safeSetPwm(3, (-v) * MAX_SAFE_PWM_VALUE);
            }
        }

        void setClimbMotorSpeed(double v) {
            if (v > 0) {
                safeSetPwm(12, v * MAX_SAFE_PWM_VALUE);
                safeSetPwm(13, 0);
            }
            else {
                safeSetPwm(12, 0);
                safeSetPwm(13, (-v) * MAX_SAFE_PWM_VALUE);
            }
        }

        double currentSwingAngle = 0;
        double currentSwingIntensity = 0;
        double currentClimbDelta = 0;

        void updateClimb(double deltaSeconds) {
            if (currentClimbDelta == 0) {
                setClimbMotorSpeed(0);
            }
            else if (deltaSeconds >= abs(currentClimbDelta)) {
                setClimbMotorSpeed(0);
                currentClimbDelta = 0;
            }
            else {
                if (currentClimbDelta > 0) {
                    setClimbMotorSpeed(1);
                    currentClimbDelta -= deltaSeconds;
                }
                else {
                    setClimbMotorSpeed(-1);
                    currentClimbDelta += deltaSeconds;
                }
            }
        }

        double swingTimer = -1;
        void updateSwingTail(double deltaSeconds) {
            if (currentSwingIntensity == 0) {
                swingTimer = -1;
            }
            else {
                if (swingTimer < -0.99) {
                    swingTimer = 0;
                }
                swingTimer += deltaSeconds;
                double leftIntensity = min(1.0, currentSwingIntensity * (1 + currentSwingAngle));
                double rightIntensity = min(1.0, currentSwingIntensity * (1 - currentSwingAngle));

                if (swingTimer >= 0.8) {
                    swingTimer -= 0.8;
                }

                if (swingTimer < 0.2) {
                    setTailMotorSpeed(leftIntensity);
                }
                else if (swingTimer < 0.4) {
                    setTailMotorSpeed(0);
                }
                else if (swingTimer < 0.6) {
                    setTailMotorSpeed(-rightIntensity);
                }
                else if (swingTimer < 0.8) {
                    setTailMotorSpeed(0);
                }
            }
        }

    public:
        void Update(double deltaSeconds) {
            updateClimb(deltaSeconds);
            updateSwingTail(deltaSeconds);
        }

        void SwingTail(double angle, double intensity) {
            currentSwingAngle = angle;
            currentSwingIntensity = intensity;
            updateSwingTail(0);
        }

        void Climb(double value) {
            currentClimbDelta += value;
            updateClimb(0);
        }

        SharkController() {
            wiringPiSetup();
            softPwmCreate(2, 0, PWM_RANGE);
            softPwmCreate(3, 0, PWM_RANGE);
            softPwmCreate(12, 0, PWM_RANGE);
            softPwmCreate(13, 0, PWM_RANGE);
        }
};

void *shark_control_thread_function() {
    sleep(1);
    cout << "hi from shark control thread" << endl;
    return NULL;
}

int main(int argc, char** argv) {
    VideoCapture cap(0); //capture the video from webcam

    if (!cap.isOpened()) {
        cout << "Cannot open the VideoCapture cap(0)" << endl;
    }
    else {
        cout << "Open VideoCapture cap(0). Success!" << endl;
    }

    SharkController sharkController;

    auto startTime = chrono::high_resolution_clock::now();
    auto lastFrameTime = startTime;
    auto currentTime = startTime;

    sharkController.SwingTail(0.5, 1);
    sharkController.Climb(-1);

    while(1){
        currentTime = chrono::high_resolution_clock::now();

        double secsElapsed = chrono::duration<double>(currentTime-startTime).count();
        cout.precision(17);
        cout << fixed << secsElapsed << endl;

        sharkController.Update(chrono::duration<double>(currentTime - lastFrameTime).count());

        Mat frame;
        cap >> frame;
        if (frame.empty())
            break;

        //imshow( "Frame", frame );
        //waitKey(25);
        //
        lastFrameTime = currentTime;
    }

    //sharkController.SetClimbMotorSpeed(0.5);
    //delay(300);

    cap.release();
    destroyAllWindows();

}

