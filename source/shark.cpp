#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <time.h>

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

    public:
        void SetTailMotorSpeed(double v) {
            if (v > 0) {
                safeSetPwm(2, v * MAX_SAFE_PWM_VALUE);
                safeSetPwm(3, 0);
            }
            else {
                safeSetPwm(2, 0);
                safeSetPwm(3, (-v) * MAX_SAFE_PWM_VALUE);
            }
        }

        void SetClimbMotorSpeed(double v) {
            if (v > 0) {
                safeSetPwm(12, v * MAX_SAFE_PWM_VALUE);
                safeSetPwm(13, 0);
            }
            else {
                safeSetPwm(12, 0);
                safeSetPwm(13, (-v) * MAX_SAFE_PWM_VALUE);
            }
        }

        SharkController() {
            wiringPiSetup();
            softPwmCreate(2, 0, PWM_RANGE);
            softPwmCreate(3, 0, PWM_RANGE);
            softPwmCreate(12, 0, PWM_RANGE);
            softPwmCreate(13, 0, PWM_RANGE);
        }
};

int main(int argc, char** argv)
{
    VideoCapture cap(0); //capture the video from webcam

    if (!cap.isOpened()) {
        cout << "Cannot open the VideoCapture cap(0)" << endl;
    }
    else {
        cout << "Open VideoCapture cap(0). Success! << endl;
    }

    SharkController sharkController;

    time_t startTimer = time(NULL);

    while(1){
        double timeElapsed = difftime(time(NULL), startTimer);
        cout << timeElapsed << endl;

        Mat frame;
        cap >> frame;
        if (frame.empty())
            break;

        imshow( "Frame", frame );
        waitKey(0);
    }

    //sharkController.SetClimbMotorSpeed(0.5);
    //delay(300);

    cap.release();
    destroyAllWindows();

}



int main2 (void)
{
    for (int i=0; i<1; i++) {
        sharkController.SetClimbMotorSpeed(0.5);
        delay(300);
        sharkController.SetClimbMotorSpeed(0);
        delay(500);
        sharkController.SetClimbMotorSpeed(-0.5);
        delay(300);
        sharkController.SetClimbMotorSpeed(0);
        delay(500);
    }

    return 0 ;
}
