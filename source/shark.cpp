#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>
#include <iostream>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "facedetectcnn.h"

using namespace cv;
using namespace std;

/*
  #####                                                                         
 #     #   ####   #    #  #####  #####    ####   #       #       ######  #####  
 #        #    #  ##   #    #    #    #  #    #  #       #       #       #    # 
 #        #    #  # #  #    #    #    #  #    #  #       #       #####   #    # 
 #        #    #  #  # #    #    #####   #    #  #       #       #       #####  
 #     #  #    #  #   ##    #    #   #   #    #  #       #       #       #   #  
  #####    ####   #    #    #    #    #   ####   ######  ######  ######  #    # 
*/
class SharkController {
    private:
        const int PWM_RANGE = 100;
        //const int PWM_RANGE = 0;
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
        double currentClimbSpeed = 0;

        void updateClimb(double deltaSeconds) {
            setClimbMotorSpeed(currentClimbSpeed);
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
                double leftIntensity = max(0.0, min(1.0, currentSwingIntensity * (1 + currentSwingAngle)));
                double rightIntensity = max(0.0, min(1.0, currentSwingIntensity * (1 - currentSwingAngle)));

                double loopDuration = 0.35;

                if (swingTimer >= loopDuration) {
                    swingTimer -= loopDuration;
                }


                if (swingTimer < loopDuration * (0.1 + 0.4 * (1.0+currentSwingAngle))) {
                    setTailMotorSpeed(leftIntensity);
                }
                else {
                    setTailMotorSpeed(-rightIntensity);
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

        void Climb(double speed) {
            currentClimbSpeed = speed;
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
/*
  #####                                                  #######                                         
 #     #   ####   #    #  #####  #####    ####   #          #     #    #  #####   ######    ##    #####  
 #        #    #  ##   #    #    #    #  #    #  #          #     #    #  #    #  #        #  #   #    # 
 #        #    #  # #  #    #    #    #  #    #  #          #     ######  #    #  #####   #    #  #    # 
 #        #    #  #  # #    #    #####   #    #  #          #     #    #  #####   #       ######  #    # 
 #     #  #    #  #   ##    #    #   #   #    #  #          #     #    #  #   #   #       #    #  #    # 
  #####    ####   #    #    #    #    #   ####   ######     #     #    #  #    #  ######  #    #  #####  
*/
pthread_mutex_t lock;
double target_turn_rate;
double target_intensity;
void *shark_control_thread_function(void * data) {
    SharkController sharkController;
    while (1) {
        pthread_mutex_lock(&lock);
        sharkController.SwingTail(target_turn_rate, target_intensity);
        pthread_mutex_unlock(&lock);

        sharkController.Update(0.01);
        usleep(10000);
    }
    return NULL;
}

/*
 ######                                                     
 #     #  #####    ####   ######  #  #       ######  #####  
 #     #  #    #  #    #  #       #  #       #       #    # 
 ######   #    #  #    #  #####   #  #       #####   #    # 
 #        #####   #    #  #       #  #       #       #####  
 #        #   #   #    #  #       #  #       #       #   #  
 #        #    #   ####   #       #  ######  ######  #    # 
*/
int idx = 0;
double profiler_timer[10];
auto prev_time = chrono::high_resolution_clock::now();
void PROFILER_TIMER(bool is_loop_begin = false) {
    if (is_loop_begin) {
        idx = 0;
        prev_time = chrono::high_resolution_clock::now();
        for (int i=0; i<10; i++) {
            cout << profiler_timer[i] << "\t || ";
        }
        cout << endl;
    }
    else {
        auto currentTime = chrono::high_resolution_clock::now();
        double delta = chrono::duration<double, std::milli>(currentTime - prev_time).count();
        profiler_timer[idx] = profiler_timer[idx] * 0.9 + delta * 0.1;
        prev_time = currentTime;
        idx ++;
    }
}

/*
 #     #                    
 ##   ##    ##    #  #    # 
 # # # #   #  #   #  ##   # 
 #  #  #  #    #  #  # #  # 
 #     #  ######  #  #  # # 
 #     #  #    #  #  #   ## 
 #     #  #    #  #  #    # 
*/
int main(int argc, char** argv) {
    VideoCapture cap(0); //capture the video from webcam

    if (!cap.isOpened()) {
        cout << "Cannot open the VideoCapture cap(0)" << endl;
    }
    else {
        cout << "Open VideoCapture cap(0). Success!" << endl;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH,128);//320x240
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,96);
    cap.set(CV_CAP_PROP_BUFFERSIZE, 1);
    //cout << cap.set(CV_CAP_PROP_CONVERT_RGB, false) << endl;

    pthread_t tid;
    pthread_create(&tid, NULL, shark_control_thread_function, NULL);

    // auto startTime = chrono::high_resolution_clock::now();
    // auto currentTime = startTime;
    // auto lastFrameTime = startTime;

    while(1) {
        PROFILER_TIMER(1);
        Mat imgOriginal;
        cap >> imgOriginal;
        if (imgOriginal.empty())
            break;

        PROFILER_TIMER();
        flip(imgOriginal, imgOriginal, -1);

        // PROFILER_TIMER();
        // Mat imgHSV;
        // cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

        // PROFILER_TIMER();
        // Mat channelV(imgHSV.rows, imgHSV.cols, CV_8UC1);
        // int from_to[] = {2, 0};
        // mixChannels(&imgHSV, 1, &channelV, 1, from_to, 1);

        // PROFILER_TIMER();
        // double minVal;
        // double maxVal;
        // Point minLoc;
        // Point maxLoc;
        // minMaxLoc(channelV, &minVal, &maxVal, &minLoc, &maxLoc);

        // PROFILER_TIMER();
        // rectangle(imgOriginal, Point(maxLoc.x - 15, maxLoc.y - 15),
        //         Point(maxLoc.x + 15, maxLoc.y + 15), Scalar::all(0), 2, 8, 0 );

        PROFILER_TIMER();
        int *pResults = NULL;
        unsigned char *pBuffer = (unsigned char *)malloc(0x20000);
        if (!pBuffer)
        {
            cout << "Failed to allocate buffer." << endl;
            return -1;
        }

        PROFILER_TIMER();
        pResults = facedetect_cnn(pBuffer, (unsigned char *)(imgOriginal.ptr(0)), imgOriginal.cols, imgOriginal.rows, (int)imgOriginal.step);

        PROFILER_TIMER();
        printf("%d faces detected.\n", (pResults ? *pResults : 0));
        for (int i = 0; i < (pResults ? *pResults : 0); i++)
        {
            short *p = ((short *)(pResults + 1)) + 142 * i;
            int x = p[0];
            int y = p[1];
            int w = p[2];
            int h = p[3];
            int confidence = p[4];
            int angle = p[5];

            //printf("face_rect=[%d, %d, %d, %d], confidence=%d, angle=%d\n", x, y, w, h, confidence, angle);
            rectangle(imgOriginal, Rect(x, y, w, h), Scalar(0, 255, 0), 2);
        }

        PROFILER_TIMER();
        imshow("Frame", imgOriginal);
        waitKey(25);

        PROFILER_TIMER();
        //double face_target = (maxLoc.x / 160.0) - 1.0;

        pthread_mutex_lock(&lock);
        //target_turn_rate = face_target;
        //target_intensity = 0.5 * (maxVal / 255.0);
        //std::cout << target_turn_rate << "\t" << target_intensity << std::endl;
        pthread_mutex_unlock(&lock);
        PROFILER_TIMER();
    }

    cap.release();
    destroyAllWindows();

}

