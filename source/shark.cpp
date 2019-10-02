// #include <wiringPi.h>
// #include <softPwm.h>
// #include <chrono>
// #include <iostream>
// #include <time.h>
// #include <math.h>
// #include <pthread.h>
// #include <unistd.h>

// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc.hpp"

#include "tiny_dnn/tiny_dnn.h"

// using namespace cv;
// using namespace std;

/////
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;

int main() {
    using namespace tiny_dnn;

    network<sequential> net;

    // add layers
    net << tiny_dnn::layers::conv(32, 32, 5, 1, 6) << tiny_dnn::activation::tanh()  // in:32x32x1, 5x5conv, 6fmaps
        << tiny_dnn::layers::ave_pool(28, 28, 6, 2) << tiny_dnn::activation::tanh() // in:28x28x6, 2x2pooling
        << tiny_dnn::layers::fc(14 * 14 * 6, 120) << tiny_dnn::activation::tanh()   // in:14x14x6, out:120
        << tiny_dnn::layers::fc(120, 10);                                           // in:120,     out:10

    assert(net.in_data_size() == 32 * 32);
    assert(net.out_data_size() == 10);

    // load MNIST dataset
    std::vector<label_t> train_labels;
    std::vector<vec_t> train_images;

    parse_mnist_labels("train-labels.idx1-ubyte", &train_labels);
    parse_mnist_images("train-images.idx3-ubyte", &train_images, -1.0, 1.0, 2, 2);

    // declare optimization algorithm
    adagrad optimizer;

    // train (50-epoch, 30-minibatch)
    net.train<mse, adagrad>(optimizer, train_images, train_labels, 30, 50);

    // save
    net.save("net");

    // load
    // network<sequential> net2;
    // net2.load("net");
}
//////

// /*
//   #####                                                                         
//  #     #   ####   #    #  #####  #####    ####   #       #       ######  #####  
//  #        #    #  ##   #    #    #    #  #    #  #       #       #       #    # 
//  #        #    #  # #  #    #    #    #  #    #  #       #       #####   #    # 
//  #        #    #  #  # #    #    #####   #    #  #       #       #       #####  
//  #     #  #    #  #   ##    #    #   #   #    #  #       #       #       #   #  
//   #####    ####   #    #    #    #    #   ####   ######  ######  ######  #    # 
// */
// class SharkController {
//     private:
//         const int PWM_RANGE = 100;
//         //const int PWM_RANGE = 0;
//         const int MAX_SAFE_PWM_VALUE = PWM_RANGE * 0.3;

//         void safeSetPwm(int pin, int value) {
//             if (value > MAX_SAFE_PWM_VALUE) {
//                 value = MAX_SAFE_PWM_VALUE;
//                 std::cout << "WARNING: trying to set pwm higher than safe level" << std::endl;
//             }
//             softPwmWrite(pin, value);
//         }

//         void setTailMotorSpeed(double v) {
//             if (v > 0) {
//                 safeSetPwm(2, v * MAX_SAFE_PWM_VALUE);
//                 safeSetPwm(3, 0);
//             }
//             else {
//                 safeSetPwm(2, 0);
//                 safeSetPwm(3, (-v) * MAX_SAFE_PWM_VALUE);
//             }
//         }

//         void setClimbMotorSpeed(double v) {
//             if (v > 0) {
//                 safeSetPwm(12, v * MAX_SAFE_PWM_VALUE);
//                 safeSetPwm(13, 0);
//             }
//             else {
//                 safeSetPwm(12, 0);
//                 safeSetPwm(13, (-v) * MAX_SAFE_PWM_VALUE);
//             }
//         }

//         double currentSwingAngle = 0;
//         double currentSwingIntensity = 0;
//         double currentClimbSpeed = 0;

//         void updateClimb(double deltaSeconds) {
//             setClimbMotorSpeed(currentClimbSpeed);
//         }

//         double swingTimer = -1;
//         void updateSwingTail(double deltaSeconds) {
//             if (currentSwingIntensity == 0) {
//                 swingTimer = -1;
//             }
//             else {
//                 if (swingTimer < -0.99) {
//                     swingTimer = 0;
//                 }
//                 swingTimer += deltaSeconds;
//                 double leftIntensity = max(0.0, min(1.0, currentSwingIntensity * (1 + currentSwingAngle)));
//                 double rightIntensity = max(0.0, min(1.0, currentSwingIntensity * (1 - currentSwingAngle)));

//                 double loopDuration = 0.35;

//                 if (swingTimer >= loopDuration) {
//                     swingTimer -= loopDuration;
//                 }


//                 if (swingTimer < loopDuration * (0.1 + 0.4 * (1.0+currentSwingAngle))) {
//                     setTailMotorSpeed(leftIntensity);
//                 }
//                 else {
//                     setTailMotorSpeed(-rightIntensity);
//                 }
//             }
//         }

//     public:
//         void Update(double deltaSeconds) {
//             updateClimb(deltaSeconds);
//             updateSwingTail(deltaSeconds);
//         }

//         void SwingTail(double angle, double intensity) {
//             currentSwingAngle = angle;
//             currentSwingIntensity = intensity;
//             updateSwingTail(0);
//         }

//         void Climb(double speed) {
//             currentClimbSpeed = speed;
//             updateClimb(0);
//         }

//         SharkController() {
//             wiringPiSetup();
//             softPwmCreate(2, 0, PWM_RANGE);
//             softPwmCreate(3, 0, PWM_RANGE);
//             softPwmCreate(12, 0, PWM_RANGE);
//             softPwmCreate(13, 0, PWM_RANGE);
//         }
// };
// /*
//   #####                                                  #######                                         
//  #     #   ####   #    #  #####  #####    ####   #          #     #    #  #####   ######    ##    #####  
//  #        #    #  ##   #    #    #    #  #    #  #          #     #    #  #    #  #        #  #   #    # 
//  #        #    #  # #  #    #    #    #  #    #  #          #     ######  #    #  #####   #    #  #    # 
//  #        #    #  #  # #    #    #####   #    #  #          #     #    #  #####   #       ######  #    # 
//  #     #  #    #  #   ##    #    #   #   #    #  #          #     #    #  #   #   #       #    #  #    # 
//   #####    ####   #    #    #    #    #   ####   ######     #     #    #  #    #  ######  #    #  #####  
// */
// pthread_mutex_t lock;
// double target_heading_diff;
// double target_intensity;
// void *shark_control_thread_function(void * data) {
//     SharkController sharkController;
//     while (1) {
//         pthread_mutex_lock(&lock);
//         sharkController.SwingTail(target_heading_diff, target_intensity);
//         pthread_mutex_unlock(&lock);

//         sharkController.Update(0.01);
//         usleep(10000);
//     }
//     return NULL;
// }

// /*
//  ######                                                     
//  #     #  #####    ####   ######  #  #       ######  #####  
//  #     #  #    #  #    #  #       #  #       #       #    # 
//  ######   #    #  #    #  #####   #  #       #####   #    # 
//  #        #####   #    #  #       #  #       #       #####  
//  #        #   #   #    #  #       #  #       #       #   #  
//  #        #    #   ####   #       #  ######  ######  #    # 
// */
// int idx = 0;
// double profiler_timer[10];
// auto prev_time = chrono::high_resolution_clock::now();
// void PROFILER_TIMER(bool is_loop_begin = false) {
//     // if (is_loop_begin) {
//     //     idx = 0;
//     //     prev_time = chrono::high_resolution_clock::now();
//     //     for (int i=0; i<10; i++) {
//     //         cout << profiler_timer[i] << "\t || ";
//     //     }
//     //     cout << endl;
//     // }
//     // else {
//     //     auto currentTime = chrono::high_resolution_clock::now();
//     //     double delta = chrono::duration<double, std::milli>(currentTime - prev_time).count();
//     //     profiler_timer[idx] = profiler_timer[idx] * 0.9 + delta * 0.1;
//     //     prev_time = currentTime;
//     //     idx ++;
//     // }
// }

// /*
//  #     #                    
//  ##   ##    ##    #  #    # 
//  # # # #   #  #   #  ##   # 
//  #  #  #  #    #  #  # #  # 
//  #     #  ######  #  #  # # 
//  #     #  #    #  #  #   ## 
//  #     #  #    #  #  #    # 
// */
// int main(int argc, char** argv) {
//     VideoCapture cap(0); //capture the video from webcam

//     if (!cap.isOpened()) {
//         cout << "Cannot open the VideoCapture cap(0)" << endl;
//     }
//     else {
//         cout << "Open VideoCapture cap(0). Success!" << endl;
//     }
//     cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
//     cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
//     //cout << cap.set(CV_CAP_PROP_CONVERT_RGB, false) << endl;

//     pthread_t tid;
//     pthread_create(&tid, NULL, shark_control_thread_function, NULL);

//     auto startTime = chrono::high_resolution_clock::now();
//     auto currentTime = startTime;
//     auto lastFrameTime = startTime;

//     while(1) {
//         PROFILER_TIMER(1);
//         currentTime = chrono::high_resolution_clock::now();
//         //double deltaSeconds = chrono::duration<double>(currentTime-lastFrameTime).count();
//         //double elapsedSeconds = chrono::duration<double>(currentTime-startTime).count();
//         lastFrameTime = currentTime;

//         PROFILER_TIMER();
//         Mat imgOriginal;
//         cap >> imgOriginal;
//         if (imgOriginal.empty())
//             break;

//         PROFILER_TIMER();
//         flip(imgOriginal, imgOriginal, -1);

//         PROFILER_TIMER();
//         Mat imgHSV;
//         cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

//         PROFILER_TIMER();
//         // vector<Mat> channels(3);
//         //split(imgHSV, channels);
//         Mat channelV(imgHSV.rows, imgHSV.cols, CV_8UC1);
//         int from_to[] = {2, 0};
//         mixChannels(&imgHSV, 1, &channelV, 1, from_to, 1);

//         PROFILER_TIMER();
//         double minVal;
//         double maxVal;
//         Point minLoc;
//         Point maxLoc;
//         //minMaxLoc(channels[2], &minVal, &maxVal, &minLoc, &maxLoc);
//         minMaxLoc(channelV, &minVal, &maxVal, &minLoc, &maxLoc);

//         PROFILER_TIMER();
//         rectangle(imgOriginal, Point(maxLoc.x - 15, maxLoc.y - 15),
//                 Point(maxLoc.x + 15, maxLoc.y + 15), Scalar::all(0), 2, 8, 0 );

//         PROFILER_TIMER();
//         imshow( "Frame", imgOriginal);
//         waitKey(25);

//         PROFILER_TIMER();
//         double light_target = (maxLoc.x/160.0) - 1.0;
//         //cout << light_target << "\t"<< maxVal << endl;

//         pthread_mutex_lock(&lock);
//         target_heading_diff = light_target;
//         target_intensity = 0.5 * (maxVal / 255.0);
//         //std::cout << target_heading_diff << "\t" << target_intensity << std::endl;
//         pthread_mutex_unlock(&lock);
//         PROFILER_TIMER();
//     }

//     cap.release();
//     destroyAllWindows();

// }

