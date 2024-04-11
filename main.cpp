#include <iostream>
#include <stdio.h>
#include <softPwm.h>
#include <wiringPi.h>
#include <opencv2/opencv.hpp>

#define PWM_PIN 18 // 定义PWM引脚

int kp, ki, kd;
int offset, error;


using namespace std;

enum State {
    LINE_FOLLOWING;
    IDLE;
};

enum State state;

void setup(void) {
    wiringPiSetupGpio(); // 初始化wiringPi库，使用BCM编号系统
    pinMode(PWM_PIN, OUTPUT); // 配置舵机输出引脚，并设置为默认位置
    digitalWrite(PWM_PIN, LOW);
    softPwmCreate(PWM_PIN, 0, 200); // 初始化软件PWM
    state = IDLE;
}

// calculate
void pidCalc() {

}

void sendCmd() {

}

void moveCamera() {
    softPwmWrite(PWM_PIN, 25);
    delay(1000); // 舵机转到22的位置
    softPwmWrite(PWM_PIN, 15);
    delay(1000); // 舵机转到22的位置
    softPwmWrite(PWM_PIN, 25);
    delay(1000); // 舵机转到22的位置
}

int main()
{
    setup();
    switch (state) {
        case LINE_FOLLOWING:
            pidCalc();
            sendCmd();
            break;

    }

}
