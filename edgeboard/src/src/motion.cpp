#include <fstream>
#include <iostream>
#include <cmath>
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter.cpp"
#include "recognition/ring.cpp"
#include "FuzzyPID.cpp"
using namespace std;
using namespace cv;

/**
 * @brief 运动控制器
 *
 */
class Motion
{
private:
    int countShift = 0; // 变速计数器
    std::vector<double> errors;
    //FuzzyPID fuzzy;

public:
    /**
     * @brief 初始化：加载配置文件
     *
     */
    Motion()
    {
        string jsonPath = "../src/config/config.json";
        std::ifstream config_is(jsonPath);//打开json文件
        if (!config_is.good())
        {
            std::cout << "Error: Params file path:[" << jsonPath
                      << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }

        speed = params.speedLow;
        //FuzzyPID pid;
    };

    /**
     * @brief 控制器核心参数
     *
     */
    struct Params
    {
        float up_speed=0;
        float down_speed=0;
        float speedLow = 0.8;                              // 智能车最低速
        float speedHigh = 0.8;                             // 智能车最高速
        float speedBridge = 0.6;                           // 坡道速度
        float speedDown = 0.5;                             // 特殊区域降速速度
        float danger_speed=0.0;
        bool ring_change=true;
        float ringspeed=0.0;
        float ringp=0;
        float runP1 = 0;                                 // 一阶比例系数：直线控制量
        float danger_P1 = 0.0;
        float danger_P2 = 0;
        float danger_P3 = 0.0;                                 // 三阶比例系数：弯道控制量
        float runP4=0;
        float runP=0.0;
        float prospect=0.0;
        int danger_time=0;
        float danger_prospect=0.0;
        float rescue_prospect=0.0;
        bool rescue_switch=false;           
        bool rescue_change=false;    
        int  rescue_time=0;
        int rescue_pwm=0;
        int left_rescue_enable=0;
        int right_rescue_enable=0;
        int rescue_exit=0;
        float turnD1 = 0;
        float turnD2= 0;
        float turnD3 = 0;                                 // 一阶微分系数：转弯控制量
        float turnD=0;
        int bridge_time=0;
        int ring_time=0;
        int park_time=0;
        int cross_time=0;
        bool debug = true;                                // 调试模式使能
        bool saveImg = false;                              // 存图使能
        uint16_t rowCutUp = 10;                            // 图像顶部切行
        uint16_t rowCutBottom = 10;                        // 图像低部切行
        bool bridge = true;                                // 坡道区使能
        bool danger = true;                                // 危险区使能
        bool rescue = true;                                // 救援区使能
        bool racing = true;                                // 追逐区使能
        bool parking = true;                               // 停车区使能
        bool debug_fps=true;                               //帧率测试使能
        bool debug_uart=true;                              //串口发送使能
        bool ring = true;                                  // 环岛使能
        bool cross = true;                                 // 十字道路使能
        float score = 0.5;                                 // AI检测置信度
        string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
        string video = "../res/samples/sample.mp4";          // 视频路径
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params,up_speed,down_speed, speedLow, speedHigh, speedBridge, speedDown, danger_speed,rescue_switch,rescue_change,
        rescue_time,ring_change,ringspeed,ringp,runP1, 
                                        danger_time,danger_P1,danger_P2,danger_P3,runP4,runP,prospect,danger_prospect,rescue_prospect,rescue_pwm,
                                        left_rescue_enable,rescue_exit,right_rescue_enable,turnD1,turnD2,
                                        turnD3,ring_time, bridge_time,turnD, cross_time,debug, saveImg, rowCutUp, rowCutBottom, bridge, danger,
                                       rescue, racing, parking, debug_fps,debug_uart,ring, cross, score, model, video); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
    float speed = 0.3;               // 发送给电机的速度
    int pwm_Diff;
    int errp_ppre = 0;  
    /**
     * @brief 姿态PD控制器
     *
     * @param controlCenter 智能车控制中心
     */
    void poseCtrl(int controlCenter,Ring &ring1)
    {
        float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
                //std::cout<<error<<std::endl;
        static int errorLast = 0;                    // 记录前一次的偏差    
        if (abs(error - errorLast) > COLSIMAGE / 10)
        {
            error = error > errorLast ? errorLast + COLSIMAGE / 10 : errorLast - COLSIMAGE / 10;
        }
        // 每收集10个误差样本后进行一次PID参数的自适应调
        // if(speed==params.speedLow){
        //     errors.push_back(std::abs(error));
        //     if (errors.size() >= 10) {
        //         adaptPIDGains();
        //     }
        // }
        // std::cout<<"\tp"<<params.runP1;
        // std::cout<<"\tp2"<<params.runP2;
        // std::cout<<"\td"<<params.turnD1<<std::endl;
        if(speed==params.danger_speed){
            int pwmDiff = (error * params.danger_P1) + (error - errorLast) * params.turnD3;
            if(abs(pwmDiff)>320){
                if(pwmDiff>0)
                    pwmDiff=320;
                else
                    pwmDiff=-320;
            }
             servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff);
        }
        else if(speed==params.speedHigh){ 
            params.runP = params.runP1;
            int pwmDiff = (error * params.runP) + (error - errorLast) * params.turnD1;
            if(abs(pwmDiff)>320){
                //speed=params.speedLow-0.3;
                if(pwmDiff>0)
                    pwmDiff=320;
                else
                    pwmDiff=-320;
            }
            servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff); 
        }
        else if(speed==params.ringspeed&&ring1.ringStep>1){
            if(ring1.ringType==1)
                params.runP = params.ringp-0.3;
            else
                params.runP = params.ringp;
            int pwmDiff = (error * params.runP);
            if(abs(pwmDiff)>320){
                //speed=params.speedLow-0.3;
                if(pwmDiff>0)
                    pwmDiff=320;
                else
                    pwmDiff=-320;
            }
            servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff); 
        }
        else{
            params.runP =params.runP4;
            int pwmDiff = (error * params.runP) + (error - errorLast) * params.turnD2;
            // if(abs(pwmDiff)>120){
            //     speed=params.speedLow-0.2;}
            // if(abs(pwmDiff)>220){
            //     speed=params.speedLow-0.3;}
             if(abs(pwmDiff)>320){
            //     speed=params.speedLow-0.2;
                if(pwmDiff>0)
                    pwmDiff=320;
                else
                    pwmDiff=-320;
            }
            servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff); 
        }
        //std::cout<<servoPwm<<std::endl;
        errp_ppre=errorLast;
        errorLast = error;
    }

    /**
     * @brief 变加速控制
     *
     * @param enable 加速使能
     * @param control
     */
    void speedCtrl(bool enable, bool slowDown, ControlCenter control)
    {
        // 控制率
        uint8_t controlLow = 0;   // 速度控制下限
        uint8_t controlMid = 3;   // 控制率
        uint8_t controlHigh = 5; // 速度控制上限

        if (slowDown)
        {
            countShift = controlLow;
            speed = params.speedDown;
        }
        else if (enable) // 加速使能
        {
            if (control.centerEdge.size() < 10)
            {
                speed = params.speedLow;
                countShift = controlLow;
                return;
            }
            if (abs(control.sigmaCenter) <50.0)
            {
                countShift++;
                if (countShift > controlHigh)
                    countShift = controlHigh;
            }
            else if (abs(control.sigmaCenter)>2000.0)
            {
                speed = params.speedLow;
                return;
            }
            else
            {
                countShift--;
                if (countShift < controlLow)
                    countShift = controlLow;
            }

            if (countShift > controlMid)
                speed = params.speedHigh;
            else
                speed = params.speedLow;
        }
        else
        {
            countShift = controlLow;
            speed = params.speedLow;
        }
    }
    float speed_control(float target_speed,float now_speed){

        if(target_speed>now_speed-0.1){
            float real_speed=now_speed;
            if(now_speed>0)
                real_speed=now_speed*params.up_speed;
            else
                real_speed=1;
            if(real_speed>target_speed)
                real_speed=target_speed;
            return real_speed;
        }
        else{
            if(target_speed>0){
                if(target_speed==params.danger_speed||target_speed==params.speedDown)
                    target_speed=params.down_speed;
            }
            return target_speed;
        }
    }
};
