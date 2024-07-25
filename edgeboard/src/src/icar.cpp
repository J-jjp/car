/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file icar.cpp
 * @author Leo
 * @brief 智能汽车-顶层框架（TOP）
 * @version 0.1
 * @date 2023-12-25
 * @copyright Copyright (c) 2024
 *
 */
#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "controlcenter.cpp"         //控制中心计算类
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/danger.cpp"      //AI检测：危险区
#include "detection/parking.cpp"     //AI检测：停车区
#include "detection/racing.cpp"      //AI检测：追逐区
#include "detection/rescue.cpp"      //AI检测：救援区
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
using namespace std;
using namespace cv;
cv::Scalar getCvcolor(int index);
    void drawBox(Mat &img);
Mat ai_image;
Mat save_age;
bool start_ai=false;
bool start_uart=false;
bool save=false;
std::thread ai_thread;
std::thread receive_thread;
std::thread send_thread;
std::thread save_thread;
std::timed_mutex  mut;
std::mutex mut_image;
std::vector<PredictResult> result;
float speed=1;
Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
void thread_ai(std::shared_ptr<Detection> detection1){
  while(1){
    //std::cout<<"进去"<<std::endl;
    if(start_ai){
      mut.lock();
      Mat ii=ai_image.clone();
      start_ai=false;
      mut.unlock();
      detection1->inference(ii);
      mut.lock();
      result=detection1->results;
      mut.unlock();
    //std::cout<<"进去2"<<std::endl;
    }
    else{
       waitKey(1);
    }
  }
}
void Send(std::shared_ptr<Uart> uart1,Motion& motion1) {
  // 启动串口接收子线程
  if (!uart1->isOpen) // 串口是否正常打开
    return;
  while (1) {
    if(start_uart){
      uart1->carControl(motion1.speed, motion1.servoPwm);
      //std::cout<<"进去了"<<motion1.servoPwm<<std::endl;
      start_uart=false;
    }
    else{
      //std::cout<<"进去了"<<std::endl;
      //uart1->receiveCheck();
      waitKey(1);
    }
  }
}
void Receive(std::shared_ptr<Uart> uart1) {
    if (!uart1->isOpen) // 串口是否正常打开
      return;
    // 启动串口接收子线程
      while (1) {
          uart1->receiveCheck(); // 串口接收校验
      }
}
void Save_image(){
    while(1){
      //std::cout<<"进去"<<std::endl;
      if(save){
        string str;
        Mat kk;
        mut_image.lock();
        kk=save_age.clone();
        save=false;
        str = "speed: " + formatDoble2String(speed, 1);
                putText(kk, str, Point(COLSIMAGE - 280, 2 * 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
        str = "scenc: " + formatDoble2String(scene, 1);
                putText(kk, str, Point(COLSIMAGE - 200, 7 * 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
        mut_image.unlock();
        savePicture(kk);
      }
      else{
        waitKey(1);
      }
  }
}
int main(int argc, char const *argv[]) {
  Preprocess preprocess;    // 图像预处理类
  Motion motion;            // 运动控制类
  Tracking tracking;        // 赛道识别类
  Crossroad crossroad;      // 十字道路识别类
  Ring ring;                // 环岛识别类
  Bridge bridge;            // 坡道区检测类
  Parking parking;          // 停车区检测类
  Danger danger;            // 危险区检测类
  Rescue rescue;            // 救援区检测类
  Racing racing;            // 追逐区检测类
  ControlCenter ctrlCenter; // 控制中心计算类
  Display display(2);       // 初始化UI显示窗口
  VideoCapture capture;     // Opencv相机类
  // 目标检测类(AI模型文件)
  shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
  detection->score = motion.params.score; // AI检测置信度
  // USB转串口初始化： /dev/ttyUSB0
  shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
  int ret = uart->open();
  if (ret != 0) {
    printf("[Error] Uart Open failed!\n");
    return -1;
  }
  //uart->startReceive(); // 启动数据接收子线程
  receive_thread=std::thread(Receive,uart);
  //save_thread=std::thread(Save_image);
  if (!motion.params.debug) {  
    ai_thread = std::thread(thread_ai, detection); // 在这里初始化 ai_thread 对象  
  }  
  //send_thread=std::thread(Send,uart,std::ref(motion));
  // USB摄像头初始化
  if (0)
    capture = VideoCapture(motion.params.video); // 打开本地视频
  else
    capture = VideoCapture(0,cv::CAP_V4L2); // 打开摄像头
  if (!capture.isOpened()) {
    printf("can not open video device!!!\n");
    return 0;
  }
  else{
    printf("can open video\n");
  }
  VideoWriter vw;
  int fourcc = vw.fourcc('M','J','P','G');//设置摄像头编码方式
  capture.set(CAP_PROP_FOURCC,fourcc);
  capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
  capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率
  capture.set(CAP_PROP_FPS,120);
  std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();  
  int frame_count = 0;  
  double fps = 0.0; 
  int prak_count=0;
  int rescue_count=0;
  bool danger_enable=false;
  int rescue_n=0;
  motion.params.danger=false;
  bool ring_enable=false;
  int ring_cout=0;
  bool ring_dirle=false;
  // 等待按键发车
  if (0) {
    printf("--------------[等待按键发车!]-------------------\n");
    while (!uart->keypress)
      waitKey(300);
    while (ret < 10) // 延时3s
    {
      uart->carControl(0, PWMSERVOMID); // 通信控制车辆停止运动
      waitKey(300);
      ret++;
    }
    uart->keypress = false;
  }
  long preTime;
  Mat img;
  bool park=false;
  int park_n=0;
  int time=0;
  float danger_hightspeed=1.8;
  bool danger_section1=false;
  bool danger_section2=false;
  while (1) {
    //[01] 视频源读取
    if (motion.params.debug_fps) // 综合显示调试UI窗口
      preTime = chrono::duration_cast<chrono::milliseconds>(
                    chrono::system_clock::now().time_since_epoch())
                    .count();
    if (!capture.read(img))
       continue;
    if (0)//如果不是debug模式,则旋转摄像头
      flip(img, img, -1);
    // //[02] 图像预处理
    Mat imgCorrect = img.clone();
     if (mut.try_lock()) {  
      ai_image=img.clone();
      start_ai=true;
      mut.unlock();  
    }
    Mat imgBinary;
    imgBinary = preprocess.binaryzation(img); // 图像二值化
    // cv::warpPerspective(imgBinary, imgBinary, H_inv, img.size());
    // cv::warpPerspective(imgCorrect, imgCorrect, H_inv, img.size());  
    // //[03] 启动AI推理
    if(motion.params.debug){
       detection->inference(img);
       result=detection->results;
    }

    // //[04] 赛道识别
    tracking.rowCutUp = motion.params.rowCutUp; // 图像顶部切行（前瞻距离）
    tracking.rowCutBottom = motion.params.rowCutBottom; // 图像底部切行（盲区距离）
    tracking.trackRecognition(imgBinary);

    if (mut.try_lock()) {  
      ai_image=img.clone();
      start_ai=true;
      mut.unlock();  
    }
        // if(time>motion.params.park_time){
    //   park=true;
    //   parking.countExit=4;
    // }
    //[05] 停车区检测
    if (motion.params.parking) {
      if (parking.process(result)||park==true) {
        scene = Scene::ParkingScene;
        if (parking.countExit >3) {
          park=true;
          park_n++;
          ctrlCenter.fitting(tracking,0.8,imgCorrect,scene,ring.ringStep,danger.block_cone);
          motion.poseCtrl(ctrlCenter.controlCenter,ring); // 姿态控制（舵机
          uart->carControl(0, motion.servoPwm); // 串口通信控制车辆
          continue;
          if(park_n>100){
            sleep(1);
            printf("-----> System Exit!!! <-----\n");
            exit(0); // 程序退出
          }
        }
      }
    }

    // //[06] 救援区检测
    // // // [09] 危险区检测
    if ((scene == Scene::DangerScene||scene == Scene::NormalScene) &&
        motion.params.danger) {
      if (danger.process(tracking,result,motion)) {
        //uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
        scene = Scene::DangerScene;
      } else
        scene = Scene::NormalScene;
    }
    if ((scene == Scene::NormalScene || scene == Scene::RescueScene) &&
        motion.params.rescue) {
      if (rescue.process(tracking,result,time,motion)){
        scene = Scene::RescueScene;
      }
      else
        scene = Scene::NormalScene;
    }
    // //[11] 环岛识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      if (ring.process(tracking, imgCorrect))
        scene = Scene::RingScene;
      else
        scene = Scene::NormalScene;
    }
    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross) {
      if (crossroad.crossRecognition(tracking))
        scene = Scene::CrossScene;
      else
        scene = Scene::NormalScene;
    }
    if(time>motion.params.danger_time&&danger_section1==false){
      danger_section1=true;
      motion.params.danger=true;
      float change_speed=motion.params.speedHigh;
      motion.params.speedHigh=danger_hightspeed;
      danger_hightspeed=change_speed;
      ring.reset();
      //rescue.reset();
      motion.params.ring=false;
      //motion.params.rescue=false;
    }
    if(time-motion.params.danger_time>250&&danger_section2==false){
      std::cout<<"一直"<<std::endl;
        danger_section2=true;
        motion.params.danger=false;
        motion.params.speedHigh=danger_hightspeed;
        ring.reset();
        //rescue.reset();
        motion.params.ring=true;
       // motion.params.rescue=true;
        scene = Scene::NormalScene;
    }
    if(time>motion.params.rescue_time&&motion.params.rescue_switch==true){
      if(result.size()>1&&scene == Scene::NormalScene){
        rescue_count++;
        if(rescue_count>3){
          if(motion.params.rescue_change==false)
            rescue.entryLeft=false;
          else
            rescue.entryLeft=true;
          motion.params.rescue_switch=false;
          rescue.step= Rescue::Step::Enable;
      }
      }
    }
    if(time>motion.params.ring_time&&scene == Scene::NormalScene&&motion.params.ring){
      //if(time>motion.params.ring_time+40){
        if(motion.params.ring_change){
          ring.ringType=Ring::RingType::RingLeft;
       }
        else{
          ring.ringType=Ring::RingType::RingRight;
        }
        ring.ringStep=Ring::RingStep::Entering;
      //}
    }
    if(scene == Scene::RingScene&&ring.ringStep>2){
      if(ring.ringType==1)
        ring_dirle=true;
      else if(ring.ringType==2)
        ring_dirle=false;
    }
    if(sceneLast == Scene::RingScene&&scene == Scene::NormalScene){
        ring_enable=true;
    }
    if(ring_enable==true){
      ring_cout++;
      motion.params.speedHigh=2.2;
      if(ring_cout>20){
        ring_cout=0;
        ring_enable=false;
        motion.params.ring=false;
      }
      if(ring_dirle==false&&tracking.stdevRight>40){
        float k=0;
        for(int i=0;i<tracking.pointsEdgeLeft.size();i++){
          if(tracking.pointsEdgeRight[i].y-tracking.pointsEdgeRight[i+1].y>10){
            k=(float)(tracking.pointsEdgeRight[i+5].y-tracking.pointsEdgeRight[i+10].y)/(float)(5);
            for(int n=i+5;n>0;n--){
              int c=(int)(k*(float)(i+5-n));
              tracking.pointsEdgeRight[n].y=tracking.pointsEdgeRight[i+5].y+c;
              if(tracking.pointsEdgeRight[n].y>COLSIMAGE-1)
                tracking.pointsEdgeRight[n].y=COLSIMAGE-1;
            }
            break;
          }
        }
      }
      if(ring_dirle==true&&tracking.stdevLeft>40){
        float k=0;
        for(int i=0;i<tracking.pointsEdgeRight.size();i++){
          if(tracking.pointsEdgeLeft[i+1].y-tracking.pointsEdgeLeft[i].y>10){
            k=(float)(tracking.pointsEdgeLeft[i+10].y-tracking.pointsEdgeLeft[i+5].y)/(float)(5);
            for(int n=i+5;n>0;n--){
              int c=(int)(k*(float)(i+5-n));
              tracking.pointsEdgeLeft[n].y=tracking.pointsEdgeLeft[i+5].y-c;
              if(tracking.pointsEdgeLeft[n].y<1)
                tracking.pointsEdgeLeft[n].y=1;
            }
            break;
          }
        }
      }
    }
    //[12] 车辆控制中心拟合
    float prospect=0;
    if(speed==motion.params.speedHigh)
      prospect= motion.params.prospect;
    else
      prospect= motion.params.prospect+0.1;
    if(scene==4)
      prospect=motion.params.danger_prospect;
    if(scene == Scene::NormalScene&&(time<motion.params.cross_time||time>motion.params.cross_time+300)){
    for(int i=tracking.pointsEdgeLeft.size()*4/5;i<tracking.pointsEdgeLeft.size();i++){
      if(tracking.pointsEdgeLeft[i].y<2){
        for(int n=i;n<tracking.pointsEdgeLeft.size();n++){
          if(tracking.pointsEdgeLeft[n].y>1&&n<tracking.pointsEdgeLeft.size()){
            std::cout<<"补了"<<std::endl;
            tracking.pointsEdgeRight[n].y= tracking.pointsEdgeLeft[n].y;
            tracking.pointsEdgeLeft[n].y=0;
          }
        }
        break;
      }
    }
    for(int i=tracking.pointsEdgeRight.size()*4/5;i<tracking.pointsEdgeRight.size();i++){
      if(tracking.pointsEdgeRight[i].y>COLSIMAGE-2){
        for(int n=i;n<tracking.pointsEdgeRight.size();n++){
          if(tracking.pointsEdgeRight[n].y<COLSIMAGE-2&&n<tracking.pointsEdgeRight.size()){
            std::cout<<"补了"<<std::endl;
            tracking.pointsEdgeLeft[n].y= tracking.pointsEdgeRight[n].y;
            tracking.pointsEdgeRight[n].y=COLSIMAGE-2;
          }
        }
        break;
      }
    }
    }
    ctrlCenter.fitting(tracking,prospect,imgCorrect,scene,ring.ringStep,danger.block_cone);
    if (scene != Scene::RescueScene) {
      if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
      {
        uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
        sleep(1);
        printf("-----> System Exit!!! <-----\n");
        exit(0); // 程序退出
      }
    }
    if (parking.park) // 特殊区域停车
      motion.speed = 0;
    else if((scene == Scene::RescueScene && rescue.carStoping) ||
        racing.carStoping)
      motion.speed = 0;
    else if (scene == Scene::RescueScene && rescue.carExitting) // 倒车出库
      motion.speed = -motion.params.speedLow;
    else if(scene == Scene::DangerScene)
      motion.speed = motion.params.danger_speed;
    else if(scene ==Scene::RingScene)
      motion.speed = motion.params.ringspeed;
    else if (scene == Scene::RescueScene&&rescue.step==1) // 减速
      motion.speed = motion.params.speedLow;
    else if (scene == Scene::RescueScene&&rescue.step==2) // 减速
      motion.speed = motion.params.speedDown;
    else
      motion.speedCtrl(true, false, ctrlCenter); // 车速控制
    motion.poseCtrl(ctrlCenter.controlCenter,ring); // 姿态控制（舵机
    std::cout<<"speed"<<motion.speed;
    speed=motion.speed_control(motion.speed,speed);

    if(scene ==Scene::RescueScene&&(rescue.step==2||rescue.step==4||rescue.step==5)){
        if(rescue.entryLeft){
          if(rescue.step==2)
            motion.servoPwm=1500+motion.params.rescue_pwm;
          else
            motion.servoPwm=1600+motion.params.rescue_pwm;
        }
        else{
          if(rescue.step==2)
            motion.servoPwm=1500-motion.params.rescue_pwm;
          else
            motion.servoPwm=1400-motion.params.rescue_pwm;
        }
    }
    // std::cout<<"到这"<<std::endl;
    // std::cout<<"pwm:"<<motion.servoPwm<<std::endl;
    if(motion.speed==motion.params.speedHigh)
      speed=motion.params.speedHigh;
    uart->carControl(speed, motion.servoPwm); // 串口通信控制车辆
    // if(ring_enable==true){
    //   ring_cout++;
    //   if(ring_cout>240&&speed==motion.params.speedHigh){
    //     motion.params.speedLow=2;
    //     motion.params.runP4=2.4;
    //   }
    // }
    // //[14] 综合显示调试UI窗口
    if (motion.params.debug) {
      // 帧率计算
      display.setNewWindow(2, "Binary", imgBinary);
      Mat imgRes =
          Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
      // display.setNewWindow(1, getScene(scene),
      //                      imgRes);   // 图像绘制特殊场景识别结果
      detection->drawBox(imgCorrect); // 图像绘制AI结果
      ctrlCenter.drawImage(tracking,
                           imgCorrect); // 图像绘制路径计算结果（控制中心）
      string str;
      str = "speed: " + formatDoble2String(speed, 1);
        putText(imgCorrect, str, Point(COLSIMAGE - 280, 2 * 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
                str = "scenc: " + formatDoble2String(scene, 1);
        putText(imgCorrect, str, Point(COLSIMAGE - 200, 7 * 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
      display.setNewWindow(1, "Ctrl", imgCorrect);
      display.show(); // 显示综合绘图
      waitKey(8);    // 等待显示
    }
    //std::cout<<scene<<std::endl;
    sceneLast = scene; // 记录当前状态
    
    //[16] 按键退出程序
    if (uart->keypress) {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      sleep(1);
      printf("-----> System Exit!!! <-----\n");
      exit(0); // 程序退出
    }
    if(motion.params.debug_fps){
        auto startTime = chrono::duration_cast<chrono::milliseconds>(
                          chrono::system_clock::now().time_since_epoch())
                          .count();
        std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();  
        std::chrono::duration<double> elapsed_seconds = current_time - prev_time;  
        if (elapsed_seconds.count() >= 1)  
        {  
            fps = frame_count / elapsed_seconds.count();
            printf(">> FrameTime: %ldms | %.2ffps \t", startTime - preTime,1000.0 / (startTime - preTime));
            std::cout << "FPS: " << fps << std::endl;  
            // 重置时间点和帧数  
            prev_time = current_time;  
            frame_count = 0;  
        }  
        frame_count++;
    }
    if (motion.params.saveImg) // 存储原始图像
    {
        drawBox(imgCorrect); 
        ctrlCenter.drawImage(tracking,
                              imgCorrect); // 图像绘制路径计算结果（控制中心
        string str;
        str = "speed: " + formatDoble2String(speed, 1);
        putText(imgCorrect, str, Point(COLSIMAGE - 280, 2 * 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
        str = "scenc: " + formatDoble2String(scene, 1);
        putText(imgCorrect, str, Point(COLSIMAGE - 200, 7 * 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右
        savePicture(imgCorrect);
    }
    if (scene == Scene::CrossScene)
      scene = Scene::NormalScene;
    time++;
  }
  start_ai=false;
  if (ai_thread.joinable()) { // 检查线程是否已被创建（即可连接）  
    ai_thread.join(); // 等待线程结束  
  }
  if (save_thread.joinable()) { // 检查线程是否已被创建（即可连接）  
    save_thread.join(); // 等待线程结束  
  }
    if (receive_thread.joinable()) { // 检查线程是否已被创建（即可连接）  
    receive_thread.join(); // 等待线程结束  
  }
  //send_thread.join();
  uart->close(); // 串口通信关闭
  capture.release();
  return 0;
}









































void drawBox(Mat &img)
{
    for (uint16_t i = 0; i < result.size(); i++)
    {
        PredictResult result1 = result[i];

        auto score = std::to_string(result1.score);
        int pointY = result1.y - 20;
        if (pointY < 0)
            pointY = 0;
        cv::Rect rectText(result1.x, pointY, result1.width, 20);
        cv::rectangle(img, rectText, getCvcolor(result1.type), -1);
        std::string label_name = result1.label + " [" + score.substr(0, score.find(".") + 3) + "]";
        cv::Rect rect(result1.x, result1.y, result1.width, result1.height);
        cv::rectangle(img, rect, getCvcolor(result1.type), 1);
        cv::putText(img, label_name, Point(result1.x, result1.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
    }
}
cv::Scalar getCvcolor(int index)
{
    switch (index)
    {
    case 0:
        return cv::Scalar(0, 255, 0); // 绿
        break;
    case 1:
        return cv::Scalar(255, 255, 0); // 天空蓝
        break;
    case 2:
        return cv::Scalar(0, 0, 255); // 大红
        break;
    case 3:
        return cv::Scalar(0, 250, 250); // 大黄
        break;
    case 4:
        return cv::Scalar(250, 0, 250); // 粉色
        break;
    case 5:
        return cv::Scalar(0, 102, 255); // 橙黄
        break;
    case 6:
        return cv::Scalar(255, 0, 0); // 深蓝
        break;
    case 7:
        return cv::Scalar(255, 255, 255); // 大白
        break;
    case 8:
        return cv::Scalar(247, 43, 113);
        break;
    case 9:
        return cv::Scalar(40, 241, 245);
        break;
    case 10:
        return cv::Scalar(237, 226, 19);
        break;
    case 11:
        return cv::Scalar(245, 117, 233);
        break;
    case 12:
        return cv::Scalar(55, 13, 19);
        break;
    case 13:
        return cv::Scalar(255, 255, 255);
        break;
    case 14:
        return cv::Scalar(237, 226, 19);
        break;
    case 15:
        return cv::Scalar(0, 255, 0);
        break;
    default:
        return cv::Scalar(255, 0, 0);
        break;
    }
}