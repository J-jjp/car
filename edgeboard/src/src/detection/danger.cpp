/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file danger.cpp
 * @author Leo
 * @brief 危险区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测

using namespace std;
using namespace cv;

/**
 * @brief 危险区AI识别与路径规划类
 *
 */
class Danger
{

public:
    bool bomb_enable=false;
    bool block_cone=false;
    /**
     * @brief 危险区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(Tracking &track, vector<PredictResult> predict, Motion &motion1)
    {
        enable = false; // 场景检测使能标志
        bool block_poit=false;
        bool block_enable=false;
        block_cone=false;
        int block_x=0;
        int block_y=0;
        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 3 || track.pointsEdgeRight.size() < ROWSIMAGE / 3)
            return enable;
        if(track.stdevLeft>100){
            for(size_t i=0;i<track.pointsEdgeLeft.size();i++){
                if(track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-1].y>30){
                    block_poit=true;
                    block_y=i;
                    //std::cout<<"有角点";
                    for(size_t ii=i+5;ii<track.pointsEdgeLeft.size();ii++){
                        if(std::abs(track.pointsEdgeLeft[ii-1].y-track.pointsEdgeLeft[i].y>10))
                            break;
                        if(track.pointsEdgeLeft[ii-1].y-track.pointsEdgeLeft[ii].y>20){
                            //std::cout<<"有黑块";
                            block_enable=true;
                            block_x=ii;
                        }
                    }
                }
            }
        }
        if(block_enable==true){
            if(track.pointsEdgeLeft[block_y].x-track.pointsEdgeLeft[block_x-1].x<ROWSIMAGE/3){
                //std::cout<<"是块";
            PredictResult block;
            block.type=LABEL_BLOCK;
            block.label="block";
            block.score=0.9;
            block.x=track.pointsEdgeLeft[block_x].y;
            block.y=track.pointsEdgeLeft[block_x].x;
            block.width=track.pointsEdgeLeft[block_y].y-track.pointsEdgeLeft[block_y-1].y;
            block.height=track.pointsEdgeLeft[block_y].x-track.pointsEdgeLeft[block_x-1].x;
            predict.push_back(block);
            }
        }
        vector<PredictResult> resultsObs; // 锥桶AI检测数据
        
        vector<PredictResult> bomb; // 锥桶AI检测数据
        for (int i = 0; i < predict.size(); i++)
        {
            if(predict[i].type == LABEL_BOMB||predict[i].type == LABEL_BRIDGE){
                enable=true;
            }
            else
                bomb_enable=false;
            if ((predict[i].type == LABEL_CONE || predict[i].type == LABEL_BLOCK) && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.1) // AI标志距离计算
                resultsObs.push_back(predict[i]);
        }
        // if(bomb_enable==true){
        //     enable=true;
        // }
        if (resultsObs.size() <= 0){
            bomb_enable=false;
            return enable;
        }
        // 选取距离最近的锥桶
        std::cout<<"锥桶"<<std::endl;
        int areaMax = 0; // 框面积
        int index = 0;   // 目标序号
        for (int i = 0; i < resultsObs.size(); i++)
        {
            int area=0;
            if(predict[i].type == LABEL_CONE)
                area =3* resultsObs[i].width * resultsObs[i].height;
            else
                area = resultsObs[i].width * resultsObs[i].height;
            if (area >= areaMax)
            {
                index = i;
                areaMax = area;
            }
        }
        resultObs = resultsObs[index];
        if(resultObs.y +resultObs.height>ROWSIMAGE-20&&resultObs.type == LABEL_CONE){
                resultObs.height= ROWSIMAGE-20-resultObs.y;
        }
        int danger_i=0;
        for(int i=0;i<track.pointsEdgeRight.size();i++){
            if(track.pointsEdgeRight[i].x<resultObs.y){
                danger_i=i;
                break;
            }
        }
        if(danger_i==0)
            return enable;
        if((resultObs.x>track.pointsEdgeLeft[danger_i].y&&resultObs.x<track.pointsEdgeRight[danger_i].y&&resultObs.y&&resultObs.type == LABEL_CONE)
        ||resultObs.type == LABEL_BLOCK){
            enable = true; // 场景检测使能标志
        }
        else
            return enable;
        if (resultObs.type == LABEL_CONE && (resultObs.y + resultObs.height) < ROWSIMAGE * 0.3)
            return enable;
        if(track.pointsEdgeLeft.back().x>resultObs.y||track.pointsEdgeRight.back().x>resultObs.y)
            return enable;
        // 障碍物方向判定（左/右）
        int row=0;
        for(int i=0;i<track.pointsEdgeRight.size();i++){
            if(track.pointsEdgeRight[i].x<resultObs.y){
                row=i;
                break;
            }
        }
        if (row < 0) // 无需规划路径
            return enable;
        int disLeft=0;
        int disRight=0;
        if(resultObs.type == LABEL_BLOCK){
            disLeft =abs(track.pointsEdgeLeft[row].y - resultObs.x);
            disRight =abs(track.pointsEdgeRight[row].y - resultObs.x);
            std::cout<<"坐标"<< resultObs.x<<"左"<<track.pointsEdgeLeft[row].y<<"由"<<track.pointsEdgeRight[row].y<<std::endl;
        }
        else{
            disLeft = resultObs.x + resultObs.width - track.pointsEdgeLeft[row].y;
            disRight = track.pointsEdgeRight[row].y - resultObs.x;
        }
        if(disLeft<0||disRight<0)
            return enable;
        if (((resultObs.x + resultObs.width > track.pointsEdgeLeft[row].y &&
            track.pointsEdgeRight[row].y > resultObs.x)||resultObs.type == LABEL_BLOCK )&&
            disLeft <= disRight) //[1] 障碍物靠左
        {
            if (resultObs.type == LABEL_BLOCK) // 黑色路障特殊处理
            {
                motion1.params.danger_P1= motion1.params.danger_P3;
                //std::cout<<"缩进了";
                curtailTracking(track, false); // 缩减优化车道线（双车道→单车道）
            }
            else
            {
                block_cone=true;
                        motion1.params.danger_P1= motion1.params.danger_P2;
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeLeft[10];
                points[1] = {resultObs.y + resultObs.height, resultObs.x + resultObs.width*2};
                points[2] = {(resultObs.y + resultObs.height + resultObs.y) / 2, resultObs.x + resultObs.width*2};
                if (resultObs.y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x)
                    points[3] = track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1];
                else
                    points[3] = {resultObs.y, resultObs.x + resultObs.width};

                track.pointsEdgeLeft.clear(); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);  // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                    track.pointsEdgeLeft.push_back(repair[i]);
            }
        }
        else if (((resultObs.x + resultObs.width > track.pointsEdgeLeft[row].y &&
                 track.pointsEdgeRight[row].y > resultObs.x )||resultObs.type == LABEL_BLOCK )&&
                 disLeft > disRight) //[2] 障碍物靠右
        {
            if (resultObs.type == LABEL_BLOCK) // 黑色路障特殊处理
            {
                motion1.params.danger_P1= motion1.params.danger_P3;
                curtailTracking(track, true); // 缩减优化车道线（双车道→单车道）
            }
            else
            {
                block_cone=true;
                motion1.params.danger_P1= motion1.params.danger_P2;
                vector<POINT> points(4); // 三阶贝塞尔曲线
                points[0] = track.pointsEdgeRight[10];
                points[1] = {resultObs.y + resultObs.height, resultObs.x - resultObs.width*2};
                points[2] = {(resultObs.y + resultObs.height + resultObs.y) / 2, resultObs.x - resultObs.width*2};
                if (resultObs.y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x)
                    points[3] = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1];
                else
                    points[3] = {resultObs.y+resultObs.height, resultObs.x};

                track.pointsEdgeRight.clear(); // 删除错误路线
                vector<POINT> repair = Bezier(0.01, points);   // 重新规划车道线
                for (int i = 0; i < repair.size(); i++)
                    track.pointsEdgeRight.push_back(repair[i]);
            }
        }
        else{
            std::cout<<"x:"<<resultObs.x + resultObs.width<<"X"<<track.pointsEdgeLeft[row].y<<"y:"<<track.pointsEdgeRight[row].y <<"Y"<< resultObs.x;
        }

        return enable;
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (enable)
        {
            putText(img, "[2] DANGER - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
            cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
            cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        }
    }

private:
    bool enable = false;     // 场景检测使能标志
    PredictResult resultObs; // 避障目标锥桶

    /**
     * @brief 缩减优化车道线（双车道→单车道）
     *
     * @param track
     * @param left
     */
    void curtailTracking(Tracking &track, bool left)
    {
        if (left) // 向左侧缩进
        {
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

            for (uint16_t i = 0; i < track.pointsEdgeRight.size(); i++)
            {
                track.pointsEdgeRight[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
        else // 向右侧缩进
        {
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (uint16_t i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                track.pointsEdgeLeft[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
    }
};
