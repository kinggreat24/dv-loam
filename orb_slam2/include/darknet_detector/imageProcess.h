/*
 * @Author: kinggreat24
 * @Date: 2020-12-09 09:46:38
 * @LastEditTime: 2021-04-07 18:34:11
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/include/darknet_detector/imageProcess.h
 * @可以输入预定的版权声明、个性签名、空行等
 */
#ifndef IMPROCESS_H
#define IMPROCESS_H

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
    void imgConvert(const cv::Mat &img, float *dst);

    void imgResize(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight);

    void resizeInner(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight);
} // namespace dedvo

#endif // IMPROCESS_H