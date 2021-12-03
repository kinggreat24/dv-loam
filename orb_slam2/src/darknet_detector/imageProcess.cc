/*
 * @Author: kinggreat24
 * @Date: 2020-12-09 09:48:10
 * @LastEditTime: 2020-12-09 12:49:06
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /direct_lidar_alignment/src/darknet_detector/imageProcess.cc
 * @可以输入预定的版权声明、个性签名、空行等
 */
#include "darknet_detector/imageProcess.h"

namespace ORB_SLAM2
{
void imgConvert(const cv::Mat &img, float *dst)
{
    uchar *data = img.data;
    int h = img.rows;
    int w = img.cols;
    int c = img.channels();

    for (int k = 0; k < c; ++k)
    {
        for (int i = 0; i < h; ++i)
        {
            for (int j = 0; j < w; ++j)
            {
                dst[k * w * h + i * w + j] = data[(i * w + j) * c + k] / 255.;
            }
        }
    }
}

void imgResize(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight)
{
    int new_w = srcWidth;
    int new_h = srcHeight;
    if (((float)dstWidth / srcWidth) < ((float)dstHeight / srcHeight))
    {
        new_w = dstWidth;
        new_h = (srcHeight * dstWidth) / srcWidth;
    }
    else
    {
        new_h = dstHeight;
        new_w = (srcWidth * dstHeight) / srcHeight;
    }

    float *ImgReInner;
    size_t sizeInner = new_w * new_h * 3 * sizeof(float);
    ImgReInner = (float *)malloc(sizeInner);
    resizeInner(src, ImgReInner, srcWidth, srcHeight, new_w, new_h);

    for (int i = 0; i < dstWidth * dstHeight * 3; i++)
    {
        dst[i] = 0.5;
    }

    for (int k = 0; k < 3; ++k)
    {
        for (int y = 0; y < new_h; ++y)
        {
            for (int x = 0; x < new_w; ++x)
            {
                float val = ImgReInner[k * new_w * new_h + y * new_w + x];
                dst[k * dstHeight * dstWidth + ((dstHeight - new_h) / 2 + y) * dstWidth + (dstWidth - new_w) / 2 + x] = val;
            }
        }
    }
    free(ImgReInner);
}

void resizeInner(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight)
{
    float *part;
    size_t sizePa = dstWidth * srcHeight * 3 * sizeof(float);
    part = (float *)malloc(sizePa);

    float w_scale = (float)(srcWidth - 1) / (dstWidth - 1);
    float h_scale = (float)(srcHeight - 1) / (dstHeight - 1);

    for (int k = 0; k < 3; ++k)
    {
        for (int r = 0; r < srcHeight; ++r)
        {
            for (int c = 0; c < dstWidth; ++c)
            {
                float val = 0;
                if (c == dstWidth - 1 || srcWidth == 1)
                {
                    val = src[k * srcWidth * srcHeight + r * srcWidth + srcWidth - 1];
                }
                else
                {
                    float sx = c * w_scale;
                    int ix = (int)sx;
                    float dx = sx - ix;
                    val = (1 - dx) * src[k * srcWidth * srcHeight + r * srcWidth + ix] + dx * src[k * srcWidth * srcHeight + r * srcWidth + ix + 1];
                }
                part[k * srcHeight * dstWidth + r * dstWidth + c] = val;
            }
        }
    }

    for (int k = 0; k < 3; ++k)
    {
        for (int r = 0; r < dstHeight; ++r)
        {
            float sy = r * h_scale;
            int iy = (int)sy;
            float dy = sy - iy;
            for (int c = 0; c < dstWidth; ++c)
            {
                float val = (1 - dy) * part[k * dstWidth * srcHeight + iy * dstWidth + c];
                dst[k * dstWidth * dstHeight + r * dstWidth + c] = val;
            }
            if (r == dstHeight - 1 || srcHeight == 1)
                continue;
            for (int c = 0; c < dstWidth; ++c)
            {
                float val = dy * part[k * dstWidth * srcHeight + (iy + 1) * dstWidth + c];
                dst[k * dstWidth * dstHeight + r * dstWidth + c] += val;
            }
        }
    }
    free(part);
}

} // namespace dedvo