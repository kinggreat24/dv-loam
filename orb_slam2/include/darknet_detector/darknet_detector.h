/*
 * @Author: kinggreat24
 * @Date: 2020-12-09 09:51:21
 * @LastEditTime: 2021-04-07 18:34:05
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/include/darknet_detector/darknet_detector.h
 * @可以输入预定的版权声明、个性签名、空行等
 */
#ifndef DARKNET_DETECTOR_H
#define DARKNET_DETECTOR_H

#include "darknet/darknet.h"
#include "darknet_detector/imageProcess.h"

extern "C" {
// #include "blas.h"
// #include "network.h"
// #include "detection_layer.h"
// #include "region_layer.h"
// #include "cost_layer.h"
// #include "utils.h"
// #include "parser.h"
// #include "box.h"
// #include <sys/time.h>
}

namespace ORB_SLAM2
{

//! Bounding box of the detected object.
typedef struct
{
  float x, y, w, h, prob;
  int num, Class;
  std::string label;
  void print_info()
  {
      std::cout<<"label: "<<label
        <<" class id: "<<Class
        <<" prob: "<<prob
        <<" x: "<<x<<" y: "<<y<<" w: "<<w<<" h: "<<h<<std::endl;
  }

  bool isInsideBox(int u, int v)
  {
    if( u >= x  && u <= x+w && v>=y && v<= y+h)
        return true;
    else
        return false;
  }
} ObjBox_;


class DarknetDetector
{
public:
    DarknetDetector(std::string setting_file);
    ~DarknetDetector();

    void objectPredictions(const cv::Mat& imgGray,std::vector<ObjBox_>&  obj_box_lists);

protected:
    int sizeNetwork(network *net);
    void setupNetwork(char *cfgfile, char *weightfile);

private:
    network *net_;
    std::vector<ObjBox_>  obj_box_lists_;
    float demoThresh_;
    float demoHier_;
    int detectionClasses_;
    std::vector<std::string> detectionNames_;
};

} // namespace dedvo

#endif //DARKNET_DETECTOR_H