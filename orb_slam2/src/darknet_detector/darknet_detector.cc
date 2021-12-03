/*
 * @Author: kinggreat24
 * @Date: 2020-12-09 09:57:25
 * @LastEditTime: 2021-04-07 18:35:30
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/src/darknet_detector/darknet_detector.cc
 * @可以输入预定的版权声明、个性签名、空行等
 */
#include "darknet_detector/darknet_detector.h"

namespace ORB_SLAM2
{
DarknetDetector::DarknetDetector(std::string setting_file)
{
     //Check settings file
    cv::FileStorage fsSettings(setting_file.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << setting_file <<std::endl;
        exit(-1);
    }
    std::string cfg_file    = fsSettings["Yolo.cfg_file"];
    std::string weight_file = fsSettings["Yolo.weight_file"];
    demoThresh_             = fsSettings["Yolo.threshold"];
    demoHier_               = fsSettings["Yolo.hierhold"];

    cv::FileNode object_labels = fsSettings["Yolo.detection_classes"];
    if(object_labels.type() != cv::FileNode::SEQ)
    {
        std::cerr <<"Yolo.detection_classes is not a sequence!"<<std::endl;
        return;
    }
    cv::FileNodeIterator it = object_labels.begin(), it_end = object_labels.end();
    for(int count = 0;it != it_end; it++, count++)
    {
        detectionNames_.push_back(*it);
    }
    detectionClasses_ =   detectionNames_.size();//目标检测类别数目
    std::cout<<"Total classes: "<<detectionClasses_<<std::endl;

    setupNetwork(const_cast<char*>(cfg_file.c_str()),const_cast<char*>(weight_file.c_str()));
}

DarknetDetector::~DarknetDetector(){}


void DarknetDetector::objectPredictions(const cv::Mat& imgGray, std::vector<ObjBox_>&  obj_box_lists)
{
    cv::Mat rgbImg = imgGray.clone();
    if(rgbImg.channels() == 1)
        cv::cvtColor(rgbImg,rgbImg,CV_GRAY2RGB);

    //图像目标检测
    float* srcImg;
    size_t srcSize=rgbImg.rows*rgbImg.cols*3*sizeof(float);
    srcImg=(float*)malloc(srcSize);
    imgConvert(rgbImg,srcImg);//将图像转为yolo形式

    float* resizeImg;
    size_t resizeSize=net_->w*net_->h*3*sizeof(float);
    resizeImg=(float*)malloc(resizeSize);
    imgResize(srcImg,resizeImg,rgbImg.cols,rgbImg.rows,net_->w,net_->h); //缩放图像
    
    network_predict(*net_,resizeImg);//网络推理
    int nboxes=0;
    detection *dets = get_network_boxes(net_, imgGray.cols, imgGray.rows, demoThresh_, demoHier_, 0, 1, &nboxes, 1);
    // std::cout<<"object num: "<<nboxes<<std::endl;
    
    layer l = net_->layers[net_->n - 1];
    float nms = .4;
    if (nms > 0)
	{
        do_nms_obj(dets, nboxes, l.classes, nms);
    }	

    // extract the bounding boxes and send them to ROS
	int i, j;
	int count = 0;
    obj_box_lists.clear();
    int frameWidth_  = imgGray.cols;
    int frameHeight_ = imgGray.rows;

	for (i = 0; i < nboxes; ++i)
	{
		float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
		float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
		float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
		float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

		if (xmin < 0)
			xmin = 0;
		if (ymin < 0)
			ymin = 0;
		if (xmax > 1)
			xmax = 1;
		if (ymax > 1)
			ymax = 1;

		// iterate through possible boxes and collect the bounding boxes
		for (j = 0; j < detectionClasses_; ++j)
		{
			if (dets[i].prob[j])
			{
				float x_center = (xmin + xmax) / 2;
				float y_center = (ymin + ymax) / 2;
				float BoundingBox_width = xmax - xmin;
				float BoundingBox_height = ymax - ymin;

				// define bounding box
				// BoundingBox must be 1% size of frame (3.2x2.4 pixels)
				if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01)
				{
                    int box_xmin = (x_center - BoundingBox_width  / 2) * frameWidth_;
					int box_ymin = (y_center - BoundingBox_height / 2) * frameHeight_;
					int box_xmax = (x_center + BoundingBox_width  / 2) * frameWidth_;
					int box_ymax = (y_center + BoundingBox_height / 2) * frameHeight_;
                    
                    ObjBox_ objbox;
					objbox.x = box_xmin;
					objbox.y = box_ymin;
					objbox.w = box_xmax - box_xmin;
					objbox.h = box_ymax - box_ymin;
					objbox.Class = j;
					objbox.prob  = dets[i].prob[j];
                    objbox.label = detectionNames_[j];

                    obj_box_lists.push_back(objbox);
					count++;
				}
			}
		}
	}
    free_detections(dets, nboxes);

    //free image data 
    free(resizeImg);
    free(srcImg);
}

void DarknetDetector::setupNetwork(char *cfgfile, char *weightfile)
{
    printf("YOLO\n");
    net_ = load_network_custom(cfgfile, weightfile, 1, 1);
}

} // namespace dedvo