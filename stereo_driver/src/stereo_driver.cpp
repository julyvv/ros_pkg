#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

const int imageWidth = 320;                             //摄像头的分辨率  
const int imageHeight = 240;
Size imageSize = Size(imageWidth, imageHeight);
Mat cam,cam_left,cam_right ;
Mat rectifyImageL, rectifyImageR;
Mat mapLx, mapLy, mapRx, mapRy;     //映射表 
Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat cameraMatrixL = (Mat_<double>(3, 3) << 237.68468, 0, 151.02894,
	0, 237.82392, 89.54192,
	0, 0, 1);//对应matlab里的左相机标定矩阵
Mat distCoeffL = (Mat_<double>(5, 1) << 0.08572,  -0.08156,  -0.00607  , -0.00368  ,0.00000);
//对应Matlab所得左i相机畸变参数
Mat cameraMatrixR = (Mat_<double>(3, 3) << 237.71579, 0, 150.69910,
	0, 238.07833, 100.19867,
	0, 0, 1);//对应matlab里的右相机标定矩阵
Mat distCoeffR = (Mat_<double>(5, 1) << 0.06458  , -0.03643  , -0.00587 , 0.00061,  0.00000);
//对应Matlab所得右相机畸变参数
Mat T = (Mat_<double>(3, 1) << -58.63590  , 0.43682 , -0.59459);//T平移向量
                                                    //对应Matlab所得T参数
Mat rec = (Mat_<double>(3, 1) << 0.00726 ,  -0.02415  ,-0.00226);//rec旋转向量，对应matlab om参数
Mat R;//R 旋转矩阵
Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/left/image_raw", 1);
  image_transport::Publisher pub2 = it.advertise("camera/right/image_raw", 1);

/*
	立体校正
	*/
	Rodrigues(rec, R); //Rodrigues变换
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
		0, imageSize, &validROIL, &validROIR);
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);


  
  cv::VideoCapture cap(0);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
  if(!cap.isOpened()) 
  {
      ROS_INFO("can not opencv video device\n");
     
  }

  //ros::Rate loop_rate(20);
  while(nh.ok())
  {
	  cap>>cam;
          cam_left=cam(Rect(0,0,320,240));
	  cam_right=cam(Rect(320,0,320,240));
	  //cv::imshow("left",cam_left);
	 // cv::imshow("right",cam_right);


	remap(cam_left, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(cam_right, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

          sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",rectifyImageL).toImageMsg();
          sensor_msgs::ImagePtr msg2=cv_bridge::CvImage(std_msgs::Header(),"bgr8",rectifyImageR).toImageMsg();
          
         pub.publish(msg);pub2.publish(msg2);
	  ros::spinOnce();
          //loop_rate.sleep();
  
  }
  
}

