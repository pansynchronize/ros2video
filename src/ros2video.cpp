#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

const std::string keys =
	"{help h usage ? |      |  print this message }"
	"{@bagfile       |      |  bagfile location   }"
	"{output o       |output|  video output file name}"
	"{fps            |30   |  set the fps of output video}";

int main(int argc, char** argv){
	cv::CommandLineParser parser(argc, argv, keys);
	if(parser.has("help")){
		parser.printMessage();
		return 0;
	}
	std::string bagfile = parser.get<std::string>("@bagfile");
	std::string output = parser.get<std::string>("output");
	int fps = parser.get<int>("fps");
	std::cout << "Output: " << output << std::endl;
	rosbag::Bag bag;
	try{
		bag.open(bagfile);
	}catch(...){
		std::cerr << "Bagfile error, check if the file is broken!" << std::endl;
		return -1;
	}
    cv::Mat img;
    cv::VideoWriter video;
    bool isVWInitialized = false;

	for(rosbag::MessageInstance const m: rosbag::View(bag)){
		sensor_msgs::CompressedImage::ConstPtr i = m.instantiate<sensor_msgs::CompressedImage>();
		if(i != nullptr){
            img = cv_bridge::toCvCopy(i)->image;
            if(!isVWInitialized){
                cv::Size size = img.size();
                video.open(output+".avi",CV_FOURCC('M','J','P','G'), fps, size);
                video.write(img);
                isVWInitialized = true;
            }else{
                video.write(img);
//                 std::cout << img << std::endl;
            }
		}
	}
	video.release();
	bag.close();
	return 0;
}

