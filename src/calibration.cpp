#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
int main(){
    cv::VideoCapture cap(0);
    if(!cap.isOpened()){
        std::cout<<"cannot open camera";
        return -1;
    }

    cv::Size broadsize(10,10);
    float squaresize;

    std::vector<std::vector<cv::Piont3f>> objpoints;
    std::vector<std::vector<cv::Piont2f>> imgpoints;

    std::vector<cv::Point3f> objp;
    objp.reserve(boardsize.width*boardsize.height);
    for(int i=0;i<boardsize.height;i++){
        for(int j=0;j<boardsize.width;j++){
            objp.emplace_back(j*squaresize,i*squaresize,0.0f);
        }
    }
    cv::Mat frame;
    cv::Mat gray;
    int savecount=0;

    while(1){
        cap>>frame;
        if(frame.empty()){
            break;
        }

        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;

        
    }
    return 0;
}