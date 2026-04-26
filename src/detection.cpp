#include<iostream>
#include<opencv2/opencv.hpp>
#include<filesystem>
#include<string>

cv::Mat process(cv::Mat img,int i){
    cv::Mat red;
    cv::extractChannel(img,red,i);
    
    cv::Mat img_blur;
    cv::GaussianBlur(red, img_blur, cv::Size(3, 3), 1.0);

    cv::Mat im;
    cv::threshold(img_blur,im,238,255,cv::THRESH_BINARY_INV);


    cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, sobel_edge;
    cv::Sobel(im, grad_x, CV_16S, 1, 0, 3, 0.1);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::Sobel(im, grad_y, CV_16S, 0, 1, 3, 0.1);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::addWeighted(abs_grad_x, 0.6, abs_grad_y, 0.4, 0, sobel_edge);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(sobel_edge, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    std::vector<std::vector<cv::Point>> valid_contours;
    for (auto& cnt : contours) {
        double area = contourArea(cnt);
        cv::Rect rect = boundingRect(cnt);
        double aspect_ratio = (double)rect.height / rect.width;

        if (area > 5 && aspect_ratio > 1) {
            valid_contours.push_back(cnt);
        }
    }

    cv::Mat result = img.clone();
    std::vector<cv::Point2f> midpoints;
    for (auto& cnt : valid_contours) {
        cv::Rect rect = cv::boundingRect(cnt);
        cv::rectangle(result, rect, cv::Scalar(0, 0, 255), 2);
        float midx=rect.x+rect.width/2.0;
        cv::Point2f midpoint1(midx,rect.y);
        cv::Point2f midpoint2(midx,rect.y+rect.height);

        midpoints.push_back(midpoint1);
        midpoints.push_back(midpoint2);

        cv::circle(result,midpoint1,3,cv::Scalar(0,0,255),-1);
        cv::circle(result,midpoint2,3,cv::Scalar(0,0,255),-1);
    }
    if(midpoints.size()>=4){
        for(int j=0;j<midpoints.size();j=j+4){
    cv::line(result,midpoints[j+0],midpoints[j+2],cv::Scalar(0,0,255),2);
    cv::line(result,midpoints[j+1],midpoints[j+3],cv::Scalar(0,0,255),2);}
    }

    return result;

}
int main(){
    std::filesystem::create_directories("../output");
    for(const auto entry:std::filesystem::directory_iterator("../assets/armors")){
        std::cout << entry.path() << std::endl;
        if(entry.path().extension()==".jpg"){
            cv::Mat img=cv::imread(entry.path().string());
            cv::Mat result=process(img,0);
            std::filesystem::path inputpath=entry.path();
            std::filesystem::path outputdir="../output";
            std::filesystem::path outputpath=outputdir/inputpath.filename();
            cv::imwrite(outputpath.string(),result);
        }else{
            cv::VideoCapture cap(entry.path());
            if(!cap.isOpened()){
                    return -1;
            }
            int width=(int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
            int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
            double fps = cap.get(cv::CAP_PROP_FPS);

            std::filesystem::path inputpath=entry.path();
            std::filesystem::path outputdir="../output";
            std::filesystem::path outputpath=outputdir/inputpath.filename();
            cv::VideoWriter writer(outputpath.string(),cv::VideoWriter::fourcc('m','p','4','v'),fps,cv::Size(width, height));

            if (!writer.isOpened()) {
                return -1;
            }
            cv::Mat frame;

            while(true){
                cap>>frame;
                if(frame.empty()){
                    break;
                }

                cv::Mat result=process(frame,2);
                writer.write(result);
            }
            cap.release();
            writer.release();
        }
    }
    return 0;
}