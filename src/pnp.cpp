#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <filesystem>

double getDistance(float width, float height,
                   float x0, float y0, float x1, float y1, 
                   float x2, float y2, float x3, float y3,
                   cv::Mat& R, cv::Mat& t) {  
    cv::Mat K = (cv::Mat_<double>(3, 3) << 2453.3, 0, 949.3,0, 2455.2, 554.5,0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(1, 5) << -0.0682737005569565, 0.1983544402464456, 0.0016855914452479342, 0.0024125119646311016, 0.0);
    
    std::vector<cv::Point3f> obj = {
        {-width/2, -height/2, 0},
        {-width/2,  height/2, 0},
        { width/2,  height/2, 0},
        { width/2, -height/2, 0}
    };
    
    std::vector<cv::Point2f> img = {
        {x0, y0}, {x1, y1}, {x2, y2}, {x3, y3}
    };
    
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(obj, img, K, D, rvec, tvec)) {
        return -1;
    }
    
    cv::Rodrigues(rvec, R);
    t = tvec.clone();
    
    double x = tvec.at<double>(0);
    double y = tvec.at<double>(1);
    double z = tvec.at<double>(2);
    return std::sqrt(x*x + y*y + z*z) / 1000.0;
}

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
        for(int j=0;j+3<midpoints.size();j=j+4){
            cv::line(result,midpoints[j+0],midpoints[j+2],cv::Scalar(0,0,255),2);
            cv::line(result,midpoints[j+1],midpoints[j+3],cv::Scalar(0,0,255),2);
            cv::Mat R, t;
            double dist = getDistance(135.0, 55.0,midpoints[j+1].x, midpoints[j+1].y,
                midpoints[j+0].x, midpoints[j+0].y,midpoints[j+2].x, midpoints[j+2].y,midpoints[j+3].x, midpoints[j+3].y,R, t); 
                if (dist > 0) {
                    std::string dist_text = std::to_string(dist).substr(0, 4) + " m";
                    cv::putText(result, dist_text,
                        cv::Point(midpoints[j+0].x/2 + midpoints[j+3].x/2 - 60,midpoints[j+0].y/2 + midpoints[j+3].y/2 - 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                        std::string r_text = cv::format("R: [%.2f, %.2f, %.2f; %.2f, %.2f, %.2f; %.2f, %.2f, %.2f]",
                            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                            std::string t_text = cv::format("t: [%.1f, %.1f, %.1f]",
                                t.at<double>(0), t.at<double>(1), t.at<double>(2));
                                cv::putText(result, r_text,cv::Point(10, 60),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
                                cv::putText(result, t_text,cv::Point(10, 90),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
                            }
            }
    }

    return result;

}

int main(){
    std::filesystem::create_directories("../output2");
    for(const auto entry:std::filesystem::directory_iterator("../assets/armors")){
        std::cout << entry.path() << std::endl;
        if(entry.path().extension()==".jpg"){
            cv::Mat img=cv::imread(entry.path().string());
            cv::Mat result=process(img,0);
            std::filesystem::path inputpath=entry.path();
            std::filesystem::path outputdir="../output2";
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
            std::filesystem::path outputdir="../output2";
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