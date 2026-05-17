#include<iostream>
#include<opencv2/opencv.hpp>
#include<filesystem>
#include<string>
#include<cmath>

cv::Mat process(cv::Mat img,int i){
    cv::Mat red;
    cv::extractChannel(img,red,i);


    cv::Mat im;
    cv::threshold(red,im,150,255,cv::THRESH_BINARY);

    cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(im,im,cv::MORPH_CLOSE,kernel);
    cv::morphologyEx(im,im,cv::MORPH_OPEN,kernel); 

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(im, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    cv::Mat result=img.clone();
    std::vector<std::vector<float>>lights;
    std::vector<std::vector<cv::Point2f>>lights_v;
    for(auto& cnt:contours) {
        cv::RotatedRect rect=cv::minAreaRect(cnt);
            double area=cv::contourArea(cnt);
            if(rect.size.width>rect.size.height){
                std::swap(rect.size.width,rect.size.height);
                rect.angle=90+rect.angle;
            }
            if(rect.size.height<rect.size.width*2||rect.size.height>rect.size.width*10){
                continue;
            }
            if(area>50){
                cv::Point2f ver[4];
                rect.points(ver);
                std::vector<cv::Point2f>v(ver,ver+4);
                lights_v.push_back(v);
                lights.push_back({rect.center.x,rect.center.y,rect.size.width,rect.size.height,rect.angle});
            }
    }

    
    std::vector<bool> used(lights.size(),false);
    for(int i=0;i<lights.size();i++){
        if(used[i]){
            continue;
        }
        std::vector<float>light1{lights[i][0],lights[i][1],lights[i][2],lights[i][3],lights[i][4]};//x,y,宽，高，倾斜角
        int best=-1;
        for(size_t j=i+1;j<lights.size();j++){
            if(used[j]){
                continue;
            }
            std::vector<float>light2{lights[j][0],lights[j][1],lights[j][2],lights[j][3],lights[j][4]};//x,y,宽，高，倾斜角
            float h_max=std::max(light1[3],light2[3]);
            float h_min=std::min(light1[3],light2[3]);
            if(h_min/h_max<0.7f){
                continue;
            }
            if(std::abs(light1[4]-light2[4])>15){
                continue;
            }
            if(std::pow(std::pow(light1[1]-light2[1],2)+std::pow(light1[0]-light2[0],2),0.5)>h_max*3){
                continue;
            }
            std::vector<cv::Point2f> lv,rv;
            if(light1[0]<=light2[0]){
                lv=lights_v[i];
                rv=lights_v[j];
            }else{
                lv=lights_v[j];
                rv=lights_v[i];
            }

            cv::Point2f top_edge=rv[0]-lv[1];
            cv::Point2f bottom_edge=rv[3]-lv[2];
            cv::Point2f left_edge=lv[2]-lv[1];
            cv::Point2f right_edge=rv[3]-rv[0];
            auto angle_between=[](cv::Point2f v1,cv::Point2f v2)->float{float dot=v1.x*v2.x+v1.y*v2.y;float norm=cv::norm(v1)*cv::norm(v2);
                if(norm<1e-6)return 0;
                return std::acos(dot/norm)*180.0f/CV_PI;
            };
            float angle_top_bottom=angle_between(top_edge,bottom_edge);
            float angle_left_right=angle_between(left_edge,right_edge);
            if(angle_top_bottom>3||angle_left_right>3){
                continue;
            }

            float diag1=cv::norm(rv[1]-lv[3]);
            float diag2=cv::norm(rv[3]-lv[1]);
            float diag_ratio=std::min(diag1,diag2)/std::max(diag1,diag2);
            if(diag_ratio<0.72f){
                continue;
            }
            if(best<0){
                best=j;
                continue;
            }
            float diag3=cv::norm(lights_v[i][1]-lights_v[best][3]);
            float diag4=cv::norm(lights_v[i][3]-lights_v[best][1]);
            float diag_ratio_=std::min(diag3,diag4)/std::max(diag3,diag4);
            if(diag_ratio_<diag_ratio){
                best=j;
            }
            best=j;
        }
        if(best<0){
            continue;
        }
        for(int k=0;k<4;k++){
            cv::line(result,lights_v[i][k],lights_v[i][(k+1)%4],cv::Scalar(0,0,255),2);
        }
        for(int k=0;k<4;k++){
            cv::line(result,lights_v[best][k],lights_v[best][(k+1)%4],cv::Scalar(0,0,255),2);
        }
        cv::line(result,lights_v[i][1],lights_v[best][1],cv::Scalar(0,0,255),2);
        cv::line(result,lights_v[i][3],lights_v[best][3],cv::Scalar(0,0,255),2);
        used[i]=true;
        used[best]=true;
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