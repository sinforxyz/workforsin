#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<filesystem>
#include<string>

int main(){
    cv::VideoCapture cap(0);
    if(!cap.isOpened()){
            std::cout<<"cannot open camera";
            return -1;
        }

    cv::Size broadsize(8,6);
    float squaresize=2.0f;

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    std::vector<cv::Point3f> objp;
    objp.reserve(broadsize.width*broadsize.height);
    for(int i=0;i<broadsize.height;i++){
        for(int j=0;j<broadsize.width;j++){
            objp.emplace_back(j*squaresize,i*squaresize,0.0f);
        }
    }
    cv::Mat frame;
    cv::Mat gray;
    int savecount=0;

    std::filesystem::path currentpath=std::filesystem::current_path();
    std::filesystem::path parentpath=currentpath.parent_path();
    std::string savepath=parentpath.string();

    bool cameraopen=true;
    while(cameraopen){
        cap>>frame;
        if(frame.empty()){
            break;
        }
        

        char key=(char)cv::waitKey(30);
        
        if(key==27&&cameraopen){
            cap.release();
            cameraopen=false;
            break;
        }
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found=cv::findChessboardCorners(gray,broadsize,corners,cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE|cv::CALIB_CB_FAST_CHECK);
        cv::drawChessboardCorners(frame, broadsize, corners, found);
        cv::imshow("window",frame);

        if(key=='s'||key=='S'){
            if(found){
                cv::cornerSubPix(gray, corners, cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
                objpoints.emplace_back(objp);
                imgpoints.emplace_back(corners);
                savecount++;
            }
        }
        if(key=='c'||key=='C'){
        if(savecount>=5){
            cv::Mat cameram=cv::Mat::eye(3,3,CV_64F);
            cv::Mat distcoeffs=cv::Mat::zeros(8,1,CV_64F);
            std::vector<cv::Mat>rvecs,tvecs;

            double rms=cv::calibrateCamera(objpoints,imgpoints,frame.size(),cameram,distcoeffs,rvecs,tvecs,0);
            std::cout<<rms<<std::endl;

            cv::FileStorage fs(savepath+"/cab_result.xml",cv::FileStorage::WRITE);
            fs<<"camera_matrix"<<cameram;
            fs<<"distcoffs"<<distcoeffs;
            fs.release();
            cap.release();
            cameraopen=false;
            break;
        }}
    }
    cv::destroyAllWindows(); 
    return 0;
}