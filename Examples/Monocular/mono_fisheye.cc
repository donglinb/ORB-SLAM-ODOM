#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<System.h>

using namespace std;

extern bool bTightCouple=true;
extern bool bLooseCouple=true;
extern bool bFixScale=true;

void LoadDataset(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps, vector<Eigen::Vector3d> vodomPos, vector<Eigen::AngleAxisd> vodomOri);
void LoadDataset(const string &strFile, vector<string> &vstrImageFilenames, 
                vector<double> &vTimestamps, vector<cv::Vec3d> &vodomPose);
void applyMask(const cv::Mat& src, cv::Mat& dst, const cv::Mat& mask);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    bTightCouple=true;
    bLooseCouple=true;
    bFixScale=false;

    
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    vector<cv::Vec3d> vodomPose;
    //vector<Eigen::AngleAxisd> vodomOri;

    string strFile = string(argv[3])+"/associate.txt";
    // string strFile = string(argv[3])+"/groundtruth.txt";

    //LoadDataset(strFile, vstrImageFilenames, vTimestamps, vodomPos, vodomOri);
    LoadDataset(strFile, vstrImageFilenames, vTimestamps, vodomPose);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cv::Mat mask_img=cv::imread("Examples/Monocular/mask_new_front.png");
    if(mask_img.empty())
    {
        cerr<<"failed to read mask image."<<endl;
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        cv::Vec3d odomframe=vodomPose[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }
        // apply mask
        if(!mask_img.empty())
        {
            cv::Mat im_masked;
            applyMask(im,im_masked,mask_img);
            im=im_masked.clone();
        }
        // crop image
        {
            int crop_origin_x_=0,crop_origin_y_=0,crop_width_=1900,crop_height_=800;
            cv::Mat im_croped=im(cv::Rect(crop_origin_x_, crop_origin_y_, crop_width_, crop_height_));
            im=im_croped.clone();
        }
        // down sample
        cv::resize(im,im,cv::Size(0,0),0.5,0.5);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        //SLAM.TrackMonocular(im,tframe);
        SLAM.TrackMonocularWithOdom(im,odomframe,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    cv::waitKey(0);
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectoryTUM.txt");
    // SLAM.SaveTrajectoryTUM("TrajectoryTUM.txt");
    SLAM.SaveKeyFrameTrajectoryOdomTUM("KeyFrameTrajectoryOdomTUM.txt");

    return 0;
}

void LoadDataset(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, vector<Eigen::Vector3d> vodomPos, vector<Eigen::AngleAxisd> vodomOri)
{
    ifstream f;
    f.open(strFile.c_str());

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            double x,y,theta;
            string image;
            ss >> t;
            vTimestamps.push_back(t);
            ss>>x>>y>>theta;
            vodomPos.push_back(Eigen::Vector3d(x,y,0));
            Eigen::AngleAxisd rotationVector(theta,Eigen::Vector3d(0,0,1));
            vodomOri.push_back(rotationVector);
            ss >> image;
            vstrImageFilenames.push_back("front/"+image);
        }
    }
    double t0=vTimestamps[0];
    for_each(vTimestamps.begin(),vTimestamps.end(),[t0](double &t){t-=t0;});
}

void LoadDataset(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, vector<cv::Vec3d> &vodomPose)
{
    ifstream f;
    f.open(strFile.c_str());

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            double x,y,theta;
            string image;
            ss >> t;
            vTimestamps.push_back(t);
            ss>>x>>y>>theta;
            vodomPose.push_back(cv::Vec3d(x,y,theta));
            ss >> image;
            vstrImageFilenames.push_back("image/"+image);
        }
    }
    // double t0=vTimestamps[0];
    // for_each(vTimestamps.begin(),vTimestamps.end(),[t0](double &t){t-=t0;});
}

void applyMask(const cv::Mat& src, cv::Mat& dst, const cv::Mat& mask)
{
  dst = src.clone();
  for (int i = 0; i < src.rows; ++i)
    for (int j = 0; j < src.cols; ++j)
    {
      cv::Vec3b pixel = mask.at<cv::Vec3b>(i, j);
      if (pixel[1] > 250)
        dst.at<cv::Vec3b>(i, j) = 0;
    }
}