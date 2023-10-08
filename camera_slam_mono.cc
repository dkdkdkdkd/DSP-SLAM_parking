/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;
using namespace cv;

void LoadImages(const string &strSequence, const float &fps, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings cameranumber path_to_saved_trajectory [1|0](save map?)" << endl;
        return 1;
    }

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);
    float fps = fSettings["Camera.fps"];

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], argv[3], ORB_SLAM2::System::MONOCULAR, (bool)atoi(argv[5]));

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // VideoCapture cap(atoi(argv[3]));
    VideoCapture cap(1);

    if (!cap.isOpened())
    {
        cout << "can't open the camera" << endl;
        return -1;
    }

    // Main loop
    cv::Mat im;
    double timestamp = 0;
    
    while(1)
    {
        // Read image from file
        cap >> im;
        
        double tframe = timestamp ;
        timestamp += 0.05;


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        

        // Wait to load the next frame
        double T=0;
        T = (timestamp-tframe);

        if(ttrack<T)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));
        }

//        if (SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK)
//            SLAM.SaveMapCurrentFrame(string(argv[4]), ni);
        if (waitKey(1) == 27)
            break;
    }

    SLAM.SaveEntireMap(string(argv[4]));

    cv::waitKey(0);

    // Stop all threads
    SLAM.Shutdown();


    
    cout << "-------" << endl << endl;

    return 0;
}

void LoadImages(const string &strPathToSequence, const float &fps, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    float dt = 1. / fps;
    float t = 0.;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            vTimestamps.push_back(t);
            t += dt;
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vTimestamps.push_back(t);
        t += dt;
    }
}


