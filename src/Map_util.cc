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

#include "Map.h"
#include<mutex>
#include<sstream>
namespace ORB_SLAM2
{

void Map::AddMapObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.insert(pMO);
    // vector<vector<float>> area;

    // auto pose = pMO->GetPoseSim3();

    // // cout << "hello" << typeid(pose(0, 3)).name() << endl;

    // ifstream file("/home/jiho/slam/DSP-SLAM_parking/parking_areas.txt");

    // string line;
    // string tmp;
    // bool check = false;
    // int count = 0;
    // float distance;

    // if(file.is_open()){
    //     while(getline(file, line)) {
    //         vector<float> area;
    
    //         if (count%2==1){

    //             stringstream ss(line);
    //             while(getline(ss, tmp, ' ')){
    //                 // cout << tmp << endl;
    //                 area.push_back(std::stof(tmp));
    //             }
    //             distance = 0.0;

    //             for(int i=0;i<3;i++){
    //                 distance += (pose(i, 3) - area[i*4+3])*(pose(i, 3) - area[i*4+3]);
    //             }
    //             distance = sqrt(distance);
    //             if (distance<1.5){
    //                 check = true;
                    
    //                 cout<<"id: " << pMO->mnId << " " << "True" << endl;
    //             }
    //         }
    //         count++;
    //     }
    //     if (!check)
    //         cout<<"id: " << pMO->mnId << " " << "False" << endl;
        
    // }
    // else
    //     cout<< "not file" << endl;

}

void Map::EraseMapObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.erase(pMO);
}

vector<MapObject*> Map::GetAllMapObjects()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapObject*>(mspMapObjects.begin(), mspMapObjects.end());
}

MapObject* Map::GetMapObject(int object_id)
{
    unique_lock<mutex> lock(mMutexMap);
    for (auto mspMapObject : mspMapObjects)
    {
        if(mspMapObject->mnId != object_id)
            continue;
        return mspMapObject;
    }
    return NULL;
}

}

