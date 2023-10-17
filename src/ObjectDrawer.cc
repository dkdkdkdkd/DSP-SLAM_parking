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

#include "ObjectDrawer.h"

struct Point
{
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

// 직사각형 구조체 정의
struct Rectangle
{
    Point A, B, C, D;
    Rectangle(Point A, Point B, Point C, Point D) : A(A), B(B), C(C), D(D) {}
};

bool isPointInsideRectangle(const Rectangle &rect, const Point &P)
{
    Eigen::Vector3d AB(rect.B.x - rect.A.x, rect.B.y - rect.A.y, 0);
    Eigen::Vector3d BC(rect.C.x - rect.B.x, rect.C.y - rect.B.y, 0);
    Eigen::Vector3d CD(rect.D.x - rect.C.x, rect.D.y - rect.C.y, 0);
    Eigen::Vector3d DA(rect.A.x - rect.D.x, rect.A.y - rect.D.y, 0);

    Eigen::Vector3d PA(rect.A.x - P.x, rect.A.y - P.y, 0);
    Eigen::Vector3d PB(rect.B.x - P.x, rect.B.y - P.y, 0);
    Eigen::Vector3d PC(rect.C.x - P.x, rect.C.y - P.y, 0);
    Eigen::Vector3d PD(rect.D.x - P.x, rect.D.y - P.y, 0);

    // 사각형(삼각형 2개의 합) 넓이
    double areaABD = 0.5 * AB.cross(-DA).norm();
    double areaBCD = 0.5 * CD.cross(-BC).norm();
    double totalArea = areaABD + areaBCD;

    // 4개의 삼각형 넓이
    double totalAreaFourTriangles = 0;
    totalAreaFourTriangles += 0.5 * PA.cross(PB).norm();
    totalAreaFourTriangles += 0.5 * PB.cross(PC).norm();
    totalAreaFourTriangles += 0.5 * PC.cross(PD).norm();
    totalAreaFourTriangles += 0.5 * PD.cross(PA).norm();

    // 오차 인정 범위
    double error = 6;
    return totalArea + error >= totalAreaFourTriangles && totalAreaFourTriangles >= totalArea - error;
}

namespace ORB_SLAM2
{

ObjectDrawer::ObjectDrawer(Map *pMap, MapDrawer *pMapDrawer, const string &strSettingPath) : mpMap(pMap), mpMapDrawer(pMapDrawer)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mvObjectColors.push_back(std::tuple<float, float, float>({230. / 255., 0., 0.}));	 // red  0
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 0., 255. / 255.}));	 // blue  1
    mvObjectColors.push_back(std::tuple<float, float, float>({60. / 255., 180. / 255., 75. / 255.}));   // green  2
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 0, 255. / 255.}));   // Magenta  3
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 165. / 255., 0}));   // orange 4
    mvObjectColors.push_back(std::tuple<float, float, float>({128. / 255., 0, 128. / 255.}));   //purple 5
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 255. / 255., 255. / 255.}));   //cyan 6
    mvObjectColors.push_back(std::tuple<float, float, float>({210. / 255., 245. / 255., 60. / 255.}));  //lime  7
    mvObjectColors.push_back(std::tuple<float, float, float>({250. / 255., 190. / 255., 190. / 255.})); //pink  8
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 128. / 255., 128. / 255.}));   //Teal  9
    SE3Tcw = Eigen::Matrix4f::Identity();
    SE3TcwFollow = Eigen::Matrix4f::Identity();
}

void ObjectDrawer::SetRenderer(ObjectRenderer *pRenderer)
{
    mpRenderer = pRenderer;
}

void ObjectDrawer::AddObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexObjects);
    mlNewMapObjects.push_back(pMO);
}

void ObjectDrawer::ProcessNewObjects()
{
    unique_lock<mutex> lock(mMutexObjects);
    auto pMO = mlNewMapObjects.front();
    if (pMO)
    {
        int renderId = (int) mpRenderer->AddObject(pMO->vertices, pMO->faces);
        pMO->SetRenderId(renderId);
        mlNewMapObjects.pop_front();
    }
}

void ObjectDrawer::DrawObjects(bool bFollow, const Eigen::Matrix4f &Tec)
{
        unique_lock<mutex> lock(mMutexObjects);

        auto mvpMapObjects = mpMap->GetAllMapObjects();

        for (MapObject *pMO : mvpMapObjects)
        {
            if (!pMO)
                continue;
            if (pMO->isBad())
                continue;

            Eigen::Matrix4f Sim3Two = pMO->GetPoseSim3();
            int idx = pMO->GetRenderId();

            if (bFollow)
            {
                SE3TcwFollow = SE3Tcw;
            }
            if (pMO->GetRenderId() >= 0)
            {

                std::ifstream file("/home/jiho/slam/DSP-SLAM_parking/data/legal_parking_zone_coordinates.txt");
                std::vector<Rectangle> rectangles;
                std::string line;
                while (std::getline(file, line))
                {
                    std::istringstream iss(line);
                    double x1, y1, x2, y2, x3, y3, x4, y4;
                    if (iss >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4)
                    {
                        Point lt(x1, y1);
                        Point lb(x2, y2);
                        Point rb(x3, y3);
                        Point rt(x4, y4);
                        Rectangle rect(lt, lb, rb, rt);
                        rectangles.push_back(rect);
                    }
                    else
                    {
                        std::cerr << "잘못된 입력 형식입니다: " << line << std::endl;
                    }
                }

                file.close();

                double m, n;
                m = Sim3Two(0, 3);
                n = Sim3Two(2, 3);

                int color = 0;
                for (size_t i = 0; i < rectangles.size(); ++i)
                {
                    if (isPointInsideRectangle(rectangles[i], Point(m, n)))
                    {
                        color = 1;
                        pMO->checkArea = 1;
                        break;
                    }
                }

                cout << m << ", " << n << "에 위치한 차량은 " << ((color == 0) ? "불법" : "합법") << "차량입니다" << endl;

                // mpRenderer->Render(idx, Tec * SE3TcwFollow * Sim3Two, mvObjectColors[pMO->GetRenderId() % mvObjectColors.size()]);
                mpRenderer->Render(idx, Tec * SE3TcwFollow * Sim3Two, mvObjectColors[color]);
            }
            // DrawCuboid(pMO);
        }
    }

void ObjectDrawer::DrawCuboid(MapObject *pMO)
{
    const float w = pMO->w / 2;
    const float h = pMO->h / 2;
    const float l = pMO->l / 2;

    glPushMatrix();

    pangolin::OpenGlMatrix Two = Converter::toMatrixPango(pMO->SE3Two);
#ifdef HAVE_GLES
    glMultMatrixf(Two.m);
#else
    glMultMatrixd(Two.m);
#endif

    const float mCuboidLineWidth = 3.0;
    glLineWidth(mCuboidLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);

    glVertex3f(w,h,l);
    glVertex3f(w,-h,l);

    glVertex3f(-w,h,l);
    glVertex3f(-w,-h,l);

    glVertex3f(-w,h,l);
    glVertex3f(w,h,l);

    glVertex3f(-w,-h,l);
    glVertex3f(w,-h,l);

    glVertex3f(w,h,-l);
    glVertex3f(w,-h,-l);

    glVertex3f(-w,h,-l);
    glVertex3f(-w,-h,-l);

    glVertex3f(-w,h,-l);
    glVertex3f(w,h,-l);

    glVertex3f(-w,-h,-l);
    glVertex3f(w,-h,-l);

    glVertex3f(w,h,-l);
    glVertex3f(w,h,l);

    glVertex3f(-w,h,-l);
    glVertex3f(-w,h,l);

    glVertex3f(-w,-h,-l);
    glVertex3f(-w,-h,l);

    glVertex3f(w,-h,-l);
    glVertex3f(w,-h,l);

    glEnd();

    glPopMatrix();
}

void ObjectDrawer::SetCurrentCameraPose(const Eigen::Matrix4f &Tcw)
{
    unique_lock<mutex> lock(mMutexObjects);
    SE3Tcw = Tcw;
}

}

