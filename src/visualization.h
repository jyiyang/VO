//
// Created by jack on 12/28/18.
//

#ifndef VO_VISUALIZATION_H
#define VO_VISUALIZATION_H

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <mutex>

class Mapping;

class Visualization {
public:
    Visualization();
    void Run();
    void DrawCurrentCamera(pangolin::OpenGlMatrix& Twc);
    void DrawAllCameras();
    void UpdateGLCameraPose(pangolin::OpenGlMatrix& Twc);
    void SetCameraPose(const Eigen::Matrix4d& currPose);
    void DrawMapPoints();
    void SetMapper(Mapping* mapper);

public:
    float viewPointX, viewPointY, viewPointZ, viewPointF;
    Eigen::Matrix4d camPose;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poseGraph;
    std::mutex cameraMutex;

private:
    Mapping* mapper_;
};


#endif //VO_VISUALIZATION_H
