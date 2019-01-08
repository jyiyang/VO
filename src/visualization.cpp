//
// Created by jack on 12/28/18.
//

#include "visualization.h"
#include "mapping.h"

Visualization::Visualization() {
    viewPointX = 0;
    viewPointY = -10;
    viewPointZ = -0.1f;
    viewPointF = 2000;
    camPose = Eigen::Matrix4d::Identity();
}

void Visualization::SetMapper(Mapping *mapper) {
    mapper_ = mapper;
}

void Visualization::Run() {
    pangolin::CreateWindowAndBind("Visual Odometry", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, viewPointF, viewPointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(viewPointX, viewPointY, viewPointZ, 0, 0, 0, 0.0, -1.0, 0.0)
            );

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    bool follow = true;

    while (1) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        UpdateGLCameraPose(Twc);
        if (follow) {
            s_cam.Follow(Twc);
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        DrawCurrentCamera(Twc);
        DrawAllCameras();
        DrawMapPoints();
        pangolin::FinishFrame();
    }
}

void Visualization::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
    const float &w = 0.15;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(2);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void Visualization::UpdateGLCameraPose(pangolin::OpenGlMatrix& Twc) {
    std::unique_lock<std::mutex> lock(cameraMutex);
    Twc.m[0] = camPose(0, 0);
    Twc.m[1] = camPose(1, 0);
    Twc.m[2] = camPose(2, 0);
    Twc.m[3] = 0.0;

    Twc.m[4] = camPose(0, 1);
    Twc.m[5] = camPose(1, 1);
    Twc.m[6] = camPose(2, 1);
    Twc.m[7] = 0.0;

    Twc.m[8] = camPose(0, 2);
    Twc.m[9] = camPose(1, 2);
    Twc.m[10] = camPose(2, 2);
    Twc.m[11] = 0.0;

    Twc.m[12] = camPose(0, 3);
    Twc.m[13] = camPose(1, 3);
    Twc.m[14] = camPose(2, 3);
    Twc.m[15] = 1.0;
}

void Visualization::SetCameraPose(const Eigen::Matrix4d &currPose) {
    std::unique_lock<std::mutex> lock(cameraMutex);
    camPose = currPose.inverse();
    poseGraph.push_back(camPose);
}

void Visualization::DrawAllCameras() {
    const float &w = 0.15;
    const float h = w*0.75;
    const float z = w*0.6;

    for (size_t i = 0; i < poseGraph.size(); i = i + 10) {
        Eigen::Matrix4d pose = poseGraph[i];

        glPushMatrix();

        glMultMatrixd(pose.data());

        glLineWidth(2);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void Visualization::DrawMapPoints() {
    std::vector<Eigen::Vector3d*> mapPoints = mapper_->GetMapPoints();
    glPointSize(2);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for (auto& mpPt: mapPoints) {
        glVertex3f(mpPt->x(), mpPt->y(), mpPt->z());
    }
    glEnd();
}
