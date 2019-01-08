//
// Created by jack on 12/25/18.
//
#include <fstream>
#include <iomanip>
#include <chrono>

#include "visualization.h"
#include <colmap/lib/SiftGPU/SiftGPU.h>
#include <colmap/util/threading.h>
#include <colmap/feature/sift.h>
#include <colmap/base/image_reader.h>
#include <colmap/util/option_manager.h>
#include <colmap/base/pose.h>
#include <colmap/base/triangulation.h>
#include <colmap/base/projection.h>

#include "odometry.h"
#include "mapping.h"

inline void ScaleKeypoints(const Bitmap& bitmap, const Camera& camera,
                    FeatureKeypoints* keypoints) {
    if (static_cast<size_t>(bitmap.Width()) != camera.Width() ||
        static_cast<size_t>(bitmap.Height()) != camera.Height()) {
        const float scale_x = static_cast<float>(camera.Width()) / bitmap.Width();
        const float scale_y = static_cast<float>(camera.Height()) / bitmap.Height();
        for (auto& keypoint : *keypoints) {
            keypoint.Rescale(scale_x, scale_y);
        }
    }
}

Odometry::Odometry(OptionManager* optionManager, ImageReaderOptions* readerOptions) {
    reader_options_ = readerOptions;
    option_manager_ = optionManager;

    siftGPU_ = new SiftGPU();
    CreateSiftGPUExtractor(*(option_manager_->sift_extraction), siftGPU_);
    siftMatchGPU_ = new SiftMatchGPU();
    if (!CreateSiftGPUMatcher(*(option_manager_->sift_matching), siftMatchGPU_)) {
        std::cout << "Error in creating sift gpu" << std::endl;
    }

    twoViewOptions_ = new TwoViewGeometry::Options();
    twoViewOptions_->ransac_options.min_num_trials = 100;
    twoViewOptions_->ransac_options.max_error = 4.0;
    twoViewGeometry_ = new TwoViewGeometry();

    T12_ = Eigen::Matrix4d::Identity();
    currPose_ = Eigen::Matrix4d::Identity();
    state_ = INIT;
}

void Odometry::Run() {

    ExtractSiftFeaturesGPU(*(option_manager_->sift_extraction),
                           bitmap_,
                           siftGPU_,
                           &currKeypoints_,
                           &currDescriptors_);
    ScaleKeypoints(bitmap_, camera_, &currKeypoints_);

     switch (state_) {
         case INIT:
             state_ = TRACKING;
             break;
         case TRACKING:
            {
                MatchSiftFeaturesGPU(*(option_manager_->sift_matching),
                                  &prevDescriptors_,
                                  &currDescriptors_,
                                  siftMatchGPU_,
                                  &matches_);

                std::vector<Eigen::Vector2d> points1, points2;
                points1.reserve(prevKeypoints_.size());
                points2.reserve(currKeypoints_.size());

                for (const auto& kpt: prevKeypoints_) {
                    Eigen::Vector2d pt;
                    pt << kpt.x, kpt.y;
                    points1.emplace_back(pt);
                }
                for (const auto& kpt: currKeypoints_) {
                    Eigen::Vector2d pt;
                    pt << kpt.x, kpt.y;
                    points2.emplace_back(pt);
                }
                twoViewGeometry_->EstimateCalibrated(camera_, points1, camera_, points2,
                                                  matches_, *twoViewOptions_);
                if (!twoViewGeometry_->EstimateRelativePose(camera_, points1, camera_, points2)) {
                    std::cout << "Unable to estimate pose, use constant velocity model. " << std::endl;
                }
                else {
                    // update
                    T12_.topLeftCorner<3, 3>() = QuaternionToRotationMatrix(twoViewGeometry_->qvec);
                    T12_.topRightCorner<3, 1>() = twoViewGeometry_->tvec;
                }
                Eigen::Matrix3x4d projMatrix1, projMatrix2;
                Eigen::Vector3d projCenter1, projCenter2;
                projMatrix1 = camera_.CalibrationMatrix() * currPose_.topLeftCorner<3, 4>();
                projCenter1 = ProjectionCenterFromMatrix(projMatrix1);
                currPose_ = T12_ * currPose_;
                projMatrix2 = camera_.CalibrationMatrix() * currPose_.topLeftCorner<3, 4>();
                projCenter2 = ProjectionCenterFromMatrix(projMatrix2);

                for (const auto& corr: matches_) {
                    Eigen::Vector2d point1n = camera_.ImageToWorld(points1[corr.point2D_idx1]);
                    Eigen::Vector2d point2n = camera_.ImageToWorld(points2[corr.point2D_idx2]);
                    Eigen::Vector3d xyz = TriangulatePoint(projMatrix1, projMatrix2, point1n, point2n);
                    double triAngle = CalculateTriangulationAngle(projCenter1, projCenter2, xyz);
                    if (triAngle > 0.02 &&
                            HasPointPositiveDepth(projMatrix1, xyz) && HasPointPositiveDepth(projMatrix2, xyz)) {
                        mapper_->AddNewMapPoint(xyz);
                    }
                }

                break;
            }
         case LOST:
             break;
    }

    prevKeypoints_ = currKeypoints_;
    prevDescriptors_ = currDescriptors_;

    DrawCameraPose(currPose_);
    poseGraph_.push_back(currPose_);
}

void Odometry::WritePoseGraph(std::string& filename) {
    filename.append(".freiburg");

    std::ofstream f;
    f.open(filename.c_str(), std::fstream::out);
    for (size_t i = 0; i < poseGraph_.size(); ++i) {

        std::stringstream strs;
        strs << std::setprecision(6) << std::fixed << i << " ";

        Eigen::Vector3d trans = poseGraph_[i].topRightCorner<3, 1>();
        Eigen::Matrix3d rots  = poseGraph_[i].topLeftCorner<3, 3>();
        f << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";
        Eigen::Vector4d qvec = RotationMatrixToQuaternion(rots);
        f << qvec(1) << " " << qvec(2) << " " << qvec(3) << " " << qvec(0) << "\n";
    }
    f.close();
}

Odometry::~Odometry() {
    delete siftGPU_;
    delete siftMatchGPU_;
    delete twoViewOptions_;
    delete twoViewGeometry_;
}

void Odometry::DrawCameraPose(const Eigen::Matrix4d& currPose) {
    visualizer_->SetCameraPose(currPose);
}

void Odometry::SetVisualizer(Visualization *visualizer) {
    visualizer_ = visualizer;
}

void Odometry::SetMapper(Mapping *mapper) {
    mapper_ = mapper;
}


