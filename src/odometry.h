//
// Created by jack on 12/25/18.
//

#ifndef VO_ODOMETRY_H
#define VO_ODOMETRY_H

#include <colmap/feature/extraction.h>
#include <colmap/feature/matching.h>
//#include <colmap/lib/SiftGPU/SiftGPU.h>
#include <Eigen/StdVector>

using namespace colmap;

class Visualization;
class Mapping;

namespace colmap {
    class ImageReader;
    class ImageReaderOptions;
    class Database;
    class OptionManager;
    class TwoViewGeometry;
}

class Odometry {
public:
    typedef std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> PoseGraph;

    enum TrackerState {
        INIT = 0,
        TRACKING = 1,
        LOST = 2
    };

    Odometry(OptionManager* optionManager, ImageReaderOptions* readerOptions);
    ~Odometry();
    void Run();
    void FeatureExtraction();
    void FeatureMatchingAgainstPreviousFrame();
    void FeatureMatchingAgainstMap();
    void CalculateTwoViewGeometry();
    void CalculatePnP();
    void WritePoseGraph(std::string& filename);
    void SetVisualizer(Visualization* visualizer);
    void SetMapper(Mapping *mapper);
    void DrawCameraPose(const Eigen::Matrix4d& currPose);

public:
    Camera camera_;
    Image image_;

    Bitmap bitmap_;

    FeatureDescriptors currDescriptors_, prevDescriptors_;
    FeatureKeypoints currKeypoints_, prevKeypoints_;
    FeatureMatches matches_;
    Eigen::Matrix4d T12_, currPose_;
    TrackerState state_;

private:
    OptionManager* option_manager_;
    ImageReaderOptions* reader_options_;
//    Database* database_;
//    ImageReader* reader_;
    SiftGPU* siftGPU_;
    SiftMatchGPU* siftMatchGPU_;

    TwoViewGeometry::Options* twoViewOptions_;
    TwoViewGeometry* twoViewGeometry_;
    PoseGraph poseGraph_;

    Visualization* visualizer_;
    Mapping* mapper_;

};


#endif //VO_ODOMETRY_H
