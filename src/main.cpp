#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include "visualization.h"
#include "odometry.h"
#include "mapping.h"

#include <colmap/base/image_reader.h>
#include <colmap/util/option_manager.h>

void SetupOptions(OptionManager& optionManager, ImageReaderOptions& readerOptions) {
    readerOptions.database_path = "/home/jack/Documents/test/database.db";
//    readerOptions.image_path = "/home/jack/Documents/ICL-lr1/livingroom1-color";
    readerOptions.image_path = "/home/jack/Documents/fr3_long_office/new_rgb";
    readerOptions.depth_path = "/home/jack/Documents/fr3_long_office/new_depth";

    readerOptions.single_camera = true;
    readerOptions.camera_model = "PINHOLE";
    readerOptions.camera_params = "525.0, 525.0, 319.5, 239.5";

    optionManager.AddAllOptions();
    optionManager.sift_extraction->num_threads = -1;
    optionManager.sift_matching->num_threads = -1;

    optionManager.sift_extraction->use_gpu = true;
    optionManager.sift_matching->use_gpu = true;

    optionManager.sift_extraction->gpu_index = "0";
    optionManager.sift_matching->gpu_index = "0";
    optionManager.sift_extraction->max_num_features = 1000;
    optionManager.sift_matching->max_num_matches = 2000;
}

int main(int argc, char* argv[]) {

    OptionManager optionManager;
    ImageReaderOptions readerOptions;
    SetupOptions(optionManager, readerOptions);
    Database database(readerOptions.database_path);
    ImageReader rgbReader(readerOptions, &database);


    Odometry odom(&optionManager, &readerOptions);
    Mapping mapper;
    Visualization visualizer;

    odom.SetVisualizer(&visualizer);
    odom.SetMapper(&mapper);
    visualizer.SetMapper(&mapper);

    std::thread* vizThread = new std::thread(&Visualization::Run, &visualizer);

    for (size_t i = 0; i < rgbReader.NumImages(); ++i) {
        rgbReader.Next(&odom.camera_, &odom.image_, &odom.bitmap_, nullptr);
        odom.Run();
        std::cout << mapper.mapPoints_.size() << std::endl;
    }

    std::string outputTraj = "/home/jack/Documents/test/icl-lr1";
    odom.WritePoseGraph(outputTraj);
    delete vizThread;
    return 0;
}