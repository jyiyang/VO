//
// Created by jack on 12/28/18.
//

#ifndef VO_MAPPING_H
#define VO_MAPPING_H

#include <list>
#include <set>
#include <mutex>
#include <Eigen/Core>

class Mapping {
public:
    Mapping();
    ~Mapping();
    void Run();
    void AddNewMapPoint(const Eigen::Vector3d& mapPoint);
    std::vector<Eigen::Vector3d*> GetMapPoints();

    std::set<Eigen::Vector3d*> mapPoints_;

    std::mutex mapPointLock;

};


#endif //VO_MAPPING_H
