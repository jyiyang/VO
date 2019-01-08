//
// Created by jack on 12/28/18.
//

#include "mapping.h"

Mapping::Mapping() {

}

Mapping::~Mapping() {
    for (auto& mp: mapPoints_) {
        delete mp;
    }
}

void Mapping::Run() {

}

void Mapping::AddNewMapPoint(const Eigen::Vector3d& mapPoint) {
    std::unique_lock<std::mutex> lock(mapPointLock);
    Eigen::Vector3d* newMapPoint = new Eigen::Vector3d(mapPoint);
    mapPoints_.insert(newMapPoint);
}

std::vector<Eigen::Vector3d*> Mapping::GetMapPoints() {
    std::unique_lock<std::mutex> lock(mapPointLock);
    return std::vector<Eigen::Vector3d*>(mapPoints_.begin(), mapPoints_.end());
}
