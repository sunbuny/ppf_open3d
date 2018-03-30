//
// Created by sun on 18-3-30.
//

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>
#include "PPF_PoseEstimation.h"

void PrintPointCloud(const three::PointCloud &pointcloud)
{
    using namespace three;

    bool pointcloud_has_normal = pointcloud.HasNormals();
    PrintInfo("Pointcloud has %d points.\n",
              (int)pointcloud.points_.size());

    Eigen::Vector3d min_bound = pointcloud.GetMinBound();
    Eigen::Vector3d max_bound = pointcloud.GetMaxBound();
    PrintInfo("Bounding box is: (%.4f, %.4f, %.4f) - (%.4f, %.4f, %.4f)\n",
              min_bound(0), min_bound(1), min_bound(2),
              max_bound(0), max_bound(1), max_bound(2));

    for (size_t i = 0; i < pointcloud.points_.size(); i++) {
        if (pointcloud_has_normal) {
            const Eigen::Vector3d &point = pointcloud.points_[i];
            const Eigen::Vector3d &normal = pointcloud.normals_[i];
            PrintDebug("%.6f %.6f %.6f %.6f %.6f %.6f\n",
                       point(0), point(1), point(2),
                       normal(0), normal(1), normal(2));
        } else {
            const Eigen::Vector3d &point = pointcloud.points_[i];
            PrintDebug("%.6f %.6f %.6f\n", point(0), point(1), point(2));
        }
    }
    PrintDebug("End of the list.\n\n");
}

int main(int argc, char** argv)
{

    using namespace three;
    SetVerbosityLevel(VerbosityLevel::VerboseAlways);
    auto m_raw = CreatePointCloudFromFile("/home/sun/ClionProjects/ppf_open3d/data/mian_T-rex_high.ply");
    auto s_raw = CreatePointCloudFromFile("/home/sun/ClionProjects/ppf_open3d/data/rs1.ply");
    auto coord = CreateMeshCoordinateFrame(100);


    Eigen::Vector3d min_bound = m_raw->GetMinBound();
    Eigen::Vector3d max_bound = m_raw->GetMaxBound();
    double d = (max_bound - min_bound).norm();
    std::cout << d << std::endl;


    auto m_sampled_voxel = VoxelDownSample(*m_raw,d/20);
    auto s_sampled_voxel = VoxelDownSample(*s_raw,d/20);
    Poses poses =  PPFPoseEstimation::getTransformationBetweenPointClouds(*m_sampled_voxel,*s_sampled_voxel, true);
    printPose(poses[0],"0");
    m_raw->Transform(poses[0].first.matrix().cast<double>());
    m_sampled_voxel->Transform(poses[0].first.matrix().cast<double>());
    m_raw->PaintUniformColor(Eigen::Vector3d(1,0,0));
    s_raw->PaintUniformColor(Eigen::Vector3d(0,0,1));
    m_sampled_voxel->PaintUniformColor(Eigen::Vector3d(1,0,0));
//    ICPConvergenceCriteria criteria;
//    criteria.max_iteration_ = 15;
//    RegistrationResult eval_res = EvaluateRegistration(*m_raw,*s_raw,2);
//    std::cout <<"fitness: " << eval_res.fitness_ << std::endl;
//    std::cout <<"rmse: " << eval_res.inlier_rmse_ << std::endl;
//    RegistrationResult res = RegistrationICP(*m_raw,*s_raw,2,Eigen::Matrix4d::Identity(),TransformationEstimationPointToPlane(),criteria);
//    m_raw->Transform(res.transformation_);
    DrawGeometries({m_raw,s_raw});
    return 0;
}