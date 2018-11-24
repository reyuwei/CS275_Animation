#pragma once
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <nanogui/glutil.h>

class Spring
{
private:
    Eigen::Vector3f start_pos;
    Eigen::Vector3f end_pos;

    float rest_length = 10;
    float curr_length = -1;
    float k_stiffness = 0.5f;
    float k_damping = 0.5f;

public:
    float getForce()
    {
        return k_stiffness * (curr_length - rest_length);
    }
    Eigen::Vector3f computeEndPos()
    {

    }

};

class SingleHair
{
    std::vector<Spring> segments;
    Eigen::Vector3f root_pos;
public:
    SingleHair(Eigen::Vector3f root_, int segment_count_)
    {
        root_pos = Eigen::Vector3f(root_);
        for (int i = 0; i < segment_count_; i++)
            segments.push_back(Spring());
    }
};

class Hair
{
    std::vector<SingleHair> guide_strands;
    std::vector<std::vector<SingleHair>> interpolate_strands;
    int interpolate_count = -1;
public:
    Hair() {}
    Hair(Eigen::MatrixXf hairpoints_, int guide_strand_count_, int interpolate_count_, int segment_count_) :interpolate_count(interpolate_count_)
    {
        int hair_points_count = hairpoints_.cols();
        for (int i = 0; i < guide_strand_count_; i++)
        {
            auto sample_point = hairpoints_.col(hair_points_count % guide_strand_count_ + i);
            Eigen::Vector3f sample_hair_point;
            sample_hair_point << sample_point.x(), sample_point.y(), sample_point.z();
            guide_strands.push_back(SingleHair(sample_hair_point, segment_count_));
        }
    }

    void interpolateStrands()
    {

    }
};
