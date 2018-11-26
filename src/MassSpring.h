#pragma once
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <nanogui/glutil.h>

typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

const static float SPRING_REST_LENGTH = 10.0f;
const static float GRAVITY = 10.0f;
const static float MASS = 10.0f;
const static float TIME_STEP = 0.2f; // 20ms

class Spring
{
private:
    Eigen::Vector3f anchor;
    Eigen::Vector3f direction;

    float rest_length = SPRING_REST_LENGTH;
    float curr_length = SPRING_REST_LENGTH;
    float k_stiffness = 7.0f;
    float k_damping = 0.5f;
    float velocity = 0;

public:
    Spring() {}
    Spring(Eigen::Vector3f start_, Eigen::Vector3f dir_)
    {
        anchor = start_;
        direction = dir_.normalized();
    }
    float getForce()
    {
        return k_stiffness * (curr_length - rest_length);
    }
    Eigen::Vector3f computeEndPos()
    {
        float spring_force = -k_stiffness * (curr_length - rest_length);
        float general_force = spring_force + GRAVITY * MASS;
        float acceleration = general_force*1.0f / MASS;
        velocity = velocity + acceleration * TIME_STEP;
        curr_length = curr_length + velocity *TIME_STEP;
        //std::cout << curr_length << std::endl;
        return anchor + curr_length * direction;
    }

};

class SingleHair
{
    std::vector<Spring> segments;
    Eigen::Vector3f root_pos;
    Eigen::Vector3f dir;
public:
    SingleHair(Eigen::Vector3f root_, Eigen::Vector3f dir_, int segment_count_)
    {
        root_pos = Eigen::Vector3f(root_);
        dir = Eigen::Vector3f(dir_);
        for (int i = 0; i < segment_count_; i++)
        {
            Eigen::Vector3f segment_i_root = root_ + dir_*segment_count_ * SPRING_REST_LENGTH;
            segments.push_back(Spring(segment_i_root, dir_));
        }
    }

    Eigen::Vector3f get_root_pos()
    {
        return root_pos;
    }

    Eigen::Vector3f get_segment_end(int i)
    {
        return segments[i].computeEndPos();
    }

};

class Hair
{
    std::vector<SingleHair> guide_strands;
    std::vector<std::vector<SingleHair>> interpolate_strands;
    int interpolate_count = -1;
    int guide_strand_count = -1;
    int segment_count = -1;
public:
    Hair() {}
    Hair(Eigen::MatrixXf hairpoints_, Eigen::MatrixXf hairnormals_, int guide_strand_count_, int interpolate_count_, int segment_count_)
        :guide_strand_count(guide_strand_count_), interpolate_count(interpolate_count_), segment_count(segment_count_)
    {
        int maxhaircount = hairpoints_.cols();
        for (int i = 0; i < guide_strand_count_; i++)
        {
            auto sample_point = hairpoints_.col(int(i * maxhaircount / guide_strand_count_));
            Eigen::Vector3f sample_hair_root_point;
            sample_hair_root_point << sample_point.x(), sample_point.y(), sample_point.z();

            auto sample_point_dir = hairnormals_.col(int(i * maxhaircount / guide_strand_count_));
            Eigen::Vector3f sample_hair_root_point_dir;
            sample_hair_root_point_dir << sample_point_dir.x(), sample_point_dir.y(), sample_point_dir.z();

            guide_strands.push_back(SingleHair(sample_hair_root_point, sample_hair_root_point_dir, segment_count_));
        }
    }

    void interpolateStrands()
    {

    }

    Eigen::MatrixXf get_positions()
    {
        Eigen::MatrixXf hair_points = Eigen::MatrixXf(3, guide_strand_count * (segment_count + 1));

        int counter = 0;
        for (int i = 0; i < guide_strand_count; i++)
        {
            auto strandrootpos = guide_strands[i].get_root_pos();

            hair_points.col(counter++) << strandrootpos.x(), strandrootpos.y(), strandrootpos.z();
            for (int j = 0; j < segment_count; j++)
            {
                auto nextpos = guide_strands[i].get_segment_end(j);
                hair_points.col(counter++) << nextpos.x(), nextpos.y(), nextpos.z();
            }
        }
        return hair_points;
    }

    Eigen::MatrixXf get_colors()
    {
        int pointcount = guide_strand_count * (segment_count + 1);
        Eigen::MatrixXf hair_point_colors = Eigen::MatrixXf(3, pointcount);
        for (int i = 0; i < pointcount; i++)
        {
            //hair_point_colors.col(i) << 1.0f, 0.0f, 0.0f;
            hair_point_colors.col(i) << 1.0f * rand() / RAND_MAX, 1.0f * rand() / RAND_MAX, 1.0f * rand() / RAND_MAX;
        }
        return hair_point_colors;
    }

    MatrixXu get_indices()
    {
        MatrixXu hair_indices = MatrixXu(2, 2 * (guide_strand_count * (segment_count + 1)));
        int counter = 0;
        for (int i = 0; i < guide_strand_count; i++)
        {
            for (int j = 0; j < segment_count; j++)
            {
                hair_indices.col(counter++) << i * (segment_count + 1) + j + 0, i * (segment_count + 1) + j + 1;
            }
        }
        return hair_indices;
    }

    int get_number_hair()
    {
        return guide_strand_count * (segment_count + 1);
    }
};
