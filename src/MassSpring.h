#pragma once
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <nanogui/glutil.h>

typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

const static float SPRING_REST_LENGTH = 30.0f;
const static float GRAVITY = 10.0f;
const static float MASS = 10.0f;
const static float TIME_STEP = 0.2f; // 20ms
const static Eigen::Vector3f GRAVITY_DIRECTION = Eigen::Vector3f(0.0f, 0.0f, -1.0f);

class Spring
{
private:
    Eigen::Vector3f anchor;
    Eigen::Vector3f direction;
    float rest_length = SPRING_REST_LENGTH;

    Eigen::Vector3f endpos;
    Eigen::Vector3f velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    float k_stiffness = 7.0f;
    float k_damping = 3.0f;

public:
    Spring() {}
    Spring(Eigen::Vector3f start_, Eigen::Vector3f dir_)
    {
        anchor = start_;
        direction = dir_;
        endpos = anchor + dir_ * rest_length;
    }
    Eigen::Vector3f getAnchor()
    {
        return anchor;
    }
    Eigen::Vector3f getEndPos()
    {
        return endpos;
    }

    float getCurrLength()
    {
        return (anchor - endpos).stableNorm();
    }

    Eigen::Vector3f getGravityF()
    {
        return GRAVITY * MASS * GRAVITY_DIRECTION;
    }
    Eigen::Vector3f getStiffF()
    {
        return -k_stiffness * (getCurrLength() - rest_length) * direction;
    }
    Eigen::Vector3f getDampingF()
    {
        return k_damping * velocity;
    }

    void animate(Spring *before, Spring *after)
    {
        if (before != NULL)
        {
            anchor = before->getEndPos();
        }
        Eigen::Vector3f general_force = getStiffF() + getGravityF() - getDampingF();
        if (after != NULL)
        {
            general_force = general_force - after->getStiffF() + after->getDampingF();
        }

        Eigen::Vector3f acceleration = general_force*1.0f / MASS;
        velocity = velocity + acceleration * TIME_STEP;
        endpos = endpos + velocity * TIME_STEP;
        direction = endpos - anchor;
        direction.normalize();
    }



};

class SingleHair
{
    std::vector<Spring> segments;
    Eigen::Vector3f root;
public:
    SingleHair(Eigen::Vector3f root_, Eigen::Vector3f dir_, int segment_count_)
    {
        root = root_;
        for (int i = 0; i < segment_count_; i++)
        {
            dir_ = GRAVITY_DIRECTION;
            Eigen::Vector3f segment_i_root = root_ + dir_*segment_count_ * SPRING_REST_LENGTH;
            segments.push_back(Spring(segment_i_root, dir_));
        }
    }

    void Animate()
    {
        for (int i = 0; i < segments.size(); i++)
        {
            Spring *before;
            Spring *after;
            if (i == 0)
                before = NULL;
            else
                before = &segments[i - 1];
            if (i == segments.size() - 1)
                after = NULL;
            else
                after = &segments[i + 1];

            segments[i].animate(before, after);
        }
    }

    Eigen::Vector3f get_root_pos()
    {
        //std::cout << root.x() << " " << root.y() << " " << root.z() << std::endl;
        return root;
    }

    Eigen::Vector3f get_segment_end(int i)
    {
        auto endp = segments[i].getEndPos();
        //std::cout << endp.x() << " " << endp.y() << " " << endp.z() << std::endl;
        return endp;
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
            guide_strands[i].Animate();

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
