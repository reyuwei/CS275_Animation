#pragma once
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <nanogui/glutil.h>
#include <omp.h>

namespace ms {
    typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
}
using namespace ms;
const static float SPRING_REST_LENGTH = 20.0f;
const static float GRAVITY = 0.0f;
const static float MASS = 30.0f;
static float TIME_STEP = 0.2f; // 20ms
const static Eigen::Vector3f GRAVITY_DIRECTION = Eigen::Vector3f(0.0f, 0.0f, -1.0f);


static int factorial(int number)
{
    if (number <= 1)
        return 1;
    else
        return number*factorial(number - 1);
}

static int combinator(int n, int m)
{
    int temp;
    if (n < m)
    {
        temp = n;
        n = m;
        m = temp;
    }
    return factorial(n) / (factorial(m)*factorial(n - m));
}

class Spring
{
private:
    Eigen::Vector3f anchor = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f extra_dir = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    Eigen::Vector3f endpos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    float k_stiffness = 30.0f;
    float k_damping = 30.0f;

public:
    Spring() {}
    Spring(Eigen::Vector3f start_, Eigen::Vector3f dir_)
    {
        anchor = start_;
        endpos = anchor + dir_ * SPRING_REST_LENGTH;
    }
    Eigen::Vector3f getAnchor()
    {
        return anchor;
    }
    Eigen::Vector3f getEndPos()
    {
        return endpos;
    }
    void setAnchor(Eigen::Vector3f start_)
    {
        anchor = start_;
    }
    float getCurrLength()
    {
        return (anchor - endpos).norm();
    }
    Eigen::Vector3f getCurrDir()
    {
        return (endpos - anchor).normalized();
    }
    Eigen::Vector3f getStiffF()
    {
        return -k_stiffness * (getCurrLength() - SPRING_REST_LENGTH) * getCurrDir();
    }
    Eigen::Vector3f getDampingF()
    {
        return k_damping * velocity;
    }
    Eigen::Vector3f getCurrVelocity()
    {
        return velocity;
    }
    Eigen::Vector3f animate(Spring *before, Spring *after)
    {
        if (before != NULL)
        {
            anchor = before->getEndPos();
        }

        Eigen::Vector3f gravityF = GRAVITY * MASS * GRAVITY_DIRECTION;
        //std::cout << extra_dir.norm() << std::endl;
        Eigen::Vector3f extraF = k_stiffness * extra_dir * 130.0f;
        //Eigen::Vector3f extraF = k_stiffness * extra_dir * 45.0f;
        Eigen::Vector3f general_force = getStiffF() + gravityF - getDampingF();

        if (after != NULL)
        {
            general_force = general_force - after->getStiffF() + after->getDampingF();
        }

        Eigen::Vector3f acceleration = general_force*1.0f / MASS;
        velocity = velocity + acceleration * TIME_STEP;

        Eigen::Vector3f extra_v = extra_dir * velocity.norm() * TIME_STEP;
        velocity += extra_v;
        if (velocity.norm() < 0.2f)
            velocity << 0, 0, 0;

        endpos = endpos + velocity * TIME_STEP;

        extra_dir = Eigen::Vector3f(0, 0, 0);

        /////// length constraint 
        if (abs(getCurrLength() - SPRING_REST_LENGTH) > 0.1 * SPRING_REST_LENGTH)
        {
            endpos = anchor + getCurrDir() * 1.1 * SPRING_REST_LENGTH;
        }
        return endpos;
    }

    void setExtraDir(Eigen::Vector3f extra_force_dir)
    {
        extra_dir = extra_force_dir;
    }
};

class SingleHair
{
    std::vector<Spring> segments;
    Eigen::Vector3f root;
    Eigen::Vector3f ori_root;
    Eigen::Vector3f ori_dir;
public:
    SingleHair(Eigen::Vector3f root_, Eigen::Vector3f dir_, int segment_count_)
    {
        root = root_;
        ori_root = root_;
        ori_dir = dir_;
        root = root + (dir_).normalized() * 2.0f;
        for (int i = 0; i < segment_count_; i++)
        {
            //dir_ = GRAVITY_DIRECTION;

            Eigen::Vector3f segment_i_root = root + dir_*i * SPRING_REST_LENGTH;
            segments.push_back(Spring(segment_i_root, dir_));
        }
    }

    std::vector<Eigen::Vector3f> Animate(bool doanimate)
    {
        std::vector<Eigen::Vector3f> points;
        points.push_back(root);
        for (int i = 0; i < segments.size(); i++)
        {
            if (!doanimate)
            {
                points.push_back(segments[i].getEndPos());
            }
            else
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

                points.push_back(segments[i].animate(before, after));
            }
        }
        return points;
    }

    Eigen::Vector3f get_root_dir()
    {
        //std::cout << root.x() << " " << root.y() << " " << root.z() << std::endl;
        return ori_dir;
    }

    Eigen::Vector3f get_velocity_at(int segnemt_id)
    {
        return segments[segnemt_id].getCurrVelocity();
    }

    //Eigen::Vector3f get_segment_end(int i)
    //{
    //    auto endp = segments[i].getEndPos();
    //    return endp;
    //}

    void reset()
    {
        root = ori_root;
        segments[0].setAnchor(root);
    }

    void add_force(int segment_id, Eigen::Vector3f normal)
    {
        segments[segment_id].setExtraDir(normal);
        //Animate(true);
    }

    void transform_root(Eigen::Matrix4f& model)
    {
        Eigen::Vector4f root_v4 = Eigen::Vector4f(ori_root.x(), ori_root.y(), ori_root.z(), 1.0f);
        Eigen::Vector4f vpoint = model * root_v4;
        Eigen::Vector4f newroot = vpoint;
        root << newroot.x(), newroot.y(), newroot.z();
        segments[0].setAnchor(root);
    }
};

class Hair
{
    std::vector<SingleHair> guide_strands;
    std::vector<std::vector<Eigen::Vector3f>> interpolate_strands_roots;
    int interpolate_count = -1;
    int guide_strand_count = -1;
    int segment_count = -1;
    int bline_sample_points = -1;
    int bline_points_perseg = 4;
public:
    Hair() {}
    Hair(Eigen::MatrixXf hairpoints_, Eigen::MatrixXf hairnormals_, int guide_strand_count_, int interpolate_count_, int segment_count_)
        :guide_strand_count(guide_strand_count_), interpolate_count(interpolate_count_), segment_count(segment_count_)
    {
        //bline_sample_points = segment_count + 1;
        bline_sample_points = bline_points_perseg * segment_count + segment_count + 1;
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

    Eigen::Vector3f InterpolateStrandPoint(float t, Eigen::Vector3f guide_p, Eigen::Vector3f dir_)
    {
        ///// u v vector of the plane
        Eigen::Vector3f v(-dir_.y(), dir_.z(), 0);
        v.normalize();
        Eigen::Vector3f u = v.cross(dir_);
        u.normalize();

        int a = 3; int b = 3;
        t = (t * 360) * 3.14 / 180.0;
        t = a + b*t;

        float u_s = t * cos(t);
        float v_s = t * sin(t);
        Eigen::Vector3f sample_p = u*u_s + v*v_s;
        return sample_p + guide_p;
    }

    Eigen::MatrixXf get_contrlpoints()
    {
        Eigen::MatrixXf hair_points = Eigen::MatrixXf(3, guide_strand_count * (segment_count + 1));
        for (int i = 0; i < guide_strand_count; i++)
        {
            std::vector<Eigen::Vector3f> control_points = guide_strands[i].Animate(false);
            for (int k = 0; k < (segment_count + 1); k++)
            {
                Eigen::Vector3f sample_p = control_points[k];
                hair_points.col(i * (segment_count + 1) + k) << sample_p.x(), sample_p.y(), sample_p.z();
            }
        }
        return hair_points;
    }

    Eigen::MatrixXf get_positions(Eigen::MatrixXf &hair_points_normal, bool doanimate = false)
    {
        hair_points_normal = Eigen::MatrixXf(3, guide_strand_count *(interpolate_count + 1)* bline_sample_points);
        Eigen::MatrixXf hair_points = Eigen::MatrixXf(3, guide_strand_count *(interpolate_count + 1)* bline_sample_points);

        for (int i = 0; i < guide_strand_count; i++)
        {
            std::vector<Eigen::Vector3f> control_points = guide_strands[i].Animate(doanimate);
            std::vector<Eigen::Vector3f> bazier_points = BazierLinePoint(bline_points_perseg, control_points);
            assert(bazier_points.size() == bline_sample_points);
            for (int k = 0; k < bline_sample_points; k++)
            {
                Eigen::Vector3f sample_p = bazier_points[k];
                hair_points.col(i * bline_sample_points + k) << sample_p.x(), sample_p.y(), sample_p.z();
            }
            for (int k = 0; k < bline_sample_points; k++)
            {
                int current_id = i * bline_sample_points + k;
                Eigen::Vector3f tangent(0, 0, 0);
                Eigen::Vector3f before, after, current;
                current = hair_points.col(current_id);
                if (k == 0)
                {
                    after = hair_points.col(current_id + 1);
                    tangent = after - current;
                }
                else if (k == bline_sample_points - 1)
                {
                    before = hair_points.col(current_id - 1);
                    tangent = current - before;
                }
                else
                {
                    before = hair_points.col(current_id - 1);
                    after = hair_points.col(current_id + 1);
                    tangent = 0.5f * (after - before);
                }
                hair_points_normal.col(current_id) << tangent.x(), tangent.y(), tangent.z();
            }
        }


        for (int i = 0; i < guide_strand_count; i++)
        {
            Eigen::Vector3f dir_ = guide_strands[i].get_root_dir();
            for (int j = 0; j < interpolate_count; j++)
            {
                for (int k = 0; k < bline_sample_points; k++)
                {
                    int randj = j * 1.0f / interpolate_count * 10.0f;
                    Eigen::Vector3f guide_p = hair_points.col(i * bline_sample_points + k);
                    Eigen::Vector3f inter_p = InterpolateStrandPoint(j*1.0f / interpolate_count, guide_p, dir_);
                    hair_points.col((guide_strand_count + (i*interpolate_count + j))* bline_sample_points + k) <<
                        inter_p.x(), inter_p.y(), inter_p.z();
                }

                for (int k = 0; k < bline_sample_points; k++)
                {
                    int current_id = (guide_strand_count + (i*interpolate_count + j))* bline_sample_points + k;

                    Eigen::Vector3f tangent(0, 0, 0);
                    Eigen::Vector3f before, after, current;
                    current = hair_points.col(current_id);
                    if (k == 0)
                    {
                        after = hair_points.col(current_id + 1);
                        tangent = after - current;
                    }
                    else if (k == bline_sample_points - 1)
                    {
                        before = hair_points.col(current_id - 1);
                        tangent = current - before;
                    }
                    else
                    {
                        before = hair_points.col(current_id - 1);
                        after = hair_points.col(current_id + 1);
                        tangent = 0.5f * (after - before);
                    }
                    hair_points_normal.col(current_id) << tangent.x(), tangent.y(), tangent.z();
                }
            }
        }
        return hair_points;

    }

    Eigen::MatrixXf get_colors()
    {
        int pointcount = guide_strand_count * (bline_sample_points) * (interpolate_count + 1);
        int haircount = guide_strand_count  * (interpolate_count + 1);
        Eigen::MatrixXf hair_point_colors = Eigen::MatrixXf(3, pointcount);
        //for (int i = 0; i + 1 < pointcount; i += 2)
        for (int i = 0; i < haircount; i++)
        {
            float r = 1.0f * rand() / RAND_MAX;
            float g = 1.0f * rand() / RAND_MAX;
            float b = 1.0f * rand() / RAND_MAX;

            //r = 237 / 255.0f;
            //g = 75 / 255.0f;
            //b = 75 / 255.0f;

            r = 0.8f;
            g = 0.8f;
            b = 0.0f;

            for (int k = 0; k < bline_sample_points; k++)
            {
                hair_point_colors.col(i*(bline_sample_points)+k) << r, g, b;
            }
            //hair_point_colors.col(i) << r, g, b;
            //hair_point_colors.col(i + 1) << r, g, b;
        }
        return hair_point_colors;
    }

    MatrixXu get_indices()
    {
        int temp_segment_count = bline_sample_points - 1;
        MatrixXu hair_indices = MatrixXu(2, 2 * (guide_strand_count *(interpolate_count + 1)* (temp_segment_count + 1)));
        int counter = 0;
        for (int i = 0; i < guide_strand_count*interpolate_count; i++)
        {
            for (int j = 0; j < temp_segment_count; j++)
            {
                hair_indices.col(counter++) << i * (temp_segment_count + 1) + j + 0, i * (temp_segment_count + 1) + j + 1;
            }
        }
        return hair_indices;
    }

    Eigen::Vector3f get_velocity_at(int strand_id, int segment_id)
    {
       return guide_strands[strand_id].get_velocity_at(segment_id);
    }

    int get_number_hair()
    {
        return guide_strand_count *(interpolate_count + 1)* bline_sample_points;
    }

    void add_force(int strand_id, int segment_id, Eigen::Vector3f normal)
    {
        guide_strands[strand_id].add_force(segment_id, normal);
    }

    void transform(Eigen::Matrix4f& model)
    {
        for (int i = 0; i < guide_strands.size(); i++)
        {
            guide_strands[i].transform_root(model);
        }
    }
    void reset()
    {
        for (int i = 0; i < guide_strands.size(); i++)
        {
            guide_strands[i].reset();
        }
    }
    std::vector<Eigen::Vector3f> BazierLinePoint(int point_per_segment, std::vector<Eigen::Vector3f> control_points)
    {
        std::vector<Eigen::Vector3f> results;

        float step = 1.0f / (point_per_segment + 1);
        for (int i = 0; i < control_points.size() - 1; i++)
        {
            for (float t = 0; t < 1; t += step)
            {
                //results.push_back(control_points[i]);

                Eigen::Vector3f sample_p(0, 0, 0);
                Eigen::Vector3f p0, p1, p00, p2;
                p0 << control_points[i].x(), control_points[i].y(), control_points[i].z();
                p1 << control_points[i + 1].x(), control_points[i + 1].y(), control_points[i + 1].z();

                p00 = p0; p2 = p1;

                if (i - 1 >= 0) p00 << control_points[i - 1].x(), control_points[i - 1].y(), control_points[i - 1].z();
                if (i + 2 < control_points.size()) p2 << control_points[i + 2].x(), control_points[i + 2].y(), control_points[i + 2].z();

                Eigen::Vector3f c0 = p0 + (p1 - p00) / 6;
                Eigen::Vector3f d0 = p1 - (p2 - p0) / 6;

                float a = pow(1 - t, 3);
                float b = 3 * t * pow(1 - t, 2);
                float c = 3 * t * t * (1 - t);
                float d = t * t * t;
                sample_p = a * p0 + b * c0 + c * d0 + d * p1;
                results.push_back(sample_p);
            }
        }
        results.push_back(control_points[control_points.size() - 1]);
        return results;
    }

};

