#pragma once
#include <map>
#include <vector>
#include <nanogui/glutil.h>
using namespace nanogui;


class TransFactor
{
private:
    float scale;
    float trans_x;
    float trans_y;
    float trans_z;
    float rotation_x;
    float rotation_y;
    float rotation_z;
    float rotation_w;
public:
    TransFactor()
    {
        scale = 0.0f;
        trans_x = 0.0f;
        trans_y = 0.0f;
        trans_z = 0.0f;
        rotation_x = 0.0f;
        rotation_y = 0.0f;
        rotation_z = 0.0f;
        rotation_w = 1.0f;
    }

    TransFactor(const TransFactor &other)
    {
        scale = other.scale;
        trans_x = other.trans_x;
        trans_y = other.trans_y;
        trans_z = other.trans_z;
        rotation_w = other.rotation_w;
        rotation_x = other.rotation_x;
        rotation_y = other.rotation_y;
        rotation_z = other.rotation_z;
    }

    Matrix4f ToMatrix4f()
    {
        Eigen::Matrix4f trans;
        trans.setIdentity();

        nanogui::Quaternionf rotation(rotation_w, rotation_x, rotation_y, rotation_z);
        trans.topLeftCorner<3, 3>() = rotation.toRotationMatrix();
        trans = trans * nanogui::scale(Eigen::Vector3f::Constant(scale));
        trans = trans * nanogui::translate(Eigen::Vector3f(trans_x, trans_y, trans_z));
        return trans;
    }

    Eigen::Vector3f GetTranslation()
    {
        return Eigen::Vector3f(trans_x, trans_y, trans_z);
    }

    nanogui::Quaternionf GetRotation()
    {
        return nanogui::Quaternionf(rotation_w, rotation_x, rotation_y, rotation_z);
    }

    void SetTranslation(const Eigen::Vector3f trans)
    {
        trans_x = trans.x();
        trans_y = trans.y();
        trans_z = trans.z();
    }

    void SetRotation(const nanogui::Quaternionf &rot)
    {
        rotation_x = rot.x();
        rotation_y = rot.y();
        rotation_z = rot.z();
        rotation_w = rot.w();
    }

    float GetScale()
    {
        return scale;
    }

    void SetScale(float s)
    {
        scale = s;
    }

    static nanogui::Quaternionf InterpolateQ(const nanogui::Quaternionf &s,const nanogui::Quaternionf &e, float t)
    {
        nanogui::Quaternionf inter_q;
        inter_q.setIdentity();

        // Calculate angle between them
        double cosHalfTheta = (s.w() * e.w()) + (s.x() * e.x()) + (s.y() * e.y()) + (s.z() * e.z());

        if (abs(cosHalfTheta) >= 1.0)
        {
            inter_q.w() = s.w();
            inter_q.x() = s.x();
            inter_q.y() = s.y();
            inter_q.z() = s.z();

            return inter_q;
        }

        // Calculate temporary values
        double halfTheta = acos(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0f - cosHalfTheta * cosHalfTheta);

        // if theta = 180 degrees then result is not fully defined
        // we could rotate around any axis normal to s or e
        if (fabs(sinHalfTheta) < 0.001)
        {
            inter_q.w() = (s.w() * 0.5 + e.w() * 0.5);
            inter_q.x() = (s.x() * 0.5 + e.x() * 0.5);
            inter_q.y() = (s.y() * 0.5 + e.y() * 0.5);
            inter_q.z() = (s.z() * 0.5 + e.z() * 0.5);

            return inter_q;
        }

        double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
        double ratioB = sin(t * halfTheta) / sinHalfTheta;

        // Calculate Quaternion
        inter_q.w() = (s.w() * ratioA + e.w() * ratioB);
        inter_q.x() = (s.x() * ratioA + e.x() * ratioB);
        inter_q.y() = (s.y() * ratioA + e.y() * ratioB);
        inter_q.z() = (s.z() * ratioA + e.z() * ratioB);

        return inter_q;
    }
};


class KeyFrameAnimator
{
private:
    int keyframe_num = 0;
    int frame_num = 0;
    std::vector<int> keyframe_id;
    std::map<int, int> frame_id_mapper;
    //std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> view_matrix;
    std::vector<TransFactor> view_matrix;

public:
    KeyFrameAnimator(TransFactor initview, int frame_num_)
    {
        AddKeyFrame(0, initview);
        frame_num = frame_num_;
    }

    Matrix4f GetView(int frameid)
    {
        int idx = GetKeyFrameIndex(frameid);
        if (idx != -1)
            return view_matrix[idx].ToMatrix4f();
        else
        {
            int before_idx = 0, before_dis = 0;
            int after_idx = keyframe_num - 1, after_dis = 0;
            std::map<int, int>::iterator itr = frame_id_mapper.begin();
            while (itr != frame_id_mapper.end())
            {
                if (itr->first <= frameid && itr->second >= before_idx)
                {
                    before_idx = itr->second;
                    before_dis = frameid - itr->first;
                }
                if (itr->first >= frameid && itr->second <= after_idx)
                {
                    after_idx = itr->second;
                    after_dis = itr->first - frameid;
                }
                itr++;
            }

            if (after_idx == before_idx)
            {
                after_idx = 0;
                after_dis = frame_num - frameid;
                //return before_view.ToMatrix4f();
            }

            TransFactor before_view = view_matrix[before_idx];
            TransFactor after_view = view_matrix[after_idx];

            float t = before_dis / (before_dis + after_dis + 1e-5);

            TransFactor inter_trans;
            inter_trans.SetScale(before_view.GetScale() * (1 - t) + t * after_view.GetScale());

            Eigen::Vector3f before_pos = before_view.GetTranslation();
            Eigen::Vector3f after_pos = after_view.GetTranslation();

            inter_trans.SetTranslation(before_pos *(1 - t) + t * after_pos);


            inter_trans.SetRotation(TransFactor::InterpolateQ(before_view.GetRotation(), after_view.GetRotation(), t));



            //////////////////////////

            return inter_trans.ToMatrix4f();
        }
    }

    bool AddKeyFrame(int frameid, TransFactor view)
    {
        if (GetKeyFrameIndex(frameid) == -1)
        {
            frame_id_mapper.insert(std::pair<int, int>(frameid, keyframe_num));
            view_matrix.push_back(view);
            keyframe_id.push_back(frameid);
            keyframe_num++;
            return true;
        }
        else
        {
            std::cout << "EXISTING KEYFRAME, DO YOU WANT TO EDIT?" << std::endl;
            return false;
        }
    }

    bool EditKeyFrame(int frameid, TransFactor view)
    {
        int idx = GetKeyFrameIndex(frameid);
        if (idx != -1)
        {
            view_matrix[idx] = view;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool DeleteKeyFrame(int frameid)
    {
        int idx = GetKeyFrameIndex(frameid);
        if (idx != -1)
        {
            if (idx == 0 && frameid == 0) 
            {
                std::cout << "CAN NOT DELETE FIRST KEY FRAME!!" << std::endl;
                return false;
            }
            keyframe_num--;
            view_matrix.erase(view_matrix.begin() + idx);
            keyframe_id.erase(keyframe_id.begin() + idx);

            std::map<int, int>::iterator iter;
            iter = frame_id_mapper.find(frameid);
            frame_id_mapper.erase(iter);

            std::map<int, int>::iterator itr = frame_id_mapper.begin();
            while (itr != frame_id_mapper.end())
            {
                if (itr->second > idx)
                    frame_id_mapper[itr->first]--;
                itr++;
            }
            return true;
        }
        else
            return false;
    }

    int GetKeyFrameIndex(int frameid)
    {
        std::map<int, int>::iterator iter;
        iter = frame_id_mapper.find(frameid);
        if (iter != frame_id_mapper.end())
            return iter->second;
        else
        {
            //std::cout << "NOT A KEYFRAME!!!" << std::endl;
            return -1;
        }
    }
};


