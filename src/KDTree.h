#pragma once
#ifndef KETREE_H
#define KETREE_H

#include <string>
#include <Eigen/Sparse>
#include <vector>
#define EPSILON 1e-2
#define M_PI 3.14159265358979323846

class Ray
{
public:
    Eigen::Vector3f u_;
    Eigen::Vector3f dir_;
public:
    Ray();
    Ray(Eigen::Vector3f s, Eigen::Vector3f dir);
    Eigen::Vector3f GetPoint(double t);
    ~Ray();
};

class BoundingBox
{
public:
    std::vector<Eigen::Vector3f> bounds;
    BoundingBox();

    void Expand(BoundingBox another);
    bool RayHitTest(Ray ray);
};

class Tri
{
public:
    Eigen::Vector3f center_;
    Eigen::Vector3f color_;
    std::vector<Eigen::Vector3f> colortable_;

    Eigen::Vector3f normal_;
    Eigen::Vector3f v1_, v2_, v3_;
    Tri(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c);
    Tri(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f normal);
    Tri();
    Eigen::Vector3f RayHitTest(Ray ray, Eigen::Vector3f &outnormal);
    BoundingBox GetBBox();
};

class Triidx
{
public:
    int v1i_, v2i_, v3i_;
    Triidx(int a, int b, int c);
    Triidx();
    ~Triidx();
};


struct Intersect
{
    Eigen::Vector3f hitpoint;
    Eigen::Vector3f outnormal;
};

class KDNode
{
public:
    BoundingBox box;
    KDNode *left, *right;
    std::vector<Tri*> triangles;

    KDNode();
    KDNode* Build(std::vector<Tri*>& tris, int depth);
    bool Hit(KDNode *node, Ray ray, Eigen::Vector3f &outnormal, Tri &hittriangle);
    Intersect* Hit(KDNode *node, Ray ray);
};



#endif //KDTREE_H