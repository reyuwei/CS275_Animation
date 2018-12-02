#include "KDTree.h"
Ray::Ray()
{
}



Ray::Ray(Eigen::Vector3f s, Eigen::Vector3f dir)
{
    this->u_ = s;
    this->dir_ = dir;
    dir_.normalize();
}


Eigen::Vector3f Ray::GetPoint(double t)
{
    Eigen::Vector3f point;
    point = this->dir_*t + this->u_;
    return point;
}

Ray::~Ray()
{
}


BoundingBox::BoundingBox()
{
    bounds.push_back(Eigen::Vector3f(0, 0, 0));
    bounds.push_back(Eigen::Vector3f(0, 0, 0));
}

void BoundingBox::Expand(BoundingBox another)
{
    double minx = std::min(this->bounds[0].x(), another.bounds[0].x());
    double miny = std::min(this->bounds[0].y(), another.bounds[0].y());
    double minz = std::min(this->bounds[0].z(), another.bounds[0].z());

    double maxx = std::max(this->bounds[1].x(), another.bounds[1].x());
    double maxy = std::max(this->bounds[1].y(), another.bounds[1].y());
    double maxz = std::max(this->bounds[1].z(), another.bounds[1].z());

    this->bounds[0] = Eigen::Vector3f(minx, miny, minz);
    this->bounds[1] = Eigen::Vector3f(maxx, maxy, maxz);
}
bool BoundingBox::RayHitTest(Ray ray)
{
    Eigen::Vector3f inv_dir = Eigen::Vector3f(1.0f / ray.dir_.x(), 1.0f / ray.dir_.y(), 1.0f / ray.dir_.z());
    Eigen::Vector3f tMin = inv_dir.cwiseProduct(this->bounds[0] - ray.u_);
    Eigen::Vector3f tMax = inv_dir.cwiseProduct(this->bounds[1] - ray.u_);

    Eigen::Vector3f t1 = Eigen::Vector3f(std::min(tMin.x(), tMax.x()), std::min(tMin.y(), tMax.y()), std::min(tMin.z(), tMax.z()));
    Eigen::Vector3f t2 = Eigen::Vector3f(std::max(tMin.x(), tMax.x()), std::max(tMin.y(), tMax.y()), std::max(tMin.z(), tMax.z()));
    float tNear = std::max(std::max(t1.x(), t1.y()), t1.z());
    float tFar = std::min(std::min(t2.x(), t2.y()), t2.z());
    if (tNear > tFar)
        return false;
    else
        return true;
}


Tri::Tri(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f normal)
{
    this->v1_ = a;
    this->v2_ = b;
    this->v3_ = c;
    this->normal_ = normal;
    this->center_ = (a + b + c) / 3;
}

Tri::Tri(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c)
{
    this->v1_ = a;
    this->v2_ = b;
    this->v3_ = c;
    this->center_ = (a + b + c) / 3;

    Eigen::Vector3f normal = (b - a).cross(c - a);
    normal.normalize();
    this->normal_ = normal;
}

Tri::Tri()
{
    this->v1_ = this->v2_ = this->v3_ = Eigen::Vector3f();
}

Eigen::Vector3f Tri::RayHitTest(Ray ray, Eigen::Vector3f &outnormal)
{
    outnormal = this->normal_;
    Eigen::Vector3f hitpoint = Eigen::Vector3f(0, 0, 0);

    Eigen::Vector3f v1 = ray.u_;
    Eigen::Vector3f raydir = ray.dir_;

    if (raydir.dot(this->normal_) == 0)//didn't hit
        return hitpoint;


    double a = raydir.x();
    double b = raydir.y();
    double c = raydir.z();

    double A = this->normal_.x();
    double B = this->normal_.y();
    double C = this->normal_.z();
    double D = -this->center_.dot(this->normal_);


    double t = (-D - A * v1.x() - B * v1.y() - C * v1.z()) / (A * a + B * b + C * c);
    if (t > EPSILON)
    {
        hitpoint = ray.GetPoint(t);

        Eigen::Vector3f v1h = v1_ - hitpoint; v1h.normalize();
        Eigen::Vector3f v2h = v2_ - hitpoint; v2h.normalize();
        Eigen::Vector3f v3h = v3_ - hitpoint; v3h.normalize();

        //if point in triangle
        double cos1 = v1h.dot(v2h);
        double cos2 = v2h.dot(v3h);
        double cos3 = v3h.dot(v1h);

        double theta1 = acos(cos1);
        double theta2 = acos(cos2);
        double theta3 = acos(cos3);

        if (abs(2 * M_PI - (theta1 + theta2 + theta3)) < EPSILON)
        {
            return hitpoint;
        }
    }
    return Eigen::Vector3f(0, 0, 0);
}

//Bbox Tri::GetBBox()
//{
//	Eigen::Vector3f center = (v1_ + v2_ + v3_) / 3;
//	double length12 = (v1_ - v2_).length();
//	double length13 = (v1_ - v3_).length();
//	double length23 = (v3_ - v2_).length();
//	if (length12 >= length13 && length12 >= length23)
//		return Bbox(center, length12);
//	if (length13 >= length12 && length13 >= length23)
//		return Bbox(center, length13);
//	if (length23 >= length13 && length23 >= length12)
//		return Bbox(center, length23);
//}

BoundingBox Tri::GetBBox()
{
    BoundingBox box = BoundingBox();

    std::vector<double> x;  x.push_back(v1_.x());  x.push_back(v2_.x());  x.push_back(v3_.x());
    std::vector<double> y;  y.push_back(v1_.y());  y.push_back(v2_.y());  y.push_back(v3_.y());
    std::vector<double> z;  z.push_back(v1_.z());  z.push_back(v2_.z());  z.push_back(v3_.z());

    std::sort(x.begin(), x.end());
    std::sort(y.begin(), y.end());
    std::sort(z.begin(), z.end());

    box.bounds[0] = Eigen::Vector3f(x[0], y[0], z[0]);
    box.bounds[1] = Eigen::Vector3f(x[x.size() - 1], y[y.size() - 1], z[z.size() - 1]);

    return box;
}

Triidx::Triidx(int a, int b, int c)
{
    this->v1i_ = a;
    this->v2i_ = b;
    this->v3i_ = c;
}

Triidx::Triidx()
{
    this->v1i_ = this->v2i_ = this->v3i_ = 0;
}

Triidx::~Triidx()
{

}


KDNode::KDNode()
{
    this->left = NULL;
    this->right = NULL;
}

KDNode* KDNode::Build(std::vector<Tri*>& tris, int depth)
{
    //if (depth >= 5)
    //    return NULL;

    //empty node
    KDNode* node = new KDNode();
    node->triangles = tris;
    node->left = NULL;
    node->right = NULL;
    node->box = BoundingBox(); //build bbox here?

    if (tris.size() == 0)
        return NULL;
    if (tris.size() == 1)
    {
        node->box = tris[0]->GetBBox();
        return node;
    }

    if (tris.size() == 2)
        int tag = 3;

    node->box = tris[0]->GetBBox();
    for (int i = 1; i < tris.size(); i++)
    {
        node->box.Expand(tris[i]->GetBBox());
    }

    Eigen::Vector3f middlept = Eigen::Vector3f(0, 0, 0);
    for (int i = 0; i < tris.size(); i++)
    {
        middlept += tris[i]->center_;
    }
    middlept /= tris.size();

    std::vector<Tri*> left_tris, right_tris;

    int axis = depth % 3;
    for (int i = 0; i < tris.size(); i++)
    {
        switch (axis)
        {
        case 0:
            middlept.x() >= tris[i]->center_.x() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
            break;
        case 1:
            middlept.y() >= tris[i]->center_.y() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
            break;
        case 2:
            middlept.z() >= tris[i]->center_.z() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
            break;
        }
    }

    if (left_tris.size() == 0 && right_tris.size() > 0) left_tris = right_tris;
    if (right_tris.size() == 0 && left_tris.size() > 0) right_tris = left_tris;

    if (left_tris.size() > 0 || right_tris.size() > 0)
    {
        node->left = Build(left_tris, depth + 1);
        node->right = Build(right_tris, depth + 1);
    }
    else
    {
        node = NULL;
    }
    return node;
}




bool KDNode::Hit(KDNode *node, Ray ray, Eigen::Vector3f &outnormal, Tri &hittriangle)
{
    if (node->box.RayHitTest(ray))
    {
        bool hittri = false;

        if (node->left->triangles.size() > 0 || node->right->triangles.size() > 0)
        {
            bool hitleft = Hit(node->left, ray, outnormal, hittriangle);
            bool hitright = Hit(node->right, ray, outnormal, hittriangle);
            return hitleft || hitright;
        }
        else
        {
            for (int i = 0; i < node->triangles.size(); i++)
            {
                Eigen::Vector3f hitpoint = node->triangles[i]->RayHitTest(ray, outnormal);
                if (hitpoint.x() != 0 && hitpoint.y() != 0 && hitpoint.z() != 0)
                {
                    hittri = true;
                    hittriangle = *node->triangles[i];
                }
            }

            if (hittri)
            {
                return true;
            }
            return false;
        }
    }
    return false;
}


Intersect* KDNode::Hit(KDNode *node, Ray ray)
{
    Intersect* nearest_inter = NULL;
    Intersect* left_inter = NULL;
    Intersect* right_inter = NULL;

    if (node->triangles.size() == 0)
        return NULL;
    if (node == NULL)
        return NULL;
    if (node->left == NULL&&node->right == NULL)
    {
        Eigen::Vector3f outnormal;
        Eigen::Vector3f hitpoint = node->triangles[0]->RayHitTest(ray, outnormal);
        if (hitpoint.x() != 0 && hitpoint.y() != 0 && hitpoint.z() != 0)
        {
            nearest_inter = new Intersect();
            nearest_inter->hitpoint = hitpoint;
            nearest_inter->outnormal = outnormal;
        }
    }
    else
    {
        if (node->box.RayHitTest(ray))
        {
            if (node->left != NULL)
                left_inter = Hit(node->left, ray);
            if (node->right != NULL)
                right_inter = Hit(node->right, ray);
            if (left_inter != NULL && nearest_inter != NULL)
            {
                if ((left_inter->hitpoint - ray.u_).norm() < (nearest_inter->hitpoint - ray.u_).norm())
                {
                    nearest_inter->hitpoint = left_inter->hitpoint;
                    nearest_inter->outnormal = left_inter->outnormal;
                }
            }
            else
            {
                if (left_inter != NULL) nearest_inter = left_inter;
            }
            if (right_inter != NULL&&nearest_inter != NULL)
            {
                if ((right_inter->hitpoint - ray.u_).norm() < (nearest_inter->hitpoint - ray.u_).norm())
                {
                    nearest_inter->hitpoint = right_inter->hitpoint;
                    nearest_inter->outnormal = right_inter->outnormal;
                }
            }
            else
            {
                if (right_inter != NULL) nearest_inter = right_inter;
            }
        }
    }
    return nearest_inter;
}