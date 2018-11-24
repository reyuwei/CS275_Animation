//
// Created by stephan-lb on 23/03/2017.
//

#ifndef TINYOBJVIEWER_MESH_H
#define TINYOBJVIEWER_MESH_H

#include <string>
#include <Eigen/Sparse>
#include <vector>
#include "GLTexture.h"
#include "tiny_obj_loader.h"
#include "MassSpring.h"

typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

class Mesh {

public:

    Mesh(const std::string &filename);

    ~Mesh();

    bool load_mesh(const std::string &filename);

    bool load_label(const std::string &filename);

    //void export_mesh(const std::string &filename);

    unsigned int get_number_of_face();

    const Eigen::Vector3f get_mesh_center();

    const Eigen::MatrixXf *get_points();

    const MatrixXu *get_indices();

    const Eigen::MatrixXf *get_normals();

    const Eigen::MatrixXf *get_uvs();

    const Eigen::MatrixXf *get_hairpos();
    const MatrixXu *get_hairindices();
    int get_number_of_hair();

    float get_dist_max();

    GLTexture* get_texture();

    void generateHair();

private:

    std::vector<tinyobj::shape_t> m_shapes;
    std::vector<tinyobj::material_t> m_materials;

    GLTexture m_texture;

    size_t m_num_vertices;
    size_t m_num_faces;
    size_t m_num_uvs;


    Eigen::Vector3f m_bmin;
    Eigen::Vector3f m_bmax;
    Eigen::Vector3f m_mesh_center;
    float m_dist_max = 0.0f;
    Eigen::MatrixXf m_points;
    MatrixXu m_indices;
    Eigen::MatrixXf m_normals;
    Eigen::MatrixXf m_uvs;

    MatrixXu m_hairindices;
    Eigen::MatrixXf m_hair;
    std::vector<float> ishair;

    Hair hairs;
    int hair_num;


};


#endif //TINYOBJVIEWER_MESH_H