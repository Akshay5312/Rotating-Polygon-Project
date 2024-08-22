#include "ElementStorage.h"
#pragma once

#include "VoronoiDiagram.h"
#include "quaternion.h"


/**
 * @brief The vertices of a polygon in 3 dimensions. Represented as a 3xN matrix.
 * 
 * @tparam N The number of vertices
 */
template <int N>
using PolygonVerts = BLA::Matrix<3, N>;

/**
 * @brief Get a pyramid with a triangular base
 * 
 * @return PolygonVerts<4> 
 */
PolygonVerts<4> get_pyramid(){
    PolygonVerts<4> pyramid;

    pyramid.Column(0) = BLA::Matrix<3>({-0.5, -0.5,  -0.5});
    pyramid.Column(1) = BLA::Matrix<3>({ 0.5, -0.5,  -0.5});
    pyramid.Column(2) = BLA::Matrix<3>({   0, 0.75,  -0.5});
    pyramid.Column(3) = BLA::Matrix<3>({   0,    0,  0.75});

    return pyramid;
};

/**
 * @brief Get a cube
 * 
 * @return PolygonVerts<4> 
 */
PolygonVerts<4> get_cube(){
    PolygonVerts<4> cube;

    cube.Column(0) = BLA::Matrix<3>({0.5, 0.5, 0.5});
    cube.Column(1) = BLA::Matrix<3>({0.5,-0.5, 0.5});
    cube.Column(2) = BLA::Matrix<3>({0.5, 0.5,-0.5});
    cube.Column(3) = BLA::Matrix<3>({0.5,-0.5,-0.5});

    return cube;
};

/**
 * @brief Get a triangle (2d)
 * 
 * @return PolygonVerts<3> 
 */
PolygonVerts<3> get_triangle(){
    PolygonVerts<3> triangle;

    triangle.Column(0) = BLA::Matrix<3>({0.5, -0.5,     0});
    triangle.Column(1) = BLA::Matrix<3>({-0.5,-0.5,  0.75});
    triangle.Column(2) = BLA::Matrix<3>({   0,   0,     0});

    return triangle;
};


/**
 * @brief A class to represent a rotating polygon. 
 *      This contains vertices, a voronoi diagram, an orientation.
 * 
 *  Dynamics of rotation need to be implemented.
 * 
 * @tparam N The number of vertices in the polygon
 */
template <int N>
class RotatingPolygon{
    public:
    RotatingPolygon(PolygonVerts<N> verts) : verts_(verts), rot_verts_t_(verts) { 
        // voronoi_ = VoronoiDiagram<3,N>(verts_);
    }

    /**
     * @brief Step the simulation forward by dt. 
     *  This effectively rotates the polygon.
     * 
     * @param dt The time step
     */
    void step(float dt){
        // apply dynamics
        QuaternionO q = q_;
        apply_dynamics(dt, q, dq_);

        // apply rotation
        apply_rotation(q);
    }

    /**
     * @brief get an updated state of the polygon (q, dq). This is _const_. do_dynamics must be implemented to find ddq.
     * 
     * @param dt The time step
     * @param q The orientation quaternion
     * @param dq The angular velocity
     */
    virtual void apply_dynamics(float dt, QuaternionO& q, BLA::Matrix<3,1>& dq) const {    
        BLA::Matrix<3> ddq_stepped;
        do_dynamics(q, dq, ddq_stepped);

        // apply dq to q. dq is an axis_angle rotation. 
        QuaternionO dq_quat = QuaternionO(dq * dt);
        q = dq_quat * q;
        dq = dq + ddq_stepped * dt;
    }

    /**
     * @brief get ddq given the state (q and dq). This is _const_.
     */
    virtual void do_dynamics(const QuaternionO& q, 
        const BLA::Matrix<3>& dq, BLA::Matrix<3>& ddq) const { 
        ddq = {0,0,0};
    }

    /**
     * @brief Apply a rotation to the polygon.
     * 
     * @param q The orientation quaternion
     */
    void apply_rotation(const QuaternionO& q){
        q_ = q;

        // update rotation matrix
        rot_matrix_ = q_.getRotationMatrix();

        // rotate vertices
        rot_verts_t_ = rot_matrix_ * verts_;
    }
        


    /**
     * @brief Get   the vertex at index i
     * 
     * @param i The index
     * @return BLA::Matrix<3> The vertex
     */
    BLA::Matrix<3> get_vert(int i){
        return rot_verts_t_.Column(i);
    }

    /**
     * @brief Get the index of the closest vertex to a point
     * 
     * @param point The point
     * @return int The index of the closest vertex
     */
    int vert_closest_to(BLA::Matrix<3,1> point){
        // We apply an inverse rotation to the point to get it in the frame of the voronoi diagram object.
        // return voronoi_.get_cell_index(BLA::MatrixTranspose<BLA::Matrix<3,3>>(rot_matrix_) * point);
    }

    /**
     * @brief Get the closest vertex to a point
     * 
     * @param point The point
     * @return BLA::Matrix<3> The closest vertex
     */
    BLA::Matrix<3> get_closest_vert(BLA::Matrix<3> point){
        return get_vert(vert_closest_to(point));
    }

    private:

    // VoronoiDiagram<3,N> voronoi_;

    PolygonVerts<N> verts_;
    PolygonVerts<N> rot_verts_t_;

    BLA::Matrix<3, 3, float> rot_matrix_ = BLA::Eye<3,3>();

    QuaternionO q_; // orientation quaternion
    BLA::Matrix<3> dq_ = {0,0,0}; // angular velocity
};