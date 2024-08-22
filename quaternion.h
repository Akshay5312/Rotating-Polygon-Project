#pragma once

#include <BasicLinearAlgebra.h>

    //unit vectors
    BLA::Matrix<3> i = {1,0,0};
    BLA::Matrix<3> j = {0,1,0};
    BLA::Matrix<3> k = {0,0,1};


    //cross product
    BLA::Matrix<3> cross(BLA::Matrix<3> V1, BLA::Matrix<3> V2){
        BLA::Matrix<3> retVal;
        retVal(0) = V1(1) * V2(2) - V1(2) * V2(1);
        retVal(1) = -V1(0) * V2(2) + V1(2) * V2(0);
        retVal(2) = V1(0) * V2(1) - V1(1) * V2(0);
        return retVal;
    }

    //dot product of 2 spatial vectors
    float dot(BLA::Matrix<3> V1, BLA::Matrix<3> V2){
        // BLA transpose is giving an error. Will inspect.
        return V1(0) * V2(0) + V1(1) * V2(1) + V1(2) * V2(2);
    }

    class QuaternionO : public BLA::Matrix<4>{
        private:
            const float& q0() const { return (*this)(0); }
            const BLA::Matrix<3>& axisVect() const { return (*this).Submatrix<3, 1>(1, 0); } 
        public:

            QuaternionO() : BLA::Matrix<4>(){}
            
            QuaternionO(float w, float i, float j, float k) : BLA::Matrix<4>({w,i,j,k}){}

            /**
             * @brief Construct a new Quaternion from an axis angle representation
             * 
             * @param axis_angle 
             */
            QuaternionO(BLA::Matrix<3> axis_angle) : BLA::Matrix<4>(){
                float angle = BLA::Norm(axis_angle);
                BLA::Matrix<3> axis = axis_angle * (1/angle);
                float half_angle = angle/2;
                BLA::Matrix<3> half_axis = axis * (float)sin(half_angle);
                (*this) = {cos(half_angle), half_axis(0), half_axis(1), half_axis(2)};
            }

            ~QuaternionO(){}

            //i,j,k
            // QuaternionO(char unitDirection, float angle);
            /**
             * @brief multiplication operator, transforms a quaternion through this
             * 
             * @param QO 
             * @return QuaternionO 
             */
            QuaternionO operator * (const QuaternionO& QO) const {
                float retq0 = q0() * QO.q0() - dot(axisVect(), QO.axisVect());

                BLA::Matrix<3> retVec = cross(axisVect(), QO.axisVect()) + QO.axisVect() * q0() + axisVect() * QO.q0();

                return {retq0, retVec(0), retVec(1), retVec(2)};
            }

            /**
             * @brief Get the Imaginary Part of the quaternion
             * 
             * @return BLA::Matrix<3> 
             */
            const BLA::Matrix<3>& getImaginaryPart(){return axisVect();}

            /**
             * @brief Get the Inverse
             * 
             * @return QuaternionO 
             */
            QuaternionO getInverse() const {
                BLA::Matrix<3> invVect = axisVect()*(float)(-1.0);
                return QuaternionO(q0(), invVect(0), invVect(1), invVect(2));
            }

            /**
             * @brief transform a spatial vector
             * 
             * @param V3 
             * @return BLA::Matrix<3> 
             */
            BLA::Matrix<3> operator * (const BLA::Matrix<3>& V3){
                return transform(V3);
            }

            /**
             * @brief transform a spatial vector
             * 
             * @param VectorInI 
             * @return BLA::Matrix<3> 
             */
            BLA::Matrix<3> transform(BLA::Matrix<3> VectorInI) const{
                QuaternionO midVal(0,VectorInI(0),VectorInI(1),VectorInI(2));
                return ((*this) * midVal * getInverse()).getImaginaryPart();
            }

            /**
             * @brief Get the rotation axis
             * 
             * @return spatialP 
             */
            BLA::Matrix<3> getAxis() const {
                if(BLA::Norm(axisVect()) == 0){return {0,0,0};}

                return axisVect() * (1/BLA::Norm(axisVect())); 
            }

            /**
             * @brief Get the angle of displacement
             * 
             * @return float 
             */
            float getAngle() const {return 2*acos(q0());}

            /**
             * @brief Get the Axis Angle representation
             * 
             * @return BLA::Matrix<3> 
             */
            BLA::Matrix<3> getAxisAngle() const {return getAxis() * getAngle();}


            /**
             * @brief Get the rotation matrix
             * 
             * @return BLA::Matrix<3, 3> 
             */
            BLA::Matrix<3,3> getRotationMatrix() const {
                
                BLA::Matrix<3,3> rot_mat;
                rot_mat.Column(0) = transform(i);
                rot_mat.Column(1) = transform(j);
                rot_mat.Column(2) = transform(k);
                
                return rot_mat;
            }
    };