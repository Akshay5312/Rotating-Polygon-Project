#include "ElementStorage.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

/**
 * @brief A class to represent a Voronoi cell
 * 
 * @tparam d The dimension of the space
 * @tparam N The number of points in the space
 */
template <int d, int N>
class VoronoiCell {
    public:
     
     /**
     * @brief Default constructor, all zeros. 
     */
    VoronoiCell(){
      A_ = BLA::Zeros<N,d>();
      b_ = BLA::Ones<N,1>();
    }

    /** 
     * @brief Construct a new Voronoi Cell object. 
     *  This represents all points in the space closer to a designated vertex.
     */
    VoronoiCell(BLA::Matrix<d> center, BLA::Matrix<d,N> points_in_space){ 
        center_ = center;
        
        BLA::Matrix<d,N> diffs = (points_in_space);
        
        for(int i = 0; i < N; i++){
          diffs.Column(i) = diffs.Column(i) - center;
        }

        BLA::Matrix<N> sq_norm_diffs;

        // Sort the points by distance from the center. 
        // This will make runtime computation easier
        for(int i = 0; i < N; i++){
            BLA::Matrix<d> diff = (diffs.Column(i));
            BLA::Matrix<1,d> diff_transpose = BLA::MatrixTranspose<BLA::Matrix<d>>(diff);
            sq_norm_diffs(i) = (diff_transpose * diff)(0);
            
            // insert i into sorted_indices by comparing sq_norm_diffs[i] to sq_norm_diffs[sorted_indices[j]]
            for(int j = 0; j < i; j++){
                if(sq_norm_diffs(i) < sq_norm_diffs(sorted_indices_[j])){ 
                    for(int k = i; k > j; k--){
                        sorted_indices_[k] = sorted_indices_[k-1];
                    }
                    sorted_indices_[j] = i;
                    break;
                }
            }
        }

        for(int i = 0; i < N; i++){
            if(sq_norm_diffs(i) == 0){
                // If the point is the center, this constraint is invalid.
                // Under the case that the center point is repeated in the space, 
                // the cells associated with each point are identical.
                A_ = BLA::Zeros<N,d>();
                continue;
            }

            BLA::Matrix<d> diff = diffs.Column(i);
            
            A_.Row(i) = BLA::MatrixTranspose<BLA::Matrix<d>>(diff) / sq_norm_diffs(i);
        }

        b_ = BLA::Ones<N,1>() * (float)(-0.5);
    };

    /**
     * @brief Check if a point is in the cell. If it is, it is closer to the desired vertex than any other vertex.
     * 
     * @param x The point to check
     * @return true if the point is in the cell  
     */ 
    bool contains(BLA::Matrix<d> x){                                                                                                                                 
        // Faster to iterate over the rows of A and fail early
        // as we have sorted the constraints by the chance they are violated
        for(int i = 0; i < N; i++){
            int j = sorted_indices_[i];
            if((A_.Row(j) * (x - center_))(0) > b_(j)){
                return false;
            }
        }
        return true;
    }

    private:

    int sorted_indices_[N];

    BLA::Matrix<d> center_;
    
    BLA::Matrix<N,d> A_;
    BLA::Matrix<N,1> b_;
};


/**
 * @brief A class to represent a Voronoi diagram
 * 
 * @tparam d The dimension of the space
 * @tparam N The number of points in the space
 */
template <int d, int N>
class VoronoiDiagram {
    public:
    /**
    * @brief Default constructor, cells are unimplemented.
    */
    VoronoiDiagram(){}

    VoronoiDiagram(BLA::Matrix<d,N> points_in_space){
        for(int i = 0; i < N; i++){
            // there is a lot of repeated computation here... whoops
            cells_[i] = VoronoiCell<d,N>(points_in_space.Column(i), points_in_space);
        }
    }

    /**
     * @brief Get the index of the cell that contains a point
     * 
     * @param x The point to check
     * @return int The index of the cell that contains the point.
     */
    int get_cell_index(BLA::Matrix<d> x){
        //check each cell to see if it contains the point
        for(int i = 0; i < N; i++){
            if(cells_[i].contains(x)){
                return i;
            }
        }

        // If the point is not in any cell, return -1. 
        // This should never happen. 
        return -1;
    }

    private:
    VoronoiCell<d,N> cells_[N]; // N voronoi cells
};