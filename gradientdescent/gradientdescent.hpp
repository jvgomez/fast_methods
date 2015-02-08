#ifndef GRADIENTDESCENT_H_
#define GRADIENTDESCENT_H_

#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <numeric>

#include "../ndgridmap/ndgridmap.hpp"
#include "../fmm/fmdata/fmcell.h"

// TODO: check if points fall in obstacles, points in the borders, etc.

template <typename T>
T sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <class grid_t> class GradientDescent {

    static constexpr size_t ndims_ = grid_t::getNDims();
    typedef typename std::array<unsigned int, ndims_> Coord;
    typedef typename std::array<double, ndims_> Point;
    typedef typename std::vector <Point> Path;

    public:
       /** Computes the path over grid from idx to a minimum and extracts the velocity in every point.

           Simple gradient approximation is used: dimension 0: gx = f((x-1,y)+f(x+1,y))/2
           dimension 1: gy = f((x,y-1)+f(x,y+1))/2
           and so on.

           No checks are done (points in the borders, points in obstacles...).
           IMPORTANT NOTE: both minimum and initial index cannot be in the
           border of the map. This situation is not checked. We recommend to set a 1 pixel
           black border around the map image. */
      static void apply
      (grid_t & grid, unsigned int &  idx, Path & path, std::vector <double> & path_velocity, double step = 1) {

          Coord current_coord;
          Point current_point;
          Coord dimsize = grid.getDimSizes();

          std::array<unsigned int, ndims_-1> d_; //  Same as nDGridMap class auxiliar array d_.
          d_[0] = dimsize[0];
          for (size_t i = 1; i < ndims_; ++i)
              d_[i] = dimsize[i]*d_[i-1];

          grid.idx2coord(idx, current_coord);
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to int.
          path.push_back(current_point);
          path_velocity.push_back(grid[idx].getVelocity());

          std::array<double, ndims_> grads;

          while(grid[idx].getArrivalTime() != 0) {
              // Every iteration the gradient is computed for all dimensions. If is infinite, we convert it to 1 (keeping the sign).
              // The static_cast are necessary because the conversion between coordinate (we check the value in coordinates) and points
              // (the path is composed by continuous points).

              // First dimension done apart.
              grads[0] = - grid[idx-1].getValue()/2 + grid[idx+1].getValue()/2;
              if (isinf(grads[0]))
                  grads[0] = sgn<double>(grads[0]);
              double max_grad = std::abs(grads[0]);

              for (size_t i = 1; i < ndims_; ++i) {
                  grads[i] = - grid[idx-d_[i-1]].getValue()/2 + grid[idx+d_[i-1]].getValue()/2;
                  if (isinf(grads[i]))
                      grads[i] = sgn<double>(grads[i]);
                  if (std::abs(max_grad) < std::abs(grads[i]))
                      max_grad = grads[i];
              }

              // Updating points
              for (size_t i = 0; i < ndims_; ++i) {
                  // Moving the point in dim i.
                  current_point[i] = current_point[i] - step*grads[i]/std::abs(max_grad);
                  current_coord[i] = current_point[i];
              }
              path.push_back(current_point);
              path_velocity.push_back(grid[idx].getVelocity());
              grid.coord2idx(current_coord,idx);
          }
          //Adding exactly the last point at the end.
          grid.idx2coord(idx, current_coord);
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to double.
          path.push_back(current_point);
          path_velocity.push_back(grid[idx].getVelocity());
      }
};

#endif /* GRADIENTDESCENT_H_*/
