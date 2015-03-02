/*! \class GradientDescent
    \brief Implements gradient descent to be used together with nDGridMap
    and FMCell or derived types.
   
    Copyright (C) 2014 Javier V. Gomez
    www.javiervgomez.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

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

/// \todo Check if points fall in obstacles, points in the borders, etc.

/** \brief User-implemented and templated sign function. */
template <typename T>
T sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <class grid_t> class GradientDescent {

    /** \brief Shorthand for number of dimensions. */
    static constexpr size_t ndims_ = grid_t::getNDims();

    /** \brief Shorthand for coordinates. */
    typedef typename std::array<unsigned int, ndims_> Coord;

    /** \brief Shorhand for real points. */
    typedef typename std::array<double, ndims_> Point;

    /** \brief Shorthand for path type of real points. */
    typedef typename std::vector <Point> Path;

    public:
       /** \brief Computes the path over grid from idx to a minimum and extracts the velocity in every point.

           Simple gradient approximation is used: dimension 0: gx = f((x-1,y)+f(x+1,y))/2
           dimension 1: gy = f((x,y-1)+f(x,y+1))/2
           and so on.

           No checks are done (points in the borders, points in obstacles...).
           IMPORTANT NOTE: both minimum and initial index cannot be in the
           border of the map. This situation is not checked. We recommend to set a 1 pixel
           black border around the map image. */
      static void apply
      (grid_t & grid, unsigned int & idx, Path & path, std::vector <double> & path_velocity, double step = 1) {

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
                  current_coord[i] = int(current_point[i]+0.5);
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
