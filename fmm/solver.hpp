/*! \file solver.hpp
    \brief Templated class which computes the basic Fast Marching Method (FMM).

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.

    The leafsize of the grid map is ignored since it has to be >=1 and that
    depends on the units employed.

    The type of the heap introduced is very important for the behaviour of the
    algorithm. The following heaps are provided:

    - FMDaryHeap wrap for the Boost D_ary heap (generalization of binary heaps).
    * Set by default if no other heap is specified. The arity has been set to 2
    * (binary heap) since it has been tested to be the more efficient in this algorithm.
    - FMFibHeap wrap for the Boost Fibonacci heap.
    - FMPriorityQueue wrap to the std::PriorityQueue class. This heap implies the implementation
    * of the Simplified FMM (SFMM) method, done automatically because of the FMPriorityQueue::increase implementation.
    *
    @par External documentation:
        FMM:
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013. DOI: <a href="http://dx.doi.org/10.1109/MRA.2013.2248309">10.1109/MRA.2013.2248309></a><br>
           <a href="http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6582543">[PDF]</a>

        SFMM:
          M.W. Jones, J.A. Baerentzen, M. Sramek, 3D Distance Fields: A Survey of Techniques and Applications, IEEE Transactions on Visualization and Computer Graphics, Vol. 12, No. 4, 2006. DOI <a href=http://dx.doi.org/10.1109/TVCG.2006.56">110.1109/TVCG.2006.56</a><br>
          <a href="http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=1634323">[PDF]</a>

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

#ifndef SOLVER_H_
#define SOLVER_H_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>

template <class grid_t>
class Solver {

    public:
        Solver() :name_("GenericSolver"), initialized_(false) {}

        Solver(const std::string& name) : name_(name), initialized_(false) {}

        virtual ~Solver() {}

         /**
          * Sets the input grid in which operations will be performed.
          *
          * @param g input grid map.
          */
        void setEnvironment
        (grid_t * g) {
            grid_ = g;
            if (!grid_->isClear())
                grid_->clear();
        }

        /**
         * Sets the initial points by the indices in the nDGridMap and
         * computes the initialization of the Fast Marching Method calling
         * the init() function.
         *
         * @param init_points contains the indices of the init points.
         *
         * @param goal contains the index of the goal point. If -1 (default) no goal point is set.
         *
         * @see init()
         */
        void setInitialAndGoalPoints
        (const std::vector<int> & init_points, int goal_idx = -1) {
            init_points_ = init_points;
            goal_idx_ = goal_idx;
        }

        void setInitialPoints(const std::vector<int> & init_points)
        {
            setInitialAndGoalPoints(init_points);
        }

        virtual void init
        () {
            if (!sanityChecks())
            {
                console::error("Global sanity checks not successful.");
                exit(1);
            }
            grid_->setClear(false);
            initialized_ = true;
        }

        virtual void compute() = 0;

        const std::string& getName() const
        {
            return name_;
        }

        virtual void clear
        () {
            initialized_ = false;
            init_points_.clear();
            goal_idx_ = -1;
            grid_ = NULL;
        }

        virtual void reset
        () {
            initialized_ = false;
            grid_->clear();
        }

    protected:

        bool sanityChecks
        () {
            if (grid_ == NULL) return false;
            if (!grid_->isClear()) return false;
            if (init_points_.empty()) return false;
            return true;
        }

        grid_t* grid_; /*!< Main container. */

        std::string name_;
        bool initialized_;

        std::vector<int> init_points_;  /*!< Initial points. */
        int goal_idx_;
};

#endif /* SOLVER_H_*/
