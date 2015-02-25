/*! \file solver.hpp
    \brief Templated class which computes the basic Fast Marching Method (FMM).

    It uses as a main container the nDGridMap class. The nDGridMap template paramenter
    has to be an FMCell or something inherited from it.

    The type of the heap introduced is very important for the behaviour of the
    algorithm. The following heaps are provided:

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
    along with this program. If not, see <http://www.gnu.org/licenses/>. */

#ifndef SOLVER_H_
#define SOLVER_H_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>
#include <chrono>

template <class grid_t>
class Solver {

    public:
        Solver() :name_("GenericSolver"), setup_(false) {}

        Solver(const std::string& name) : name_(name), setup_(false) {}

        virtual ~Solver() { clear(); }

         /** Sets and cleans the grid in which operations will be performed. */
        virtual void setEnvironment
        (grid_t * g) {
            grid_ = g;
            grid_->clean();
        }

        /** Sets the initial and goal points by the indices of the grid. */
        virtual void setInitialAndGoalPoints
        (const std::vector<unsigned int> & init_points, unsigned int goal_idx) {
            init_points_ = init_points;
            goal_idx_ = goal_idx;
        }

        /** Sets the initial points by the indices of the grid. */
        virtual void setInitialPoints
        (const std::vector<unsigned int> & init_points)
        {
            setInitialAndGoalPoints(init_points, -1);
        }

        /** Sets the initial and goal points by the coordinates of the grid. */
        virtual void setInitialAndGoalPoints
        (const std::array<unsigned int, grid_t::getNDims()> & init_coord, const std::array<unsigned int, grid_t::getNDims()> & goal_coord) {
            std::vector<unsigned int> init_points;
            unsigned int idx;
            grid_->coord2idx(init_coord, idx);
            init_points.push_back(idx);
            grid_->coord2idx(goal_coord, idx);
            setInitialAndGoalPoints(init_points, idx);
        }

        /** Sets the initial point by the coordinates of the grid. */
        virtual void setInitialPoints
        (const std::array<unsigned int, grid_t::getNDims()> & init_coord)
        {
            std::vector<unsigned int> init_points;
            unsigned int idx;
            grid_->coord2idx(init_coord, idx);
            init_points.push_back(idx);
            setInitialAndGoalPoints(init_points, -1);
        }

        /** Checks that the solver is ready to run. Sets the grid unclean. */
        virtual void setup
        () {
            const int err = sanityChecks();
            if (err)
            {
                console::error("Global sanity checks not successful: ");
                switch(err) {
                    case 1:
                        console::error("No grid map set.");
                        break;
                    case 2:
                        console::error("Grid map set is not clean.");
                        break;
                    case 3:
                        console::error("Initial points were not set.");
                        break;
                    case 4:
                        console::error("A init point is in a obstacle.");
                        break;
                    case 5:
                        console::error("A goal point is in a obstacle.");
                        break;
                    case 6:
                        console::error("A start is equal to a goal point.");
                        break;
                    default:
                        console::error("Uknown error.");
                }
                exit(1);
            }
            grid_->setClean(false);
            setup_ = true;
        }

        /** Computes the distances map. Will call setup() if not done already. */
        void compute() {
            start_ = std::chrono::system_clock::now();
            computeInternal();
            end_ = std::chrono::system_clock::now();
            time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_-start_).count();
        }

        /** Actual compute function to be implemented in each solver. */
        virtual void computeInternal() = 0;

        /** @return name of the solver. */
        const std::string& getName() const
        {
            return name_;
        }

        /** Clears the solver, it is not recommended to be used out of the destructor. */
        virtual void clear
        () {
            init_points_.clear();
            goal_idx_ = -1;
            setup_ = false;
        }

        /** Clears temporal data, so it is ready to run again. */
        virtual void reset
        () {
            setup_ = false;
            grid_->clean();
        }

        /** Returns a pointer to the grid used. */
        grid_t* getGrid() const
        {
            return grid_;
        }

        virtual double getTime
        () const {
            return time_;
        }

    protected:
        /** Performs different check before a solver can proceed. */
        int sanityChecks
        () {
            if (grid_ == NULL) return 1;
            if (!grid_->isClean()) return 2;
            if (init_points_.empty()) return 3;

            // When more that 1 initial point is given, this check is ommitted
            // since it could be FM2-like velocities map computation.
            if (init_points_.size() == 1 &&
                grid_->getCell(init_points_[0]).isOccupied()) return 4;

            if(int(goal_idx_) != -1 && grid_->getCell(goal_idx_).isOccupied()) return 5;

            for (int ip : init_points_)
                if(int(goal_idx_) == ip) return 6;

            return 0;
        }

        grid_t* grid_; /*!< Grid container. */

        std::string name_; /*!< Solver name. */
        bool setup_; /*!< Setup status. */

        std::vector<unsigned int> init_points_;  /*!< Initial index. */
        unsigned int goal_idx_; /*!< Goal index. */

        std::chrono::time_point<std::chrono::system_clock> start_;
        std::chrono::time_point<std::chrono::system_clock> end_;
        double time_;
};

#endif /* SOLVER_H_*/
