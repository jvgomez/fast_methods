/*! \file fm2.hpp
    \brief Templated class which computes the Fast Marching Square (FM2).

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.

    The leafsize of the grid map is ignored since it has to be >=1 and that
    depends on the units employed.

    The type of the heap are the same as for FMM class.
    *
    @par External documentation:
        FM2:
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.

    Copyright (C) 2014 Javier V. Gomez and Jose Pardeiro
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
    along with this program. If not, see <http://www.gnu.org/licenses/>.*/

#ifndef FM2_HPP_
#define FM2_HPP_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>
#include <limits>

#include "../fmm/fmm.hpp"
#include "../gradientdescent/gradientdescent.hpp"

template < class grid_t, class heap_t = FMDaryHeap<FMCell>  >  class FM2 : public Solver<grid_t> {

    public:
        typedef std::vector< std::array<double, grid_t::getNDims()> > path_t;

        FM2
        (double maxDistance = -1) : Solver<grid_t>("FM2"), maxDistance_(maxDistance) {
            solver_ = new FMM<grid_t,heap_t>();
        }

        FM2
        (const std::string& name, double maxDistance = -1) : Solver<grid_t>(name), maxDistance_(maxDistance) {
            solver_ = new FMM<grid_t,heap_t>();
        }

        virtual ~FM2 () { clear(); }

        virtual void setEnvironment
        (grid_t * g) {
            Solver<grid_t>::setEnvironment(g);
            grid_->getOccupiedCells(fmm2_sources_);
        }

        /**
         * Main Fast Marching Square Function with velocity saturation. It requires to call first the setInitialAndGoalPoints() function.
         *
         * @see setInitialAndGoalPoints()
         */
        virtual void compute
        () {
            if (!setup_)
                 setup();

            computeVelocitiesMap();

            // According to the theoretical basis the wave is expanded from the goal point to the initial point.
            std::vector <unsigned int> wave_init;
            wave_init.push_back(goal_idx_);
            unsigned int wave_goal = init_points_[0];

            solver_->reset(false);
            solver_->setInitialAndGoalPoints(wave_init, wave_goal);
            solver_->compute();
        }

        /**
         * Computes the path from the previous given goal index to the minimum
         * of the times of arrival map. According to the theoretical basis the 
         * wave is expanded from the goal point to the initial point. For these 
         * reasons the gradient must to be applied from the initial point.
         *
         * No checks are done (points in the borders, points in obstacles...).
         *
         * The included scripts will parse the saved path.
         *
         * @param path the resulting path (output).
         *
         * @param velocity the resulting path (output).
         */
        virtual void computePath
        (path_t * p, std::vector <double> * path_velocity) {
            path_t* path_ = p;
            GradientDescent< nDGridMap<FMCell, grid_t::getNDims()> > grad;
            grad.apply(*grid_,init_points_[0],*path_, *path_velocity);
        }

        /**
         * Computes the path from the given goal index to the minimum
         * of the times of arrival map.
         *
         * No checks are done (points in the borders, points in obstacles...).
         *
         * The included scripts will parse the saved path.
         *
         * @param path the resulting path (output).
         *
         * @param velocity the resulting path (output).
         *
         * @param goal_idx index of the goal point, where gradient descent will start.
         */
        virtual void computePath
        (path_t * p, std::vector <double> * path_velocity, unsigned int goal_idx) {
            path_t* path_ = p;
            GradientDescent< nDGridMap<FMCell, grid_t::getNDims()> > grad;
            grad.apply(*grid_,goal_idx,*path_, *path_velocity);
        }

        virtual void clear
        () {
            Solver<grid_t>::clear();
            fmm2_sources_.clear();
            maxDistance_ = -1;
            delete solver_;
        }

        virtual void reset
        () {
            Solver<grid_t>::reset();
            maxDistance_ = -1;
            solver_->reset();
        }

    protected:

        // Computes the velocities map of the FM2 algorithm.If  maxDistance_ != -1 then the map is saturated
        // to the set value.
        void computeVelocitiesMap
        () {
            solver_->setEnvironment(grid_);
            solver_->setInitialPoints(fmm2_sources_);
            solver_->compute();

            // Rescaling and saturating to relative velocities: [0,1]
            double maxValue = grid_->getMaxValue();
            double maxVelocity = 0;

            if (maxDistance_ != -1)
                maxVelocity = maxDistance_ / grid_->getLeafSize();

            for (unsigned int i = 0; i < grid_->size(); i++) {
                double vel = grid_->getCell(i).getValue() / maxValue;

                if (maxDistance_ != -1)
                    if (vel < maxVelocity)
                        grid_->getCell(i).setVelocity(vel / maxVelocity);
                    else
                        grid_->getCell(i).setVelocity(1);
                else
                    grid_->getCell(i).setVelocity(vel);

                // Restarting grid values for second wave expasion.
                grid_->getCell(i).setValue(std::numeric_limits<double>::infinity());
                grid_->getCell(i).setState(FMState::OPEN);
                grid_->setClean(true);
            }
        }

        using Solver<grid_t>::grid_;
        using Solver<grid_t>::init_points_;
        using Solver<grid_t>::goal_idx_;
        using Solver<grid_t>::setup;
        using Solver<grid_t>::setup_;

        std::vector<unsigned int> fmm2_sources_;  /*!< Wave propagation sources for the Fast Marching Square. */

        Solver<grid_t>* solver_;

        double maxDistance_; /*!< Distance value to saturate the first potential. */
};

#endif /* FM2_H_*/
