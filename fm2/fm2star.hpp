/*! \file fm2star.hpp
    \brief Templated class which computes the Fast Marching Square Star (FM2*).

    This class is the same as FM2 but uses heuristics in the second wave propagation.

    Only FMM is available as underlying planner since using heuristics is not that
    obvious in other planners.

    IMPORTANT NOTE: When running FM2 many times on the same grid it is recommended
    to completely restart the grid (erase and create or resize). See test_fm2.cpp.

    @par External documentation:
        FM2:
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno,
          The Path to Efficiency: Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories,
          IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.

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
    along with this program. If not, see  < http://www.gnu.org/licenses/>. */

#ifndef FM2STAR_HPP_
#define FM2STAR_HPP_

#include  <iostream>
#include  <cmath>
#include  <algorithm>
#include  <numeric>
#include  <fstream>
#include  <array>
#include  <limits>

#include "../fmm/fmdata/fmcell.h"
#include "../fmm/fmm.hpp"
#include "fm2.hpp"
#include "../gradientdescent/gradientdescent.hpp"

// TODO: include suppoert to other solvers (GMM, FIM, UFMM). It requires a better way of setting parameters.
template < class grid_t, class heap_t = FMDaryHeap<FMCell> > class FM2Star : public FM2<grid_t, heap_t> {

    typedef std::vector< std::array<double, grid_t::getNDims()> > path_t;
    typedef FM2<grid_t, heap_t > FM2Base;

    public:
        /** maxDistance sets the velocities map saturation distance in real units (before normalization). */
        FM2Star
        (double maxDistance = -1) : FM2Base("FM2*", maxDistance) { }

        /** maxDistance sets the velocities map saturation distance in real units (before normalization). */
        FM2Star
        (const std::string& name, double maxDistance = -1) : FM2Base(name, maxDistance) { }

        /** Overloaded from FM2. In this case the precomputeDistances() method is called. */
        virtual void setInitialAndGoalPoints
        (const std::vector<unsigned int> & init_points, unsigned int goal_idx) {
            FM2Base::setInitialAndGoalPoints(init_points, goal_idx);
            // Goal and initial points are inverted because the second wave is propagated from the goal
            // to the start, so that the heuristics have to be compared from the initial_point.
            grid_->idx2coord(init_points_[0], heur_coord_);
            solver_->precomputeDistances();
        }

        virtual void computeInternal
        () {
            if (!setup_)
                 setup();

            computeVelocitiesMap();

            // According to the theoretical basis the wave is expanded from the goal point to the initial point.
            std::vector <unsigned int> wave_init;
            wave_init.push_back(goal_idx_);
            unsigned int wave_goal = init_points_[0];

            solver_->setInitialAndGoalPoints(wave_init, wave_goal);
            solver_->setHeuristics(true);
            solver_->compute();
            // Restore the actual grid status.
            grid_->setClean(false);
        }

    protected:
        using FM2Base::grid_;
        using FM2Base::init_points_;
        using FM2Base::goal_idx_;
        using FM2Base::solver_;
        using FM2Base::setup;
        using FM2Base::setup_;
        using FM2Base::computeVelocitiesMap;
        using FM2Base::maxDistance_;
        using FM2Base::fm2_sources_;
        using FM2Base::computePath;
        using FM2Base::reset;
        using FM2Base::clear;
        using FM2Base::time_;
        using FM2Base::end_;

        using FM2Base::start_;


        std::array <unsigned int, grid_t::getNDims()> heur_coord_; /*!< Goal coord, goal of the second wave propagation (actually the initial point of the path). */
};

#endif /* FM2STAR_HPP_*/
