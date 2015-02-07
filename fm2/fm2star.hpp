/*! \file fm2star.hpp
    \brief Templated class which computes the Fast Marching Square Star (FM2*).

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMStarCell or something inherited from it.

    Only FMM is available as underlying planner since using heuristics is not that
    obvious in other planners.

    @par External documentation:
        FM2 and old FM2*:
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno, Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories.

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
    along with this program. If not, see  < http://www.gnu.org/licenses/>.
*/

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

template < class grid_t, class heap_t = FMDaryHeap<FMCell> > class FM2Star : public FM2<grid_t, FMM<grid_t, heap_t> > {

    typedef std::vector< std::array<double, grid_t::getNDims()> > path_t;
    typedef FM2<grid_t, FMM<grid_t, heap_t> > FM2Base;

    public:
        FM2Star
        (double maxDistance = -1) : FM2Base("FM2*", maxDistance) { }

        FM2Star
        (const std::string& name, double maxDistance = -1) : FM2Base(name, maxDistance) { }

        virtual void setInitialAndGoalPoints
        (const std::vector<unsigned int> & init_points, unsigned int goal_idx) {
            FM2Base::setInitialAndGoalPoints(init_points, goal_idx);
            // Goal and initial points are inverted because the second wave is propagated from the goal
            // to the start, so that the heuristics have to be compared from the initial_point.
            grid_->idx2coord(init_points_[0], heur_coord_);
            solver_->setEnvironment(grid_);
            solver_->precomputeDistances();
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

            solver_->setInitialAndGoalPoints(wave_init, wave_goal);
            solver_->setHeuristics(true);
            solver_->compute();
        }

        /**
         * Sets the initial points by the indices in the nDGridMap,
         * computes the initialization of the Fast Marching Method calling
         * the init() function and the euclidean distance between every pixel
         * calling the precomputeDistances() function.
         *
         * @param init_points contains the indices of the init points.
         *
         * @see init()
         *
         * @see precomputeDistances()
         */
        /*virtual void setInitialPoints
        (const std::vector <int> & init_points) {
            init_points_ = init_points;
            for (const int &i: init_points) {
                grid_->getCell(i).setArrivalTime(0);
                grid_->getCell(i).setState(FMState::FROZEN);
            }

            if (init_points.size() > 1)
                init(false, false);
            else
            {
                precomputeDistances();
                init(true, true);
            }
        }*/

        /**
         * Extract the euclidean distance calculated from precomputeDistances
         * function distance between two positions. It obtain the difference vector
         * coordinates in every dimension between a given and a goal point and convert
         * it into an index of the distances array.
         *
         * @param idx index of the cell to be evaluated.
         *
         * @see precomputeDistances()
         */
        /*virtual double getPrecomputedDistance
        (const int idx) {
            std::array < int,grid_->getNDims()> position, distance;

            grid_->idx2coord(idx, position);

            for (int i = 0; i < grid_->getNDims(); i++)
               distance[i] = std::abs(position[i] - goal[i]);

            int idx_dist;
            grid_->coord2idx(distance, idx_dist);

            return distances[idx_dist];
        }*/

         /**
         * Internal function although it is set to public so it can be accessed if desired.
         *
         * Computes the Fast Marching Method initialization from the initial points given. Programmed following the paper:
            A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer,
            More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.
         *
         * @param stop selects if the wave has to stop when it arrives to the goal point
         *
         * @param star selects of mode to expand the wave.
         *
         * @see setInitialPoints()
         */
        /*virtual void init
        (const bool stop = true, const bool star = true) {
            // TODO: neighbors computed twice for every cell. We can save time here.
            // TODO: check if the previous steps have been done (loading grid map and setting initial points.)
            int j = 0;
            int n_neighs = 0;
            for (int &i: init_points_) { // For each initial point
                n_neighs = grid_->getNeighbors(i, neighbors);
                for (int s = 0; s  <  n_neighs; ++s){  // For each neighbor
                    j = neighbors[s];
                    if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied() || grid_->getCell(j).getVelocity() == 0) // If Frozen or obstacle
                        continue;
                    else {
                        double new_arrival_time = 0;

                        new_arrival_time = solveEikonal(j, star);

                        if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time  <  grid_->getCell(j).getArrivalTime()) {
                                grid_->getCell(j).setArrivalTime(new_arrival_time);
                                narrow_band_.increase( &(grid_->getCell(j)) );
                            }
                        }
                        else {
                            grid_->getCell(j).setState(FMState::NARROW);
                            grid_->getCell(j).setArrivalTime(new_arrival_time);
                            narrow_band_.push( &(grid_->getCell(j)) );
                        } // neighbors open.
                    } // neighbors not frozen.
                } // For each neighbor.
            } // For each initial point.
        } // init()*/

        /**
        * Solves the Eikonal equation for a given cell. This function is generalized
        * to any number of dimensions.
        *
        * @param idx index of the cell to be evaluated.
        *
        * @param star selects of mode to resolve Eikonal equation depending of the heuristic
        *
        * @return the distance (or time of arrival) value.
        */
       /*virtual double solveEikonal
       (const int & idx) {
           // TODO: Here neighbors are computed and then in the computeFM. There should be a way to avoid computing
           // neighbors twice.
           int a = grid_t::getNDims(); // a parameter of the Eikonal equation.

           double updatedT;
           sumT = 0;
           sumTT = 0;

           for (int dim = 0; dim  <  grid_t::getNDims(); ++dim) {
               double minTInDim = grid_->getMinValueInDim(idx, dim);
               if (!isinf(minTInDim)) {
                   Tvalues[dim] = minTInDim;
                   sumT +=  Tvalues[dim];
                   TTvalues[dim] = Tvalues[dim]*Tvalues[dim];
                   sumTT +=  TTvalues[dim];
               }
               else {
                   Tvalues[dim] = 0;
                   TTvalues[dim] = 0;
                   a -= 1 ;
               }
           }

           double b = -2*sumT;
           double c = sumTT - grid_->getLeafSize() * grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()); // leafsize not taken into account here.
           double quad_term = b*b - 4*a*c;
           if (quad_term  <  0) {
               double minT = *(std::min_element(Tvalues.begin(), Tvalues.end()));
               updatedT = grid_->getLeafSize() * grid_->getLeafSize() / (grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()) + minT; // leafsize not taken into account here.
           }
           else
               updatedT = (-b + sqrt(quad_term))/(2*a);

           // Apply the FM2* heuristic
           updatedT +=  distances_[idx]/grid_->getCell(idx).getVelocity();

           return updatedT;
       }*/

        /**
         * Main Fast Marching Function. It requires to call first the setInitialPoints() function.
         *
         * @param stop selects if the wave has to stop when it arrives to the goal point
         *
         * @param star selects if star heuristic must be applied
         *
         * @see setInitialPoints()
         */
        /*virtual void computeSecondPotential
        () {
            // TODO: check if the previous steps have been done (initialization).
            int j =  0;
            int n_neighs = 0;
            bool stopWavePropagation = 0;

            while (narrow_band_.size() > 0 && stopWavePropagation == 0) {
                int idxMin = narrow_band_.popMinIdx();
                n_neighs = grid_->getNeighbors(idxMin, neighbors);
                grid_->getCell(idxMin).setState(FMState::FROZEN);

                for (int s = 0; s  <  n_neighs; ++s) {
                    j = neighbors[s];
                    if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied()) // If Frozen or obstacle
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j, star);

                        if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time  <  grid_->getCell(j).getArrivalTime()) {
                                grid_->getCell(j).setArrivalTime(new_arrival_time);

                                narrow_band_.increase( &(grid_->getCell(j)) );
                            }
                        }
                        else {
                            grid_->getCell(j).setState(FMState::NARROW);
                            grid_->getCell(j).setArrivalTime(new_arrival_time);

                            narrow_band_.push( &(grid_->getCell(j)) );
                        } // neighbors open.
                    } // neighbors not frozen.
                    if (idxMin == initial_point_[0] && stop)
                        stopWavePropagation = 1;
                } // For each neighbor.
            } // while narrow band not empty
        }*/

        /**
         * Computes the path from the given index to a minimum (the one
         * gradient descent choses) and returns the velocity. According to 
         * the theoretical basis the wave is expanded from the goal point 
         * to the initial point. For these reasons the gradient must to be 
         * applied from the initial point.
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
            constexpr int ndims = grid_t::getNDims();

            GradientDescent < nDGridMap < FMCell, ndims > > grad;
            grad.apply(*grid_, init_points_[0], *path_, *path_velocity);
        }

        virtual void reset
        () {
            FM2Base::reset();
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

        //using solver_::distances_;

       // using FMM < grid_t, heap_t>::neighbors;

       // using FMM < grid_t, heap_t>::Tvalues;
       // using FMM < grid_t, heap_t>::TTvalues;
       // using FMM < grid_t, heap_t>::narrow_band_;

        //double sumT; /*! <  Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
        //double sumTT; /*! <  Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */

        std::array <unsigned int, grid_t::getNDims()> heur_coord_; /*! <  'Goal' coord, goal of the second wave propagation (actually the initial point of the path). */
        //std::vector<double> distances_; /*! <  Auxiliar container of euclidean distances for the Fast Marching Square Star heuristic. */
};

#endif /* FM2STAR_HPP_*/
