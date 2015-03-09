/*! \class FSM
    \brief Implements Fast Sweeping Method up to 3D.

    IMPORTANT NOTE: A maximum number of dimensions is set because of the sweeps implementation
    which require nested loops. The alternative is to do it recursively, but that would make the
    algorithm slower and FSM is rarely used above 3D.

    The FSM code contains information about how to set the maximum number of dimensions.

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to use an FMCell or derived.

    The grid is assumed to be squared, that is Delta(x) = Delta(y) = leafsize_

    @par External documentation:
        H. Zhao, A fast sweeping method for Eikonal equations, Math. Comp. 74 (2005), 603-627.
        <a href="http://www.ams.org/journals/mcom/2005-74-250/S0025-5718-04-01678-3/S0025-5718-04-01678-3.pdf">[PDF]</a>

    NOTE: The sweeping directions are inverted with respect to the paper to make implementation easier.

    Copyright (C) 2015 Javier V. Gomez
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

#ifndef FSM_HPP_
#define FSM_HPP_

#include "solver.hpp"

// Dimension: change this value to the number of dimensions required.
#define MAXDIMS 3

template < class grid_t > class FSM : public Solver<grid_t> {

    public:
        FSM(unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : Solver<grid_t>("FSM"),
            sweeps_(0),
            maxSweeps_(maxSweeps) {
            initializeSweepArrays();
        }

        FSM(const char * name, unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : Solver<grid_t>(name),
            sweeps_(0),
            maxSweeps_(maxSweeps) {
            initializeSweepArrays();
        }

        /** \brief Sets and cleans the grid in which operations will be performed.
             Since a maximum number of dimensions is assumed, fills the rest with size 1. */
        virtual void setEnvironment
        (grid_t * g) {
            Solver<grid_t>::setEnvironment(g);
            // Filling the size of the dimensions...
            std::array<unsigned, grid_t::getNDims()> dimsize = g->getDimSizes();
            for (size_t i = 0; i < grid_t::getNDims(); ++i)
                dimsize_[i] = dimsize[i];
            // ... and the extended dimensions.
            for (size_t i = grid_t::getNDims(); i < MAXDIMS; ++i)
                dimsize_[i] = 1;
        }

        /** \brief Actual method that implements FSM. */
        virtual void computeInternal
        () {
            if (!setup_)
                setup();

            // Initialization
            for (unsigned int i: init_points_) // For each initial point
                grid_->getCell(i).setArrivalTime(0);

            // Getting dimsizes and filling the other dimensions.
            bool keepSweeping = true;
            bool stopPropagation = false;
            unsigned idx = 0;

            while (keepSweeping && !stopPropagation && sweeps_ < maxSweeps_)
            {
                keepSweeping = false;
                setSweep();
                ++sweeps_;

                // Dimension: nest as many for loops as MAXDIMS.
                for (int k = inits_[2]; k != ends_[2]; k+=incs_[2])
                    for (int j = inits_[1]; j != ends_[1]; j+=incs_[1])
                        for (int i = inits_[0]; i != ends_[0]; i+=incs_[0])
                        {
                            // Dimension: update the index computation (for 4D it is
                            // l*dimsize_[0]*dimsize_[1]*dimsize_[2] + k*...
                            // and so on.
                            idx = k*dimsize_[0]*dimsize_[1] + j *dimsize_[0] + i;
                            const double prevTime = grid_->getCell(idx).getArrivalTime();
                            const double newTime = solveEikonal(idx);
                            if(newTime + utils::COMP_MARGIN < prevTime) {
                                grid_->getCell(idx).setArrivalTime(newTime);
                                keepSweeping = true;
                            }
                            // Value not updated, it has converged
                            else if(!isnan(newTime) && (idx == goal_idx_)) {
                                std::cout << newTime << '\n';
                                stopPropagation = true;
                            }
                        }

            }
        }

        /** \brief FSM-specific solver. Solves nD Eikonal equation for cell idx.
             If the neighbor value is higher than the current value is treated as
             an infinite. */
        virtual double solveEikonal
        (const int & idx) {
            unsigned int a = grid_t::getNDims(); // a parameter of the Eikonal equation.

            double updatedT;
            sumT = 0;
            sumTT = 0;

            for (unsigned int dim = 0; dim < grid_t::getNDims(); ++dim) {
                double minTInDim = grid_->getMinValueInDim(idx, dim);

                if (!isinf(minTInDim) && minTInDim < grid_->getCell(idx).getArrivalTime()) {
                    Tvalues[dim] = minTInDim;
                    sumT += Tvalues[dim];
                    TTvalues[dim] = Tvalues[dim]*Tvalues[dim];
                    sumTT += TTvalues[dim];
                }
                else {
                    Tvalues[dim] = 0;
                    TTvalues[dim] = 0;
                    a -=1 ;
                }
            }

            double b = -2*sumT;
            double c = sumTT - grid_->getLeafSize() * grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity());
            double quad_term = b*b - 4*a*c;
            if (quad_term < 0) {
                double minT = *(std::min_element(Tvalues.begin(), Tvalues.end()));
                updatedT = grid_->getLeafSize() * grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()) + minT;
            }
            else
                updatedT = (-b + sqrt(quad_term))/(2*a);

            return updatedT;
        }

        virtual void reset
        () {
            Solver<grid_t>::reset();
            sweeps_ = 0;
            initializeSweepArrays();
        }

        virtual void printRunInfo
        () const {
            console::info("Fast Sweeping Method");
            std::cout << '\t' << name_ << '\n'
                      << '\t' << "Maximum sweeps: " << maxSweeps_ << '\n'
                      << '\t' << "Sweeps performed: " << sweeps_ << '\n'
                      << '\t' << "Elapsed time: " << time_ << " ms\n";
        }

    protected:
        /** \brief Set the sweep variables: initial and final indices for iterations,
             and the increment of each iteration in every dimension.

             Generates a periodical pattern for incs_ (example for 3D):
            [-1-1-1, 1-1-1, -11-1, 11-1, -1-11, 1-11, -111, 111]

             Stablishes inits_ and ends_ accordingly. */
        virtual void setSweep
        () {
            // Inspired in http://stackoverflow.com/a/17758788/2283531
            for (size_t i = 0; i < grid_t::getNDims(); ++i)
            {
                if((incs_[i] += 2) <=1)
                    break;
                else
                    incs_[i] = -1;
            }

            /*for (size_t i = 0; i < grid_t::getNDims(); ++i)
                std::cout << incs_[i] << "  ";
            std::cout << '\n';*/

            // Setting inits and ends.
            for (size_t i = 0; i < grid_t::getNDims(); ++i)
            {
                if (incs_[i] == 1)
                {
                    inits_[i] = 0;
                    ends_[i] = dimsize_[i];
                }
                else
                {
                    inits_[i] = dimsize_[i]-1;
                    ends_[i] = -1;
                }
            }
        }

        virtual void initializeSweepArrays
        () {
            for (size_t i = 0; i < MAXDIMS; ++i) {
                incs_[i] = 1;
                inits_[i] = 0;
                ends_[i] = 1;
            }
        }

        using Solver<grid_t>::grid_;
        using Solver<grid_t>::init_points_;
        using Solver<grid_t>::goal_idx_;
        using Solver<grid_t>::setup;
        using Solver<grid_t>::setup_;
        using Solver<grid_t>::name_;
        using Solver<grid_t>::time_;

    private:
        /** \brief Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
        double                                          sumT;

        /** \brief Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */
        double                                          sumTT;

        /** \brief Auxiliar array with values T0,T1...Tn-1 variables in the Discretized Eikonal Equation. */
        std::array<double, grid_t::getNDims()>          Tvalues;

        /** \brief Auxiliar array with values T0^2,T1^2...Tn-1^2 variables in the Discretized Eikonal Equation. */
        std::array<double, grid_t::getNDims()>          TTvalues;

        /** \brief Number of sweeps performed. */
        unsigned int sweeps_;

        /** \brief Number of maximum sweeps to perform. */
        unsigned maxSweeps_;

        /** \brief Sweep directions {-1,1} for each dimension. Extended dimensions always 1. */
        std::array<int, MAXDIMS> incs_;

        /** \brief Initial indices for each dimension. Extended dimensions always 0. */
        std::array<int, MAXDIMS> inits_;

        /** \brief Final indices for each dimension. Extended dimensions always 1. */
        std::array<int, MAXDIMS> ends_;

        /** \brief Size of each dimension, extended to the maximum size. Extended dimensions always 1. */
        std::array<int, MAXDIMS> dimsize_;
};

#endif /* FSM_HPP_*/
