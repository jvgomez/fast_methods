/*! \class FSM
    \brief Implements Fast Sweeping Method.
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to use an FMCell or derived

    The grid is assumed to be squared, that is Delta(x) = Delta(y) = leafsize_

    @par External documentation:
        H. Zhao, A fast sweeping method for Eikonal equations, Math. Comp. 74 (2005), 603-627.
        <a href="http://www.ams.org/journals/mcom/2005-74-250/S0025-5718-04-01678-3/S0025-5718-04-01678-3.pdf">[PDF]</a>

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

#ifndef FSM_HPP_
#define FSM_HPP_

#include "fmm.hpp"

#include <array>

#define MAXDIMS 3

// \TODO: Implement exponentiation by squaring: http://stackoverflow.com/questions/101439/the-most-efficient-way-to-implement-an-integer-based-power-function-powint-int
// \TODO: support for a goal point.
template < class grid_t > class FSM : public FMM <grid_t> {

    public:
        unsigned maxSweeps;

        FSM(const char * name = "FMS") : FMM<grid_t>(name), sweeps_(0) {
            incs_[0] = -1;
            maxSweeps = std::numeric_limits<unsigned>::max();
        }

        virtual ~FSM() { clear(); }

        virtual void setEnvironment
        (grid_t * g) {
            FMM<grid_t>::setEnvironment(g);
            std::array<unsigned, grid_t::getNDims()> dimsize = g->getDimSizes();
            for (size_t i = 0; i < grid_t::getNDims(); ++i)
                dimsize_[i] = dimsize[i];
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
            unsigned idx = 0;

            while (keepSweeping && sweeps_ < maxSweeps)
            {
                keepSweeping = false;
                setSweep();

                for (int k = inits_[2]; k != ends_[2]; k+=incs_[2])
                    for (int j = inits_[1]; j != ends_[1]; j+=incs_[1])
                        for (int i = inits_[0]; i != ends_[0]; i+=incs_[0])
                        {
                            idx = k*dimsize_[0]*dimsize_[1] + j *dimsize_[0] + i;
                            const double prevTime = grid_->getCell(idx).getArrivalTime();
                            const double newTime = solveEikonal(idx);
                            if(newTime + utils::COMP_MARGIN < prevTime) {
                                grid_->getCell(idx).setArrivalTime(newTime);
                                keepSweeping = true;
                            }
                        }
                ++sweeps_;
            }

            std::cout << sweeps_ << "  " << std::numeric_limits<double>::epsilon() << '\n';
        }

        virtual void clear
        () {
            incs_[0] = -1;
            maxSweeps = std::numeric_limits<unsigned>::max();
        }

        virtual void reset
        () {
            FMM<grid_t>::reset();
            sweeps_ = 0;
            incs_[0] = -1;
        }

    protected:
        /** \brief Set the sweep variables: initial and final indices for iterations,
             and the increment of each iteration in every dimension.

             Generates the pattern (111, -111, 1-11, -1,-1,1, 11-1, -11-1,...). */
        void setSweep
        (){
            // Dimension 0 changes sweep direction every time.
            incs_[0] *= -1;
            for (size_t i = 1; i < grid_t::getNDims(); ++i)
            {
                unsigned it = sweeps_ % unsigned(pow(2,i+1));
                if (it > pow(2,i)-1)
                    incs_[i] = -1;
                else
                    incs_[i] = 1;
            }

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

            // Setting extra dimensions
            for (size_t i = grid_t::getNDims(); i < MAXDIMS; ++i) {
                incs_[i] = 1;
                inits_[i] = 0;
                ends_[i] = 1;
            }
        }

        using FMM<grid_t>::grid_;
        using FMM<grid_t>::neighbors;
        using FMM<grid_t>::solveEikonal;
        using FMM<grid_t>::init_points_;
        using FMM<grid_t>::goal_idx_;
        using Solver<grid_t>::setup;
        using FMM<grid_t>::setup_;

    private:
        unsigned int sweeps_;
        std::array<int, MAXDIMS> incs_;
        std::array<int, MAXDIMS> inits_;
        std::array<int, MAXDIMS> ends_;
        std::array<int, MAXDIMS> dimsize_;


};

#endif /* FSM_HPP_*/
