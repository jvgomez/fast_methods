/*! \file benchmark.hpp
    \brief
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

#ifndef BENCHMARK_HPP_
#define BENCHMARK_HPP_

#include <chrono>
#include <sstream>

#include "solver.hpp"
#include "../io/gridwriter.hpp"

// TODO: In case of poor performance, improve the logging (too many string copies and creations).

template <class grid_t>
class Benchmark {

    public:

        Benchmark (const bool saveGrid = false) : saveGrid_(saveGrid) {
            runID_ = 0;
        }

        void addSolver
        (Solver<grid_t>* solver) {
            solvers_.push_back(solver);
        }

        void setSaveGrid
        (const bool s) {
            saveGrid_ = s;
        }

        void setEnvironment
        (grid_t* grid) {
            grid_ = grid;
        }

        void setInitialAndGoalPoints
        (const std::vector<int> & init_points, int goal_idx) {
            init_points_ = init_points;
            goal_idx_ = goal_idx;
        }

        void setInitialPoints(const std::vector<int> & init_points)
        {
            setInitialAndGoalPoints(init_points,-1);
        }

        void run
        () {
            setupSolvers();

            for (Solver<grid_t>* s :solvers_)
            {
                ++runID_;
                start_ = std::chrono::system_clock::now();
                s->compute();
                end_ = std::chrono::system_clock::now();
                const double time_elapsed = getMilliSeconds(start_, end_);

                logRun(s, time_elapsed);

                if (saveGrid_)
                    saveGrid(s);

                //std::cout << "\tElapsed "<< s->getName() <<" time: " << time_elapsed << " ms" << '\n';
                /*std::string filename ("test_");
                filename += s->getName();
                filename += ".txt";
                GridWriter::saveGridValues(filename.c_str(), grid_fmm);*/
                s->reset();
            }

            std::cout << log_.str();
        }

    private:

        void logRun
        (const Solver<grid_t>* s, const double& time)
        {
            std::ios init(NULL);
            init.copyfmt(std::cout);
            log_ << getRunID();

            std::cout.copyfmt(init);
            log_ << '\t' << s->getName() << "\t" << s->getGrid()->getNDims()
                 << '\t' << s->getGrid()->getDimSizesStr() << time << '\n';
        }

        void saveGrid
        (Solver<grid_t>* s) {
            std::string filename ("grid_");
            filename += s->getName();
            filename += '_';
            filename += getRunID();
            filename += ".txt";
            GridWriter::saveGridValues(filename.c_str(), *(s->getGrid()));
        }

        void setupSolvers
        () {
            for (Solver<grid_t>* s :solvers_)
            {
                s->setEnvironment(grid_);
                s->setInitialAndGoalPoints(init_points_, goal_idx_);
                s->setup();
            }
        }

        const std::string getRunID
        () {
            std::ostringstream oss;
            oss << std::setw(4) << std::setfill('0') << runID_;
            return oss.str();
        }

        double getMilliSeconds
        (const std::chrono::time_point<std::chrono::system_clock>& s,
         const std::chrono::time_point<std::chrono::system_clock>& e) const {
            return std::chrono::duration_cast<std::chrono::milliseconds>(e-s).count();
        }

        std::vector<Solver<grid_t>*> solvers_;
        grid_t * grid_;

        std::vector<int> init_points_;  /*!< Initial points. */
        int goal_idx_;   /*!< Index of goal point. */

        std::chrono::time_point<std::chrono::system_clock> start_, end_; /*!< Time measuring variables. */

        std::ostringstream log_;

        bool saveGrid_;
        int runID_;
};

#endif /* BENCHMARK_HPP_*/
