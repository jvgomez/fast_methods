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
#include <limits>

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include "../fmm/solver.hpp"
#include "../io/gridwriter.hpp"
#include "benchmarkcfg.hpp"

template <class grid_t>
class Benchmark {

    public:

        Benchmark
        (bool saveGrid = false, bool saveLog = true) :
        saveGrid_(saveGrid),
        saveLog_(saveLog),
        runID_(0),
        nruns_(10),
        path_("results") {}

        Benchmark
        (BenchmarkCFG & bcfg) :
        saveLog_(true),
        runID_(0) {
            saveGrid_ = bcfg.getValue<bool>("benchmark.savegrid");
            nruns_ = bcfg.getValue<unsigned int>("benchmark.runs");
            path_ = boost::filesystem::path("results_" + bcfg.getValue<std::string>("benchmark.name"));

            for(const auto & s : bcfg.getSolverNames())
            {
                if (s == "fmm")
                        std::cout << "FMM!" <<'\n';
                else if (s == "fmmfib")
                        std::cout << "FMMFib!" <<'\n';
                else if (s == "sfmm")
                        std::cout << "SFMM!" <<'\n';
                else if (s == "gmm")
                        std::cout << "GMM!" <<'\n';
                else if (s == "fim")
                        std::cout << "FIM!" <<'\n';
                else if (s == "ufmm")
                        std::cout << "UFMM!" <<'\n';
            }


        }

        void addSolver
        (Solver<grid_t>* solver) {
            solvers_.push_back(solver);
        }

        void setSaveGrid
        (bool s) {
            saveGrid_ = s;
        }

        void setEnvironment
        (grid_t* grid) {
            grid_ = grid;
        }

        void setNRuns
        (unsigned int n) {
            nruns_ = n;
        }

        void setInitialAndGoalPoints
        (const std::vector<unsigned int> & init_points, unsigned int goal_idx) {
            init_points_ = init_points;
            goal_idx_ = goal_idx;
        }

        void setInitialPoints(const std::vector<unsigned int> & init_points)
        {
            setInitialAndGoalPoints(init_points, std::numeric_limits<unsigned int>::quiet_NaN());
        }

        void run
        () {
            if (saveGrid_ || saveLog_)
                boost::filesystem::create_directory(path_);

            boost::progress_display showProgress (solvers_.size()*nruns_);

            setupSolvers();

            for (Solver<grid_t>* s :solvers_)
            {
                for (unsigned int i = 0; i < nruns_; ++i)
                {
                    ++runID_;
                    start_ = std::chrono::system_clock::now();
                    s->compute();
                    end_ = std::chrono::system_clock::now();
                    const double time_elapsed = getMilliSeconds(start_, end_);

                    logRun(s, time_elapsed);

                    if (saveGrid_)
                        saveGrid(s);

                    s->reset();
                    ++showProgress;
                }
            }

            if (saveLog_)
                saveLog();
            else
                std::cout << log_.str();
        }

        void logRun
        (const Solver<grid_t>* s, const double& time)
        {
            std::ios init(NULL);
            init.copyfmt(std::cout);
            formatID();
            log_ << fmtID_;

            std::cout.copyfmt(init);
            log_ << '\t' << s->getName() << "\t" << s->getGrid()->getNDims()
                 << '\t' << s->getGrid()->getDimSizesStr() << time << '\n';
        }

        void saveGrid
        (Solver<grid_t>* s) const {
            thread_local std::string filename;
            filename = path_.string();
            filename += '/';
            filename += fmtID_;
            filename += ".grid";
            GridWriter::saveGridValues(filename.c_str(), *(s->getGrid()));
        }

        void saveLog
        () const {
            std::ofstream ofs (path_.string() + "/benchmark.log");
            ofs << log_.rdbuf();
            ofs.close();
        }

    private:
        void setupSolvers
        () {
            for (Solver<grid_t>* s :solvers_)
            {
                s->setEnvironment(grid_);
                s->setInitialAndGoalPoints(init_points_, goal_idx_);
                s->setup();
            }
        }

        void formatID
        () {
            thread_local std::ostringstream oss; // To optimize a bit.
            oss.str("");
            oss.clear();
            oss << std::setw(4) << std::setfill('0') << runID_;
            fmtID_ = oss.str();
        }

        double getMilliSeconds
        (const std::chrono::time_point<std::chrono::system_clock>& s,
         const std::chrono::time_point<std::chrono::system_clock>& e) const {
            return std::chrono::duration_cast<std::chrono::milliseconds>(e-s).count();
        }

        std::vector<Solver<grid_t>*> solvers_;
        grid_t * grid_;

        std::vector<unsigned int> init_points_;  /*!< Initial points. */
        unsigned int goal_idx_;   /*!< Index of goal point. */

        std::chrono::time_point<std::chrono::system_clock> start_, end_; /*!< Time measuring variables. */

        std::stringstream log_;

        bool saveGrid_;
        bool saveLog_;

        unsigned int runID_;
        std::string fmtID_;
        unsigned int nruns_;

        boost::filesystem::path path_;
};

#endif /* BENCHMARK_HPP_*/
