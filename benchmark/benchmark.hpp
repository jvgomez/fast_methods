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

template <class grid_t>
class Benchmark {

    public:

        Benchmark
        (bool saveGrid = false, bool saveLog = true) :
        saveGrid_(saveGrid),
        saveLog_(saveLog),
        runID_(0),
        nruns_(10),
        path_("results"),
        name_("benchmark"){}

        virtual ~Benchmark()
        {
            clear();
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

        void setPath
        (const boost::filesystem::path & path) {
            path_ = path;
        }

        void setSaveLog
        (bool s) {
            saveLog_ = s;
        }

        void setInitialAndGoalPoints
        (const std::vector<unsigned int> & init_points, unsigned int goal_idx) {
            init_points_ = init_points;
            goal_idx_ = goal_idx;
        }

        void setInitialPoints(const std::vector<unsigned int> & init_points)
        {
            setInitialAndGoalPoints(init_points, -1);
        }

        void run
        () {
            if (saveGrid_)
            {
                boost::filesystem::create_directory(path_);
                boost::filesystem::create_directory(path_ / name_);
            }
            else if (saveLog_)
                boost::filesystem::create_directory(path_);

            boost::progress_display showProgress (solvers_.size()*nruns_);

            setupSolvers();

            logConfig();

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
            log_ << '\n' << fmtID_;

            std::cout.copyfmt(init);
            log_ << '\t' << s->getName() << "\t" << time;
        }

        void saveGrid
        (Solver<grid_t>* s) const {
            thread_local boost::filesystem::path filename;
            filename = path_ / name_ / fmtID_;
            filename.replace_extension(".grid");
            GridWriter::saveGridValues(filename.string().c_str(), *(s->getGrid()));
        }

        void saveLog
        () const {
            std::ofstream ofs (path_.string() + "/" + name_ + ".log");
            ofs << log_.rdbuf();
            ofs.close();
        }

        void setName
        (const std::string & n)
        {
            name_ = n;
        }

        void clear
        () {
            for (auto & s : solvers_)
            {
                s->clear();
                delete s;
            }
            solvers_.clear();
            delete grid_;
        }

    private:
        void logConfig
        () {
            log_ << name_ << '\t' << nruns_ << '\t' << grid_->getNDims()
                 << '\t' << grid_->getDimSizesStr();

            // Saving starting points.
            log_ << init_points_.size() << '\t';
            std::copy(init_points_.begin(), init_points_.end(), std::ostream_iterator<unsigned int>(log_, "\t"));

            if (goal_idx_ == -1)
                log_ << "nan";
            else
                log_ << goal_idx_;
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
        std::string name_;
};

#endif /* BENCHMARK_HPP_*/
