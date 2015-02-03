/*! \file benchmarkcfg.hpp
    \brief

    Inspired in the OMPLapp benchmarking software: http://ompl.kavrakilab.org

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

#ifndef BENCHMARKCFG_HPP_
#define BENCHMARKCFG_HPP_

#include <string>
#include <unordered_map>

#include <boost/program_options.hpp>
#include <boost/algorithm/string/case_conv.hpp>



/*#include <chrono>
#include <sstream>
#include <limits>

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include "../fmm/solver.hpp"
#include "../io/gridwriter.hpp"*/

// TODO: the getter functions do not check if the types are admissible.

class BenchmarkCFG {

    public:

        BenchmarkCFG
        () {}

        BenchmarkCFG
        (const char * filename) {
            readOptions(filename);
        }

        bool readOptions(const char * filename)
        {
            static const std::vector<std::string> knownSolvers = {
                "fmm", "fmmfib", "sfmm",
                "gmm", "fim", "ufmm"
            };

            std::fstream cfg(filename);
            if (!cfg.good())
            {
               std::string error("Unable to open file: ");
               error += filename;
               console::error(error);
               return 0;
            }

            boost::program_options::options_description desc;
            desc.add_options()
               ("grid.ndims",         boost::program_options::value<std::string>()->default_value("2"),         "Number of dimensions.")
               ("grid.cell",          boost::program_options::value<std::string>()->default_value("FMCell"),    "Type of cell. FMCell by default.")
               ("grid.dimsize",       boost::program_options::value<std::string>()->required(),                 "Size of dimensions: N,M,O...")
               ("problem.start",      boost::program_options::value<std::string>()->required(),                 "Start point: s1,s2,s3...")
               ("problem.goal",       boost::program_options::value<std::string>()->required(),                 "Goal point: g1,g2,g3...")
               ("benchmark.name",     boost::program_options::value<std::string>()->default_value("benchmark"), "Name of the benchmark.")
               ("benchmark.runs",     boost::program_options::value<std::string>()->default_value("10"),        "Number of runs per solver.")
               ("benchmark.savegrid", boost::program_options::value<std::string>()->default_value("0"),         "Save grid values of each run.");

            boost::program_options::variables_map vm;
            boost::program_options::parsed_options po = boost::program_options::parse_config_file(cfg, desc, true);
            boost::program_options::store(po, vm);
            boost::program_options::notify(vm);
            cfg.close();

            // Storing registered options, intended for grid, problem and benchmark configuration.
            for (const auto& var : vm)
                options_[var.first] = boost::any_cast<std::string>(var.second.value());

            // Analyze unregistered options, intended for solvers.
            std::vector<std::string> unr = boost::program_options::collect_unrecognized(po.options, boost::program_options::exclude_positional);
            for (std::size_t i = 0 ; i < unr.size()/2 ; ++i)
            {
                std::string key = boost::to_lower_copy(unr[i*2]);
                // std::string val = unr[i*2 + 1]; // Values not used.

                if (key.substr(0,8) == "solvers.")
                {
                    std::string solver = key.substr(8);

                    for (std::size_t i = 0; i < knownSolvers.size(); ++i)
                        if (solver.substr(0, knownSolvers[i].length()) == knownSolvers[i])
                            solverNames_.push_back(knownSolvers[i]);

                }
                else
                    continue;
            }

            return 1;
       }

        template<typename T>
        T getValue
        (const std::string & key) const {
            std::unordered_map<std::string, std::string>::const_iterator iter = options_.find(key);
            if (iter != options_.end())
                return boost::lexical_cast<T>(iter->second);
            else
                return boost::lexical_cast<T>(0);
        }

        const std::vector<std::string> & getSolverNames
        () {
            return solverNames_;
        }

    private:

       std::vector<std::string> solverNames_;
       std::unordered_map<std::string, std::string> options_;

};

#endif /* BENCHMARKCFG_HPP_*/
