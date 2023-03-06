/*
 * Copyright (c) 2019, Ramkumar Natarajan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   sliding_puzzle_test.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   3/29/19
 */

// Project
#include <dijkstra/dijkstra.hpp>
#include <amha_star/amha_star.hpp>
#include <sliding_puzzle/sliding_puzzle.h>

// STL
#include <iostream>


int main()
{
  typedef csbpl_planner::Dijkstra<sliding_puzzle::SlidingPuzzle,
                                  sliding_puzzle::SlidingPuzzleState,
                                  double> DijkstraType;
  typedef csbpl_planner::AMHAStar<sliding_puzzle::SlidingPuzzle,
                                  sliding_puzzle::SlidingPuzzleState,
                                  double> AMHAStarType;

  int puzzle_size = 5;
  int puzzle_anchor = 0;
  sliding_puzzle::SlidingPuzzle::Ptr env_ptr =
    std::make_shared<sliding_puzzle::SlidingPuzzle>(puzzle_size,
                                                    puzzle_anchor);

  double time_limit = 10;
  double eps = 10;
  double del_eps = 1;
  
  double mha_eps = 10;
  double del_mha_eps = 1;
  
  int num_inad = 4;

  std::shared_ptr<DijkstraType> planner;
  planner = std::make_shared<AMHAStarType>(env_ptr,
                                           time_limit,
                                           eps,
                                           mha_eps,
                                           del_eps,
                                           del_mha_eps,
                                           num_inad);

  sliding_puzzle::SlidingPuzzleState::Ptr start_state = env_ptr->getStartState();
  sliding_puzzle::SlidingPuzzleState::Ptr goal_state = env_ptr->getGoalState();

  
  if (!planner->run(start_state, goal_state))
  {
    std::cout << "Plannar did not return any solution. Error!" << std::endl;
    return 0;
  }

  csbpl_planner::AnytimePlannerStats<sliding_puzzle::SlidingPuzzleState>
    anytime_stats = planner->getAnytimePlannerStats();
  std::cout << "The num solutions found are: "
            << anytime_stats.solution_paths_.size() << std::endl;
  
  for (int i=0; i<anytime_stats.solution_paths_.size(); ++i)
  {
    std::cout << "Path lengths are: " << anytime_stats.solution_paths_[i].size()
              << " with epsilon: " << anytime_stats.solution_eps_[i]
              << " and planning time is: " << anytime_stats.solution_time_[i]
              << " and hash time is: " << sliding_puzzle::hash_time
              << " and num expansions is: " << anytime_stats.num_expansions_[i] << std::endl;
  }

  return 0;
}
