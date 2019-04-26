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
 * \file   sliding_puzzle.h
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   3/27/19
 */

#ifndef PROJECT_SLIDING_PUZZLE_H
#define PROJECT_SLIDING_PUZZLE_H

// Project
#include <sliding_puzzle/hash_functions.h>
#include <common/common.hpp>

// Boost
#include <boost/functional/hash.hpp>

// STL
#include <unordered_map>
#include <array>

namespace sliding_puzzle
{
typedef std::chrono::high_resolution_clock Clock;
using namespace std::chrono;

class SlidingPuzzle
{
public:

  typedef std::shared_ptr<SlidingPuzzle> Ptr;

  typedef typename SlidingPuzzleState::Ptr SlidingPuzzleStatePtr;
  typedef typename SlidingPuzzleState::Compare SlidingPuzzleStateCompare;
  typedef csbpl_planner::Vertex<SlidingPuzzleState,
                                double> SearchVertex;
  typedef typename SearchVertex::Ptr SearchVertexPtr;

  SlidingPuzzle(int size, int anchor);

  bool generateSolvableStartState(int soln_lower_bound);

  bool terminatePlanning(const SlidingPuzzleStatePtr& state);

  bool generateSuccessorForAction(const SlidingPuzzleStatePtr& current_state,
                                  const int* anchor_location,
                                  const char action,
                                  SlidingPuzzleState& successor_state);

  std::vector<SearchVertexPtr> getValidSuccessors(const SearchVertexPtr& current_vertex);

  int getCostToSuccessor(const SlidingPuzzleStatePtr& current_state,
                         const SlidingPuzzleStatePtr& successor_state);

  int calculateManhattanDistance(const SlidingPuzzleStatePtr& state);

  int calculateHammingDistance(const SlidingPuzzleStatePtr& state);

  int calculateLinearConflict(const SlidingPuzzleStatePtr& state);

  double getAdmissibleHeuristicCost(const SlidingPuzzleStatePtr& state);

  double getInadmissibleHeuristicCost(const SlidingPuzzleStatePtr& state,
                                      int hidx);
  
  SlidingPuzzleStatePtr getStartState();

  SlidingPuzzleStatePtr getGoalState();

//protected:

  std::unordered_map<SlidingPuzzleStatePtr,
                     SearchVertexPtr,
                     std::hash<SlidingPuzzleStatePtr>,
                     SlidingPuzzleStateCompare> explored_;

  SlidingPuzzleStatePtr start_;
  SlidingPuzzleStatePtr goal_;

  //@TODO Move this to csbpl_planner Params struct and pass around struct object for all the planners
  double heuristic_weight_range_[2];
  int num_inadmissible_heuristic_;
  std::vector<std::array<double,3> > heuristic_weight_;
  
  };



} // namespace sliding_puzzle

#endif //PROJECT_SLIDING_PUZZLE_H



