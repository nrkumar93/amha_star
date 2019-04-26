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
 * \file   common.hpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   3/27/19
 */

#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

// csbpl_planner
#include <common/binary_heap.h>

// STL
#include <limits>
#include <chrono>

namespace csbpl_planner
{
  
typedef std::chrono::high_resolution_clock Clock;
using namespace std::chrono;
  
struct Params
{
  // Anytime
  double eps_;
  double delta_eps_;
  double E_;
  double G_;

  // MHA*
  double mha_eps_;
  double delta_mha_eps_;
  int num_inad_;

  Clock::time_point start_time_;
  double time_limit_;
};

template<typename State>
struct PlannerStats
{
  typedef typename State::Ptr StatePtr;
  
  std::vector<StatePtr> solution_paths_;
  double solution_eps_;
  double solution_time_;
  int num_expansions_;
};


template<typename State>
struct AnytimePlannerStats
{
  typedef typename State::Ptr StatePtr;

  std::vector<std::vector<StatePtr> > solution_paths_;
  std::vector<double> solution_eps_;
  std::vector<double> solution_time_;
  std::vector<double> cummulative_time_;
  std::vector<int> num_expansions_;
};

template<typename State,
         typename HeapValueType>
struct Vertex : public csbpl_common::Heap<HeapValueType>::Element
{
  typedef std::shared_ptr<Vertex<State,
                          HeapValueType> > Ptr;
  typedef typename State::Ptr StatePtr;

  Vertex(StatePtr state) : state_(state)
  {
    this->key_ = std::numeric_limits<double>::infinity();
    parent_vertex_ = nullptr;
    g_cost_ = std::numeric_limits<double>::infinity();
    f_cost_ = std::numeric_limits<double>::infinity();
  }

  StatePtr state_;

  Ptr parent_vertex_;

  // Copies for multi heuristic search
  std::vector<Ptr> mha_copy_;
  Ptr self_ptr_;
  
  // Nonparametric copy
  Ptr np_copy_;

  double g_cost_;

  double h_cost_;
  
  double e_cost_;
  
  double f_cost_;

  int root_id_;

  int depth_;
};



} // namespace csbpl_planner

#endif //PROJECT_COMMON_H
