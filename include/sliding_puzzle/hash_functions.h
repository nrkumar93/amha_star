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
 * \file   hash_functions.h
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   3/27/19
 */

#ifndef PROJECT_HASH_FUNCTIONS_H
#define PROJECT_HASH_FUNCTIONS_H

// Project
#include <sliding_puzzle/config.h>
#include <common/binary_heap.h>
#include <sliding_puzzle/sliding_puzzle.h>

// Boost
#include <boost/functional/hash.hpp>

// STL
#include <unordered_map>
#include <memory>
#include <cstdio>
#include <vector>
#include <limits>
#include <string>
#include <chrono>

namespace sliding_puzzle
{

struct SlidingPuzzleState
{
  typedef std::shared_ptr<SlidingPuzzleState> Ptr;

  SlidingPuzzleState() : state_(puzzle_size, std::vector<int>(puzzle_size))
  {
  }

  friend std::ostream &operator<<(std::ostream &output,
                                  const SlidingPuzzleState &state)
  {
    for (int i=0; i<puzzle_size; ++i)
    {
      for (int j=0; j<puzzle_size; ++j)
      {
        output << state.state_[i][j] << "\t";
      }
      output << std::endl;
    }
    return output;
  }

  std::vector<std::vector<int> > state_;

  struct Compare
  {
    bool operator()(const Ptr& a,
                    const Ptr& b) const
    {
      for (int i=0; i<a->state_.size(); ++i)
      {
        if (a->state_[i] != b->state_[i])
        {
          return false;
        }
        if (i == a->state_.size()-1)
        {
          return true;
        }
      }
    }
  };
};

} // namespace sliding_puzzle

namespace std
{
//typedef std::chrono::high_resolution_clock Clock;
//using namespace std::chrono;

template <>
struct hash<sliding_puzzle::SlidingPuzzleState>
{
  std::size_t operator()(const sliding_puzzle::SlidingPuzzleState &state) const
  {
    std::string state_string;
    std::string space = " ";
  
    std::chrono::high_resolution_clock::time_point hash_prof = std::chrono::high_resolution_clock::now();
  
    for (int i=0; i<state.state_.size(); ++i)
    {
      for (int j=0; j<state.state_[i].size(); ++j)
      {
        state_string += std::to_string(state.state_[i][j]) + space;
      }
    }
    
    auto hash_value = static_cast<std::size_t>(std::hash<std::string>()(state_string));

    sliding_puzzle::hash_time += std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::high_resolution_clock::now() - hash_prof).count();

    return hash_value;
  }

};

template <>
struct hash<sliding_puzzle::SlidingPuzzleState::Ptr>
{
  std::size_t operator()(const sliding_puzzle::SlidingPuzzleState::Ptr& state_ptr) const
  {
    return hash<sliding_puzzle::SlidingPuzzleState>()(*state_ptr);
  }
};

} // namespace std


#endif //PROJECT_HASH_FUNCTIONS_H
