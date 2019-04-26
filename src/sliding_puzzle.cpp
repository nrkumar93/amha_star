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
 * \file   sliding_puzzle.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   3/27/19
 */

// Project
#include <sliding_puzzle/sliding_puzzle.h>
#include <sliding_puzzle/config.h>

// STL
#include <stdlib.h>
#include <time.h>

int sliding_puzzle::puzzle_size;
int sliding_puzzle::puzzle_anchor;
double sliding_puzzle::hash_time;

namespace sliding_puzzle
{

SlidingPuzzle::SlidingPuzzle(int size, int anchor)
{
  static bool seed_once = true;
  if (seed_once)
  {
    seed_once = false;
    srand(time(NULL));
  }
  
  puzzle_size = size;
  puzzle_anchor = anchor;
  
  hash_time = 0;
  
  // Multi heuristic params
  num_inadmissible_heuristic_ = 4;
  heuristic_weight_range_[0] = 1.0;
  heuristic_weight_range_[1] = 5.0;
  for (int i=0; i<num_inadmissible_heuristic_; ++i)
  {
    std::array<double,3> weight;
    for (int j=0; j<3; ++j)
    {
      weight[j] = heuristic_weight_range_[0] +
        static_cast<double>(rand())/
          (RAND_MAX/(heuristic_weight_range_[1]-heuristic_weight_range_[0]));
    }
    heuristic_weight_.push_back(weight);
  }
  
  start_ = std::make_shared<SlidingPuzzleState>();
  goal_ = std::make_shared<SlidingPuzzleState>();
  int value = 1;
  for (int i=0; i<puzzle_size; ++i)
  {
    for (int j=0; j<puzzle_size; ++j)
    {
      goal_->state_[i][j] = value % static_cast<int>(pow(puzzle_size,2));
      ++value;
    }
  }

  // Random start state
  std::vector<int> start_tiles;
  for (int i=0; i<puzzle_size*puzzle_size; ++i)
  {
    start_tiles.push_back(i);
  }
  
  for (int i=0; i<puzzle_size; ++i)
  {
    for (int j=0; j<puzzle_size; ++j)
    {
      int index = rand() % start_tiles.size();
      start_->state_[i][j] = start_tiles[index];
      start_tiles.erase(start_tiles.begin()+index);
    }
  }
  
  start_->state_[0][0] = 8;
  start_->state_[0][1] = 6;
  start_->state_[0][2] = 4;
  start_->state_[0][3] = 14;
  start_->state_[1][0] = 3;
  start_->state_[1][1] = 11;
  start_->state_[1][2] = 12;
  start_->state_[1][3] = 5;
  start_->state_[2][0] = 2;
  start_->state_[2][1] = 13;
  start_->state_[2][2] = 15;
  start_->state_[2][3] = 9;
  start_->state_[3][0] = 7;
  start_->state_[3][1] = 10;
  start_->state_[3][2] = 1;
  start_->state_[3][3] = 0;
  
  std::cout << "The random start state is: " << std::endl;
  std::cout << *start_ << std::endl;

}

bool SlidingPuzzle::generateSolvableStartState(int soln_lower_bound)
{
  int anchor_location[2];

  for (int j=0; j<goal_->state_.size(); ++j)
  {
    for (int k=0; k<goal_->state_[j].size(); ++k)
    {
      if (goal_->state_[j][k] == puzzle_anchor)
      {
        anchor_location[0] = j;
        anchor_location[1] = k;
      }
    }
  }

  SlidingPuzzleStatePtr rand_state = std::make_shared<SlidingPuzzleState>(*goal_);
  SlidingPuzzleStatePtr new_state = std::make_shared<SlidingPuzzleState>();

  for (int i=0; i<soln_lower_bound;)
  {
    char successor_action[] = {'N', 'E', 'W', 'S'};
    int rand_ind = rand() % static_cast<int>(4);


    if (generateSuccessorForAction(rand_state,
                                   anchor_location,
                                   successor_action[rand_ind],
                                   *new_state))
    {
      for (int j=0; j<new_state->state_.size(); ++j)
      {
        for (int k=0; k<new_state->state_[j].size(); ++k)
        {
          if (new_state->state_[j][k] == puzzle_anchor)
          {
            anchor_location[0] = j;
            anchor_location[1] = k;
          }
        }
      }

      *rand_state = *new_state;
      ++i;
    }
  }

  start_ = rand_state;

  std::cout << "The solvable start state is: " << std::endl;
  std::cout << *start_ << std::endl;

}

bool SlidingPuzzle::terminatePlanning(const SlidingPuzzleStatePtr& state)
{
  for (int i=0; i<state->state_.size(); ++i)
  {
    for (int j=0; j<state->state_[i].size(); ++j)
    {
      if (goal_->state_[i][j] != state->state_[i][j])
      {
        return false;
      }
    }
  }
  return true;
}

bool SlidingPuzzle::generateSuccessorForAction(const SlidingPuzzleStatePtr& current_state,
                                               const int* anchor_location,
                                               const char action,
                                               SlidingPuzzleState& successor_state)
{

  switch(action)
  {
    case 'N':
      if (anchor_location[0]==0)
      {
        return false;
      }
      else
      {
        successor_state = *current_state;
        successor_state.state_[anchor_location[0]-1][anchor_location[1]] =
                current_state->state_[anchor_location[0]][anchor_location[1]];
        successor_state.state_[anchor_location[0]][anchor_location[1]] =
                current_state->state_[anchor_location[0]-1][anchor_location[1]];
        return true;
      }
    case 'E':
      if (anchor_location[1]==puzzle_size-1)
      {
        return false;
      }
      else
      {
        successor_state = *current_state;
        successor_state.state_[anchor_location[0]][anchor_location[1]+1] =
                current_state->state_[anchor_location[0]][anchor_location[1]];
        successor_state.state_[anchor_location[0]][anchor_location[1]] =
                current_state->state_[anchor_location[0]][anchor_location[1]+1];
        return true;
      }
    case 'W':
      if (anchor_location[1]==0)
      {
        return false;
      }
      else
      {
        successor_state = *current_state;
        successor_state.state_[anchor_location[0]][anchor_location[1]-1] =
                current_state->state_[anchor_location[0]][anchor_location[1]];
        successor_state.state_[anchor_location[0]][anchor_location[1]] =
                current_state->state_[anchor_location[0]][anchor_location[1]-1];
        return true;
      }
    case 'S':
      if (anchor_location[0]==puzzle_size-1)
      {
        return false;
      }
      else
      {
        successor_state = *current_state;
        successor_state.state_[anchor_location[0]+1][anchor_location[1]] =
                current_state->state_[anchor_location[0]][anchor_location[1]];
        successor_state.state_[anchor_location[0]][anchor_location[1]] =
                current_state->state_[anchor_location[0]+1][anchor_location[1]];
        return true;
      }
  }
}

std::vector<SlidingPuzzle::SearchVertexPtr> SlidingPuzzle::getValidSuccessors(
        const sliding_puzzle::SlidingPuzzle::SearchVertexPtr &current_vertex)
{
  std::vector<SearchVertexPtr> successors;

  int anchor_location[2];

  for (int i=0; i<current_vertex->state_->state_.size(); ++i)
  {
    for (int j=0; j<current_vertex->state_->state_[i].size(); ++j)
    {
      if (current_vertex->state_->state_[i][j] == puzzle_anchor)
      {
        anchor_location[0] = i;
        anchor_location[1] = j;
      }
    }
  }

  char successor_action[] = {'N', 'E', 'W', 'S'};

  for (int i=0; i<4; ++i)
  {
    SlidingPuzzleState successor_state;

    if (generateSuccessorForAction(current_vertex->state_,
                                   anchor_location,
                                   successor_action[i],
                                   successor_state))
    {
      SlidingPuzzleStatePtr successor_state_ptr =
              std::make_shared<SlidingPuzzleState>(successor_state);
      if (explored_.find(successor_state_ptr) == explored_.end())
      {
        SearchVertexPtr successor =
                std::make_shared<SearchVertex>(successor_state_ptr);
        successors.push_back(successor);
        explored_[successor_state_ptr] = successor;
      }
      else
      {
        successors.push_back(explored_[successor_state_ptr]);
      }
    }
  }

  return successors;
}

int SlidingPuzzle::getCostToSuccessor(const SlidingPuzzleStatePtr &current_state,
                                      const SlidingPuzzleStatePtr &successor_state)
{
  return 1;
}

int SlidingPuzzle::calculateManhattanDistance(const SlidingPuzzleStatePtr& state)
{
  int manhattan_distance = 0;

  int target_location[2];
  for (int i=0; i<state->state_.size(); ++i)
  {
    for (int j=0; j<state->state_[i].size(); ++j)
    {
      if (state->state_[i][j] != 0)
      {
        target_location[0] = (state->state_[i][j]-1)/puzzle_size;
        target_location[1] = (state->state_[i][j]-1)%puzzle_size;

        manhattan_distance += abs(i-target_location[0]) +
                abs(j-target_location[1]);
      }
    }
  }

  return manhattan_distance;
}

int SlidingPuzzle::calculateHammingDistance(const SlidingPuzzleStatePtr& state)
{
  int hamming_distance = 0;

  int target_location[2];
  for (int i=0; i<state->state_.size(); ++i)
  {
    for (int j=0; j<state->state_[i].size(); ++j)
    {
      if (state->state_[i][j] != 0)
      {
        target_location[0] = (state->state_[i][j]-1)/puzzle_size;
        target_location[1] = (state->state_[i][j]-1)%puzzle_size;

        if (i != target_location[0] ||
            j != target_location[1])
        {
          hamming_distance += 1;
        }
      }
    }
  }

  return hamming_distance;
}

int SlidingPuzzle::calculateLinearConflict(const SlidingPuzzleStatePtr& state)
{
  int linear_conflict = 0;

  int target_location_a[2];
  int target_location_b[2];
  for (int i=0; i<state->state_.size(); ++i)
  {
    for (int j=0; j<state->state_[i].size(); ++j)
    {

      if (state->state_[i][j] != 0)
      {
        target_location_a[0] = (state->state_[i][j]-1)/puzzle_size;
        target_location_a[1] = (state->state_[i][j]-1)%puzzle_size;

        if (i == target_location_a[0] &&
            j == target_location_a[1])
        {
          continue;
        }
      }

      for (int k=i+1; k<state->state_.size(); ++k)
      {
        target_location_b[0] = (state->state_[k][j]-1)/puzzle_size;
        target_location_b[1] = (state->state_[k][j]-1)%puzzle_size;

        if ((target_location_a[1] == j) &&
            (target_location_b[1] == j))
        {
          if (target_location_a[0] > target_location_b[0])
          {
            linear_conflict += 1;
          }
        }
      }

      for (int k=j+1; k<state->state_[i].size(); ++k)
      {
        target_location_b[0] = (state->state_[i][k]-1)/puzzle_size;
        target_location_b[1] = (state->state_[i][k]-1)%puzzle_size;

        if ((target_location_a[0] == i) &&
            (target_location_b[0] == i))
        {
          if (target_location_a[1] > target_location_b[1])
          {
            linear_conflict += 1;
          }
        }
      }

    }
  }
  return linear_conflict;

}

double SlidingPuzzle::getAdmissibleHeuristicCost(const SlidingPuzzleStatePtr &state)
{
  return calculateManhattanDistance(state) +
          calculateLinearConflict(state);
}

double SlidingPuzzle::getInadmissibleHeuristicCost(const SlidingPuzzleStatePtr &state,
                                                   int hidx)
{
  if (hidx > num_inadmissible_heuristic_-1)
  {
    std::cerr << "Heuristic index out of range...!" << std::endl;
    return -1.0;
  }
  
  return heuristic_weight_[hidx][0]*calculateManhattanDistance(state) +
         heuristic_weight_[hidx][1]*calculateLinearConflict(state) +
         heuristic_weight_[hidx][2]*calculateHammingDistance(state);
}

SlidingPuzzle::SlidingPuzzleStatePtr SlidingPuzzle::getStartState()
{
  return start_;
}

SlidingPuzzle::SlidingPuzzleStatePtr SlidingPuzzle::getGoalState()
{
  return goal_;
}

} // namespace sliding_puzzle





