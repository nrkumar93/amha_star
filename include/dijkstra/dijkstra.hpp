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
 * \file   dijkstra.hpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   3/27/19
 */

#ifndef PROJECT_DIJKSTRA_HPP
#define PROJECT_DIJKSTRA_HPP

// Project
#include <common/common.hpp>
#include <common/config.h>
#include <common/macros.h>

// STL
#include <unordered_map>
#include <algorithm>


namespace csbpl_planner
{
  
  template<typename Environment,
    typename State,
    typename HeapValueType>
  class Dijkstra
  {
  
  public:
    
    typedef typename Environment::Ptr EnvironmentPtr;
    typedef Vertex<State,
    HeapValueType> SearchVertex;
    typedef typename SearchVertex::Ptr SearchVertexPtr;
    typedef typename State::Ptr StatePtr;
    typedef typename State::Compare StateCompare;
    
    Dijkstra(EnvironmentPtr& env_ptr,
             double time_limit) : env_ptr_(env_ptr)
    {
      params_.time_limit_ = time_limit;
    }
    
    virtual bool run(StatePtr& start_state,
                     StatePtr& goal_state)
    {
      
      start_vertex_ = std::make_shared<SearchVertex>(start_state);
      start_vertex_->g_cost_ = 0;
      start_vertex_->root_id_ = 1;
      start_vertex_->depth_ = 0;
      env_ptr_->explored_[start_state] = start_vertex_;
      open_.insert(start_vertex_);
      open_.decreaseKey(start_vertex_, 0.0);
      
      goal_vertex_ = std::make_shared<SearchVertex>(goal_state);
      env_ptr_->explored_[goal_state] = goal_vertex_;

#if DEBUG
      int num_expansions = 0;
    double nearest_node_to_goal_dist = std::numeric_limits<double>::infinity();
    SearchVertexPtr nearest_node_to_goal;
#endif
      
      while(open_.size() != 0)
      {
        
        SearchVertexPtr least_cost_vertex =
          std::static_pointer_cast<SearchVertex>(open_.remove());
        
        closed_[least_cost_vertex->state_] = true;
        
        if (env_ptr_->terminatePlanning(least_cost_vertex->state_))
        {
          std::cout << "[Dijkstra::run] Goal node expanded. "
                       "Premature termiation!" << std::endl;
          return true;
        }

#if DEBUG
        ++num_expansions;
#endif
        
        std::vector<SearchVertexPtr> least_cost_vertex_successors =
          env_ptr_->getValidSuccessors(least_cost_vertex);
        
        for(const SearchVertexPtr &successor : least_cost_vertex_successors)
        {
#if DEBUG
          bool goal_visited = true;
        for (int i=0; i<successor->state_->state_.size(); ++i)
        {
          for (int j=0; j<successor->state_->state_.size(); ++j)
          {
            if (successor->state_->state_[i][j] != env_ptr_->goal_->state_[i][j])
            {
              goal_visited = false;
            }
          }
        }
        if (goal_visited)
        {
          std::cout << "goal visited" << std::endl;
        }

        if (env_ptr_->getCostToSuccessor(least_cost_vertex->state_, successor->state_) < nearest_node_to_goal_dist)
        {
          nearest_node_to_goal = successor;
        }
#endif
          if(closed_.find(successor->state_) == closed_.end())
          {
            double new_cost = least_cost_vertex->g_cost_ +
                              env_ptr_->getCostToSuccessor(least_cost_vertex->state_, successor->state_);
            
            if (new_cost < successor->g_cost_)
            {
              successor->g_cost_ = new_cost;
              successor->parent_vertex_ = least_cost_vertex;
              successor->root_id_ = least_cost_vertex->root_id_;
              successor->depth_ = successor->parent_vertex_->depth_ + 1;
              
              double f_cost = successor->g_cost_;
              if (successor->heap_index_ == -1)
              {
                open_.insert(successor);
                open_.decreaseKey(successor,
                                  f_cost);
              }
              else
              {
                open_.decreaseKey(successor,
                                  f_cost);
              }
            }
          }
        }
      }

#if DEBUG
      std::cout << "[Dijkstra::run] Total number of expansions: "
              << num_expansions << std::endl;
#endif
      
      
      if (this->open_.size() == 0)
      {
#if DEBUG
        std::cout << "[Dijkstra::run] Open list is empty! Error... "
                   "Returning nearest node reached" << std::endl;
      this->env_ptr_->goal_ = nearest_node_to_goal->state_;
      return true;
#endif
        std::cout << "[Dijkstra::run] Open list is empty! Error... " << std::endl;
        return false;
      }
      
      return true;
    }
    
    std::vector<StatePtr> backtrackPath(const StatePtr &state)
    {
      path_.clear();
      SearchVertexPtr bactrack_vertex = env_ptr_->explored_[state];

//  @TODO Double check if this closed list check is needed or not
//    if (closed_.find(state) != closed_.end())
//    {
      while (bactrack_vertex->parent_vertex_)
      {
        path_.push_back(bactrack_vertex->state_);
        bactrack_vertex = bactrack_vertex->parent_vertex_;
      }
      path_.push_back(bactrack_vertex->state_);
//    }
      
      std::reverse(path_.begin(), path_.end());
      return path_;
    }
    
    virtual PlannerStats<State> getPlannerStats() {}
    
    virtual AnytimePlannerStats<State> getAnytimePlannerStats() {}
  
  protected:
    
    EnvironmentPtr env_ptr_;
    
    Params params_;
    
    SearchVertexPtr start_vertex_;
    SearchVertexPtr goal_vertex_;
    
    csbpl_common::Heap<HeapValueType> open_;
    
    std::unordered_map<StatePtr,
      bool,
      std::hash<StatePtr>,
      StateCompare> closed_;
    
    std::vector<StatePtr> path_;
    
    PlannerStats<State> planner_stats_;
    
  };
  
} // namespace csbpl_planner

#endif //PROJECT_DIJKSTRA_HPP
