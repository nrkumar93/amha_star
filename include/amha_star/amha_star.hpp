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
 * \file   amha_star.hpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   4/1/19
 */

#ifndef PROJECT_AMHA_STAR_HPP
#define PROJECT_AMHA_STAR_HPP

// Project
#include <common/common.hpp>
#include <common/config.h>

#include <dijkstra/dijkstra.hpp>

// STL
#include <unordered_map>

namespace csbpl_planner
{
  
  template<typename Environment,
           typename State,
           typename HeapValueType>
  class AMHAStar : public Dijkstra<Environment, State, HeapValueType>
  {
  
  public:
    
    typedef typename Environment::Ptr EnvironmentPtr;
    typedef Vertex<State,
                   HeapValueType> SearchVertex;
    typedef typename SearchVertex::Ptr SearchVertexPtr;
    typedef typename State::Ptr StatePtr;
    typedef typename State::Compare StateCompare;
    
    AMHAStar(EnvironmentPtr& env_ptr,
            double time_limit,
            double epsilon,
            double mha_epsilon,
            double del_epsilon,
            double del_mha_epsilon,
            int num_inad) : Dijkstra<Environment,
                                     State,
                                     HeapValueType>(env_ptr,
                                                    time_limit)
    {
      this->params_.eps_ = epsilon;
      this->params_.mha_eps_ = mha_epsilon;
      
      this->params_.delta_eps_ = del_epsilon;
      this->params_.delta_mha_eps_ = del_mha_epsilon;
      
      this->params_.num_inad_ = num_inad;
      
      open_inad_.reserve(num_inad);
    }
  
    virtual bool expand(SearchVertexPtr& vertex)
    {
      if (vertex->heap_index_ != -1)
      {
        this->open_.decreaseKey(vertex, -INFINITY);
        this->open_.remove();
        vertex->key_ = INFINITY;
      }
    
      for (int i=0; i<this->params_.num_inad_; ++i)
      {
        if (vertex->mha_copy_[i]->heap_index_ != -1)
        {
          open_inad_[i].decreaseKey(vertex->mha_copy_[i], -INFINITY);
          open_inad_[i].remove();
          vertex->mha_copy_[i]->key_ = INFINITY;
        }
      }
    
      std::vector<SearchVertexPtr> vertex_successors =
        this->env_ptr_->getValidSuccessors(vertex);
    
      for(const SearchVertexPtr &successor : vertex_successors)
      {
        if (successor->mha_copy_.empty())
        {
          for (int i=0; i<this->params_.num_inad_; ++i)
          {
            SearchVertexPtr vertex_copy = std::make_shared<SearchVertex>(successor->state_);
            successor->mha_copy_.push_back(vertex_copy);
            successor->mha_copy_[i]->self_ptr_ = successor;
          }
        }
      
        double new_cost = vertex->g_cost_ +
                          this->env_ptr_->getCostToSuccessor(vertex->state_, successor->state_);
      
        if (new_cost < successor->g_cost_)
        {
          successor->g_cost_ = new_cost;
          successor->parent_vertex_ = vertex;
          successor->root_id_ = vertex->root_id_;
          successor->depth_ = successor->parent_vertex_->depth_ + 1;
  
          if (this->closed_.find(successor->state_) != this->closed_.end())
          {
            incons_.push_back(successor->state_);
            incons_inad_.push_back(successor->state_);
          }
          else
          {
            // Insert/Update in OPEN_0 with admissible heuristic
            double f_cost_ad = successor->g_cost_ +
                               this->params_.eps_*this->env_ptr_->getAdmissibleHeuristicCost(successor->state_);
            if (successor->heap_index_ == -1)
            {
              successor->key_ = INFINITY;
              this->open_.insert(successor);
              this->open_.decreaseKey(successor,
                                      f_cost_ad);
            }
            else
            {
              successor->key_ = INFINITY;
              this->open_.decreaseKey(successor,
                                      f_cost_ad);
            }
  
            if (closed_inad_.find(successor->state_) != closed_inad_.end())
            {
              incons_inad_.push_back(successor->state_);
            }
            else
            {
              for (int i=0; i<this->params_.num_inad_; ++i)
              {
                double f_cost_inad = successor->g_cost_ +
                                     this->params_.eps_*
                                     this->env_ptr_->getInadmissibleHeuristicCost(successor->state_, i);
    
                if (f_cost_inad <= this->params_.mha_eps_*f_cost_ad)
                {
                  if (successor->mha_copy_[i]->heap_index_ == -1)
                  {
                    successor->mha_copy_[i]->key_ = INFINITY;
                    open_inad_[i].insert(successor->mha_copy_[i]);
                    open_inad_[i].decreaseKey(successor->mha_copy_[i],
                                              f_cost_inad);
                  }
                  else
                  {
                    successor->mha_copy_[i]->key_ = INFINITY;
                    open_inad_[i].decreaseKey(successor->mha_copy_[i],
                                              f_cost_inad);
                  }
                }
              }
            }
          }
        }
      }
    }
    
    
    virtual bool improvePath()
    {
#if DEBUG
  int num_expansions = 0;
  double nearest_node_to_goal_dist = std::numeric_limits<double>::infinity();
  SearchVertexPtr nearest_node_to_goal;
#endif
      this->anytime_stats_.num_expansions_.push_back(0);
      std::cout << "Index: " << this->anytime_stats_.num_expansions_.size() << " ";
      while (this->open_.top()->key_ < INFINITY)
      {
  
        double time_elapsed =
          duration_cast<duration<double> >(Clock::now() -
                                           this->params_.start_time_).count();
  
        if (time_elapsed > this->params_.time_limit_)
        {
          std::cout << "[AMHAStar::improvePath] Time limit exceeded... Exiting!" << std::endl;
          return false;
        }
        
        for (int i=0; i<this->params_.num_inad_; ++i)
        {
          if (open_inad_[i].size() > 0 &&
              open_inad_[i].top()->key_ <= this->params_.mha_eps_*this->open_.top()->key_)
          {
            if (this->goal_vertex_->g_cost_ < open_inad_[i].top()->key_)
            {
              std::cout << "Goal node expanded. Premature termiation!" << std::endl;
              return true;
            }
            else
            {
              SearchVertexPtr least_cost_vertex =
                std::static_pointer_cast<SearchVertex>(open_inad_[i].top())->self_ptr_;
              expand(least_cost_vertex);
              closed_inad_[least_cost_vertex->state_] = true;
              ++this->anytime_stats_.num_expansions_.back();
#if DEBUG
              ++num_expansions;
#endif
            }
          }
          else
          {
            if (this->goal_vertex_->g_cost_ < this->open_.top()->key_)
            {
              std::cout << "Goal node expanded. Premature termiation!" << std::endl;
              return true;
            }
            else
            {
              SearchVertexPtr least_cost_vertex =
                std::static_pointer_cast<SearchVertex>(this->open_.top());
              expand(least_cost_vertex);
              this->closed_[least_cost_vertex->state_] = true;
              ++this->anytime_stats_.num_expansions_.back();
#if DEBUG
              ++num_expansions;
#endif
            }
          }
        }
      }

#if DEBUG
      std::cout << "Total number of expansions: " << num_expansions << std::endl;
#endif
  
  
      if (this->open_.size() == 0)
      {
#if DEBUG
        std::cout << "Open list is empty! Error... Returning nearest node reached" << std::endl;
    this->env_ptr_->goal_ = nearest_node_to_goal->state_;
    return true;
#endif
        std::cout << "Open list is empty! Error... " << std::endl;
        return false;
      }
  
      return true;
    }
    
    
    virtual bool run(StatePtr& start_state,
                     StatePtr& goal_state)
    {
      
      this->start_vertex_ = std::make_shared<SearchVertex>(start_state);
      this->start_vertex_->self_ptr_ = this->start_vertex_;
      this->start_vertex_->g_cost_ = 0;
      this->start_vertex_->root_id_ = 1;
      this->start_vertex_->depth_ = 0;
      this->env_ptr_->explored_[start_state] = this->start_vertex_;
      
      this->open_.insert(this->start_vertex_);
      this->open_.decreaseKey(this->start_vertex_, 0.0);
      
      for (int i=0; i<this->params_.num_inad_; ++i)
      {
        // Duplicating vertex
        // @FIXME Why am I using std::move() here?
        SearchVertexPtr vertex_copy = std::make_shared<SearchVertex>(std::move(start_state));
        this->start_vertex_->mha_copy_.push_back(vertex_copy);
        this->start_vertex_->mha_copy_[i]->self_ptr_ = this->start_vertex_;
        
        csbpl_common::Heap<HeapValueType> unit_open_inad;
        open_inad_.push_back(unit_open_inad);
        
        open_inad_[i].insert(this->start_vertex_->mha_copy_[i]);
        open_inad_[i].decreaseKey(this->start_vertex_->mha_copy_[i], 0.0);
      }
      
      this->goal_vertex_ = std::make_shared<SearchVertex>(goal_state);
      this->env_ptr_->explored_[goal_state] = this->goal_vertex_;
      for (int i=0; i<this->params_.num_inad_; ++i)
      {
        SearchVertexPtr vertex_copy = std::make_shared<SearchVertex>(goal_state);
        this->goal_vertex_->mha_copy_.push_back(vertex_copy);
        this->goal_vertex_->mha_copy_[i]->self_ptr_ = this->goal_vertex_;
      }
  
      // Start timer
      this->params_.start_time_ = Clock::now();
  
      // Suboptimality bound on current solution
//      for (const auto& incons_node: incons_)
//      {
//        double f_incons = this->env_ptr_->explored_[incons_node]->g_cost_ +
//          this->params_.eps_*this->env_ptr_->getAdmissibleHeuristicCost(incons_node);
//      }
  
      if (improvePath())
      {
        std::vector<StatePtr> path =
          this->backtrackPath(this->env_ptr_->getGoalState());
        anytime_stats_.solution_paths_.push_back(path);
        anytime_stats_.solution_eps_.push_back(this->params_.eps_*this->params_.mha_eps_);
        double unit_solution_time = duration_cast<duration<double> >(Clock::now() - this->params_.start_time_).count();
        anytime_stats_.solution_time_.push_back(unit_solution_time);
        anytime_stats_.cummulative_time_.push_back(duration_cast<duration<double> >(Clock::now() -
                                                                                    this->params_.start_time_).count());
      }
  
      double time_elapsed =
        duration_cast<duration<double> >(Clock::now() -
                                         this->params_.start_time_).count();
  
      // Updating w1 and w2
      this->params_.eps_ -= this->params_.delta_eps_;
      this->params_.mha_eps_ -= this->params_.delta_mha_eps_;
      
      // Outer loop
      while (this->params_.eps_ >= 1 &&
             this->params_.mha_eps_ >= 1 &&
             time_elapsed < this->params_.time_limit_)
      {
        // OPEN U INCONS and clearing CLOSED
        std::vector<SearchVertexPtr> open_u_incons;
        for (const auto& incons_state: incons_)
        {
          SearchVertexPtr incons_vertex = this->env_ptr_->explored_[incons_state];
          incons_vertex->f_cost_ = incons_vertex->g_cost_ +
                                   this->params_.eps_*this->env_ptr_->getAdmissibleHeuristicCost(incons_state);
          open_u_incons.push_back(incons_vertex);
        }
        while (this->open_.size() > 0)
        {
          SearchVertexPtr open_vertex =
            std::static_pointer_cast<SearchVertex>(this->open_.remove());
          open_vertex->f_cost_ = open_vertex->g_cost_ +
                                 this->params_.eps_*this->env_ptr_->getAdmissibleHeuristicCost(open_vertex->state_);
          open_u_incons.push_back(open_vertex);
        }
    
        for (const SearchVertexPtr& new_open : open_u_incons)
        {
          this->open_.insert(new_open);
          this->open_.decreaseKey(new_open,
                                  new_open->f_cost_);
        }
    
        this->closed_.clear();
  
  
        // OPEN_INAD U INCONS_INAD and clearing CLOSED_INAD
        for (int i=0; i<this->params_.num_inad_; ++i)
        {
          std::vector<SearchVertexPtr> open_inad_u_incons_inad;
          open_inad_u_incons_inad.reserve(open_inad_.size()+incons_inad_.size());
          for (const auto& incons_inad_state: incons_inad_)
          {
            SearchVertexPtr incons_inad_vertex = this->env_ptr_->explored_[incons_inad_state];
            incons_inad_vertex->mha_copy_[i]->f_cost_ = incons_inad_vertex->g_cost_ +
                                          this->params_.eps_*this->env_ptr_->getInadmissibleHeuristicCost(incons_inad_state, i);
            open_inad_u_incons_inad.push_back(incons_inad_vertex->mha_copy_[i]);
          }
          while (open_inad_[i].size() > 0)
          {
            SearchVertexPtr open_inad_vertex =
              std::static_pointer_cast<SearchVertex>(open_inad_[i].remove());
            open_inad_vertex->f_cost_ = open_inad_vertex->self_ptr_->g_cost_ +
                                        this->params_.eps_*this->env_ptr_->getInadmissibleHeuristicCost(open_inad_vertex->state_, i);
            open_inad_u_incons_inad.push_back(open_inad_vertex);
          }
  
          for (const SearchVertexPtr& new_open : open_inad_u_incons_inad)
          {
            open_inad_[i].insert(new_open);
            open_inad_[i].decreaseKey(new_open,
                                    new_open->f_cost_);
          }
        }
  
        closed_inad_.clear();
  
        if (improvePath())
        {
          std::vector<StatePtr> path =
            this->backtrackPath(this->env_ptr_->getGoalState());
          anytime_stats_.solution_paths_.push_back(path);
          anytime_stats_.solution_eps_.push_back(this->params_.eps_*this->params_.mha_eps_);
          double unit_solution_time = anytime_stats_.solution_time_.size()==0?
                                      duration_cast<duration<double> >(Clock::now() - this->params_.start_time_).count():
                                      duration_cast<duration<double> >(Clock::now() - this->params_.start_time_).count() -
                                      time_elapsed;
          anytime_stats_.solution_time_.push_back(unit_solution_time);
          anytime_stats_.cummulative_time_.push_back(duration_cast<duration<double> >(Clock::now() -
                                                                                      this->params_.start_time_).count());
        }
    
        // Epsilon update
        this->params_.eps_ -= this->params_.delta_eps_;
        this->params_.mha_eps_ -= this->params_.delta_mha_eps_;
  
        // Time elapsed update
        time_elapsed =
          duration_cast<duration<double> >(Clock::now() -
                                           this->params_.start_time_).count();
    
      }
  
      if (time_elapsed > this->params_.time_limit_ &&
          anytime_stats_.solution_paths_.size()==0)
      {
        std::cout << "[AMHAStar::run] Time limit exceeded and"
          " no solution found... Exiting!" << std::endl;
        return false;
      }
  
      return true;

    }
  
  
    virtual AnytimePlannerStats<State> getAnytimePlannerStats()
    {
      return anytime_stats_;
    }
  
  protected:
    
    std::vector<csbpl_common::Heap<HeapValueType> > open_inad_;
    
    std::unordered_map<StatePtr,
                       bool,
                       std::hash<StatePtr>,
                       StateCompare> closed_inad_;
  
    std::vector<StatePtr> incons_;
    std::vector<StatePtr> incons_inad_;
  
    AnytimePlannerStats<State> anytime_stats_;
  
  };
  
} // namespace csbpl_planner


#endif //PROJECT_AMHA_STAR_HPP
