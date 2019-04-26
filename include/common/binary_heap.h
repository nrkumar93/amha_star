/*
 * Copyright (c) 2018, Ramkumar Natarajan
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
 * \file   binary_heap.h
 * \author Ioan Sucan (From OMPL)
 * \maintainer Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   10/7/18
 */

#ifndef CSBPL_COMMON_BINARY_HEAP_H
#define CSBPL_COMMON_BINARY_HEAP_H

#include <vector>
#include <iostream>
#include <memory>

namespace csbpl_common
{

template <typename KeyType>
class Heap
{
public:
  struct Element
  {
    Element(KeyType key = KeyType(), int heap_index = -1)
            : key_(key), heap_index_(heap_index)
    {
    }

    KeyType key_;
    int heap_index_;
  };

  typedef std::shared_ptr<Element> Ptr;

  Heap()
  {
  }

  Heap(int initial_size)
  {
    data_.reserve(initial_size);
  }

  ~Heap()
  {
    clear();
  }

  Ptr remove()
  {
    // Remove the first element in the heap and replace it with the last
    Ptr temp = data_[0];
    data_[0] = data_[size() - 1];
    data_[0]->heap_index_ = 0;
    data_.pop_back();

    // Restore the min-heap property
    heapifyDown(0);

    temp->heap_index_ = -1;
    return temp;
  }

  Ptr top()
  {
    if (data_.empty())
    {
      std::cerr << "Error in top: Heap is empty!" << std::endl;
      return nullptr;
    }

    return data_[0];
  }

  bool decreaseKey(Ptr e, KeyType k)
  {
    // Get the index of the element
    int i = e->heap_index_;

    if (i == -1)
    {
      std::cerr << "Error in decreaseKey: Invalid heap index for heap element" << std::endl;
      return false;
    }

    if (k > data_[i]->key_)
    {
      std::cerr << "Error in decreaseKey: New key must be smaller than existing key" << std::endl;
      return false;
    }

    data_[i]->key_ = k;

    while (i > 0 && data_[getParent(i)]->key_ > data_[i]->key_)
    {
      // Perform a swap of data_[i] with data_[getParent(i)]
      Ptr temp = data_[i];
      data_[i] = data_[getParent(i)];
      data_[i]->heap_index_ = i;
      data_[getParent(i)] = temp;
      data_[getParent(i)]->heap_index_ = getParent(i);

      i = getParent(i);
    }

    return true;
  }

  void insert(Ptr e)
  {
    e->heap_index_ = static_cast<int>(data_.size());
    data_.push_back(e);

    // Percolate up
    int i = size() - 1;
    int parent = getParent(i);
    while (parent != -1 && data_[i]->key_ < data_[parent]->key_)
    {
      // Perform a swap
      Ptr temp = data_[i];
      data_[i] = data_[parent];
      data_[i]->heap_index_ = i;
      data_[parent] = temp;
      data_[parent]->heap_index_ = parent;
      // Continue to percolate up the tree
      i = parent;
      parent = getParent(parent);
    }
  }

  int size() const
  {
    return static_cast<int>(data_.size());
  }

  void clear()
  {
    for (Ptr e : data_)
    {
      if (e)
        e->heap_index_ = -1;
    }

    data_.clear();
  }

private:
  int getParent(int i) const
  {
    return (i == 0 ? -1 : (i - 1) / 2);
  }

  int getLeftChild(int i) const
  {
    return ((2 * i + 1) < size() ? (2 * i + 1) : -1);
  }

  int getRightChild(int i) const
  {
    return ((2 * i + 2) < size() ? (2 * i + 2) : -1);
  }

  void heapifyDown(int i)
  {
    int l = getLeftChild(i);
    int r = getRightChild(i);
    int root;

    if (l != -1 && data_[l]->key_ < data_[i]->key_)
    {
      root = l;
    }
    else root = i;

    if (r != -1 && data_[r]->key_ < data_[root]->key_)
    {
      root = r;
    }

    if (root != i)
    {
      // Swap data_[i] with data_[root]
      Ptr temp = data_[i];
      data_[i] = data_[root];
      data_[i]->heap_index_ = i;
      data_[root] = temp;
      data_[root]->heap_index_ = root;

      heapifyDown(root);
    }
  }

  void heapifyUp(int i)
  {
    int parent = getParent(i);
    if (parent != -1 && data_[i]->key_ < data_[parent]->key_)
    {
      // Perform a swap
      Ptr temp = data_[i];
      data_[i] = data_[parent];
      data_[i]->heap_index_ = i;
      data_[parent] = temp;
      data_[parent]->heap_index_ = parent;

      // Continue to percolate up the tree
      heapifyUp(parent);
    }
  }

  std::vector<Ptr> data_;

};

}

#endif //CSBPL_COMMON_BINARY_HEAP_H
