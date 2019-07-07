/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : particle.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "particle.h"
//
using namespace state_estimate_filter;

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
Particle::Particle()
{
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  obj The object.
**/
Particle::Particle(
  Particle% obj)
{
  stateDistribution_ = gcnew NormalDistribution(obj.stateDistribution_);
  weight_ = obj.weight_;
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param obj The object.
**/
Particle::Particle(
  Particle^ obj)
{
  stateDistribution_ = gcnew NormalDistribution(obj->stateDistribution_);
  weight_ = obj->weight_;
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateDistribution_  Zero-based index of the state distribution.
 * @param i_weight              Zero-based index of the weight.
**/
Particle::Particle(
  NormalDistribution^ i_stateDistribution_,
  const double i_weight)
{
  stateDistribution_ = gcnew NormalDistribution(i_stateDistribution_);
  weight_ = i_weight;
}

/**
* 代入演算子.
*
* @author  Morita
* @date  2015/06/17
*
* @param [in,out]  rhs 代入元オブジェクト.
*
* @return  代入元オブジェクト.
**/
Particle Particle::operator = (Particle% rhs)
{
  // コピーを返す
  return Particle(gcnew Particle(rhs));
}

/**
 * Gets the stateDistribution_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  nullptr if it fails, else a stateDistribution_^.
**/
NormalDistribution^ Particle::StateDistribution_::get()
{
  return stateDistribution_;
}

/**
 * Sets the given value to stateDistribution_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void Particle::StateDistribution_::set(
  NormalDistribution^ value)
{
  delete(stateDistribution_);
  stateDistribution_ = gcnew NormalDistribution(value);
}

/**
 * Gets the weight_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A weight_.
**/
double Particle::Weight_::get()
{
  return weight_;
}

/**
 * Sets the given value to weight_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void Particle::Weight_::set(
  const double value)
{
  weight_ = value;
}
