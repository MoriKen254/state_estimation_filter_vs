/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : particle_distribution.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "particle_distribution.h"
//
using namespace state_estimate_filter;

/**
 * コピーコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param obj コピー元オブジェクト.
**/
ParticleDistribution::ParticleDistribution(
  ParticleDistribution ^obj)
{
  do
  {
    if (obj == nullptr)
      break;
    VecMean_ = obj->VecMean_;
    MatVariance_ = obj->MatVariance_;
    // particles_ = gcnew List<Particle^>(obj->particles_); // これだと内部のParcileがシャローコピーになってしまう。
    particles_ = gcnew List<Particle^>();
    for (int i = 0; i < obj->Particles_->Count; i++)
      particles_->Add(gcnew Particle(obj->Particles_[i]));

  } while (false);
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  obj コピー元オブジェクト.
**/
ParticleDistribution::ParticleDistribution(
  ParticleDistribution% obj)
{
  VecMean_ = obj.VecMean_;
  MatVariance_ = obj.MatVariance_;
  // particles_ = gcnew List<Particle^>(obj.particles_); // これだと内部のParcileがシャローコピーになってしまう。
  particles_ = gcnew List<Particle^>();
  for (int i = 0; i < obj.Particles_->Count; i++)
    particles_->Add(gcnew Particle(obj.Particles_[i]));
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecMean     期待値ベクトル.
 * @param i_matVariance 共分散行列.
 * @param i_particles   パーティクル群.
**/
ParticleDistribution::ParticleDistribution(
  const cv::Mat i_vecMean,
  const cv::Mat i_matVariance,
  List<Particle^>^ i_particles) :
  ProbabilityDistribution(
  i_vecMean,
  i_matVariance)
{
  //particles_ = gcnew List<Particle^>(i_particles); // これだと内部のParcileがシャローコピーになってしまう。
  particles_ = gcnew List<Particle^>();
  for (int i = 0; i < i_particles->Count; i++)
  {
    Particle^ particle = gcnew Particle(i_particles[i]);
    particles_->Add(particle);
  }
}

/**
 * get particles_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  nullptr if it fails, else a List&lt;Particle^&gt;^.
**/
List<Particle^>^ ParticleDistribution::Particles_::get()
{
  return particles_;
}

/**
 * Sets the given value to particles_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ParticleDistribution::Particles_::set(List<Particle^>^ value)
{
  delete(particles_);
  particles_ = gcnew List<Particle^>(value);
}
