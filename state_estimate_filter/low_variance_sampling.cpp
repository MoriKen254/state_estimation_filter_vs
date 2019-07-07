/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : low_variance_sampling.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "low_variance_sampling.h"
//
using namespace state_estimate_filter;

/**
 * リサンプルする. 
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_priorEstimationCur  現時刻事前推定値.
 *
 * @return  リサンプル後のパーティクル分布.
**/
ParticleDistribution^ LowVarianceSampling::Resample(
  ParticleDistribution^ i_priorEstimationCur)
{
  // リサンプリングしようがしまいが、リサンプリング前の状態はそのまま全部コピー
  //-- リサンプリング時は必要な項目だけ更新する
  //delete (PosterioriEstimationCur_);
  ParticleDistribution^ posterioriEstimationCur =
    gcnew ParticleDistribution(static_cast<ParticleDistribution^>(i_priorEstimationCur));

  // リサンプリング判定
  //-- パーティクル数
  int countParticles = static_cast<ParticleDistribution^>(i_priorEstimationCur)->Particles_->Count;
  //-- パーティクル数の逆数。
  double perCountParticles = 1. / countParticles; // 固定値の割算は先にやっておく。
  //-- ESS計算
  double ess = CalcEffectiveSampleSize(i_priorEstimationCur);
  //-- ESSが全パーティクル数の半分未満の場合にリサンプルフラグを立てる。
  bool IsResumpleRequired = (ess < countParticles * threshEss_);
  //-- リサンプル不要なら、終了。
  if (IsResumpleRequired == false)
  {
    posterioriEstimationCur->resampledCurr_ = false;
    return posterioriEstimationCur;
  }

  // リサンプリング処理
  // arrange for resample
  //-- 現時刻事前推定パーティクルエイリアス
  List<Particle^>^ priorParticleListCur = static_cast<ParticleDistribution^>(i_priorEstimationCur)->Particles_;
  //-- 現時刻事後推定パーティクルエイリアス
  List<Particle^>^ posterioriParticleListCur = static_cast<ParticleDistribution^>(posterioriEstimationCur)->Particles_;
  //-- 重みの累積和
  vector<double> cumulativeSumsWeight = CalcCumulutiveSumsWeight(priorParticleListCur);
  //-- リサンプルID
  vector<double> resampleIDs = CalcResampleID(priorParticleListCur);

  // act of resample: リサンプリングメインルーチン。ここでパーティクルの更新や重みの初期化を行う。
  int newIndex = 0;
  for (int i = 0; i < countParticles; i++)
  {
    while (resampleIDs[i] > cumulativeSumsWeight[newIndex])
      newIndex++;
    // パーティクル更新
    posterioriParticleListCur[i] = gcnew Particle(priorParticleListCur[newIndex]);
    // 重みを(1/パーティクル数)で初期化
    posterioriParticleListCur[i]->Weight_ = perCountParticles;
  }
  posterioriEstimationCur->resampledCurr_ = true;
  return posterioriEstimationCur;
}

/**
 * Calculates the effective sample size.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_priorEstimationCur  現時刻事前推定値.
 *
 * @return  The calculated effective sample size.
**/
double LowVarianceSampling::CalcEffectiveSampleSize(
  ParticleDistribution^ i_priorEstimationCur)
{
  // arrange
  //-- 現時刻事前推定パーティクル
  List<Particle^>^ priorParticleListCur = static_cast<ParticleDistribution^>(i_priorEstimationCur)->Particles_;

  // act: Effective Sample Size = 1 / Σ{wi*wi}
  //-- 全部の重みが均等でM, 1つ以外全部重み0だと1
  //-- Effective Sample Size がある値以下の場合にばらつきがなくなってきていると判定し、リサンプルする。
  double essDen = 0.;
  for (int i = 0; i < priorParticleListCur->Count; i++)
  {
    double wi = priorParticleListCur[i]->Weight_;
    essDen += (wi * wi);
  }

  double ess = 1. / essDen;

  return ess;
}

/**
 * Calculates the cumulutive sums weight.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_priorParticleListCur  現時刻事前推定パーティクルリスト.
 *
 * @return  The calculated cumulutive sums weight.
**/
vector<double> LowVarianceSampling::CalcCumulutiveSumsWeight(
  List<Particle^>^ i_priorParticleListCur)
{
  // arrange
  //-- パーティクル数
  int countParticles = i_priorParticleListCur->Count;
  //-- 重みの累積和
  vector<double> cumulativeSums = vector<double>(countParticles, 0.);  // countParticle個の要素を0.で初期化

  // act
  cumulativeSums[0] = i_priorParticleListCur[0]->Weight_; // 最初の要素はそのままコピー
  //-- 2番目以降の要素で累積和
  for (int i = 1; i < countParticles; i++)
    cumulativeSums[i] = cumulativeSums[i - 1] + i_priorParticleListCur[i]->Weight_;

  return cumulativeSums;
}

/**
 * Calculates the cumulutive sums base.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_priorParticleListCur  現時刻事前推定パーティクルリスト.
 *
 * @return  The calculated cumulutive sums base.
**/
vector<double> LowVarianceSampling::CalcCumulutiveSumsBase(
  List<Particle^>^ i_priorParticleListCur)
{
  // arrange
  //-- パーティクル数
  int countParticles = i_priorParticleListCur->Count;
  //-- リサンプルベース累積和
  vector<double> cumulativeSumsBases = vector<double>(countParticles, 0.);  // countParticle個の要素を0.で初期化
  //-- パーティクル数の逆数。
  double perCountParticles = 1. / countParticles; // 固定値の割算は先にやっておく。

  // act
  for (int i = 0; i < countParticles; i++)
    cumulativeSumsBases[i] = perCountParticles * i; // 等間隔累積

  return cumulativeSumsBases;
}

/**
 * Calculates the resample ID.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_priorParticleListCur  現時刻事前推定パーティクルリスト.
 *
 * @return  The calculated resample identifier.
**/
vector<double> LowVarianceSampling::CalcResampleID(
  List<Particle^>^ i_priorParticleListCur)
{
  // arrange
  //-- パーティクル数
  int countParticles = i_priorParticleListCur->Count;
  //--　パーティクル数の逆数。
  double perCountParticles = 1. / countParticles; // 定数の割算は先にやっておく。
  //-- 等間隔(1/パーティクル数)累積和を計算
  vector<double> cumulativeSumsBases = CalcCumulutiveSumsBase(i_priorParticleListCur);
  //-- 等間隔累積和に乱数を加算する。乱数は[0,1/パーティクル数)である。
  vector<double> resampleIDs = vector<double>(countParticles, 0.);

  // act
  for (int i = 0; i < countParticles; i++)
  {
    double random = (double)rand() / RAND_MAX;
    // 等間隔累積に乱数を加える
    resampleIDs[i] = cumulativeSumsBases[i] + random * perCountParticles;
  }

  return resampleIDs;
}
