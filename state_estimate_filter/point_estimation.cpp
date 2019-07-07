/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : point_estimation.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "point_estimation.h"
#include "particle_filter.h"
//
using namespace state_estimate_filter;

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
AbstractPointEstimation::AbstractPointEstimation()
{
}

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
MMSEPointEstimation::MMSEPointEstimation()
{
}

/**
  * パーティクル分布から点推定を行う. 純粋仮想関数.
  *
  * @author  Morita
  * @date  2015/06/17
  *
  * @param i_stateEstimateFilter 状態推定フィルタオブジェクト.
  *
  * @return  点推定値.
**/
cv::Mat MMSEPointEstimation::EstimateFromParticles(
  AbstractStateEstimateFilter^ i_stateEstimateFilter)
{
  // arrange
  const int rows = i_stateEstimateFilter->PriorEstimationCur_->VecMean_.rows;
  const int cols = i_stateEstimateFilter->PriorEstimationCur_->VecMean_.cols;

  // TODO: 要継続チェック
  // ここが汚い。引数が抽象型でパーティクルの有無は問わない方なのに、中で勝手に粒子分布に直している。
  // でも、現状手の打ちようがない。
  // 抽象カルマンフィルタクラスとパーティクルフィルタクラスを一つ間にかませば解決可能ではある。
  // 冗長かもしれないが、正しいかもしれない。しばらく運用して考える。

  // パーティクルフィルタクラスでないなら、点推定いらないよ、という意味を込めて不正値を返す。
  // カルマンフィルタなら勝手に期待値出るっしょ。

  // act
  if (i_stateEstimateFilter->GetType() != ParticleFilter::typeid)
    return cv::Mat::ones(rows, cols, CV_64F) * (-999.);

  //-- ここが汚い。引数が抽象型でパーティクルの有無は問わない方なのに、中で勝手に粒子分布に直している。
  List<Particle^>^ particleList
    = (static_cast<ParticleDistribution^>(i_stateEstimateFilter->PosterioriEstimationCur_)->Particles_);

  cv::Mat mmseResult = cv::Mat::zeros(rows, cols, CV_64F);

  for (int i = 0; i < particleList->Count; i++)
    mmseResult += particleList[i]->Weight_ * particleList[i]->StateDistribution_->VecMean_;

#if MY_DEBUG
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < particleList->Count; i++)
  {
    result[debugIndex++] = particleList[i]->Weight_;
    result[debugIndex++] = particleList[i]->StateDistribution_->VecMean_.at<double>(0, 0);
  }
  result[debugIndex++] = mmseResult.at<double>(0, 0);
#endif

  return mmseResult;
}

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
MaxWeightPointEstimation::MaxWeightPointEstimation()
{
}

/**
 * パーティクル分布から点推定を行う. MMSEのメインルーチン. 
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateFilter 状態推定フィルタオブジェクト.
 *
 * @return  点推定値.
**/
cv::Mat MaxWeightPointEstimation::EstimateFromParticles(
  AbstractStateEstimateFilter^ i_stateEstimateFilter)
{
  // arrange
  //-- 現時刻　リサンプリング前　推定パーティクル群
  List<Particle^>^ priorEstimationCur
    = static_cast<ParticleDistribution^>(i_stateEstimateFilter->PriorEstimationCur_)->Particles_;
  int weightMaxIndex = 0;
  double weightMax = 0;

  // act
  for (int i = 0; i < priorEstimationCur->Count; i++)
  {
    // 最大値更新なし
    if (weightMax > priorEstimationCur[i]->Weight_)
      continue;

    // 最大値更新
    weightMax = priorEstimationCur[i]->Weight_;
    weightMaxIndex = i;
  }
#if MY_DEBUG
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < particleList->Count; i++)
  {
    result[debugIndex++] = particleList[i]->Weight_;
    result[debugIndex++] = particleList[i]->StateDistribution_->VecMean_.at<double>(0, 0);
  }
#endif

  return priorEstimationCur[weightMaxIndex]->StateDistribution_->VecMean_;
}

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
PfMapPointEstimation::PfMapPointEstimation()
{
}

/**
 * パーティクル分布から点推定を行う. MaxWeightのメインルーチン.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateFilter 状態推定フィルタオブジェクト.
 *
 * @return  点推定値.
**/
cv::Mat PfMapPointEstimation::EstimateFromParticles(
  AbstractStateEstimateFilter^ i_stateEstimateFilter)
{
  // arrange
  //-- 現時刻　事後推定パーティクル群
  List<Particle^>^ posterioriCurParticleList
    = static_cast<ParticleDistribution^>(i_stateEstimateFilter->PosterioriEstimationCur_)->Particles_;
  //-- 現時刻　事前推定パーティクル群
  List<Particle^>^ priorCurParticleListWithoutNoise
    = gcnew List<Particle^>(posterioriCurParticleList);

  // メインアルゴリズム
  //-- x(i)[k-1] → x(i)-[k](ノイズ無し)　事前分布を求める
  priorCurParticleListWithoutNoise = this->PredictNextStateWithoutNoise(i_stateEstimateFilter);
  //-- 予測尤度を求める: p(xhat(i)[k] | yhat[1:k-1])
  //-- モンテカルロ積分結果 = Σ{ 事前推定尤度 * 前時刻重み }
  this->CalcLikelihoodsOfXhatFromYOldSum(i_stateEstimateFilter, priorCurParticleListWithoutNoise);
  //-- 対数事後確率を求める
  vector<cv::Mat> posterioriOfXhatFromYCur = CalcPosterioriProbability(i_stateEstimateFilter);
  //-- 事後確率が最大となるパーティクルインデックスを取得する
  int mapMaxIndex = AcquireMapMaxIndex(posterioriOfXhatFromYCur);

  // MAP推定した状態
  return posterioriCurParticleList[mapMaxIndex]->StateDistribution_->VecMean_;
}

/**
 * x(i)[k-1] → x(i)-[k](ノイズ無し)　事前分布を求める.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateFilter 状態推定フィルタ.
 *
 * @return  事前推定後のパーティクル分布.
**/
List<Particle^>^ PfMapPointEstimation::PredictNextStateWithoutNoise(
  AbstractStateEstimateFilter^ i_stateEstimateFilter)
{
  // arrange
  //-- 一時刻前　事後推定パーティクル群
  List<Particle^>^ posterioriOldParticleList
    = static_cast<ParticleDistribution^>(i_stateEstimateFilter->PosterioriEstimationOld_)->Particles_;
  //-- 現時刻　事前推定パーティクル群
  List<Particle^>^ priorCurWithoutNoiseParticleList
    = gcnew List<Particle^>(posterioriOldParticleList);
  //-- パーティクル数
  const int countParticles = posterioriOldParticleList->Count;
  //-- 時刻
  const int time = i_stateEstimateFilter->TimeCur_;

  // act
  for (int i = 0; i < countParticles; i++)
  {
    // x(i)[k-1]
    cv::Mat posterioriStateOld = posterioriOldParticleList[i]->StateDistribution_->VecMean_;
    // 事前分布を求める: x(i)-[k] = f(x(i)[k - 1])(ノイズ無し)
    //-- 状態方程式エイリアス
    ProcessModelEquation^ processModelEauation =
      i_stateEstimateFilter->StateSpaceModel_->ProcessModelEquation_;
    //-- ノイズ無し一段先予測
    priorCurWithoutNoiseParticleList[i]->StateDistribution_->VecMean_ =
      processModelEauation->CalcFunctionWithoutNoise(posterioriStateOld, time);
  }

  return priorCurWithoutNoiseParticleList;
}

/**
 * Calculates the likelihoods of xhat from y coordinate old sum.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateFilter               状態推定フィルタオブジェクト.
 * @param i_priorCurParticleListWithoutNoise  ノイズ無しの事前推定パーティクル群.
**/
void PfMapPointEstimation::CalcLikelihoodsOfXhatFromYOldSum(
  AbstractStateEstimateFilter^ i_stateEstimateFilter
, List<Particle^>^ i_priorCurParticleListWithoutNoise)
{
  // arrange
  //-- 一時刻前　事後推定パーティクル群
  List<Particle^>^ posterioriOldParticleList
    = static_cast<ParticleDistribution^>(i_stateEstimateFilter->PosterioriEstimationOld_)->Particles_;
  //-- 現時刻　事後推定パーティクル群
  List<Particle^>^ posterioriCurParticleList
    = static_cast<ParticleDistribution^>(i_stateEstimateFilter->PosterioriEstimationCur_)->Particles_;
  //-- パーティクル数
  const int countParticles = posterioriCurParticleList->Count;
  //-- 合計値格納メンバ変数初期化
  likelihoodsOfXhatFromYOldSum_->clear();

  // act
  //-- p(y[k] | x(m)[k])
  for (int m = 0; m < countParticles; m++)
  {
    cv::Mat sum = cv::Mat::zeros(1, 1, CV_64F);
    vector<cv::Mat> likelihoodsOfXhatFromXhatm;

    //-- p(x[k] | y[1:k-1])
    for (int i = 0; i < countParticles; i++)
    {
      // Σ_i { p(x(m)[k] | x(i)[k-1]) * w(i)[k-1]}
      // xhat[k]
      //-- 事前推定値(ノイズ無し): xhat-[k] = f(xhat[k])
      cv::Mat priorStateCurWithountNoize_i = i_priorCurParticleListWithoutNoise[i]->StateDistribution_->VecMean_;
      //-- 尤度: p'(xhat(1)[k]|xhat(1)[k-1])
      cv::Mat likelihoodsOfXhatFromXhatmTemp(1, 1, CV_64F, cv::Scalar(0.));
      posterioriCurParticleList[m]->StateDistribution_->CalcDencityFunction(
        priorStateCurWithountNoize_i, &likelihoodsOfXhatFromXhatmTemp);
      //-- 計算結果の格納
      likelihoodsOfXhatFromXhatm.push_back(likelihoodsOfXhatFromXhatmTemp);
    }

    // 各パーティクルの前時刻までの観測から現時刻状態の予測対数尤度(モンテカルロ積分の各項)
    // = 事前推定尤度 x 前時刻重み
    vector<cv::Mat> likelihoodsOfXhatFromYOld;
    cv::Mat likelihoodOfXhatFromYOldSumTemp(1, 1, CV_64F, cv::Scalar(0.));
    for (int i = 0; i < countParticles; i++)
    {
      // log{ p'(xhat(i)[k]|xhat(i)[k-1]) } + log{ w(i)[k-1] }
      likelihoodsOfXhatFromYOld.push_back(likelihoodsOfXhatFromXhatm[i] * posterioriOldParticleList[i]->Weight_);
      likelihoodOfXhatFromYOldSumTemp += likelihoodsOfXhatFromYOld[i];
    }
    likelihoodsOfXhatFromYOldSum_->push_back(likelihoodOfXhatFromYOldSumTemp);

#if 1
    double lxtoxm[3] = {
      likelihoodsOfXhatFromXhatm[0].at<double>(0, 0),
      likelihoodsOfXhatFromXhatm[1].at<double>(0, 0),
      likelihoodsOfXhatFromXhatm[2].at<double>(0, 0) };
    double likelihoodsOfXhatFromYOldResult[3] = {
      likelihoodsOfXhatFromYOld[0].at<double>(0, 0),
      likelihoodsOfXhatFromYOld[1].at<double>(0, 0),
      likelihoodsOfXhatFromYOld[2].at<double>(0, 0) };
    double likelihoodsOfXhatFromYOldSumResult = (*likelihoodsOfXhatFromYOldSum_)[m].at<double>(0, 0);
#endif
  }
}

/**
 * 対数事後確率を求める.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateFilter 状態推定フィルタ.
 *
 * @return  全パーティクルの事後推定確率分布.
**/
vector<cv::Mat> PfMapPointEstimation::CalcPosterioriProbability(
  AbstractStateEstimateFilter^ i_stateEstimateFilter)
{
  // arrange
  //-- パーティクル数
  const int countParticles =
    static_cast<ParticleDistribution^>(i_stateEstimateFilter->PosterioriEstimationCur_)->Particles_->Count;
  //-- 観測尤度：p(y[k] | x(m)[k])
  vector<double> logLikelihoodsPriorStateToObserve =
    static_cast<ParticleFilter^>(i_stateEstimateFilter)->LogLikelihoodsPriorStateToObserve_;

  // act
  //-- 対数事後確率を求める
  vector<cv::Mat> posterioriOfXhatFromYCur;
  for (int i = 0; i < countParticles; i++)
  {
    // 予測対数分布を求める
    cv::Mat logLikelihoodsOfXhatFromYOldSum(1, 1, CV_64F, cv::log((*likelihoodsOfXhatFromYOldSum_)[i].at<double>(0, 0)));
    double logLikelihoodsOfXhatFromYOldSumResult = logLikelihoodsOfXhatFromYOldSum.at<double>(0, 0);
    // log{ p(xhat(1)[k] | y[k]) } = log{ p(y[k] | xhat-(1)[k]) } +log{ { p(xhat(1)[k] | yhat[1:k-1]) } }
    posterioriOfXhatFromYCur.push_back(logLikelihoodsPriorStateToObserve[i] + logLikelihoodsOfXhatFromYOldSum);
  }

#if 1
  double posterioriOfXhatFromYCuresult[3] = {
    posterioriOfXhatFromYCur[0].at<double>(0, 0),
    posterioriOfXhatFromYCur[1].at<double>(0, 0),
    posterioriOfXhatFromYCur[2].at<double>(0, 0) };
#endif

  return posterioriOfXhatFromYCur;
}

/**
 * 事後確率が最大となるパーティクルインデックスを取得する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param posterioriOfXhatFromYCur  事後確率分布.
 *
 * @return  事後確率最大パーティクルインデックス.
**/
int PfMapPointEstimation::AcquireMapMaxIndex(
  vector<cv::Mat> posterioriOfXhatFromYCur)
{
  // arrange
  //-- パーティクル数
  const int countParticles = posterioriOfXhatFromYCur.size();
  //-- 事後確率最大値
  int mapMaxIndex = 0;

  // act
  //-- 事後確率が最大となるパーティクルのインデックス
  double mapMax = posterioriOfXhatFromYCur[0].at<double>(0, 0);  // 初回はただ代入
  for (int i = 1; i < countParticles; i++)
  {
    // Max更新なし
    if (posterioriOfXhatFromYCur[i].at<double>(0, 0) <= mapMax)
      continue;

    // Max更新
    mapMaxIndex = i;
    mapMax = posterioriOfXhatFromYCur[i].at<double>(0, 0);
  }

  return mapMaxIndex;
}

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
KernelMapPointEstimation::KernelMapPointEstimation()
{
}

/**
 * パーティクル分布から点推定を行う. カーネルMAPのメインルーチン.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateFilter 状態推定フィルタオブジェクト.
 *
 * @return  点推定値.
**/
cv::Mat KernelMapPointEstimation::EstimateFromParticles(
  AbstractStateEstimateFilter^ i_stateEstimateFilter)
{
  return cv::Mat::ones(1, 1, CV_64F) * (-999);
}
