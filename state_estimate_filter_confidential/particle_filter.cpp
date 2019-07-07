/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : particle_filter.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "particle_filter.h"
#include "state_estimater_factory.h"
#include "low_variance_sampling.h"  // 実装側では継承クラスを宣言(デフォルト用)
#include "truth_observation_generator.h"
//
using namespace state_estimate_filter;

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
ParticleFilter::ParticleFilter() :
AbstractStateEstimateFilter()
{
  ConfigureDefaultParameters();
  //InitializeParameters();
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel 状態推定用モデル方程式一式
 *
 * ### remarks  Morita, 2015/06/17.
**/
ParticleFilter::ParticleFilter(
  StateSpaceModel^ i_stateSpaceModel)
: AbstractStateEstimateFilter(
    i_stateSpaceModel)
{
  ConfigureDefaultParameters();
  InitializeParameters(i_stateSpaceModel, nullptr, nullptr);
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_processModelEquation      プロセスモデル方程式.
 * @param i_observationModelEquation  観測モデル方程式.
 * @param i_vecStateTrueInit          初期状態真値.
**/
ParticleFilter::ParticleFilter(
  ProcessModelEquation^ i_processModelEquation
, List<ObservationModelEquation^>^ i_observationModelEquation
, const cv::Mat i_vecStateTrueInit)
: AbstractStateEstimateFilter(
    i_processModelEquation
  , i_observationModelEquation
  , i_vecStateTrueInit)
{
  ConfigureDefaultParameters();

  // 非線形システムモデル
  NonLinearStateSpaceModel ^nonlinearStateSpaceModel
    = gcnew NonLinearStateSpaceModel(
      i_processModelEquation
    , i_observationModelEquation
    , i_vecStateTrueInit);

  InitializeParameters(nonlinearStateSpaceModel, nullptr, nullptr);
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel           状態推定用モデル方程式一式.
 * @param i_posterioriEstimationInit  事後推定値初期値.
**/
ParticleFilter::ParticleFilter(
  StateSpaceModel ^i_stateSpaceModel
, ParticleDistribution^ i_posterioriEstimationInit)
: AbstractStateEstimateFilter(
    i_stateSpaceModel)
{
  PosterioriEstimationOld_ = gcnew ParticleDistribution(i_posterioriEstimationInit);
  ConfigureDefaultParameters();
  InitializeParameters(i_stateSpaceModel, nullptr, nullptr);

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = PosterioriEstimationOld_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->MatVariance_.at<double>(0, 0);
#endif
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel           状態推定用モデル方程式一式.
 * @param i_posterioriEstimationInit  事後推定値初期値.
 * @param i_pointEstimation           指定する点推定アルゴリズム.
**/
ParticleFilter::ParticleFilter(
  StateSpaceModel ^i_stateSpaceModel,
  ProbabilityDistribution^ i_distRandomPredict,
  ParticleDistribution^ i_posterioriEstimationInit,
  AbstractPointEstimation^ i_pointEstimation,
  AbstractResampling^ i_resampling)
  : AbstractStateEstimateFilter(i_stateSpaceModel)
  , distRandomPredict_(i_distRandomPredict)
{
  PosterioriEstimationOld_ = gcnew ParticleDistribution(i_posterioriEstimationInit);
  ConfigureDefaultParameters();
  InitializeParameters(i_stateSpaceModel, i_pointEstimation, i_resampling);
}

/**
* 一段先予測. 運動モデルに従って予測する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecInputCur 現在推定値.
**/
void ParticleFilter::Predict(
  const cv::Mat i_vecInputCur)
{
  // 名前が長くなるのでエイリアス。パーティクルリストのポインタをコピー。
  List<Particle^>^ posteriorParticleListOld = static_cast<ParticleDistribution^>(PosterioriEstimationOld_)->Particles_;
  List<Particle^>^ priorParticleListCur = static_cast<ParticleDistribution^>(PriorEstimationCur_)->Particles_;

  for (int i = 0; i < priorParticleListCur->Count; i++) // パーティクル(空間方向)でループ
  {
    // arrange
    const cv::Mat vecMeanPosterioriPredictedOld =
      posteriorParticleListOld[i]->StateDistribution_->VecMean_.clone(); // 一時刻前事後分布
    cv::Mat vecMeanPriorPredictedCur =
      priorParticleListCur[i]->StateDistribution_->VecMean_.clone(); // 

    // act
    //--状態: xhat[k] = f(x[k-1]) + b*(u[k]) + v[k](全パーティクルの事前予測結果にノイズを追加)
#if 0
    StateSpaceModel_->PredictPriorStateCurWithNoise(vecMeanPosterioriPredictedOld, i_vecInputCur, TimeCur_, &vecMeanPriorPredictedCur);
    priorParticleListCur[i]->StateDistribution_->VecMean_ = vecMeanPriorPredictedCur;
#endif
    priorParticleListCur[i]->StateDistribution_->VecMean_ = 
      StateSpaceModel_->PredictPriorStateCurWithoutNoise(vecMeanPosterioriPredictedOld, i_vecInputCur, TimeCur_) + distRandomPredict_->GenRandomValue();
  }

  // debug
#if 1
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < priorParticleListCur->Count; i++)
  {
    result[debugIndex++] = posteriorParticleListOld[i]->StateDistribution_->VecMean_.at<double>(0, 0);
    // 初回はノイズなしだと2.8988620358133890(真値)。これと違う値ならちゃんとノイズが乗っている。
    result[debugIndex++] = priorParticleListCur[i]->StateDistribution_->VecMean_.at<double>(0, 0);
  }
  result[debugIndex++] = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);
  for (int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0);
#endif
};

/**
* フィルタリング. 観測の結果を考慮する.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void ParticleFilter::Filter()
{
  int countSensor = StateSpaceModel_->ObservationModelEquations_->Count;
  List<Particle^>^ priorParticleListCurAllSensor = (static_cast<ParticleDistribution^>(PriorEstimationCur_))->Particles_;
  int countParticle = priorParticleListCurAllSensor->Count;
  vector<double> likelihoodsPriorStateToObserveSum(countParticle, 0.);
  vector<double> logWeightAllTemp(countParticle, 0.); // 要素数: countParticle, 初期値 0
  
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    (*logLikelihoodsPriorStateToObserve_)[indexParticle] = 0;

  // センサ毎に尤度を計算する
  for (int indexSensor = 0; indexSensor < countSensor; indexSensor++)
  {
    vector<double> logWeightSingle;

    // 尤度計算
    logWeightSingle = CalcLogLikelifood(indexSensor); // 対数尤度
    
    // 通常尤度で足し算
    for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    {
#if 1
      double weight = exp(logWeightSingle[indexParticle]);
#endif
      likelihoodsPriorStateToObserveSum[indexParticle] += exp(logWeightSingle[indexParticle]);
    }
#if 0
    // CalcLikelifood(); 
    // CalcWeight();
#endif
  }
  // 対数尤度に戻す
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    (*logLikelihoodsPriorStateToObserve_)[indexParticle] = log(likelihoodsPriorStateToObserveSum[indexParticle]);

  // 重み計算
  logWeightAllTemp = CalcLogWeight();  // 正規化後重み from 対数尤度

  // 
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
  {
    priorParticleListCurAllSensor[indexParticle]->Weight_ = logWeightAllTemp[indexParticle];
  }

  // リサンプリング
  PosterioriEstimationCur_ = resampling_->Resample(static_cast<ParticleDistribution^>(PriorEstimationCur_));
  
#if 0
  // TODO: 尤度比検定
  //-- 一時刻前事後推定値
  cv::Mat vecMeanOld = PosterioriEstimationOld_->VecMean_;
  double meanOld = vecMeanOld.at<double>(0, 0);
  //-- 現時刻予測状態値 from 一時刻前事後推定値
  cv::Mat vecMeanStatePredictedFromOld = StateSpaceModel_->ProcessModelEquation_->CalcFunctionWithoutNoise(vecMeanOld, TimeCur_);
  double statePredictedFromOld = vecMeanStatePredictedFromOld.at<double>(0, 0);
  //-- 現時刻予測観測値 の対数尤度を求める
  vector<double> vecMeansObservePredictedFromStatePredicted;
  vector<double> observations;
  vector<double> logLikelihoods;
  for (int indexSensor = 0; indexSensor < countSensor; indexSensor++)
  {
    //-- 現時刻予測観測値 from 現時刻予測状態値
    cv::Mat predicted = StateSpaceModel_->ObservationModelEquations_[indexSensor]->CalcFunctionWithoutNoise(vecMeanOld, TimeCur_);
    vecMeansObservePredictedFromStatePredicted.push_back(predicted.at<double>(0, 0));

    //-- 現時刻予測観測値 対数尤度
    cv::Mat logLikelihood;
    StateSpaceModel_->ObservationModelEquations_[indexSensor]->observationCur_->CalcLogDencityFunction(predicted, &logLikelihood);
    logLikelihoods.push_back(logLikelihood.at<double>(0, 0));

    // debug
    observations.push_back(StateSpaceModel_->ObservationModelEquations_[indexSensor]->observationCur_->VecMean_.at<double>(0, 0));
  }
  // 対数尤度が最大の値を持つセンサを、最も信頼できるセンサと仮定する
  //-- 尤度が最大となるセンサのインデックス
  int indexSensorMaxLikelihood = 0;
  //-- 最大尤度
  double logLiklihoodMax = logLikelihoods[0];

  for (int indexSensor = 1; indexSensor < countSensor; indexSensor++)
  {
    logLiklihoodMax = logLiklihoodMax > logLikelihoods[indexSensor] ? logLiklihoodMax : logLikelihoods[indexSensor];
    if (logLiklihoodMax == logLikelihoods[indexSensor])
      indexSensorMaxLikelihood = indexSensor;
  }
//  delete[] vecMeansObservePredictedFromStatePredicted;
//  delete[] logLikelihoods;


  // センサが二つ以上あれば以下に
  if (observations.size() == 1)
    return;

  // 帰無仮説: 尤度の低いセンサの分布が、尤度の高いセンサの分布と同一だと仮定した場合の、全粒子の尤度の和を計算
  likelihoodsH0_ = 0;
  // 尤度計算
  logLikelihoods = CalcLogLikelifoodAfterResample(indexSensorMaxLikelihood); // 対数尤度
  // 対数尤度で足し算
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    likelihoodsH0_ += logLikelihoods[indexParticle];

  // 対立仮説: 尤度の低いセンサの分布と、尤度の高いセンサの分布が異なると仮定した場合の、全粒子の尤度の和を計算
  // センサ毎に尤度を計算する
  likelihoodsH1_ = 0;
  double aveObservation = (observations[0] + observations[1]) / 2;
  double threshMin[2] = { aveObservation, -99999.0 };
  double threshMax[2] = { 99999, aveObservation };
  logLikelihoods.clear();
  logLikelihoods.resize(countParticle);
  for (int indexSensor = 0; indexSensor < countSensor; indexSensor++)
  {
    // arrange
    const int countParticles = static_cast<ParticleDistribution^>(PriorEstimationCur_)->Particles_->Count;
    //vector<double> logLikelihoods(countParticles, 0.);

    // act
    //-- リサンプリング後観測推定値を求める
    CalcPosterioriPredictionObserveCurAfterResample(indexSensor);
    //-- 事前観測推定値から対数尤度を求める
    for (int indexParticle = 0; indexParticle < countParticles; indexParticle++)
    {
      // 観測予測値が範囲を超えていたら次のインデックスへ
      double predictionObserveCur = (*vecPosterioriPredictionObserveCur_)[indexParticle].at<double>(0, 0);
      if (predictionObserveCur < threshMin[indexSensor] ||
        predictionObserveCur >= threshMax[indexSensor])
        continue;

      cv::Mat logLikelihoodTemp = cv::Mat::zeros(1, 1, CV_64F);
      // y[k]に対するyhat-[k]の対数尤度を計算する。
      StateSpaceModel_->ObservationModelEquations_[indexSensor]->observationCur_->CalcLogDencityFunction((*vecPosterioriPredictionObserveCur_)[indexParticle], &logLikelihoodTemp);
      //(*logLikelihoodsPriorStateToObserve_)[indexParticle] = logLikelihoodTemp.at<double>(0, 0);
      logLikelihoods[indexParticle] = logLikelihoodTemp.at<double>(0, 0);

      // 対数尤度で足し算
      likelihoodsH1_ += logLikelihoods[indexParticle];
    }
  }
  deviance_ = likelihoodsH1_ - likelihoodsH0_;  // 逸脱度
  stateObserve_ = 0;

  if (deviance_ > 300/*3.84*/)  // センサ壊れている
    stateObserve_ = 1;

  if (stateObserve_ == 0) // センサが壊れていなければ、分散を元の値に戻して終了
  {
    StateSpaceModel_->ObservationModelEquations_[1]->observationCur_->MatVariance_.at<double>(0, 0) =
      StateSpaceModel_->ObservationModelEquations_[1]->matVarianceObservationOri_->at<double>(0, 0);
    return;
  }

  // センサがおかしいと判断したら、そのセンサの分散を大きくする
  //-- 逸脱度は高々1000. 分散は高々2Vまで広げるようにする. 
  const double devianceMax = 1000.;
  //-- 更新前の分散取得
  double varSensorBrokenBeforeUpdate = 
    StateSpaceModel_->ObservationModelEquations_[1]->matVarianceObservationOri_->at<double>(0, 0);
  //-- 逸脱度計算
  double devianceTemp = deviance_ < devianceMax ? deviance_ : devianceMax;  // 高々1000
  //-- 更新後分散
  double varSensorBrokenAfterUpdate = varSensorBrokenBeforeUpdate * (1 + deviance_ / devianceMax); // 高々2V
  StateSpaceModel_->ObservationModelEquations_[1]->observationCur_->MatVariance_.at<double>(0, 0)
    = varSensorBrokenAfterUpdate;
#endif

#if 1 // 故障検出アルゴリズム SEED
  if (countSensor < 2)
    return;

  // TODO: 尤度比検定
  // リサンプリング後　再重み付け前結果格納: 表示用なので消しても良い。
  delete posterioriEstimationBeforeResample_;
  posterioriEstimationBeforeResample_ = new vector<double>(countParticle, 0);
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    (*posterioriEstimationBeforeResample_)[indexParticle] = priorParticleListCurAllSensor[indexParticle]->Weight_;
  
  //-- 一時刻前事後推定値
  cv::Mat vecMeanOld = PosterioriEstimationOld_->VecMean_;
  double meanOld = vecMeanOld.at<double>(0, 0);
  //-- 現時刻予測状態値 from 一時刻前事後推定値
  cv::Mat vecMeanStatePredictedFromOld = StateSpaceModel_->ProcessModelEquation_->CalcFunctionWithoutNoise(vecMeanOld, TimeCur_);
  double statePredictedFromOld = vecMeanStatePredictedFromOld.at<double>(0, 0);
  //-- 現時刻予測観測値 の対数尤度を求める
  vector<double> vecMeansObservePredictedFromStatePredicted;
  vector<double> observations;
  vector<double> logLikelihoods;
  for (int indexSensor = 0; indexSensor < countSensor; indexSensor++)
  {
    //-- 現時刻予測観測値 from 現時刻予測状態値
    cv::Mat predicted = StateSpaceModel_->ObservationModelEquations_[indexSensor]->CalcFunctionWithoutNoise(vecMeanOld, TimeCur_);
    vecMeansObservePredictedFromStatePredicted.push_back(predicted.at<double>(0, 0));

    //-- 現時刻予測観測値 対数尤度
    cv::Mat logLikelihood;
    StateSpaceModel_->ObservationModelEquations_[indexSensor]->observationCur_->CalcLogDencityFunction(predicted, &logLikelihood);
    logLikelihoods.push_back(logLikelihood.at<double>(0, 0));

    // debug
    observations.push_back(StateSpaceModel_->ObservationModelEquations_[indexSensor]->observationCur_->VecMean_.at<double>(0, 0));
  }
  // 対数尤度が最大の値を持つセンサを、最も信頼できるセンサと仮定する
  //-- 尤度が最大となるセンサのインデックス
  int indexSensorMaxLikelihood = 0;
  //-- 最大尤度
  double logLiklihoodMax = logLikelihoods[0];

  for (int indexSensor = 1; indexSensor < countSensor; indexSensor++)
  {
    logLiklihoodMax = logLiklihoodMax > logLikelihoods[indexSensor] ? logLiklihoodMax : logLikelihoods[indexSensor];
    if (logLiklihoodMax == logLikelihoods[indexSensor])
      indexSensorMaxLikelihood = indexSensor;
  }
  //  delete[] vecMeansObservePredictedFromStatePredicted;
  //  delete[] logLikelihoods;


  // センサが二つ以上あれば以下に
  if (observations.size() == 1)
    return;

  // 帰無仮説: 尤度の低いセンサの分布が、尤度の高いセンサの分布と同一だと仮定した場合の、全粒子の尤度の和を計算
  likelihoodsH0_ = 0;
  // 尤度計算
  logLikelihoods = CalcLogLikelifoodAfterResample(indexSensorMaxLikelihood); // 対数尤度
  // 対数尤度で足し算
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    likelihoodsH0_ += logLikelihoods[indexParticle];

  // 対立仮説: 尤度の低いセンサの分布と、尤度の高いセンサの分布が異なると仮定した場合の、全粒子の尤度の和を計算
  // センサ毎に尤度を計算する
  likelihoodsH1_ = 0;
  double aveObservation = (observations[0] + observations[1]) / 2;
  double threshMin[2] = { aveObservation, -99999.0 };
  double threshMax[2] = { 99999, aveObservation };
  logLikelihoods.clear();
  logLikelihoods.resize(countParticle);
  for (int indexSensor = 0; indexSensor < countSensor; indexSensor++)
  {
    // arrange
    const int countParticles = static_cast<ParticleDistribution^>(PriorEstimationCur_)->Particles_->Count;
    //vector<double> logLikelihoods(countParticles, 0.);

    // act
    //-- リサンプリング後観測推定値を求める
    CalcPosterioriPredictionObserveCurAfterResample(indexSensor);
    //-- 事前観測推定値から対数尤度を求める
    for (int indexParticle = 0; indexParticle < countParticles; indexParticle++)
    {
      // 観測予測値が範囲を超えていたら次のインデックスへ
      double predictionObserveCur = (*vecPosterioriPredictionObserveCur_)[indexParticle].at<double>(0, 0);
      if (predictionObserveCur < threshMin[indexSensor] ||
        predictionObserveCur >= threshMax[indexSensor])
        continue;

      cv::Mat logLikelihoodTemp = cv::Mat::zeros(1, 1, CV_64F);
      // y[k]に対するyhat-[k]の対数尤度を計算する。
      StateSpaceModel_->ObservationModelEquations_[indexSensor]->observationCur_->CalcLogDencityFunction((*vecPosterioriPredictionObserveCur_)[indexParticle], &logLikelihoodTemp);
      //(*logLikelihoodsPriorStateToObserve_)[indexParticle] = logLikelihoodTemp.at<double>(0, 0);
      logLikelihoods[indexParticle] = logLikelihoodTemp.at<double>(0, 0);

      // 対数尤度で足し算
      likelihoodsH1_ += logLikelihoods[indexParticle];
    }
  }
  deviance_ = likelihoodsH1_ - likelihoodsH0_;  // 逸脱度
  stateObserve_ = 0;

  if (deviance_ > 300/*3.84*/)  // センサ壊れている
    stateObserve_ = 1;

  if (stateObserve_ == 0) // センサが壊れていなければ、分散を元の値に戻して終了
  {
    StateSpaceModel_->ObservationModelEquations_[1]->observationCur_->MatVariance_.at<double>(0, 0) =
      StateSpaceModel_->ObservationModelEquations_[1]->matVarianceObservationOri_->at<double>(0, 0);
    return;
  }

  // センサがおかしいと判断したら、そのセンサの分散を大きくする
  //-- 逸脱度は高々1000. 分散は高々3Vまで広げるようにする. 
  const double devianceMax = 1000.;
  //-- 更新前の分散取得
  double varSensorBrokenBeforeUpdate =
    StateSpaceModel_->ObservationModelEquations_[1]->matVarianceObservationOri_->at<double>(0, 0);
  //-- 逸脱度計算
  double devianceTemp = deviance_ < devianceMax ? deviance_ : devianceMax;  // 高々1000
#if 0
  //-- 更新後分散
  double varSensorBrokenAfterUpdate = varSensorBrokenBeforeUpdate * (1 + 8 * deviance_ / devianceMax); // 高々9V
  StateSpaceModel_->ObservationModelEquations_[1]->observationCur_->MatVariance_.at<double>(0, 0)
    = varSensorBrokenAfterUpdate;
#endif
#if 0 // 重み再計算1(pfMAP向け)
  vector<double> likelihoodsPosterioriStateToObserveSumAfterVarUpdate(countParticle, 0);
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    (*logLikelihoodsPriorStateToObserve_)[indexParticle] = 0;

  // センサ毎に尤度を計算する
  for (int indexSensor = 0; indexSensor < countSensor; indexSensor++)
  {
    vector<double> logWeightSingle;

    // 尤度計算
    logWeightSingle = CalcLogLikelifoodAfterResample(indexSensor); // 対数尤度

    // 通常尤度で足し算
    for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    {
      likelihoodsPosterioriStateToObserveSumAfterVarUpdate[indexParticle] += exp(logWeightSingle[indexParticle]);
    }
  }
  // 対数尤度に戻す
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    (*logLikelihoodsPriorStateToObserve_)[indexParticle] = log(likelihoodsPosterioriStateToObserveSumAfterVarUpdate[indexParticle]);

  // 重み計算
  logWeightAllTemp = CalcLogWeight();  // 正規化後重み from 対数尤度

  // 
  List<Particle^>^ posterioriParticleListCurAllSensor = (static_cast<ParticleDistribution^>(PosterioriEstimationCur_))->Particles_;
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
  {
    posterioriParticleListCurAllSensor[indexParticle]->Weight_ = logWeightAllTemp[indexParticle];
  }
#endif  // 重み再計算1(pfMAP向け)
#if 0 // 重み再計算2(MMSE向け)
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
  {
    (*logLikelihoodsPriorStateToObserve_)[indexParticle] = 0;
    likelihoodsPriorStateToObserveSum[indexParticle] = 0;
  }
  int indexSensorBroken = 1;
  // センサ毎に尤度を計算する
  for (int indexSensor = 0; indexSensor < countSensor; indexSensor++)
  {
    // 壊れたセンサ分はスキップ
    if (indexSensor == indexSensorBroken)
      continue;

    vector<double> logWeightSingle;

    // 尤度計算
    logWeightSingle = CalcLogLikelifood(indexSensor); // 対数尤度

    // 通常尤度で足し算
    for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    {
      likelihoodsPriorStateToObserveSum[indexParticle] += exp(logWeightSingle[indexParticle]);
    }
  }
  // 対数尤度に戻す
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
    (*logLikelihoodsPriorStateToObserve_)[indexParticle] = log(likelihoodsPriorStateToObserveSum[indexParticle]);

  // 重み計算
  logWeightAllTemp = CalcLogWeight();  // 正規化後重み from 対数尤度

  // 
  for (int indexParticle = 0; indexParticle < countParticle; indexParticle++)
  {
    priorParticleListCurAllSensor[indexParticle]->Weight_ = logWeightAllTemp[indexParticle];
  }
#endif // 重み再計算2(MMSE向け)

#endif
}

/**
 * MAP推定をする.ここでvecStateEstimatedCurを更新する. MMSE, MAP等を適応的に選ぶ.
 * 循環参照の問題があり基底クラスでAbstractPointEstimationは使えない.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_pointEstimation           指定する点推定アルゴリズム.
**/
void ParticleFilter::EstimateMAP(
  AbstractPointEstimation^ i_pointEstimation)
{
  if (i_pointEstimation == nullptr)  // nullptr指定の場合は、デフォルトのアルゴリズムを使用する
  {
    if (pointEstimation_ == nullptr)  // メンバがnullptrの場合はデフォを確保してあげる。
      pointEstimation_ = gcnew MMSEPointEstimation();
  }
  else
  {
    pointEstimation_ = i_pointEstimation;
  }
  // 引数で推定アルゴリズムを指定されたら、それを使う
  PosterioriEstimationCur_->VecMean_ = pointEstimation_->EstimateFromParticles(this);
}

/**
 * シミュレーションを行う.
 * 循環参照の問題があり基底クラスでAbstractPointEstimationは使えない
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_count             ループ回数.
 * @param i_estimationResult  状態推定結果.
**/
void ParticleFilter::Loop(
  const int i_count
, EstimationResult ^i_estimationResult)
{
  // 入力を常にゼロにするシミュレーション
  cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

  TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
  List<TruthAndObservation^>^ truthAndObservations =
    truthAndObservationGenerator->Generate(this->StateSpaceModel_, i_count + 1);

  // パーティクルフィルタ開始。時刻k=1からk=4まで実施   
  for (int k = 0; k < i_count; k++)
  {
    this->TimeCur_ = k + 1;
    // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
    // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
    //this->UpdateToNextTime(vecInputCur);
    this->UpdateToNextTimeFromOuterValue(truthAndObservations[k]->truth_, truthAndObservations[k]->observations_);
    this->Estimate(vecInputCur); // 入力値を元に推定
    this->EstimateMAP(pointEstimation_); // MAP推定
    this->AcquireEstimationResult(i_estimationResult);  // 結果を引数オブジェクトに格納
  }
#if 0
  String^ fileName = gcnew String("particle_filter_result.xml");
  i_estimationResult->Save(fileName);

  EstimationResult estimationResult;
  estimationResult.Load(fileName);
  estimationResult.Save(fileName);
#endif
#if MY_DEBUG
  vector<double> truths(truthAndObservations->Count);
  vector<double> observations(truthAndObservations->Count);
  for (int i = 0; i < truthAndObservations->Count; i++)
  {
    truths[i] = truthAndObservations[i]->truth_[0];
    observations[i] = truthAndObservations[i]->observation_[0];
  }
#endif
}

/**
 * 外部から与えられた真値と観測値を元にシミュレーションを行う.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_truthAndObservations  真値と推定値一式.格納された個数回シミュレーションする.
 *
 * @return  シミュレーション結果.
**/
EstimationResult^ ParticleFilter::Loop(
  List<TruthAndObservation^>^ i_truthAndObservations)
{
  const int countTime = i_truthAndObservations->Count;
  // 入力を常にゼロにするシミュレーション
  cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

  EstimationResult^ estimationResult = gcnew EstimationResult;

  // パーティクルフィルタ開始。時刻k=1からk=4まで実施   
  for (int k = 0; k < countTime; k++)
  {
    this->TimeCur_ = k + 1;
    // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
    // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
    //this->UpdateToNextTime(vecInputCur);
    this->UpdateToNextTimeFromOuterValue(i_truthAndObservations[k]->truth_, i_truthAndObservations[k]->observations_);
    this->Estimate(vecInputCur); // 入力値を元に推定
    this->EstimateMAP(pointEstimation_); // MAP推定
    this->AcquireEstimationResult(estimationResult);  // 結果を引数オブジェクトに格納
  }

  return estimationResult;
}

/**
 * 内部変数を次の時刻のものに更新する. 真値, 観測値等.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecInput  制御入力.
**/
void ParticleFilter::UpdateToNextTime(
  const cv::Mat i_vecInput)
{
  // 現時刻事後推定値　→　一時刻前事後推定値にシフト
  PosterioriEstimationOld_ = gcnew ParticleDistribution(static_cast<ParticleDistribution^>(PosterioriEstimationCur_));
  // パーティクル
  PosterioriEstimationOld_ = gcnew ParticleDistribution(static_cast<ParticleDistribution^>(PosterioriEstimationCur_));
  // 真値更新(状態空間モデル内の真値を更新する)
  StateSpaceModel_->UpdateInnerTrueValueWithNoise(i_vecInput, TimeCur_);
  // 観測値更新(状態空間モデル内の観測値を更新する)
  StateSpaceModel_->ObserveWithNoise(StateSpaceModel_->VecStateTrueCur_, TimeCur_);  // 観測

  // debug
#if 1
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = PosterioriEstimationCur_->VecMean_.at<double>(0, 0); // 一時刻シフト
  result[debugIndex++] = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);  // 真値
  for(int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0);  // 観測
#endif
}

/**
 * 真値, 観測値を外部変数によって更新する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_truth       真値.
 * @param i_observation 観測値.
**/
void ParticleFilter::UpdateToNextTimeFromOuterValue(
  double i_truth,
  List<double>^ i_observations)
{
  // 現時刻事後推定値　→　一時刻前事後推定値にシフト
  PosterioriEstimationOld_ = gcnew ParticleDistribution(static_cast<ParticleDistribution^>(PosterioriEstimationCur_));
  // パーティクル
  PosterioriEstimationOld_ = gcnew ParticleDistribution(static_cast<ParticleDistribution^>(PosterioriEstimationCur_));

  // 真値更新(状態空間モデル内の真値を更新する)
  //StateSpaceModel_->UpdateInnerTrueValueWithNoise(i_vecInput,vs TimeCur_);
  StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0) = i_truth;

  // 観測値更新(状態空間モデル内の観測値を更新する)
  for (int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0) = i_observations[i];

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = PosterioriEstimationCur_->VecMean_.at<double>(0, 0); // 一時刻シフト
  result[debugIndex++] = PosterioriEstimationOld_->VecMean_.at<double>(0, 0);  // 真値
  result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);  // 観測
#endif
}

/**
 * 状態推定結果を取得する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param o_estimationResult  状態推定結果格納先.
**/
void ParticleFilter::AcquireEstimationResult(
  EstimationResult^ o_estimationResult)
{
  // arrange
  double posterioriEstimation = PosterioriEstimationCur_->VecMeanList_[0];// VecMean_.at<double>(0, 0);
  double posterioriErrorCovariance = PosterioriEstimationCur_->MatVariance_.at<double>(0, 0);
  double priorEstimation = PosterioriEstimationCur_->VecMeanList_[0];
  double trueValue = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);
  //-- センサは複数対応
  //---- 観測値
  List<double>^ observationsMultiSensor = gcnew List<double>;
  for (int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    observationsMultiSensor->Add(StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0));
  //---- 分散
  List<double>^ varObservationsMultiSensor = gcnew List<double>;
  for (int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    varObservationsMultiSensor->Add(StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->MatVariance_.at<double>(0, 0));

  //-- for PF
  //---- フィルタ後パーティクル
  List<double>^ posterioriParticlesState = gcnew List<double>();
  List<double>^ posterioriParticlesWeight = gcnew List<double>();
  List<Particle^>^ posterioriParticles = (static_cast<ParticleDistribution^>(PosterioriEstimationCur_))->Particles_;
  for each(Particle^ particle in posterioriParticles)
  {
    double state = particle->StateDistribution_->VecMean_.at<double>(0, 0);
    double weight = particle->Weight_;
    posterioriParticlesState->Add(state);
    posterioriParticlesWeight->Add(weight);
  }
  //---- フィルタ前パーティクル(リサンプリング前)
  List<double>^ priorParticlesState = gcnew List<double>();
  List<double>^ priorParticlesWeight = gcnew List<double>();
  List<Particle^>^ priorParticles = (static_cast<ParticleDistribution^>(PriorEstimationCur_))->Particles_;
  for each(Particle^ particle in priorParticles)
  {
    double state = particle->StateDistribution_->VecMean_.at<double>(0, 0);
    double weight = particle->Weight_;
    priorParticlesState->Add(state);
    priorParticlesWeight->Add(weight);
  }
  // リサンプリングビット
  bool resampledCurr = (static_cast<ParticleDistribution^>(PosterioriEstimationCur_))->resampledCurr_;

  // act
  //-- 事後推定値
  o_estimationResult->posterioriEstimations->Add(posterioriEstimation);
  o_estimationResult->posterioriErrorDeviations->Add(sqrt(posterioriErrorCovariance));
  //-- 事前推定値
  o_estimationResult->priorEstimations->Add(priorEstimation);
  //-- 真値
  o_estimationResult->trueValues->Add(trueValue);
  //-- 観測値. センサは複数対応
  o_estimationResult->observationsMultiSensor->Add(observationsMultiSensor);
  //-- 観測分散. センサは複数対応
  o_estimationResult->varObservationsMultiSensor->Add(varObservationsMultiSensor);
  //-- 事後状態推定値と真値の誤差
  o_estimationResult->errsBtwnPosterioriEstimationAndTruth->Add(ErrorBtwnPosterioriEstimationAndTruth);
  //-- 事後状態推定値から予測した観測値と実観測値の誤差
  o_estimationResult->errsBtwnPosterioriEstimationAndObservation->Add(ErrorBtwnPosterioriEstimationAndObservation);
  //-- For PF
  o_estimationResult->posterioriParticlesStates->Add(posterioriParticlesState);
  o_estimationResult->posterioriParticlesWeights->Add(posterioriParticlesWeight);
  o_estimationResult->priorParticlesStates->Add(priorParticlesState);
  o_estimationResult->priorParticlesWeights->Add(priorParticlesWeight);
  o_estimationResult->resampleBit->Add(resampledCurr);
  o_estimationResult->statesObserve->Add(stateObserve_);
  o_estimationResult->deviance_->Add(deviance_);
  o_estimationResult->likelihoodsH0_->Add(likelihoodsH0_);
  o_estimationResult->likelihoodsH1_->Add(likelihoodsH1_);

  List<double>^ posterioriOfXhatFromYCur = gcnew List<double>();
  for (int indexParticle = 0; indexParticle < priorParticles->Count; indexParticle++)
  {
    double posteriori = 0;
    if (pointEstimation_->GetType() == PfMapPointEstimation::typeid)
      posteriori = (*((PfMapPointEstimation^)pointEstimation_)->posterioriOfXhatFromYCur_)[indexParticle];
    else
      posteriori = -1;
    posterioriOfXhatFromYCur->Add(posteriori);
  };
  o_estimationResult->posterioriPfMap->Add(posterioriOfXhatFromYCur);
  
}

/**
 * パラメータを初期化する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel 状態推定用モデル方程式一式.
 * @param i_pointEstimation 指定する点推定アルゴリズム.
**/
void ParticleFilter::InitializeParameters(
  StateSpaceModel^ i_stateSpaceModel,
  AbstractPointEstimation^ i_pointEstimation,
  AbstractResampling^ i_resampling)
{
  PriorEstimationCur_ = gcnew ParticleDistribution(static_cast<ParticleDistribution^>(PosterioriEstimationOld_));
  PosterioriEstimationCur_ = gcnew ParticleDistribution(static_cast<ParticleDistribution^>(PosterioriEstimationOld_));

  // 観測モデルは正規分布とする
  for (int indexSensor = 0; indexSensor < i_stateSpaceModel->ObservationModelEquations_->Count; indexSensor++)
  {
    cv::Mat vecMeanInit = i_stateSpaceModel->ObservationModelEquations_[indexSensor]->observationCur_->VecMean_;
    cv::Mat matVarianceInit = i_stateSpaceModel->ObservationModelEquations_[indexSensor]->observationCur_->MatVariance_;
    StateSpaceModel_->ObservationModelEquations_[indexSensor]->observationCur_ = gcnew NormalDistribution(vecMeanInit, matVarianceInit);
  }

  int countParticle = (static_cast<ParticleDistribution^>(PosterioriEstimationOld_))->Particles_->Count;
  likelihoodsPriorStateToObserve_ = new vector<double>(countParticle, 0.);
  logLikelihoodsPriorStateToObserve_ = new vector<double>(countParticle, 0.);

  // デフォルトの点推定アルゴリズムはMMSE
  pointEstimation_ = (i_pointEstimation != nullptr) ? i_pointEstimation : gcnew MMSEPointEstimation();
  // デフォルトのリサンプリングアルゴリズムはLow Variance Sampling
  resampling_ = (i_resampling != nullptr) ? i_resampling : gcnew LowVarianceSampling();

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = vecMeanInit.at<double>(0, 0);
  result[debugIndex++] = matVarianceInit.at<double>(0, 0);
  result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->MatVariance_.at<double>(0, 0);
#endif
}

#if 0
/**
 * パーティクルの尤度を計算する. 実際に得られた観測値とパーティクルの事前推定値の間の尤度.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void ParticleFilter::CalcLikelifood()
{
  // arrange
  const int countParticles = likelihoodsPriorStateToObserve_->size();

  // act
  //-- 事前観測推定値を求める
  CalcPriorPredictionObserveCur();
  //-- 対数尤度を求める
  for (int i = 0; i < countParticles; i++)
  {
    cv::Mat likelihoodTemp = cv::Mat::zeros(1, 1, CV_64F);
    // y[k]に対するyhat-[k]の対数尤度を計算する。
    StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->CalcDencityFunction((*vecPriorPredictionObserveCur_)[i], &likelihoodTemp);
    (*likelihoodsPriorStateToObserve_)[i] = likelihoodTemp.at<double>(0, 0);
  }

  // debug
#if MY_DEBUG
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < priorParticleListCur->Count; i++)
  {
    result[debugIndex++] = priorParticleListCur[i]->StateDistribution_->VecMean_.at<double>(0, 0);
    result[debugIndex++] = vecPriorPredictionObserveCur[i].at<double>(0, 0);
    result[debugIndex++] = (*likelihoodsPriorStateToObserve_)[i];
  }
  result[debugIndex++] = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);
  result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);
#endif
}

/**
* パーティクルの尤度から重みを計算する. 計算結果はメンバ変数へ格納.
*
* @author  Morita
* @date  2015/06/17
**/
void ParticleFilter::CalcWeight()
{
  // arrange
  //-- 一時刻前事後推定パーティクル
  List<Particle^>^ posteriorParticleListOld = (static_cast<ParticleDistribution^>(PosterioriEstimationOld_))->Particles_;
  //-- 現時刻事前推定パーティクル
  List<Particle^>^ priorParticleListCur = (static_cast<ParticleDistribution^>(PriorEstimationCur_))->Particles_;
  //--  パーティクル数
  const int countParticles = static_cast<int>((*likelihoodsPriorStateToObserve_).size());

  // act
  //-- 正規化前の重み: w(m)[k] = w(m)[k-1] * h(m)(y[k]|xhat-(m)[k])
  //--               : 現時刻重み ∝ (一時刻前重み) * (現時刻尤度)
  //-- 重みの総和
  double sumWeight = 0.;
  vector<double> weightsBeforeNormal(countParticles);
  for (int i = 0; i < countParticles; i++)
  {
    weightsBeforeNormal[i] = posteriorParticleListOld[i]->Weight_ * (*likelihoodsPriorStateToObserve_)[i];
    sumWeight += weightsBeforeNormal[i];
  }
  //-- 重みの正規化
  for (int i = 0; i < countParticles; i++)
    priorParticleListCur[i]->Weight_ = weightsBeforeNormal[i] / sumWeight;

#if MY_DEBUG
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < priorParticleListCur->Count; i++)
  {
    result[debugIndex++] = (*likelihoodsPriorStateToObserve_)[i];
    result[debugIndex++] = posteriorParticleListOld[i]->Weight_;
    result[debugIndex++] = weightsBeforeNormal[i];
    result[debugIndex++] = priorParticleListCur[i]->Weight_;
  }
#endif
}
#endif

/**
 * パーティクルの対数尤度を計算する. 実際に得られた観測値とパーティクルの事前推定値の間の尤度.
 *
 * @author  Morita
 * @date  2015/06/17
**/
vector<double> ParticleFilter::CalcLogLikelifood(
  const int i_indexSensor)
{
  // arrange
  const int countParticles = static_cast<ParticleDistribution^>(PriorEstimationCur_)->Particles_->Count;
  vector<double> logWeightSingle(countParticles, 0.);

  // act
  //-- 事前観測推定値を求める
  CalcPriorPredictionObserveCur(i_indexSensor);
  //-- 事前観測推定値から対数尤度を求める
  for (int indexParticle = 0; indexParticle < countParticles; indexParticle++)
  {
    cv::Mat logLikelihoodTemp = cv::Mat::zeros(1, 1, CV_64F);
#if 1
    double var = StateSpaceModel_->ObservationModelEquations_[i_indexSensor]->observationCur_->MatVariance_.at<double>(0, 0);
    double varOri = StateSpaceModel_->ObservationModelEquations_[i_indexSensor]->matVarianceObservationOri_->at<double>(0, 0);
#endif
    // y[k]に対するyhat-[k]の対数尤度を計算する。
    StateSpaceModel_->ObservationModelEquations_[i_indexSensor]->observationCur_->CalcLogDencityFunction((*vecPriorPredictionObserveCur_)[indexParticle], &logLikelihoodTemp);
    //(*logLikelihoodsPriorStateToObserve_)[indexParticle] = logLikelihoodTemp.at<double>(0, 0);
    logWeightSingle[indexParticle] = logLikelihoodTemp.at<double>(0, 0);
  }
  //-- 尤度変数も求められるが、以降不要なのでコメントアウト
  // CalcLogLikelifoodFromLiklifood();

  // debug
#if 1
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < countParticles; i++)
  {
    //result[debugIndex++] = priorParticleListCur[i]->StateDistribution_->VecMean_.at<double>(0, 0);
    //result[debugIndex++] = vecPriorPredictionObserveCur[i].at<double>(0, 0);
    result[debugIndex++] = logWeightSingle[i];//(*logLikelihoodsPriorStateToObserve_)[i];
    result[debugIndex++] = (*vecPriorPredictionObserveCur_)[i].at<double>(0, 0);
  }
  result[debugIndex++] = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);
  for (int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0);
#endif

  return logWeightSingle;
}

vector<double> ParticleFilter::CalcLogLikelifoodAfterResample(
  const int i_indexSensor)
{
  // arrange
  const int countParticles = static_cast<ParticleDistribution^>(PosterioriEstimationCur_)->Particles_->Count;
  vector<double> logWeightSingle(countParticles, 0.);

  // act
  //-- 事前観測推定値を求める
  CalcPosterioriPredictionObserveCurAfterResample(i_indexSensor);
  //-- 事前観測推定値から対数尤度を求める
  for (int indexParticle = 0; indexParticle < countParticles; indexParticle++)
  {
    cv::Mat logLikelihoodTemp = cv::Mat::zeros(1, 1, CV_64F);
    // y[k]に対するyhat-[k]の対数尤度を計算する。
    StateSpaceModel_->ObservationModelEquations_[i_indexSensor]->observationCur_->CalcLogDencityFunction((*vecPosterioriPredictionObserveCur_)[indexParticle], &logLikelihoodTemp);
    //(*logLikelihoodsPriorStateToObserve_)[indexParticle] = logLikelihoodTemp.at<double>(0, 0);
    logWeightSingle[indexParticle] = logLikelihoodTemp.at<double>(0, 0);
  }

  // debug
#if 1
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < countParticles; i++)
  {
    //result[debugIndex++] = priorParticleListCur[i]->StateDistribution_->VecMean_.at<double>(0, 0);
    //result[debugIndex++] = vecPriorPredictionObserveCur[i].at<double>(0, 0);
    result[debugIndex++] = logWeightSingle[i];//(*logLikelihoodsPriorStateToObserve_)[i];
    result[debugIndex++] = (*vecPosterioriPredictionObserveCur_)[i].at<double>(0, 0);
  }
#endif

  return logWeightSingle;
}

/**
 * パーティクルの事前観測値を計算する. 事前推定値から予測した観測値.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void ParticleFilter::CalcPriorPredictionObserveCur(
  const int i_indexSensor)
{
  // arrange
  //-- 現時刻事前推定パーティクル群
  //---- 名前が長くなるのでエイリアス。パーティクルリストのポインタをコピー。
  List<Particle^>^ priorParticleListCur = (static_cast<ParticleDistribution^>(PriorEstimationCur_))->Particles_;
  //-- パーティクル数
  const int countParticles = priorParticleListCur->Count;
  //-- 事前観測推定
  delete vecPriorPredictionObserveCur_;
  vecPriorPredictionObserveCur_ = new vector<cv::Mat>(countParticles, cv::Mat::zeros(1, 1, CV_64F));

  // act
  //-- 事前観測値を求める
  for (int i = 0; i < countParticles; i++)
  {
    // xhat-[k]
    cv::Mat vecPriorPredictionCur = priorParticleListCur[i]->StateDistribution_->VecMean_;
    cv::Mat logLikelihoodTemp = cv::Mat::zeros(1, 1, CV_64F);

    // yhat-[k] = h(xhat-[k])
    cv::Mat vecPriorPredictionObservationCur
      = StateSpaceModel_->ObservationModelEquations_[i_indexSensor]->CalcFunctionWithoutNoise(vecPriorPredictionCur, TimeCur_);
    (*vecPriorPredictionObserveCur_)[i] = vecPriorPredictionObservationCur;
  }

  // debug
#if MY_DEBUG
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < countParticles; i++)
  {
    result[debugIndex++] = (*vecPriorPredictionObserveCur_)[i].at<double>(0, 0);
  }
  result[debugIndex++] = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);
  for (int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0);
#endif
}

void ParticleFilter::CalcPosterioriPredictionObserveCurAfterResample(
  const int i_indexSensor)
{
  // arrange
  //-- 現時刻リサンプリング後推定パーティクル群
  //---- 名前が長くなるのでエイリアス。パーティクルリストのポインタをコピー。
  List<Particle^>^ posterioriEstimationCur = (static_cast<ParticleDistribution^>(PosterioriEstimationCur_))->Particles_;
  //-- パーティクル数
  const int countParticles = posterioriEstimationCur->Count;
  //-- 事前観測推定
  delete vecPosterioriPredictionObserveCur_;
  vecPosterioriPredictionObserveCur_ = new vector<cv::Mat>(countParticles, cv::Mat::zeros(1, 1, CV_64F));

  // act
  //-- リサンプリング後予測観測値を求める
  for (int i = 0; i < countParticles; i++)
  {
    // xhat[k]
    cv::Mat vecPosterioriPredictionCur = posterioriEstimationCur[i]->StateDistribution_->VecMean_;
    cv::Mat logLikelihoodTemp = cv::Mat::zeros(1, 1, CV_64F);

    // yhat[k] = h(xhat[k])
    cv::Mat vecPosterioriPredictionObservationCur
      = StateSpaceModel_->ObservationModelEquations_[i_indexSensor]->CalcFunctionWithoutNoise(vecPosterioriPredictionCur, TimeCur_);
    (*vecPosterioriPredictionObserveCur_)[i] = vecPosterioriPredictionObservationCur;
  }
#if 1
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < countParticles; i++)
  {
    result[debugIndex++] = posterioriEstimationCur[i]->StateDistribution_->VecMean_.at<double>(0, 0);
    result[debugIndex++] = (*vecPosterioriPredictionObserveCur_)[i].at<double>(0, 0);
  }
#endif
}

/**
 * パーティクルの尤度を計算する. 対数尤度から尤度を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void ParticleFilter::CalcLogLikelifoodFromLiklifood()
{
  // arrange
  const int countParticles = logLikelihoodsPriorStateToObserve_->size();
  double logLikelihoodTempMin = (*logLikelihoodsPriorStateToObserve_)[0];

  // act
  //-- 最小対数尤度を求める
  for (int i = 0; i < countParticles; i++)
    logLikelihoodTempMin =
    ((*logLikelihoodsPriorStateToObserve_)[i] < logLikelihoodTempMin) ? (*logLikelihoodsPriorStateToObserve_)[i] : logLikelihoodTempMin;

  //-- 事前推定値の観測に対する対数尤度 - 当該尤度の最小値: これにより対数尤度の取る値の範囲が 0 ~ (l_max - l_min) に抑えられる。 
  vector<double> logLikelihoodsPriorStateToObserveMinusMin = vector<double>(countParticles, 0.);
  //-- 尤度 = exp(対数尤度 - 当該尤度の最小値): exp(正の値)となり、アンダーフローが回避。される 
  vector<double> weightsBeforeNormal = vector<double>(countParticles, 0);
  for (int i = 0; i < countParticles; i++)
  {
    logLikelihoodsPriorStateToObserveMinusMin[i] = (*logLikelihoodsPriorStateToObserve_)[i] - logLikelihoodTempMin;
    (*likelihoodsPriorStateToObserve_)[i] = exp(logLikelihoodsPriorStateToObserveMinusMin[i]);
  }
}

/**
 * パーティクルの尤度から対数重みを計算する. 計算結果はメンバ変数へ格納.
 *
 * @author  Morita
 * @date  2015/06/17
**/
vector<double> ParticleFilter::CalcLogWeight()
{
  // パーティクルの尤度から対数重みを計算する(最大値引算無/正規化前)。
  CalcLogWeightWithoutMaxSubtractionBeforeNormal();

  // パーティクルの尤度から対数重みを計算する(最大値引算有/正規化後)。
  vector<double> weights;
  weights = CalcLogWeightWithMaxSubtractionAfterNormal();

  return weights;
}

/**
 * パーティクルの尤度から対数重みを計算する(最大値引き算無/正規化前). CalcLogWeight()内サブルーチン.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void ParticleFilter::CalcLogWeightWithoutMaxSubtractionBeforeNormal()
{
  // arrange
  //-- 一時刻前事後推定パーティクル
  List<Particle^>^ posteriorParticleListOld = (static_cast<ParticleDistribution^>(PosterioriEstimationOld_))->Particles_;
  //-- パーティクル数
  const int countParticles = posteriorParticleListOld->Count;
  //-- 正規化前の対数重み: w(m)[k] = w(m)[k-1] * h(m)(y[k]|xhat-(m)[k])
  //--               　　: log{w(m)[k]} = log{w(m)[k-1]} + log{h(m)(y[k]|xhat-(m)[k])}
  //--               　　: 現時刻対数重み ∝ (一時刻前対数重み) * (現時刻対数尤度)
  delete logWeightsBeforeNormal_;
  logWeightsBeforeNormal_ = new vector<double>(countParticles, 0.);

  // act
  for (int i = 0; i < countParticles; i++)
  {
    // 正規化前の対数重み
    (*logWeightsBeforeNormal_)[i] = log(posteriorParticleListOld[i]->Weight_) + (*logLikelihoodsPriorStateToObserve_)[i];
    // 初期値を入れる
    if (i == 0)
    {
#if 0
      //-- 正規化前の最小対数重みを求める
      logWeightsBeforeNormalMin_ = (*logWeightsBeforeNormal_)[0];
#endif
      //-- 正規化前の最大対数重みを求める
      logWeightsBeforeNormalMax_ = (*logWeightsBeforeNormal_)[0];
    }
#if 0
    // 正規化前の最小対数重み 計算値が現在の最小よりも小さければ、最小値を更新する
    logWeightsBeforeNormalMin_ =
      ((*logWeightsBeforeNormal_)[i] < logWeightsBeforeNormalMin_) ? (*logWeightsBeforeNormal_)[i] : logWeightsBeforeNormalMin_;
#endif
    // 正規化前の最大対数重み 計算値が現在の最大よりも大きければ、最大値を更新する
    logWeightsBeforeNormalMax_ =
      ((*logWeightsBeforeNormal_)[i] > logWeightsBeforeNormalMax_) ? (*logWeightsBeforeNormal_)[i] : logWeightsBeforeNormalMax_;
  }
  
#if 1
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < posteriorParticleListOld->Count; i++)
  {
    result[debugIndex++] = (*logLikelihoodsPriorStateToObserve_)[i];
    result[debugIndex++] = posteriorParticleListOld[i]->Weight_;
    result[debugIndex++] = (*logWeightsBeforeNormal_)[i];
  }
#endif
}

/**
 * パーティクルの尤度から対数重みを計算する(最大値引算有/正規化後). CalcLogWeight()内サブルーチン.
 *
 * @author  Morita
 * @date  2015/06/17
**/
vector<double> ParticleFilter::CalcLogWeightWithMaxSubtractionAfterNormal()
{
  // arrange
  //-- 現時刻事前推定パーティクル: テンポラリ
#if 0
  List<Particle^>^ priorParticleListCurTemp
    = gcnew List<Particle^>((static_cast<ParticleDistribution^>(PriorEstimationCur_))->Particles_);
#endif
  //-- パーティクル数
  const int countParticles = ((static_cast<ParticleDistribution^>(PriorEstimationCur_))->Particles_)->Count;

  vector<double> weights;
  weights.resize(countParticles);

  //-- 正規化前の対数重み - 当該尤度の最大値: これにより正規化前重みの取る値の範囲が 0~1 に抑えられる。 
  vector<double> logWeightsBeforeNormalMinusMax = vector<double>(countParticles, 0);
  //-- 正規化前の重み = exp(正規化前の対数重み - 当該尤度の最大値)-> 0~1となり、正規化前重みの総和が1より大きくなる. 
  //-- ※はじめ、アンダーフロー防止のためと思い最小値で引いていたがこれは×. 総和が0になり正規化重みがNaNになる.
  vector<double> weightsBeforeNormal = vector<double>(countParticles, 0);
  //-- 正規化前の重みの総和. 
  double sumWeightBeforeNormal = 0.;

  // act
  for (int i = 0; i < countParticles; i++)
  {
    // 正規化前の対数重み - 当該尤度の最大値
    logWeightsBeforeNormalMinusMax[i] = (*logWeightsBeforeNormal_)[i] - logWeightsBeforeNormalMax_;
    // 正規化前の重み
    weightsBeforeNormal[i] = exp(logWeightsBeforeNormalMinusMax[i]);
    // 正規化前の重みの総和
    sumWeightBeforeNormal += weightsBeforeNormal[i];
  }

  //-- 重みの正規化
  double perSumWeightBeforeNormal = 1. / sumWeightBeforeNormal;
  for (int i = 0; i < static_cast<int>(weights.size()); i++)
    weights[i] = weightsBeforeNormal[i] * perSumWeightBeforeNormal;

  // debug
#if 1
  double result[1000] = { 0 };
  int debugIndex = 0;
  for (int i = 0; i < static_cast<int>(weightsBeforeNormal.size()); i++)
  {
    result[debugIndex++] = (*logLikelihoodsPriorStateToObserve_)[i];
    result[debugIndex++] = weightsBeforeNormal[i];
  }
#endif
  return weights;
}

/**
 * Gets the likelihoodsPriorStateToObserve_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A vector&lt;double&gt; likelihoodsPriorStateToObserve_.
**/
vector<double> ParticleFilter::LikelihoodsPriorStateToObserve_::get()
{
  return *likelihoodsPriorStateToObserve_;
}

/**
 * Sets the given value to likelihoodsPriorStateToObserve_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ParticleFilter::LikelihoodsPriorStateToObserve_::set(
  vector<double> value)
{
  *likelihoodsPriorStateToObserve_ = value;
}

/**
 * Gets the logLikelihoodsPriorStateToObserve_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A vector&lt;double&gt; logLikelihoodsPriorStateToObserve_.
**/
vector<double> ParticleFilter::LogLikelihoodsPriorStateToObserve_::get()
{
  return *logLikelihoodsPriorStateToObserve_;
}

/**
 * Sets the given value to logLikelihoodsPriorStateToObserve_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ParticleFilter::LogLikelihoodsPriorStateToObserve_::set(
  vector<double> value)
{
  *logLikelihoodsPriorStateToObserve_ = value;
}

/**
 * Sets the given value to resampling_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ParticleFilter::Resampling_::set(
  AbstractResampling^ value)
{
  resampling_ = value;
}

/**
 * Sets the given value to pointEstimation_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ParticleFilter::PointEstimation_::set(
  AbstractPointEstimation^ value)
{
  pointEstimation_ = value;
}
