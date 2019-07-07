/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : state_estimater_factory.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/#include "stdafx.h"
//
#include "state_estimater_factory.h"
//
using namespace state_estimate_filter;

/**
 * 状態推定フィルタモデルを生成ためにパラメータを然るべきオブジェクトに直す.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定フィルタパラメータ一式.
**/
void AbstractStateEstimateFilterFactory::PrepareParamsForStateSpaceModel(
  StateEstimaterParam^ i_stateEstimateParam)
{
  // プロセスモデル
  processModelEquation_ = CreateProcessModelEquation(i_stateEstimateParam);
  // 観測モデル
  for (int indexSensor = 0; indexSensor < i_stateEstimateParam->modelParamsObservation_->Count; indexSensor++)
  {
    observationModelEquation_->Add(CreateObservationModelEquation(i_stateEstimateParam->modelParamsObservation_[indexSensor]));
  }
  // 真値初期値
  vecStateTrueInit_ = new cv::Mat((cv::Mat_<double>(1, 1) << i_stateEstimateParam->stateTrueInit_));
}

/**
 * プロセスモデル式オブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定パラメータ一式.
 *
 * @return  プロセスモデル式オブジェクト.
**/
ProcessModelEquation^ AbstractStateEstimateFilterFactory::CreateProcessModelEquation(
  StateEstimaterParam^ i_stateEstimateParam)
{
  // 運動方程式 f(x_k)
  ModelFunc^ functionProcess = i_stateEstimateParam->modelParamProcess_->function_; // プロセスモデル関数ポインタ配列(デリゲード)。
  List<ModelFunc^>^ functionProcessJacobians = i_stateEstimateParam->modelParamProcess_->derivations_; // プロセスモデルヤコビアン関数ポインタ配列(デリゲード)。
//  functionProcess = gcnew ModelFunc(i_stateEstimateParam->modelFuncList_, &FunctionList::ProcessFuntion);
//  functionProcessJacobians = gcnew List < ModelFunc^ >;
 // functionProcessJacobians->Add(gcnew ModelFunc(i_stateEstimateParam->modelFuncList_, &FunctionList::ProcessDerivedFuntion));

  // A行列はEquationクラス内部でヤコビアンを元に生成する。

  cv::Mat vecb = (cv::Mat_<double>(1, 1) << 1.0);
  // プロセスノイズ: 平均, 分散を設定
  cv::Mat vecMeanNoiseProcess = (cv::Mat_<double>(1, 1) << i_stateEstimateParam->modelParamProcess_->noizeMean_);
  cv::Mat matVarianceNoiseProcess = (cv::Mat_<double>(1, 1) << i_stateEstimateParam->modelParamProcess_->noizeVariance_);
  NormalDistribution^ noizeProcess = gcnew NormalDistribution(vecMeanNoiseProcess, matVarianceNoiseProcess);

  // プロセスモデル式オブジェクト
  return gcnew NonLinearProcessModelEquation(functionProcess, functionProcessJacobians, vecb, noizeProcess);
}

/**
 * 観測モデル式オブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定パラメータ一式.
 *
 * @return  観測モデル式オブジェクト.
**/
ObservationModelEquation^ AbstractStateEstimateFilterFactory::CreateObservationModelEquation(
  ModelParam^ i_modelParamObservation)
{
  // 観測方程式 h(x_k)
  ModelFunc^ functionObservation = i_modelParamObservation->function_; // 観測モデル関数ポインタ配列(デリゲード)。
  List<ModelFunc^>^ functionObservationJacobians = i_modelParamObservation->derivations_; // 観測モデルヤコビアン関数ポインタ配列(デリゲード)。

//  functionObservation = gcnew ModelFunc(i_stateEstimateParam->modelFuncList_, &FunctionList::ObservationFuntion);
//  functionObservationJacobians = gcnew List < ModelFunc^ >;
//  functionObservationJacobians->Add(gcnew ModelFunc(i_stateEstimateParam->modelFuncList_, &FunctionList::ObservationDerivedFuntion));

  // C行列はEquationクラス内部でヤコビアンを元に生成する。

  // 観測ノイズ: 平均, 分散を設定
  cv::Mat vecMeanNoiseObserve = (cv::Mat_<double>(1, 1) << i_modelParamObservation->noizeMean_);
  cv::Mat matVarianceNoiseObserve = (cv::Mat_<double>(1, 1) << i_modelParamObservation->noizeVariance_);
  NormalDistribution^ noizeObserve = gcnew NormalDistribution(vecMeanNoiseObserve, matVarianceNoiseObserve);

  // 観測モデル式オブジェクト
  return gcnew NonLinearObservationModelEquation(functionObservation, functionObservationJacobians, noizeObserve);
}

/**
 * 初期事後推定確立分布オブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定パラメータ一式.
 *
 * @return  初期事後推定確立分布オブジェクト.
**/
ProbabilityDistribution^ AbstractStateEstimateFilterFactory::CreatePosterioriEstimationInit(
  DistributionParam^ i_distParamPosteriorEstimationInit)
{
  // 事後推定初期期待値
  cv::Mat vecMeanPosterioriEstimationInit =
    (cv::Mat_<double>(1, 1) << i_distParamPosteriorEstimationInit->mean_);
  // 事後推定初期分散
  cv::Mat matVariancePosterioriEstimationInit =
    (cv::Mat_<double>(1, 1) << i_distParamPosteriorEstimationInit->variance_);

  return gcnew NormalDistribution(vecMeanPosterioriEstimationInit, matVariancePosterioriEstimationInit);
}

/**
 * 状態推定フィルタオブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定フィルタパラメータ一式.
 *
 * @return  状態推定フィルタオブジェクト.
**/
AbstractStateEstimateFilter^ LinearKalmanFilterFactory::Create(
  StateEstimaterParam^ i_stateEstimateParam)
{
  // システムモデル生成に必要なパラメータを生成する(プロセスモデル式、観測モデル方程式、真値初期値)
  PrepareParamsForStateSpaceModel(i_stateEstimateParam);
  // 線形システムモデル
  StateSpaceModel ^linearStateSpaceModel =
    gcnew LinearStateSpaceModel(processModelEquation_, observationModelEquation_, *vecStateTrueInit_);
  // 事後推定値初期値(カルマンフィルタは正規性を仮定しているので、明示的にキャストする)
  NormalDistribution^ distPosterioriEstimationInit =
    static_cast<NormalDistribution^>(CreatePosterioriEstimationInit(i_stateEstimateParam->posteriorEstimationStateInit_));

  // 拡張カルマンフィルタ
  return gcnew LinearKalmanFilter(linearStateSpaceModel, distPosterioriEstimationInit);
};

/**
 * プロセスモデル式オブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定パラメータ一式.
 *
 * @return  プロセスモデル式オブジェクト.
**/
ProcessModelEquation^ LinearKalmanFilterFactory::CreateProcessModelEquation(
  StateEstimaterParam^ i_stateEstimateParam)
{
  ModelFunctionSetProcess^ modelParamProcess = (ModelFunctionSetProcess^)i_stateEstimateParam->modelParamProcess_->modelFunctionSet_;
  //-- 運動方程式 x_k = A*x_k-1 + b*u
  cv::Mat matA = *(modelParamProcess->matA);//(cv::Mat_<double>(1, 1) << 1.0);
  cv::Mat vecb = *(modelParamProcess->vecb); //(cv::Mat_<double>(1, 1) << 1.0);

  // プロセスノイズ: 平均, 分散を設定
  cv::Mat vecMeanNoiseProcess = (cv::Mat_<double>(1, 1) << i_stateEstimateParam->modelParamProcess_->noizeMean_);
  cv::Mat matVarianceNoiseProcess = (cv::Mat_<double>(1, 1) << i_stateEstimateParam->modelParamProcess_->noizeVariance_);
  NormalDistribution^ noizeProcess = gcnew NormalDistribution(vecMeanNoiseProcess, matVarianceNoiseProcess);

  // プロセスモデル式オブジェクト
  return gcnew LinearProcessModelEquation(matA, vecb, noizeProcess);
}

/**
 * 観測モデル式オブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定パラメータ一式.
 *
 * @return  観測モデル式オブジェクト.
**/
ObservationModelEquation^ LinearKalmanFilterFactory::CreateObservationModelEquation(
  ModelParam^ i_modelParamObservation)
{
  //-- 観測方程式 y_k = C * x_k
  ModelFunctionSetObserve^ modelFunctionSetObservation = (ModelFunctionSetObserve^)i_modelParamObservation->modelFunctionSet_;
  cv::Mat matC = *modelFunctionSetObservation->matC;//*i_stateEstimateParam->modelFuncList_->matC;
  // 観測ノイズ: 平均, 分散を設定
  cv::Mat vecMeanNoiseObserve = (cv::Mat_<double>(1, 1) << i_modelParamObservation->noizeMean_);
  cv::Mat matVarianceNoiseObserve = (cv::Mat_<double>(1, 1) << i_modelParamObservation->noizeVariance_);
  NormalDistribution^ noizeObserve = gcnew NormalDistribution(vecMeanNoiseObserve, matVarianceNoiseObserve);

  // 観測モデル式オブジェクト
  return gcnew LinearObservationModelEquation(matC, noizeObserve);

}

/**
 * 状態推定フィルタオブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定フィルタパラメータ一式.
 *
 * @return  状態推定フィルタオブジェクト.
**/
AbstractStateEstimateFilter^ ExtendedKalmanFilterFactory::Create(
  StateEstimaterParam^ i_stateEstimateParam)
{
  // システムモデル生成に必要なパラメータを生成する(プロセスモデル式、観測モデル方程式、真値初期値)
  PrepareParamsForStateSpaceModel(i_stateEstimateParam);
  // 非線形システムモデル
  StateSpaceModel ^nonlinearStateSpaceModel
    = gcnew NonLinearStateSpaceModel(processModelEquation_, observationModelEquation_, *vecStateTrueInit_);
  // 事後推定値初期値(カルマンフィルタは正規性を仮定しているので、明示的にキャストする)
  NormalDistribution^ distPosterioriEstimationInit =
    static_cast<NormalDistribution^>(CreatePosterioriEstimationInit(i_stateEstimateParam->posteriorEstimationStateInit_));

  // 拡張カルマンフィルタ
  return gcnew ExtendedKalmanFilter(nonlinearStateSpaceModel, distPosterioriEstimationInit);
};

/**
 * 状態推定フィルタオブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定フィルタパラメータ一式.
 *
 * @return  状態推定フィルタオブジェクト.
**/
AbstractStateEstimateFilter^ UnscentedKalmanFilterFactory::Create(
  StateEstimaterParam^ i_stateEstimateParam)
{
  // システムモデル生成に必要なパラメータを生成する(プロセスモデル式、観測モデル方程式、真値初期値)
  PrepareParamsForStateSpaceModel(i_stateEstimateParam);
  // 非線形システムモデル
  StateSpaceModel ^nonlinearStateSpaceModel =
    gcnew NonLinearStateSpaceModel(processModelEquation_, observationModelEquation_, *vecStateTrueInit_);
  // 事後推定値初期値(カルマンフィルタは正規性を仮定しているので、明示的にキャストする)
  NormalDistribution^ distPosterioriEstimationInit =
    static_cast<NormalDistribution^>(CreatePosterioriEstimationInit(i_stateEstimateParam->posteriorEstimationStateInit_));

  // アンセンテッドカルマンフィルタ
  return gcnew UnscentedKalmanFilter(nonlinearStateSpaceModel, distPosterioriEstimationInit);
};

/**
 * 状態推定フィルタオブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定フィルタパラメータ一式.
 *
 * @return  状態推定フィルタオブジェクト.
**/
AbstractStateEstimateFilter^ ParticleFilterFactory::Create(
  StateEstimaterParam^ i_stateEstimateParam)
{
  // システムモデル生成に必要なパラメータを生成する(プロセスモデル式、観測モデル方程式、真値初期値)
  PrepareParamsForStateSpaceModel(i_stateEstimateParam);

  // 非線形システムモデル
  StateSpaceModel ^nonlinearStateSpaceModel =
    gcnew NonLinearStateSpaceModel(processModelEquation_, observationModelEquation_, *vecStateTrueInit_);

  // パーティクル数
  countParticles_ = i_stateEstimateParam->countParticles_;

  // パーティクル予測時乱数分布
  NormalDistribution ^distRandomDistributionForPredict_ =
    static_cast<NormalDistribution^>(CreateRandomDistributionForPredict(i_stateEstimateParam->randomDistributionForPredict_));

  // 事後推定値初期値(パーティクルフィルタ分布型に明示的にキャストする)
  ParticleDistribution ^distPosterioriEstimationInit =
    static_cast<ParticleDistribution^>(CreatePosterioriEstimationInit(i_stateEstimateParam->posteriorEstimationStateInit_));

  // パーティクルフィルタ
  return gcnew ParticleFilter(
    nonlinearStateSpaceModel, 
    distRandomDistributionForPredict_, 
    distPosterioriEstimationInit, 
    i_stateEstimateParam->pointEstimation_,
    i_stateEstimateParam->resampling_);
}

/**
 * 初期事後推定確立分布オブジェクトを生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimateParam  状態推定パラメータ一式.
 *
 * @return  初期事後推定確立分布オブジェクト.
**/
ProbabilityDistribution^ ParticleFilterFactory::CreatePosterioriEstimationInit(
  DistributionParam^ i_distParamPosteriorEstimationInit)
{
  // 事後推定値初期値
  cv::Mat vecMeanPosterioriEstimationInit =
    (cv::Mat_<double>(1, 1) << i_distParamPosteriorEstimationInit->mean_);
  cv::Mat matVariancePosterioriEstimationInit = (cv::Mat_<double>(1, 1) << i_distParamPosteriorEstimationInit->variance_);

  // 初期パーティクル生成
  NormalDistribution^ particleDistributionInit =
    gcnew NormalDistribution(vecMeanPosterioriEstimationInit, matVariancePosterioriEstimationInit);

  // パーティクル群生成
  List<Particle^>^ particles = gcnew List < Particle^ >;
  const double weightInit = 1. / static_cast<double>(countParticles_);
  for (int i = 0; i < countParticles_; i++)
    particles->Add(gcnew Particle(particleDistributionInit, weightInit));

  // パーティクル分布
  return gcnew ParticleDistribution(vecMeanPosterioriEstimationInit, matVariancePosterioriEstimationInit, particles);
}

ProbabilityDistribution^ ParticleFilterFactory::CreateRandomDistributionForPredict(
  DistributionParam^ i_distParamRandomPredict)
{
  // 事後推定値初期値
  cv::Mat vecMeanRandomPredict =
    (cv::Mat_<double>(1, 1) << i_distParamRandomPredict->mean_);
  cv::Mat matVarianceRandomPredict = (cv::Mat_<double>(1, 1) << i_distParamRandomPredict->variance_);

     
  // 乱数運否
  return gcnew NormalDistribution(vecMeanRandomPredict, matVarianceRandomPredict);
}