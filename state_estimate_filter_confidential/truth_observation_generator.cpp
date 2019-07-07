/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : truth_observation_generator.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "truth_observation_generator.h"
//
using namespace state_estimate_filter;

/**
 * 真値と観測値を生成する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel 状態空間モデル.
 * @param i_timeMax         全シミュレーション時間.
 *
 * @return  真値及び観測値リスト.
**/
List<TruthAndObservation^>^ TruthAndObservationGenerator::Generate(
  StateSpaceModel^ i_stateSpaceModel,
  const int i_timeMax)
{
  // arrange
  List<TruthAndObservation^>^ truthAndObservations = gcnew List<TruthAndObservation^>();
  cv::Mat vecTruth = i_stateSpaceModel->VecStateTrueCur_;
  cv::Mat vecObservation(1, 1, CV_64F, 0.);

  // act
  for (int k = 0; k < i_timeMax; k++)
  {
    double truth;
    List<double> observations;

    // 真値を次時刻に更新
    vecTruth = i_stateSpaceModel->ProcessModelEquation_->CalcFunctionWithNoise(vecTruth, k + 1);
    truth = vecTruth.at<double>(0, 0);
    // 観測を次時刻に更新
    for (int i = 0; i < i_stateSpaceModel->ObservationModelEquations_->Count; i++)
    {
      vecObservation = i_stateSpaceModel->ObservationModelEquations_[i]->CalcFunctionWithNoise(vecTruth, k + 1);
      observations.Add(vecObservation.at<double>(0, 0));
    }
    // 真値観測格納オブジェクトに追加
    truthAndObservations->Add(gcnew TruthAndObservation(truth, %observations));
  }
#if MY_DEBUG
  vector<double> truths(truthAndObservations->Count);
  vector<double> observations(truthAndObservations->Count);
  for (int i = 0; i < truthAndObservations->Count; i++)
  {
    truths[i] = truthAndObservations[i]->truth_[0];
    observations[i] = truthAndObservations[i]->observation_[0];
  }
#endif
  return truthAndObservations;
}
