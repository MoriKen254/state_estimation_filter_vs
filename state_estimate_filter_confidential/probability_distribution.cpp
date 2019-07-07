/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : probability_distribution.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "probability_distribution.h"
//
using namespace state_estimate_filter;

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
ProbabilityDistribution::ProbabilityDistribution()
{
  if (vecMean_ == nullptr)
    vecMean_ = new cv::Mat(cv::Mat_<double>(1, 1) << 0);
  if (matVariance_ == nullptr)
    matVariance_ = new cv::Mat(cv::Mat_<double>(1, 1) << 1);
}

/**
 * Copy constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param obj コピー元オブジェクト.
**/
ProbabilityDistribution::ProbabilityDistribution(
  ProbabilityDistribution ^obj)
{
  vecMean_ = new cv::Mat(obj->VecMean_);
  matVariance_ = new cv::Mat(obj->MatVariance_);
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecMean     期待値ベクトル.
 * @param i_matVariance 共分散行列.
**/
ProbabilityDistribution::ProbabilityDistribution(
  const cv::Mat i_vecMean,
  const cv::Mat i_matVariance)
{
  vecMean_ = new cv::Mat(i_vecMean.clone());
  matVariance_ = new cv::Mat(i_matVariance.clone());
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_mean      期待値ベクトル.
 * @param i_variance  共分散行列.
**/
ProbabilityDistribution::ProbabilityDistribution(
  const double i_mean,
  const double i_variance)
{
  vecMean_ = new cv::Mat(1, 1, CV_64F, i_mean);
  matVariance_ = new cv::Mat(1, 1, CV_64F, i_variance);
}

/**
* 確率密度関数値を計算する.
*
* @author  Morita
* @date  2015/06/17
*
* @param i_matValue            関数入力値.
* @param [in,out]  o_vecResult 関数出力値.
**/
void ProbabilityDistribution::CalcDencityFunction(
  const cv::Mat i_matValue,
  cv::Mat* o_vecResult)
{
}

/**
 * 対数確率密度関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_matValue            関数入力値.
 * @param [in,out]  o_vecResult 関数出力値.
**/
void ProbabilityDistribution::CalcLogDencityFunction(
  const cv::Mat i_matValue,
  cv::Mat* o_vecResult)
{
}

double ProbabilityDistribution::CalcDencityFunction(
  const double i_value)
{
  return -99;
}

/**
 * 定義された正規分布に従って乱数を生成する. 初期シードは固定.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  乱数値.
**/
cv::Mat ProbabilityDistribution::GenRandomValue()
{ 
  return cv::Mat::ones(1, 1, CV_64F) * (-999); 
}

/**
 * Gets the vecMean_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A vecMean_.
**/
cv::Mat ProbabilityDistribution::VecMean_::get()
{
  if (vecMean_ == NULL)
    return cv::Mat::ones(1, 1, CV_64F) * (-999);

  return *vecMean_;
}

/**
 * Sets the given value to vecMean_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ProbabilityDistribution::VecMean_::set(
  const cv::Mat value)
{
  if (vecMean_ == NULL)
  {
    vecMean_ = new cv::Mat(value.clone());
    return;
  }

  *vecMean_ = value;
}

/**
 * Gets the vecMean_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  nullptr if it fails, else a vecMean_.
**/
List<double>^ ProbabilityDistribution::VecMeanList_::get()
{
  List<double>^ vecMeanList = gcnew List<double>;

  for (int i = 0; i < vecMean_->rows; i++)
  {
    vecMeanList->Add(vecMean_->at<double>(i, 0));
  }

  return vecMeanList;
}

/**
 * Gets the get matVariance_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A matVariance_.
**/
cv::Mat ProbabilityDistribution::MatVariance_::get()
{
  return *matVariance_;
}

/**
 * Sets the given value to matVariance_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ProbabilityDistribution::MatVariance_::set(
  const cv::Mat value)
{
  *matVariance_ = value;
}

