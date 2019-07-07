/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : likelihood_ratio_test.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "abstract_test.h"

namespace state_estimate_filter
{
  /**
  * @brief Low Variance Sampling アルゴリズム リサンプリングクラス.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class LikelihoodRatioTest : AbstractTest
  {
  public:
    //static const double kThreshEssDefault = 0.33;
  public:
    LikelihoodRatioTest()
    {

    }

    LikelihoodRatioTest(const double i_threshEss)
    {
      //threshEss_ = i_threshEss;
    }

    ~LikelihoodRatioTest()
    {
      delete[] means_;
    }

  public: // methods

    /**
     * 検定する.
     *
     * @author  Morita
     * @date  2015/06/27
     *
     * @return  nullptr if it fails, else a TestResult^.
    **/
    TestResult^ Test() override;


  private:
    vector<double> *means_ = new vector<double>();
    double deviance_ = 0; // 逸脱度
    //double threshEss_ = kThreshEssDefault;
  };
}
