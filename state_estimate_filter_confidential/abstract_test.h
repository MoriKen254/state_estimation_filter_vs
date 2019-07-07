/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : abstract_test.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "particle_distribution.h"
//
using namespace System;
using std::vector;

namespace state_estimate_filter
{
  public ref class TestResult
  {
  public:
    bool isRejectedH0;  // 帰無仮説が棄却されたか. true なら主張が通っている.
    double pValue;  // p値
  };

  /**
   * 抽象検定クラス. 実装は無し.
   *
   * @author  Morita
   * @date  2015/06/26
  **/
  public ref class AbstractTest abstract
  {
  public: // methods

    /**
    * Default constructor.
    *
    * @author  Morita
    * @date  2015/06/23
    **/
    AbstractTest(){};

    /**
     * 検定する.
     *
     * @author  Morita
     * @date  2015/06/23
     *
     * @return  検定結果.
    **/
    virtual TestResult^ Test() = 0;
  };
}
