/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : truth_observation_generator.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "state_estimate_filter.h"

namespace state_estimate_filter
{
  /**
   * @brief 真値及び観測値生成クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class TruthAndObservationGenerator
  {
  public:
    virtual List<TruthAndObservation^>^ Generate(
      StateSpaceModel ^i_stateSpaceModel,
      const int i_timeMax);
  };
}
