// particle_filter.h
#pragma once

#include "state_estimate_filter.h"
#include "particle_distribution.h"
#include "point_estimation.h"
#include "resampling.h" // 宣言側では基底クラスを宣言

namespace state_estimate_filter
{
  /**
   * @brief パーティクルフィルタフィルタクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ParticleFilter : AbstractStateEstimateFilter
  {
  public:	// constructors & destoructors
    ParticleFilter();

    ParticleFilter(
      StateSpaceModel^ i_stateSpaceModel);

    ParticleFilter(
        ProcessModelEquation^ i_processModelEquation
      , List<ObservationModelEquation^>^ i_observationModelEquation
      , const cv::Mat i_vecStateTrueInit);

    ParticleFilter(
        StateSpaceModel ^i_stateSpaceModel
      , ParticleDistribution^ i_posterioriEstimationInit);

    ParticleFilter(
      StateSpaceModel ^i_stateSpaceModel,
      ProbabilityDistribution^ vecRandomDistributionForPredict,
      ParticleDistribution^ i_posterioriEstimationInit,
      AbstractPointEstimation^ i_pointEstimation,
      AbstractResampling^ i_resampling);

  public: // methods
    void Predict(
      const cv::Mat i_vecInputCur) override; // 予測ステップ(事前推定)

    void Filter() override; // フィルタリングステップ(事後推定)

    void EstimateMAP(
      AbstractPointEstimation^ i_pointEstimation);  // 点推定

    void Loop(
      const int i_count
    , EstimationResult ^i_estimationResult) override;

    EstimationResult^ Loop(
      List<TruthAndObservation^>^ i_truthAndObservations) override;

    void UpdateToNextTime(
      const cv::Mat i_vecInput) override;

    void UpdateToNextTimeFromOuterValue(
      double i_truth,
      List<double>^ i_observation) override;

    void AcquireEstimationResult(
      EstimationResult^ i_estimationResult) override;


  private: // methods
    void InitializeParameters(
      StateSpaceModel^ i_stateSpaceModel,
      AbstractPointEstimation^ i_pointEstimation,
      AbstractResampling^ i_resampling);
#if 0
    void CalcLikelifood();
    void CalcWeight();
#endif
    vector<double> CalcLogLikelifood(
      const int i_indexSensor);

    void CalcPriorPredictionObserveCur(
      const int i_indexSensor);

    void CalcLogLikelifoodFromLiklifood();

    vector<double> CalcLogWeight();
  
    void CalcLogWeightWithoutMaxSubtractionBeforeNormal();

    vector<double> CalcLogWeightWithMaxSubtractionAfterNormal();

    // 以下、故障検出アルゴリズム用
    vector<double> CalcLogLikelifoodAfterResample(
          const int i_indexSensor);
    void CalcPosterioriPredictionObserveCurAfterResample(
       const int i_indexSensor);
    // 以上、故障検出アルゴリズム用
    
  public: // properties
    /**
     * 事前推定値の観測に対する尤度：p(y[k] | x(m)[k])のプロパティ(点推定アルゴリズムに対するインタフェース)
     *
     * @return  The likelihoods prior state to observe.
     *
     * ### value  ^likelihoodsPriorStateToObserve_のget/setを提供.
    **/
    property vector<double> LikelihoodsPriorStateToObserve_
    {
      vector<double> get(); 
      void set(vector<double> value);
    }

    /**
     * 事前推定値の観測に対する対数尤度：log(p(y[k] | x(m)[k]))のプロパティ.
     *
     * @return  The log likelihoods prior state to observe.
     *
     * ### value  ^likelihoodsPriorStateToObserve_のget/setを提供.
    **/
    property vector<double> LogLikelihoodsPriorStateToObserve_
    { 
      vector<double> get();
      void set(vector<double> value); 
    }

    /**
     * リサンプリングアルゴリズムのプロパティ.
     *
     * @return  The resampling.
     *
     * ### value  ^resampling_のsetを提供.
    **/
    property AbstractResampling^ Resampling_
    { 
      void set(AbstractResampling^ value);
    }

    /**
     * 点推定アルゴリズムのプロパティ.
     *
     * @return  The point estimation.
     *
     * ### value  ^pointEstimation_のsetを提供.
    **/
    property AbstractPointEstimation^ PointEstimation_
    { 
      void set(AbstractPointEstimation^ value); 
    }

  public:
    /** パーティクル予測時乱数分布(プロセルモデル誤差とは別に用意) */
    ProbabilityDistribution^ distRandomPredict_;

    /** 観測状態. */
    int stateObserve_ = 0;

    /** 逸脱度. */
    double deviance_ = 0;
    double likelihoodsH0_ = 0;
    double likelihoodsH1_ = 0;

  private:  // fields
    /** 事前推定値から予測した観測値: yhat-[k] = h(xhat-[k]) */
    vector<cv::Mat>* vecPriorPredictionObserveCur_;

    /** 事前推定値の観測に対する尤度：p(y[k] | x(m)[k]) */
    vector<double>* likelihoodsPriorStateToObserve_;

    /** 事前推定値の観測に対する対数尤度：log(p(y[k] | x(m)[k])) */
    vector<double>* logLikelihoodsPriorStateToObserve_;

    /**
     * 正規化前の対数重み: w(m)[k] = w(m)[k - 1] * h(m)(y[k] | xhat - (m)[k])
     *               　　: log{w(m)[k]} = log{w(m)[k-1]} + log{h(m)(y[k]|xhat-(m)[k])}
     *               　　: 現時刻対数重み ∝ (一時刻前対数重み) * (現時刻対数尤度)
    **/
    vector<double>* logWeightsBeforeNormal_;

#if 0
    /** 正規化前の最小対数重み. */
    double logWeightsBeforeNormalMin_;
#endif
    /** 正規化前の最大対数重み. */
    double logWeightsBeforeNormalMax_;

    /** リサンプリングアルゴリズム. */
    AbstractResampling^ resampling_;

    /** 点推定アルゴリズム. */
    AbstractPointEstimation^ pointEstimation_;

    // 以下、故障検出アルゴリズム用
    /** リサンプリング後推定値から予測した観測値: yhat-[k] = h(xhat-[k]) */
    vector<cv::Mat>* vecPosterioriPredictionObserveCur_;

    vector<double>* posterioriEstimationBeforeResample_;
    // 以上、故障検出アルゴリズム用

  };
}
