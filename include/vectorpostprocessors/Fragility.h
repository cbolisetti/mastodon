/*************************************************/
/*           DO NOT MODIFY THIS HEADER           */
/*                                               */
/*                     MASTODON                  */
/*                                               */
/*    (c) 2015 Battelle Energy Alliance, LLC     */
/*            ALL RIGHTS RESERVED                */
/*                                               */
/*   Prepared by Battelle Energy Alliance, LLC   */
/*     With the U. S. Department of Energy       */
/*                                               */
/*     See COPYRIGHT for full restrictions       */
/*************************************************/

#ifndef FRAGILITY_H
#define FRAGILITY_H

// MOOSE includes
#include "GeneralVectorPostprocessor.h"

// Forward Declarations
class Fragility;

template <>
InputParameters validParams<Fragility>();

/**
 *  The `Fragility` VectorPostprocessor computes the seismic fragility of a component
 *  given the dynamic properties of the component, capacity distribution of the component,
 *  and seismic demands from the probabilistic simulations. `Fragility` operates
 *  in the following steps: 1) calculate the demand distribution for each hazard bin, 2) use
 *  demand distribution and capacity distribution and calculate the conditional probability of
 * failure
 *  in each bin, and 3) fit a lognormal distribution in the conditional probabilities to calculate a
 *  fragility. This kind of fragility is also referred to as 'enhanced fragility'.
 */

class Fragility : public GeneralVectorPostprocessor
{
public:
  Fragility(const InputParameters & parameters);
  virtual void initialize() override;
  virtual void execute() override;
  /**
   *  Function to calculate the vector of spectral demands at the frequency of
   *  the SSC.
   */
  std::vector<Real> calcDemandsFromFile(unsigned int bin);

protected:
  /// Name of the master file without extension
  const std::string & _master_file;

  /// Name of the multiapp corresponding to ground motion sampling
  const std::string & _hazard_multiapp;

  /// Name of the multiapp corresponding to the probabilistic simulations
  const std::string & _probabilistic_multiapp;

  /// Number of ground motions used in each intensity bin
  const unsigned int & _num_gms;

  /// Demand variable names of the ssc that is also column name in the output csv file
  const std::vector<std::string> & _demand_variable_names;

  /// Frequencies at which the spectral acceleration demand of the SSC is to be calculated
  const std::vector<Real> & _ssc_freq;

  /// Damping ratio of the ssc for spectral acceleration demand calculation
  const Real & _ssc_xi;

  /// Time step of the simulations
  const Real & _dtsim;

  /// Median capacity of the ssc as a function of the demand parameter
  const Real & _median_cap;

  /// Uncertainty in the ssc capacity as a lognormal standard deviation
  const Real & _beta_ssc_cap;

  /// Number of bins used in time-based assessment
  const unsigned int & _num_bins;

  /// Number of samples of simulations for each ground motion in each bin
  const unsigned int & _num_samples;

  /// Number of collapses required to calculate likelihood when using Baker's MLE.
  const unsigned int & _num_collapses;

  /// IM values that are used in the hazard curve
  const std::vector<Real> & _im_values;

  /// Limits (minimum, maximum) for median component fragility
  const std::vector<Real> & _median_fragility_limits;

  /// Limits (minimum, maximum) for lognormal standard deviation of the component fragility
  const std::vector<Real> & _beta_fragility_limits;

  /// Intensity measures corresponding to the bins for time-based assessment (calculated by the HazardCurve object)
  VectorPostprocessorValue & _im;

  /// Median demand on the ssc
  VectorPostprocessorValue & _median_demand;

  /// Lognormal standard deviation of the demand of the ssc
  VectorPostprocessorValue & _beta_demand;

  /// Conditional failure probability of the SSCs
  VectorPostprocessorValue & _conditional_pf;

  /// Median of the fitted fragility
  VectorPostprocessorValue & _median_fragility;

  /// Beta of the fitted fragility
  VectorPostprocessorValue & _beta_fragility;

  /// Method for optimization
  const bool & _brute_force;

  /// Tolerance for Stochastic Gradient Descent
  const Real _sgd_tolerance;

  /// Parameter controlling step size for Stochastic Gradient Descent
  const Real _sgd_gamma;

  /// Number of random initializations for Stochastic Gradient Descent
  const int _sgd_numrnd;

  /// Seed for the random number generator
  const int _sgd_seed;

  /// Demand calculation type (peak or average)
  const std::string _demand_calc_type;
};

#endif
