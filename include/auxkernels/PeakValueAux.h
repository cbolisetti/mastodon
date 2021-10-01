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

#pragma once

#include "AuxKernel.h"

class PeakValueAux : public AuxKernel
{
public:
  static InputParameters validParams();

  /**
   *Computes Acceleration using Newmark Time integration scheme
   */
  PeakValueAux(const InputParameters & parameters);

  virtual ~PeakValueAux() {}

protected:
  virtual Real computeValue();

  const VariableValue & _var_old;
  const VariableValue & _var;
  const VariableValue & _u_old;
};
