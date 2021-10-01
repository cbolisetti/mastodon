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

#include "PeakValueAux.h"

registerMooseObject("MastodonApp", PeakValueAux);

InputParameters
PeakValueAux::validParams()
{
  InputParameters params = AuxKernel::validParams();
  params.addClassDescription("Computes the peak of the variable in time and stores it in a field variable.");
  params.addRequiredCoupledVar("param_variable", "Variable of which the peak is calculated");
  return params;
}

PeakValueAux::PeakValueAux(const InputParameters & parameters)
  : AuxKernel(parameters),
    _var_old(coupledValueOld("param_variable")),
    _var(coupledValue("param_variable")),
    _u_old(uOld())
{
}

Real
PeakValueAux::computeValue()
{
  if (!isNodal())
    mooseError("must run on a nodal variable");

  if (_dt == 0)
    return abs(_var[_qp]);

  if (_u_old[_qp] < abs(_var[_qp]))
    return abs(_var[_qp]);
  else
    return _u_old[_qp];
}
