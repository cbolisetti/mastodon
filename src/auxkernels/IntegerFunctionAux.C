//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

#include "IntegerFunctionAux.h"
#include "Function.h"

registerMooseObject("MastodonApp", IntegerFunctionAux);

defineLegacyParams(IntegerFunctionAux);

InputParameters
IntegerFunctionAux::validParams()
{
  InputParameters params = AuxKernel::validParams();
  params.addClassDescription("Auxiliary Kernel that creates and updates a field variable by "
                             "sampling a function through space and time.");
  params.addRequiredParam<FunctionName>("function", "The function to use as the value");
  params.addParam<bool>("integerize", false, "If true, will integerize and abs the function value.");
  return params;
}

IntegerFunctionAux::IntegerFunctionAux(const InputParameters & parameters)
  : AuxKernel(parameters),
  _func(getFunction("function")),
  _integerize(getParam<bool>("integerize"))
{
}

Real
IntegerFunctionAux::computeValue()
{
  Real func_value;
  if (isNodal())
  {
    func_value = _func.value(_t, *_current_node);
    return (_integerize ? round(func_value) : func_value);
  }
  else
  {
    func_value = _func.value(_t, _current_elem->vertex_average());
    return (_integerize ? round(func_value) : func_value);
  }
}
