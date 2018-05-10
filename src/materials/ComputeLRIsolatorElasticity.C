// MASTODON includes
#include "ComputeLRIsolatorElasticity.h"

// MOOSE includes
#include "MooseMesh.h"
#include "Assembly.h"
#include "NonlinearSystem.h"
#include "MooseVariable.h"
#include "RankTwoTensor.h"

// libmesh includes
#include "libmesh/quadrature.h"
#include "libmesh/utility.h"

registerMooseObject("MastodonApp", ComputeLRIsolatorElasticity);

template <>
InputParameters
validParams<ComputeLRIsolatorElasticity>()
{
  InputParameters params = validParams<Material>();
  params.addClassDescription("Compute the forces and the stiffness matric corresponding to a two-noded isolator element.");
  // Switches
  params.addParam<bool>("cavitation", false, "Switch for modeling cavitation and post-cavitation.");
  params.addParam<bool>("buckling_load_variation", false, "Switch for modeling buckling load variation during the analysis.");
  params.addParam<bool>("horizontal_stiffness_variation", false, "Switch for modeling variation of horizontal stiffness during the analysis.");
  params.addParam<bool>("vertical_stiffness_variation", false, "Switch for modeling variation of vertical stiffness during the analysis.");
  params.addParam<bool>("strength_degradation", false, "Switch for modeling strength degradation due to lead core heating.");           // Strength degradation due to heating
  // Material properties (see pg 117)
  params.addRequiredParam<Real>("fy", "Yield stress of the bearing.");
  params.addRequiredParam<Real>("alpha", "Yield displacement of the bearing.");
  params.addRequiredParam<Real>("G_rubber", "Shear modulus of rubber.");
  params.addRequiredParam<Real>("K_rubber", "Bulk modulus of rubber.");
  params.addRequiredParam<Real>("D1", "Diameter of lead core.");
  params.addRequiredParam<Real>("D2", "Outer diameter of the bearing.");
  params.addRequiredParam<Real>("ts", "Thickness of a single steel shim.");
  params.addRequiredParam<Real>("tr", "Thickness of a single rubber layer.");
  params.addRequiredParam<Real>("n", "Number of rubber layers.");
  params.addRequiredParam<Real>("tc", "Thickness of the rubber cover of the bearing.");
  params.addParam<Real>("rho_lead", 11200.0, "Density of lead. Defaults to 11200 kg/m3.");
  params.addParam<Real>("c_lead", 130.0, "Specific heat capacity of lead. Defaults to 130.0 N-m/kg oC.");
  params.addParam<Real>("k_steel", 50.0, "Thermal conductivity of steel. Defaults to 50.0 W/(m-oC).");
  params.addParam<Real>("a_steel", 1.41e-05, "Thermal diffusivity of steel. Defaults to 1.41e-05 m2/s.");
  params.addParam<Real>("kc", 10.0, "Cavitation parameter.");
  params.addParam<Real>("phi_m", 0.5, "Damage index.");
  params.addParam<Real>("ac", 1.0, "Strength degradation parameter.");
  // params.addRequiredParam<Real>("mass", "Bearing mass.");
  params.addParam<Real>("cd", 0.0, "Viscous damping parameter.");
  params.set<MooseEnum>("constant_on") = "ELEMENT"; // _qp = 0
  return params;
}

ComputeLRIsolatorElasticity::ComputeLRIsolatorElasticity(const InputParameters & parameters)
  : Material(parameters),
    _cavitation(getParam<bool>("cavitation")),
    _buckling_load_variation(getParam<bool>("buckling_load_variation")),
    _horizontal_stiffness_variation(getParam<bool>("horizontal_stiffness_variation")),
    _vertical_stiffness_variation(getParam<bool>("vertical_stiffness_variation")),
    _strength_degradation(getParam<bool>("strength_degradation")),
    _fy(getParam<Real>("fy")),
    _alpha(getParam<Real>("alpha")),
    _Gr(getParam<Real>("G_rubber")),
    _Kr(getParam<Real>("K_rubber")),
    _d1(getParam<Real>("D1")),
    _d2(getParam<Real>("D2")),
    _ts(getParam<Real>("ts")),
    _tr(getParam<Real>("tr")),
    _n(getParam<Real>("n")),
    _tc(getParam<Real>("tc")),
    _rhoL(getParam<Real>("rho_lead")), //qL in opensees
    _cL(getParam<Real>("c_lead")),
    _kS(getParam<Real>("k_steel")),
    _aS(getParam<Real>("a_steel")),
    _kc(getParam<Real>("kc")),
    _phi_m(getParam<Real>("phi_m")),
    _ac(getParam<Real>("ac")),
    _cd(getParam<Real>("cd")),
    // _sD(getMaterialPropertyByName<Real>("sd_ratio")), // Shear distance ratio
    _sD(0.5),
    _basic_def(getMaterialPropertyByName<ColumnMajorMatrix>("deformations")),
    _basic_vel(getMaterialPropertyByName<ColumnMajorMatrix>("deformation_rates")),
    _Fb(declareProperty<ColumnMajorMatrix>("basic_forces")),
    _Fl(declareProperty<ColumnMajorMatrix>("local_forces")),
    _Fg(declareProperty<ColumnMajorMatrix>("global_forces")),
    _Kb(declareProperty<ColumnMajorMatrix>("basic_stiffness_matrix")),
    _Kl(declareProperty<ColumnMajorMatrix>("local_stiffness_matrix")),
    _Kg(declareProperty<ColumnMajorMatrix>("global_stiffness_matrix")),
    _total_gl(getMaterialPropertyByName<ColumnMajorMatrix>("total_global_to_local_transformation")),
    _total_lb(getMaterialPropertyByName<ColumnMajorMatrix>("total_local_to_basic_transformation")),
    _length(getMaterialPropertyByName<Real>("initial_isolator_length")),
    _pi(libMesh::pi)
    // _TL_trial(0.0),
    // _TL_commit(0.0)
{
  // Bearing material and geometric parameters
  _A = (_pi / 4.0) * ((_d2 + _tc) * (_d2 + _tc) - _d1 * _d1); // Bonded rubber area of bearing
  Real S = (_d2 * _d2 - _d1 * _d1) / (4 * _d2 * _tr); // Shape factor for case with lead core
  _Tr = _n * _tr; // height of rubber in the bearing
  _h = _Tr + (_n - 1) * _ts; // height of rubber + shims
  Real F; // Dimension modification factor
  if (_d1 == 0) // If no lead core, i.e., elastomeric bearing
    F = 1.0;
  else
  {
    Real r = _d2/_d1; // Outer to inner diameter ratio
    F = (r*r+1)/((r-1)*(r-1)) + (1+r)/((1-r)*log(r)); // Dimension modification factor
  }
  Real Ec = 1.0 / ((1.0 / (6 * _Gr * S * S * F)) + (4.0 / 3.0) * (1.0 / _Kr)); // Compressive modulus of elastomeric bearing
  Real Er = 3.0 * _Gr; // Elastic modulus of rubber (assuming nu = 0.5)
  Real I = (_pi / 64.0) * (pow((_d2 + _tc), 4) - pow(_d1, 4)); // Moment of inertia of bearing
  _rg = sqrt(I / _A); // Radius of gyration

  // Bearing shear behavior parameters
  _qYield0 = _fy * (1 - _alpha);
  _qYield = _qYield0; // This yield stress changes with time
  _ke = _Gr * _A / _Tr; // Stiffness of elastic component (due to rubber)
  _k0 = (1.0 / _alpha - 1.0) * _ke; // Initial stiffness of hysteretic component (due to lead)

  // Axial parameters: compression
  Real Erot = Ec / 3.0; // Rotation modulus of bearing
  Real As = _A * _h / _Tr; // Adjusted shear area of bearing
  Real Is = I * _h / _Tr; // Adjusted moment of intertia of bearing
  Real Pe = _pi * _pi * Er * Is / (_h * _h); // Euler buckling load of bearing
  _kv0 = _A * Ec / _Tr; // Pre-cavitation tensile stiffness at zero lateral displacement
  _kv = _kv0; // Pre-cavitation stiffness initialized to that at zero displacement
  _Fcr = -sqrt(Pe * _Gr * As); // Critical buckling load in compression
  _Fcrn = _Fcr; // Current value of critical buckling load
  _Fcrmin = _Fcr; // Initial value of critical buckling load during loading
  _ucr = _Fcr / _kv0; // Initial value of critical buckling deformation
  _ucrn = _ucr; // Current value of critical buckling deformation

  // Axial parameters: tension
  _Fc = 3.0 * _Gr * _A; // Force that initiates cavitation
  _Fcn = _Fc; // Initial value of cavitation force (will be updated each time step)
  _uc = _Fc / _kv0; // Deformation at which cavitation is first initiated
  _ucn = _uc; // Initial value of cavitation deformation (will be updated each time step)
  _Fmax = _Fc; // Initial value of maximum tensile force (will be updated each time step)
  _umax = _uc;  // Initial value of maximum tensile deformation (will be updated each time step)
  // Real qTrial = _kv * _basic_def[_qp](0, 0);
  // if (_Kl < DBL_EPSILON) {
  //     kc = 0.0001;                                        // cavitation parameter
  // } else {
  //     kc = _Kl;                                            // cavitation parameter
  // }


}

void
ComputeLRIsolatorElasticity::initializeLRIsolator()
{
  Real I = (_pi / 64.0) * (pow((_d2 + _tc), 4) - pow(_d1, 4)); // Moment of inertia of bearing

  Real Is = I * _h / _Tr; // Adjusted moment of intertia of bearing
  Real Er = 3.0 * _Gr; // Elastic modulus of rubber (assuming nu = 0.5)


  // Initializing stiffness matrices
  // _qp = 0;
  std::cout << "elasticity****QUADRATURE POINT IS: " << _qp << "*******************\n";
  _Kb[_qp].reshape(6, 6);
  _Kb[_qp].zero();
  _Kb[_qp](0, 0) = _kv0;
  _Kb[_qp](1, 1) = _k0 + _ke;
  _Kb[_qp](2, 2) = _k0 + _ke;
  _Kb[_qp](3, 3) = _Gr * (2 * Is) / _h; // torsional stiffness
  _Kb[_qp](4, 4) = Er * Is / _h; // rotational stiffness
  _Kb[_qp](5, 5) = Er * Is / _h; // rotational stiffness

  // Initializing forces
  _Fb[_qp].reshape(6, 1); // forces in the basic system
  _Fb[_qp].zero();
  _Fl[_qp].reshape(12, 1); // forces in local system (6+6 = 12dofs)
  _Fl[_qp].zero();
  _Fg[_qp] = _Fl[_qp]; // forces in global system

  // Making all stiffnesses in _Kb as ones
  _Kb[_qp].identity();
  _Kb[_qp](1, 2) = 1.0;
  _Kb[_qp](2, 1) = 1.0;
  _Fb[_qp] = _Kb[_qp] * _basic_def[_qp];
  std::cout << "**** Force vector basic*** = \n";
  _Fb[_qp].print();
  std::cout << "**** Stiffness matrix basic**** =\n";
  _Kb[_qp].print();
}

void
ComputeLRIsolatorElasticity::computeQpProperties()
{
  initializeLRIsolator();
  // Compute axial forces and stiffness terms
  // computeAxial();

  // Computing shear forces and stiffness terms
  // computeShear();

  // Add P-∆ effects
  // addPDeltaEffects();

  // Compute rotational stiffnesses
  // 3) get moment and stiffness in basic x-direction
  // _Fb[_qp](3, 0) = _Kb[_qp](3,3) * _basic_def[_qp](3, 0);
  // _Kb[_qp](3,3) = Kt;

  // 4) get moment and stiffness in basic y-direction
  // _Fb[_qp](4, 0) = _Kb[_qp](4,4) * _basic_def[_qp](4, 0);
  // _Kb[_qp](4,4) = Kr;

  // 5) get moment and stiffness in basic z-direction
  // _Fb[_qp](5, 0) = _Kb[_qp](5, 5) * _basic_def[_qp](5, 0);
  // _Kb[_qp](5, 5) = Kr;

  // Finalize forces and stiffness matrix
  // and convert them into global co-ordinate system
  finalize();
}

void
ComputeLRIsolatorElasticity::computeAxial()
{
  // 1) get axial force and stiffness in basic x-direction
  // If buckling

  if (_basic_def[_qp](0, 0) <= _ucrn)
  {
    if (_buckling_load_variation)
    {
      _Kb[_qp](0,0) = _kv / 1000.0;
      _Fb[_qp](0, 0) = _Fcrmin + _Kb[_qp](0,0) * (_basic_def[_qp](0, 0) - _ucrn);
    }
    else
    {
      _Kb[_qp](0,0) = _kv;
      _Fb[_qp](0, 0) = _Kb[_qp](0, 0) * _basic_def[_qp](0, 0);
    }
  }

  _ucn = _Fcn / _kv; // cavitation deformation
  _Fmax = _Fc * (1.0 + (1.0 / (_Tr * _kc)) * (1.0 - exp(-_kc * (_umax - _uc))));

  if (_basic_def[_qp](0, 0) > _ucrn)
  {
    if (_cavitation) // cavitation effects
    {
      if (_basic_def[_qp](0, 0) <= _ucn)
      {
        _Kb[_qp](0, 0) = _kv;
        _Fb[_qp](0, 0) = _Kb[_qp](0, 0) * _basic_def[_qp](0, 0);
      }
      else if (_basic_def[_qp](0, 0) < _umax)
      {
        _Kb[_qp](0, 0) = (_Fmax - _Fcn) / (_umax - _ucn);
        _Fb[_qp](0, 0) = _Fcn + (_Fmax - _Fcn) / (_umax - _ucn) * (_basic_def[_qp](0, 0) - _ucn);
      }
      else // _basic_def[_qp](0, 0) > _umax
      {
        _Kb[_qp](0, 0) = (_Fc / _Tr) * exp(-_kc * (_basic_def[_qp](0, 0) - _uc));
        _Fb[_qp](0, 0) = _Fc * (1.0 + (1.0 / (_Tr * _kc)) * (1.0 - exp(-_kc * (_basic_def[_qp](0, 0) - _uc))));
        _umax = _basic_def[_qp](0, 0);
        _Fcn = _Fc * (1.0 - _phi_m * (1.0 - exp(-_ac * (_basic_def[_qp](0, 0) - _uc) / _uc)));
      }
    }
    else // cavitation effects not simulated
    {
      _Kb[_qp](0,0) = _kv;
      _Fb[_qp](0, 0) = _Kb[_qp](0, 0) * _basic_def[_qp](0, 0);
    }
  }
}

// void
// ComputeLRIsolatorElasticity::computeShear()
// {
//   // get the current temperature of the lead core
//   Real vel = sqrt(_basic_vel[_qp](1) * _basic_vel[_qp](1) + _basic_vel[_qp](2) * _basic_vel[_qp](2));
//   _TL_trial = getCurrentTemp(_qYield, _TL_commit, vel);
//
//   //2) calculate shear forces and stiffnesses in basic y- and z-direction
//   // get displacement increments (trial-commited)
//   Vector delta_ub = ub - ubC;
//   if (sqrt(pow(delta_ub(1),2)+pow(delta_ub(2),2)) > 0.0)
//   {
//
//       // get yield displacement
//       Real uy = qYield/k0;
//
//       // calculate hysteretic evolution parameter z using Newton-Raphson
//       unsigned int iter = 0;
//       unsigned int maxIter = 1000;
//       Real tol = 1E-6;
//       Real beta = 0.1; // note here beta and gamma are as per Nagarajaih(1991), which is opposite to Park et al.(1986)
//       Real gamma = 0.9;
//       Real zNrm, tmp1, tmp2, tmp3;
//       Vector f(2), delta_z(2);
//       Matrix Df(2,2);
//       do
//       {
//           zNrm = z.norm();
//           if (zNrm == 0.0)  // check because of negative exponents
//               zNrm = DBL_EPSILON;
//           tmp1 = beta + gamma*sgn(z(0)*delta_ub(1));
//           tmp2 = beta + gamma*sgn(z(1)*delta_ub(2));
//           tmp3 = z(0)*delta_ub(1)*tmp1 + z(1)*delta_ub(2)*tmp2;
//
//           // function and derivative
//           f(0) = z(0) - zC(0) - 1.0/uy*(delta_ub(1) - z(0)*tmp3);
//           f(1) = z(1) - zC(1) - 1.0/uy*(delta_ub(2) - z(1)*tmp3);
//
//           Df(0,0) = 1.0 + (1.0/uy)*(2*z(0)*delta_ub(1)*tmp1+z(1)*delta_ub(2)*tmp2);
//           Df(1,0) = (tmp1/uy)*z(1)*delta_ub(1);
//           Df(0,1) = (tmp2/uy)*z(0)*delta_ub(2);
//           Df(1,1) = 1.0 + (1.0/uy)*(z(0)*delta_ub(1)*tmp1+2*z(1)*delta_ub(2)*tmp2);
//
//           // issue warning if diagonal of derivative Df is zero
//           if ((fabs(Df(0,0)) <= DBL_EPSILON) || (fabs(Df(1,1)) <= DBL_EPSILON))
//           {
//               opserr << "WARNING: LeadRubberX::update() - "
//                   << "zero Jacobian in Newton-Raphson scheme for hysteretic "
//                   << "evolution parameter z.\n";
//               return -1;
//           }
//
//           // advance one step
//           // delta_z = f/Df; either write a function to do matrix devision or use the solution below
//           delta_z(0) = (f(0)*Df(1,1)-f(1)*Df(0,1))/(Df(0,0)*Df(1,1)-Df(0,1)*Df(1,0));
//           delta_z(1) = (f(0)*Df(1,0)-f(1)*Df(0,0))/(Df(0,1)*Df(1,0)-Df(0,0)*Df(1,1));
//           z -= delta_z;
//           iter++;
//       }
//       while ((delta_z.Norm() >= tol) && (iter < maxIter));
//
//       // issue warning if Newton-Raphson scheme did not converge
//       if (iter >= maxIter)
//       {
//           opserr << "WARNING: LeadRubberX::update() - "
//               << "did not find the hysteretic evolution parameters z after "
//               << iter << " iterations and norm: " << delta_z.Norm() << endln;
//           return -2;
//       }
//
//       // get derivative of hysteretic evolution parameter
//       Real du1du2 = delta_ub(1)/delta_ub(2);
//       Real du2du1 = delta_ub(2)/delta_ub(1);
//       if (delta_ub(1)*delta_ub(2) == 0)
//       {
//           du1du2 = 0.0;
//           du2du1 = 0.0;
//       }
//
//       dzdu(0,0) = (1.0/uy)*(1.0 - z(0)*(z(0)*tmp1+z(1)*tmp2*du2du1));
//       dzdu(0,1) = (1.0/uy)*(du1du2-z(0)*(z(0)*tmp1*du1du2+z(1)*tmp2));
//       dzdu(1,0) = (1.0/uy)*(du2du1-z(1)*(z(0)*tmp1+z(1)*tmp2*du2du1));
//       dzdu(1,1) = (1.0/uy)*(1.0 - z(1)*(z(0)*tmp1*du1du2+z(1)*tmp2));
//
//       tCurrent = (this->getDomain())->getCurrentTime();
//       Real dt = tCurrent - tCommit;
//
//       // set shear force
//       _Fb[_qp](1, 0) = cd*_basic_vel[_qp](1) + qYield*z(0) + ke*_basic_def[_qp](1, 0);
//       _Fb[_qp](2, 0) = cd*_basic_vel[_qp](2) + qYield*z(1) + ke*_basic_def[_qp](2, 0);
//       // set tangent stiffness
//       _Kb[_qp](1,1) = cd/dt + qYield*dzdu(0,0) + ke;
//       _Kb[_qp](1,2) = qYield*dzdu(0,1);
//       _Kb[_qp](2,1) = qYield*dzdu(1,0);
//       _Kb[_qp](2,2) = cd/dt + qYield*dzdu(1,1) + ke;
//   }
//
//   /* if buckling
//   if (tag==1) {
//       tag = 0;
//       return -1;  // return any negative integer
//   }*/
//
//   return 0;
// }

void
ComputeLRIsolatorElasticity::addPDeltaEffects()
{
  // add geometric stiffness to local stiffness
  _Kl[_qp](5, 1) -= 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](5, 7) += 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](11, 1) -= 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](11, 7) += 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](4, 2) += 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](4, 8) -= 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](10, 2) += 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](10, 8) -= 0.5 * _Fb[_qp](0, 0);
  _Kl[_qp](5, 5) += 0.5 * _Fb[_qp](0, 0) * _sD * _length[_qp];
  _Kl[_qp](11, 5) -= 0.5 * _Fb[_qp](0, 0) * _sD * _length[_qp];
  _Kl[_qp](4, 4) += 0.5 * _Fb[_qp](0, 0) * _sD * _length[_qp];
  _Kl[_qp](10, 4) -= 0.5 * _Fb[_qp](0, 0) * _sD * _length[_qp];
  _Kl[_qp](5, 11)  -= 0.5 * _Fb[_qp](0, 0) * (1.0 - _sD) * _length[_qp];
  _Kl[_qp](11, 11) += 0.5 * _Fb[_qp](0, 0) * (1.0 - _sD) * _length[_qp];
  _Kl[_qp](4, 10) -= 0.5 * _Fb[_qp](0, 0) * (1.0 - _sD) * _length[_qp];
  _Kl[_qp](10, 10) += 0.5 * _Fb[_qp](0, 0) * (1.0 - _sD) * _length[_qp];
}

// Real
// ComputeLRIsolatorElasticity::calculateCurrentTemperature(Real _qYield, Real _TL_commit, Real vel)
// {
//   // lead core heating
//   if (_t < _tCommit)
//     tCommit = 0.0;
//   Real a = _d1/2;
//   Real dt = _t - _tCommit;
//   // if (dt>1) dt = 0;
//   Real a_lead = _pi * pow(a, 2);
//   Real tau = (_aS * _tCurrent) / (pow(a, 2));
//   Real F;
//   if (tau < 0.6)
//   {
//     F = 2.0*sqrt(tau/_pi)-(tau/_pi)*(2.0-(tau/4.0)-pow(tau/4.0,2)-(15.0/4.0)*(pow(tau/4.0,3)));
//   }
//   else
//   {
//     F = 8.0/(3.0*_pi)-(1.0/(2.0*sqrt(_pi*tau)))*(1.0-(1.0/(12.0*tau))+(1.0/(6.0*pow(4.0*tau,2)))-(1.0/(12.0*pow(4.0*tau,3))));
//   }
//   Real deltaT1 = (dt/(_qL*_cL*_h))*((_qYield*vel*zC.norm())/a_lead-(_kS*_TL_commit/a)*(1.0/F+1.274*((_n-1)*_ts/a)*pow(tau,-1.0/3.0)));
//   if (deltaT1 <= 0.0) {
//       deltaT1 = 0.0;
//   }
//
//   // use improved euler method to obtain final temperature
//   Real TL_trial1 = _TL_commit + deltaT1;
//   Real _tCurrent2 = _tCurrent + dt;
//   tau = (_aS * tCurrent2) / (pow(a, 2));
//   if (tau < 0.6) {
//       F = 2.0*sqrt(tau/_pi)-(tau/_pi)*(2.0-(tau/4.0)-pow(tau/4.0,2)-(15.0/4.0)*(pow(tau/4.0,3)));
//   } else {
//       F = 8.0/(3.0*_pi)-(1.0/(2.0*sqrt(_pi*tau)))*(1.0-(1.0/(12.0*tau))+(1.0/(6.0*pow(4.0*tau,2)))-(1.0/(12.0*pow(4.0*tau,3))));
//   }
//   Real deltaT2 = (dt/(_qL*_cL*_h))*((_qYield*vel*zC.norm())/a_lead-(_kS*TL_trial1/a)*(1.0/F+1.274*((_n-1)*_ts/a)*pow(tau,-1.0/3.0)));
//   if (deltaT2 <= 0.0) {
//       deltaT2 = 0.0;
//   }
//
//   Real TL_trial = TL_commit + 0.5*(deltaT1+deltaT2);
//
//   return TL_trial;
// }

void
ComputeLRIsolatorElasticity::finalize()
{
  // Real uh = sqrt(_basic_def[_qp](1, 0) * _basic_def[_qp](1, 0) + _basic_def[_qp](2, 0) * _basic_def[_qp](2, 0));

  // vertical motion
  // if (_vertical_stiffness_variation)
  // {
  //   _kv = _kv0 * (1.0 / (1.0 + (3.0 / (_pi * _pi)) * (uh / _rg) * (uh / _rg)));
  //   if (uh > 0.0)
  //     _uc = _Fc/_kv;
  // }

  // compression
  // if (_buckling_load_variation) {
  //     Real Delta = 2.0*acos(uh/D2);   // this becomes undefined for uh/D2 > 1.0
  //     //Ar = (D2*D2/4.0)*(Delta-sin(Delta));
  //     Ar = ((D2+_tc)*(D2+_tc) - D1*D1)/4.0*(Delta-sin(Delta));  // A does not include lead core
  //     if (Ar/A < 0.2 || uh/D2 >= 1.0) {
  //         _Fcrn = 0.2*_Fcr;
  //     } else {
  //         _Fcrn = _Fcr*Ar/A;
  //     }
  //
  //     if (_Fcrn > _Fcrmin)
  //         _Fcrmin = _Fcrn;
  //     _ucrn = _Fcrn/_kv;
  // }

  // horizontal motion
  // if (_horizontal_stiffness_variation)
  // {
  //   _ke = (G*A/Tr)*(1.0-pow(_Fb[_qp](0, 0)/_Fcrn,2));
  //   //if (ke < 0) {
  //   //    ke = 0.01*(G*A/Tr);  // a fraction of ke to avoid convergence issues
  //   //    opserr << "WARNING LeadRubberX::commitState() - Negative horizontal stiffness\n";
  //   //}
  // }

  // lead core heating
  // _TL_commit = _TL_trial;
  // _tCommit = _t;
  // if (_strength_degradation) {
  //     _qYield = _qYield0 * exp(-0.0069 * _TL_commit);
  // }
  //
  // // commit trial history variables for horizontal direction
  // ubC = ub;
  // zC = z;

  // Converting forces from basic to local to global
  _Fl[_qp] = _total_lb[_qp].transpose() * _Fb[_qp]; // local forces
  _Fg[_qp] = _total_gl[_qp].transpose() * _Fl[_qp]; // global forces

  // Converting stiffness matrix from basic to local to global
  _Kl[_qp] = _total_lb[_qp].transpose() * _Kb[_qp] * _total_lb[_qp]; // convert basic stiffness to local
  // addPDeltaEffects(); // add P-∆ effects to local stiffness
  _Kg[_qp] = _total_gl[_qp].transpose() * _Kl[_qp] * _total_gl[_qp];

  std::cout << "**** Force vector global*** = \n";
  _Fg[_qp].print();
  std::cout << "**** Force vector local*** = \n";
  _Fl[_qp].print();
  std::cout << "**** Stiffness matrix global**** =\n";
  _Kg[_qp].print();
}
