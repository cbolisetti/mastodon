# Example 8: Seismic risk assessment of a generic nuclear facility using the MASTODON Python PRA module

## Model description

This example demonstrates the seismic risk assessment of a generic nuclear facility (GNF) located at the
INL site and subject to the INL site seismic hazard. The risk analysis is performed using the MASTODON's
FTA Python module that performs fault tree analysis and calculates the seismic risk.

The generic nuclear facility (GNF) represents a safety-critical facility owned and
operated by the Department of Energy (DOE). The GNF is assumed to host materials at risk (MAR) that
need to be contained in the facility and is classified as a Seismic Design category 3 (SDC 3) structure
per ASCE 43 [!citep](asce43-05). The containment structure of GNF is assumed to be the
primary containment and that the collapse of the structure will release the MAR
into the atmosphere leading to radiation exposure to collocated workers and off-site
receptors. Therefore, the seismic risk assessment of the GNF involves the calculation
of the collapse risk of the GNF structure.

The inputs for seismic risk assessment of GNF include the seismic hazard at the INL
site, the seismic collapse fragility of the GNF structure, and the event tree and
fault tree for the collapse. For this example, the seismic hazard curve is adopted
from the probabilistic seismic hazard analyses performed for the INL site. The
seismic fragilities, event tree, and fault trees are gathered from those of similar
facilities and are assumed to be representative of safety-critical DOE facilities.

### Seismic hazard

The seismic risk assessment of GNF is performed for two seismic hazard curves, both
corresponding to the INL site. The first hazard curve is calculated from the PSHA for
the INL site performed in 2006, and the second hazard curve corresponds to an updated
PSHA performed in 2015 [!citet](inl-ext-16-37751). The updated hazard curve is a result of using
new data and models for characterizing the seismic sources and the ground motions
at the target site, as described in [!citet](Payne2017b) and [!citet](Payne2017).
[fig:SH] presents the original and updated seismic hazard curves and shows that
the 2015 curve forecasts a smaller seismic hazard than the 2006 curve for most
spectral accelerations. For most hazard levels the implies the 2015 curve predicts
a smaller peak ground acceleration (PGA) and $Sa$(10Hz) values than the 2006 curve.

!media media/examples/Seismic_Hazard.png
       style=width:100%;margin-right:0px;float:center;
       id=fig:SH
       caption=Seismic hazard curves and the corresponding bin values for the 2006 and the 2015 assessments.

### Seismic fragility

Seismic fragility here refers to the probability of collapse of the GNF under
seismic shaking. It is assumed that the spectral acceleration at 10Hz is a
representative seismic demand for the GNF and the seismic fragility (as well as
the seismic hazard) is expressed in terms of $Sa$ at 10Hz. The fragility curve
and its uncertainty are expressed using a double lognormal distribution described
by the lognormal median (Am), and logarithmic standard deviations representing randomness
and uncertainty, $\beta_R$ and $\beta_U$, respectively. The median fragility
function, which is the collapse probability conditioned on the input ground spectral
acceleration at 10Hz, is used for the risk assessment in this example. This fragility
is calculated as:

\begin{equation}
\label{eq2}
P\big(collapse|Sa(10Hz)) = \Phi \Big(\frac{ln\big(Sa(10Hz)/A_m\big)}{\beta_C}\Big)
\end{equation}

where, $\beta_C$ is the composite logarithmic standard deviation calculated as $\sqrt{\beta_R^2 + \beta_U^2}$
and $\Phi$ is the standard normal cumulative distribution function.

### Event and fault trees for risk evaluation

The seismic failure case of the GNF can be expressed as a simple fault tree with
a single basic event, which is the collapse of the structure from a seismic event.
The event tree can also be expressed using a single event, where the collapse of
the structure leads to a breach in containment. The event tree and fault tree of
the GNF used in this example are presented in [fig:ET] and [fig:FT], respectively.

!media media/examples/ET.png
       style=width:80%;margin-right:0px;float:center;
       id=fig:ET
       caption=Event tree for each bin of the seismic hazard curve for both the 2006 and the 2015 assessments. The logic of the event trees across the bins is the same except for the initiating event.

!media media/examples/FT.png
       style=width:80%;margin-right:0px;float:center;
       id=fig:FT
       caption=Fault tree used for aggregating the seismic risk for both the 2006 and the 2015 assessments.

## Inputs to the MASTODON FTA python module

Seismic risk assessment of the GNF is performed separately for the two seismic
hazard curves. The seismic risk is calculated by splitting the hazard curve into
a number of bins, and calculating the risk of collapse for each bin. In this
example, both the seismic hazard curves in [fig:SH] are split into 10 bins. The
spectral acceleration corresponding to each bin is calculated as the geometric
mean of the spectral accelerations of the extents of the bin.

The inputs required for the seismic risk assessment using the FTA python module
include the seismic hazard curve, fault tree logic, and the seismic fragilities
of the basic events. The FTA module is currently limited to fault tree analyses,
and not event tree analyses. Since the event tree in this example comprises of a
single event, event tree analysis will not be required. The seismic hazard,
fault tree logic, and basic event fragilities can be either input directly as
python lists, or as csv files. For this example, they are provided as csv files.
The seismic hazard files are listed below. As seen in the files, the seismic
hazard is simply specified as pairs of intensity measure and the corresponding
mean annual frequency of exceedance.

!listing examples/ex08/seismic_hazard_2006.csv

!listing examples/ex08/seismic_hazard_2015.csv



## Results

As per the DOE order 420.1C, a 10 year re-evaluation of the seismic hazard at an
INL facility was conducted. Consequently, a re-evaluation of the seismic collapse
capacity of this facility was also made. This section presents and discusses the
seismic hazard, facility collapse fragility, and risk results.



### Facility collapse fragility function

As discussed previously, the fragility function is dependent on the median acceleration
capacity $(A_m)$ and the resultant standard deviation $(\beta_T)$. These parameters
are obtained from the separation-of-variables method presented in EPRI TR-103959
[!citep](EPRI2018). In this method, $A_m$ is expressed as:

\begin{equation}
\label{eq3}
A_m = F_C~F_{RS}~Sa_{REF}
\end{equation}

\noindent where, $F_C$ is the median capacity factor, $F_{RS}$ is the median structural
response factor, and $Sa_{REF}$ is the ground motion parameter for the reference
earthquake. The median capacity factor is dependent on the median elastic strength
capacity and the median inelastic energy absorption of the facility. The median
structural response factor depends on several attributes such as:

- Earthquake response spectrum shape
- Horizontal component peak response to account for the possibility that the response spectrum in one horizontal direction may be higher than the response spectrum in the other direction
- Vertical component response to account for the variability in vertical ground response spectrum
- Peak and valley variability
- Structure damping
- Structure frequency
- Structure mode shape
- Structure mode combination
- Time history simulation to account for any differences between the response spectra for the time histories and the intended target spectra
- Soil structure interaction
- Ground motion incoherence to account for differences in ground motion amplitudes at all the points under the structure foundation
- Earthquake component combination to account for the combination of structural responses due to the three components of ground motion input

The resultant standard deviation is computed by accounting for the standard deviations
in the capacity and the structural response factors:

\begin{equation}
\label{eq4}
\beta_T = \sqrt{\beta_{R,C}^2 + \beta_{R,RS}^2 + \beta_{U,C}^2 + \beta_{U,RS}^2}
\end{equation}

where, $\beta_{R,C}$ and $\beta_{R,RS}$ represent the randomness in the capacity
and the structural response, respectively, and $\beta_{U,C}$ and $\beta_{U,RS}$
represent the uncertainty in the capacity and the structural response, respectively.

!row!

!col! small=12 medium=6 large=6
!media media/examples/Fragility_2006.png
       style=width:100% id=fig:fra1
       caption=Mean, median, and 5-95 percentile fragility functions for the 2006 assessment.
!col-end!

!col! small=12 medium=6 large=6
!media media/examples/Fragility_2015.png
       style=width:100% id=fig:fra2
       caption=Mean, median, and 5-95 percentile fragility functions for the 2015 assessment.
!col-end!

!row-end!

[fig:fra1] and [fig:fra2] present the facility collapse fragility functions for
the 2006 and the 2015 seismic risk evaluations. For illustration, the mean and the
median functions, and the $5^{th}-95^{th}$ percentile functions around the median
are presented. While the mean function considers both the randomness and uncertainty
around $A_m$, the median and the $5^{th}-95^{th}$ consider only the randomness around
$A_m$. Peak Ground Acceleration is used as the ground motion parameter. It is noted
that for the 2015 evaluation, the collapse capacity of facility increases when compared
with the 2006 evaluation. This increment in capacity can be attributed to several
factors. First, as noted previously, the seismic hazard has reduced from 2006 to 2015
for most frequencies of exceedance ([fig:SH]), which results in a design response
spectrum with lower amplitudes, and therefore, the selected ground motions for structural
response analysis. Second, the concrete masonry unit walls of the facility have been
strengthened post 2006. Third, a structural re-evaluation of the facility was performed
in 2015. These three factors have contributed to the lower facility collapse probability
in 2015 than in 2006.      

### Risk assessment

The seismic hazard curve and the facility fragility function are combined using the
fault tree of [fig:FT] to compute the risk of collapse. Both the aggregated
risk across all the ground motion levels and the conditional risk given a ground motion
level can be computed. Figure [fig:Risk] presents the conditional risk in terms of
the collapse frequency. Because the seismic hazard at the site and the vulnerability
reduced in 2015, the collapse risk also reduced in 2015 for most $Sa(10Hz)$ levels.
However, at extremely large $Sa(10Hz)$ values, since the seismic hazard increased
in 2015, the risk has also slightly increased. In 2006 and 2015 the aggregated facility
collapse risk is $4.461e-05$ and $1.671e-06$, respectively.

!media media/examples/Risk_PGA.png
       style=width:100%;margin-right:0px;float:center;
       id=fig:Risk
       caption=Conditional risk curves and the corresponding bin values for the 2006 and the 2015 assessments.


!bibtex bibliography
