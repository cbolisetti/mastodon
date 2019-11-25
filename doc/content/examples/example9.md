# Example 9: Fault tree analysis using the Python module in MASTODON

This example demonstrates the use of the fault tree analysis and risk assessment
in MASTODON using the FTA Python module.

## Simple fault tree analysis with risk values
Activated by `analysis='Risk'`
Will assume that the distributions or the points estimates provided by the
`basic_events` parameter are risk values. If distributions are provided it is
assumed that they correspond to the risk distributions.


## Fragility
Activated by `analysis='Fragility'`
Will assume that the distributions provided by the `basic_events` parameter
correspond to the fragilities of the basic events.

### Approach 1
Basic event fragilities are first convolved with the hazard and the risk values
for each basic event are calculated. The risk values are then propagated
through the fault tree.

### Approach 2
Basic event fragilities are propagated through the fault tree first and the top
event fragility is calculated. The top event fragility is convolved with the
hazard curve to calculate the risk.

### Lite
Activated by `lite=True`
Will only perform fragility analysis with approach 2 and the importance measures
are not calculated. Results files are also not created. Good for situations in
which, we only need the top risk with approach 2. 

!bibtex bibliography
