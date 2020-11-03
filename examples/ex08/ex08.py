# This code executes the faulttree analyses of example 8

# Importing the FTA module
from mastodonutils import FTA

# Executing the quantification class
quant2006 = FTA.Quantification("gnf_2006",
                           logic='logic.csv',
                           basic_events='bas_events_2006.csv',
                           analysis='Fragility',
                           hazard='seismic_hazard_2006.csv',
                           IM=[0.04, 0.11, 0.21, 0.42, 0.64, 0.87, 1.09, 1.52, 3.26, 4.34, 6.51],
                           lite=True,
                           nbins=10,
                           write_output=True)
