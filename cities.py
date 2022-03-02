# This file contains list of major cities and their coordinates (longitude-latitude)
# Mainly used for testing TSP and mTSP solvers
# Waypoints are in lat-long format
LOC_NAMES = ["DC", "NYC", "Cambridge MA", "Livingston NJ", "Pittsburgh PA",
             "Baltimore MD", "Philly PA", "Charlotte NC", "San Jose CA",
             "San Antonio CA", "Austin TX", "Chicago IL", "Phoenix AZ",
             "LA", "Seattle WA", "Denver CO", "Nashville TN", "Las Vegas NV",
             "Detroit MI", "Albuquerque NM", "Tucson AZ", "Kansas City MO",
             "Raleigh NC", "Miami FL", "Omaha NE", "Minneapolis MN",
             "Wichita KS", "New Orleans LA", "Cleveland OH", "Honolulu HI",
             "Lexington KY", "Cincinnati OH", "Saint Paul MN", "Fort Wayne IN",
             "Buffalo NY", "Norfolk VA", "Boise ID", "Des Moines IA",
             "Birmingham AL", "Salt Lake City UT"]

WAYPOINTS = [(38.907192, -77.036873), (40.712776, -74.005974),
             (42.365250, -71.105011), (40.783909, -74.314346),
             (40.440624, -79.995888), (39.299236, -76.609383),
             (39.952583, -75.165222), (35.227085, -80.843124),
             (37.279518, -121.867905), (29.387428, -98.496574),
             (30.266666, -97.733330), (41.881832, -87.623177),
             (33.448376, -112.074036), (34.052235, -118.243683),
             (47.608013, -122.335167), (39.742043, -104.991531),
             (36.174465, -86.767960), (36.114647, -115.172813),
             (42.331429, -83.045753), (35.106766, -106.629181),
             (32.253460, -110.911789), (39.099724, -94.578331),
             (35.787743, -78.644257), (25.761681, -80.181788),
             (41.257160, -95.995102), (44.986656, -93.258133),
             (37.697948, -97.314835), (29.951065, -90.071533),
             (41.505493, -81.681290), (21.315603, -157.858093),
             (38.047989, -84.501640), (39.103119, -84.512016),
             (44.954445, -93.091301), (41.093842, -85.139236),
             (42.880230, -78.878738), (36.850769, -76.285873),
             (43.618881, -116.215019), (41.619549, -93.598022),
             (33.543682, -86.779633), (40.758701, -111.876183)]

# sanity check
#assert(len(LOC_NAMES) == len(WAYPOINTS))

# location names dictionary
NAMES_DICT = {WAYPOINTS[i]: LOC_NAMES[i] for i in range(len(LOC_NAMES))}
