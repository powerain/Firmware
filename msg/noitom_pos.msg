uint64 timestamp		# time of this estimate, in microseconds since system start
uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0 

# GPS position in WGS84 coordinates.
int32 lat			# Latitude in 1E-7 degrees
int32 lon			# Longitude in 1E-7 degrees 
int32 alt			# Altitude in 1E-3 meters above MSL, (millimetres)
