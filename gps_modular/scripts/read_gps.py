import gps
from math import sin, cos
import time
 
# Listen on port 2947 (gpsd) of localhost
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
initialized = False
N0 = 0
E0 = 0
lat = 40.89
lon = -90
lat_factor = cos(lat*3.14159/180.0)

deg2meters = 111111.11
lat_meters = 0
lon_meters = 0

init_count_limit = 10
init_lat_count = 0
init_lon_count = 0

disp_count = 0
 
while True:
    try:
    	report = session.next()
		# Wait for a 'TPV' report and display the current time
		# To see all report data, uncomment the line below
		# print report
        if report['class'] == 'TPV':
            disp_count = disp_count + 1
            if hasattr(report,'lon'):
                lon = report.lon
                if(init_lon_count < init_count_limit):
                    E0 = E0 + lon
                    print "long(E0): ", E0
                    init_lon_count = init_lon_count + 1
                    if(init_lon_count == init_count_limit):
                        E0 = E0/init_count_limit
                        print "long(E0): ", E0
                else:
                    lon_meters = deg2meters*lat_factor*(lon-E0)
            if hasattr(report,'lat'):
                lat = report.lat
                if (init_lat_count < init_count_limit):
                    N0 = N0 + lat
                    print "lat(N0): ", N0
                    init_lat_count = init_lat_count + 1
                    if(init_lat_count == init_count_limit):
                        N0 = N0/init_count_limit
                        print "lat(N0): ", N0
                else:
                    lat_factor = cos(lat*3.14159/180.0)
                    lat_meters = deg2meters*(lat-N0)
        
        #time.sleep(1.0)
        if(disp_count == 1):
            disp_count = 0
            print "lat(N), lon(E): ", lat_meters, lon_meters
    except KeyError:
		pass
    except KeyboardInterrupt:
		quit()
    except StopIteration:
		session = None
		print "GPSD has terminated"
