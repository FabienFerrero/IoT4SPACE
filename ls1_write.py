import math
import time
import csv
from datetime import datetime
import ephem
import sys
degrees_per_radian = 180.0 / math.pi
from math import degrees

sat = ephem.readtle('LS1',
    '1 44109U 19018AF  20327.47693938  .00004175  00000-0  13256-3 0  9997',
    '2 44109  97.4496  37.6999 0057930  11.5295 348.7258 15.31595990 91766'
)

home = ephem.Observer()
path=str(sys.argv[1])
path2 = str(sys.argv[2])
home.lon = '7.12'   # +E
home.lat = '43.58'      # +N
home.elevation = 10 # meters


# Function to format the date for piephem
def format_date(date):
        results = ""
        for letter in date:
                if letter == '-':
                        results += '/'
                elif letter == 'T':
                        results += ' '
                elif letter == 'Z':
                        results += '\''
                else:
                        results += letter
        #print (results)
        return results        



# Open file with received packet and calculate coordinate

with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter='\t')
	line_count = 0
	line_sat= 2
	str1 = list()
	str1.append("Time,Elevation,Azymuth,Range,RSSI,Latitude,Longitude")
	for row in csv_reader:
		if row[2] == 'eui-f01898219e90f018':
                        home.date = format_date(str(row[0]))
                        sat.compute(home)
                        el = sat.alt * degrees_per_radian
                        az = sat.az * degrees_per_radian
                        lat = degrees (sat.sublat)
                        lon = degrees (sat.sublong)
                        str2 = f'\t{home.date} {el} {az} {sat.range} {row[3]}'
                        str3 = f'\t{home.date},{el},{az},{sat.range},{row[3]},{lat},{lon}'
                        print(str2)
                        str1.append(str3)
                        # filout.write("{}\n".format(str1))
                        line_count += 1
                        line_sat += 1
		else:
			#print(f'\t{row[0]}  RSSI: {row[3]} EL : {el}')
			line_count += 1
	print(f'Processed {line_count} lines.')
	
csv_file.close


# Write results in path2 file
with open(path2, "w") as filout:	
	for row in str1:
                filout.write(row + "\n")		
filout.close

    



    

