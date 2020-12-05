import math
import time
import csv
from datetime import datetime,timedelta
import ephem
import sys
degrees_per_radian = 180.0 / math.pi
from math import degrees

TLE0 = 'LS1'
TLE1=  '1 44109U 19018AF  20327.47693938  .00004175  00000-0  13256-3 0  9997'
TLE2=  '2 44109  97.4496  37.6999 0057930  11.5295 348.7258 15.31595990 91766'

# setting the sat here is not anymore needed, will be suppressed later
sat1 = ephem.readtle('LS1',
    '1 44109U 19018AF  20327.47693938  .00004175  00000-0  13256-3 0  9997',
    '2 44109  97.4496  37.6999 0057930  11.5295 348.7258 15.31595990 91766'
)

sat2 = ephem.readtle('LS2D',
    '1 46492U 20068G   20323.79362494 +.00000254 +00000-0 +23102-4 0  9998',
    '2 46492 097.6735 256.8338 0020788 068.7862 291.5584 15.03504381007686'
)
# ----------------------

home = ephem.Observer()
path=str(sys.argv[1])
path2 = str(sys.argv[2])
path_LS1 = "LS1_TLE.json"
path_LS2D = "LS2D_TLE.json"

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

# Compute day from date provided by TTN
def extract_day(date):
        temp = ""
        results = ""
        for letter in date:
                if letter == '-':
                        temp += '/'
                elif letter == 'T':
                        results =temp
                elif letter == 'Z':
                        temp = ""
                else:
                        temp += letter
        #print (results)
        return results

# Compute hour from date provided by TTN
def extract_hour(date):
        temp = ""
        results = ""
        for letter in date:
                if letter == '-':
                        temp += '/'
                elif letter == 'T':
                        temp = ""
                elif letter == '.':
                        results = temp
                else:
                        temp += letter
        #print (results)
        return results 

# TLE are not saved dayly in a separate file
# The function set the TLE depending on packet date and satellite number

def set_TLE(date, satnum):
        num_list = []
        TLE_ok=0
        path_sat=path_LS1
        if (satnum == 1):
                path_sat=path_LS1
        if (satnum ==2):
                path_sat=path_LS2D
        date = datetime.strptime(extract_day(date),'%Y/%m/%d') - timedelta(days=1) # set time one day before to limit error in table
        with open(path_sat, 'r') as fh:
            for line in fh:
                if TLE_ok == 3:
                        TLE2=line
                        #print(f'{line}')
                        break
                if TLE_ok == 2:
                        TLE1=line
                        TLE_ok=3
                        #print(f'{line}')
                if TLE_ok == 1:
                        TLE_ok=2
                        #print(f'{line}')    
                test_date = extract_day(line)
                if test_date == '': line = 'no'
                else :        
                        if date <= datetime.strptime(test_date,'%Y/%m/%d'):    
                                TLE_ok = 1
                                #print(extract_day(line))
        fh.close




# Open file with received packet and calculate coordinate

with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter='\t')
	line_count = 0
	line_sat= 2
	sat_num = 1
	str1 = list()
	str1.append("Time,Day,Hour,Elevation,Azymuth,Range,RSSI,Latitude,Longitude,sat,cnt")
	for row in csv_reader:
                #print(f'Number of row is {len(row)} ')
                if len(row)==5:
                        sat_num = str(row[4])
                        #print(f'sat_num is {sat_num}')
                if row[2] == 'eui-f01898219e90f018':
                        set_TLE(str(row[0]),sat_num)
                        sat = ephem.readtle(TLE0, TLE1, TLE2)
                        home.date = format_date(str(row[0]))
                        day = extract_day(str(row[0]))
                        hour = extract_hour(str(row[0]))
                        if sat_num == '1':
                                sat = sat1
                        else:
                                sat = sat2      
                        sat.compute(home)
                        el = sat.alt * degrees_per_radian
                        az = sat.az * degrees_per_radian
                        lat = degrees (sat.sublat)
                        lon = degrees (sat.sublong)
                        str2 = f'\t{home.date}   {el}   {sat.range}   {row[3]}   {sat_num}'
                        str3 = f'\t{home.date},{day},{hour},{el},{az},{sat.range},{row[3]},{lat},{lon},{sat_num},{row[1]}'
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

    



    

