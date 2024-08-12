import simplekml
import csv
kml = simplekml.Kml()
style = simplekml.Style()
style.labelstyle.color = simplekml.Color.red  # Make the text red
style.labelstyle.scale = 2  # Make the text twice as big

file = open('/home/groundstation/github/Rover_2023_2024/software/react_groundstation/ros_packages/gps-mapper/gps-mapper/coords.csv', 'r')
csv_reader = csv.reader(file)

linestring = kml.newlinestring(name="A Line")
coords = []
for row in csv_reader:
    
    coords.append((row[1], row[0]))
    
linestring.coords = coords
kml.save("/home/groundstation/github/Rover_2023_2024/software/react_groundstation/ros_packages/gps-mapper/gps-mapper/Point Shared Style.kml")