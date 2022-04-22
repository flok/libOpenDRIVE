# Copy the pyd file from the build folder in the same folder as this python file
import argparse
from build.Debug.PyOpenDrive import OpenDriveMap

# argument parser
parser = argparse.ArgumentParser(description="Test application using python binding of libOpenDRIVE")
parser.add_argument("xodr", type=str, help="Path to the to be loaded xodr file")
args = parser.parse_args()


test = OpenDriveMap(args.xodr) # load Viewer test xodr file
roads = test.get_roads()

for road in roads:
    print(f"road : {road.id}, length: {road.length}")

