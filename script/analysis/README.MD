# bebop_hri Data Analysis Scripts

Extracting useful data for each experiment from bag files as CSV files for analysis.

## Depth Estimation

``extract.sh`` in ``depth`` folder extraxts three CSV files for each bag, target and robot's vicon data for calculating the depth ground-truth and robot's ``vservo`` topic for the estimated depth by system.
``extract.sh`` gets two arguments, first argument is the path to folder containing all the bags that should be proccessed, second argument is the path to destination folder for saving all the CSV files.
For IROS16 we had 2 different experiments, with and without detection for each frame, use the Matlab code provided in ``detect`` and ``track`` folder for drawing the depth error/distance charts.
Palce all the extracted CSV files in corresponding folder (use second argument of ``extract.sh`` bash script), then enter the base name of CSV files that you want to be proccessed in ``files.txt`` (remove ``_*.csv`` from them) then run ``draw_all.m`` Matlab script.

## Trajectory

``extract.sh`` in ``trajectory`` folder extraxts three CSV files for each bag, target and robot's vicon data, robot's ``land`` topic for detectting when the robot has reach the destination and decided to land, and behavior's ``status`` data to know when robot started servoing.
``extract.sh`` gets two arguments, first argument is the path to folder containing all the bags that should be proccessed, second argument is the path to destination folder for saving all the CSV files.
Palce all the extracted CSV files in corresponding folder (use second argument of ``extract.sh`` bash script), then enter the base name of CSV files that you want to be proccessed in ``files.txt`` (remove ``_*.csv`` from them) in the subfolder containing CSV files (``track`` or ``detect``) then run ``draw_all_trajectories.m`` Matlab script for drawing track and detect trajectories side by side or ``draw_all_targets.m`` Matlab script for drawing track and detect final position in which they decided to land.

## GPS

Use ``extract.sh`` in ``gps`` folder to extract the robots GPS data, usage is same as previous ones, gets two arguments, first argument is the path to folder containing all the bags that should be proccessed, second argument is the path to destination folder for saving all the CSV files.

!!! This script does not work for expteriments in which we have used cellphone to capture target's GPS position due to android ros bug, for does play the bag manually and extract the GPS toppic using following command: !!!

```bash
$ rostopic echo -p /topic/name > filename.csv
```

## Velocity Visualizer

Tool for visualizing the robot speed based on ground-truth (vicon data) for seeing the velocity controller output visualy.
In ``vel`` folder, ``process_bag.sh`` process the bags using ``post_process.py``, creates a temporary bag, extracts the vicon data and bebop speed from temporary bag, and deletes the temp bag.
``process_bag.sh`` also gets two arguments same as previous ones, first argument is the path to folder containing all the bags that should be proccessed, second argument is the path to destination folder for saving all the CSV files.
Then using ``all_tests.m`` Matlab script, draws the speed profile for all the CSV files that are in the same folder as the script. (!!! Messy code, should update the way it reads all the files like other scripes from a ```files.txt``` not eveything from the folder as is right now !!!)
