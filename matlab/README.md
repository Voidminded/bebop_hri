- Vicon tag: bebop_blue_en (rotate angles to match REP 103)
- cmd_vel is not stamped, use the provided python script to convert the bag file to a proccessed bag file with stamped cmd_vel and other required topics
- Play the processed bag file and run `bebop_iden.py` from `bebop_hri` package. That will dump out a CSV file. All positions are in world coordinate systems and angles are in Bebop's REP 103's local coordinate system.

