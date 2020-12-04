# ChArUco

Launch file: `launch/charcuo.launch`

This launch file only launches `localization/charuco.py`, which publishes:
- Marker pose: `charuco/marker_pose`, likely to be used for goal
- Robot pose, transformed to base_link (can be commented out if no base_link exists): `charuco/rover_pose`
- Other various images and data for debugging.
