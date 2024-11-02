import cv2

gray = cv2.cvtColor(cv2.imread("/home/siddharth/fall_ws/image.png"), cv2.COLOR_BGR2GRAY)


# Define the aruco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
parameters = cv2.aruco.DetectorParameters_create()

# Detect the markers in the image
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

if ids is not None:
    # current_pose.marker_ids = ids.flatten().tolist()
    # self.current_pose.poses = []

    for corner in corners:
        # Calculate the center of the marker
        center_x = int((corner[0][0][0] + corner[0][2][0]) / 2)
        center_y = int((corner[0][0][1] + corner[0][2][1]) / 2)

        print("Center of the marker: ({}, {})".format(center_x, center_y))
        # Create a Pose object for the marker
        # pose = Pose()
        # pose.position.x = center_x
        # pose.position.y = center_y
        # pose.position.z = 0.0  # Assuming the marker is on a flat surface

        # self.current_pose.poses.append(pose)