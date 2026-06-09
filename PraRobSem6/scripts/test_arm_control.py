import subprocess
x, y, z = map(float, input("x, y, z odvojeno razmacima: ").split())

coords_str = "{x: " + str(x) + ", y: " + str(y) + ", z: " + str(z) +"}"

subprocess.run(["ros2", "topic", "pub", "/marker_target", "geometry_msgs/Point", coords_str, "--once"])
