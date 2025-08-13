#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math

def linspace(a, b, n):
    if n <= 1:
        return [a]
    step = (b - a) / float(n - 1)
    return [a + i * step for i in range(n)]

def make_grid(n=40, size=2.0, z=0.0):
    """ Crea una grilla n x n en el plano XY (z constante). """
    half = size / 2.0
    xs = linspace(-half, half, n)
    ys = linspace(-half, half, n)
    pts = [(x, y, z) for x in xs for y in ys]
    return pts

def make_ring(n=360, r=1.0, z=0.2):
    """ Crea un anillo (circunferencia) para ver algo 'curvo' en RViz. """
    pts = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append((r * math.cos(a), r * math.sin(a), z))
    return pts

def main():
    rospy.init_node("reto_pointcloud_publisher", anonymous=True)

    pub = rospy.Publisher("/reto_pointcloud2", PointCloud2, queue_size=1)

    frame_id = rospy.get_param("~frame_id", "base_link")
    rate_hz  = float(rospy.get_param("~rate", 5))
    shape    = rospy.get_param("~shape", "grid")   # "grid" o "ring"
    n        = int(rospy.get_param("~n", 40))
    size     = float(rospy.get_param("~size", 2.0))

    if shape == "ring":
        points = make_ring(n=max(20, n), r=size/2.0, z=0.2)
    else:
        points = make_grid(n=n, size=size, z=0.0)

    rate = rospy.Rate(rate_hz)
    header = Header(frame_id=frame_id)

    rospy.loginfo("Publicando PointCloud2 en /reto_pointcloud2 (%s) a %.1f Hz, frame: %s",
                  shape, rate_hz, frame_id)

    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        msg = pc2.create_cloud_xyz32(header, points)
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
