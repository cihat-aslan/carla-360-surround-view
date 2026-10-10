"""
ROS2 köprüsü: surround_view.py çıktısını ROS2 topic'lerine yayınlar.
Yalnızca `--ros` bayrağıyla yüklenir; ROS2 yoksa ana program yine çalışır.

Yayınlananlar
  /surround_view/image_raw        sensor_msgs/Image   (bgr8, kuş bakışı, frame: base_link)
  /surround_view/pipeline_ms      std_msgs/Float32    (birleştirme süresi)
  /surround_view/overlap_error    std_msgs/Float32    (kameralar arası örtüşme hatası, 0-255)
  /cameras/<ad>/image_raw         sensor_msgs/Image   (--ros-cams ile)
  /cameras/<ad>/camera_info       sensor_msgs/CameraInfo (--ros-cams ile)
  /tf_static                      base_link -> cam_<ad>_optical (kamera extrinsics)

Koordinat dönüşümü: CARLA/Unreal sol-elli (x ileri, y sağ, z yukarı), ROS sağ-elli
(REP-103: x ileri, y sol, z yukarı). Bu yüzden y ve yaw/pitch işaretleri çevrilir.
Görüntü çerçevesi ROS "optical" kuralındadır (z ileri, x sağ, y aşağı).
"""
import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import StaticTransformBroadcaster


# ----------------------------- kuaterniyon yardımcıları ---------------------
def quat_from_rpy(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) -> (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    """Hamilton çarpımı, (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# ROS gövde çerçevesi (x ileri, y sol, z yukarı) -> optical (z ileri, x sağ, y aşağı)
Q_BODY_TO_OPTICAL = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)


def carla_cam_to_ros(tf):
    """carla.Transform (araca göreli) -> ((x, y, z), (qx, qy, qz, qw)) optical çerçeve, ROS."""
    loc, rot = tf.location, tf.rotation
    pos = (loc.x, -loc.y, loc.z)
    q_body = quat_from_rpy(math.radians(rot.roll),
                           -math.radians(rot.pitch),
                           -math.radians(rot.yaw))
    return pos, quat_mul(q_body, Q_BODY_TO_OPTICAL)


def cam_frame(name):
    return f"cam_{name}_optical"


# ----------------------------- köprü ----------------------------------------
class RosBridge:
    def __init__(self, cameras, cam_w, cam_h, fov_deg, px_per_m, range_m, publish_cams=False):
        rclpy.init()
        self.node = rclpy.create_node('surround_view')
        self.node.declare_parameter('px_per_m', float(px_per_m))
        self.node.declare_parameter('range_m', float(range_m))
        self.cam_w, self.cam_h = cam_w, cam_h
        self.publish_cams = publish_cams

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_bev = self.node.create_publisher(Image, '/surround_view/image_raw', qos)
        self.pub_pipe = self.node.create_publisher(Float32, '/surround_view/pipeline_ms', qos)
        self.pub_err = self.node.create_publisher(Float32, '/surround_view/overlap_error', qos)

        self.pub_cam, self.pub_info = {}, {}
        if publish_cams:
            for n in cameras:
                self.pub_cam[n] = self.node.create_publisher(Image, f'/cameras/{n}/image_raw', qos)
                self.pub_info[n] = self.node.create_publisher(CameraInfo, f'/cameras/{n}/camera_info', qos)

        f = cam_w / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
        cx, cy = cam_w / 2.0, cam_h / 2.0
        self.K = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        self.P = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # kamera extrinsics: bir kez yayınlanır (static TF)
        self.static_tf = StaticTransformBroadcaster(self.node)
        msgs = []
        for n, cfg in cameras.items():
            (x, y, z), (qx, qy, qz, qw) = carla_cam_to_ros(cfg['tf'])
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = cam_frame(n)
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            msgs.append(t)
        self.static_tf.sendTransform(msgs)

    # -- yardımcılar --
    @staticmethod
    def _stamp(t):
        s = int(t)
        return Time(sec=s, nanosec=int((t - s) * 1e9))

    def _image(self, bgr, stamp, frame_id):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, frame_id
        m.height, m.width = int(bgr.shape[0]), int(bgr.shape[1])
        m.encoding, m.is_bigendian = 'bgr8', 0
        m.step = m.width * 3
        m.data = bgr.tobytes()
        return m

    def _info(self, stamp, frame_id):
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, frame_id
        m.width, m.height = self.cam_w, self.cam_h
        m.distortion_model = 'plumb_bob'
        m.d = [0.0] * 5
        m.k = list(self.K)
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = list(self.P)
        return m

    # -- ana yayın --
    def publish(self, sim_time_s, bev, cams, pipeline_ms, overlap_err):
        stamp = self._stamp(sim_time_s)
        self.pub_bev.publish(self._image(bev, stamp, 'base_link'))
        self.pub_pipe.publish(Float32(data=float(pipeline_ms)))
        self.pub_err.publish(Float32(data=float(overlap_err)))
        if self.publish_cams and cams:
            for n, img in cams.items():
                self.pub_cam[n].publish(self._image(img, stamp, cam_frame(n)))
                self.pub_info[n].publish(self._info(stamp, cam_frame(n)))
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
