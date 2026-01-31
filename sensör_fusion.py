#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import cv2
import os
import pandas as pd
import threading
import serial
from sensor_msgs.msg import LaserScan, Image
from laser_geometry import LaserProjection
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
from ultralytics import YOLO
from pyntcloud import PyntCloud
import random
from collections import defaultdict
import matplotlib.pyplot as plt
from pymavlink import mavutil
import re
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


"""
# Start a connection listening on a UDP port
vehicle= mavutil.mavlink_connection('com11',baudrate=115200)

vehicle.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))
"""

class ScanTo3DPCD(Node):
    def __init__(self):
        super().__init__('scan_to_3d_pcd')
        self.laser_projector = LaserProjection()
        self.bridge = CvBridge()
        self.lidar_points = []
        self.all_points = []
        self.scan_count = 0
        self.z_center = 1.0
        self.z_amplitude = 2 #1.285
        self.z_step = 0.3  # 0.1
        self.image = None

        self.class_colors = {}
        self.detected_objects_info = {}

        self.motion_detected = False

        # Kamera kalibrasyon matrisi
        self.camera_matrix = np.array([
            [554.39280307, 0, 290.42810266],
            [0, 559.30289238, 202.21305254],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1))

        # Dönüşüm matrisleri
        self.R = np.array([
            [0.95708973, -0.08376518, 0.2774214],
            [0.03008838, 0.98086298, 0.19236037],
            [-0.28822548, -0.17575897, 0.94129424]
        ])
        self.T = np.array([[0.54417241], [2.74380996], [21.64559949]])

        # YOLO modeli
        self.model = YOLO("/home/barlas/Desktop/ros2/yolo11n.pt")

        # ROS abonelikleri
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 1000)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Arduino seri bağlantı
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("🔌 Arduino bağlantısı başarılı.")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Arduino bağlantısı başarısız: {e}")
            self.arduino = None

        # Radar dinleyici thread
        self.radar_thread = threading.Thread(target=self.listen_radar_serial, daemon=True)
        self.radar_thread.start()
        #self.ply_to_png_thread= threading.Thread(target=self.save_ply_to_png,daemon=True).start()

    def listen_radar_serial(self):
        print(f"[RADAR thread started]")
        if self.arduino is None:
            print(f"[RADAR] Arduino yok, thread kapandı.")
            return
        try:
            while True:
                #print(f"[RADAR] Bytes waiting: {self.arduino.in_waiting}")
                if self.arduino.in_waiting > 0:
                    
                    line = self.arduino.readline().decode('utf-8').strip()
                    print(f"[RADAR]: {line}")
                    if "Hareket algılandı" in line:
                        self.motion_detected = True
        except Exception as e:
            print(f"Radar dinleyici hatası: {e}")

    
    def image_callback(self, msg):
        try:

            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Görüntü dönüşüm hatası: {e}")
    
    
    def scan_callback(self, scan_msg):
        print("a")
        if self.image is None:
            return
        print("bb")
        try:
            cloud = self.laser_projector.projectLaser(scan_msg)
            points = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
            z_offset = self.z_center + math.sin(self.scan_count * self.z_step) * self.z_amplitude
            self.scan_count += 1
            print("ccc")
            lidar_points = [(x, y, z_offset) for x, y, _ in points]
            self.all_points.extend(lidar_points)
            
            #Radar hareket algıladıysa haritaya siyah nokta ekle
            if self.motion_detected:
                for i in range(20):
                    x = random.uniform(-1.0, 1.0)
                    y = random.uniform(-1.0, 1.0)
                    z = z_offset
                    motion_pt = (x, y, z)
                    self.all_points.append(motion_pt)
                    self.lidar_points.append((motion_pt, "motion"))
                self.class_colors["motion"] = [1.0, 0.0, 0.0]  # mavi
                self.detected_objects_info["motion"] = (1.0, (255, 0, 0))
                self.motion_detected = False
                


            results = self.model(self.image, verbose=False)[0]
            matched_points = []

            for pt in lidar_points:
                pt_vec = np.array(pt).reshape(3, 1)
                pt_cam = self.R @ pt_vec + self.T
                img_pts, _ = cv2.projectPoints(pt_cam.reshape(1, 3), np.zeros((3, 1)), np.zeros((3, 1)),
                                               self.camera_matrix, self.dist_coeffs)
                u, v = map(int, img_pts[0][0])

                for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
                    x1, y1, x2, y2 = map(int, box)
                    if conf > 0.4 and x1 <= u <= x2 and y1 <= v <= y2:
                        label = results.names[int(cls)]
                        #print(f"Matched points: {pt}")
                        matched_points.append((pt, label))
                        ####### --------------------------------------------- ##########
                        # , make split according to ' , ' and put it to list
                        #pattern = f"{x1},{x2},{y1},{y2},{label}\n"
                        pattern = f"{u},{v},{label}\n"
                        with open("/home/barlas/output/coordinat_detected_object.txt", "w") as f:
                            f.write(pattern)
                        ####### --------------------------------------------- ##########
                        if label not in self.class_colors:
                            self.class_colors[label] = [random.random(), random.random(), random.random()]
                        if label not in self.detected_objects_info or conf > self.detected_objects_info[label][0]:
                            dominant_color = self.get_dominant_color(self.image[y1:y2, x1:x2])
                            self.detected_objects_info[label] = (conf.item(), dominant_color)
                        break

            self.lidar_points.extend(matched_points)
            """
            if os.environ.get("DISPLAY", ""):
                cv2.imshow("YOLO + LIDAR", self.image)
                cv2.waitKey(1)
            """
            if self.scan_count >= 5:
                self.save_point_cloud()
                self.scan_count = 0
                self.all_points.clear()
                self.lidar_points.clear()
                self.detected_objects_info.clear()

        except Exception as e:
            #self.get_logger().warn(f"Taramayı işlerken hata: {e}")
            pass

    def get_dominant_color(self, roi):
        if roi.size == 0:
            return (0, 0, 0)
        roi = cv2.resize(roi, (1, 1), interpolation=cv2.INTER_AREA)
        color = roi[0, 0]
        return tuple(int(c) for c in color)

    def save_point_cloud(self):
        if not self.all_points:
            self.get_logger().info("Kaydedilecek nokta yok.")
            return

        matched_keys = {tuple(np.round(p[0], 3)) for p in self.lidar_points}
        points = []
        colors = []

        for pt in self.all_points:
            pt_key = tuple(np.round(pt, 3))
            points.append(pt)
            if pt_key in matched_keys:
                for matched_pt, label in self.lidar_points:
                    if np.allclose(pt, matched_pt, atol=1e-3):
                        color = self.class_colors[label]
                        break
                else:
                    color = [0.5, 0.5, 0.5]
            else:
                color = [0.5, 0.5, 0.5]
            colors.append(color)

        df = pd.DataFrame(points, columns=['x', 'y', 'z'])
        color_array = (np.array(colors) * 255).astype(int)
        df.loc[:, ['red', 'green', 'blue']] = color_array
        cloud = PyntCloud(df)

        os.makedirs("/home/barlas/output", exist_ok=True)
        cloud.to_file("/home/barlas/output/output.ply")

        with open("/home/barlas/output/detected_objects.txt", "w") as f:
            for label, (conf, color) in self.detected_objects_info.items():
                f.write(f"{label} - {conf:.2f} - RGB{color}\n")

        self.get_logger().info("✅ Nokta bulutu ve sınıf verisi kaydedildi.")
        self.save_ply_to_png()

    def save_ply_to_png(self):
        """   
        while True:
            msg=vehicle.recv_match(
                type="HEARTBEAT", blocking=True
            )
            if msg.custom_mode == 9:
                print('Mode: %s' % msg.custom_mode)
        """
        # .ply dosyasını yükle
        cloud = PyntCloud.from_file("/home/barlas/output/output.ply")

        # Nokta verilerini DataFrame olarak al
        points = cloud.points

        # 3D noktaları 2D projeksiyonla çiz (örneğin X-Y düzleminde)
        fig = plt.figure(figsize=(10, 10))
        plt.scatter(points["x"], points["y"], c=points.get("z", 0), cmap="viridis", s=0.5)
        plt.axis("equal")
        plt.axis("off")

        # Görseli kaydet
        plt.savefig("/home/barlas/Desktop/ros2/images/output_orginal.png", bbox_inches='tight', pad_inches=0, dpi=300)

        with open("/home/barlas/output/coordinat_detected_object.txt") as f:
            raw = f.read().replace('\n', ',').strip()
            items = raw.split(',')
            print(f"Items: {items}")

        img = cv2.imread("/home/barlas/Desktop/ros2/images/output_orginal.png")

        for i in range(0, len(items) - 2, 3):
            try:
                u = int(items[i].strip())
                v = int(items[i + 1].strip())
                #y1 = int(items[i + 2].strip())
                #y2 = int(items[i + 3].strip())

                # Sayı olmayan karakterleri label olarak al (örnek: motion160 → motion)
                raw_label = items[i + 2].strip()
                label_match = re.match(r"[A-Za-z]+", raw_label)
                label = label_match.group(0).lower() if label_match else "unknown"
                if label != "motion":
                    #cx = int((x1 + x2) / 2)
                    #cy = int((y1 + y2) / 2)
                    img_h, img_w = img.shape[:2]
                    u_scaled = int(u * img_w / 640)
                    v_scaled = int(v * img_h / 480)
                    print(f"u:{u} aand v {v}")
                    print(f"u_scaled:{u_scaled} aand v_scaled {v_scaled}")
                    print("PNG size:", img.shape)  # (h, w)
                    cv2.putText(img, label, (u_scaled, v_scaled), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)

            except Exception as e:
                print(f"Hata: {e}")
                continue
            
        cv2.imwrite("/home/barlas/Desktop/ros2/images/output_processed.png",img)
        print("PNG olarak kaydedildi: output_processed.png")


def main(args=None):
    rclpy.init(args=args)
    node = ScanTo3DPCD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
