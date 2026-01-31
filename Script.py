#!/usr/bin/env python3

import subprocess
import time
import os
import signal
"""
def launch_process(command_list):
    return subprocess.Popen(command_list, preexec_fn=os.setsid)

try:
    print("RPLIDAR başlatılıyor...")
    rplidar_proc = launch_process(["ros2", "launch", "rplidar_ros", "rplidar_a1_launch.py"])
    time.sleep(3)

    print("SLAM Toolbox başlatılıyor...")
    slam_proc = launch_process(["ros2", "launch", "slam_toolbox", "online_async_launch.py"])
    time.sleep(3)

    print("Kamera ve LIDAR kaydı başlatılıyor...")
    take_video_proc = launch_process(["ros2", "run", "scan_pcd_pkg", "take_video"])
    time.sleep(3)

    print("camera_and_lidar_ros2_jazzy.py başlatılıyor...")
    camera_and_lidar_proc = launch_process(["python3", "/home/barlas/ros2_ws/src/scan_pcd_pkg/scan_pcd_pkg/camera_and_lidar_ros2_jazzy.py"])

    print("\nTüm işlemler başlatıldı. Ctrl+C ile çıkabilirsiniz.")

    # Ana döngü: işlemler bitene kadar bekle
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nCtrl+C alındı, işlemler sonlandırılıyor...")

    for proc in [rplidar_proc, slam_proc, take_video_proc, camera_and_lidar_proc]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception as e:
            print(f"Bir işlem sonlandırılamadı: {e}")

    print("Tüm işlemler durduruldu.")
"""
#!/usr/bin/env python3

import subprocess
import time
import os
import signal

def launch_process(command_list):
    return subprocess.Popen(command_list, preexec_fn=os.setsid)

try:
    # --- USB port izinlerini ayarla ---
    print("USB port izinleri ayarlanıyor (/dev/ttyUSB0)...")
    subprocess.run(["sudo", "chmod", "666", "/dev/ttyUSB0"])
    time.sleep(1)

    print("RPLIDAR başlatılıyor...")
    rplidar_proc = launch_process(["ros2", "launch", "rplidar_ros", "rplidar_a1_launch.py"])
    time.sleep(3)

    print("SLAM Toolbox başlatılıyor...")
    slam_proc = launch_process(["ros2", "launch", "slam_toolbox", "online_async_launch.py"])
    time.sleep(3)

    print("Kamera ve LIDAR kaydı başlatılıyor...")
    take_video_proc = launch_process(["ros2", "run", "scan_pcd_pkg", "take_video"])
    time.sleep(3)

    print("camera_and_lidar_ros2_jazzy2.py başlatılıyor...")
    camera_and_lidar_proc = launch_process(["python3", "/home/barlas/ros2_ws/src/scan_pcd_pkg/scan_pcd_pkg/camera_and_lidar_ros2_jazzy.py"])

    print("\nTüm işlemler başlatıldı. Ctrl+C ile çıkabilirsiniz.")

    # Ana döngü: işlemler bitene kadar bekle
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nCtrl+C alındı, işlemler sonlandırılıyor...")

    for proc in [rplidar_proc, slam_proc, take_video_proc, camera_and_lidar_proc]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception as e:
            print(f"Bir işlem sonlandırılamadı: {e}")

    print("Tüm işlemler durduruldu.")
