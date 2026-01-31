import cv2
import numpy as np

# -------------------------------
# AYARLAR – Burayı kendine göre düzenle
# -------------------------------

image_path = "/home/emre/Desktop/ROS/camera_calibration/image_0.png"  # Görüntü dosyası yolu
square_size = 0.15  # Her karenin boyutu (metre cinsinden)
#3385 px for py 3385 
camera_matrix = np.array([
    [2000, 0, 1280],
    [0, 2000, 1280],
    [0,   0,   1]
], dtype=np.float64)

dist_coeffs = np.zeros((5, 1))  # Gerekirse kendi distortion vektörünü koy

# -------------------------------
# 1. Nokta Seçme Arayüzü
# -------------------------------

clicked_points = []


def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))
        print(f"Point {len(clicked_points)}: ({x}, {y})")

        # Görüntü üzerinde tıklanan noktaya daire çiz
        cv2.circle(clone, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Select corners in order (e.g. TL → TR → BR → BL for each square)", clone)

img = cv2.imread(image_path)
clone = img.copy()
cv2.imshow("Select corners in order (e.g. TL → TR → BR → BL for each square)", clone)
cv2.setMouseCallback("Select corners in order (e.g. TL → TR → BR → BL for each square)", click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()

# -------------------------------
# 2. 2D ve 3D Nokta Eşleşmesi
# -------------------------------

# Manuel girilmiş noktalar (img üzerindeki 2D)
image_points = np.array(clicked_points, dtype=np.float32)

# Gerçek dünyadaki karşılık gelen noktalar (3D)
# 4 kare, her biri 4 köşe → 16 nokta
object_points = []
for j in range(2):  # Satır
    for i in range(2):  # Sütun
        x = i * square_size
        y = j * square_size
        object_points.extend([
            [x, y, 0],
            [x + square_size, y, 0],
            [x + square_size, y + square_size, 0],
            [x, y + square_size, 0],
        ])
object_points = np.array(object_points, dtype=np.float32)

# -------------------------------
# 3. solvePnP ile Kalibrasyon
# -------------------------------

retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

R, _ = cv2.Rodrigues(rvec)
extrinsic = np.hstack((R, tvec))
print("Rotation matrix:\n", R)
print("Translation vector:\n", tvec)
print("Extrinsic matrix [R|T]:\n", extrinsic)

def format_matrix(mat):
    return "\n".join(" ".join(f"{num:.2f}" for num in row) for row in mat)

with open("/home/emre/Desktop/ROS/camera_calibration/info.txt", "a") as f:
    f.write("Rotation Matrix:\n")
    f.write(format_matrix(R) + "\n\n")
    f.write("Translation Vector:\n")
    f.write(format_matrix(tvec) + "\n\n")
    f.write("Extrinsic Matrix:\n")
    f.write(format_matrix(extrinsic) + "\n\n")

# -------------------------------
# 4. Doğrulama için projeksiyon
# -------------------------------

projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)

for p in projected_points:
    x, y = p.ravel().astype(int)
    cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

cv2.imshow("Projection Verification", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

