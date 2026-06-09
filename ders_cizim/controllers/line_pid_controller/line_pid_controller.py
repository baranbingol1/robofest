from controller import Robot
from PID import PID


# ============================================================
# Webots setup
# ============================================================

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
motor_rl = robot.getDevice("motor_rl")
motor_fl = robot.getDevice("motor_fl")
motor_rr = robot.getDevice("motor_rr")
motor_fr = robot.getDevice("motor_fr")

motors = [motor_rl, motor_fl, motor_rr, motor_fr]

for motor in motors:
    motor.setPosition(float("inf"))  # velocity control mode
    motor.setVelocity(0.0)

# Senin world yapına göre:
# y negatif taraf: motor_rl + motor_fl
# y pozitif taraf: motor_rr + motor_fr
left_motors = [motor_rl, motor_fl]
right_motors = [motor_rr, motor_fr]

# Camera
camera = robot.getDevice("camera")
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()

CAMERA_CENTER_X = (width - 1) / 2


# ============================================================
# PID setup
# ============================================================

# PID.py içinde round(feedback_value) kullanıldığı için
# küçük kamera farkları kaybolmasın diye ölçümü büyütüyoruz.
PID_SCALE = 10.0

# Eski P=0.03 çok agresifti.
# Bu değerle robot pivot atmadan çizgiyi aramalı.
pid = PID(P=0.006, I=0.000, D=0.000)
pid.SetPoint = CAMERA_CENTER_X * PID_SCALE
pid.setSampleTime(0.01)
pid.setWindup(50)

pid.m1 = 1
pid.m2 = 1


# ============================================================
# Driving constants
# ============================================================

BASE_SPEED = 2.0
MAX_SPEED = 5.0

# Çizgi izlerken uygulanabilecek maksimum sağ-sol farkı.
# Büyük olursa robot kendi etrafında döner.
MAX_CORRECTION = 1.55

# Siyah algılama eşiği.
# 80 fazla genişti; gölgeyi de siyah sayabilir.
BLACK_THRESHOLD = 50

# Eğer robot çizgiye yaklaşmak yerine çizgiden kaçarsa bunu -1 yap.
STEERING_SIGN = 1.0

last_line_x = CAMERA_CENTER_X
last_correction = 0.0


# ============================================================
# Helper functions
# ============================================================

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def set_left_right_speed(left_speed, right_speed):
    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    for motor in left_motors:
        motor.setVelocity(left_speed)

    for motor in right_motors:
        motor.setVelocity(right_speed)


def get_black_segments(black_pixels):
    """
    Ardışık siyah piksel gruplarını bulur.
    Örnek: [5,6,7,20,21] -> [(5,7), (20,21)]
    """
    if not black_pixels:
        return []

    segments = []
    start = black_pixels[0]
    prev = black_pixels[0]

    for x in black_pixels[1:]:
        if x == prev + 1:
            prev = x
        else:
            segments.append((start, prev))
            start = x
            prev = x

    segments.append((start, prev))
    return segments


def find_line_center_x():
    """
    Kamera görüntüsünde siyah çizginin merkezini bulur.

    Kritik nokta:
    - Tamamen siyah olan satırları kullanmıyoruz.
    - Çok az siyah olan satırları da kullanmıyoruz.
    - En düzgün siyah segmenti veren satırdan merkez hesaplıyoruz.
    """
    image = camera.getImage()

    # Alttan yukarı doğru bakıyoruz.
    # Alt kısım robota çok yakın olduğu için bazen komple siyah çıkar.
    start_y = int(height * 0.90)
    end_y = int(height * 0.20)

    best_center = None
    best_segment_width = 0
    best_black_count = 0
    best_scan_y = None

    for scan_y in range(start_y, end_y, -2):
        black_pixels = []

        for x in range(width):
            r = camera.imageGetRed(image, width, x, scan_y)
            g = camera.imageGetGreen(image, width, x, scan_y)
            b = camera.imageGetBlue(image, width, x, scan_y)

            gray = (r + g + b) / 3

            if gray < BLACK_THRESHOLD:
                black_pixels.append(x)

        black_count = len(black_pixels)

        # Siyah çok azsa çizgi yok say.
        if black_count < 3:
            continue

        # Satırın çoğu siyahsa merkez bilgisi güvenilir değildir.
        # 64 px kamera için 48+ siyah piksel kötü veridir.
        if black_count > width * 0.75:
            continue

        segments = get_black_segments(black_pixels)

        if not segments:
            continue

        largest_segment = max(segments, key=lambda seg: seg[1] - seg[0])
        segment_width = largest_segment[1] - largest_segment[0] + 1
        center = (largest_segment[0] + largest_segment[1]) / 2

        # En geniş ama komple ekranı kaplamayan segmenti seç.
        if segment_width > best_segment_width:
            best_segment_width = segment_width
            best_center = center
            best_black_count = black_count
            best_scan_y = scan_y

    if best_center is None:
        return None, 0, None

    return best_center, best_black_count, best_scan_y


def follow_line(line_x):
    """
    PID ile sağ-sol motor hızlarını üretir.
    Çizgi izlerken hiçbir motorun geri gitmesine izin vermiyoruz.
    """
    global last_correction

    pid.update(line_x * PID_SCALE)

    correction = pid.filteredoutput * STEERING_SIGN

    # En kritik düzeltme burada.
    # PID çok büyürse robot kendi etrafında döner.
    correction = clamp(correction, -MAX_CORRECTION, MAX_CORRECTION)

    last_correction = correction

    left_speed = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction

    # Çizgi takip modunda geri vites yok.
    # Geri vites robotu çizgi izleyiciden pivot atan tanka çeviriyor.
    left_speed = clamp(left_speed, 0.0, MAX_SPEED)
    right_speed = clamp(right_speed, 0.0, MAX_SPEED)

    set_left_right_speed(left_speed, right_speed)

    return correction, left_speed, right_speed


def search_line():
    """
    Çizgi kaybolursa sert pivot atmak yerine yavaşça son görülen tarafa döner.
    """
    if last_line_x > CAMERA_CENTER_X:
        # Çizgi son olarak sağdaydı; sağa yumuşak ara.
        left_speed = 1.5
        right_speed = 0.4
    else:
        # Çizgi son olarak soldaydı; sola yumuşak ara.
        left_speed = 0.4
        right_speed = 1.5

    set_left_right_speed(left_speed, right_speed)

    return left_speed, right_speed


# ============================================================
# Main loop
# ============================================================

while robot.step(timestep) != -1:
    line_x, black_count, scan_y = find_line_center_x()

    if line_x is not None:
        last_line_x = line_x

        correction, left_speed, right_speed = follow_line(line_x)

        error = pid.SetPoint - round(line_x * PID_SCALE)

        print(
            "line_x:",
            round(line_x, 2),
            "target:",
            round(CAMERA_CENTER_X, 2),
            "black:",
            black_count,
            "scan_y:",
            scan_y,
            "error:",
            round(error, 2),
            "pid:",
            round(correction, 3),
            "left:",
            round(left_speed, 3),
            "right:",
            round(right_speed, 3)
        )

    else:
        left_speed, right_speed = search_line()

        print(
            "Line lost. Searching...",
            "last_line_x:",
            round(last_line_x, 2),
            "left:",
            round(left_speed, 3),
            "right:",
            round(right_speed, 3)
        )