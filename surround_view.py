"""
CARLA 360° Surround View (kuş bakışı) - geometriden türetilmiş sürüm

Fikir: Elle seçilmiş piksel noktaları yok. Her kamera için
  - intrinsic matris  : FOV ve çözünürlükten
  - extrinsic         : araca monte edilen carla.Transform'dan
bulunur. Çıktı tuvalindeki her piksel = araç çevresindeki zemin düzleminde (z=0)
gerçek bir (X, Y) metre koordinatı. Bu nokta kameraya izdüşürülür ve
cv2.remap haritaları BİR KEZ hesaplanır. Kameralar arası geçiş açısal
ağırlıklarla yumuşatılır (alpha blending).

Araç koordinatı (CARLA / Unreal): X ileri, Y sağ, Z yukarı.
Tuvalde araç ileri = yukarı.

Tuşlar:  q = çıkış | r = 10 sn video kaydı | g = metrik ızgara aç/kapa
"""
import argparse
import queue
import time

import carla
import cv2
import numpy as np

# ----------------------------- AYARLAR --------------------------------------
CAM_W, CAM_H, FOV = 640, 480, 120.0   # 120° köşegen boşluğu kapatır, 640x480 keskinliği korur
HOLE_FILL = True                     # hiçbir kameranın görmediği pikselleri komşulardan doldur
RANGE_M = 10.0                       # araçtan her yöne kapsama (metre)
PX_PER_M = 40                        # çözünürlük: 1 metre = 40 piksel
CANVAS = int(2 * RANGE_M * PX_PER_M)  # 800 x 800
BLEND_DEG = 12.0                     # kameralar arası geçiş genişliği (derece)
FEATHER_PX = 25                      # görüntü kenarında yumuşatma (piksel)
DT = 0.05                            # senkron mod adımı (20 FPS simülasyon)

# Montaj noktaları (araç koordinatında). heading: kameranın baktığı yön (derece,
# 0 = ileri, +90 = sağ). Burayı değiştirirsen homografi kendiliğinden güncellenir.
CAMERAS = {
    'front': dict(tf=carla.Transform(carla.Location(x=2.45, z=0.7),
                                     carla.Rotation(pitch=-32, yaw=0)), heading=0.0),
    'right': dict(tf=carla.Transform(carla.Location(y=1.05, z=1.15),
                                     carla.Rotation(pitch=-33, yaw=90)), heading=90.0),
    'rear':  dict(tf=carla.Transform(carla.Location(x=-2.45, z=0.85),
                                     carla.Rotation(pitch=-32, yaw=180)), heading=180.0),
    'left':  dict(tf=carla.Transform(carla.Location(y=-1.05, z=1.15),
                                     carla.Rotation(pitch=-33, yaw=-90)), heading=-90.0),
}
NAMES = list(CAMERAS.keys())


# ----------------------------- GEOMETRİ -------------------------------------
def intrinsics():
    fx = CAM_W / (2.0 * np.tan(np.radians(FOV) / 2.0))
    return fx, fx, CAM_W / 2.0, CAM_H / 2.0          # fx, fy, cx, cy


def ground_grid():
    """Tuvaldeki her pikselin araç koordinatındaki zemin noktası (X ileri, Y sağ)."""
    cols, rows = np.meshgrid(np.arange(CANVAS, dtype=np.float32),
                             np.arange(CANVAS, dtype=np.float32))
    X = (CANVAS / 2.0 - rows) / PX_PER_M
    Y = (cols - CANVAS / 2.0) / PX_PER_M
    return X, Y


def build_camera_maps(tf, heading, X, Y):
    """Bir kamera için remap haritaları + ham ağırlık haritası üretir."""
    fx, fy, cx, cy = intrinsics()

    # araç çerçevesi -> kamera çerçevesi (Unreal: x ileri, y sağ, z yukarı)
    T_inv = np.array(tf.get_inverse_matrix(), dtype=np.float64)
    P = np.stack([X.ravel(), Y.ravel(), np.zeros(X.size), np.ones(X.size)])
    Pc = T_inv @ P
    # Unreal kamera -> OpenCV kamera (x sağ, y aşağı, z ileri)
    xc, yc, zc = Pc[1], -Pc[2], Pc[0]

    in_front = zc > 0.1
    zs = np.where(in_front, zc, 1.0)
    u = fx * xc / zs + cx
    v = fy * yc / zs + cy

    inside = in_front & (u >= 0) & (u <= CAM_W - 1) & (v >= 0) & (v <= CAM_H - 1)

    # görüntü kenarına yaklaştıkça ağırlığı azalt (dikiş yumuşatma)
    edge = np.minimum.reduce([u, CAM_W - 1 - u, v, CAM_H - 1 - v])
    feather = np.clip(edge / FEATHER_PX, 0.0, 1.0)

    # aracın merkezinden bakınca piksel hangi açıda? (0 = ileri, + = sağ)
    theta = np.degrees(np.arctan2(Y.ravel(), X.ravel()))
    diff = (theta - heading + 180.0) % 360.0 - 180.0
    ang_w = np.clip((45.0 + BLEND_DEG - np.abs(diff)) / (2.0 * BLEND_DEG), 0.0, 1.0)

    w = ((ang_w + 0.02) * feather * inside).astype(np.float32)
    map_x = np.where(inside, u, -1).astype(np.float32).reshape(CANVAS, CANVAS)
    map_y = np.where(inside, v, -1).astype(np.float32).reshape(CANVAS, CANVAS)
    return map_x, map_y, w.reshape(CANVAS, CANVAS)


def build_all():
    X, Y = ground_grid()
    maps, weights = {}, {}
    for n in NAMES:
        mx, my, w = build_camera_maps(CAMERAS[n]['tf'], CAMERAS[n]['heading'], X, Y)
        maps[n], weights[n] = (mx, my), w
    total = sum(weights.values())
    total[total < 1e-6] = 1.0                      # kimsenin görmediği yerler siyah kalır
    for n in NAMES:
        weights[n] = (weights[n] / total)[:, :, None].astype(np.float32)
    return maps, weights


def build_overlap_indices(weights):
    """Komşu kameraların ikisinin de güvenilir gördüğü pikseller (doğrulama metriği için)."""
    pairs = [('front', 'right'), ('right', 'rear'), ('rear', 'left'), ('left', 'front')]
    idx = {}
    for a, b in pairs:
        both = (weights[a][:, :, 0] > 0.25) & (weights[b][:, :, 0] > 0.25)
        idx[(a, b)] = np.nonzero(both)
    return idx


def overlap_error(warped, idx):
    """Komşu kameraların örtüştüğü bölgede ortalama mutlak renk farkı (0-255).
    Zemin geometrisi doğruysa düşük olur; zeminden yüksek nesnelerde (araç,
    kaldırım) doğal olarak artar."""
    errs = []
    for (a, b), (r, c) in idx.items():
        if r.size == 0:
            continue
        d = np.abs(warped[a][r, c].astype(np.int16) - warped[b][r, c].astype(np.int16))
        errs.append(d.mean())
    return float(np.mean(errs)) if errs else 0.0


def make_grid_overlay():
    ov = np.zeros((CANVAS, CANVAS, 3), np.uint8)
    for m in range(-int(RANGE_M), int(RANGE_M) + 1, 2):
        p = int(CANVAS / 2 + m * PX_PER_M)
        p = min(max(p, 0), CANVAS - 1)
        cv2.line(ov, (p, 0), (p, CANVAS - 1), (255, 255, 255), 1)
        cv2.line(ov, (0, p), (CANVAS - 1, p), (255, 255, 255), 1)
    for r in (5, 10):
        cv2.circle(ov, (CANVAS // 2, CANVAS // 2), int(r * PX_PER_M), (0, 200, 255), 1)
        cv2.putText(ov, f"{r} m", (CANVAS // 2 + 4, CANVAS // 2 - int(r * PX_PER_M) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
    return ov, ov.any(axis=2)


# ----------------------------- CARLA YARDIMCILARI ---------------------------
def to_bgr(img):
    a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
    return np.ascontiguousarray(a[:, :, :3])


def grab(q, frame_id, timeout=5.0):
    """Senkron modda tüm kameralardan AYNI tick'e ait kareyi al."""
    while True:
        img = q.get(timeout=timeout)
        if img.frame == frame_id:
            return img


# ----------------------------- ANA PROGRAM ----------------------------------
def main():
    ap = argparse.ArgumentParser(description="CARLA 360 surround view")
    ap.add_argument('--ros', action='store_true',
                    help="ROS2 topic'lerine yayinla (rclpy gerekir)")
    ap.add_argument('--ros-cams', action='store_true',
                    help="--ros ile birlikte ham kamera goruntu + camera_info da yayinla")
    args = ap.parse_args()
    bridge = None

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    orig_settings = world.get_settings()
    tm = client.get_trafficmanager()
    actor_list = []
    recording, video_writer, record_frames = False, None, 0
    max_record_frames = int(10.0 / DT)

    try:
        # Senkron mod: dört kamera aynı anda çekilir, hareket halinde kayma olmaz
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = DT
        world.apply_settings(s)
        tm.set_synchronous_mode(True)

        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        spawn_pt = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_pt)
        actor_list.append(vehicle)
        vehicle.set_autopilot(True, tm.get_port())
        ext = vehicle.bounding_box.extent

        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(CAM_W))
        cam_bp.set_attribute('image_size_y', str(CAM_H))
        cam_bp.set_attribute('fov', str(FOV))

        queues = {n: queue.Queue() for n in NAMES}
        for n in NAMES:
            cam = world.spawn_actor(cam_bp, CAMERAS[n]['tf'], attach_to=vehicle)
            actor_list.append(cam)
            cam.listen(queues[n].put)

        print("Remap haritalari hesaplaniyor (tek seferlik)...")
        maps, weights = build_all()
        overlap_idx = build_overlap_indices(weights)

        # Kör bölge (hiçbir kameranın görmediği pikseller) ve dolgu hazırlığı
        hole = sum(weights[n][:, :, 0] for n in NAMES) < 1e-6
        SM, SIG = CANVAS // 4, 10
        valid_s = cv2.resize((~hole).astype(np.float32), (SM, SM), interpolation=cv2.INTER_AREA)
        den = np.maximum(cv2.GaussianBlur(valid_s, (0, 0), SIG), 1e-3)[:, :, None]
        grid_ov, grid_mask = make_grid_overlay()
        show_grid = False
        if args.ros:
            from ros_bridge import RosBridge      # sadece --ros ile yuklenir
            bridge = RosBridge(CAMERAS, CAM_W, CAM_H, FOV, PX_PER_M, RANGE_M, args.ros_cams)
            print("ROS2 koprusu aktif -> /surround_view/image_raw")
        print("Hazir. q: cikis | r: 10 sn video | g: izgara\n")

        half_l, half_w = int(ext.x * PX_PER_M), int(ext.y * PX_PER_M)
        c = CANVAS // 2
        car_tl, car_br = (c - half_w, c - half_l), (c + half_w, c + half_l)

        fps_hist, stitch_hist, prev = [], [], time.perf_counter()
        err_val, frame_i = 0.0, 0

        while True:
            t_tick = time.perf_counter()
            frame_id = world.tick()
            imgs = {n: to_bgr(grab(queues[n], frame_id)) for n in NAMES}
            tick_ms = (time.perf_counter() - t_tick) * 1000.0

            t0 = time.perf_counter()
            warped = {n: cv2.remap(imgs[n], maps[n][0], maps[n][1], cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT) for n in NAMES}
            acc = np.zeros((CANVAS, CANVAS, 3), np.float32)
            for n in NAMES:
                acc += warped[n] * weights[n]
            canvas = acc.astype(np.uint8)
            if HOLE_FILL and hole.any():
                small = cv2.resize(canvas, (SM, SM), interpolation=cv2.INTER_AREA).astype(np.float32)
                fill = cv2.resize(cv2.GaussianBlur(small, (0, 0), SIG) / den, (CANVAS, CANVAS),
                                  interpolation=cv2.INTER_LINEAR)
                canvas[hole] = np.clip(fill[hole], 0, 255).astype(np.uint8)
            stitch_ms = (time.perf_counter() - t0) * 1000.0

            # doğrulama metriği (her 10 karede bir)
            if frame_i % 10 == 0:
                err_val = overlap_error(warped, overlap_idx)
            frame_i += 1

            if show_grid:
                canvas[grid_mask] = (0.65 * canvas[grid_mask] + 0.35 * grid_ov[grid_mask]).astype(np.uint8)

            # araç gövdesi (gerçek bounding box boyutunda)
            cv2.rectangle(canvas, car_tl, car_br, (45, 45, 45), -1)
            cv2.rectangle(canvas, car_tl, car_br, (0, 255, 0), 2)
            cv2.arrowedLine(canvas, (c, c + 15), (c, c - 25), (200, 200, 200), 2, tipLength=0.35)

            # ROS2 (HUD'suz temiz görüntü yayınlanır)
            if bridge:
                bridge.publish(world.get_snapshot().timestamp.elapsed_seconds, canvas,
                               imgs if args.ros_cams else None, stitch_ms, err_val)

            # HUD
            now = time.perf_counter()
            fps_hist.append(1.0 / max(now - prev, 1e-6))
            prev = now
            fps_hist = fps_hist[-30:]
            stitch_hist.append(stitch_ms)
            stitch_hist = stitch_hist[-30:]
            avg_stitch = float(np.mean(stitch_hist))
            hud = [f"Pipeline: {avg_stitch:.1f} ms (~{1000.0 / max(avg_stitch, 1e-3):.0f} FPS)",
                   f"Sim loop: {np.mean(fps_hist):.1f} FPS (CARLA tick+render: {tick_ms:.0f} ms)",
                   f"Overlap err: {err_val:.1f}/255",
                   f"Scale: {PX_PER_M} px/m  Range: {RANGE_M:.0f} m"]
            y1, x1 = 12 + 20 * len(hud), 440
            canvas[4:y1, 4:x1] = (canvas[4:y1, 4:x1] * 0.35).astype(np.uint8)
            for i, t in enumerate(hud):
                cv2.putText(canvas, t, (12, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 1, cv2.LINE_AA)

            if recording:
                cv2.circle(canvas, (CANVAS - 25, 25), 8, (0, 0, 255), -1)
                cv2.putText(canvas, "REC", (CANVAS - 75, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 2)
                video_writer.write(canvas)
                record_frames += 1
                if record_frames >= max_record_frames:
                    recording = False
                    video_writer.release()
                    print("[OK] Video 'surround_demo.mp4' olarak kaydedildi.")

            cv2.imshow("360 Surround View", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                show_grid = not show_grid
            elif key == ord('r') and not recording:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter('surround_demo.mp4', fourcc, 1.0 / DT,
                                               (CANVAS, CANVAS))
                recording, record_frames = True, 0
                print("[BILGI] Kayit basladi (10 sn)...")

    finally:
        if recording and video_writer:
            video_writer.release()
        print("\nTemizlik yapiliyor...")
        try:
            for a in actor_list:
                if a.type_id.startswith("sensor"):
                    a.stop()
            client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        finally:
            world.apply_settings(orig_settings)   # senkron modu kapat
            tm.set_synchronous_mode(False)
            if bridge:
                bridge.shutdown()
            cv2.destroyAllWindows()
        print("Sistem kapatildi.")


if __name__ == '__main__':
    main()
