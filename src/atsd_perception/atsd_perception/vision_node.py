#!/usr/bin/env python3
import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from atsd_perception.vision_logic import (CameraGeom, Confirm, LightHsv, SpotCfg, classify_light,
                                         distance_from_height, find_red_spot)

try:
    import cv2
    CV_OK = True
except Exception:
    CV_OK = False


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')
        d = self.declare_parameter
        d('device', '/dev/camera')
        d('width', 640)
        d('height', 480)
        d('fps', 30)
        d('model_path', '/opt/atsd/models/signs_ncnn_model')
        d('backend', 'auto')
        d('threads', 3)
        d('imgsz', 640)
        d('conf', 0.5)
        d('infer_hz', 8.0)
        d('focal_px', 460.0)
        d('camera_to_front_m', 0.07)
        d('sign_height_m', 0.30)
        d('light_height_m', 0.30)
        d('class_stop', ['stop'])
        d('class_cross', ['crosswalk'])
        d('class_bump', ['bump_sign', 'bump_warn'])
        d('class_light', ['traffic_light'])
        d('sign_confirm', 2)
        d('light_confirm', 3)
        d('max_sign_distance_m', 6.0)
        d('light_lost_s', 2.0)
        d('debug_image_hz', 2.0)
        d('hsv_s_min', 90)
        d('hsv_v_min', 140)
        d('spot_enabled', True)
        d('camera_x', 0.28)
        d('camera_y', 0.0)
        d('camera_z', 0.385)
        d('camera_pitch_deg', 10.0)
        d('spot_s_min', 110)
        d('spot_v_min', 60)
        d('spot_min_area_m2', 0.02)
        d('spot_max_area_m2', 2.5)
        d('spot_max_range_m', 3.5)

        p = lambda n: self.get_parameter(n).value
        self.P = {n: p(n) for n in ('device', 'width', 'height', 'fps', 'model_path', 'backend', 'threads',
                                    'imgsz', 'conf',
                                    'focal_px', 'camera_to_front_m', 'sign_height_m', 'light_height_m',
                                    'max_sign_distance_m', 'light_lost_s')}
        self.classes = {}
        for kind, key in (('STOP', 'class_stop'), ('CROSS', 'class_cross'),
                          ('BUMP', 'class_bump'), ('LIGHT', 'class_light')):
            for name in p(key):
                self.classes[str(name)] = kind
        self.hsv = LightHsv(int(p('hsv_s_min')), int(p('hsv_v_min')))
        self.sign_conf = {k: Confirm(int(p('sign_confirm')), 3) for k in ('STOP', 'CROSS', 'BUMP')}
        self.light_conf = Confirm(int(p('light_confirm')), int(p('light_confirm')) + 1)
        self.last_light_seen = 0.0
        self.last_n_sent = 0.0
        self.debug_period = 1.0 / max(float(p('debug_image_hz')), 0.1)
        self.last_debug = 0.0

        self.pub_sign = self.create_publisher(String, 'perception/sign', 10)
        self.pub_light = self.create_publisher(String, 'perception/light', 10)
        self.pub_det = self.create_publisher(String, 'perception/detections', 10)
        self.pub_spot = self.create_publisher(String, 'perception/spot', 10)
        self.spot_enabled = bool(p('spot_enabled'))
        self.cam = CameraGeom(focal_px=float(p('focal_px')), cx=int(p('width')) / 2.0, cy=int(p('height')) / 2.0,
                              x=float(p('camera_x')), y=float(p('camera_y')), z=float(p('camera_z')),
                              pitch_rad=float(p('camera_pitch_deg')) * 3.141592653589793 / 180.0)
        self.spot_cfg = SpotCfg(s_min=int(p('spot_s_min')), v_min=int(p('spot_v_min')),
                                min_area_m2=float(p('spot_min_area_m2')),
                                max_area_m2=float(p('spot_max_area_m2')),
                                max_range_m=float(p('spot_max_range_m')))
        self.last_spot = None
        self.pub_dbg = self.create_publisher(CompressedImage, 'perception/debug/compressed', 2)

        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.model = None
        self.backend = None

        if not CV_OK:
            self.get_logger().error('Нет OpenCV: sudo apt install python3-opencv')
            return
        self.load_model()
        threading.Thread(target=self.capture_loop, daemon=True).start()
        threading.Thread(target=self.infer_loop, args=(float(p('infer_hz')),), daemon=True).start()

    def load_model(self):
        backend = self.P['backend']
        errors = []
        if backend in ('auto', 'ncnn'):
            try:
                from atsd_perception.ncnn_yolo import NcnnYolo
                self.model = NcnnYolo(self.P['model_path'], threads=int(self.P['threads']))
                self.backend = 'ncnn'
                names = self.model.names
            except Exception as e:
                self.model = None
                errors.append(f'ncnn: {e}')
        if self.model is None and backend in ('auto', 'ultralytics'):
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.P['model_path'], task='detect')
                self.backend = 'ultralytics'
                names = getattr(self.model, 'names', {}) or {}
            except Exception as e:
                self.model = None
                errors.append(f'ultralytics: {e}')
        if self.model is None:
            self.get_logger().error('Модель не загружена, знаки и светофор не распознаются. ' + '; '.join(errors))
            return
        self.get_logger().info(f'Модель {self.P["model_path"]} через {self.backend}, классы: {names}')
        unknown = [n for n in (names.values() if isinstance(names, dict) else names) if n not in self.classes]
        if unknown:
            self.get_logger().warn(f'Классы без действия: {unknown}')

    def capture_loop(self):
        cap = None
        while self.running and rclpy.ok():
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(self.P['device'], cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.P['width']))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.P['height']))
                cap.set(cv2.CAP_PROP_FPS, int(self.P['fps']))
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    self.get_logger().error(f'Камера {self.P["device"]} не открылась',
                                            throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
                self.get_logger().info(f'Камера {self.P["device"]} открыта')
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warn('Кадр не прочитан, переоткрываю камеру', throttle_duration_sec=5.0)
                cap.release()
                cap = None
                time.sleep(0.5)
                continue
            with self.frame_lock:
                self.frame = frame
        if cap is not None:
            cap.release()

    def infer_loop(self, hz):
        period = 1.0 / hz
        while self.running and rclpy.ok():
            t0 = time.monotonic()
            with self.frame_lock:
                frame = None if self.frame is None else self.frame.copy()
            if frame is not None:
                try:
                    self.process(frame)
                except Exception as e:
                    self.get_logger().error(f'Ошибка обработки кадра: {e}', throttle_duration_sec=5.0)
            now = time.monotonic()
            if now - self.last_light_seen > self.P['light_lost_s'] and now - self.last_n_sent > 1.0:
                self.last_n_sent = now
                self.pub_light.publish(String(data='N'))
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def process(self, frame):
        if self.spot_enabled:
            spot = find_red_spot(frame, self.cam, self.spot_cfg)
            self.last_spot = spot
            self.pub_spot.publish(String(
                data=f'{spot["x"]:.3f} {spot["y"]:.3f} {spot["area"]:.3f}' if spot else 'none'))
        dets = []
        if self.model is not None and self.backend == 'ncnn':
            for det in self.model.predict(frame, conf=float(self.P['conf']), iou=0.45):
                det['kind'] = self.classes.get(det['cls'])
                dets.append(det)
        elif self.model is not None:
            res = self.model.predict(frame, imgsz=int(self.P['imgsz']), conf=float(self.P['conf']),
                                     verbose=False)[0]
            names = res.names
            for b in res.boxes:
                cls = names[int(b.cls[0])]
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                dets.append({'cls': cls, 'kind': self.classes.get(cls), 'conf': float(b.conf[0]),
                             'box': [x1, y1, x2, y2]})

        seen = {'STOP': None, 'CROSS': None, 'BUMP': None}
        best_light = None
        for det in dets:
            x1, y1, x2, y2 = det['box']
            h = y2 - y1
            if det['kind'] == 'LIGHT':
                dist = distance_from_height(h, self.P['light_height_m'], self.P['focal_px'])
                roi = frame[max(0, int(y1)):int(y2), max(0, int(x1)):int(x2)]
                color, _ = classify_light(roi, self.hsv)
                det['color'] = color
                det['dist'] = round(max(0.0, dist - self.P['camera_to_front_m']), 2)
                if color and (best_light is None or h > best_light[2]):
                    best_light = (color, det['dist'], h)
            elif det['kind'] in seen:
                dist = distance_from_height(h, self.P['sign_height_m'], self.P['focal_px'])
                dist = max(0.0, dist - self.P['camera_to_front_m'])
                det['dist'] = round(dist, 2)
                if dist <= self.P['max_sign_distance_m']:
                    prev = seen[det['kind']]
                    if prev is None or dist < prev:
                        seen[det['kind']] = dist

        for kind, dist in seen.items():
            confirmed = self.sign_conf[kind].update(kind if dist is not None else None)
            if confirmed:
                self.pub_sign.publish(String(data=f'{kind} {dist:.2f}'))

        if best_light is not None:
            color = self.light_conf.update(best_light[0])
            if color:
                self.last_light_seen = time.monotonic()
                self.pub_light.publish(String(data=f'{color} {best_light[1]:.2f}'))
        else:
            self.light_conf.update(None)

        if dets:
            self.pub_det.publish(String(data=json.dumps(
                [{k: v for k, v in d.items() if k not in ('box', 'cls_id')} for d in dets], ensure_ascii=False)))
        self.publish_debug(frame, dets)

    def publish_debug(self, frame, dets):
        now = time.monotonic()
        if now - self.last_debug < self.debug_period or self.pub_dbg.get_subscription_count() == 0:
            return
        self.last_debug = now
        img = frame.copy()
        colors = {'R': (0, 0, 255), 'Y': (0, 220, 255), 'G': (0, 200, 0)}
        for d in dets:
            x1, y1, x2, y2 = (int(v) for v in d['box'])
            c = colors.get(d.get('color'), (255, 180, 0))
            cv2.rectangle(img, (x1, y1), (x2, y2), c, 2)
            label = f'{d["cls"]} {d["conf"]:.2f} {d.get("dist", 0):.1f}m {d.get("color") or ""}'
            cv2.putText(img, label, (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1)
        if self.last_spot:
            x1, y1, x2, y2 = self.last_spot['box']
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.putText(img, f'spot {self.last_spot["x"]:.2f} {self.last_spot["y"]:+.2f}', (x1, y2 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.pub_dbg.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
