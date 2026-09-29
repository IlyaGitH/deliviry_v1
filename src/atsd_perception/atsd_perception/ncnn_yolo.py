from pathlib import Path

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None


def letterbox(bgr, size, pad=114):
    h, w = bgr.shape[:2]
    r = min(size / h, size / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dx, dy = (size - nw) / 2.0, (size - nh) / 2.0
    img = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nw, nh) != (w, h) else bgr
    top, left = int(round(dy - 0.1)), int(round(dx - 0.1))
    out = cv2.copyMakeBorder(img, top, size - nh - top, left, size - nw - left,
                             cv2.BORDER_CONSTANT, value=(pad, pad, pad))
    return out, r, left, top


def decode(pred, r, left, top, shape, conf=0.5, iou=0.45, names=None):
    pred = np.asarray(pred, dtype=np.float32)
    if pred.ndim == 3:
        pred = pred[0]
    if pred.shape[0] < pred.shape[1]:
        pred = pred.T
    scores = pred[:, 4:]
    if scores.size == 0:
        return []
    cls = scores.argmax(1)
    cf = scores[np.arange(len(cls)), cls]
    keep = cf >= conf
    if not np.any(keep):
        return []
    b, cls, cf = pred[keep, :4], cls[keep], cf[keep]
    x1 = (b[:, 0] - b[:, 2] / 2 - left) / r
    y1 = (b[:, 1] - b[:, 3] / 2 - top) / r
    x2 = (b[:, 0] + b[:, 2] / 2 - left) / r
    y2 = (b[:, 1] + b[:, 3] / 2 - top) / r
    h, w = shape[:2]
    x1, x2 = np.clip(x1, 0, w), np.clip(x2, 0, w)
    y1, y2 = np.clip(y1, 0, h), np.clip(y2, 0, h)
    offset = cls.astype(np.float32) * 4096.0
    rects = np.stack([x1 + offset, y1, x2 - x1, y2 - y1], 1).tolist()
    idx = cv2.dnn.NMSBoxes(rects, cf.tolist(), conf, iou)
    idx = np.array(idx).reshape(-1) if len(idx) else []
    names = names or {}
    return [{'cls_id': int(cls[i]), 'cls': names.get(int(cls[i]), str(int(cls[i]))), 'conf': float(cf[i]),
             'box': [float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i])]} for i in idx]


def read_metadata(folder: Path):
    meta = folder / 'metadata.yaml'
    names, size = {}, 640
    if meta.exists():
        import yaml
        md = yaml.safe_load(meta.read_text(encoding='utf-8')) or {}
        raw = md.get('names', {})
        names = {int(k): str(v) for k, v in raw.items()} if isinstance(raw, dict) else dict(enumerate(raw))
        imgsz = md.get('imgsz', 640)
        size = int(imgsz[0] if isinstance(imgsz, (list, tuple)) else imgsz)
    return names, size


class NcnnYolo:

    def __init__(self, path, threads=4):
        import ncnn
        self.ncnn = ncnn
        p = Path(path)
        param = p if p.suffix == '.param' else next(iter(sorted(p.glob('*.param'))), None)
        if param is None or not param.with_suffix('.bin').exists():
            raise FileNotFoundError(f'в {p} нет *.param и *.bin')
        self.names, self.size = read_metadata(param.parent)
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = int(threads)
        if self.net.load_param(str(param)) != 0 or self.net.load_model(str(param.with_suffix('.bin'))) != 0:
            raise RuntimeError(f'ncnn не загрузил {param}')
        self.inp = self.net.input_names()[0]
        self.out = sorted(self.net.output_names())[0]

    def predict(self, bgr, conf=0.5, iou=0.45):
        img, r, left, top = letterbox(bgr, self.size)
        x = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32) / 255.0
        with self.net.create_extractor() as ex:
            ex.input(self.inp, self.ncnn.Mat(x))
            _, out = ex.extract(self.out)
        return decode(np.array(out), r, left, top, bgr.shape, conf, iou, self.names)
