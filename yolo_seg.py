#!/usr/bin/env python3
"""YOLO 세그멘테이션(ONNX)을 OpenCV DNN(CUDA)으로 실행한다. torch가 필요 없다.

단독 실행 시 이미지 폴더/파일에 추론 결과를 그려서 보여준다.
    python3 yolo_seg.py datasets/session_xxx/images
    (스페이스/→: 다음, ←: 이전, q/ESC: 종료)
"""

import argparse
import os
import time

import cv2
import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(HERE, 'models', 'best.onnx')


def _palette(n):
    rng = np.random.default_rng(7)
    return [tuple(int(c) for c in rng.integers(60, 256, 3)) for _ in range(n)]


class YoloSeg:
    def __init__(self, model=DEFAULT_MODEL, names=None, imgsz=640,
                 conf=0.25, iou=0.45, cuda=True):
        self.net = cv2.dnn.readNetFromONNX(model)
        if cuda and cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        self.out_names = self.net.getUnconnectedOutLayersNames()
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        names_file = names or os.path.splitext(model)[0] + '.names'
        if os.path.exists(names_file):
            with open(names_file) as f:
                self.names = [line.strip() for line in f if line.strip()]
        else:
            self.names = None
        self.colors = None

    def _letterbox(self, img):
        h, w = img.shape[:2]
        r = min(self.imgsz / h, self.imgsz / w)
        nw, nh = round(w * r), round(h * r)
        px, py = (self.imgsz - nw) / 2, (self.imgsz - nh) / 2
        out = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        top, left = round(py - 0.1), round(px - 0.1)
        out = cv2.copyMakeBorder(out, top, self.imgsz - nh - top,
                                 left, self.imgsz - nw - left,
                                 cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return out, r, left, top

    def __call__(self, img):
        """반환: [(class_id, score, (x1,y1,x2,y2), mask(bool HxW))]"""
        h, w = img.shape[:2]
        lb, r, left, top = self._letterbox(img)
        blob = cv2.dnn.blobFromImage(lb, 1 / 255.0, swapRB=True)
        self.net.setInput(blob)
        pred, proto = self.net.forward(self.out_names)
        pred = pred[0].T                     # (8400, 4 + nc + 32)
        nm = proto.shape[1]
        nc = pred.shape[1] - 4 - nm
        if self.names is None:
            self.names = [str(i) for i in range(nc)]
        if self.colors is None:
            self.colors = _palette(nc)

        scores_all = pred[:, 4:4 + nc]
        cls = scores_all.argmax(1)
        scores = scores_all[np.arange(len(cls)), cls]
        keep = scores > self.conf
        pred, cls, scores = pred[keep], cls[keep], scores[keep]
        if len(pred) == 0:
            return []

        cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        boxes_xywh = np.stack([cx - bw / 2, cy - bh / 2, bw, bh], 1)
        idx = cv2.dnn.NMSBoxesBatched(boxes_xywh.tolist(), scores.tolist(),
                                      cls.tolist(), self.conf, self.iou)
        if len(idx) == 0:
            return []
        idx = np.array(idx).flatten()[:100]
        pred, cls, scores, boxes_xywh = pred[idx], cls[idx], scores[idx], boxes_xywh[idx]

        # 마스크: (n,32) @ (32, 160*160) -> sigmoid
        _, _, mh, mw = proto.shape
        masks = pred[:, 4 + nc:] @ proto[0].reshape(nm, -1)
        masks = 1 / (1 + np.exp(-masks))
        masks = masks.reshape(-1, mh, mw)

        # letterbox 영역만 잘라서 원본 크기로
        s = mh / self.imgsz
        y0, x0 = int(round(top * s)), int(round(left * s))
        y1, x1 = int(round((self.imgsz - top) * s)), int(round((self.imgsz - left) * s))

        results = []
        for m, c, sc, b in zip(masks, cls, scores, boxes_xywh):
            x1b = (b[0] - left) / r
            y1b = (b[1] - top) / r
            x2b = x1b + b[2] / r
            y2b = y1b + b[3] / r
            box = (int(max(0, x1b)), int(max(0, y1b)),
                   int(min(w - 1, x2b)), int(min(h - 1, y2b)))
            m = cv2.resize(m[y0:y1, x0:x1], (w, h), interpolation=cv2.INTER_LINEAR) > 0.5
            crop = np.zeros_like(m)
            crop[box[1]:box[3] + 1, box[0]:box[2] + 1] = True
            results.append((int(c), float(sc), box, m & crop))
        return results

    def draw(self, img, results, alpha=0.45):
        out = img.copy()
        overlay = img.copy()
        for c, _, _, m in results:
            overlay[m] = self.colors[c]
        cv2.addWeighted(overlay, alpha, out, 1 - alpha, 0, out)
        for c, sc, (x1, y1, x2, y2), _ in results:
            color = self.colors[c]
            label = f'{self.names[c]} {sc:.2f}'
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(y1, th + 4)
            cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 2, ty), color, -1)
            cv2.putText(out, label, (x1 + 1, ty - 3), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('source', help='이미지 파일 또는 폴더')
    p.add_argument('--model', default=DEFAULT_MODEL)
    p.add_argument('--conf', type=float, default=0.25)
    p.add_argument('--save', help='결과 이미지를 저장할 폴더')
    args = p.parse_args()

    if os.path.isdir(args.source):
        files = sorted(os.path.join(args.source, f) for f in os.listdir(args.source)
                       if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')))
    else:
        files = [args.source]
    if not files:
        raise SystemExit('이미지가 없습니다.')

    yolo = YoloSeg(args.model, conf=args.conf)
    if args.save:
        os.makedirs(args.save, exist_ok=True)
    i = 0
    while 0 <= i < len(files):
        img = cv2.imread(files[i])
        t = time.perf_counter()
        res = yolo(img)
        ms = (time.perf_counter() - t) * 1000
        print(f'[{i + 1}/{len(files)}] {os.path.basename(files[i])} {ms:.0f}ms: '
              + ', '.join(f'{yolo.names[c]}({s:.2f})' for c, s, _, _ in res))
        vis = yolo.draw(img, res)
        if args.save:
            cv2.imwrite(os.path.join(args.save, os.path.basename(files[i])), vis)
            i += 1
            continue
        cv2.imshow('YOLO', vis)
        key = cv2.waitKeyEx(0)
        if key in (ord('q'), 27):
            break
        i += -1 if key in (65361, 2424832, ord('a')) else 1
        i = max(i, 0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
