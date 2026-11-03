"""onnx 检测后端的预处理/后处理回归。

为什么这几段值得单独测：它们是**纯几何**，错了不会抛异常。
漏乘 scale 的框还是框、漏减 padding 的框还是框，只是位置不对 ——
表现是「检测得到人，但距离永远差一截」，查起来要绕很久。

所以这里不测「能不能跑」，只测**位置对不对**，用一个正向映射做基准：
已知原图上的矩形 -> 手工算到 canvas 坐标 -> decode -> 断言回到原矩形。
"""

import os
import sys
import time
import types
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import (extract_funcs, extract_const, extract_methods,   # noqa: E402
                     bind, source_of, Log)

SRC = source_of('lightdog_person_detector.py')


def load():
    """按依赖顺序抽取纯函数（每个函数在独立的 globals 里 exec，所以要分步）。"""
    ns = {'np': np}
    ns['ONNX_PAD'] = extract_const(SRC, 'ONNX_PAD')
    ns.update(extract_funcs(SRC, ['_resize_bilinear'], ns=ns))
    ns.update(extract_funcs(SRC, ['nms'], ns=ns))
    ns.update(extract_funcs(SRC, ['letterbox', 'decode_yolo_onnx'], ns=ns))
    ns.update(extract_funcs(SRC, ['check_onnx_input'], ns=ns))
    return ns


F = load()


def letterbox_params(img_w, img_h, size):
    """手工复算 letterbox 的 scale/dx/dy，用来交叉验证实现没算错。

    刻意在测试里独立写一遍而不是调实现的返回值 —— 否则实现错了
    测试也跟着错，等于没测。
    """
    scale = min(size / float(img_w), size / float(img_h))
    nw = max(1, int(round(img_w * scale)))
    nh = max(1, int(round(img_h * scale)))
    return scale, (size - nw) // 2, (size - nh) // 2


def forward(box, img_w, img_h, size):
    """原图 xyxy -> canvas 上的 (cx, cy, bw, bh)。decode 的逆运算。"""
    scale, dx, dy = letterbox_params(img_w, img_h, size)
    x1, y1, x2, y2 = box
    return ((x1 + x2) * 0.5 * scale + dx,
            (y1 + y2) * 0.5 * scale + dy,
            (x2 - x1) * scale,
            (y2 - y1) * scale)


def fake_output(entries, num_cls=80, num_anchors=2100, person_class=0):
    """构造 YOLOv8 的原始输出 (1, 4+nc, N)。entries: [(cx,cy,bw,bh,score)]。"""
    out = np.zeros((1, 4 + num_cls, num_anchors), dtype=np.float32)
    for i, (cx, cy, bw, bh, sc) in enumerate(entries):
        out[0, 0, i], out[0, 1, i] = cx, cy
        out[0, 2, i], out[0, 3, i] = bw, bh
        out[0, 4 + person_class, i] = sc
    return out


class TestLetterbox(unittest.TestCase):

    def test_640x480_scales_to_full_width_with_vertical_bars(self):
        # 640x480 -> 320：scale=0.5，正好铺满宽度 320，高度 240，上下各垫 40
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        canvas, scale, dx, dy = F['letterbox'](rgb, 320)
        self.assertAlmostEqual(scale, 0.5, places=6)
        self.assertEqual((canvas.shape[0], canvas.shape[1]), (320, 320))
        self.assertEqual((dx, dy), (0, 40))

    def test_padding_is_the_ultralytics_gray_not_zero(self):
        # 填 0（纯黑）也能推理，只是边缘处特征和训练时不一致，会掉分。
        # 训练时用的是 114 灰，这里必须对上。
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        canvas, _, _, _ = F['letterbox'](rgb, 320)
        self.assertEqual(int(canvas[0, 0, 0]), 114)
        self.assertEqual(int(canvas[10, 160, 1]), 114)
        self.assertEqual(int(canvas[319, 319, 2]), 114)

    def test_content_lands_exactly_where_dx_dy_says(self):
        """算出来的 dx/dy 必须等于实际粘贴位置。

        这是最容易出错的一环：dx/dy 用 int(round) 算、粘贴用切片，
        两边一旦不一致，还原时就会整体偏一两个像素起步。

        分横竖两例：640x480 只在上下垫（dx=0），480x640 只在左右垫（dy=0）。
        只在有填充的那一侧断言 —— dx=0 时写 canvas[dy, dx-1] 会变成
        numpy 的负索引，绕到最右列去，看着像实现错了。
        """
        # 横向内容：上下垫 dy=40
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb[0:10, 0:10] = 255            # 左上角一块白
        canvas, scale, dx, dy = F['letterbox'](rgb, 320)
        self.assertEqual(dx, 0)
        self.assertGreater(int(canvas[dy, dx, 0]), 200)     # 白块从这里开始
        self.assertEqual(int(canvas[dy - 1, dx, 0]), 114)   # 上面还是填充

        # 竖向内容：左右垫 dx=40
        rgb2 = np.zeros((640, 480, 3), dtype=np.uint8)
        rgb2[0:10, 0:10] = 255
        canvas2, _, dx2, dy2 = F['letterbox'](rgb2, 320)
        self.assertEqual(dy2, 0)
        self.assertGreater(int(canvas2[dy2, dx2, 0]), 200)
        self.assertEqual(int(canvas2[dy2, dx2 - 1, 0]), 114)  # 左边还是填充

    def test_square_input_needs_no_padding(self):
        rgb = np.zeros((640, 640, 3), dtype=np.uint8)
        canvas, scale, dx, dy = F['letterbox'](rgb, 320)
        self.assertAlmostEqual(scale, 0.5, places=6)
        self.assertEqual((dx, dy), (0, 0))
        self.assertEqual(canvas.shape, (320, 320, 3))

    def test_portrait_input_pads_horizontally(self):
        # 竖图（例如手机竖拍）应该左右垫，上下铺满
        rgb = np.zeros((640, 480, 3), dtype=np.uint8)
        _, scale, dx, dy = F['letterbox'](rgb, 320)
        self.assertAlmostEqual(scale, 0.5, places=6)
        self.assertEqual((dx, dy), (40, 0))

    def test_output_dtype_stays_uint8(self):
        # 转 float32 是给推理用的，canvas 本身保持 uint8：
        # 后面还要乘 255 反向转回来才是一份正常图像，float 会白占 4 倍内存。
        rgb = np.full((480, 640, 3), 200, dtype=np.uint8)
        canvas, _, _, _ = F['letterbox'](rgb, 320)
        self.assertEqual(canvas.dtype, np.uint8)

    def test_empty_image_raises_instead_of_silently_returning(self):
        with self.assertRaises(ValueError):
            F['letterbox'](np.zeros((0, 0, 3), dtype=np.uint8), 320)


class TestResize(unittest.TestCase):

    def test_same_size_is_identity(self):
        img = np.arange(4 * 4 * 3, dtype=np.uint8).reshape(4, 4, 3)
        out = F['_resize_bilinear'](img, 4, 4)
        np.testing.assert_allclose(out, img.astype(np.float32))

    def test_uniform_image_stays_uniform(self):
        # 双线性的常见 bug：边界像素权重没归零导致边缘变暗
        img = np.full((7, 5, 3), 180, dtype=np.uint8)
        out = F['_resize_bilinear'](img, 13, 19)
        self.assertLess(float(np.abs(out - 180.0).max()), 1e-3)

    def test_downscale_keeps_average_close(self):
        img = np.zeros((64, 64, 3), dtype=np.uint8)
        img[:, :32] = 100
        out = F['_resize_bilinear'](img, 32, 32)
        # 左半仍然明显亮于右半（没有整体平移/翻转）
        self.assertGreater(float(out[:, :8].mean()), float(out[:, -8:].mean()) + 50)


class TestNms(unittest.TestCase):

    def test_overlapping_boxes_collapse_to_the_higher_score(self):
        boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11]], dtype=np.float32)
        keep = F['nms'](boxes, np.array([0.9, 0.8], dtype=np.float32), 0.45)
        self.assertEqual(keep, [0])

    def test_disjoint_boxes_both_survive(self):
        boxes = np.array([[0, 0, 10, 10], [50, 50, 60, 60]], dtype=np.float32)
        keep = F['nms'](boxes, np.array([0.9, 0.7], dtype=np.float32), 0.45)
        self.assertEqual(sorted(keep), [0, 1])

    def test_empty_input(self):
        self.assertEqual(F['nms'](np.zeros((0, 4)), np.zeros((0,)), 0.45), [])

    def test_zero_area_boxes_do_not_divide_by_zero(self):
        # 退化框（w 或 h = 0）在低分辨率下真的会出现。两个零面积框的交集是 0、
        # 并集也是 0 —— 这里要保证**不崩、不出 nan**，而不是谁抑制谁：
        # 交集为 0 就按「不重叠」处理（都留下）是唯一不产生 nan 的合理定义。
        boxes = np.array([[5, 5, 5, 5], [5, 5, 5, 5]], dtype=np.float32)
        keep = F['nms'](boxes, np.array([0.9, 0.8], dtype=np.float32), 0.45)
        self.assertEqual(sorted(keep), [0, 1])


class TestDecode(unittest.TestCase):

    def test_roundtrip_returns_the_original_box(self):
        """端到端：原图矩形 -> canvas 坐标 -> decode -> 应回到原矩形。

        这一条同时钉住 scale、dx、dy 三者的用法。任何一项写错，
        结果都会偏出容差（偏多少恰好等于那一项的量）。
        """
        w, h, size = 640, 480, 320
        box = (100.0, 80.0, 260.0, 400.0)
        cx, cy, bw, bh = forward(box, w, h, size)
        out = fake_output([(cx, cy, bw, bh, 0.92)])
        dets = F['decode_yolo_onnx'](out, 0.5, 0, 40, w, h, conf_thr=0.35)
        self.assertEqual(len(dets), 1)
        x1, y1, x2, y2, sc = dets[0]
        self.assertAlmostEqual(x1, box[0], delta=1.0)
        self.assertAlmostEqual(y1, box[1], delta=1.0)
        self.assertAlmostEqual(x2, box[2], delta=1.0)
        self.assertAlmostEqual(y2, box[3], delta=1.0)
        self.assertAlmostEqual(sc, 0.92, places=5)

    def test_roundtrip_with_horizontal_padding(self):
        """竖图（dx≠0）的往返：上一条只覆盖了 dy，这一条补 dx。

        两个方向都写，是因为符号写反时只有一个方向会露馅 ——
        640x480 那张图 dx=0，dx 写错也测不出来。
        """
        w, h, size = 480, 640, 320
        box = (60.0, 100.0, 300.0, 500.0)
        cx, cy, bw, bh = forward(box, w, h, size)
        out = fake_output([(cx, cy, bw, bh, 0.88)])
        dets = F['decode_yolo_onnx'](out, 0.5, 40, 0, w, h, conf_thr=0.35)
        self.assertEqual(len(dets), 1)
        x1, y1, x2, y2, _ = dets[0]
        self.assertAlmostEqual(x1, box[0], delta=1.0)
        self.assertAlmostEqual(x2, box[2], delta=1.0)

    def test_forgetting_padding_would_shift_the_box(self):
        """反向验证：把 dy 当 0 传进去，y 必须整体偏 dy/scale。

        如果这条测试解出来的 y 和上一条一样，说明 decode 里根本没用到 dy
        —— 那上一条的「通过」就是假的。
        """
        w, h, size = 640, 480, 320
        box = (100.0, 80.0, 260.0, 400.0)
        cx, cy, bw, bh = forward(box, w, h, size)
        out = fake_output([(cx, cy, bw, bh, 0.92)])
        dets = F['decode_yolo_onnx'](out, 0.5, 0, 0, w, h, conf_thr=0.35)
        # 少了 dy=40，等价于把框整体往下挪了 40/0.5 = 80 像素（y 变大）
        self.assertAlmostEqual(dets[0][1], box[1] + 80.0, delta=1.0)

    def test_confidence_threshold_filters(self):
        w, h = 640, 480
        cx, cy, bw, bh = forward((100, 80, 260, 400), w, h, 320)
        out = fake_output([(cx, cy, bw, bh, 0.30)])
        self.assertEqual(F['decode_yolo_onnx'](out, 0.5, 0, 40, w, h, conf_thr=0.35), [])
        self.assertEqual(len(F['decode_yolo_onnx'](
            out, 0.5, 0, 40, w, h, conf_thr=0.25)), 1)

    def test_accepts_transposed_output_layout(self):
        # 有些导出器给 (1, N, 4+nc)。布局判错的话会把 2100 当成类别数，
        # 解出一堆垃圾框而不是报错 —— 所以必须显式支持。
        w, h = 640, 480
        cx, cy, bw, bh = forward((100, 80, 260, 400), w, h, 320)
        a = fake_output([(cx, cy, bw, bh, 0.9)])[0]      # (84, 2100)
        dets = F['decode_yolo_onnx'](a.T[None], 0.5, 0, 40, w, h, conf_thr=0.35)
        self.assertEqual(len(dets), 1)
        self.assertAlmostEqual(dets[0][0], 100.0, delta=1.0)

    def test_class_index_beyond_model_is_a_hard_error(self):
        # 模型只有 3 类却把 onnx_person_class 设成 5：静默返回空列表会表现成
        # 「永远检测不到人」，所以这里要求抛错，让启动预热阶段就暴露。
        out = fake_output([(100, 100, 20, 40, 0.9)], num_cls=3)
        with self.assertRaises(ValueError):
            F['decode_yolo_onnx'](out, 1.0, 0, 0, 640, 480, person_class=5)

    def test_coordinates_are_clipped_to_the_image(self):
        # padding 区域解出来的坐标是负的；直接留着会让下游取深度时
        # numpy 负索引静默绕回（比报错更难查）。
        w, h = 640, 480
        # 故意造一个中心在填充区里的框
        out = fake_output([(0.0, 0.0, 40.0, 40.0, 0.9)])
        dets = F['decode_yolo_onnx'](out, 0.5, 0, 40, w, h, conf_thr=0.35)
        self.assertEqual(len(dets), 1)
        self.assertGreaterEqual(dets[0][0], 0.0)
        self.assertGreaterEqual(dets[0][1], 0.0)

    def test_empty_output_returns_empty_list(self):
        out = np.zeros((1, 84, 2100), dtype=np.float32)
        self.assertEqual(F['decode_yolo_onnx'](out, 0.5, 0, 40, 640, 480), [])

    def test_nms_is_applied_to_multiple_overlapping_candidates(self):
        w, h = 640, 480
        cx, cy, bw, bh = forward((100, 80, 260, 400), w, h, 320)
        out = fake_output([(cx, cy, bw, bh, 0.90),
                           (cx + 2, cy + 2, bw, bh, 0.80)])
        dets = F['decode_yolo_onnx'](out, 0.5, 0, 40, w, h,
                                     conf_thr=0.35, iou_thr=0.45)
        self.assertEqual(len(dets), 1)
        self.assertAlmostEqual(dets[0][4], 0.90, places=5)

    def test_results_are_sorted_by_score(self):
        w, h = 640, 480
        a = forward((10, 10, 60, 200), w, h, 320)
        b = forward((400, 10, 450, 200), w, h, 320)
        out = fake_output([(a[0], a[1], a[2], a[3], 0.50),
                           (b[0], b[1], b[2], b[3], 0.95)])
        dets = F['decode_yolo_onnx'](out, 0.5, 0, 40, w, h, conf_thr=0.35)
        self.assertEqual(len(dets), 2)
        self.assertGreater(dets[0][4], dets[1][4])


class TestInputShapeCheck(unittest.TestCase):
    """模型输入形状的校验。

    这条校验是「坐标整体偏且不报错」的唯一防线：letterbox 按 onnx_size 做，
    模型如果是 640 而参数写 320，推理照样跑、框照样解，只是全都差一个比例。
    真模型不在仓库里，所以把校验抽成纯函数单独测。
    """

    def test_matching_static_shape_passes(self):
        self.assertIsNone(F['check_onnx_input']([1, 3, 320, 320], 320))

    def test_wrong_static_size_is_rejected_with_both_numbers(self):
        msg = F['check_onnx_input']([1, 3, 640, 640], 320)
        self.assertIsNotNone(msg)
        # 报错里必须同时出现「模型是 640」和「参数是 320」，
        # 只说「尺寸不匹配」的话现场还得自己去翻模型
        self.assertIn('640', msg)
        self.assertIn('320', msg)

    def test_dynamic_dimensions_are_accepted(self):
        # 导出时开了 dynamic 的模型维度是字符串，应该按参数走而不是拒绝
        self.assertIsNone(F['check_onnx_input']([1, 3, 'height', 'width'], 320))
        self.assertIsNone(F['check_onnx_input'](['batch', 3, 320, 320], 320))

    def test_non_4d_input_is_rejected(self):
        self.assertIsNotNone(F['check_onnx_input']([1, 3, 320], 320))
        self.assertIsNotNone(F['check_onnx_input']([320, 320], 320))

    def test_non_3_channel_is_rejected(self):
        # 灰度或 RGBA 模型：推理能跑，但通道数对不上时像素全是错的
        self.assertIsNotNone(F['check_onnx_input']([1, 1, 320, 320], 320))
        self.assertIsNotNone(F['check_onnx_input']([1, 4, 320, 320], 320))


class TestBackendFallback(unittest.TestCase):
    """加载失败必须回退，不能「启动成功但永远检测不到人」。"""

    def _make(self):
        ns = dict(F)
        ns['check_onnx_input'] = F['check_onnx_input']
        fns = extract_methods(SRC, 'PersonDetector', ['_load_onnx'], ns=ns)
        log = Log()
        obj = types.SimpleNamespace(
            onnx_size=320, onnx_iou=0.45, onnx_person_class=0,
            get_logger=lambda: log)
        return bind(obj, fns), log

    def test_missing_model_returns_none_and_logs_fallback(self):
        obj, log = self._make()
        sess, name = obj._load_onnx('definitely-not-in-repo.onnx', 2)
        self.assertIsNone(sess)
        self.assertIsNone(name)
        # 不管宿主机装没装 onnxruntime（走「无法 import」还是「加载失败」），
        # 都必须留下一条明确的回退记录 —— 否则现场只能看到「检测不到人」
        self.assertIn('已回退到 color', log.text('error'))


class FakeSession:
    """冒充 onnxruntime 的 InferenceSession，只记录喂进去的 blob。"""

    def __init__(self, raw):
        self.raw = raw
        self.feeds = []

    def run(self, out_names, feeds):
        self.feeds.append(feeds)
        return [self.raw]


class TestBboxOnnxPipeline(unittest.TestCase):
    """`_bbox_onnx` 的整条链路：letterbox -> blob -> 推理 -> 解码 -> 过滤。

    用假 session 注入预设输出，所以**不需要真的装 onnxruntime**。
    这样能在宿主机上钉住三件容易错的事：blob 的形状、通道顺序、
    以及从「模型输出」到「原图整数框」的换算。
    """

    def _make(self, raw, min_h=30):
        ns = dict(F)
        ns['letterbox'] = F['letterbox']
        ns['decode_yolo_onnx'] = F['decode_yolo_onnx']
        ns['np'] = np
        ns['time'] = time
        fns = extract_methods(
            SRC, 'PersonDetector', ['_bbox_onnx', '_note_onnx_latency'], ns=ns)
        log = Log()
        obj = types.SimpleNamespace(
            _onnx=FakeSession(raw), _onnx_input='images',
            onnx_size=320, onnx_conf=0.35, onnx_iou=0.45, onnx_person_class=0,
            min_h=min_h, _warned_onnx=False, _onnx_ms=[],
            _onnx_slow_warned=False, get_logger=lambda: log)
        return bind(obj, fns), log

    def _img(self):
        # 灰色底 + 一块橙色「人」，640x480
        rgb = np.full((480, 640, 3), 40, dtype=np.uint8)
        rgb[80:400, 100:260] = (230, 90, 20)
        return rgb

    def test_whole_chain_returns_the_box_in_original_coordinates(self):
        box = (100.0, 80.0, 260.0, 400.0)
        cx, cy, bw, bh = forward(box, 640, 480, 320)
        obj, _ = self._make(fake_output([(cx, cy, bw, bh, 0.93)]))
        got = obj._bbox_onnx(self._img())
        self.assertIsNotNone(got)
        for a, b in zip(got, (100, 80, 260, 400)):
            self.assertLessEqual(abs(a - b), 1)

    def test_blob_shape_and_range_match_what_the_model_expects(self):
        # (1,3,320,320) NCHW、float32、0~1。形状错的话 onnxruntime 会报错，
        # 但值域错（忘了 /255）不报错，只是分数全乱 —— 这条挡的是后者。
        cx, cy, bw, bh = forward((100, 80, 260, 400), 640, 480, 320)
        obj, _ = self._make(fake_output([(cx, cy, bw, bh, 0.93)]))
        obj._bbox_onnx(self._img())
        blob = obj._onnx.feeds[0]['images']
        self.assertEqual(blob.shape, (1, 3, 320, 320))
        self.assertEqual(blob.dtype, np.float32)
        self.assertGreaterEqual(float(blob.min()), 0.0)
        self.assertLessEqual(float(blob.max()), 1.0)

    def test_channel_order_is_bgr(self):
        # 纯红像素 (255,0,0)：按 BGR 排列后第 0 通道是 0、第 2 通道是 1。
        # 传反了推理照样能跑，只是分数低一截 —— 那种「偶尔漏检」最难归因。
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb[:, :] = (255, 0, 0)
        cx, cy, bw, bh = forward((100, 80, 260, 400), 640, 480, 320)
        obj, _ = self._make(fake_output([(cx, cy, bw, bh, 0.93)]))
        obj._bbox_onnx(rgb)
        blob = obj._onnx.feeds[0]['images'][0]
        # 中间那一片是内容区（不是 padding），取它的中心
        v = blob[:, 160, 160]
        self.assertAlmostEqual(float(v[0]), 0.0, places=3)   # B
        self.assertAlmostEqual(float(v[2]), 1.0, places=3)   # R

    def test_no_detection_returns_none(self):
        obj, _ = self._make(fake_output([(160, 160, 40, 80, 0.10)]))
        self.assertIsNone(obj._bbox_onnx(self._img()))

    def test_too_short_box_is_rejected_like_the_other_backends(self):
        # 只剩一条横带（贴脸）时不能当成完整的人，min_h 三个后端共用
        box = (100.0, 300.0, 500.0, 315.0)
        cx, cy, bw, bh = forward(box, 640, 480, 320)
        obj, _ = self._make(fake_output([(cx, cy, bw, bh, 0.9)]), min_h=30)
        self.assertIsNone(obj._bbox_onnx(self._img()))

    def test_inference_exception_is_swallowed_and_warned_once(self):
        class Boom:
            def run(self, *a, **k):
                raise RuntimeError('模型崩了')

        ns = dict(F)
        ns['letterbox'] = F['letterbox']
        ns['decode_yolo_onnx'] = F['decode_yolo_onnx']
        ns['np'] = np
        ns['time'] = time
        fns = extract_methods(
            SRC, 'PersonDetector', ['_bbox_onnx', '_note_onnx_latency'], ns=ns)
        log = Log()
        obj = types.SimpleNamespace(
            _onnx=Boom(), _onnx_input='images', onnx_size=320,
            onnx_conf=0.35, onnx_iou=0.45, onnx_person_class=0,
            min_h=30, _warned_onnx=False, _onnx_ms=[],
            _onnx_slow_warned=False, get_logger=lambda: log)
        bind(obj, fns)
        # 单帧失败只丢这一帧，不抛出去（抛出去会让 10Hz 定时器整个停掉）
        self.assertIsNone(obj._bbox_onnx(self._img()))
        self.assertIsNone(obj._bbox_onnx(self._img()))
        self.assertEqual(log.text('error').count('onnx 推理失败'), 1)

    def test_largest_box_wins_when_several_people(self):
        far = (40.0, 40.0, 100.0, 140.0)       # 小
        near = (200.0, 60.0, 560.0, 460.0)     # 大
        a = forward(far, 640, 480, 320)
        b = forward(near, 640, 480, 320)
        obj, _ = self._make(fake_output([(a[0], a[1], a[2], a[3], 0.90),
                                         (b[0], b[1], b[2], b[3], 0.60)]))
        got = obj._bbox_onnx(self._img())
        # 取面积最大的那个，与 color/yolo 后端一致（近处的人才是要跟的）
        self.assertAlmostEqual(got[0], 200, delta=1)


    def test_slow_inference_warns_only_once(self):
        # 慢了要报，但不能每帧都报：6 FPS 下每秒 6 条 warn 会把 journal 刷满，
        # 真正要查的问题反而被淹没。
        obj, log = self._make(fake_output([]))
        for _ in range(5):
            obj._note_onnx_latency(400.0)
        self.assertEqual(log.text('warn').count('超过 333ms'), 1)

    def test_average_is_reported_every_200_frames(self):
        obj, log = self._make(fake_output([]))
        for _ in range(199):
            obj._note_onnx_latency(100.0)
        self.assertNotIn('平均推理', log.text('info'))
        obj._note_onnx_latency(100.0)
        self.assertIn('平均推理', log.text('info'))
        # 100ms/帧 = 10 FPS，日志里得是这么个数，不能只是个 ms
        self.assertIn('10.0 FPS', log.text('info'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
