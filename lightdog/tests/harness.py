"""离线逻辑回归的公共底座。

为什么要有这个东西：P2/P3 的**逻辑**（速度仲裁、回充状态机、帧新鲜度、分段写盘）
不依赖 DDS 和物理引擎，它们只是普通方法，只用到 math / time / numpy / cv2。
但这些方法住在 rclpy 节点类里，而宿主机上没有 ROS。

做法：用 ast 把目标方法从源文件里**原样**抽出来（不是复制一份代码，源文件改了
测试自动跟着变），绑定到一个假 self 上执行。这样既不用装 ROS，也不会出现
「测试里那份副本和真代码不一致」这种最坑的情况。

约束：被测方法必须是纯逻辑，不能碰 rclpy 的真实对象。凡是用了
create_publisher / create_timer / get_parameter 的 __init__ 都不抽，
而是在测试里手工布置状态 —— 这也是刻意为之：状态布置得越显式，
测试表达式越接近「这一段逻辑到底在保证什么」。
"""

import ast
import os
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# 测试放在包内（lightdog/tests/），所以包目录就是上一级；
# 但也兼容历史上放在仓库根的布局（仓库根/lightdog）。
PKG = os.path.join(ROOT, 'lightdog') if os.path.isdir(os.path.join(ROOT, 'lightdog')) else ROOT


# ---------------------------------------------------------------- 源码抽取

def _strip_decorators(code):
    """ast.get_source_segment 会把 @decorator 一起带出来，exec 后会变成
    staticmethod 对象而不是函数，所以先剥掉。"""
    out = []
    for ln in code.splitlines():
        if not out and ln.lstrip().startswith('@'):
            continue
        out.append(ln)
    return '\n'.join(out)


def source_of(name):
    return os.path.join(PKG, name)


def extract_methods(path, cls_name, names, ns=None):
    """从 path 的 cls_name 类里抽出 names 指定的方法，返回 {名字: 函数}。"""
    with open(path, encoding='utf-8') as f:
        src = f.read()
    tree = ast.parse(src)
    names = set(names)
    found = {}
    for node in tree.body:
        if not (isinstance(node, ast.ClassDef) and node.name == cls_name):
            continue
        for fn in node.body:
            if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)) and fn.name in names:
                code = _strip_decorators(ast.get_source_segment(src, fn))
                g = dict(ns or {})
                exec(compile(code, f'{cls_name}.{fn.name}', 'exec'), g)
                found[fn.name] = g[fn.name]
    missing = names - set(found)
    if missing:
        raise KeyError(
            f'{os.path.basename(path)} 的 {cls_name} 里找不到 '
            f'{sorted(missing)}（方法被改名或删了？）')
    return found


def extract_funcs(path, names, ns=None):
    """从模块顶层抽普通函数（safe_token / clamp_sec 这类纯函数）。

    和 extract_methods 同一个理由：直接执行源文件里的那份代码，
    不复制实现，改源文件测试自动跟着变。
    """
    with open(path, encoding='utf-8') as f:
        src = f.read()
    tree = ast.parse(src)
    names = set(names)
    found = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            code = _strip_decorators(ast.get_source_segment(src, node))
            g = dict(ns or {})
            exec(compile(code, node.name, 'exec'), g)
            found[node.name] = g[node.name]
    missing = names - set(found)
    if missing:
        raise KeyError(f'{os.path.basename(path)} 里找不到顶层函数 {sorted(missing)}')
    return found


def extract_const(path, name):
    """抽模块级常量（RULES / ALERT_HOLD_STATES 这种纯字面量）。

    抽出来的函数体里如果引用了模块级常量，必须把它一起塞进 ns，
    否则 exec 出来的函数会在调用时 NameError —— 而且是在被测逻辑**内部**
    炸，报错位置离真正的原因很远。用这个函数取，别在测试里抄一份副本。
    """
    with open(path, encoding='utf-8') as f:
        src = f.read()
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == name:
                    return ast.literal_eval(node.value)
    raise KeyError(f'{os.path.basename(path)} 里找不到模块常量 {name}')


def extract_class(path, cls_name, ns=None):
    """整类抽取（用于 CmdVelSource 这种纯数据类）。"""
    with open(path, encoding='utf-8') as f:
        src = f.read()
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            g = dict(ns or {})
            exec(compile(ast.get_source_segment(src, node), cls_name, 'exec'), g)
            return g[cls_name]
    raise KeyError(f'{os.path.basename(path)} 里找不到类 {cls_name}')


def bind(obj, funcs):
    for name, fn in funcs.items():
        setattr(obj, name, types.MethodType(fn, obj))
    return obj


# ---------------------------------------------------------------- 消息假件

class Vec3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def __repr__(self):
        return f'({self.x:.3f}, {self.y:.3f}, {self.z:.3f})'


class Twist:
    def __init__(self):
        self.linear = Vec3()
        self.angular = Vec3()

    def is_zero(self, eps=1e-9):
        return (abs(self.linear.x) < eps and abs(self.linear.y) < eps
                and abs(self.angular.z) < eps)

    def __repr__(self):
        return f'Twist(v={self.linear.x:.3f}, w={self.angular.z:.3f})'


class Msg:
    """Bool / Float32 / String 同构，只用 .data。"""

    def __init__(self, data=None):
        self.data = data

    def __repr__(self):
        return f'Msg({self.data!r})'


Bool = Float32 = String = Msg


class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Header:
    def __init__(self, frame_id='base_link'):
        self.frame_id = frame_id
        self.stamp = None


class PointStamped:
    """geometry_msgs/PointStamped 的最小替身（带 header，跟随器要查 frame_id）。"""

    def __init__(self, x=0.0, y=0.0, frame_id='base_link'):
        self.point = Point(x, y)
        self.header = Header(frame_id)


class Pub:
    """记录所有发布的假 publisher。"""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)

    @property
    def last(self):
        return self.sent[-1] if self.sent else None

    @property
    def values(self):
        return [m.data for m in self.sent]

    def clear(self):
        self.sent.clear()


class Log:
    def __init__(self, echo=False):
        self.lines = []
        self.echo = echo

    def _rec(self, level):
        def f(msg, **kw):
            self.lines.append((level, str(msg)))
            if self.echo:
                print(f'      [{level}] {msg}')
        return f

    def __getattr__(self, name):
        return self._rec(name)

    def text(self, level=None):
        return '\n'.join(m for lv, m in self.lines if level is None or lv == level)


class RosTime:
    def __init__(self, ns):
        self.nanoseconds = int(ns)

    def __sub__(self, other):
        return RosTime(self.nanoseconds - other.nanoseconds)


class Clock:
    """可控 ROS 时钟。墙钟走多快与它无关，由测试显式 advance()。"""

    def __init__(self, start=0.0):
        self.ns = int(start * 1e9)

    def now(self):
        return RosTime(self.ns)

    def advance(self, sec):
        self.ns += int(round(sec * 1e9))
        return self.now()


class _Param:
    def __init__(self, value):
        self.value = value


class Params:
    """模拟 declare_parameter + get_parameter：没声明的名字直接报错。"""

    def __init__(self, values):
        self._v = dict(values)

    def __call__(self, name):
        if name not in self._v:
            raise KeyError(f'参数 {name} 未声明（测试里的参数表漏了？）')
        return _Param(self._v[name])


class Future:
    """模拟 rclpy 的 Future：result() 返回预设值。"""

    def __init__(self, value):
        self._v = value
        self._cb = []

    def result(self):
        return self._v

    def add_done_callback(self, cb):
        self._cb.append(cb)


class GoalHandle:
    """模拟 action goal handle，记录 cancel_goal_async 是否真的被调用。"""

    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancelled = 0
        self.result_cbs = []
        self.result_future = Future(None)

    def cancel_goal_async(self):
        self.cancelled += 1
        return Future(True)

    def get_result_async(self):
        return self.result_future
