"""跨文件一致性检查。

这类问题在仿真里不会报错、也不会打日志，只是「参数写了但没生效」，
比崩溃更难查。此前已经踩过一次：p2_params.yaml 里 aspect_min 写 0.55，
代码默认写 1.20，不带 params 启动就复现「人贴近就丢检」。

检查项：
  1. yaml 里的每个参数，对应节点必须 declare_parameter 过（否则是死参数）
  2. package.xml 的版本要和 HTTP 的 server_version 一致（对外可观测的版本号）
  3. launch 文件引用的 params 文件必须真实存在
"""

import glob
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import PKG, ROOT, source_of  # noqa: E402

try:
    import yaml
except ImportError:                                   # pragma: no cover
    yaml = None

CONFIG_DIR = os.path.join(PKG, 'config')

# rclpy 的 Node 自己就声明了这些，yaml 里写它们是合法的
IMPLICIT_PARAMS = {'use_sim_time'}


def declares(py_path, name):
    with open(py_path, encoding='utf-8') as f:
        src = f.read()
    return re.search(r"declare_parameter\(\s*['\"]%s['\"]" % re.escape(name), src) is not None


def declared_names(py_path):
    with open(py_path, encoding='utf-8') as f:
        src = f.read()
    return set(re.findall(r"declare_parameter\(\s*['\"]([A-Za-z0-9_]+)['\"]", src))


def node_module(node_key):
    """'/lightdog_person_detector' -> lightdog/lightdog_person_detector.py"""
    name = node_key.strip('/')
    path = os.path.join(PKG, name + '.py')
    return path if os.path.exists(path) else None


@unittest.skipIf(yaml is None, '宿主机没有 PyYAML，跳过参数一致性检查')
class TestParamsWired(unittest.TestCase):

    def test_every_yaml_param_is_declared(self):
        problems = []
        inspected = 0
        for cfg in sorted(glob.glob(os.path.join(CONFIG_DIR, '*.yaml'))):
            with open(cfg, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            for node_key, body in data.items():
                mod = node_module(node_key)
                if mod is None:
                    # led_visuals.yaml 的键是 'lightdog_led'（无斜杠），也走这里
                    continue
                inspected += 1
                params = (body or {}).get('ros__parameters') or {}
                for pname in params:
                    if pname in IMPLICIT_PARAMS:
                        continue
                    if not declares(mod, pname):
                        problems.append(
                            f'{os.path.basename(cfg)}: {node_key} 的 {pname} '
                            f'在 {os.path.basename(mod)} 里没声明（写了也不生效）')
        self.assertGreater(inspected, 0, '一个节点都没匹配上，映射规则坏了')
        self.assertEqual(problems, [], '\n'.join(problems))

    def test_twist_mux_hold_le_timeout(self):
        """hold > timeout 会被 CmdVelSource 静默钳制，配置写反了看不出来。"""
        cfg = os.path.join(CONFIG_DIR, 'p2_params.yaml')
        with open(cfg, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = data['/lightdog_twist_mux']['ros__parameters']
        for src in ('nav', 'follow', 'dock'):
            hold = params.get(f'{src}_hold')
            timeout = params.get(f'{src}_timeout')
            if hold is None or timeout is None:
                continue
            self.assertLessEqual(
                hold, timeout,
                f'{src}_hold={hold} 大于 {src}_timeout={timeout}，hold 会失效')


class TestVersions(unittest.TestCase):

    def test_package_and_http_version_match(self):
        with open(os.path.join(PKG, 'package.xml'), encoding='utf-8') as f:
            m = re.search(r'<version>([^<]+)</version>', f.read())
        self.assertIsNotNone(m, 'package.xml 里没有 <version>')
        pkg_ver = m.group(1).strip()

        with open(source_of('lightdog_server.py'), encoding='utf-8') as f:
            src = f.read()
        m = re.search(r"server_version\s*=\s*'LightDog/([^']+)'", src)
        self.assertIsNotNone(m, 'server_version 格式变了，正则失效')
        http_ver = m.group(1).strip()

        self.assertEqual(pkg_ver, http_ver,
                         f'package.xml={pkg_ver} 但 HTTP 报 {http_ver}')

    def test_launch_params_files_exist(self):
        missing = []
        for path in glob.glob(os.path.join(PKG, 'launch', '*.py')):
            with open(path, encoding='utf-8') as f:
                src = f.read()
            for ref in re.findall(r"['\"]config/([A-Za-z0-9_]+\.yaml)['\"]", src):
                if not os.path.exists(os.path.join(CONFIG_DIR, ref)):
                    missing.append(f'{os.path.basename(path)} 引用了不存在的 config/{ref}')
        self.assertEqual(missing, [], '\n'.join(missing))


if __name__ == '__main__':
    unittest.main(verbosity=2)
