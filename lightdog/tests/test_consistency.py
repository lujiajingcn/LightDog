"""跨文件一致性检查。

这类问题在仿真里不会报错、也不会打日志，只是「参数写了但没生效」，
比崩溃更难查。此前已经踩过一次：p2_params.yaml 里 aspect_min 写 0.55，
代码默认写 1.20，不带 params 启动就复现「人贴近就丢检」。

检查项：
  1. yaml 里的每个参数，对应节点必须 declare_parameter 过（否则是死参数）
  2. 反过来，每个 declare_parameter 过的参数也必须出现在 yaml 里 ——
     否则真机上想调它只能改代码，而代码里那份默认值是照仿真调的
  3. package.xml 的版本要和 HTTP 的 server_version 一致（对外可观测的版本号）
  4. launch 文件引用的 params 文件必须真实存在
  5. 部署脚本 / 文档里提到的 `lightdog-xxx` 命令必须真实存在
"""

import glob
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import PKG, source_of  # noqa: E402

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

    def test_every_declared_param_is_in_yaml(self):
        """反方向：代码里声明了、yaml 里没有 = 现场调不了。

        这条比上一条更容易漏。上一条（yaml->代码）写错了参数不生效，至少
        还能看出「改了没反应」；这一条漏了新参数，现象是「默认值刚好能用，
        上真机想调的时候发现根本没有这个旋钮」，往往要翻源码才发现。
        """
        index = {}                                   # 节点名 -> yaml 里的参数集
        for cfg in sorted(glob.glob(os.path.join(CONFIG_DIR, '*.yaml'))):
            with open(cfg, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            for node_key, body in data.items():
                if node_module(node_key) is None:
                    continue
                index.setdefault(node_key.strip('/'), set()).update(
                    (body or {}).get('ros__parameters') or {})

        problems = []
        for name, have in sorted(index.items()):
            mod = os.path.join(PKG, name + '.py')
            for pname in sorted(declared_names(mod) - have - IMPLICIT_PARAMS):
                problems.append(
                    f'{os.path.basename(mod)} 的 {pname} 没写进 config/*.yaml'
                    f'（真机上只能改代码才能调它）')
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


class TestSimEchoTruncation(unittest.TestCase):
    """`ros2 topic echo` 默认把字符串截到 128 字符（Jazzy 起）。

    JSON 状态话题普遍三百多字符，于是：
      * `status` 只显示半截，排查时看不到关键字段；
      * `p5.sh watch` 里对 `/alert/state` 做 json.loads —— 截断后必然抛异常，
        被 except 吞掉 -> [alert] 一行都不显示，看着像「状态根本没在发」。
    这类失败**没有任何报错**，只表现为信息缺失，所以用静态检查钉住。
    """

    JSON_TOPICS = ('/recorder/state', '/alert/state', '/mission/state')

    def test_json_topics_are_not_truncated(self):
        sim = os.path.join(os.path.dirname(PKG), 'sim')
        if not os.path.isdir(sim):
            # 仿真容器只挂载包目录（src/lightdog），`sim/` 不在里面。
            # 跳过而不是失败：这项查的是仓库里的脚本，宿主机跑才有意义。
            self.skipTest('找不到 sim/（容器只挂包目录）')
        checked = 0
        for path in sorted(glob.glob(os.path.join(sim, '*.sh'))):
            with open(path, encoding='utf-8') as f:
                lines = f.read().splitlines()
            for ln in lines:
                if 'ros2 topic echo' not in ln:
                    continue
                if not any(t in ln for t in self.JSON_TOPICS):
                    continue
                checked += 1
                self.assertIn('truncate-length', ln,
                              f'{os.path.basename(path)}：JSON 话题被截断 -> '
                              f'{ln.strip()}')
        self.assertGreater(checked, 3, '一个 JSON 话题都没扫到，检查逻辑失效了')


class TestDeployCommandNames(unittest.TestCase):
    """部署脚本/文档里提到的 `lightdog-xxx` 必须真实存在。

    踩过的坑：`verify.sh` 发现急停处于闭锁时，让用户执行
    `sudo lightdog-estop-clear`。但 `install.sh` 只装 `/usr/local/bin/lightdog-estop`
    （解除是位置参数 `clear --yes`），`lightdog-estop-clear` 根本不存在。
    结果就是：现场唯一的解急停入口，打印出来的是一条 `command not found`，
    而车还锁着、人正站在旁边。

    这类错误没人会替你发现 —— 那句话只在「已经出故障」的分支里才打印，
    平时跑得再多次也看不见。所以用静态扫描钉住：
    允许的名字 = nodes.conf 里的单元名 + install.sh 实际安装的可执行文件。
    """

    PAT = re.compile(r'\blightdog-[a-z][a-z0-9-]*')

    def _known(self, repo):
        known = {
            'lightdog.target',      # target 单元，不是节点
            'lightdog-estop',       # install.sh 装到 /usr/local/bin 的那个
        }
        conf = os.path.join(repo, 'deploy', 'nodes.conf')
        with open(conf, encoding='utf-8') as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith('#'):
                    continue
                known.add('lightdog-' + ln.split('|')[0].strip())
        return known

    def test_referenced_commands_exist(self):
        repo = os.path.dirname(PKG)
        deploy = os.path.join(repo, 'deploy')
        if not os.path.isdir(deploy):
            # 仿真容器只挂包目录（src/lightdog），deploy/ 不在里面
            self.skipTest('找不到 deploy/（容器只挂包目录）')

        known = self._known(repo)
        files = (glob.glob(os.path.join(deploy, '*.sh'))
                 + glob.glob(os.path.join(deploy, '*.md'))
                 + glob.glob(os.path.join(repo, 'docs', '*.md'))
                 + [os.path.join(repo, 'Readme.md')])

        bad = []
        scanned = 0
        for path in files:
            if not os.path.isfile(path):
                continue
            scanned += 1
            with open(path, encoding='utf-8') as f:
                for i, ln in enumerate(f, 1):
                    for tok in self.PAT.findall(ln):
                        if tok not in known:
                            bad.append(f'{os.path.basename(path)}:{i} '
                                       f'提到了不存在的命令 {tok}')
        self.assertGreater(scanned, 5, '一个部署文件都没扫到，检查逻辑失效了')
        self.assertEqual(bad, [], '\n'.join(bad))


def _docking_threshold_pair(src):
    """从 patch_upstream.py 里取出 docking_threshold 的补丁对 (旧值, 新值)。"""
    m = re.search(
        r"'      docking_threshold: ([0-9.]+)',\s*\n\s*"
        r"'      docking_threshold: ([0-9.]+)'", src)
    return m.groups() if m else (None, None)


class TestDockingThresholdDocs(unittest.TestCase):
    """docking_threshold 是用户可见的停靠精度（±10cm），三处必须同数。

    patch_upstream.py 的文件头摘要曾经写 `0.05 -> 0.32`，而同一个文件的代码
    实际改的是 0.12、P3 文档也写 0.12 —— 只看文件头的人会以为停靠精度是
    ±32cm。摘要和代码分居同文件的两头，改了一处很容易忘掉另一处。
    """

    def test_header_value_matches_patch(self):
        repo = os.path.dirname(PKG)
        patch = os.path.join(repo, 'sim', 'patch_upstream.py')
        if not os.path.isfile(patch):
            self.skipTest('找不到 sim/（容器只挂包目录）')
        with open(patch, encoding='utf-8') as f:
            src = f.read()
        _old, new = _docking_threshold_pair(src)
        self.assertIsNotNone(new, '找不到 docking_threshold 补丁对，正则要更新')
        m = re.search(r'docking_threshold: 0\.05`\s*→\s*`([0-9.]+)`', src)
        self.assertIsNotNone(m, '文件头摘要里找不到 docking_threshold 的箭头，正则要更新')
        self.assertEqual(m.group(1), new,
                         f'文件头写 {m.group(1)}，代码实际改成 {new}')

    def test_docs_quote_same_value(self):
        repo = os.path.dirname(PKG)
        patch = os.path.join(repo, 'sim', 'patch_upstream.py')
        docs = sorted(glob.glob(os.path.join(repo, 'docs', '*.md')))
        if not os.path.isfile(patch) or not docs:
            self.skipTest('找不到 sim/ 或 docs/（容器只挂包目录）')
        with open(patch, encoding='utf-8') as f:
            _old, new = _docking_threshold_pair(f.read())
        self.assertIsNotNone(new, '找不到 docking_threshold 补丁对，正则要更新')
        bad = []
        for path in docs:
            with open(path, encoding='utf-8') as f:
                for i, ln in enumerate(f, 1):
                    for got in re.findall(
                            r'docking_threshold[`:\s]+([0-9]+\.[0-9]+)', ln):
                        if got != new:
                            bad.append(f'{os.path.basename(path)}:{i} 写 {got}，'
                                       f'补丁实际是 {new}')
        self.assertEqual(bad, [], '\n'.join(bad))


if __name__ == '__main__':
    unittest.main(verbosity=2)
