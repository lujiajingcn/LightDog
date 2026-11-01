#!/usr/bin/env bash
# 把上游 linorobot2 源码拉到 ./linorobot2/（该目录不入库，见 .gitignore）。
#
# 为什么需要它：我们只把「自己的东西」（lightdog 包 + sim 编排）放进仓库，上游源码按需拉取。
# 运行时并不需要上游源码 —— 仿真用的是已构建好的 linorobot2:gazebo 镜像。
# 拉下来主要是为了读代码 / 改补丁 / 重建镜像。
#
# 用法：
#   scripts/fetch_upstream.sh          # 首次拉取
#   scripts/fetch_upstream.sh --force  # 删掉已有目录重拉
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/upstream.lock"

DEST="$ROOT/linorobot2"

# WorkBuddy/沙箱环境自带的代理会吃掉 git 请求，必须清空走直连
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy

if [[ "${1:-}" == "--force" ]]; then
    echo "[fetch] --force: 删除已有 $DEST"
    rm -rf "$DEST"
fi

if [[ -d "$DEST/.git" ]]; then
    HAVE="$(git -C "$DEST" rev-parse HEAD 2>/dev/null || echo none)"
    if [[ "$HAVE" == "$UPSTREAM_COMMIT" ]]; then
        echo "[fetch] 已就绪: $UPSTREAM_PROJECT @ ${UPSTREAM_COMMIT:0:8}"
        exit 0
    fi
    echo "[fetch] 已有目录版本不符 ($HAVE)，重新拉取"
    rm -rf "$DEST"
fi

clone_from() {
    echo "[fetch] clone from $1"
    git clone --filter=blob:none --no-checkout --branch "$UPSTREAM_BRANCH" "$1" "$DEST"
}

if ! clone_from "$UPSTREAM_MIRROR"; then
    echo "[fetch] 镜像失败，尝试直连"
    rm -rf "$DEST"
    clone_from "$UPSTREAM_URL"
fi

# 只取需要的那个提交，不做全历史检出
git -C "$DEST" fetch --depth 1 origin "$UPSTREAM_COMMIT" || true
git -C "$DEST" checkout --detach "$UPSTREAM_COMMIT"

echo "[fetch] 完成:"
git -C "$DEST" log --oneline -1
