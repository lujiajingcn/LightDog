#!/usr/bin/env bash
# 把上游 linorobot2 源码拉到 ./linorobot2/（该目录不入库，见 .gitignore）。
#
# 为什么需要它：仓库里只放「我们自己的东西」（lightdog 包 + sim 编排），上游源码按需拉取。
# 运行时并不需要上游源码 —— 仿真用的是已构建好的 linorobot2:gazebo 镜像。
# 拉下来主要是为了读代码 / 评估补丁 / 重建镜像。
#
# 用法：
#   scripts/fetch_upstream.sh          # 首次拉取，或校验已有目录
#   scripts/fetch_upstream.sh --force  # 删掉已有目录重拉
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/upstream.lock"

DEST="$ROOT/linorobot2"

# WorkBuddy / 沙箱环境自带的代理会吃掉 git 请求，必须清空走直连
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
    echo "[fetch] 版本不符（当前 ${HAVE:0:8}），重拉"
    rm -rf "$DEST"
fi

# 只取锁定的那一个提交，不拉全历史。GitHub 支持按 sha fetch。
fetch_commit() {
    local url="$1"
    echo "[fetch] $url"
    rm -rf "$DEST"
    git init -q "$DEST"
    git -C "$DEST" remote add origin "$url"
    if ! git -C "$DEST" fetch -q --depth 1 origin "$UPSTREAM_COMMIT"; then
        echo "[fetch] 按 sha 拉取失败，退化为拉分支最新再检出"
        git -C "$DEST" fetch -q --depth 50 origin "$UPSTREAM_BRANCH"
        git -C "$DEST" checkout -q FETCH_HEAD 2>/dev/null || true
        if ! git -C "$DEST" cat-file -e "$UPSTREAM_COMMIT^{commit}" 2>/dev/null; then
            echo "[fetch] FAIL: 拉不到 $UPSTREAM_COMMIT，锁文件与上游分支可能已脱节"
            return 1
        fi
        git -C "$DEST" checkout -q "$UPSTREAM_COMMIT"
        return 0
    fi
    git -C "$DEST" checkout -q FETCH_HEAD
}

if ! fetch_commit "$UPSTREAM_MIRROR"; then
    echo "[fetch] 镜像通道失败，尝试直连"
    fetch_commit "$UPSTREAM_URL"
fi

HEAD_SHA="$(git -C "$DEST" rev-parse HEAD)"
if [[ "$HEAD_SHA" != "$UPSTREAM_COMMIT" ]]; then
    echo "[fetch] FAIL: 检出到 $HEAD_SHA，与锁定的 $UPSTREAM_COMMIT 不符"
    exit 1
fi

echo "[fetch] 完成:"
git -C "$DEST" log --oneline -1
