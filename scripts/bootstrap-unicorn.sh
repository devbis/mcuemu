#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
clone_dir="$repo_root/engine/unicorn"
# HTTPS so anyone can clone without keys. Maintainers push over SSH with
#   git config --global url."git@github.com:".pushInsteadOf "https://github.com/"
fork_url="https://github.com/devbis/unicorn.git"
upstream_url="https://github.com/unicorn-engine/unicorn.git"

if [[ ! -d "$clone_dir/.git" ]]; then
    mkdir -p "$(dirname "$clone_dir")"
    git clone "$fork_url" "$clone_dir"
fi

# Leave an existing origin alone (a maintainer may use an SSH URL there).
if ! git -C "$clone_dir" remote get-url origin >/dev/null 2>&1; then
    git -C "$clone_dir" remote add origin "$fork_url"
fi
if git -C "$clone_dir" remote get-url upstream >/dev/null 2>&1; then
    git -C "$clone_dir" remote set-url upstream "$upstream_url"
else
    git -C "$clone_dir" remote add upstream "$upstream_url"
fi

echo "Unicorn remotes:"
git -C "$clone_dir" remote -v
