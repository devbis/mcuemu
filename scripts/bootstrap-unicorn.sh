#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
clone_dir="$repo_root/engine/unicorn"
upstream_url="https://github.com/unicorn-engine/unicorn.git"
fork_url="${TC32_UNICORN_FORK_URL:-}"

if [[ ! -d "$clone_dir/.git" ]]; then
    mkdir -p "$(dirname "$clone_dir")"
    git clone "${fork_url:-$upstream_url}" "$clone_dir"
fi

if [[ -n "$fork_url" ]]; then
    if git -C "$clone_dir" remote get-url origin >/dev/null 2>&1; then
        git -C "$clone_dir" remote set-url origin "$fork_url"
    else
        git -C "$clone_dir" remote add origin "$fork_url"
    fi
    if git -C "$clone_dir" remote get-url upstream >/dev/null 2>&1; then
        git -C "$clone_dir" remote set-url upstream "$upstream_url"
    else
        git -C "$clone_dir" remote add upstream "$upstream_url"
    fi
else
    if git -C "$clone_dir" remote get-url upstream >/dev/null 2>&1; then
        git -C "$clone_dir" remote set-url upstream "$upstream_url"
    elif git -C "$clone_dir" remote get-url origin >/dev/null 2>&1; then
        origin_url="$(git -C "$clone_dir" remote get-url origin)"
        if [[ "$origin_url" == "$upstream_url" ]]; then
            git -C "$clone_dir" remote rename origin upstream
        else
            git -C "$clone_dir" remote add upstream "$upstream_url"
        fi
    else
        git -C "$clone_dir" remote add upstream "$upstream_url"
    fi
fi

echo "Unicorn remotes:"
git -C "$clone_dir" remote -v
