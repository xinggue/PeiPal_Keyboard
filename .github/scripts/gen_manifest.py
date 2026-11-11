#!/usr/bin/env python3
"""Generate release-manifest.json for PeiPal Keyboard firmware releases.

Website can always fetch the latest manifest from:
    https://github.com/xinggue/PeiPal_Keyboard/releases/latest/download/release-manifest.json

Environment variables:
    ASSET_DIR      Directory with release assets (default: release-assets)
    RELEASE_TAG    Tag name, e.g. v1.0.0 (default: $GITHUB_REF_NAME)
    PUBLISHED_AT   ISO timestamp (default: current commit time, UTC)
    GITHUB_REPO    owner/repo (default: xinggue/PeiPal_Keyboard)
    GITEE_REPO     owner/repo (default: xinggue/PeiPal_Keyboard)
"""

import glob
import json
import os
import subprocess
from datetime import datetime, timezone

# Match by substring (case-insensitive) in the asset file name.
PLATFORM_RULES = (
    ("esp32c3", "esp32c3"),
    ("esp32s3", "esp32s3"),
)


def classify(name):
    lower = name.lower()
    for needle, key in PLATFORM_RULES:
        if needle in lower:
            return key
    return None


def commit_utc_timestamp():
    """Deterministic fallback: the current commit's committer time (UTC)."""
    try:
        out = subprocess.run(
            ["git", "show", "-s", "--format=%ct"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        epoch = int(out.stdout.strip())
        return datetime.fromtimestamp(epoch, timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"
        )
    except Exception:  # noqa: BLE001
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def main():
    asset_dir = os.environ.get("ASSET_DIR", "release-assets").strip()
    tag = os.environ.get("RELEASE_TAG", "").strip() or os.environ.get(
        "GITHUB_REF_NAME", ""
    ).strip()
    published_at = (
        os.environ.get("PUBLISHED_AT", "").strip() or commit_utc_timestamp()
    )
    gh_repo = os.environ.get("GITHUB_REPO", "xinggue/PeiPal_Keyboard").strip()
    gitee_repo = os.environ.get("GITEE_REPO", "xinggue/PeiPal_Keyboard").strip()

    platforms = {}
    for path in sorted(glob.glob(os.path.join(asset_dir, "*"))):
        if not os.path.isfile(path):
            continue
        name = os.path.basename(path)
        if name == "release-manifest.json":
            continue
        key = classify(name)
        if not key:
            continue
        platforms[key] = {
            "name": name,
            "size": os.path.getsize(path),
            "github": f"https://github.com/{gh_repo}/releases/download/{tag}/{name}",
            "gitee": f"https://gitee.com/{gitee_repo}/releases/download/{tag}/{name}",
        }

    manifest = {
        "version": tag,
        "published_at": published_at,
        "source": {"github": gh_repo, "gitee": gitee_repo},
        "links": {
            "github_release": f"https://github.com/{gh_repo}/releases/tag/{tag}",
            "gitee_release": f"https://gitee.com/{gitee_repo}/releases/tag/{tag}",
        },
        "platforms": platforms,
    }

    out = os.path.join(asset_dir, "release-manifest.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"Wrote {out}")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
