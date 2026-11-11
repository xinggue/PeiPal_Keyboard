#!/usr/bin/env python3
"""Sync release assets to a Gitee release.

Intended to be run LOCALLY (China -> Gitee is fast). Uploading from GitHub
Actions runners is not viable (~13 KB/s), so keep the binary mirroring local.

Usage:
    GITEE_TOKEN=xxx RELEASE_TAG=v0.1.0 ASSET_DIR=release-assets \
        python3 sync_gitee.py

Environment variables:
    GITEE_TOKEN   Gitee personal access token (scope: projects)  [required]
    RELEASE_TAG   Release tag name, e.g. v0.1.0                  [required]
    ASSET_DIR     Directory containing the files to upload (default: release-assets)
    GITEE_OWNER   Gitee user or org (default: xinggue)
    GITEE_REPO    Gitee repo name (default: PeiPal_Keyboard)
    GITEE_TARGET  target_commitish for release creation (default: main)
    GITEE_PURGE   Comma-separated extra asset names to delete before upload
"""

import glob
import json
import os
import subprocess
import urllib.error
import urllib.parse
import urllib.request

API = "https://gitee.com/api/v5"
UA = {"User-Agent": "Mozilla/5.0"}


def request(method, url, data=None, headers=None):
    h = dict(UA)
    if headers:
        h.update(headers)
    req = urllib.request.Request(url, data=data, headers=h, method=method)
    try:
        with urllib.request.urlopen(req) as resp:
            return resp.status, resp.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode("utf-8", "replace")
    except Exception as e:  # noqa: BLE001
        return -1, str(e)


def list_assets(owner, repo, release_id, token):
    status, body = request(
        "GET",
        f"{API}/repos/{owner}/{repo}/releases/{release_id}/attach_files"
        f"?access_token={urllib.parse.quote(token)}",
    )
    if status != 200:
        return []
    try:
        data = json.loads(body)
    except Exception:  # noqa: BLE001
        return []
    return data if isinstance(data, list) else []


def delete_asset(owner, repo, release_id, attach_id, token):
    request(
        "DELETE",
        f"{API}/repos/{owner}/{repo}/releases/{release_id}/attach_files/{attach_id}"
        f"?access_token={urllib.parse.quote(token)}",
    )


def main():
    token = os.environ.get("GITEE_TOKEN", "").strip()
    tag = os.environ.get("RELEASE_TAG", "").strip()
    asset_dir = os.environ.get("ASSET_DIR", "release-assets").strip()
    owner = os.environ.get("GITEE_OWNER", "xinggue").strip()
    repo = os.environ.get("GITEE_REPO", "PeiPal_Keyboard").strip()
    target = os.environ.get("GITEE_TARGET", "main").strip()
    purge = {
        n.strip()
        for n in os.environ.get("GITEE_PURGE", "").split(",")
        if n.strip()
    }

    if not token:
        print("ERROR: GITEE_TOKEN is required.")
        return
    if not tag:
        print("ERROR: RELEASE_TAG is required.")
        return

    print(f"Syncing release {tag} to Gitee ({owner}/{repo}) from '{asset_dir}'...")

    release_id = None
    status, body = request(
        "GET",
        f"{API}/repos/{owner}/{repo}/releases/tags/{tag}"
        f"?access_token={urllib.parse.quote(token)}",
    )
    print(f"GET release by tag -> HTTP {status}")
    if status == 200:
        try:
            data = json.loads(body)
        except Exception:  # noqa: BLE001
            data = None
        if isinstance(data, dict):
            release_id = data.get("id")
    if release_id:
        print(f"Found existing Gitee release ID: {release_id}")

    if not release_id:
        payload = urllib.parse.urlencode(
            {
                "access_token": token,
                "tag_name": tag,
                "name": f"Release {tag}",
                "target_commitish": target,
                "prerelease": "false",
                "body": (
                    f"PeiPal Keyboard (小培同学固件) {tag}。\n\n"
                    "包含 ESP32-C3 与 ESP32-S3 两套固件编译产物（zip）。\n"
                    "国内镜像高速下载。"
                ),
            }
        ).encode("utf-8")
        status, body = request(
            "POST",
            f"{API}/repos/{owner}/{repo}/releases",
            data=payload,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        )
        print(f"POST create release -> HTTP {status}")
        print(f"Response: {body[:800]}")
        if status in (200, 201):
            try:
                data = json.loads(body)
            except Exception:  # noqa: BLE001
                data = None
            if isinstance(data, dict):
                release_id = data.get("id")

    if not release_id:
        print("ERROR: Unable to obtain a Gitee release ID, aborting.")
        return

    print(f"Using Gitee release ID: {release_id}")

    files = sorted(glob.glob(os.path.join(asset_dir, "*")))
    files = [f for f in files if os.path.isfile(f)]
    local_names = {os.path.basename(f) for f in files}

    # Remove same-named assets (idempotent re-upload) and any purged extras.
    existing = list_assets(owner, repo, release_id, token)
    for a in existing:
        name = a.get("name")
        if name in local_names or name in purge:
            print(f"Deleting existing asset '{name}' (id={a.get('id')})...")
            delete_asset(owner, repo, release_id, a.get("id"), token)

    print(f"Uploading {len(files)} files to Gitee release {release_id}...")
    for f in files:
        fname = os.path.basename(f)
        size_mb = os.path.getsize(f) / 1048576
        print(f"--> Uploading {fname} ({size_mb:.1f} MB) ...")
        cmd = [
            "curl", "-sS", "--retry", "5", "--retry-delay", "5",
            "--connect-timeout", "30", "-o", "/dev/null",
            "-w", "http=%{http_code} time=%{time_total}s speed=%{speed_upload}B/s",
            "-X", "POST",
            "-F", f"access_token={token}",
            "-F", f"file=@{f}",
            f"{API}/repos/{owner}/{repo}/releases/{release_id}/attach_files",
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        print(f"    {res.stdout.strip() or res.stderr.strip()}")

    print("Gitee release synchronization completed!")


if __name__ == "__main__":
    main()
