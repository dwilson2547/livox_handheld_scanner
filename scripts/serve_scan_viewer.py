#!/usr/bin/env python3
from __future__ import annotations

import argparse
import http.server
import json
import os
import shutil
import socketserver
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER_ROOT = REPO_ROOT / "viewer" / "scan_viewer"
RUNTIME_ROOT = VIEWER_ROOT / "runtime"
STAGED_SCAN = RUNTIME_ROOT / "current_scan.ply"
STAGED_META = RUNTIME_ROOT / "current_scan.json"


def _default_scan_candidates() -> list[Path]:
    candidates = []
    for base in [Path.cwd(), REPO_ROOT, *REPO_ROOT.parents]:
        candidate = (base / "sessions" / "mesh_live.ply").resolve()
        if candidate not in candidates:
            candidates.append(candidate)
    return candidates


def _resolve_scan_path(scan_arg: str | None) -> Path:
    if scan_arg:
        candidate = Path(scan_arg).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"scan artifact not found: {candidate}")
        return candidate

    for candidate in _default_scan_candidates():
        if candidate.is_file():
            return candidate

    searched = "\n".join(str(path) for path in _default_scan_candidates())
    raise FileNotFoundError(
        "scan artifact not found. Looked for sessions/mesh_live.ply in:\n" + searched
    )


def _stage_scan(scan_path: Path) -> None:
    RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)
    if STAGED_SCAN.exists() or STAGED_SCAN.is_symlink():
        STAGED_SCAN.unlink()

    try:
        STAGED_SCAN.symlink_to(scan_path)
    except OSError:
        shutil.copy2(scan_path, STAGED_SCAN)

    with STAGED_META.open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "source": str(scan_path),
                "size_bytes": scan_path.stat().st_size,
                "asset": STAGED_SCAN.name,
            },
            handle,
            indent=2,
        )


def _serve(bind: str, port: int) -> None:
    os.chdir(VIEWER_ROOT)
    handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer((bind, port), handler) as server:
        print(f"Serving scan viewer at http://{bind}:{port}")
        server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Serve the local scan viewer for a saved mesh PLY."
    )
    parser.add_argument(
        "--scan",
        help="Path to a .ply mesh to stage into the viewer. Defaults to the nearest ancestor sessions/mesh_live.ply.",
    )
    parser.add_argument(
        "--bind",
        default="127.0.0.1",
        help="Address to bind the viewer HTTP server to.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8081,
        help="Port to bind the viewer HTTP server to.",
    )
    args = parser.parse_args()

    scan_path = _resolve_scan_path(args.scan)
    _stage_scan(scan_path)
    _serve(args.bind, args.port)


if __name__ == "__main__":
    main()
