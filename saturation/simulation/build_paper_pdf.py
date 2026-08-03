"""Build ``paper.pdf`` locally with Pandoc, KaTeX, and headless Chrome.

The build uses only ``file://`` resources and never starts an HTTP server.
Pandoc's ``tex_math_single_backslash`` extension is required because the
manuscript consistently uses ``\\(...\\)`` and ``\\[...\\]`` delimiters.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path


def _find_existing(candidates: list[Path], description: str) -> Path:
    for candidate in candidates:
        if candidate.exists():
            return candidate
    tried = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Could not find {description}. Tried:\n  {tried}")


def _wait_for_stable_file(path: Path, timeout_s: float = 90.0) -> None:
    deadline = time.monotonic() + timeout_s
    last_size = -1
    stable_checks = 0
    while time.monotonic() < deadline:
        size = path.stat().st_size if path.exists() else -1
        if size > 0 and size == last_size:
            stable_checks += 1
            if stable_checks >= 3:
                return
        else:
            stable_checks = 0
        last_size = size
        time.sleep(0.2)
    raise TimeoutError(f"Chrome did not finish writing {path} within {timeout_s:.0f} s")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("markdown", type=Path, nargs="?", default=Path("paper.md"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    source = args.markdown.resolve()
    output = (args.output or source.with_suffix(".pdf")).resolve()
    stylesheet = source.parent / "paper.css"
    pandoc = shutil.which("pandoc")
    if pandoc is None:
        raise FileNotFoundError("pandoc is required to build the paper PDF")

    chrome_candidates = [
        Path("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"),
        Path("/Applications/Chromium.app/Contents/MacOS/Chromium"),
    ]
    if os.environ.get("CHROME_PATH"):
        chrome_candidates.insert(0, Path(os.environ["CHROME_PATH"]))
    chrome = _find_existing(
        chrome_candidates,
        "Chrome or Chromium",
    )
    katex_candidates = [
        Path(
            "/Applications/Visual Studio Code.app/Contents/Resources/app/"
            "node_modules/katex/dist"
        ),
    ]
    if os.environ.get("KATEX_DIST"):
        katex_candidates.insert(0, Path(os.environ["KATEX_DIST"]))
    katex_dist = _find_existing(
        katex_candidates,
        "a KaTeX distribution directory",
    )

    with tempfile.TemporaryDirectory(prefix="phri_paper_build_") as temporary:
        temp_dir = Path(temporary)
        local_katex = temp_dir / "katex"
        shutil.copytree(katex_dist, local_katex)
        temporary_pdf = temp_dir / "paper.pdf"

        # Keep the HTML next to paper.md while rendering so relative figure
        # paths resolve without an HTTP server. It is removed in ``finally``.
        handle = tempfile.NamedTemporaryFile(
            prefix=".paper_build_",
            suffix=".html",
            dir=source.parent,
            delete=False,
        )
        handle.close()
        temporary_html = Path(handle.name)
        try:
            subprocess.run(
                [
                    pandoc,
                    str(source),
                    "-f",
                    "markdown+tex_math_single_backslash",
                    "--standalone",
                    f"--katex={local_katex.as_uri()}/",
                    f"--css={stylesheet}",
                    "--metadata",
                    f"pagetitle={source.stem}",
                    "-o",
                    str(temporary_html),
                ],
                cwd=source.parent,
                check=True,
            )

            process = subprocess.Popen(
                [
                    str(chrome),
                    "--headless",
                    "--disable-gpu",
                    "--disable-background-networking",
                    "--disable-component-update",
                    "--disable-default-apps",
                    "--no-first-run",
                    "--no-pdf-header-footer",
                    "--allow-file-access-from-files",
                    f"--user-data-dir={temp_dir / 'chrome-profile'}",
                    f"--print-to-pdf={temporary_pdf}",
                    temporary_html.as_uri(),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                _wait_for_stable_file(temporary_pdf)
            finally:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.0)

            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(temporary_pdf, output)
        finally:
            temporary_html.unlink(missing_ok=True)

    print(f"Saved {output}")


if __name__ == "__main__":
    main()
