#!/usr/bin/env python3
"""Combine an archive's analysis docs into a single printable PDF.

Renders every ```mermaid block to a PNG (mmdc), rewrites image paths to
absolute, concatenates the docs in reading order with page breaks, and runs
pandoc (wkhtmltopdf engine) to produce <archive>/<name>-analysis.pdf.

Requires: pandoc, wkhtmltopdf, mmdc (mermaid-cli) on PATH.

Usage (from repo root):
    python3 docs/log-analysis/build_pdf.py docs/log-analysis/20260617-124210
"""
import os
import re
import sys
import glob
import subprocess
import tempfile

# Reading order of the combined document.
ORDER = [
    ("README.md", "Overview & provenance"),
    ("session-analysis.md", "Session analysis"),
    ("control-loop-analysis.md", "Control-loop analysis"),
    ("motor-analysis.md", "Motor analysis"),
    ("kernel-analysis.md", "Kernel / RTOS analysis"),
    ("sensor-analysis.md", "Sensor & fusion analysis"),
]

CSS = """
body { font-family: 'DejaVu Sans', sans-serif; font-size: 10.5pt; line-height: 1.4;
       color: #111; }
h1 { font-size: 19pt; border-bottom: 2px solid #333; padding-bottom: 3px; }
h2 { font-size: 14pt; border-bottom: 1px solid #bbb; padding-bottom: 2px; margin-top: 18px; }
h3 { font-size: 12pt; }
code, pre { font-family: 'DejaVu Sans Mono', monospace; font-size: 8.8pt; }
pre { background: #f5f5f5; padding: 8px; border-radius: 4px; white-space: pre-wrap;
      border: 1px solid #e0e0e0; }
table { border-collapse: collapse; font-size: 9.2pt; margin: 8px 0; }
th, td { border: 1px solid #bbb; padding: 3px 7px; }
th { background: #eee; }
img { max-width: 100%; height: auto; display: block; margin: 8px auto; }
blockquote { border-left: 3px solid #ccc; margin-left: 0; padding-left: 10px; color: #444; }
a { color: #00408a; text-decoration: none; }
.pagebreak { page-break-before: always; }
"""


def render_mermaid(block, outpng):
    with tempfile.NamedTemporaryFile("w", suffix=".mmd", delete=False) as f:
        f.write(block)
        src = f.name
    subprocess.run(["mmdc", "-i", src, "-o", outpng, "-b", "white", "-w", "1500"],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    os.unlink(src)


def process(archive):
    archive = os.path.abspath(archive)
    name = os.path.basename(archive)
    plots = os.path.join(archive, "plots")
    os.makedirs(plots, exist_ok=True)

    parts = [f"% Vayu flight-log analysis — {name}\n"]
    mmd_i = 0
    for fname, title in ORDER:
        path = os.path.join(archive, fname)
        if not os.path.exists(path):
            continue
        text = open(path).read()

        # render mermaid blocks -> png, replace fence with image
        def repl(m):
            nonlocal mmd_i
            mmd_i += 1
            png = os.path.join(plots, f"diagram_{mmd_i:02d}.png")
            render_mermaid(m.group(1), png)
            return f"\n![]({png})\n"
        text = re.sub(r"```mermaid\n(.*?)```", repl, text, flags=re.S)

        # absolutize remaining relative image paths (plots/...)
        text = re.sub(r"!\[([^\]]*)\]\((plots/[^)]+)\)",
                      lambda m: f"![{m.group(1)}]({os.path.join(archive, m.group(2))})", text)

        parts.append('\n\n<div class="pagebreak"></div>\n\n')
        parts.append(text)

    combined = "\n".join(parts)
    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False, dir=archive) as f:
        f.write(combined)
        md = f.name
    css = os.path.join(archive, ".pdf.css")
    open(css, "w").write(CSS)
    out = os.path.join(archive, f"{name}-analysis.pdf")

    cmd = ["pandoc", md, "-o", out, "-f", "gfm", "--pdf-engine=wkhtmltopdf",
           "--css", css, "--metadata", "pagetitle=Vayu log analysis",
           "--pdf-engine-opt=--enable-local-file-access",
           "--pdf-engine-opt=--page-size", "--pdf-engine-opt=A4",
           "--pdf-engine-opt=--margin-top", "--pdf-engine-opt=14mm",
           "--pdf-engine-opt=--margin-bottom", "--pdf-engine-opt=14mm",
           "--pdf-engine-opt=--margin-left", "--pdf-engine-opt=13mm",
           "--pdf-engine-opt=--margin-right", "--pdf-engine-opt=13mm"]
    subprocess.run(cmd, check=True)
    os.unlink(md); os.unlink(css)
    sz = os.path.getsize(out)
    print(f"wrote {out} ({sz/1024:.0f} kB), {mmd_i} diagrams rendered")


if __name__ == "__main__":
    arc = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "20260617-124210")
    process(arc)
