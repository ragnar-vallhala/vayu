#!/usr/bin/env python3
"""Combine an archive's analysis docs into a single printable PDF.

Renders every ```mermaid block to a PNG (mmdc), rewrites image paths to
absolute, concatenates the docs in reading order with page breaks, and runs
pandoc (xelatex engine) to produce <archive>/<name>-analysis.pdf.

xelatex is used (over wkhtmltopdf) because wkhtmltopdf drops spaces before
bold/italic at line-wrap boundaries; xelatex + DejaVu fonts also covers the
unicode (− → ≈ ° µ ─) used throughout.

Requires: pandoc, xelatex (TeX Live), mmdc (mermaid-cli) on PATH.

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
    ("estimator-analysis.md", "Estimator analysis (estimate vs ground truth)"),
    ("control-loop-analysis.md", "Control-loop analysis"),
    ("motor-analysis.md", "Motor analysis"),
    ("kernel-analysis.md", "Kernel / RTOS analysis"),
    ("sensor-analysis.md", "Sensor & fusion analysis"),
    ("vertical-analysis.md", "Vertical-channel analysis"),
    ("reference-autopilots-comparison.md", "Reference autopilots — control-loop comparison, per-problem solutions & resource analysis"),
    ("recommendations.md", "Recommendations & next-run plan"),
]

# LaTeX preamble: wrap long code lines (fvextra) so the ASCII diagrams don't
# overflow the margin, and keep headings tidy.
HEADER = r"""
\usepackage{fvextra}
\DefineVerbatimEnvironment{Highlighting}{Verbatim}{breaklines,breakanywhere,commandchars=\\\{\}}
\usepackage{sectsty}
\sectionfont{\large}
\subsectionfont{\normalsize}
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

    parts = []
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

        # raw HTML is dropped by pandoc for LaTeX; keep the <sub> footnote text
        text = text.replace("<sub>", "").replace("</sub>", "")

        # DejaVu (the PDF font) has no colour-emoji glyphs; strip the decorative
        # status emoji (the bold text label beside them carries the meaning),
        # their U+FE0F variation selectors, and any stray ones, so they don't
        # print as blanks.
        text = re.sub("[✅⚠↪✔✗]️?\\s*", "", text)
        text = text.replace("️", "")

        parts.append("\n\n\\newpage\n\n")
        parts.append(text)

    combined = "\n".join(parts)
    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False, dir=archive) as f:
        f.write(combined)
        md = f.name
    hdr = os.path.join(archive, ".pdf-header.tex")
    open(hdr, "w").write(HEADER)
    out = os.path.join(archive, f"{name}-analysis.pdf")

    cmd = ["pandoc", md, "-o", out,
           "-f", "markdown-tex_math_dollars", "--pdf-engine=xelatex",
           "-H", hdr, "--toc", "--toc-depth=2",
           "-V", "geometry:margin=1.7cm", "-V", "fontsize=10pt",
           "-V", "mainfont=DejaVu Sans", "-V", "monofont=DejaVu Sans Mono",
           "-V", "monofontoptions=Scale=0.80",
           "-V", "colorlinks=true", "-V", "linkcolor=[RGB]{0,64,138}",
           "-V", "urlcolor=[RGB]{0,64,138}", "-V", "toccolor=black",
           "-M", f"title=Vayu flight-log analysis — {name}",
           "-M", "date=decoded via NavLink v2"]
    subprocess.run(cmd, check=True)
    os.unlink(md); os.unlink(hdr)
    sz = os.path.getsize(out)
    print(f"wrote {out} ({sz/1024:.0f} kB), {mmd_i} diagrams rendered")


if __name__ == "__main__":
    arc = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "20260617-124210")
    process(arc)
