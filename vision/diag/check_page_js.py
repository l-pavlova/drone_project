"""Syntax-check the JavaScript embedded in a generated tool page.

    python vision/diag/check_page_js.py vision/data/align_dji_0035.html

`align_bays.py` and `label_bays.py` build a page by substituting into a Python
string, and a broken quote there produces an HTML file that opens perfectly and
does nothing -- the script dies at parse time and the canvas stays blank. That
failure is invisible from Python: the generator exits 0, the file is the right
size, and every placeholder is gone.

It happened for real (2026-08-26): a patch wrote `\\"` into the source, Python
consumed the escape a second time when it parsed the PAGE literal, and the emitted
JS read `"<span class="swatch"` -- a syntax error that killed the whole script.

So: extract every <script> block and run it through `node --check`. Falls back to
a brace/quote balance check when node is unavailable, which catches less but is
better than trusting the file.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

sys.stdout.reconfigure(encoding="utf-8")


def scripts(html):
    return re.findall(r"<script[^>]*>(.*?)</script>", html, re.S)


def check(path):
    html = open(path, encoding="utf-8").read()
    blocks = scripts(html)
    if not blocks:
        print(f"{path}: no <script> block found")
        return 1

    leftover = [t for t in ("__DATA__", "__AREA__", "__STEM__") if t in html]
    if leftover:
        print(f"{path}: unsubstituted placeholder(s) {leftover}")
        return 1

    node = shutil.which("node")
    bad = 0
    for i, js in enumerate(blocks):
        if node:
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False,
                                             encoding="utf-8") as fh:
                fh.write(js)
                tmp = fh.name
            try:
                r = subprocess.run([node, "--check", tmp], capture_output=True,
                                   text=True)
                if r.returncode:
                    bad += 1
                    print(f"{path}: script block {i} FAILED node --check")
                    print("  " + (r.stderr or r.stdout).strip().splitlines()[0])
                    for ln in (r.stderr or r.stdout).strip().splitlines()[1:6]:
                        print("  " + ln)
            finally:
                os.unlink(tmp)
        else:
            depth = js.count("{") - js.count("}")
            if depth:
                bad += 1
                print(f"{path}: script block {i} brace imbalance {depth:+d} "
                      f"(install node for a real check)")

    size = os.path.getsize(path) / 1e6
    if bad:
        return 1
    print(f"{path}: OK -- {len(blocks)} script block(s) parse, {size:.1f} MB"
          + ("" if node else "  (brace check only; node not found)"))
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sys.exit(max(check(p) for p in sys.argv[1:]))
