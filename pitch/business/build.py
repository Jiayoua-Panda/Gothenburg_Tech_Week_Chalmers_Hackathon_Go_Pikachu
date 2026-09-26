"""Build the Business-mode pages (big brain + cyber brain + team) for the pitch deck.

    python3 pitch/business/build.py           # writes pitch/business.html (standalone preview)
    python3 pitch/business/build.py --merge   # merges into pitch/index.html

Sources: scenes.html (one <section> per page, in order) and draw.js (canvas drawings).
index.html plays its <section class="scene"> elements as pages in HTML order, so merging means:
  - the team page (last section of scenes.html) is copied to the very start: page 1
  - all business pages are appended after the existing end page
--merge is idempotent: earlier merged blocks (between TEAM / BUSINESS markers) are removed first.
Why a script and not a hand edit: teammates keep editing index.html; re-running this after a pull
re-applies the business pages without touching their scenes.
"""
import re
import sys
from pathlib import Path

HERE = Path(__file__).parent
INDEX = HERE.parent / "index.html"
OUT = HERE.parent / "business.html"
MARK = {k: (f"<!-- {k}:BEGIN -->", f"<!-- {k}:END -->") for k in ("TEAM", "BUSINESS")}
D_BEGIN, D_END = "/* BUSINESS:BEGIN */", "/* BUSINESS:END */"
SECTION = re.compile(r'<section class="scene"[\s\S]*?</section>')
STAGE = re.compile(r'(<div id="stage">\n)([\s\S]*?)(\n</div>\n<div id="bar">)')


def strip_merged(html: str) -> str:
    for a, b in MARK.values():
        html = re.sub(r"\n?" + re.escape(a) + r"[\s\S]*?" + re.escape(b) + r"\n?", "\n", html)
    html = re.sub(re.escape(D_BEGIN) + r"[\s\S]*?" + re.escape(D_END) + r"\n?", "", html)
    return html.replace("Object.assign(DRAW, BUSINESS_DRAW);\n", "")


def add_draw(html: str, draw_js: str) -> str:
    m = re.search(r"^const DRAW = .*$", html, re.M)
    return (html[:m.start()] + f"{D_BEGIN}\n{draw_js}{D_END}\n" + m.group(0)
            + "\nObject.assign(DRAW, BUSINESS_DRAW);" + html[m.end():])


def main():
    index = strip_merged(INDEX.read_text())
    scenes = (HERE / "scenes.html").read_text()
    draw_js = (HERE / "draw.js").read_text()
    pages = SECTION.findall(scenes)
    team = scenes[scenes.rfind("<!--", 0, scenes.rfind(pages[-1])):].strip()  # team page with its comment
    # the opening copy introduces the team; only the closing one says thank you
    team = re.sub(r'data-notes="[^"]*"', 'data-notes="We are team Go Pikachu: Leif, Zhichao, Darin, Shuwen and Jianying."', team, count=1)

    stage = STAGE.search(index)
    if "--merge" not in sys.argv:
        html = index[:stage.start(2)] + scenes + index[stage.end(2):]
        html = re.sub(r"<title>.*?</title>", "<title>Business Mode</title>", html)
        OUT.write_text(add_draw(html, draw_js))
        print(f"wrote {OUT.name}: {len(pages)} pages")
        return

    body = stage.group(2).strip("\n")
    TA, TB = MARK["TEAM"]
    BA, BB = MARK["BUSINESS"]
    new_body = (f"{TA}\n{team}\n{TB}\n\n{body}\n\n"
                f"{BA}\n{scenes.strip()}\n{BB}\n")
    html = index[:stage.start(2)] + new_body + index[stage.end(2):]
    INDEX.write_text(add_draw(html, draw_js))
    total = len(SECTION.findall(html))
    print(f"merged: team page first, {len(pages)} business pages at the end; {total} pages in total")


if __name__ == "__main__":
    main()
