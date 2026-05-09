from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def draw_dotted_rect(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int], step: int = 18, dot: int = 6) -> None:
    x0, y0, x1, y1 = box
    for x in range(x0, x1, step):
        draw.line((x, y0, min(x + dot, x1), y0), fill="black", width=2)
        draw.line((x, y1, min(x + dot, x1), y1), fill="black", width=2)
    for y in range(y0, y1, step):
        draw.line((x0, y, x0, min(y + dot, y1)), fill="black", width=2)
        draw.line((x1, y, x1, min(y + dot, y1)), fill="black", width=2)


def main() -> None:
    base_dir = Path(__file__).resolve().parent
    markers_dir = base_dir / "aruco_markers_6in"
    out_pdf = base_dir / "aruco_markers_6in_cut_guides.pdf"

    marker_files = sorted(markers_dir.glob("aruco_id_*_6in_300dpi.png"))
    if not marker_files:
        raise FileNotFoundError(f"No marker PNG files found in: {markers_dir}")

    # Letter page at 300 DPI.
    page_w, page_h = 2550, 3300
    pages: list[Image.Image] = []
    font = ImageFont.load_default()

    for marker_path in marker_files:
        marker = Image.open(marker_path).convert("RGB")
        page = Image.new("RGB", (page_w, page_h), "white")
        draw = ImageDraw.Draw(page)

        x = (page_w - marker.width) // 2
        y = (page_h - marker.height) // 2
        page.paste(marker, (x, y))

        # Dotted cut guideline exactly around the printed marker square.
        draw_dotted_rect(draw, (x, y, x + marker.width, y + marker.height))
        draw.text((x, y - 28), marker_path.stem, fill="black", font=font)

        pages.append(page)

    first, rest = pages[0], pages[1:]
    first.save(out_pdf, "PDF", resolution=300.0, save_all=True, append_images=rest)
    print(f"Generated PDF: {out_pdf}")


if __name__ == "__main__":
    main()
