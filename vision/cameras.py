"""Camera intrinsics, in ONE place.

Until now `IMG_W`/`IMG_H`/`FOV` were module globals in `score_occupancy.py`
describing the Webots Mavic2Pro proto camera, and `project()` closed over them.
That was fine while the only frames in the project came out of the simulator.

Real DJI stills do not have those intrinsics -- they are 3840x2160 at 73.7 deg
horizontal against the proto's 400x240 at 45 deg -- so projecting a bay onto a
real frame with the sim's numbers is off by a factor of ~10 in scale. The real
values already existed, but only in prose (CLAUDE.md, LABELING.md,
docs/training.md), which is exactly the shape of the bug TODO #3 was written to
retire: a constant living in two places with nothing checking they agree.

So the camera becomes a value that gets passed, and `SIM_MAVIC` is built FROM
the existing globals rather than beside them -- there is still one definition of
the sim camera, and every caller that does not pass `cam=` behaves identically
to before, byte for byte.

Square pixels are assumed throughout (the vertical FOV is derived from the
horizontal one and the aspect ratio, which is what `project()` has always done).
Sanity: DJI_NADIR's derived vertical FOV is 45.7 deg against the 45.4 deg on
record -- the two agree to within the precision the FOV figure was quoted at.
"""
import math


class Camera:
    """Pinhole intrinsics: pixel dimensions plus a horizontal field of view."""

    __slots__ = ("w", "h", "fov_h", "name")

    def __init__(self, w, h, fov_h, name=""):
        self.w, self.h, self.fov_h, self.name = int(w), int(h), float(fov_h), name

    @property
    def f(self):
        """Focal length in pixels. The one number project()/unproject() need."""
        return self.w / (2.0 * math.tan(self.fov_h / 2.0))

    @property
    def fov_v(self):
        """Derived vertical FOV, radians. Square pixels."""
        return 2.0 * math.atan(self.h / (2.0 * self.f))

    def footprint(self, alt):
        """Ground footprint (across, along) in metres for a NADIR camera."""
        return (2.0 * alt * math.tan(self.fov_h / 2.0),
                2.0 * alt * math.tan(self.fov_v / 2.0))

    def gsd(self, alt):
        """Ground sample distance, metres per pixel at nadir."""
        return alt / self.f

    def __repr__(self):
        return (f"Camera({self.name or 'unnamed'}: {self.w}x{self.h}, "
                f"{math.degrees(self.fov_h):.1f} deg H)")


# The Webots Mavic2Pro proto camera. Values are NOT written here -- they are
# imported from score_occupancy so that file stays the single source and the
# existing tools/check_consistency.py camera check keeps covering them.
def _sim_camera():
    import score_occupancy as so
    return Camera(so.IMG_W, so.IMG_H, so.FOV, "sim/mavic2pro")


# The real aircraft, flight 0034-0035 (2026-08-21, Sofia, FMI block).
# 73.7 deg H is measured from the footage: at the SRT's 30.1 m the frame spans
# 45.0 m across, i.e. GSD 11.7 mm/px and a 4.5 m car ~385 px.
DJI_NADIR = Camera(3840, 2160, math.radians(73.7), "dji/nadir")

CAMERAS = {"dji": DJI_NADIR}


def get(name):
    """Look a camera up by short name; 'sim' is resolved lazily."""
    if name == "sim":
        return _sim_camera()
    if name not in CAMERAS:
        raise KeyError(f"unknown camera {name!r}; have: sim, "
                       + ", ".join(sorted(CAMERAS)))
    return CAMERAS[name]


if __name__ == "__main__":
    import sys
    sys.stdout.reconfigure(encoding="utf-8")
    for c in (_sim_camera(), DJI_NADIR):
        for alt in (30.0, 30.1):
            fw, fh = c.footprint(alt)
            print(f"{c!r}\n  f={c.f:8.2f} px   fov_v={math.degrees(c.fov_v):5.2f} deg"
                  f"   @{alt:5.1f} m: footprint {fw:6.2f} x {fh:6.2f} m, "
                  f"GSD {c.gsd(alt) * 1000:6.2f} mm/px")
