"""
## exec
python bezier.py angle_inf angle_sup

## exec debug exporting curves as SVG
python -O bezier.py angle_inf angle_sup
"""
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import typing
import sys
import os

MAX_COOR = 200
NB_CURVES = 500

INF = 60
SUP = 90
if len(sys.argv) > 2:
    INF = int(sys.argv[1])
    SUP = int(sys.argv[2])
TITLE = str(INF) + "to" + str(SUP)


def get_angle(points) -> float:
    ba = points[0] - points[1]
    bc = points[2] - points[1]
    cosine_angle = (ba @ bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    return np.degrees(angle)


def create_curve(inf, sup) -> typing.Union[np.array, float]:
    points = [
        np.array([0, 0]),
        np.random.randint(1, MAX_COOR, size=2),  # control point
        np.array([np.random.randint(1, MAX_COOR), 0])
    ]

    angle = get_angle(points)

    while angle < inf or angle > sup:
        points[1] = np.random.randint(1, MAX_COOR, size=2)
        points[2][0] = np.random.randint(1, MAX_COOR)
        angle = get_angle(points)

    return points, angle


def draw(points, angle) -> None:
    Path = mpath.Path
    fig, ctx = plt.subplots()

    pp1 = mpatches.PathPatch(
        Path([points[0], points[1], points[2]],
             [Path.MOVETO, Path.CURVE3, Path.CURVE3]),
        fc="none", transform=ctx.transData, linestyle="solid", linewidth=4, alpha=.5, color="#32a852")

    # Define lines over the control point
    ctx.plot([points[0][0], points[1][0]], [points[0][1],
                                            points[1][1]], dashes=[6, 2], color="#427852")
    ctx.plot([points[2][0], points[1][0]], [points[2][1],
                                            points[1][1]], dashes=[6, 2], color="#427852")

    # points
    ctx.plot(points[2][0], points[2][1], marker=".",
             markersize=16, color="#427852", clip_on=False)
    ctx.plot(points[1][0], points[1][1], marker=".",
             markersize=16, color="#0096b0", clip_on=False)
    ctx.plot(points[0][0], points[0][1], marker=".",
             markersize=16, color="#427852", clip_on=False)

    # Curve
    ctx.add_patch(pp1)

    title = str(np.format_float_positional(angle, precision=2)) + \
        "° " + str(points[0]) + str(points[1]) + str(points[2])
    # Title
    ctx.set_title(title)

    plt.xlim(0, MAX_COOR)
    plt.ylim(0, MAX_COOR)

    os.system("mkdir -p " + TITLE + "view")
    plt.savefig(TITLE + "view/" + title + ".svg")
    plt.close()
    # plt.show()


def main() -> None:
    f = open(TITLE + ".txt", "a")
    for i in range(0, NB_CURVES):
        points, angle = create_curve(INF, SUP)

        if not __debug__:
            draw(points, angle)  # Creating view # optional

        for e in points:
            for ee in e:
                f.write(str(ee) + " ")
        f.write(str(angle) + "\n")
        print(str(i+1) + "/" + str(NB_CURVES))

    f.close()


if __name__ == '__main__':
    sys.exit(main())
