from datetime import timedelta
from pathlib import Path
import argparse
import re


class CliParser:
  outDir: Path
  preview: bool
  time: timedelta

  rgbRes: str
  monoRes: str
  fps: int

  rgbEncoder: str
  monoEncoder: str
  quality: int
  keyframe: int

  def __init__(self):
    description = "Record RGB and stereo video with timestamp using DepthAI OAK-D"
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
      "-o",
      "--out",
      metavar="",
      type=str,
      default="out",
      help='Output directory path, default = "out"',
    )
    parser.add_argument(
      "-p",
      "--preview",
      metavar="",
      action="store_const",
      const=True,
      default=False,
      help="Show preview, nothing will be recorded",
    )
    parser.add_argument(
      "-t",
      "--time",
      metavar="",
      type=str,
      default="0",
      help='Specify recording time, 0 means manually stop, default = "0"',
    )
    parser.add_argument(
      "-r",
      "--resolution",
      type=str,
      metavar="",
      default="1080p,800p",
      help='RGB (and Mono) resolution: <12mp|4k|1080p>[,<800p|720p|400p>], default = "1080p,800p"',
    )
    parser.add_argument(
      "-f",
      "--fps",
      metavar="",
      type=int,
      choices=range(1, 61),
      default=30,
      help='Specify FPS, default = "30"',
    )
    parser.add_argument(
      "-e",
      "--encoder",
      metavar="",
      type=str,
      default="h265",
      help='Encoder for RGB and Mono: <lossless|mjpeg|h265>[,<lossless|mjpeg|h265>], default = "h265" (Single Value mean same encoder)',
    )
    parser.add_argument(
      "-q",
      "--quality",
      metavar="",
      type=int,
      choices=range(0, 101),
      default=100,
      help='Encoder quality between 0-100, default = "100"',
    )
    parser.add_argument(
      "-k",
      "--keyframe",
      metavar="",
      type=int,
      default=0,
      help='Keyframe frequency, 0 means auto, default = "0"',
    )
    args = parser.parse_args()

    self.outDir = Path(args.out).expanduser()
    self.preview = args.preview
    self.time = self.__parseCliTime(args.time)

    self.rgbRes, self.monoRes = self.__parseCliRes(args.resolution)
    self.fps = args.fps

    self.rgbEncoder, self.monoEncoder = self.__parseCliEncoder(args.encoder)
    self.quality = args.quality
    self.keyframe = args.keyframe

  def __parseCliRes(self, cliRes: str) -> tuple[str, str]:
    rgbRes = ""
    monoRes = ""
    if "," in cliRes:
      rgbRes, monoRes = cliRes.split(",", 1)
    if rgbRes == "":
      rgbRes = "1080p"
    if monoRes == "":
      monoRes = "800p"

    assert rgbRes in ("12mp", "4k", "1080p")
    assert monoRes in ("800p", "720p", "400p")
    return rgbRes, monoRes

  def __parseCliEncoder(self, cliEncoder: str) -> tuple[str, str]:
    rgbEncoder = ""
    monoEncoder = ""
    if "," in cliEncoder:
      rgbEncoder, monoEncoder = cliEncoder.split(",", 1)
      if rgbEncoder == "":
        rgbEncoder = "lossless"
      if monoEncoder == "":
        monoEncoder = "lossless"
    else:
      rgbEncoder = cliEncoder
      monoEncoder = cliEncoder

    assert rgbEncoder in ("lossless", "mjpeg", "h265")
    assert monoEncoder in ("lossless", "mjpeg", "h265")
    return rgbEncoder, monoEncoder

  def __parseCliTime(self, cliTime: str) -> timedelta:
    if cliTime == "0":
      return timedelta.max

    d: float = 0.0
    h: float = 0.0
    min: float = 0.0
    s: float = 0.0
    if ":" in cliTime:
      n = cliTime.count(":")
      if n == 1:
        min, s = map(float, cliTime.split(":"))
      if n == 2:
        h, min, s = map(float, cliTime.split(":"))
    else:
      if "d" in cliTime:
        match = re.search(r"(\d+)d", cliTime).groups(1)
        if match:
          d = float(match[0])
      if "h" in cliTime:
        match = re.search(r"(\d+)h", cliTime).groups(1)
        if match:
          h = float(match[0])
      if "min" in cliTime:
        match = re.search(r"(\d+)min", cliTime).groups(1)
        if match:
          min = float(match[0])
      if "s" in cliTime:
        match = re.search(r"(\d+)s", cliTime).groups(1)
        if match:
          s = float(match[0])
    return timedelta(days=d, hours=h, minutes=min, seconds=s)
