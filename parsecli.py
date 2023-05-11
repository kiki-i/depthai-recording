import argparse


def parseCli():
  description = "Record RGB and stereo video with DepthAI OAK-D"
  parser = argparse.ArgumentParser(description=description)
  parser.add_argument(
      "-r",
      "--resolution",
      type=str,
      metavar="",
      default="1080p,800p",
      help="RGB (and Mono) resolution: <12mp|4k|1080p>[,<800p|720p|400p>], default = \"1080p,800p\""
  )
  parser.add_argument(
      "-f",
      "--fps",
      metavar="",
      type=int,
      choices=range(1, 61),
      default=30,
      help="Specify FPS, default = \"30\"")
  parser.add_argument(
      "-p",
      "--preview",
      metavar="",
      action="store_const",
      const=True,
      default=False,
      help="Show preview, nothing will be recorded")
  parser.add_argument(
      "-e",
      "--encoder",
      metavar="",
      type=str,
      default="lossless",
      help="Encoder for RGB and Mono: <lossless|mjpeg|h265>[,<lossless|mjpeg|h265>], default = \"lossless\" (Single Value mean both have the same encoder)"
  )
  parser.add_argument(
      "-q",
      "--quality",
      metavar="",
      type=int,
      choices=range(0, 101),
      default=100,
      help="Encoder quality between 0-100, default = \"100\"")
  parser.add_argument(
      "-k",
      "--keyframe",
      metavar="",
      type=int,
      default=0,
      help="Keyframe frequency, 0 means auto, default = \"0\"")
  parser.add_argument(
      "-o",
      "--out",
      metavar="",
      type=str,
      default="out",
      help="Output directory path, default = \"out\"")
  args = parser.parse_args()
  return args


def parseCliRes(cliRes: str) -> tuple[str, str]:
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


def parseCliEncoder(cliEncoder: str) -> tuple[str, str]:
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
