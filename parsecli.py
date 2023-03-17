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
      type=int,
      metavar="",
      default=30,
      help="Specify FPS, default = \"30\"")
  parser.add_argument(
      "-p",
      "--preview",
      action="store_const",
      const=True,
      default=False,
      help="Show preview, nothing will be recorded")
  parser.add_argument(
      "-e",
      "--encoder",
      type=str,
      metavar="",
      default="lossless",
      help="Encoder for RGB and Mono: <lossless|mjpeg|h265>[,<lossless|mjpeg|h265>], default = \"lossless\""
  )
  parser.add_argument(
      "-q",
      "--quality",
      type=int,
      metavar="",
      default=100,
      help="Encoding quality between 0-100, default = \"100\"")
  parser.add_argument(
      "-k",
      "--keyframe",
      type=int,
      metavar="",
      default=0,
      help="Keyframe frequency, 0 means auto, default = \"0\"")
  parser.add_argument(
      "-o",
      "--out",
      type=str,
      metavar="",
      default="output",
      help="Output directory path, default = \"output\"")
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
  return rgbRes, monoRes


def parseCliEncoder(cliEncoder: str) -> tuple[str, str]:
  rgbEncoder = ""
  monoEncoder = ""
  if "," in cliEncoder:
    rgbEncoder, monoEncoder = cliEncoder.split(",", 1)
    if rgbEncoder == "":
      rgbEncoder = monoEncoder
    if monoEncoder == "":
      monoEncoder = rgbEncoder
  else:
    rgbEncoder = cliEncoder
    monoEncoder = cliEncoder
  return rgbEncoder, monoEncoder
