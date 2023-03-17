import argparse


def parseCli():
  description = "Record RGB and stereo video with DepthAI OAK-D"
  parser = argparse.ArgumentParser(description=description)
  parser.add_argument(
      "-p",
      "--preview",
      action="store_const",
      const=True,
      default=False,
      help="Show preview, nothing will be recorded")
  parser.add_argument(
      "-c",
      "--codec",
      type=str,
      choices=["h265", "mjpeg", "lossless"],
      metavar="",
      default="lossless",
      help="Available codecs: [\"h265\", \"mjpeg\", \"lossless\"], default = \"lossless\""
  )
  parser.add_argument(
      "-f",
      "--fps",
      type=int,
      metavar="",
      default=30,
      help="Specify FPS, default = \"30\"")
  parser.add_argument(
      "-q",
      "--quality",
      type=int,
      metavar="",
      default=100,
      help="Encoding quality between 0-100, default = \"100\"")
  parser.add_argument(
      "-m",
      "--mp4",
      action="store_const",
      const=True,
      default=False,
      help="Convert the raw stream files to MP4 after recording (Require ffmpeg be installed)"
  )
  parser.add_argument(
      "-o",
      "--out",
      type=str,
      metavar="",
      default="output",
      help="Output directory path, default = \"output\"")
  parser.add_argument(
      "--rgbres",
      type=str,
      choices=["12mp", "4k", "1080p"],
      metavar="",
      default="1080p",
      help="RGB camera resolution: [\"12mp\", \"4k\", \"1080p\"], default = \"1080p\""
  )
  parser.add_argument(
      "--monores",
      type=str,
      choices=["800p", "720p", "400p"],
      metavar="",
      default="800p",
      help="Mono camera resolution: [\"800p\", \"720p\", \"400p\"], default = \"800p\""
  )
  args = parser.parse_args()
  return args
