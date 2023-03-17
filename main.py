#!/usr/bin/env python3

from parsecli import *
from recorder import *

from pathlib import Path

if __name__ == "__main__":
  scriptPath = Path(__file__).parent

  # Init
  print("Init...")
  cliArgs = parseCli()
  rgbRes, monoRes = parseCliRes(cliArgs.resolution)
  rgbEncoder, monoEncoder = parseCliEncoder(cliArgs.encoder)
  recorder = Recorder(rgbRes, monoRes, cliArgs.fps)

  if cliArgs.preview:
    recorder.preview()
  else:
    recorder.record(
        Path(cliArgs.out), rgbEncoder, monoEncoder, cliArgs.quality,
        cliArgs.keyframe)
