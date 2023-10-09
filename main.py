#!/usr/bin/env python3

from parsecli import *

from pathlib import Path

if __name__ == "__main__":
  scriptPath = Path(__file__).parent

  # Init
  print("Init...")
  cliArgs = parseCli()
  rgbRes, monoRes = parseCliRes(cliArgs.resolution)
  rgbEncoder, monoEncoder = parseCliEncoder(cliArgs.encoder)

  if cliArgs.preview:
    from previewer import *
    preview = Previewer(rgbRes, monoRes, cliArgs.fps)
    preview.preview()
  else:
    from recorder import *
    recorder = Recorder(rgbRes, monoRes, cliArgs.fps)
    recorder.record(
        Path(cliArgs.out), rgbEncoder, monoEncoder, cliArgs.quality,
        cliArgs.keyframe)
