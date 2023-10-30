#!/usr/bin/env python3

from cliparser import *

from pathlib import Path

if __name__ == "__main__":
  scriptPath = Path(__file__).parent

  # Init
  print("Init...")
  cliParser = cliParser()

  if cliParser.preview:
    from previewer import *
    preview = Previewer(cliParser.time, cliParser.rgbRes, cliParser.monoRes,
                        cliParser.fps)
    preview.preview()
  else:
    from recorder import *
    recorder = Recorder(cliParser.time, cliParser.rgbRes, cliParser.monoRes,
                        cliParser.fps)
    recorder.record(cliParser.outDir, cliParser.rgbEncoder,
                    cliParser.monoEncoder, cliParser.quality,
                    cliParser.keyframe)
