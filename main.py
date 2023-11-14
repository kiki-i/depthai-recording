#!/usr/bin/env python3

from cliparser import CliParser

from pathlib import Path

if __name__ == "__main__":
  scriptPath = Path(__file__).parent

  # Init
  print("Init...")
  cliParser = CliParser()

  if cliParser.preview:
    from previewer import Previewer

    preview = Previewer(
      cliParser.time, cliParser.rgbRes, cliParser.monoRes, cliParser.fps
    )
    preview.preview()
  else:
    from recorder import Recorder

    recorder = Recorder(
      cliParser.time, cliParser.rgbRes, cliParser.monoRes, cliParser.fps
    )
    recorder.record(
      cliParser.outDir,
      cliParser.rgbEncoder,
      cliParser.monoEncoder,
      cliParser.quality,
      cliParser.keyframe,
    )
