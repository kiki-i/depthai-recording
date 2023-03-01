#!/usr/bin/env python3

from config import *
from pipeline import *
from preview import *

import depthai as dai

from datetime import datetime
from pathlib import Path

import argparse


def parseCli():
  description = "Record RGB and stereo video with OAK-D"
  parser = argparse.ArgumentParser(description=description)
  parser.add_argument(
      "-p",
      "--preview",
      action="store_const",
      const=True,
      default=False,
      help="Show preview, nothing will be recorded")
  parser.add_argument(
      "-m",
      "--mp4",
      action="store_const",
      const=True,
      default=False,
      help="Convert the raw H.265 file to mp4 (Require ffmpeg be installed)")
  parser.add_argument(
      "-o",
      "--out",
      type=str,
      metavar="",
      default="output",
      help="Output directory path, default=\"output\"")
  args = parser.parse_args()
  return args


if __name__ == "__main__":
  scriptPath = Path(__file__).parent

  # Init
  print("Init...")
  cliArgs = parseCli()
  if cliArgs.preview:
    preview()
  else:
    # Recording
    outputPath = cliArgs.out
    if ":" not in outputPath:
      outputDirPath = scriptPath.joinpath(outputPath)
    else:
      outputDirPath = Path(outputPath)
    if not outputDirPath.exists():
      outputDirPath.mkdir(parents=True, exist_ok=True)

    pipeline = initPipeline(rgbRes, monoRes, fps)
    with dai.Device(pipeline) as device:
      rgbH265Q = device.getOutputQueue(
          name="rgbH265", maxSize=fps, blocking=True)
      leftH265Q = device.getOutputQueue(
          name="leftH265", maxSize=fps, blocking=True)
      rightH265Q = device.getOutputQueue(
          name="rightH265", maxSize=fps, blocking=True)

      print("Press Ctrl+C to stop recording")

      timestamp = datetime.now().astimezone().isoformat().replace(":", ";")
      rgbH265Path = outputDirPath.joinpath(f".cache.[{timestamp}]rgb.h265")
      leftH265Path = outputDirPath.joinpath(f".cache.[{timestamp}]left.h265")
      rightH265Path = outputDirPath.joinpath(f".cache.[{timestamp}]right.h265")

      # Write video files
      with open(rgbH265Path,
                "wb") as rgbFile, open(leftH265Path, "wb") as leftFile, open(
                    rightH265Path, "wb") as rightFile:
        startTime = datetime.now()
        while True:
          try:
            runningTime = datetime.now() - startTime
            print(f"\rRecording: {runningTime.seconds}s...", end="")
            while rgbH265Q.has():
              rgbH265Q.get().getData().tofile(rgbFile)
            while leftH265Q.has():
              leftH265Q.get().getData().tofile(leftFile)
            while rightH265Q.has():
              rightH265Q.get().getData().tofile(rightFile)
          except KeyboardInterrupt:
            break
        print("\nStop recording...")

      timestamp = startTime.astimezone().isoformat().replace(":", ";")
      # Write calibration file
      print("Backing up calibration...")
      calibrationPath = outputDirPath.joinpath(f"[{timestamp}]calibration.json")
      calibData = device.readCalibration()
      calibData.eepromToJsonFile(calibrationPath)

    # More accurate timestamp
    rgbH265Path = rgbH265Path.rename(
        outputDirPath.joinpath(f"[{timestamp}]rgb.h265"))
    leftH265Path = leftH265Path.rename(
        outputDirPath.joinpath(f"[{timestamp}]left.h265"))
    rightH265Path = rightH265Path.rename(
        outputDirPath.joinpath(f"[{timestamp}]right.h265"))

    # Convert to MP4
    if cliArgs.mp4:
      import convertor
      rgbMp4Path = rgbH265Path.parent.joinpath(f"[{timestamp}]rgb.mp4")
      leftMp4Path = leftH265Path.parent.joinpath(f"[{timestamp}]left.mp4")
      rightMp4Path = rightH265Path.parent.joinpath(f"[{timestamp}]right.mp4")
      processPaths = {
          rgbH265Path: rgbMp4Path,
          leftH265Path: leftMp4Path,
          rightH265Path: rightMp4Path
      }
      convertor.convertAll(processPaths)

    print(f"Output files in: \"{outputDirPath}\"")
