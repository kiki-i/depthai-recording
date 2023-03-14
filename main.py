#!/usr/bin/env python3

from config import *
from pipeline import *
from preview import *

import depthai as dai

from datetime import timedelta, datetime
from pathlib import Path

import argparse


def getFrameTime(timestamp: timedelta) -> datetime:
  return datetime.now() - (dai.Clock.now() - timestamp)


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
      "-m",
      "--mp4",
      action="store_const",
      const=True,
      default=False,
      help="Convert the raw H.265 file to MP4 after recording (Require ffmpeg be installed)"
  )
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
    # Check output path
    outputDirPath = Path(cliArgs.out)
    if not outputDirPath.exists():
      outputDirPath.mkdir(parents=True, exist_ok=True)

    # Recording
    pipeline, rgbCam = initPipeline(rgbRes, monoRes, fps)
    with dai.Device(pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibration = device.readCalibration()
      lensPosition = calibration.getLensPosition(dai.CameraBoardSocket.RGB)
      rgbCam.initialControl.setManualFocus(lensPosition)

      # Output queues
      metadataQ = device.getOutputQueue(
          name="metadata", maxSize=1, blocking=False)
      rgbH265Q = device.getOutputQueue(
          name="rgbH265", maxSize=fps, blocking=True)
      leftH265Q = device.getOutputQueue(
          name="leftH265", maxSize=fps, blocking=True)
      rightH265Q = device.getOutputQueue(
          name="rightH265", maxSize=fps, blocking=True)

      # Get first RGB frame accurate timestamp
      startTime = getFrameTime(metadataQ.get().getTimestamp())
      timestamp = startTime.astimezone().isoformat().replace(":", ";")
      tag = f"[{timestamp}][{fps}FPS]"

      rgbH265Path = outputDirPath.joinpath(f"{tag}rgb.h265")
      leftH265Path = outputDirPath.joinpath(f"{tag}left.h265")
      rightH265Path = outputDirPath.joinpath(f"{tag}right.h265")

      # Backup calibration data
      calibrationPath = outputDirPath.joinpath(f"{tag}calibration.json")
      calibration.eepromToJsonFile(calibrationPath)

      # Write files
      with open(rgbH265Path,
                "wb") as rgbFile, open(leftH265Path, "wb") as leftFile, open(
                    rightH265Path, "wb") as rightFile:
        # Recording
        print(f"{startTime}, start recording (Press Ctrl+C to stop)")
        while True:
          try:
            recordingTime = datetime.now() - startTime
            if recordingTime:
              print(f"\rRecording: {recordingTime.seconds}s...", end="")
            while rgbH265Q.has():
              rgbH265Q.get().getData().tofile(rgbFile)
            while leftH265Q.has():
              leftH265Q.get().getData().tofile(leftFile)
            while rightH265Q.has():
              rightH265Q.get().getData().tofile(rightFile)
          except KeyboardInterrupt:
            break
      print("\nStop recording...")

    # Convert to MP4
    if cliArgs.mp4:
      import convertor
      rgbMp4Path = rgbH265Path.with_suffix(".mp4")
      leftMp4Path = leftH265Path.with_suffix(".mp4")
      rightMp4Path = rightH265Path.with_suffix(".mp4")
      processPaths = {
          rgbH265Path: rgbMp4Path,
          leftH265Path: leftMp4Path,
          rightH265Path: rightMp4Path
      }
      convertor.convertAll(processPaths)

    print(f"Output files in: \"{outputDirPath.absolute()}\"")
