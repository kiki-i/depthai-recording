#!/usr/bin/env python3

from pipeline import *

import cv2
import depthai as dai
import numpy as np

from datetime import datetime
from pathlib import Path

import argparse

# Config
rgbRes = dai.ColorCameraProperties.SensorResolution.THE_1080_P
monoRes = dai.MonoCameraProperties.SensorResolution.THE_720_P
fps = 30


def parseCli():
  description = "Video recording with OAK-D "
  parser = argparse.ArgumentParser(description=description)
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
  outputPath = cliArgs.out
  if ":" not in outputPath:
    outputDirPath = scriptPath.joinpath(outputPath)
  else:
    outputDirPath = Path(outputPath)
  if not outputDirPath.exists():
    outputDirPath.mkdir(parents=True, exist_ok=True)

  pipeline, disparityMultiplier = initPipeline(rgbRes, monoRes, fps)
  with dai.Device(pipeline) as device:
    rgbH265Q = device.getOutputQueue(name="rgbH265", maxSize=fps, blocking=True)
    leftH265Q = device.getOutputQueue(
        name="leftH265", maxSize=fps, blocking=True)
    rightH265Q = device.getOutputQueue(
        name="rightH265", maxSize=fps, blocking=True)

    disparityOutQ = device.getOutputQueue(
        name="disparityOut", maxSize=fps, blocking=False)
    disparityFrame = None

    print("Recording... (Press Q on frame or Ctrl+C to stop)")

    rgbH265Path = outputDirPath.joinpath(".cache.rgb.h265")
    leftH265Path = outputDirPath.joinpath(".cache.left.h265")
    rightH265Path = outputDirPath.joinpath(".cache.right.h265")

    with open(rgbH265Path,
              "wb") as rgbFile, open(leftH265Path, "wb") as leftFile, open(
                  rightH265Path, "wb") as rightFile:
      startTime = datetime.now()
      while True:
        try:
          # Write video files
          while rgbH265Q.has():
            rgbH265Q.get().getData().tofile(rgbFile)
          while leftH265Q.has():
            leftH265Q.get().getData().tofile(leftFile)
          while rightH265Q.has():
            rightH265Q.get().getData().tofile(rightFile)

          # Display preview
          disparityPreview = disparityOutQ.tryGet()
          if disparityPreview is not None:
            disparityFrame = disparityPreview.getCvFrame()
            disparityFrame = (disparityFrame * disparityMultiplier).astype(
                np.uint8)
            disparityFrame = cv2.applyColorMap(disparityFrame,
                                               cv2.COLORMAP_BONE)
            cv2.imshow("Disparity Preview", disparityFrame)

            if cv2.waitKey(1) == ord("q"):
              break
        except KeyboardInterrupt:
          break

      print("Stop recording...")
      cv2.destroyAllWindows()

    timestamp = startTime.astimezone().isoformat().replace(":", ";")
    # Write calibration file
    print("Backing up calibration...")
    calibrationPath = outputDirPath.joinpath(f"[{timestamp}]calibration.json")
    calibData = device.readCalibration()
    calibData.eepromToJsonFile(calibrationPath)

  # Add timestamp
  rgbH265Path = rgbH265Path.rename(
      outputDirPath.joinpath(f"[{timestamp}]rgb.h265"))
  leftH265Path = leftH265Path.rename(
      outputDirPath.joinpath(f"[{timestamp}]left.h265"))
  rightH265Path = rightH265Path.rename(
      outputDirPath.joinpath(f"[{timestamp}]right.h265"))

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
    convertor.convertAll(processPaths, fps)

  print(f"Output files in: \"{outputDirPath}\"")
