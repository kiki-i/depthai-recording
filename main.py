#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

from concurrent.futures import Future, ThreadPoolExecutor, as_completed
from datetime import datetime
from multiprocessing import cpu_count
from pathlib import Path

import argparse
import subprocess

# Config
## Conifg recording
rgbRes = dai.ColorCameraProperties.SensorResolution.THE_1080_P
monoRes = dai.MonoCameraProperties.SensorResolution.THE_720_P
fps = 30
## Conifg encoder
encoderProfile = dai.VideoEncoderProperties.Profile.H265_MAIN
encoderQuality = 100
## Config disparity preview
lrCheck = True
extendedDisparity = False
subpixel = False

disparityMultiplier: float = 1


def checkCliExist(path: str) -> bool:
  try:
    subprocess.run(path, capture_output=True)
  except FileNotFoundError:
    return False
  else:
    return True


def clenCliLine(override: str = "", end: str = "\n", width: int = 90):
  print("\r" + " " * width, end="")
  if override:
    print("\r" + override, end=end)


def convertToMp4(inputPath: Path, outputPath: Path) -> str:
  error = ""
  result = subprocess.run(
      f"ffmpeg -framerate {fps} -i {inputPath} -c copy -y {outputPath}",
      capture_output=True,
      text=True)
  if result.returncode != 0:
    error = f"{result.stderr}\n{result.stdout}"
  return error


def multiprocessFiles(processPaths: dict[Path, Path], fun) -> dict[Path, str]:
  threadLimit = int(cpu_count() / 2)
  errorList: dict[Path, str] = {}

  # Init thread
  total: int = len(processPaths)
  threadResult: dict[Future, Path] = {} # Future: inputPath
  with ThreadPoolExecutor(max_workers=threadLimit) as threadPool:

    # Run subprocess
    for inputPath, outputPath in processPaths.items():
      try:
        future = threadPool.submit(fun, inputPath, outputPath)
        threadResult[future] = inputPath
      except Exception as e:
        errorList[inputPath] = str(e)

    # Wait all subprocess
    current: int = 0
    clenCliLine(f"Processing {current}/{total}...", end="")
    for future in as_completed(threadResult):
      current = current + 1
      clenCliLine(f"Processing {current}/{total}...", end="")

      inputPath = threadResult[future]
      error = future.result()
      if error:
        errorList[inputPath] = error.strip()

    clenCliLine(f"Processing {current}/{total}.")
  return errorList


def parseCli():
  description = "Video recording with OAK-D "
  parser = argparse.ArgumentParser(description=description)
  parser.add_argument(
      "-o",
      "--out",
      type=str,
      metavar="",
      default="output",
      help="Output directory path, default=\"output\"")
  parser.add_argument(
      "-m",
      "--mp4",
      action="store_const",
      const=True,
      default=False,
      help="Convert the raw H.265 file to mp4 (Require ffmpeg be installed)")
  args = parser.parse_args()
  return args


def initPipeline():
  pipeline = dai.Pipeline()

  # Init nodes
  rgbCam = pipeline.create(dai.node.ColorCamera)
  leftCam = pipeline.create(dai.node.MonoCamera)
  rightCam = pipeline.create(dai.node.MonoCamera)

  rgbEncoder = pipeline.create(dai.node.VideoEncoder)
  leftEncoder = pipeline.create(dai.node.VideoEncoder)
  rightEncoder = pipeline.create(dai.node.VideoEncoder)

  rgbH265 = pipeline.create(dai.node.XLinkOut)
  leftH265 = pipeline.create(dai.node.XLinkOut)
  rightH265 = pipeline.create(dai.node.XLinkOut)

  rgbH265.setStreamName("rgbH265")
  leftH265.setStreamName("leftH265")
  rightH265.setStreamName("rightH265")

  ## Disparity preview
  depth = pipeline.create(dai.node.StereoDepth)
  disparityOut = pipeline.create(dai.node.XLinkOut)
  disparityOut.setStreamName("disparityOut")

  # Link nodes
  rgbCam.video.link(rgbEncoder.input)
  leftCam.out.link(leftEncoder.input)
  rightCam.out.link(rightEncoder.input)

  rgbEncoder.bitstream.link(rgbH265.input)
  leftEncoder.bitstream.link(leftH265.input)
  rightEncoder.bitstream.link(rightH265.input)

  leftCam.out.link(depth.left)
  rightCam.out.link(depth.right)
  depth.disparity.link(disparityOut.input)

  # Config cameras
  rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
  leftCam.setBoardSocket(dai.CameraBoardSocket.LEFT)
  rightCam.setBoardSocket(dai.CameraBoardSocket.RIGHT)

  rgbCam.setResolution(rgbRes)
  leftCam.setResolution(monoRes)
  rightCam.setResolution(monoRes)

  rgbCam.setFps(fps)
  leftCam.setFps(fps)
  rightCam.setFps(fps)

  # Config encoder
  rgbEncoder.setDefaultProfilePreset(fps, encoderProfile)
  leftEncoder.setDefaultProfilePreset(fps, encoderProfile)
  rightEncoder.setDefaultProfilePreset(fps, encoderProfile)

  rgbEncoder.setQuality(encoderQuality)
  leftEncoder.setQuality(encoderQuality)
  rightEncoder.setQuality(encoderQuality)

  # Config depth preview
  depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
  depth.setLeftRightCheck(lrCheck)
  depth.setExtendedDisparity(extendedDisparity)
  depth.setSubpixel(subpixel)
  global disparityMultiplier
  disparityMultiplier = 255 / depth.initialConfig.getMaxDisparity()

  return pipeline


if __name__ == "__main__":
  scriptPath = Path(__file__).parent

  # Init path
  cliArgs = parseCli()
  outputPath = cliArgs.out
  if ":" not in outputPath:
    outputDirPath = scriptPath.joinpath(outputPath)
  else:
    outputDirPath = Path(outputPath)
  if not outputDirPath.exists():
    outputDirPath.mkdir(parents=True, exist_ok=True)

  pipeline = initPipeline()
  with dai.Device(pipeline) as device:
    rgbH265Q = device.getOutputQueue(name="rgbH265", maxSize=fps, blocking=True)
    leftH265Q = device.getOutputQueue(
        name="leftH265", maxSize=fps, blocking=True)
    rightH265Q = device.getOutputQueue(
        name="rightH265", maxSize=fps, blocking=True)

    disparityOutQ = device.getOutputQueue(name="disparityOut", maxSize=fps)
    disparityFrame = None

    print("Press Q on frame or Ctrl+C to stop recording...")
    timestamp = datetime.now().astimezone().isoformat().replace(":", ";")

    rgbH265Path = outputDirPath.joinpath(f"[{timestamp}]rgb.h265")
    leftH265Path = outputDirPath.joinpath(f"[{timestamp}]left.h265")
    rightH265Path = outputDirPath.joinpath(f"[{timestamp}]right.h265")

    with open(rgbH265Path,
              "wb") as rgbFile, open(leftH265Path, "wb") as leftFile, open(
                  rightH265Path, "wb") as rightFile:
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

      # Write calibration file
      print("Backing up calibration...")
      calibrationPath = outputDirPath.joinpath(f"[{timestamp}]calibration.json")
      calibData = device.readCalibration()
      calibData.eepromToJsonFile(calibrationPath)

  if cliArgs.mp4:
    print("Converting to mp4...")

    if not checkCliExist("ffmpeg"):
      print("Can't find ffmpeg!")
    else:
      rgbMp4Path = rgbH265Path.parent.joinpath(f"[{timestamp}]rgb.mp4")
      leftMp4Path = leftH265Path.parent.joinpath(f"[{timestamp}]left.mp4")
      rightMp4Path = rightH265Path.parent.joinpath(f"[{timestamp}]right.mp4")
      processPaths = {
          rgbH265Path: rgbMp4Path,
          leftH265Path: leftMp4Path,
          rightH265Path: rightMp4Path
      }
      convertErrors = multiprocessFiles(processPaths, convertToMp4)
      if convertErrors:
        print("!Error: Complete conversion with error!\n")
        for file, error in convertErrors.items():
          print(f"!Error: {file}: {error}")
      else:
        print("Complete conversion!")

  print(f"Output files in: \"{outputDirPath}\"")
