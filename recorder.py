import depthai as dai

import cv2
import numpy as np

from datetime import timedelta, datetime
from pathlib import Path


class Recorder():

  def __init__(self, rgbRes: str, monoRes: str, fps: int) -> None:
    rgbResMap = {
        "12mp": dai.ColorCameraProperties.SensorResolution.THE_12_MP,
        "4k": dai.ColorCameraProperties.SensorResolution.THE_4_K,
        "1080p": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
    }
    monoResMap = {
        "800p": dai.MonoCameraProperties.SensorResolution.THE_800_P,
        "720p": dai.MonoCameraProperties.SensorResolution.THE_720_P,
        "400p": dai.MonoCameraProperties.SensorResolution.THE_400_P,
    }

    self.rgbRes = rgbRes
    self.rgbResPreset = rgbResMap[rgbRes]
    self.monoRes = monoRes
    self.monoResPreset = monoResMap[monoRes]
    self.fps = fps

  def __initRecord(self,
                   encoder: str = "h265",
                   encoderQuality: int = 100,
                   keyframeFrequency: int = 0):
    encoderProfileMap = {
        "h265": dai.VideoEncoderProperties.Profile.H265_MAIN,
        "mjpeg": dai.VideoEncoderProperties.Profile.MJPEG,
        "lossless": dai.VideoEncoderProperties.Profile.MJPEG,
    }
    encoderProfile = encoderProfileMap[encoder]

    pipeline = dai.Pipeline()

    # Init nodes
    rgbCam = pipeline.create(dai.node.ColorCamera)
    leftCam = pipeline.create(dai.node.MonoCamera)
    rightCam = pipeline.create(dai.node.MonoCamera)

    rgbEncoder = pipeline.create(dai.node.VideoEncoder)
    leftEncoder = pipeline.create(dai.node.VideoEncoder)
    rightEncoder = pipeline.create(dai.node.VideoEncoder)

    rgbEncoded = pipeline.create(dai.node.XLinkOut)
    leftEncoded = pipeline.create(dai.node.XLinkOut)
    rightH265 = pipeline.create(dai.node.XLinkOut)

    rgbEncoded.setStreamName("rgbEncoded")
    leftEncoded.setStreamName("leftEncoded")
    rightH265.setStreamName("rightH265")

    ## For get timestamp
    metadata = pipeline.create(dai.node.XLinkOut)
    metadata.setStreamName("metadata")
    metadata.setMetadataOnly(True)

    # Link nodes
    rgbCam.video.link(rgbEncoder.input)
    leftCam.out.link(leftEncoder.input)
    rightCam.out.link(rightEncoder.input)

    rgbEncoder.bitstream.link(rgbEncoded.input)
    leftEncoder.bitstream.link(leftEncoded.input)
    rightEncoder.bitstream.link(rightH265.input)

    rgbCam.video.link(metadata.input)

    # Config cameras
    rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
    leftCam.setBoardSocket(dai.CameraBoardSocket.LEFT)
    rightCam.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    rgbCam.setResolution(self.rgbResPreset)
    leftCam.setResolution(self.monoResPreset)
    rightCam.setResolution(self.monoResPreset)

    rgbCam.setFps(self.fps)
    leftCam.setFps(self.fps)
    rightCam.setFps(self.fps)

    # Config encoder
    rgbEncoder.setDefaultProfilePreset(rgbCam.getFps(), encoderProfile)
    leftEncoder.setDefaultProfilePreset(leftCam.getFps(), encoderProfile)
    rightEncoder.setDefaultProfilePreset(rgbCam.getFps(), encoderProfile)

    rgbEncoder.setQuality(encoderQuality)
    leftEncoder.setQuality(encoderQuality)
    rightEncoder.setQuality(encoderQuality)

    if encoder == "lossless":
      rgbEncoder.setLossless(True)
      leftEncoder.setLossless(True)
      rightEncoder.setLossless(True)

    if keyframeFrequency:
      rgbEncoder.setKeyframeFrequency(int(keyframeFrequency))
      leftEncoder.setKeyframeFrequency(int(keyframeFrequency))
      rightEncoder.setKeyframeFrequency(int(keyframeFrequency))

    self.__pipeline = pipeline
    self.__rgbCam = rgbCam

  def __initPreview(self):
    self.__depthWeight = 100
    self.__rgbWeight = 0

    pipeline = dai.Pipeline()

    # Init nodes
    rgbCam = pipeline.create(dai.node.ColorCamera)
    leftCam = pipeline.create(dai.node.MonoCamera)
    rightCam = pipeline.create(dai.node.MonoCamera)

    ## Preview
    stereo = pipeline.create(dai.node.StereoDepth)
    rgbOut = pipeline.create(dai.node.XLinkOut)
    disparityOut = pipeline.create(dai.node.XLinkOut)

    # Link nodes
    rgbCam.video.link(rgbOut.input)
    leftCam.out.link(stereo.left)
    rightCam.out.link(stereo.right)
    stereo.disparity.link(disparityOut.input)

    # Config cameras
    rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
    leftCam.setBoardSocket(dai.CameraBoardSocket.LEFT)
    rightCam.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    rgbCam.setResolution(self.rgbResPreset)
    leftCam.setResolution(self.monoResPreset)
    rightCam.setResolution(self.monoResPreset)

    rgbCam.setFps(self.fps)
    leftCam.setFps(self.fps)
    rightCam.setFps(self.fps)

    # Config depth preview
    rgbOut.setStreamName("rgbOut")
    disparityOut.setStreamName("disparityOut")

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    maxDisparity = stereo.initialConfig.getMaxDisparity()

    self.__pipeline = pipeline
    self.__rgbCam = rgbCam
    self.__maxDisparity = maxDisparity

  def __getFrameTime(self, timestamp: timedelta) -> datetime:
    return datetime.now() - (dai.Clock.now() - timestamp)

  def __updateBlendWeights(self, depthPercent):
    self.__depthWeight = depthPercent
    self.__rgbWeight = 100 - self.__depthWeight

  def record(self,
             outputDirPath: Path,
             encoder: str,
             encoderQuality: int,
             keyframeFrequency: int = 0):
    self.__initRecord(encoder, encoderQuality, keyframeFrequency)

    # Recording
    with dai.Device(self.__pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibration = device.readCalibration()
      lensPosition = calibration.getLensPosition(dai.CameraBoardSocket.RGB)
      self.__rgbCam.initialControl.setManualFocus(lensPosition)

      # Output queues
      metadataQ = device.getOutputQueue(
          name="metadata", maxSize=self.fps, blocking=True)
      rgbEncodedQ = device.getOutputQueue(
          name="rgbEncoded", maxSize=self.fps, blocking=True)
      leftEncodedQ = device.getOutputQueue(
          name="leftEncoded", maxSize=self.fps, blocking=True)
      rightEncoded = device.getOutputQueue(
          name="rightH265", maxSize=self.fps, blocking=True)

      # Init output path
      startTime = self.__getFrameTime(metadataQ.get().getTimestamp())
      dirTimestamp = startTime.astimezone().isoformat().replace(":", ";")
      tag = f"[{dirTimestamp}][{self.fps}FPS]"
      if encoder == "lossless":
        tag = tag + "[Lossless]"
        videoExt = "mjpeg"
      else:
        videoExt = encoder

      outputDirPath.mkdir(parents=True, exist_ok=True)
      subDirPath = outputDirPath.joinpath(f"{tag}/")
      subDirPath.mkdir(exist_ok=True)

      ## Calibration data output
      calibrationPath = subDirPath.joinpath(f"calibration.json")
      calibration.eepromToJsonFile(calibrationPath)

      ## Video output
      rgbEncodedPath = subDirPath.joinpath(
          f"[{self.rgbRes.title()}]rgb.{videoExt}")
      leftEncodedPath = subDirPath.joinpath(
          f"[{self.monoRes.title()}]left.{videoExt}")
      rightH265Path = subDirPath.joinpath(
          f"[{self.monoRes.title()}]right.{videoExt}")

      ## Timestamp output
      timestampPath = subDirPath.joinpath(f"timestamp.txt")
      timestampPath.unlink(True)

      # Write files
      with open(timestampPath, "at") as timestampFile, open(
          rgbEncodedPath,
          "wb") as rgbFile, open(leftEncodedPath,
                                 "wb") as leftFile, open(rightH265Path,
                                                         "wb") as rightFile:
        # Recording
        print(f"{startTime}, start recording (Press Ctrl+C to stop)")
        timestampFile.write(startTime.astimezone().isoformat() + "\n")
        while True:
          try:
            recordingTime = datetime.now() - startTime
            print(f"\rRecording: {recordingTime}...", end="")

            while metadataQ.has():
              timestamp = self.__getFrameTime(metadataQ.get().getTimestamp())
              timestampFile.write(timestamp.astimezone().isoformat() + "\n")
            while rgbEncodedQ.has():
              rgbEncodedQ.get().getData().tofile(rgbFile)
            while leftEncodedQ.has():
              leftEncodedQ.get().getData().tofile(leftFile)
            while rightEncoded.has():
              rightEncoded.get().getData().tofile(rightFile)
          except KeyboardInterrupt:
            break
      print("\nStop recording...")
    print(f"Output files in: \"{outputDirPath.absolute()}\"")

  def preview(self):
    self.__initPreview()
    with dai.Device(self.__pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibrationbData = device.readCalibration()
      lensPosition = calibrationbData.getLensPosition(dai.CameraBoardSocket.RGB)
      self.__rgbCam.initialControl.setManualFocus(lensPosition)

      # Trackbar adjusts blending ratio of rgb/depth
      windowName = "Preview"
      cv2.namedWindow(windowName)
      cv2.createTrackbar("Depth%", windowName, self.__depthWeight, 100,
                         self.__updateBlendWeights)

      print("Previewing... (Press Q on frame or Ctrl+C to stop)")

      rgbFrame = None
      disparityFrame = None
      while True:
        try:
          latestFrame = {"rgbOut": None, "disparityOut": None}
          queues = device.getQueueEvents([x for x in latestFrame.keys()])
          for queue in queues:
            packets = device.getOutputQueue(queue).tryGetAll()
            if len(packets) > 0:
              latestFrame[queue] = packets[-1]

          if latestFrame["rgbOut"] is not None:
            rgbFrame = latestFrame["rgbOut"].getCvFrame()

          if latestFrame["disparityOut"] is not None:
            disparityFrame = latestFrame["disparityOut"].getFrame()
            disparityFrame = (disparityFrame * 255. /
                              self.__maxDisparity).astype(np.uint8)
            disparityFrame = cv2.applyColorMap(disparityFrame,
                                               cv2.COLORMAP_BONE)
            disparityFrame = np.ascontiguousarray(disparityFrame)

          # Blend when both received
          if (rgbFrame is not None) and (disparityFrame is not None):
            if len(disparityFrame.shape) < 3:
              disparityFrame = cv2.cvtColor(disparityFrame, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(rgbFrame,
                                      float(self.__rgbWeight) / 100,
                                      disparityFrame,
                                      float(self.__depthWeight) / 100, 0)
            cv2.imshow(windowName, blended)
            rgbFrame = None
            disparityFrame = None

          if cv2.waitKey(1) == ord("q"):
            break
        except KeyboardInterrupt:
          break

      print("Stop preview...")
      cv2.destroyAllWindows()
