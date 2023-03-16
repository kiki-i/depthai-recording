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

    self.rgbRes = rgbResMap[rgbRes]
    self.monoRes = monoResMap[monoRes]
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

    rgbH265 = pipeline.create(dai.node.XLinkOut)
    leftH265 = pipeline.create(dai.node.XLinkOut)
    rightH265 = pipeline.create(dai.node.XLinkOut)

    rgbH265.setStreamName("rgbH265")
    leftH265.setStreamName("leftH265")
    rightH265.setStreamName("rightH265")

    ## For get timestamp
    metadata = pipeline.create(dai.node.XLinkOut)
    metadata.setStreamName("metadata")
    metadata.setMetadataOnly(True)

    # Link nodes
    rgbCam.video.link(rgbEncoder.input)
    leftCam.out.link(leftEncoder.input)
    rightCam.out.link(rightEncoder.input)

    rgbEncoder.bitstream.link(rgbH265.input)
    leftEncoder.bitstream.link(leftH265.input)
    rightEncoder.bitstream.link(rightH265.input)

    rgbCam.video.link(metadata.input)

    # Config cameras
    rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
    leftCam.setBoardSocket(dai.CameraBoardSocket.LEFT)
    rightCam.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    rgbCam.setResolution(self.rgbRes)
    leftCam.setResolution(self.monoRes)
    rightCam.setResolution(self.monoRes)

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

    rgbCam.setResolution(self.rgbRes)
    leftCam.setResolution(self.monoRes)
    rightCam.setResolution(self.monoRes)

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
             convertToMp4: bool,
             encoder: str,
             encoderQuality: int,
             keyframeFrequency: int = 0):
    self.__initRecord(encoder, encoderQuality, keyframeFrequency)

    # Check output path
    if not outputDirPath.exists():
      outputDirPath.mkdir(parents=True, exist_ok=True)

    # Recording
    with dai.Device(self.__pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibration = device.readCalibration()
      lensPosition = calibration.getLensPosition(dai.CameraBoardSocket.RGB)
      self.__rgbCam.initialControl.setManualFocus(lensPosition)

      # Output queues
      metadataQ = device.getOutputQueue(
          name="metadata", maxSize=1, blocking=False)
      rgbH265Q = device.getOutputQueue(
          name="rgbH265", maxSize=self.fps, blocking=True)
      leftH265Q = device.getOutputQueue(
          name="leftH265", maxSize=self.fps, blocking=True)
      rightH265Q = device.getOutputQueue(
          name="rightH265", maxSize=self.fps, blocking=True)

      # Get first RGB frame accurate timestamp
      startTime = self.__getFrameTime(metadataQ.get().getTimestamp())
      timestamp = startTime.astimezone().isoformat().replace(":", ";")

      tag = f"[{timestamp}][{self.fps}FPS]"
      if encoder == "lossless":
        tag = tag + "[Lossless]"
        extension = "mjpeg"
      else:
        extension = encoder
      rgbH265Path = outputDirPath.joinpath(f"{tag}rgb.{extension}")
      leftH265Path = outputDirPath.joinpath(f"{tag}left.{extension}")
      rightH265Path = outputDirPath.joinpath(f"{tag}right.{extension}")

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
            print(f"\rRecording: {recordingTime}...", end="")
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
    if convertToMp4:
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

  def preview(self):
    self.__initPreview()
    with dai.Device(self.__pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibrationbData = device.readCalibration()
      lensPosition = calibrationbData.getLensPosition(dai.CameraBoardSocket.RGB)
      self.__rgbCam.initialControl.setManualFocus(lensPosition)

      rgbFrame = None
      disparityFrame = None

      # Trackbar adjusts blending ratio of rgb/depth
      windowName = "Preview"
      cv2.namedWindow(windowName)
      cv2.createTrackbar("Depth%", windowName, self.__depthWeight, 100,
                         self.__updateBlendWeights)

      print("Previewing... (Press Q on frame or Ctrl+C to stop)")
      while True:
        try:
          latestPacket = {}
          latestPacket["rgbOut"] = None
          latestPacket["disparityOut"] = None

          queueEvents = device.getQueueEvents(("rgbOut", "disparityOut"))
          for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
              latestPacket[queueName] = packets[-1]

          if latestPacket["rgbOut"] is not None:
            rgbFrame = latestPacket["rgbOut"].getCvFrame()

          if latestPacket["disparityOut"] is not None:
            disparityFrame = latestPacket["disparityOut"].getFrame()
            disparityFrame = (disparityFrame * 255. /
                              self.__maxDisparity).astype(np.uint8)
            disparityFrame = cv2.applyColorMap(disparityFrame,
                                               cv2.COLORMAP_BONE)
            disparityFrame = np.ascontiguousarray(disparityFrame)

          # Blend when both received
          if (rgbFrame is not None) and (disparityFrame is not None):
            # Need to have both frames in BGR format before blending
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
