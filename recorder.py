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

  def __createPreviewWindow(self, windowName: str):
    self.__depthWeight = 100
    self.__rgbWeight = 0
    cv2.namedWindow(windowName)
    cv2.createTrackbar("Depth %", windowName, self.__depthWeight, 100,
                       self.__updateBlendWeights)

  def __getFrameDatetime(self, timestamp: timedelta) -> datetime:
    return datetime.now() - (dai.Clock.now() - timestamp)

  def __initRecord(self, rgbProfile: str, monoProfile: str, quality: int,
                   keyframeFrequency: int):
    profilePresetMap = {
        "h265": dai.VideoEncoderProperties.Profile.H265_MAIN,
        "mjpeg": dai.VideoEncoderProperties.Profile.MJPEG,
        "lossless": dai.VideoEncoderProperties.Profile.MJPEG,
    }
    rgbProfilePreset = profilePresetMap[rgbProfile]
    monoProfilePreset = profilePresetMap[monoProfile]

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
    rightEncoded = pipeline.create(dai.node.XLinkOut)

    rgbEncoded.setStreamName("rgbEncoded")
    leftEncoded.setStreamName("leftEncoded")
    rightEncoded.setStreamName("rightEncoded")

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
    rightEncoder.bitstream.link(rightEncoded.input)

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
    rgbEncoder.setDefaultProfilePreset(rgbCam.getFps(), rgbProfilePreset)
    leftEncoder.setDefaultProfilePreset(leftCam.getFps(), monoProfilePreset)
    rightEncoder.setDefaultProfilePreset(rgbCam.getFps(), monoProfilePreset)

    rgbEncoder.setQuality(quality)
    leftEncoder.setQuality(quality)
    rightEncoder.setQuality(quality)

    if rgbEncoder == "lossless":
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

  def __showPreview(self,
                    rgbFrame: cv2.Mat,
                    disparityFrame: np.ndarray,
                    scaleFactor: float = 1,
                    windowsName: str = "",
                    text: str = ""):
    disparityFrame = (disparityFrame * 255. / self.__maxDisparity).astype(
        np.uint8)
    disparityFrame = cv2.applyColorMap(disparityFrame, cv2.COLORMAP_BONE)
    disparityFrame = np.ascontiguousarray(disparityFrame)

    if len(disparityFrame.shape) < 3:
      disparityFrame = cv2.cvtColor(disparityFrame, cv2.COLOR_GRAY2BGR)
    blended = cv2.addWeighted(rgbFrame,
                              float(self.__rgbWeight) / 100, disparityFrame,
                              float(self.__depthWeight) / 100, 0)
    cv2.putText(blended, text, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (255, 255, 255), 3)
    scaledSize = (int(blended.shape[1] * scaleFactor),
                  int(blended.shape[0] * scaleFactor))
    scaled = cv2.resize(blended, scaledSize)
    cv2.imshow(windowsName, scaled)

  def __updateBlendWeights(self, depthPercent):
    self.__depthWeight = depthPercent
    self.__rgbWeight = 100 - self.__depthWeight

  def record(self, outDirPath: Path, rgbProfile: str, monoProfile: str,
             quality: int, keyframeFrequency: int):
    self.__initRecord(rgbProfile, monoProfile, quality, keyframeFrequency)

    # Init output dir
    dirTimestamp = datetime.now().astimezone().isoformat().replace(":", ";")
    dirTag = f"[{dirTimestamp}][{self.fps}FPS]"
    dirTag += f"[{rgbProfile},{monoProfile}]"
    if rgbProfile == "lossless":
      rgbExt = "mjpeg"
    else:
      rgbExt = rgbProfile
    if monoProfile == "lossless":
      monoExt = "mjpeg"
    else:
      monoExt = monoProfile

    outDirPath.mkdir(parents=True, exist_ok=True)
    subDirPath = outDirPath / f"{dirTag}/"
    subDirPath.mkdir(exist_ok=True)

    # Video output file
    rgbPath = subDirPath / f"{self.rgbRes}.rgb.{rgbExt}"
    leftPath = subDirPath / f"{self.monoRes}.left.{monoExt}"
    rightPath = subDirPath / f"{self.monoRes}.right.{monoExt}"

    # Timestamp output file
    timestampPath = subDirPath / "rgb.timestamp.txt"
    # Calibration data output
    calibrationPath = subDirPath / "calibration.json"

    print(f"Press Ctrl+C to stop recording...")
    rgbFrameCount: int = 0
    leftFrameCount: int = 0
    rightFrameCount: int = 0
    rgbFps: float = 0.0
    leftFps: float = 0.0
    rightFps: float = 0.0

    with (open(rgbPath, "wb") as rgbFile,
          open(leftPath, "wb") as leftFile,
          open(rightPath, "wb") as rightFile,
          open(timestampPath, "at") as timestampFile):
      with dai.Device(self.__pipeline) as device:
        # RGB needs fixed focus to properly align with depth
        calibration = device.readCalibration()
        lensPosition = calibration.getLensPosition(dai.CameraBoardSocket.RGB)
        self.__rgbCam.initialControl.setManualFocus(lensPosition)
        calibration.eepromToJsonFile(calibrationPath)

        # Output queues
        metadataQ = device.getOutputQueue(
            name="metadata", maxSize=self.fps, blocking=True)
        rgbEncodedQ = device.getOutputQueue(
            name="rgbEncoded", maxSize=self.fps, blocking=True)
        leftEncodedQ = device.getOutputQueue(
            name="leftEncoded", maxSize=self.fps, blocking=True)
        rightEncodedQ = device.getOutputQueue(
            name="rightEncoded", maxSize=self.fps, blocking=True)

        startTime = self.__getFrameDatetime(metadataQ.get().getTimestamp())
        timestampFile.write(startTime.astimezone().isoformat() + "\n")

        # Recording
        while True:
          try:
            recordingTime = datetime.now() - startTime
            recordingSeconds = recordingTime.seconds
            if recordingSeconds:
              rgbFps: float = rgbFrameCount / recordingTime.seconds
              leftFps: float = leftFrameCount / recordingTime.seconds
              rightFps: float = rightFrameCount / recordingTime.seconds

            print(
                f"\rFPS: {rgbFps:.1f} L{leftFps:.1f} R{rightFps:.1f}, Time: {recordingTime}",
                end="")

            while metadataQ.has():
              timestamp = self.__getFrameDatetime(metadataQ.get().getTimestamp())
              timestampFile.write(timestamp.astimezone().isoformat() + "\n")
            while rgbEncodedQ.has():
              rgbEncodedQ.get().getData().tofile(rgbFile)
              rgbFrameCount += 1
            while leftEncodedQ.has():
              leftEncodedQ.get().getData().tofile(leftFile)
              leftFrameCount += 1
            while rightEncodedQ.has():
              rightEncodedQ.get().getData().tofile(rightFile)
              rightFrameCount += 1
          except KeyboardInterrupt:
            break

      print("\nStop recording...")

    print(f"Output files in \"{outDirPath.absolute()}\"")

  def preview(self):
    self.__initPreview()
    with dai.Device(self.__pipeline) as device:
      # RGB needs fixed focus to properly align with depth
      calibrationbData = device.readCalibration()
      lensPosition = calibrationbData.getLensPosition(dai.CameraBoardSocket.RGB)
      self.__rgbCam.initialControl.setManualFocus(lensPosition)

      print("Previewing... (Press Q on frame or Ctrl+C to stop)")

      self.__createPreviewWindow("Depth")
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

          if (rgbFrame is not None) and (disparityFrame is not None):
            scaleFactor: float = 1 / 2
            self.__showPreview(rgbFrame, disparityFrame, scaleFactor, "Depth")

            rgbFrame = None
            disparityFrame = None

          if cv2.waitKey(1) == ord("q"):
            break
        except KeyboardInterrupt:
          break

      print("Stop preview...")
      cv2.destroyAllWindows()
