import depthai as dai

from datetime import datetime, timedelta
from pathlib import Path


class Recorder:
  def __init__(self, time: timedelta, rgbRes: str, monoRes: str, fps: int) -> None:
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

    self.time = time
    self.rgbRes = rgbRes
    self.rgbResPreset = rgbResMap[rgbRes]
    self.monoRes = monoRes
    self.monoResPreset = monoResMap[monoRes]
    self.fps = fps

  def __getFrameDatetime(self, frame: dai.ImgFrame) -> datetime:
    return datetime.now() - (dai.Clock.now() - frame.getTimestamp())

  def __initRecord(
    self, rgbProfile: str, monoProfile: str, quality: int, keyframeFrequency: int
  ):
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
    lCam = pipeline.create(dai.node.MonoCamera)
    rCam = pipeline.create(dai.node.MonoCamera)

    rgbEncoder = pipeline.create(dai.node.VideoEncoder)
    lEncoder = pipeline.create(dai.node.VideoEncoder)
    rEncoder = pipeline.create(dai.node.VideoEncoder)

    rgbEncoded = pipeline.create(dai.node.XLinkOut)
    lEncoded = pipeline.create(dai.node.XLinkOut)
    rEncoded = pipeline.create(dai.node.XLinkOut)

    rgbEncoded.setStreamName("rgbEncoded")
    lEncoded.setStreamName("lEncoded")
    rEncoded.setStreamName("rEncoded")

    # Link nodes
    rgbCam.video.link(rgbEncoder.input)
    lCam.out.link(lEncoder.input)
    rCam.out.link(rEncoder.input)

    rgbEncoder.bitstream.link(rgbEncoded.input)
    lEncoder.bitstream.link(lEncoded.input)
    rEncoder.bitstream.link(rEncoded.input)

    # Config cameras
    rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
    lCam.setBoardSocket(dai.CameraBoardSocket.LEFT)
    rCam.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    rgbCam.setResolution(self.rgbResPreset)
    lCam.setResolution(self.monoResPreset)
    rCam.setResolution(self.monoResPreset)

    rgbCam.setFps(self.fps)
    lCam.setFps(self.fps)
    rCam.setFps(self.fps)

    # Config encoder
    rgbEncoder.setDefaultProfilePreset(rgbCam.getFps(), rgbProfilePreset)
    lEncoder.setDefaultProfilePreset(lCam.getFps(), monoProfilePreset)
    rEncoder.setDefaultProfilePreset(rgbCam.getFps(), monoProfilePreset)

    rgbEncoder.setQuality(quality)
    lEncoder.setQuality(quality)
    rEncoder.setQuality(quality)

    if rgbEncoder == "lossless":
      rgbEncoder.setLossless(True)
      lEncoder.setLossless(True)
      rEncoder.setLossless(True)

    if keyframeFrequency:
      rgbEncoder.setKeyframeFrequency(int(keyframeFrequency))
      lEncoder.setKeyframeFrequency(int(keyframeFrequency))
      rEncoder.setKeyframeFrequency(int(keyframeFrequency))

    self.__pipeline = pipeline
    self.__rgbCam = rgbCam

  def __writeFrame(self, encodedFile, timestampFile, frame: dai.ImgFrame):
    frame.getData().tofile(encodedFile)
    timestamp = self.__getFrameDatetime(frame)
    timestampFile.write(timestamp.astimezone().isoformat() + "\n")

  def record(
    self,
    outDirPath: Path,
    rgbProfile: str,
    monoProfile: str,
    quality: int,
    keyframeFrequency: int,
  ):
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
    lPath = subDirPath / f"{self.monoRes}.left.{monoExt}"
    rPath = subDirPath / f"{self.monoRes}.right.{monoExt}"

    # Timestamp output file
    rgbTimestampPath = subDirPath / "rgb.timestamp.txt"
    lTimestampPath = subDirPath / "left.timestamp.txt"
    rTimestampPath = subDirPath / "right.timestamp.txt"
    # Calibration data output
    calibrationPath = subDirPath / "calibration.json"

    print(f"Press Ctrl+C to stop recording...")
    rgbFrameCount: int = 0
    lFrameCount: int = 0
    rFrameCount: int = 0
    rgbFps: float = 0.0
    lFps: float = 0.0
    rFps: float = 0.0

    with (
      open(rgbPath, "wb") as rgbFile,
      open(lPath, "wb") as lFile,
      open(rPath, "wb") as rFile,
      open(rgbTimestampPath, "at") as rgbTimestampFile,
      open(lTimestampPath, "at") as lTimestampFile,
      open(rTimestampPath, "at") as rTimestampFile,
    ):
      with dai.Device(self.__pipeline) as device:
        # RGB needs fixed focus to properly align with depth
        calibration = device.readCalibration()
        lensPosition = calibration.getLensPosition(dai.CameraBoardSocket.RGB)
        self.__rgbCam.initialControl.setManualFocus(lensPosition)
        calibration.eepromToJsonFile(calibrationPath)

        # Output queues
        rgbEncodedQ = device.getOutputQueue(
          name="rgbEncoded", maxSize=self.fps * 3, blocking=True
        )
        lEncodedQ = device.getOutputQueue(
          name="lEncoded", maxSize=self.fps * 3, blocking=True
        )
        rEncodedQ = device.getOutputQueue(
          name="rEncoded", maxSize=self.fps * 3, blocking=True
        )

        # Recording
        firstRgbFrame = rgbEncodedQ.get()
        self.__writeFrame(rgbFile, rgbTimestampFile, firstRgbFrame)
        startTime = self.__getFrameDatetime(firstRgbFrame)
        rgbTimestampFile.write(startTime.astimezone().isoformat() + "\n")
        rgbFrameCount += 1

        while True:
          try:
            recordingTime = datetime.now() - startTime
            if recordingTime > self.time:
              break

            recordingSeconds = recordingTime.seconds
            if recordingSeconds:
              rgbFps: float = rgbFrameCount / recordingTime.seconds
              lFps: float = lFrameCount / recordingTime.seconds
              rFps: float = rFrameCount / recordingTime.seconds

            print(
              f"\rFPS: {rgbFps:.1f} L{lFps:.1f} R{rFps:.1f}, Time: {recordingTime}",
              end="",
            )

            while rgbEncodedQ.has():
              self.__writeFrame(rgbFile, rgbTimestampFile, rgbEncodedQ.get())
              rgbFrameCount += 1
            while lEncodedQ.has():
              self.__writeFrame(lFile, lTimestampFile, lEncodedQ.get())
              lFrameCount += 1
            while rEncodedQ.has():
              self.__writeFrame(rFile, rTimestampFile, rEncodedQ.get())
              rFrameCount += 1

          except KeyboardInterrupt:
            break

      print("\nStop recording...")

    print(f'Output files in "{outDirPath.absolute()}"')
