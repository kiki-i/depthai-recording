from recorder import *

import cv2
import numpy as np


class Previewer(Recorder):
  def __createPreviewWindow(self, windowName: str):
    self.__depthWeight: int = 50
    self.__rgbWeight: int = 0
    cv2.namedWindow(windowName)
    cv2.createTrackbar(
      "Depth %", windowName, self.__depthWeight, 100, self.__updateBlendWeights
    )

  def __initPreview(self):
    self.__depthWeight: int = 100
    self.__rgbWeight: int = 0

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

  def __showPreview(
    self,
    rgbFrame: cv2.Mat,
    disparityFrame: np.ndarray,
    scaleFactor: float = 1,
    windowsName: str = "",
  ):
    disparityFrame = (disparityFrame * 255.0 / self.__maxDisparity).astype(np.uint8)
    disparityFrame = cv2.applyColorMap(disparityFrame, cv2.COLORMAP_BONE)
    disparityFrame = np.ascontiguousarray(disparityFrame)

    if len(disparityFrame.shape) < 3:
      disparityFrame = cv2.cvtColor(disparityFrame, cv2.COLOR_GRAY2BGR)
    blended = cv2.addWeighted(
      rgbFrame,
      float(self.__rgbWeight) / 100,
      disparityFrame,
      float(self.__depthWeight) / 100,
      0,
    )

    scaledSize = (
      int(blended.shape[1] * scaleFactor),
      int(blended.shape[0] * scaleFactor),
    )
    scaled = cv2.resize(blended, scaledSize)

    cv2.imshow(windowsName, scaled)

  def __updateBlendWeights(self, depthPercent: int):
    self.__depthWeight: int = depthPercent
    self.__rgbWeight: int = 100 - self.__depthWeight

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
