from config import *
from pipeline import *

import cv2
import numpy as np

depthWeight = 100
rgbWeight = 0


def updateBlendWeights(depthPercent):
  global rgbWeight
  global depthWeight
  depthWeight = depthPercent
  rgbWeight = 100 - depthWeight


def preview():
  pipeline, rgbCam, maxDisparity = initPreviewPipeline(rgbRes, monoRes, fps)
  with dai.Device(pipeline) as device:
    # RGB needs fixed focus to properly align with depth
    calibrationbData = device.readCalibration()
    lensPosition = calibrationbData.getLensPosition(dai.CameraBoardSocket.RGB)
    rgbCam.initialControl.setManualFocus(lensPosition)

    rgbFrame = None
    disparityFrame = None

    # Trackbar adjusts blending ratio of rgb/depth
    windowName = "Preview"
    cv2.namedWindow(windowName)
    cv2.createTrackbar("Depth%", windowName, depthWeight, 100,
                       updateBlendWeights)

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
          disparityFrame = (disparityFrame * 255. / maxDisparity).astype(
              np.uint8)
          disparityFrame = cv2.applyColorMap(disparityFrame, cv2.COLORMAP_BONE)
          disparityFrame = np.ascontiguousarray(disparityFrame)

        # Blend when both received
        if (rgbFrame is not None) and (disparityFrame is not None):
          # Need to have both frames in BGR format before blending
          if len(disparityFrame.shape) < 3:
            disparityFrame = cv2.cvtColor(disparityFrame, cv2.COLOR_GRAY2BGR)
          blended = cv2.addWeighted(rgbFrame,
                                    float(rgbWeight) / 100, disparityFrame,
                                    float(depthWeight) / 100, 0)
          cv2.imshow(windowName, blended)
          rgbFrame = None
          disparityFrame = None

        if cv2.waitKey(1) == ord("q"):
          break
      except KeyboardInterrupt:
        break

    print("Stop preview...")
    cv2.destroyAllWindows()
