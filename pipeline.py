from config import *

import depthai as dai


def initPipeline(rgbRes, monoRes, fps: int = 30):
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

  # Link nodes
  rgbCam.video.link(rgbEncoder.input)
  leftCam.out.link(leftEncoder.input)
  rightCam.out.link(rightEncoder.input)

  rgbEncoder.bitstream.link(rgbH265.input)
  leftEncoder.bitstream.link(leftH265.input)
  rightEncoder.bitstream.link(rightH265.input)

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
  rgbEncoder.setDefaultProfilePreset(rgbCam.getFps(), encoderProfile)
  leftEncoder.setDefaultProfilePreset(leftCam.getFps(), encoderProfile)
  rightEncoder.setDefaultProfilePreset(rgbCam.getFps(), encoderProfile)

  rgbEncoder.setQuality(encoderQuality)
  leftEncoder.setQuality(encoderQuality)
  rightEncoder.setQuality(encoderQuality)

  return pipeline


def initPreviewPipeline(rgbRes, monoRes, fps: int = 30):
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

  rgbCam.setResolution(rgbRes)
  leftCam.setResolution(monoRes)
  rightCam.setResolution(monoRes)

  rgbCam.setFps(fps)
  leftCam.setFps(fps)
  rightCam.setFps(fps)

  # Config depth preview
  rgbOut.setStreamName("rgbOut")
  disparityOut.setStreamName("disparityOut")

  stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
  stereo.setLeftRightCheck(True)
  stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
  maxDisparity = stereo.initialConfig.getMaxDisparity()

  return pipeline, maxDisparity
