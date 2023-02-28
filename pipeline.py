import depthai as dai

# Config
## Conifg encoder
encoderProfile = dai.VideoEncoderProperties.Profile.H265_MAIN
encoderQuality = 100
## Config disparity preview
lrCheck = True
extendedDisparity = False
subpixel = False


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
  rgbEncoder.setDefaultProfilePreset(rgbCam.getFps(), encoderProfile)
  leftEncoder.setDefaultProfilePreset(leftCam.getFps(), encoderProfile)
  rightEncoder.setDefaultProfilePreset(rgbCam.getFps(), encoderProfile)

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

  return pipeline, disparityMultiplier
