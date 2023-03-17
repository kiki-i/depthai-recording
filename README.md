# depthai-recording

```
Record RGB and stereo video with DepthAI OAK-D

options:
  -h, --help          show this help message and exit
  -r, --resolution    RGB (and Mono) resolution: <12mp|4k|1080p>[:<800p|720p|400p>], default = "1080p:800p"
  -f, --fps           Specify FPS, default = "30"
  -p, --preview       Show preview, nothing will be recorded
  -e, --encoder       Encoder for RGB and Mono: <lossless|mjpeg|h265>[,<lossless|mjpeg|h265>], default = "lossless"
  -q, --quality       Encoding quality between 0-100, default = "100"
  -k, --keyframe      Keyframe frequency, 0 means auto, default = "0"
  -o, --out           Output directory path, default = "output"
```

## Dependencies

* depthai
* numpy
* opencv-python

## License

[![AGPLv3](https://www.gnu.org/graphics/agplv3-155x51.png)](https://www.gnu.org/licenses/agpl-3.0.html)

Licensed under the [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.html).
