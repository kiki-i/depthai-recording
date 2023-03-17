# depthai-recording

```
Record RGB and stereo video with DepthAI OAK-D

options:
  -h, --help       show this help message and exit
  -p, --preview    Show preview, nothing will be recorded
  -c, --codec      Available codecs: ["lossless", "mjpeg", "h265"], default = "lossless"
  -f, --fps        Specify FPS, default = "30"
  -q, --quality    Encoding quality between 0-100, default = "100"
  -o, --out        Output directory path, default = "output"
  --rgbres         RGB camera resolution: ["12mp", "4k", "1080p"], default = "1080p"
  --monores        Mono camera resolution: ["800p", "720p", "400p"], default = "800p"
```

## Dependencies

* depthai
* numpy
* opencv-python

## License

[![AGPLv3](https://www.gnu.org/graphics/agplv3-155x51.png)](https://www.gnu.org/licenses/agpl-3.0.html)

Licensed under the [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.html).
