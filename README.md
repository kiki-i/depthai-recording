# depthai-video-recording

```
Record RGB and stereo video with DepthAI OAK-D

options:
  -h, --help     Show this help message and exit
  -p, --preview  Show preview, nothing will be recorded
  -m, --mp4      Convert the raw H.265 file to MP4 after recording (Require ffmpeg be installed)
  -o, --out      Specify output directory path, default value is "output"
```

## Requirements

* ffmpeg (Required if specify option `-m`)

## Dependencies

* depthai
* numpy
* opencv-python

## License

[![AGPLv3](https://www.gnu.org/graphics/agplv3-155x51.png)](https://www.gnu.org/licenses/agpl-3.0.html)

Licensed under the [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.html).
