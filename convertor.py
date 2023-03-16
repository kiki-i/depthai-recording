from concurrent.futures import Future, ThreadPoolExecutor, as_completed
from multiprocessing import cpu_count
from pathlib import Path

import re
import subprocess


def checkCliExist(path: str) -> bool:
  try:
    subprocess.run(path, capture_output=True)
  except FileNotFoundError:
    return False
  else:
    return True


def clenCliLine(override: str = "", end: str = "\n", width: int = 90):
  print("\r" + " " * width, end="")
  if override:
    print("\r" + override, end=end)


def convertAll(processPaths: dict[Path, Path]):
  print("Converting to mp4...")

  if not checkCliExist("ffmpeg"):
    print("Can't find ffmpeg!")
  else:
    convertErrors = multiprocessFiles(processPaths, convertToMp4)
    if convertErrors:
      print("!Error: Complete conversion with error!\n")
      for file, error in convertErrors.items():
        print(f"!Error: {file}: {error}")
    else:
      print("Complete conversion!")


def convertToMp4(inputPath: Path, outputPath: Path) -> str:
  error = ""
  fps = int(re.search(r"\[(\d+)FPS\]", str(inputPath.name)).group(1))
  result = subprocess.run(
      f"ffmpeg -framerate {fps} -i {inputPath} -c copy -y {outputPath}",
      capture_output=True,
      text=True)
  if result.returncode != 0:
    error = f"{result.stderr}\n{result.stdout}"
  return error


def multiprocessFiles(processPaths: dict[Path, Path], fun) -> dict[Path, str]:
  threadLimit = int(cpu_count() / 2)
  errorList: dict[Path, str] = {}

  # Init thread
  total: int = len(processPaths)
  threadResult: dict[Future, Path] = {} # Future: inputPath
  with ThreadPoolExecutor(max_workers=threadLimit) as threadPool:

    # Run subprocess
    for inputPath, outputPath in processPaths.items():
      try:
        future = threadPool.submit(fun, inputPath, outputPath)
        threadResult[future] = inputPath
      except Exception as e:
        errorList[inputPath] = str(e)

    # Wait all subprocess
    current: int = 0
    clenCliLine(f"Processing {current}/{total}...", end="")
    for future in as_completed(threadResult):
      current = current + 1
      clenCliLine(f"Processing {current}/{total}...", end="")

      inputPath = threadResult[future]
      error = future.result()
      if error:
        errorList[inputPath] = error.strip()

    clenCliLine(f"Processing {current}/{total}.")
  return errorList
