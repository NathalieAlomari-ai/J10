"""``j10-fetch-model``: downloads the default detection model into ``models/``.

Model weights are deliberately not in the repo (see the top-level ``.gitignore``), so this
is the one-time step that puts them where ``CVNodeConfig``'s defaults look. Standard
library only — it has to work on a freshly imaged Pi before anything else is installed.
"""

from __future__ import annotations

import argparse
import io
import sys
import urllib.request
import zipfile
from pathlib import Path

# Quantized SSD-MobileNet-v1, COCO classes, 300x300 input — TensorFlow's own hosted
# "starter" detection model. The zip holds exactly detect.tflite and labelmap.txt.
MODEL_URL = (
    "https://storage.googleapis.com/download.tensorflow.org/models/tflite/"
    "coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip"
)
_FILES = ("detect.tflite", "labelmap.txt")
_DEFAULT_DEST = Path(__file__).resolve().parent.parent / "models"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dest", type=Path, default=_DEFAULT_DEST, help="default: %(default)s")
    parser.add_argument("--force", action="store_true", help="re-download even if present")
    args = parser.parse_args()

    if not args.force and all((args.dest / name).is_file() for name in _FILES):
        print(f"model already present in {args.dest}")
        return 0

    print(f"downloading {MODEL_URL}")
    with urllib.request.urlopen(MODEL_URL, timeout=60) as response:
        archive = zipfile.ZipFile(io.BytesIO(response.read()))

    args.dest.mkdir(parents=True, exist_ok=True)
    for name in _FILES:
        (args.dest / name).write_bytes(archive.read(name))
        print(f"wrote {args.dest / name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
