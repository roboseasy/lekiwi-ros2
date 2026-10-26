"""워크스페이스 루트에서 pytest 를 돌려도 (colcon 설치 없이) 이 패키지를 import 하게 한다."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
