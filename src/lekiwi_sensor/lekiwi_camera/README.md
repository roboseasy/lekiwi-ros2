# lekiwi_camera

LeKiwi **전방 카메라와 손목 카메라를 V4L2로 직접 읽어, 카메라가 보낸 MJPEG 프레임을 `sensor_msgs/CompressedImage`로 발행**하는 노드 패키지.
카메라마다 노드를 하나씩 띄운다. 기본 설정에서는 JPEG를 풀지 않고 그대로 보내므로 Pi CPU를 거의 쓰지 않는다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · USB 카메라 2대 (MJPEG 640×480 @ 30 fps)
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_sensor/lekiwi_camera`

> ⚠️ 같은 카메라를 쓰는 LeRobot 호스트(`start_lekiwi_host.sh`)나 LeKiwi `lekiwi_sensors/cameras.launch.py`(usb_cam)와 **동시에 실행하지 않는다.**

---

## 하드웨어 연결

카메라 2대 → Raspberry Pi USB 포트. 두 카메라는 같은 기종이라 시리얼 번호도 같다.
그래서 `/dev/v4l/by-id` 링크가 하나만 생기므로 **USB 포트 위치로 정해지는 `/dev/v4l/by-path` 링크**를 쓴다.

| 카메라 | Pi USB 포트 | 장치 (lekiwi04) | 노드 |
|--------|:-----------:|-----------------|------|
| 전방 | 1.2 | `...usb-0:1.2:1.0-video-index0` → `/dev/video0` | `front_camera_node` |
| 손목 | 1.1 | `...usb-0:1.1:1.0-video-index0` → `/dev/video2` | `wrist_camera_node` |

```bash
ls -l /dev/v4l/by-path/        # USB 카메라마다 video-index0(영상), video-index1(메타데이터)
v4l2-ctl --list-devices        # 어떤 USB 포트에 어떤 /dev/videoN이 있는지
```

> 전방 = `/dev/video0`, 손목 = `/dev/video2`는 LeRobot LeKiwi 설정과 같은 대응이다. 실제 위치는 영상을 보고 확인한다.
> 카메라를 다른 USB 포트에 꽂으면 by-path 경로가 바뀌므로 `config/lekiwi_camera.yaml`의 `device`를 고친다.

---

## 노드

- `front_camera_node`, `wrist_camera_node` (실행 파일은 둘 다 `lekiwi_camera_node`)

## 구독 (Subscribe)

- 없음 — V4L2(`/dev/videoN`)에서 직접 읽는다.

## 발행 (Publish)

| 토픽 | 타입 | 주기 | 내용 |
|------|------|------|------|
| `/lekiwi/sensor/front_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 약 30 Hz | 전방 카메라 JPEG |
| `/lekiwi/sensor/front_camera/camera_info` | `sensor_msgs/CameraInfo` | 영상과 같이 | 크기 · frame_id (보정 전) |
| `/lekiwi/sensor/wrist_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 약 30 Hz | 손목 카메라 JPEG |
| `/lekiwi/sensor/wrist_camera/camera_info` | `sensor_msgs/CameraInfo` | 영상과 같이 | 크기 · frame_id (보정 전) |

- `frame_id`: `front_camera_optical_frame`, `wrist_camera_optical_frame` (LeKiwi 모델의 카메라 프레임 이름)
- `header.stamp`: 카메라 드라이버가 기록한 촬영 시각
- 토픽 이름은 `image_transport` 규칙(`<이름>/compressed`)을 따른다. 그래서 `rqt_image_view` 등에서 `image_raw`의 compressed 방식으로 보인다.
- QoS: reliable (depth 2). `rqt_image_view`와 RViz 기본 설정으로 받을 수 있다.
- `camera_info`는 보정 전이라 `K`가 모두 0이다.

## 파라미터

설정 파일: `config/lekiwi_camera.yaml` (노드 이름별로 `front_camera_node:`, `wrist_camera_node:` 아래에 있다)

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `device` | string | by-path 경로 | 카메라 장치. `video-index0`만 영상 장치다 |
| `width` / `height` | int | `640` / `480` | 카메라에 요청하는 MJPEG 크기 |
| `fps` | int | `30` | 카메라에 요청하는 fps |
| `rotation` | int | `0` | 시계 방향 회전 [deg]: 0, 90, 180, 270. LeRobot 기본값은 전방 180, 손목 90 |
| `jpeg_quality` | int | `80` | `rotation`을 쓸 때 다시 압축하는 품질 (1~100) |
| `frame_id` | string | `front_camera_optical_frame` 등 | 메시지 frame |
| `topic_prefix` | string | `lekiwi/sensor/front_camera` 등 | 토픽 앞부분 |
| `frame_timeout_ms` | int | `1000` | 이 시간 안에 프레임이 없으면 실패로 센다 |
| `reopen_after_failures` | int | `3` | 연속 실패가 이만큼 쌓일 때마다 장치를 다시 연다 |

> `rotation`이 0이 아니면 Pi에서 JPEG를 풀고 돌려 다시 압축한다. 카메라 한 대 30 fps 기준 CPU를 꽤 쓰므로 필요할 때만 켠다.

---

## 빌드

```bash
cd ~/lekiwi_ws
cbs lekiwi_camera && sb
```

> ⚠️ lerobot venv가 켜진 셸에서 `colcon build`를 직접 실행하면 `No module named 'catkin_pkg'`로 실패한다.
> 먼저 `deactivate` 하거나 `cb` / `cbs` / `cbsource`를 사용한다.

`config/` · `launch/` 파일은 `--symlink-install`로 연결되므로 수정 후 노드만 재시작하면 적용된다.

---

## 실행

```bash
ros2 launch lekiwi_camera lekiwi_camera.launch.py
```

카메라마다 이런 로그가 나온다. `[front_camera_node]`, `[wrist_camera_node]`로 구분된다.

```text
camera ready on /dev/v4l/by-path/...-usb-0:1.2:1.0-video-index0 (/dev/video0): MJPEG 640x480 @ 30.0 fps, rotation 0 deg, frame=front_camera_optical_frame
publishing /lekiwi/sensor/front_camera/image_raw/compressed, /lekiwi/sensor/front_camera/camera_info
first frame: <카메라 JPEG 크기> bytes from camera, <발행 크기> bytes published, 640x480
```

종료는 `Ctrl+C`.

**launch 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `params_file` | `share/lekiwi_camera/config/lekiwi_camera.yaml` | 파라미터 파일 |
| `use_front` | `true` | 전방 카메라 실행 |
| `use_wrist` | `true` | 손목 카메라 실행 |

```bash
# 예: 전방 카메라만
ros2 launch lekiwi_camera lekiwi_camera.launch.py use_wrist:=false
```

---

## 확인 명령어

노드를 켠 상태에서 **다른 터미널**에서 실행한다.

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic list` (`rt`) | `/lekiwi/sensor/front_camera/...`, `/lekiwi/sensor/wrist_camera/...`가 보이는지 |
| `ros2 topic hz /lekiwi/sensor/front_camera/image_raw/compressed` | 약 30 Hz인지 |
| `ros2 topic bw /lekiwi/sensor/front_camera/image_raw/compressed` | 초당 전송량 (Wi-Fi로 PC에 보낼 때 참고) |
| `ros2 topic echo --once /lekiwi/sensor/front_camera/camera_info` | 크기 · frame_id |

---

## PC에서 영상 보기

1. **한 번만**: PC에 압축 영상 플러그인을 설치한다. 없으면 `rqt_image_view`에 compressed 토픽이 보이지 않는다.
   ```bash
   sudo apt install ros-jazzy-compressed-image-transport
   ```
2. PC 터미널에서 Pi와 통신 설정을 맞춘다.
   ```bash
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=42
   export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
   ```
3. `ros2 run rqt_image_view rqt_image_view`
4. 위쪽 목록에서 `/lekiwi/sensor/front_camera/image_raw/compressed`를 고른다.

**전방 · 손목 대응 확인**: 전방 카메라 렌즈를 손으로 가려 `front_camera` 영상이 어두워지는지 본다.
**방향 확인**: 영상이 뒤집혀 있으면 `config/lekiwi_camera.yaml`의 `rotation`을 바꾸고 노드를 재시작한다.

---

## 처리 흐름

```text
USB 카메라 (V4L2, MJPEG 640x480 @ 30 fps, mmap 버퍼 4개)
  └─ 프레임 꺼내기 (쌓인 프레임 중 가장 최근 것만, JPEG 시작 표시 FF D8 확인)
      ├─ rotation 0  : 카메라 JPEG 그대로 ──────────────┐
      ├─ rotation ≠ 0: 디코딩 → 회전 → JPEG 재압축 ─────┤
      │                                                 └→ <topic_prefix>/image_raw/compressed
      └─ 크기 · frame_id ───────────────────────────────→ <topic_prefix>/camera_info
```

## 알려진 한계

- **카메라 보정 전**이다. `camera_info`의 `K`가 0이고, 모델의 카메라 위치(`camera_mounts.json`)도 `calibrated: false`다.
- 압축하지 않은 `sensor_msgs/Image`는 발행하지 않는다. Pi 안에서 원본 영상이 필요한 노드가 생기면 그때 추가한다.
- 노출 · 밝기 같은 카메라 설정은 카메라 자동 설정을 그대로 쓴다.
- `rotation`을 켜면 Pi CPU 사용이 크게 늘어난다.
- 전방 · 손목 대응은 lekiwi04에서 확인했다(2026-10-18). 다른 로봇은 카메라를 꽂은 USB 포트가 다를 수 있으므로 영상으로 다시 확인한다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `Package 'lekiwi_camera' not found` | 워크스페이스를 source하지 않음. `sb` |
| `failed to open ...: No such file or directory` | 카메라 USB 연결 · 포트 확인. `ls -l /dev/v4l/by-path/` 결과로 `device`를 고친다 |
| `failed to open ...: Permission denied` | 사용자에게 `video` 그룹이 필요. `id`로 확인 |
| `failed to open ...: Device or resource busy` 또는 `VIDIOC_STREAMON failed` | 다른 프로그램이 카메라를 쓰는 중. `fuser -v /dev/video0 /dev/video2` |
| `is not a video capture device` | `video-index1`(메타데이터)을 지정함. `video-index0`을 쓴다 |
| `does not support MJPEG ...` | 그 크기의 MJPEG를 지원하지 않음. `v4l2-ctl -d /dev/video0 --list-formats-ext`로 확인 |
| `no frame from ... within ... ms` 반복 | USB 접촉 · 전원 부족. 3회마다 자동으로 다시 연다 |
| PC `rqt_image_view`에 토픽이 없음 | PC에 `ros-jazzy-compressed-image-transport` 설치, 통신 설정(42 / SUBNET) 확인 |
| 영상이 끊기거나 늦음 | Wi-Fi 대역폭 부족. 한 대만 보거나 `fps`를 낮춘다 |

---

## 패키지 구조

```text
lekiwi_camera/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/lekiwi_camera.yaml          # 전방 · 손목 노드 파라미터
├── launch/lekiwi_camera.launch.py     # 두 노드 실행
├── include/lekiwi_camera/
│   ├── v4l2_capture.hpp               # V4L2 MJPEG 촬영 드라이버
│   └── lekiwi_camera_node.hpp         # ROS 2 노드
└── src/
    ├── v4l2_capture.cpp
    └── lekiwi_camera_node.cpp          # main 포함
```

## 참고한 코드

- **Linux V4L2 API (커널 문서 "Video for Linux API" · Streaming I/O, memory mapping)**: 장치를 직접 열어 mmap 버퍼로 프레임을 읽는 방식
- **LeKiwi 저장소 `lekiwi_sensors/launch/cameras.launch.py`**: 검증된 640×480 MJPEG 30 fps 설정, 카메라 두 대를 따로 켜고 끄는 구성
- **LeKiwi 저장소 `lekiwi_description/config/camera_mounts.json`**: 카메라 frame 이름, `calibrated: false`
- **LeRobot `robots/lekiwi/config_lekiwi.py`**: 전방 `/dev/video0` · 손목 `/dev/video2`, 회전 기본값
- **lekiwi_imu · lekiwi_lidar**: 패키지 구조, 파라미터 선언 · 검증, 로그 형식

## 검증 기록

- 2026-10-17, lekiwi04 (Pi 4, Jazzy): 두 카메라 모두 MJPEG 640×480 @ 30 fps 지원 확인 (`v4l2-ctl --list-formats-ext`).
  임시 폴더 빌드 성공, 빌드 경고 0. 없는 장치와 `video-index1`을 지정하면 이유를 출력하고 종료됨
- 2026-10-18, lekiwi04: `lekiwi_camera.launch.py`로 두 노드 실행, PC `rqt_image_view`에서 두 영상 표시 확인 (PC에 `ros-jazzy-compressed-image-transport` 설치).
  `front_camera`(USB 1.2, `/dev/video0`)에 정면 벽 · 바닥, `wrist_camera`(USB 1.1, `/dev/video2`)에 그리퍼 · 바닥이 보여 전방 · 손목 대응이 맞다.
  전방 영상은 `rotation: 0`에서 바른 방향이다. 화면 예시는 [docs/camera.md](../../docs/camera.md)
- 발행 주기(30 Hz), 브링업 `use_camera:=true` 경로, `rotation` 사용 시 CPU 사용량은 아직 기록하지 않았다.
