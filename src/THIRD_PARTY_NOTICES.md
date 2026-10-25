# 서드파티 자료

이 워크스페이스의 직접 작성한 ROS 2 코드는 Apache-2.0이다 (각 `package.xml`).
아래 자료는 출처와 조건을 따로 가진다. LeKiwi 저장소(`feature/lekiwi-ros2`)의 `THIRD_PARTY_NOTICES.md`에서 옮겨 왔다.

| 자료 | 출처 | 여기서 쓰는 곳 | 라이선스 · 허락 기록 |
|------|------|----------------|---------------------|
| LeKiwi 몸체 · SO101 모델 메시와 Xacro | `roboseasy-members/lekiwi_isaacsim`과 `lekiwi_description/docs/IMPORT.md` · `SOURCE.md`에 기록한 원본 워크스페이스 | `lekiwi_description/meshes`, `urdf`, 카메라 장착값 | 회사 관리 모델. 2026-09-30 제품 책임자가 재배포를 확인 |
| Cartographer · Nav2 설정 | `yeon-03/lekiwi-pill-pickup`, commit `9f4a00cfd9dd15e31e73f96825eb86193ef2a20a` (LeKiwi 저장소를 거쳐 옮김) | `lekiwi_cartographer/config`, `lekiwi_navigation2/config` | 2026-09-30 제품 책임자가 재배포 허락을 확인. 원본 스냅샷에는 라이선스 파일이 없었다 |
| lerobot 0.6.0 LeKiwi 바퀴 → 몸체 속도 공식 (`LeKiwi._wheel_raw_to_body`) | Hugging Face `huggingface/lerobot`, `src/lerobot/robots/lekiwi/lekiwi.py` | `lekiwi_lerobot/lekiwi_lerobot/follower_logic.py` `wheel_raw_to_body` (같은 상수 · 계산 순서로 옮겨 적음. lerobot 함수와 같은지 단위 시험으로 본다) | Apache-2.0 |
| YDLIDAR SDK | YDLIDAR | `lekiwi_lidar`가 `/usr/local`에 설치된 SDK를 링크한다 (이 저장소에 포함하지 않음) | 각 업스트림 저장소의 라이선스 |

모델이나 설정을 담은 압축 파일 · 이미지를 외부에 배포하기 전에, 배포 책임자가 실제 허락 기록을 보관하고 함께 나가는 고지가 그 허락과 맞는지 확인한다.
