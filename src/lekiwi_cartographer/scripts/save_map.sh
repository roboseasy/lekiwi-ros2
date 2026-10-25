#!/usr/bin/env bash
# Cartographer 지도 저장: YAML · PGM(AMCL용)과 pbstream(Cartographer 상태)
# Cartographer와 같은 컴퓨터(Pi)에서 SLAM을 유지한 채 실행한다. pbstream은 cartographer_node가 자기 컴퓨터에 쓴다.
# 실패하면 지난 지도 기록을 바꾸지 않는다.
set -eo pipefail

usage() {
	echo '사용법: ros2 run lekiwi_cartographer save_map.sh'
	echo 'Cartographer와 같은 컴퓨터(Pi)에서 실행하세요. SLAM을 유지한 상태에서 YAML·PGM·pbstream을 저장합니다.'
	echo '저장 폴더: LEKIWI_MAP_DIR (기본 ~/maps/lekiwi, Nav2 기본 지도와 같은 폴더)'
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
	usage
	exit 0
fi
if (( $# )); then
	usage >&2
	exit 2
fi

LEKIWI_SAVE_STEP='ROS 환경 확인'
trap 'printf "지도 저장 중단(%s). SLAM을 유지하고 위 오류를 확인하세요.\n" "$LEKIWI_SAVE_STEP" >&2' ERR
# ros2 run으로 실행하면 ROS가 이미 source되어 있다. 설치 경로에서 직접 실행할 때를 위해 확인한다.
if [[ -z "${ROS_DISTRO:-}" ]] || ! command -v ros2 > /dev/null; then
	echo 'ROS 환경이 없습니다. /opt/ros/jazzy/setup.bash와 워크스페이스 install/setup.bash를 source한 뒤 다시 실행하세요.' >&2
	exit 1
fi
for package in nav2_map_server cartographer_ros_msgs; do
	if ! ros2 pkg prefix "$package" > /dev/null 2>&1; then
		printf '필요한 ROS 패키지가 없습니다: %s\n' "$package" >&2
		exit 1
	fi
done
printf 'ROS 준비: %s / domain %s\n' "$ROS_DISTRO" "${ROS_DOMAIN_ID:-0}"

LEKIWI_SAVE_STEP='지도 폴더 준비'
# 설정하지 않았을 때만 기본 폴더를 쓴다. 빈 값은 오류로 본다.
LEKIWI_MAP_DIR="${LEKIWI_MAP_DIR-${HOME:?HOME이 없습니다}/maps/lekiwi}"
if [[ -z "$LEKIWI_MAP_DIR" ]]; then
	echo '지도 폴더 경로(LEKIWI_MAP_DIR)가 비어 있습니다.' >&2
	exit 1
fi
# Cartographer 노드는 다른 폴더에서 돌므로 pbstream 경로는 절대 경로여야 한다.
LEKIWI_MAP_DIR="$(realpath -m -- "$LEKIWI_MAP_DIR")"
mkdir -p -- "$LEKIWI_MAP_DIR"
exec 9>"$LEKIWI_MAP_DIR/.save-map.lock"
if ! flock -n 9; then
	echo '다른 지도 저장이 실행 중입니다. 완료된 뒤 진행하세요.' >&2
	exit 1
fi

# 기존 지도는 덮어쓰지 않고 시각을 붙인 새 이름을 쓴다.
LEKIWI_MAP_BASE="$LEKIWI_MAP_DIR/map"
for extension in yaml pgm pbstream; do
	if [[ -e "$LEKIWI_MAP_BASE.$extension" || -L "$LEKIWI_MAP_BASE.$extension" ]]; then
		LEKIWI_MAP_BASE="$LEKIWI_MAP_DIR/map_$(date +%Y%m%d_%H%M%S_%N)"
		break
	fi
done
for extension in yaml pgm pbstream; do
	if [[ -e "$LEKIWI_MAP_BASE.$extension" || -L "$LEKIWI_MAP_BASE.$extension" ]]; then
		echo '선택한 지도 이름이 이미 사용 중입니다. 기존 파일을 보존하고 중단합니다.' >&2
		exit 1
	fi
done

LEKIWI_SAVE_STEP='YAML·PGM 저장'
ros2 run nav2_map_server map_saver_cli -f "$LEKIWI_MAP_BASE" \
	--ros-args -p save_map_timeout:=10.0

LEKIWI_SAVE_STEP='Cartographer 상태 저장'
# 경로에 따옴표가 있어도 깨지지 않도록 요청을 JSON으로 만든다.
request="$(/usr/bin/python3 - "$LEKIWI_MAP_BASE.pbstream" <<'PY'
import json
import sys
print(json.dumps({'filename': sys.argv[1], 'include_unfinished_submaps': True}))
PY
)"
echo 'Cartographer 상태 저장 응답을 기다립니다(최대 60초).'
status=0
response="$(timeout 60 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "$request" 2>&1)" || status=$?
printf '%s\n' "$response"
if (( status != 0 )); then
	echo 'Cartographer 상태 저장에 실패했습니다. SLAM과 /write_state 서비스를 확인하세요.' >&2
	exit "$status"
fi
# ros2 service call은 서비스가 오류를 돌려줘도 0으로 끝난다. StatusCode.OK(0)를 직접 확인한다.
if ! grep -Eq 'StatusResponse\(code=0,' <<< "$response"; then
	echo 'Cartographer 상태 저장 성공 응답을 확인하지 못했습니다.' >&2
	exit 1
fi

LEKIWI_SAVE_STEP='저장 파일 확인'
for extension in yaml pgm pbstream; do
	if [[ ! -s "$LEKIWI_MAP_BASE.$extension" ]]; then
		printf '지도 저장 중단: 파일이 없거나 비어 있습니다: %s\n' "$LEKIWI_MAP_BASE.$extension" >&2
		exit 1
	fi
done
ls -lh -- "$LEKIWI_MAP_BASE.yaml" "$LEKIWI_MAP_BASE.pgm" "$LEKIWI_MAP_BASE.pbstream"

# 모두 성공했을 때만 마지막 지도 기록을 한 번에 바꾼다.
record="$(mktemp "$LEKIWI_MAP_DIR/.last-map.XXXXXXXX")"
printf '%s.yaml\n' "${LEKIWI_MAP_BASE##*/}" > "$record"
mv -- "$record" "$LEKIWI_MAP_DIR/last_map.txt"
printf 'Nav2에서 사용할 지도: %s.yaml\n' "$LEKIWI_MAP_BASE"
printf '실행: ros2 launch lekiwi_navigation2 navigation.launch.py map:=%q\n' "$LEKIWI_MAP_BASE.yaml"
