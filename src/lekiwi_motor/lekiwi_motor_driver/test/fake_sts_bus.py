"""가상 Feetech 버스: PTY(가짜 시리얼 장치) 위에서 STS3215 9개(팔 1~6, 바퀴 7~9)를 흉내 낸다.

READ에 레지스터 값으로 응답하고, WRITE에 ACK하며, SYNC_WRITE를 적용한다. 노드가 보낸 쓰기를 모두 기록한다.
fail_next_writes로 정한 WRITE는 기록만 하고 적용 · 응답하지 않는다 (ACK 없음).
reply_delay를 주면 응답마다 그만큼 늦게 답한다 (USB 왕복 · 모터 응답 지연 흉내). READ는 받은 시각과 함께 read_log에 남는다.
바퀴는 토크가 켜지고 속도 모드이면 Goal_Speed만큼 위치가 움직인다. 팔은 움직이지 않는다 (위치는 시험이 정한다).
실제 장치를 열지 않는다.

시험용이다. lekiwi_motor_driver 시험과, 설치 경로(share/lekiwi_motor_driver/test_support)를 거쳐
다른 패키지의 통합 시험이 함께 쓴다.
"""

import json
import os
import select
import sys
import threading
import time
import tty

ARM_IDS = [1, 2, 3, 4, 5, 6]
WHEEL_IDS = [7, 8, 9]
TICKS_PER_RAD = 4096.0 / (2.0 * 3.141592653589793)

ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11
ADDR_P_COEFFICIENT = 21
ADDR_D_COEFFICIENT = 22
ADDR_I_COEFFICIENT = 23
ADDR_HOMING_OFFSET = 31
ADDR_MODE = 33
ADDR_TORQUE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_POSITION = 56
ADDR_SPEED = 58
ADDR_STATUS = 65
STATUS_OVERHEAT = 4
STATUS_OVERLOAD = 32


def decode_sign_magnitude(word):
	magnitude = word & 0x7FFF
	return -magnitude if word & 0x8000 else magnitude


def encode_sign_magnitude(value):
	magnitude = min(abs(value), 0x7FFF)
	return magnitude | 0x8000 if value < 0 else magnitude


class FakeStsBus:
	"""PTY 위의 STS3215 9개. READ에 응답하고 WRITE에 ACK하며 SYNC_WRITE를 적용한다."""

	def __init__(self, reply_delay=0.0):
		self.__reply_delay = reply_delay   # [s] 응답마다
		self.__master, self.__slave = os.openpty()
		tty.setraw(self.__slave)
		self.port = os.ttyname(self.__slave)
		self.__lock = threading.Lock()
		self.__registers = {}
		self.__responsive = {}
		self.__failing_writes = {}   # (id, address) → 남은 ACK 없는 WRITE 수
		self.writes = []   # (instruction, id, address, bytes) 보낸 순서대로
		self.read_log = []   # (받은 시각 time.monotonic(), id, address) READ 순서대로
		for motor_id in ARM_IDS + WHEEL_IDS:
			registers = bytearray(256)
			registers[3:5] = (777).to_bytes(2, 'little')
			registers[ADDR_MODE] = 1 if motor_id in WHEEL_IDS else 0
			registers[ADDR_POSITION:ADDR_POSITION + 2] = (2048).to_bytes(2, 'little')
			registers[62] = 121   # 12.1 V
			registers[63] = 30    # 30 °C
			self.__registers[motor_id] = registers
			self.__responsive[motor_id] = True
		self.__positions = {motor_id: 2048.0 for motor_id in WHEEL_IDS}
		self.__is_running = True
		self.__thread = threading.Thread(target=self.__run, daemon=True)
		self.__thread.start()

	def close(self):
		self.__is_running = False
		self.__thread.join(timeout=2.0)
		os.close(self.__master)
		os.close(self.__slave)

	# 시험에서 상태를 바꾸는 함수
	def set_responsive(self, motor_id, is_responsive):
		with self.__lock:
			self.__responsive[motor_id] = is_responsive

	def fail_next_writes(self, motor_id, address, count):
		"""motor_id의 address로 오는 다음 WRITE count개를 적용하지 않고 응답도 하지 않는다."""
		with self.__lock:
			self.__failing_writes[(motor_id, address)] = count

	def set_register(self, motor_id, address, value):
		with self.__lock:
			self.__registers[motor_id][address] = value

	def register(self, motor_id, address):
		with self.__lock:
			return self.__registers[motor_id][address]

	def set_word(self, motor_id, address, value):
		with self.__lock:
			self.__registers[motor_id][address:address + 2] = value.to_bytes(2, 'little')

	def goal_position(self, motor_id):
		with self.__lock:
			registers = self.__registers[motor_id]
			return registers[ADDR_GOAL_POSITION] | registers[ADDR_GOAL_POSITION + 1] << 8

	def goal_speed(self, motor_id):
		with self.__lock:
			registers = self.__registers[motor_id]
			return decode_sign_magnitude(registers[ADDR_GOAL_SPEED] | registers[ADDR_GOAL_SPEED + 1] << 8)

	def written_ids(self):
		with self.__lock:
			ids = set()
			for instruction, motor_id, _, data in self.writes:
				if instruction == 0x83:
					ids.update(entry_id for entry_id, _ in data)
				else:
					ids.add(motor_id)
			return ids

	# 가상 버스 동작
	def __run(self):
		buffer = bytearray()
		last = time.monotonic()
		while self.__is_running:
			ready, _, _ = select.select([self.__master], [], [], 0.002)
			now = time.monotonic()
			self.__step(now - last)
			last = now
			if not ready:
				continue
			try:
				buffer += os.read(self.__master, 4096)
			except OSError:
				continue
			while True:
				start = buffer.find(b'\xff\xff')
				if start < 0 or len(buffer) < start + 4:
					break
				del buffer[:start]
				size = buffer[3] + 4
				if len(buffer) < size:
					break
				packet = bytes(buffer[:size])
				del buffer[:size]
				if sum(packet[2:]) & 0xFF == 0xFF:
					self.__handle(packet)

	def __step(self, dt):
		with self.__lock:
			for motor_id in WHEEL_IDS:
				registers = self.__registers[motor_id]
				speed = 0
				if registers[ADDR_TORQUE] and registers[ADDR_MODE] == 1:
					speed = decode_sign_magnitude(registers[ADDR_GOAL_SPEED] | registers[ADDR_GOAL_SPEED + 1] << 8)
				self.__positions[motor_id] = (self.__positions[motor_id] + speed * dt) % 4096
				registers[ADDR_POSITION:ADDR_POSITION + 2] = int(self.__positions[motor_id]).to_bytes(2, 'little')
				registers[ADDR_SPEED:ADDR_SPEED + 2] = encode_sign_magnitude(speed).to_bytes(2, 'little')

	def __handle(self, packet):
		motor_id, instruction = packet[2], packet[4]
		with self.__lock:
			if instruction == 0x83:   # SYNC_WRITE: 응답 없음
				address, length = packet[5], packet[6]
				entries = []
				for offset in range(7, len(packet) - 1, length + 1):
					entry_id = packet[offset]
					data = packet[offset + 1:offset + 1 + length]
					entries.append((entry_id, bytes(data)))
					if entry_id in self.__registers and self.__responsive[entry_id]:
						self.__registers[entry_id][address:address + length] = data
				self.writes.append((instruction, 0xFE, address, entries))
				return
			if motor_id not in self.__registers or not self.__responsive[motor_id]:
				return
			if instruction == 0x02:   # READ
				address, length = packet[5], packet[6]
				self.read_log.append((time.monotonic(), motor_id, address))
				self.__reply(motor_id, self.__registers[motor_id][address:address + length])
			elif instruction == 0x03:   # WRITE
				address, data = packet[5], packet[6:-1]
				self.writes.append((instruction, motor_id, address, bytes(data)))
				if self.__failing_writes.get((motor_id, address), 0) > 0:
					self.__failing_writes[(motor_id, address)] -= 1
					return
				self.__registers[motor_id][address:address + len(data)] = data
				self.__reply(motor_id, b'')

	def __reply(self, motor_id, data):
		if self.__reply_delay:
			time.sleep(self.__reply_delay)
		body = bytes([motor_id, len(data) + 2, 0]) + bytes(data)
		os.write(self.__master, b'\xff\xff' + body + bytes([~sum(body) & 0xFF]))


if __name__ == '__main__':
	# 버스만 따로 프로세스로 띄운다: 시험 프로세스의 rclpy 스레드와 GIL을 나누지 않아 응답 시각이 흔들리지 않는다.
	# 인자: 응답 지연 [s]. 포트를 한 줄 쓰고, 표준 입력이 닫히면 read_log · writes(데이터 없이)를 JSON 한 줄로 쓰고 끝낸다.
	bus = FakeStsBus(reply_delay=float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
	print(bus.port, flush=True)
	sys.stdin.read()
	bus.close()
	print(json.dumps({
		'reads': bus.read_log,
		'writes': [(instruction, motor_id, address) for instruction, motor_id, address, _ in bus.writes],
	}), flush=True)
