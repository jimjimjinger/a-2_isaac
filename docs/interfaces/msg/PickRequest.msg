# PickRequest.msg
# T3 FSM이 PICK phase 진입 시 T2 매니퓰레이션에 발행
# Producer: T3 Mission Brain
# Consumer: T2 M0609 Manipulator
# Topic: /mission/pick_request
# Rate: event-based (1회/광물)

std_msgs/Header header

# 어떤 광물을 잡을지 — T2 detection의 mineral_id 또는 T1 meta의 id
int32 mineral_id

# 잡을 위치 (월드 좌표, m)
# T3가 detection 받은 좌표를 echo
geometry_msgs/Point target_position

# Pick 작업 최대 허용 시간 (초)
# 초과 시 T2는 PickResponse로 status="timeout" 응답
# 기본값: 30.0
float32 timeout_s

# 요청 unique ID (T3가 1부터 발급, 중복 방지)
# T2는 같은 request_id를 두 번 받으면 두 번째 무시
int32 request_id
