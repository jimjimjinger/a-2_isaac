# PickResponse.msg
# T2가 PickRequest 처리 완료 후 T3에 응답
# Producer: T2 M0609 Manipulator
# Consumer: T3 Mission Brain
# Topic: /mission/pick_response
# Rate: event-based (PickRequest 1개당 1개)
# Latency: timeout_s 안에 반드시 응답

std_msgs/Header header

# 어떤 광물에 대한 응답인지
int32 mineral_id

# 어떤 요청에 대한 응답인지 (PickRequest.request_id echo)
int32 request_id

# 결과 상태
# "success"      : 정상 수집 완료 → cargo bin에 적재됨
# "failed_grasp" : 접근했으나 grasp 동작 실패
# "timeout"      : timeout_s 초과
# "no_object"    : target_position에 광물 없음 (invalid detection)
string status

# 실제 소요 시간 (초) — 디버깅/통계용
float32 duration_s

# 실패 사유 (status != "success" 일 때만, free-form)
string failure_reason
