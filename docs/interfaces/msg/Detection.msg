# Detection.msg
# 단일 광물 detection 결과
# Producer: T2 (최진우) Perception
# Consumer: T3 (이찬휘) Mission Brain, T4 (성선규) UI

# 광물 종류 — meta.json의 minerals[].type과 일치
# 가능 값: "mineral_blue" | "mineral_red" | "mineral_yellow"
string class_name

# 월드 좌표계 기준 광물 위치 (m)
geometry_msgs/Point world_position

# Detection 신뢰도 (0.0 ~ 1.0)
# T2 (최진우)는 < 0.5 인 detection은 publish 안 함
float32 confidence

# 가치 점수 (T1 (김현중) meta.json의 value_distribution과 일치)
# blue=10, red=25, yellow=50
float32 value_score

# 광물 고유 ID (T1 (김현중) meta.json의 minerals[].id와 매칭)
# 같은 광물이 연속 프레임에서 검출되면 같은 ID
int32 mineral_id

# 월드 단위 bounding box 크기 (m)
geometry_msgs/Vector3 bbox_size_m
