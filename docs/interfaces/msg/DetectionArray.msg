# DetectionArray.msg
# 한 프레임에서 검출된 모든 광물 (publish 단위)
# Producer: T2 Perception
# Consumer: T3 Mission Brain, T4 UI
# Topic: /perception/detections
# Rate: 10 Hz
# 빈 배열이라도 발행 (T3가 "아무것도 못 봄" 상태 인지 가능)

std_msgs/Header header           # frame_id = "world", stamp = publish 시각
Detection[] detections
