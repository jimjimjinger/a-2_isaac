# T2 (최진우) Perception + M0609 — Claude Code Context

> 이 파일은 Claude Code가 자동 로드하는 트랙 컨텍스트입니다.

## 너의 정체성
**T2 (최진우) 트랙 owner — 광물 vision detection + M0609 매니퓰레이션 (Tier 1.5)**

GPU: 5080 (16GB)

## 작업 시작 전 필독
1. [T2_BRIEF.md](T2_BRIEF.md) — 전체 onboarding
2. [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) — I2, I3, I4 섹션
3. [interfaces/msg/Detection.msg](../interfaces/msg/Detection.msg)
4. [interfaces/msg/PickRequest.msg](../interfaces/msg/PickRequest.msg)
5. [interfaces/msg/PickResponse.msg](../interfaces/msg/PickResponse.msg)

## 내가 만드는 2개 큰 모듈

```
1. Vision (광물 인식)
   - HSV 색기반 detection (CNN 아님)
   - 3색: blue(10pt) / red(25pt) / yellow(50pt)
   - publish /perception/detections @10Hz
   
2. M0609 Manipulation (Tier 1.5)
   - Scripted joint trajectory (실제 IK 아님)
   - 광물 → cargo bin 텔레포트
   - subscribe /mission/pick_request → publish /mission/pick_response
```

## 핵심 의존성
- **T5 (이지민)의 estimated_pose 사용** (2D→3D 좌표 변환에 필요, GT 아님)
- **T1 (김현중) meta.json 사용** (광물 id 매칭)
- **T3로부터 pick_request 받음**

## 핵심 작업 영역

```
tracks/T2 (최진우)/                                              ← 내 작업
  ├ vision/                                              ← HSV detector
  ├ manip/                                               ← M0609 controller
  └ stubs/                                               ← Day 1-3 stub

rover/sim/rover_envs/assets/robots/aau_rover.py        ← M0609 부착 (USD)
rover/sim/mission/camera_utils.py                      ← 카메라 설정 참고
```

## 절대 손대지 마라
- T3 (이찬휘) 코드 (driving 영역)
- T5 (이지민) 코드 (localization 영역)
- 인터페이스 schema (PM 승인 필수)
- 클론의 PPO 정책 파일

## 카메라 설정 (T1 (김현중)과 합의 권장)

```python
camera_position = (0.0, -0.2, 0.7)   # 마스트 위치
camera_rotation_deg = (0, 30, 180)   # 30° 아래
resolution = (640, 480)              # RGB
```

## 광물 색 사양 (T1 (김현중)과 합의)

| 광물 | RGB | HSV hue | value |
|------|-----|:------:|:-----:|
| blue | (50, 100, 240) | 105~120° | 10 |
| red | (230, 60, 60) | 0~10° | 25 |
| yellow | (240, 220, 50) | 25~35° | 50 |

## 도구
```bash
pip install opencv-python
```

## 일정 핵심
- **Day 1 EOD** ⚠️: M0609 USD 호환성 spike — 결과를 PM에 즉시
- **Day 2**: Vision 3색 detection 완성
- **Day 3**: M0609 부착 + scripted trajectory
- **Day 4**: T5 (이지민)/T3와 첫 통합 (estimated_pose 사용)
- **Day 5**: 가치점수, edge case
- **Day 6+**: 폴리싱 + T1 (김현중)/T4 (성선규) 보조

## 트러블슈팅
1. M0609 USD 깨지면 → 단순 6-link 매니퓰레이터 직접 USD 작성 (1h)
2. HSV detection false positive 많음 → S/V min 임계치 올리기 (>100)
3. 2D→3D 좌표 정확도 떨어짐 → T5 (이지민)의 estimated_pose 신뢰도 확인
