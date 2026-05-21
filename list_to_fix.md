# list_to_fix.md — 수정 대기 목록

작동에는 지장 없지만 정리가 필요한 항목들. 발견 시 추가하고, 고치면 체크.

---

## [ ] 미네랄 충돌체 — 동적 강체에 삼각형 메시 (PhysX 경고)

**증상** — Isaac Sim 기동 시 `omni.physx` 에러 로그 다수:

> `triangle mesh collision (approximation None) cannot be a part of a dynamic body, falling back to convexHull approximation: .../Minerals/blue_XXXX/Reference/Cube`

**원인** — 미네랄은 그리퍼로 집을 수 있게 동적 RigidBody인데, 마커 자산 내부
`/scene/Cube` 충돌체가 `physics:approximation = none`(정확한 삼각형 메시)으로
저작돼 있음. PhysX는 동적 강체에 삼각형 메시 충돌을 허용하지 않음.

- `isaac_sim/assets/markers/tier2_mineral/mineral_blue.usd` — 해당
- `isaac_sim/assets/markers/tier2_mineral/mineral_yellow.usd` — 해당
- `mineral_red.usd` — 충돌 Cube 없음 (에러 안 남)

**영향** — 없음. PhysX가 convexHull로 자동 대체 → 시뮬레이션 정상 동작.
기동 로그만 시끄러움.

**수정** — 위 두 자산의 `/scene/Cube` `physics:approximation`을
`none → convexHull`로 변경. world USD가 자산을 참조하므로 terrain 재생성 불필요.

**발견** — 2026-05-21, terrain_00022 시뮬레이션 중 (커밋 208e51c 시점)
