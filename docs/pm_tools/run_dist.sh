#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# Daily Integration Smoke Test (DIST)
#
# PM이 매일 18:00 실행. 통합 깨짐 즉시 감지.
# 통과 못 하면 그날 안에 fix (다음날로 넘기지 않음).
# ═══════════════════════════════════════════════════════════════

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$PROJECT_ROOT/pm_tools/dist_logs"
mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/dist_$TIMESTAMP.log"

echo "════════════════════════════════════════════════════════"
echo " 🚦 DIST — Daily Integration Smoke Test"
echo " Time: $(date)"
echo " Log:  $LOG_FILE"
echo "════════════════════════════════════════════════════════"

cd "$PROJECT_ROOT"

# ────────────────────────────────────────────────────────────
# Step 1: Git 동기화 확인
# ────────────────────────────────────────────────────────────
echo ""
echo "[1/6] Git sync check..."
{
    git fetch origin 2>&1 || echo "  ⚠️  Origin fetch failed (오프라인?)"
    BEHIND=$(git rev-list HEAD..origin/main --count 2>/dev/null || echo "0")
    if [ "$BEHIND" -gt "0" ]; then
        echo "  ⚠️  Local main is $BEHIND commits behind origin"
    else
        echo "  ✅ Up to date"
    fi
} | tee -a "$LOG_FILE"

# ────────────────────────────────────────────────────────────
# Step 2: 인터페이스 schema 검증
# ────────────────────────────────────────────────────────────
echo ""
echo "[2/6] Interface schema validation..."
{
    if [ -f "interfaces/terrain_meta_schema.json" ]; then
        # Schema 자체가 valid JSON인지
        python3 -c "import json; json.load(open('interfaces/terrain_meta_schema.json'))" \
            && echo "  ✅ terrain_meta_schema.json is valid JSON" \
            || { echo "  ❌ terrain_meta_schema.json invalid"; exit 1; }
    fi

    # Example data가 schema 통과하는지 (jsonschema 설치 필요)
    if command -v python3 &> /dev/null; then
        python3 <<'EOF' || echo "  ⚠️  Schema validation skipped (jsonschema not installed)"
import json
try:
    import jsonschema
    schema = json.load(open('interfaces/terrain_meta_schema.json'))
    example = json.load(open('interfaces/example_terrain_meta.json'))
    jsonschema.validate(example, schema)
    print("  ✅ example_terrain_meta.json passes schema")
except ImportError:
    raise
except Exception as e:
    print(f"  ❌ Validation failed: {e}")
    raise
EOF
    fi
} 2>&1 | tee -a "$LOG_FILE"

# ────────────────────────────────────────────────────────────
# Step 3: T1 출력물 점검 (terrain 생성됐는지)
# ────────────────────────────────────────────────────────────
echo ""
echo "[3/6] T1 outputs check..."
{
    if [ -d "generated_terrains" ]; then
        COUNT=$(ls -1 generated_terrains/terrain_* 2>/dev/null | wc -l)
        echo "  📊 Generated terrains: $COUNT"
        if [ "$COUNT" -ge "1" ]; then
            FIRST=$(ls -1 generated_terrains/terrain_* | head -1)
            echo "  📂 First terrain: $FIRST"
            for f in terrain_only.usd rocks_merged.usd obstacle_grid.npy heightmap.npy meta.json; do
                if [ -f "$FIRST/$f" ]; then
                    echo "    ✅ $f"
                else
                    echo "    ❌ $f MISSING"
                fi
            done
        fi
    else
        echo "  ⏳ generated_terrains/ not yet (Day 1 normal)"
    fi
} 2>&1 | tee -a "$LOG_FILE"

# ────────────────────────────────────────────────────────────
# Step 4: ROS2 토픽 확인 (실행 중일 때)
# ────────────────────────────────────────────────────────────
echo ""
echo "[4/6] ROS2 topics check (if running)..."
{
    if command -v ros2 &> /dev/null && [ -n "$ROS_DISTRO" ]; then
        TOPICS=$(ros2 topic list 2>/dev/null || echo "")
        EXPECTED=(
            "/perception/detections"
            "/mission/pick_request"
            "/mission/pick_response"
            "/rover/estimated_pose"
        )
        for topic in "${EXPECTED[@]}"; do
            if echo "$TOPICS" | grep -q "$topic"; then
                echo "  ✅ $topic"
            else
                echo "  ⏳ $topic (미실행 또는 미구현)"
            fi
        done
    else
        echo "  ⏳ ROS2 not sourced (수동 점검 필요)"
    fi
} 2>&1 | tee -a "$LOG_FILE"

# ────────────────────────────────────────────────────────────
# Step 5: 최근 git activity
# ────────────────────────────────────────────────────────────
echo ""
echo "[5/6] Recent commits (24h)..."
{
    git log --since="24 hours ago" --oneline --no-merges 2>/dev/null | head -20 \
        || echo "  ⏳ No recent commits"
} 2>&1 | tee -a "$LOG_FILE"

# ────────────────────────────────────────────────────────────
# Step 6: Day별 게이트 체크
# ────────────────────────────────────────────────────────────
echo ""
echo "[6/6] Milestone gate check..."
{
    DAY=$(( ($(date +%s) - $(date -d "2026-05-19" +%s)) / 86400 + 1 ))
    echo "  📅 Day $DAY"
    case $DAY in
        1|2)
            echo "  🎯 Gate: 각 트랙 hello-world (Day 2 EOD)"
            ;;
        3|4|5)
            echo "  🎯 Gate: End-to-end 1회 (Day 5 EOD)"
            ;;
        6)
            echo "  🎯 Gate: demo-stable-v1 git tag (TODAY)"
            if git tag | grep -q "demo-stable-v1"; then
                echo "  ✅ Tag exists"
            else
                echo "  ⏳ Tag not yet — 오늘 EOD까지 필수"
            fi
            ;;
        7|8)
            echo "  🎯 Gate: Final freeze (Day 8 정오)"
            ;;
        *)
            echo "  🎯 Beyond schedule"
            ;;
    esac
} 2>&1 | tee -a "$LOG_FILE"

# ────────────────────────────────────────────────────────────
# 마무리
# ────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo " DIST 완료. Log: $LOG_FILE"
echo " 통합 깨짐이 있으면 DAILY_STATUS.md의 블로커 섹션에 기록"
echo "════════════════════════════════════════════════════════"
