/* Mission Control front-end — Socket.IO + minimap canvas + HUD bindings.
   Read-only at this stage: buttons are placeholders, no upstream publish. */

(function () {
  const $ = (id) => document.getElementById(id);

  /* ── Connection pill ────────────────────────────────────────── */
  const sock = io();
  const pill = $("conn-pill");

  sock.on("connect", () => {
    pill.textContent = "ONLINE";
    pill.classList.remove("pill-offline");
    pill.classList.add("pill-online");
  });
  sock.on("disconnect", () => {
    pill.textContent = "OFFLINE";
    pill.classList.remove("pill-online");
    pill.classList.add("pill-offline");
  });

  /* ── Mineral counters (top-right) ───────────────────────────── */
  /* mission_manager 가 종류별 카운트를 아직 발행하지 않는다. 일단
     `last_error` 같은 메타엔 없으므로 total collected 만 우상단의 cargo
     슬롯에 표시하고, 종류별 슬롯은 동일값 보여주거나 0 유지. 추후 backend
     확장 시 mineral_counts: {blue, yellow, green} 필드 추가 예정. */

  function paintMinerals(state) {
    $("res-collected").textContent = state.collected_count | 0;
    $("res-goal").textContent = state.collection_goal | 0;
    $("res-blue").textContent   = state.collected_blue   | 0;
    $("res-yellow").textContent = state.collected_yellow | 0;
    $("res-green").textContent  = state.collected_green  | 0;
  }

  /* ── Phase / status / battery / collected bar ───────────────── */

  const PHASE_CLASSES = [
    "phase-EXPLORE", "phase-APPROACH", "phase-PICK_READY",
    "phase-RETURN_TO_BASE", "phase-MISSION_COMPLETE", "phase-MANUAL",
  ];

  function paintPhase(rawState) {
    const badge = $("phase-badge");
    const manual = rawState.startsWith("MANUAL/");
    const bare = rawState.split("/").pop();
    PHASE_CLASSES.forEach((c) => badge.classList.remove(c));
    badge.classList.add(`phase-${bare}`);
    if (manual) badge.classList.add("phase-MANUAL");
    badge.textContent = (manual ? "MANUAL · " : "") + bare;
  }

  function paintBattery(pct, low, crit) {
    pct = Math.max(0, Math.min(100, pct));
    const bar = $("batt-bar");
    bar.style.width = pct.toFixed(1) + "%";
    bar.classList.remove("bar-ok", "bar-low", "bar-crit");
    bar.classList.add(crit ? "bar-crit" : low ? "bar-low" : "bar-ok");
    $("batt-pct").textContent = pct.toFixed(0) + "%";
  }

  function paintCollected(collected, goal) {
    goal = Math.max(1, goal | 0);
    const pct = Math.min(100, (collected / goal) * 100);
    $("col-bar").style.width = pct.toFixed(1) + "%";
    $("col-text").textContent = `${collected} / ${goal}`;
  }

  sock.on("state", (s) => {
    paintPhase(s.state || "EXPLORE");
    paintBattery(s.battery_percent, s.low_battery, s.critical_battery);
    paintCollected(s.collected_count | 0, s.collection_goal | 0);
    paintMinerals(s);
    $("task").textContent = s.active_task || "—";
    $("err").textContent = s.last_error || "—";
  });

  /* ── Odom: pos / yaw / speed ─────────────────────────────────── */

  sock.on("odom", (o) => {
    $("pos-xy").textContent = `${o.x.toFixed(2)} , ${o.y.toFixed(2)}`;
    $("pos-yaw").textContent = `${o.yaw_deg.toFixed(0)} °`;
    $("spd").textContent = `${o.speed.toFixed(2)} m/s`;
  });

  sock.on("cmd_vel", (c) => {
    $("cmdv").textContent = `v=${c.lin.toFixed(2)}  ω=${c.ang.toFixed(2)}`;
  });

  /* ── Minimap canvas (OccupancyGrid + rover pos) ──────────────── */

  const cvs = $("minimap");
  const ctx = cvs.getContext("2d");
  let lastMap = null;
  let lastOdom = null;
  let lastPath = null;     // {pts: [[x,y], ...]}
  let lastTarget = null;   // {x, y} | null

  function paintMap() {
    if (!lastMap) {
      ctx.fillStyle = "#050a14";
      ctx.fillRect(0, 0, cvs.width, cvs.height);
      ctx.fillStyle = "#5a7aa0";
      ctx.font = "10px monospace";
      ctx.textAlign = "center";
      ctx.fillText("awaiting /mission/minimap...", cvs.width / 2, cvs.height / 2);
      return;
    }
    const { w, h, data } = lastMap;
    const img = ctx.createImageData(w, h);
    // minimap_publisher 가 nav_msgs/OccupancyGrid 규약대로 세 값만 발행:
    //   -1 = unknown (미밝힘, "fog of war")
    //    0 = free   (rover 가 지나간 곳, = covered)
    //  100 = occupied (장애물)
    //
    // 사용자 요청: 지나간 곳 vs 미밝힘 의 대비를 확실히. covered 를 SC2
    // 자기군 톤(노랑)으로, unknown 은 거의 검정으로 두 lightness 끝점을
    // 잡아 한눈에 구분되게.
    for (let i = 0; i < w * h; i++) {
      const v = data[i];
      let r, g, b;
      if (v < 0)        { r =  18; g =  22; b =  30; }     // unknown
      else if (v < 50)  { r = 255; g = 216; b =  74; }     // covered (v=0)
      else              { r = 255; g =  80; b =  80; }     // obstacle (v=100)
      const p = i * 4;
      img.data[p]     = r;
      img.data[p + 1] = g;
      img.data[p + 2] = b;
      img.data[p + 3] = 255;
    }
    // 원본 그리드 -> 임시 canvas -> 메인 canvas 로 scale.
    const tmp = document.createElement("canvas");
    tmp.width = w; tmp.height = h;
    tmp.getContext("2d").putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = "#050a14";
    ctx.fillRect(0, 0, cvs.width, cvs.height);
    // OccupancyGrid 의 row 0 이 origin (보통 좌하단). canvas 는 y-down 이라
    // 위아래를 뒤집어서 north-up minimap 으로 보이게.
    ctx.save();
    ctx.translate(0, cvs.height);
    ctx.scale(cvs.width / w, -cvs.height / h);
    ctx.drawImage(tmp, 0, 0);
    ctx.restore();

    // 계획 경로 (하늘색 선) — SC2 minimap 의 이동 경로 풍.
    const worldToPx = (wx, wy) => {
      const px = ((wx - lastMap.ox) / lastMap.res) * (cvs.width / lastMap.w);
      const py = cvs.height - ((wy - lastMap.oy) / lastMap.res)
                              * (cvs.height / lastMap.h);
      return [px, py];
    };
    if (lastPath && lastPath.pts && lastPath.pts.length >= 2) {
      ctx.save();
      ctx.strokeStyle = "#6cd9ff";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      const [x0, y0] = worldToPx(lastPath.pts[0][0], lastPath.pts[0][1]);
      ctx.moveTo(x0, y0);
      for (let k = 1; k < lastPath.pts.length; k++) {
        const [px, py] = worldToPx(lastPath.pts[k][0], lastPath.pts[k][1]);
        ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();
    }

    // 현재 목표 anchor (분홍 별) — coverage_node 의 SectorPlanner target.
    if (lastTarget && Number.isFinite(lastTarget.x)
                   && Number.isFinite(lastTarget.y)) {
      const [tx, ty] = worldToPx(lastTarget.x, lastTarget.y);
      ctx.save();
      ctx.fillStyle = "#ff5ce0";
      ctx.strokeStyle = "#000";
      ctx.lineWidth = 1.5;
      // 5-pointed star.
      const r1 = 9, r2 = 4;
      ctx.beginPath();
      for (let k = 0; k < 10; k++) {
        const r = (k % 2 === 0) ? r1 : r2;
        const a = -Math.PI / 2 + (k * Math.PI / 5);
        const px = tx + Math.cos(a) * r;
        const py = ty + Math.sin(a) * r;
        if (k === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      }
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    // Rover 위치 마커.
    if (lastOdom) {
      const px = ((lastOdom.x - lastMap.ox) / lastMap.res) * (cvs.width / w);
      const py = cvs.height - ((lastOdom.y - lastMap.oy) / lastMap.res)
                 * (cvs.height / h);
      if (Number.isFinite(px) && Number.isFinite(py)) {
        const yaw = (lastOdom.yaw_deg || 0) * Math.PI / 180;
        ctx.save();
        ctx.translate(px, py);
        ctx.rotate(-yaw);
        // rover marker — covered 노랑 위에서도 보이게 시안+검정 테두리.
        ctx.fillStyle = "#6cd9ff";
        ctx.strokeStyle = "#000";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(9, 0);
        ctx.lineTo(-6, 6);
        ctx.lineTo(-6, -6);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
    }
    // Basecamp marker (0,0).
    if (lastMap) {
      const bx = ((0 - lastMap.ox) / lastMap.res) * (cvs.width / lastMap.w);
      const by = cvs.height - ((0 - lastMap.oy) / lastMap.res)
                  * (cvs.height / lastMap.h);
      ctx.save();
      ctx.strokeStyle = "#6dffb0";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.arc(bx, by, 9, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }
  }

  sock.on("minimap", (m) => { lastMap = m; paintMap(); });
  // odom on a separate emit cadence; redraw to refresh rover marker.
  sock.on("odom", (o) => { lastOdom = o; paintMap(); });
  sock.on("path", (p) => { lastPath = p; paintMap(); });
  sock.on("target", (t) => {
    lastTarget = (t && Number.isFinite(t.x) && Number.isFinite(t.y)) ? t : null;
    paintMap();
  });

  // initial paint with placeholder text.
  paintMap();

  /* ── Action grid is intentionally inert at this stage ────────── */
  document.querySelectorAll(".action-grid .act").forEach((btn) => {
    btn.addEventListener("click", (e) => e.preventDefault());
  });
})();
