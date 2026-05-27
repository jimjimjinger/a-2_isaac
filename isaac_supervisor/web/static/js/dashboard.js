/* Mission Control front-end — Socket.IO + minimap canvas + HUD bindings.
   Multi-rover capable: byRover caching + selector + minimap 동시 색 분리.
   단일 rover 모드는 rover_id="" 로 자동 fallback. */

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

  /* ── Multi-rover state ───────────────────────────────────────── */
  // byRover[rid] = {state, odom, path, target, minimap}
  const byRover = {};
  let activeRover = "";  // 선택된 rover_id (selector 와 연동)
  let knownRovers = [""]; // backend 가 connect 시 emit 한 namespace 리스트

  // rover_id → minimap overlay 색. 추가 rover 는 cycle.
  const ROVER_COLORS = {
    "":         { rover: "#6cd9ff", path: "#6cd9ff" }, // 단일 모드 cyan
    "rover_1":  { rover: "#6cd9ff", path: "#6cd9ff" }, // cyan
    "rover_2":  { rover: "#ffd84a", path: "#ffd84a" }, // yellow
    "rover_3":  { rover: "#ff5ce0", path: "#ff5ce0" }, // magenta
    "rover_4":  { rover: "#6dffb0", path: "#6dffb0" }, // green
  };
  function colorFor(rid) {
    return ROVER_COLORS[rid] || { rover: "#aaaaaa", path: "#aaaaaa" };
  }

  function ensureRover(rid) {
    if (!byRover[rid]) byRover[rid] = {};
    return byRover[rid];
  }

  /* ── Selector UI ────────────────────────────────────────────── */
  function buildSelector(namespaces) {
    const host = $("rover-selector");
    if (!host) return;
    host.innerHTML = "";
    if (!namespaces || namespaces.length === 0 || (namespaces.length === 1 && namespaces[0] === "")) {
      // 단일 모드 — selector 숨김
      host.style.display = "none";
      activeRover = "";
      return;
    }
    host.style.display = "";
    namespaces.forEach((ns) => {
      const chip = document.createElement("button");
      chip.type = "button";
      chip.className = "rover-chip";
      chip.dataset.rid = ns;
      chip.style.borderColor = colorFor(ns).rover;
      chip.textContent = ns || "rover_0";
      chip.addEventListener("click", () => {
        activeRover = ns;
        updateSelectorActive();
        refreshActiveViews();
      });
      host.appendChild(chip);
    });
    if (!activeRover || !namespaces.includes(activeRover)) {
      activeRover = namespaces[0];
    }
    updateSelectorActive();
  }
  function updateSelectorActive() {
    document.querySelectorAll(".rover-chip").forEach((c) => {
      c.classList.toggle("active", c.dataset.rid === activeRover);
    });
  }

  sock.on("rovers", (msg) => {
    knownRovers = msg.namespaces || [""];
    buildSelector(knownRovers);
    refreshActiveViews();
  });

  /* ── Mineral counters / phase / battery / collected (active rover) ── */
  function paintMinerals(state) {
    $("res-collected").textContent = state.collected_count | 0;
    $("res-goal").textContent = state.collection_goal | 0;
    $("res-blue").textContent   = state.collected_blue   | 0;
    $("res-yellow").textContent = state.collected_yellow | 0;
    $("res-green").textContent  = state.collected_green  | 0;
  }

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

  function renderStatusFor(rid) {
    const r = byRover[rid];
    if (!r) return;
    if (r.state) {
      const s = r.state;
      paintPhase(s.state || "EXPLORE");
      paintBattery(s.battery_percent, s.low_battery, s.critical_battery);
      paintCollected(s.collected_count | 0, s.collection_goal | 0);
      paintMinerals(s);
      $("task").textContent = s.active_task || "—";
      $("err").textContent = s.last_error || "—";
    }
    if (r.odom) {
      const o = r.odom;
      $("pos-xy").textContent = `${o.x.toFixed(2)} , ${o.y.toFixed(2)}`;
      $("pos-yaw").textContent = `${o.yaw_deg.toFixed(0)} °`;
      $("spd").textContent = `${o.speed.toFixed(2)} m/s`;
    }
    if (r.cmd_vel) {
      $("cmdv").textContent = `v=${r.cmd_vel.lin.toFixed(2)}  ω=${r.cmd_vel.ang.toFixed(2)}`;
    }
  }

  /* ── Active-rover camera viewport src 토글 ───────────────────── */
  function refreshCameraSrcs() {
    // body cam image (id=cam-wrist viewport 이름 그대로) src 를 active ns 로
    // redirect. mission_web_node 의 cam_wrist URL 은 첫 로드 시 단일 토픽이라
    // multi 모드에선 namespace 토픽으로 강제 교체 필요.
    const bodyImg = $("cam-wrist");
    if (bodyImg && activeRover) {
      // mjpeg URL 형태: http://host:PORT/stream?topic=/perception/image_annotated&type=mjpeg&...
      // activeRover 의 토픽으로 topic 파라미터 교체.
      const base = bodyImg.getAttribute("data-src-template")
        || bodyImg.src;
      if (!bodyImg.dataset.srcTemplate) {
        bodyImg.dataset.srcTemplate = base; // 첫 실행 시 캐시
      }
      // template 의 absolute topic ("/perception/image_annotated") 부분만 namespace 적용
      const tmpl = bodyImg.dataset.srcTemplate;
      const wantTopic = activeRover
        ? `/${activeRover}/perception/image_annotated`
        : `/perception/image_annotated`;
      // web_video_server 는 raw topic path 를 받는다 — URL encoding 하면
      // /stream 핸들러가 토픽을 인식 못 해 검은 화면이 된다.
      bodyImg.src = tmpl.replace(/topic=[^&]+/, `topic=${wantTopic}`);
    }
    const chaseImg = $("cam-chase");
    if (chaseImg && activeRover) {
      // multi 모드: per-rover chase cam (run_vehicle_v3 가 각 rover 마다
      // /<ns>/camera/chase/image_raw 발행).
      if (!chaseImg.dataset.srcTemplate) {
        chaseImg.dataset.srcTemplate = chaseImg.src;
      }
      const tmpl = chaseImg.dataset.srcTemplate;
      const wantTopic = activeRover
        ? `/${activeRover}/camera/chase/image_raw`
        : `/camera/chase/image_raw`;
      chaseImg.src = tmpl.replace(/topic=[^&]+/, `topic=${wantTopic}`);
    }
  }

  function refreshActiveViews() {
    renderStatusFor(activeRover);
    refreshCameraSrcs();
    paintMap();
  }

  /* ── Socket events: per-rover caching ────────────────────────── */
  sock.on("state", (s) => {
    const rid = s.rover_id || "";
    ensureRover(rid).state = s;
    if (!knownRovers.includes(rid)) knownRovers.push(rid);
    if (rid === activeRover) renderStatusFor(rid);
  });
  sock.on("odom", (o) => {
    const rid = o.rover_id || "";
    ensureRover(rid).odom = o;
    paintMap();
    if (rid === activeRover) renderStatusFor(rid);
  });
  sock.on("cmd_vel", (c) => {
    const rid = c.rover_id || "";
    ensureRover(rid).cmd_vel = c;
    if (rid === activeRover) renderStatusFor(rid);
  });

  /* ── Minimap canvas (OccupancyGrid + rover pos) ──────────────── */
  const cvs = $("minimap");
  const ctx = cvs.getContext("2d");

  function paintMap() {
    // 어떤 rover 의 minimap 을 백그라운드로 깔지 — active 우선, 없으면 첫
    // 가용 rover. 둘 다 같은 terrain 이라 obstacle pattern 동일.
    let bgMap = null;
    const activeData = byRover[activeRover] || {};
    if (activeData.minimap) {
      bgMap = activeData.minimap;
    } else {
      for (const rid in byRover) {
        if (byRover[rid].minimap) { bgMap = byRover[rid].minimap; break; }
      }
    }
    if (!bgMap) {
      ctx.fillStyle = "#050a14";
      ctx.fillRect(0, 0, cvs.width, cvs.height);
      ctx.fillStyle = "#5a7aa0";
      ctx.font = "10px monospace";
      ctx.textAlign = "center";
      ctx.fillText("awaiting /mission/minimap...", cvs.width / 2, cvs.height / 2);
      return;
    }
    const { w, h, data } = bgMap;
    const img = ctx.createImageData(w, h);
    for (let i = 0; i < w * h; i++) {
      const v = data[i];
      let r, g, b;
      if (v < 0)        { r =  18; g =  22; b =  30; }
      else if (v < 50)  { r = 255; g = 216; b =  74; }
      else              { r = 255; g =  80; b =  80; }
      const p = i * 4;
      img.data[p]     = r;
      img.data[p + 1] = g;
      img.data[p + 2] = b;
      img.data[p + 3] = 255;
    }
    const tmp = document.createElement("canvas");
    tmp.width = w; tmp.height = h;
    tmp.getContext("2d").putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = "#050a14";
    ctx.fillRect(0, 0, cvs.width, cvs.height);
    ctx.save();
    ctx.translate(0, cvs.height);
    ctx.scale(cvs.width / w, -cvs.height / h);
    ctx.drawImage(tmp, 0, 0);
    ctx.restore();

    const worldToPx = (wx, wy) => {
      const px = ((wx - bgMap.ox) / bgMap.res) * (cvs.width / bgMap.w);
      const py = cvs.height - ((wy - bgMap.oy) / bgMap.res)
                              * (cvs.height / bgMap.h);
      return [px, py];
    };

    // 모든 rover 의 path 를 색 분리해 그림.
    for (const rid in byRover) {
      const r = byRover[rid];
      const col = colorFor(rid);
      if (r.path && r.path.pts && r.path.pts.length >= 2) {
        ctx.save();
        ctx.strokeStyle = col.path;
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        // coverage map 은 탭과 무관하게 모든 rover 의 trail 을 동등 가시.
        ctx.globalAlpha = 1.0;
        ctx.beginPath();
        const [x0, y0] = worldToPx(r.path.pts[0][0], r.path.pts[0][1]);
        ctx.moveTo(x0, y0);
        for (let k = 1; k < r.path.pts.length; k++) {
          const [px, py] = worldToPx(r.path.pts[k][0], r.path.pts[k][1]);
          ctx.lineTo(px, py);
        }
        ctx.stroke();
        ctx.restore();
      }
    }

    // 모든 rover 의 target marker (분홍 별).
    for (const rid in byRover) {
      const t = byRover[rid].target;
      if (t && Number.isFinite(t.x) && Number.isFinite(t.y)) {
        const [tx, ty] = worldToPx(t.x, t.y);
        ctx.save();
        ctx.fillStyle = "#ff5ce0";
        ctx.strokeStyle = "#000";
        ctx.lineWidth = 1.5;
        ctx.globalAlpha = 1.0;
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
    }

    // 모든 rover 의 위치 마커 (색 분리, active 는 진하게).
    for (const rid in byRover) {
      const o = byRover[rid].odom;
      if (!o) continue;
      const col = colorFor(rid);
      const px = ((o.x - bgMap.ox) / bgMap.res) * (cvs.width / w);
      const py = cvs.height - ((o.y - bgMap.oy) / bgMap.res)
                  * (cvs.height / h);
      if (!Number.isFinite(px) || !Number.isFinite(py)) continue;
      const yaw = (o.yaw_deg || 0) * Math.PI / 180;
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-yaw);
      ctx.globalAlpha = 1.0;
      ctx.fillStyle = col.rover;
      ctx.strokeStyle = "#000";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(9, 0);
      ctx.lineTo(-6, 6);
      ctx.lineTo(-6, -6);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      // rover_id 라벨 (작게)
      if (rid) {
        ctx.rotate(yaw);
        ctx.globalAlpha = 0.95;
        ctx.fillStyle = "#fff";
        ctx.strokeStyle = "#000";
        ctx.lineWidth = 3;
        ctx.font = "bold 10px monospace";
        ctx.textAlign = "center";
        ctx.strokeText(rid, 0, -10);
        ctx.fillText(rid, 0, -10);
      }
      ctx.restore();
    }

    // Basecamp marker (0,0).
    {
      const bx = ((0 - bgMap.ox) / bgMap.res) * (cvs.width / bgMap.w);
      const by = cvs.height - ((0 - bgMap.oy) / bgMap.res)
                  * (cvs.height / bgMap.h);
      ctx.save();
      ctx.strokeStyle = "#6dffb0";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.arc(bx, by, 9, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }
  }

  sock.on("minimap", (m) => {
    const rid = m.rover_id || "";
    ensureRover(rid).minimap = m;
    paintMap();
  });
  sock.on("path", (p) => {
    const rid = p.rover_id || "";
    ensureRover(rid).path = p;
    paintMap();
  });
  sock.on("target", (t) => {
    const rid = t.rover_id || "";
    const target = (Number.isFinite(t.x) && Number.isFinite(t.y))
                  ? { x: t.x, y: t.y } : null;
    ensureRover(rid).target = target;
    paintMap();
  });

  // initial paint with placeholder text.
  paintMap();

  /* ── Action grid is intentionally inert at this stage ────────── */
  document.querySelectorAll(".action-grid .act").forEach((btn) => {
    btn.addEventListener("click", (e) => e.preventDefault());
  });
})();
