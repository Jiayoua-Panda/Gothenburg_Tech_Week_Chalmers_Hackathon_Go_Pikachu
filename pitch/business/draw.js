/* Business mode drawings. Runs inside the film's IIFE and uses its helpers:
   ease, easeIO, clamp, lerp, rrect, label, FONT, COL. Registered via BUSINESS_DRAW. */

function wrapText(ctx, s, maxW, size, weight = 500) {
  ctx.font = `${weight} ${size}px ${FONT}`;
  const out = []; let line = "";
  for (const w of s.split(" ")) {
    const t = line ? line + " " + w : w;
    if (ctx.measureText(t).width > maxW && line) { out.push(line); line = w; } else line = t;
  }
  if (line) out.push(line);
  return out;
}
function labelPair(ctx, a, b, x, y, sa, sb) {
  label(ctx, a, x, y, sa, COL.text, 700, "left");
  ctx.font = `700 ${sa}px ${FONT}`;
  label(ctx, b, x + ctx.measureText(a).width + 18, y, sb, COL.grey, 500, "left");
}
function miniRobot(ctx, x, base, head) {
  ctx.fillStyle = "#f5f5f7"; rrect(ctx, x - 8, base - 58, 16, 56, 8); ctx.fill();
  ctx.fillStyle = head; ctx.beginPath(); ctx.arc(x, base - 68, 9, 0, Math.PI * 2); ctx.fill();
}

// B1: cyber brain (fleet) around big brain (factory pipeline) around small brain (what we built).
function drawNest(ctx, W, H, local, amb) {
  const kO = ease(local / 0.8);
  ctx.save(); ctx.globalAlpha = kO;
  ctx.fillStyle = "#060607"; rrect(ctx, 0, 0, W, H, 36); ctx.fill();
  ctx.strokeStyle = "rgba(245,245,247,0.16)"; ctx.lineWidth = 2; ctx.stroke();
  labelPair(ctx, "Cyber brain", "the whole fleet · many robots, one shared brain", 36, 58, 34, 24);
  ctx.restore();

  const kI = ease((local - 0.5) / 0.8);
  if (kI <= 0) return;
  ctx.save(); ctx.globalAlpha = kI;
  ctx.fillStyle = "#0b0b0d"; rrect(ctx, 30, 90, W - 60, H - 120, 28); ctx.fill();
  ctx.strokeStyle = "rgba(245,245,247,0.24)"; ctx.lineWidth = 2; ctx.stroke();
  labelPair(ctx, "Big brain", "the factory pipeline, one skill at a time", 66, 148, 34, 24);
  ctx.restore();

  const names = ["Collect", "Build scene", "Train", "Validate", "Sim-to-real", "Deploy"];
  const subs = ["scan the floor", "digital twin", "in simulation", "the same tests", "onto a real robot", "on the shop floor"];
  const cy = 300, r = 58, x0 = 160, gap = (W - 320) / 5, cx = i => x0 + i * gap;
  const kl = easeIO((local - 0.9) / 1.4);
  ctx.strokeStyle = "rgba(245,245,247,0.16)"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx(0), cy); ctx.lineTo(lerp(cx(0), cx(5), kl), cy); ctx.stroke();

  // small brain: the part we built, outlined green once the stages are in
  const kS = ease((local - 2.4) / 0.7);
  if (kS > 0) {
    ctx.save(); ctx.globalAlpha = kS;
    const bx = cx(1) - 118, bw = cx(3) - cx(1) + 236;
    ctx.fillStyle = "rgba(48,209,88,0.06)"; rrect(ctx, bx, 200, bw, 262, 24); ctx.fill();
    ctx.strokeStyle = "rgba(48,209,88,0.85)"; ctx.lineWidth = 3; ctx.stroke();
    label(ctx, "Small brain · what you just saw", cx(2), 500, 26, COL.green, 700);
    ctx.restore();
  }

  names.forEach((n, i) => {
    const k = ease((local - 1.0 - i * 0.2) / 0.6);
    if (k <= 0) return;
    ctx.save(); ctx.globalAlpha = k; ctx.translate(cx(i), cy + (1 - k) * 20);
    ctx.fillStyle = "#0e0e10"; ctx.beginPath(); ctx.arc(0, 0, r, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = i === 3 ? "rgba(48,209,88,0.8)" : "rgba(245,245,247,0.22)"; ctx.lineWidth = 2; ctx.stroke();
    stageGlyph(ctx, i, amb);
    label(ctx, n, 0, r + 48, 32, COL.text, 700);
    label(ctx, subs[i], 0, r + 82, 22, COL.grey, 500);
    ctx.restore();
  });
}
function stageGlyph(ctx, i, t) {
  ctx.strokeStyle = COL.text; ctx.lineWidth = 3; ctx.lineJoin = "round"; ctx.lineCap = "round";
  if (i === 0) { // scan dots
    for (let a = 0; a < 4; a++) for (let b = 0; b < 3; b++) {
      const k = 0.25 + 0.75 * (0.5 + 0.5 * Math.sin(t * 5 - a * 0.9));
      ctx.fillStyle = `rgba(255,176,32,${k})`; ctx.beginPath(); ctx.arc(-18 + a * 12, -12 + b * 12, 3.5, 0, Math.PI * 2); ctx.fill();
    }
  } else if (i === 1) { // twin: two offset floor plans
    rrect(ctx, -24, -20, 34, 30, 5); ctx.stroke();
    ctx.strokeStyle = "rgba(245,245,247,0.45)"; rrect(ctx, -10, -8, 34, 30, 5); ctx.stroke();
  } else if (i === 2) { // training loop
    ctx.beginPath(); ctx.arc(0, 0, 20, -0.3, Math.PI * 1.55); ctx.stroke();
    ctx.fillStyle = COL.text; ctx.beginPath(); ctx.moveTo(20, -14); ctx.lineTo(24, 2); ctx.lineTo(10, -4); ctx.fill();
  } else if (i === 3) { // test gate
    ctx.strokeStyle = COL.green; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(-18, 2); ctx.lineTo(-5, 15); ctx.lineTo(20, -13); ctx.stroke();
  } else if (i === 4) { // real robot
    miniRobot(ctx, 0, 36, COL.amber);
  } else { // factory roof
    ctx.beginPath(); ctx.moveTo(-26, 20); ctx.lineTo(-26, -4); ctx.lineTo(-12, -16); ctx.lineTo(-12, -4); ctx.lineTo(2, -16); ctx.lineTo(2, -4);
    ctx.lineTo(16, -16); ctx.lineTo(16, -4); ctx.lineTo(26, -4); ctx.lineTo(26, 20); ctx.closePath(); ctx.stroke();
  }
}

// B2: eight skill groups; only locomotion (stairs) has been tested.
const SKILLS = [
  ["Locomotion", "walk, turn, sidestep, step over a cable", true],
  ["Balance", "recover from a push or a slip, carry an uneven load"],
  ["Navigation", "around pallets, past forklifts, through doorways"],
  ["Manipulation", "grasp, release, rotate, pull, push, insert"],
  ["Whole-body", "carry a box while walking, crouch and reach"],
  ["Tool use", "screwdriver, scanner, torque wrench"],
  ["Inspection", "read a gauge, a barcode, a part's orientation"],
  ["Recovery", "dropped object, blocked path, failed grasp"],
];
function drawLibrary(ctx, W, H, local, amb) {
  const gap = 26, tw = (W - 3 * gap) / 4, th = 234;
  const pulse = 0.5 + 0.5 * Math.sin(amb * 4);
  SKILLS.forEach(([name, sk, tested], i) => {
    const k = ease((local - 0.2 - i * 0.12) / 0.6);
    if (k <= 0) return;
    const x = (i % 4) * (tw + gap), y = Math.floor(i / 4) * (th + gap) + (1 - k) * 20;
    ctx.save(); ctx.globalAlpha = k;
    ctx.fillStyle = "#0e0e10"; rrect(ctx, x, y, tw, th, 22); ctx.fill();
    ctx.strokeStyle = tested ? `rgba(48,209,88,${0.7 + 0.3 * pulse})` : "rgba(245,245,247,0.16)"; ctx.lineWidth = tested ? 3 : 2; ctx.stroke();
    label(ctx, name, x + 28, y + 60, 38, tested ? COL.text : "#c7c7cc", 700, "left");
    wrapText(ctx, sk, tw - 56, 24).forEach((ln, j) => label(ctx, ln, x + 28, y + 104 + j * 32, 24, COL.grey, 500, "left"));
    if (tested) label(ctx, "climb stairs · tested", x + 28, y + th - 28, 26, COL.green, 700, "left");
    else label(ctx, "idea", x + 28, y + th - 28, 22, COL.dim, 600, "left");
    ctx.restore();
  });
}

// B2b: a new skill → this floor's digital twin (flaws kept) → a robot; then what it must understand, in order.
function drawTeach(ctx, W, H, local, amb) {
  const box = (x, y, w, h, at, hl) => {
    const k = ease((local - at) / 0.6); if (k <= 0) return 0;
    ctx.save(); ctx.globalAlpha = k;
    ctx.fillStyle = "#0e0e10"; rrect(ctx, x, y, w, h, 22); ctx.fill();
    ctx.strokeStyle = hl || "rgba(245,245,247,0.16)"; ctx.lineWidth = hl ? 3 : 2; ctx.stroke();
    ctx.restore(); return k;
  };
  const arrow = (x, y, at) => {
    const k = ease((local - at) / 0.5); if (k <= 0) return;
    ctx.save(); ctx.globalAlpha = k; ctx.strokeStyle = "rgba(245,245,247,0.42)"; ctx.fillStyle = "rgba(245,245,247,0.42)"; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + 40, y); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x + 50, y); ctx.lineTo(x + 38, y - 9); ctx.lineTo(x + 38, y + 9); ctx.fill(); ctx.restore();
  };
  const text = (k, fn) => { if (k <= 0) return; ctx.save(); ctx.globalAlpha = k; fn(); ctx.restore(); };

  // sources
  [["Staff instruction", 0], ["Central skill library", 160]].forEach(([n, y], i) => {
    const k = box(0, y, 380, 130, 0.2 + i * 0.2);
    text(k, () => { label(ctx, "source", 28, y + 46, 20, COL.grey, 600, "left"); label(ctx, n, 28, y + 92, 34, COL.text, 700, "left"); });
  });
  arrow(400, 145, 0.8);

  // digital twin: a floor with its flaws, scanned
  const kT = box(470, 0, 580, 290, 1.0, "rgba(255,176,32,0.8)");
  text(kT, () => {
    label(ctx, "Digital twin of this floor", 498, 46, 20, COL.grey, 600, "left");
    label(ctx, "Scan it. Keep the flaws.", 498, 92, 34, COL.text, 700, "left");
    const fy = 236, fx = 510;
    ctx.strokeStyle = "rgba(245,245,247,0.55)"; ctx.lineWidth = 3; ctx.lineJoin = "round";
    ctx.beginPath(); ctx.moveTo(fx, fy); ctx.lineTo(fx + 110, fy); ctx.lineTo(fx + 110, fy - 26); ctx.lineTo(fx + 190, fy - 32);  // tilted step
    ctx.lineTo(fx + 190, fy - 58); ctx.lineTo(fx + 270, fy - 66); ctx.lineTo(fx + 290, fy - 54); ctx.lineTo(fx + 330, fy - 60);  // damaged floor
    ctx.lineTo(fx + 500, fy - 60); ctx.stroke();
    ctx.strokeRect(fx + 380, fy - 110, 70, 50);                                                                                 // the pallet
    for (let i = 0; i < 24; i++) {
      const x = fx + 20 + i * 20, k = 0.3 + 0.7 * (0.5 + 0.5 * Math.sin(amb * 5 - i * 0.5));
      const y = x < fx + 110 ? fy : x < fx + 190 ? fy - 30 : x < fx + 330 ? fy - 60 : x > fx + 375 && x < fx + 455 ? fy - 112 : fy - 60;
      ctx.fillStyle = `rgba(255,176,32,${k})`; ctx.beginPath(); ctx.arc(x, y - 10, 4, 0, Math.PI * 2); ctx.fill();
    }
    label(ctx, "tilted stairs · a damaged floor · the pallet that is always there", 498, 276, 19, COL.grey, 500, "left");
  });
  arrow(1070, 145, 1.6);

  // robot
  const kB = box(1140, 0, 480, 290, 1.8);
  text(kB, () => {
    label(ctx, "Robot", 1168, 46, 20, COL.grey, 600, "left");
    label(ctx, "Learns it for this floor,", 1168, 92, 32, COL.text, 700, "left");
    label(ctx, "then does it on the real one", 1168, 132, 24, COL.grey, 500, "left");
    miniRobot(ctx, 1380, 262, COL.amber);
  });

  // what the robot has to understand, in this order
  const kH = ease((local - 2.4) / 0.6);
  text(kH, () => label(ctx, "What the robot has to understand, in this order", 0, 368, 22, COL.grey, 600, "left"));
  const steps = ["Its body", "The task", "The environment", "Working with others"], pw = 360, gap = (W - 4 * pw) / 3;
  steps.forEach((s, i) => {
    const x = i * (pw + gap), k = box(x, 400, pw, 96, 2.6 + i * 0.25);
    text(k, () => label(ctx, s, x + pw / 2, 460, 30, COL.text, 700));
    if (i < 3) arrow(x + pw + 5, 448, 2.8 + i * 0.25);
  });
}

// B3: three brains on three clocks; then one robot fails and the whole fleet gets the fix.
const BRAINS = [
  { n: "Collective brain", w: "one for the whole factory", d: "skills, training, digital twins" },
  { n: "Edge brain", w: "one per robot", d: "keeps replanning on its own, | in step with its team" },
  { n: "Execute brain", w: "on the robot", d: "balance, joints, the safety stop" },
];
function drawFleet(ctx, W, H, local, amb) {
  const bandY = i => i * 195, bandH = 160, mid = i => bandY(i) + 80;
  const pulse = 0.5 + 0.5 * Math.sin(amb * 4);
  const xs = [1130, 1245, 1360, 1475, 1590], top = { x: 1360, y: mid(0) }, base = mid(2) + 40;
  const b = local - 5.3;          // second beat starts with the second headline
  const FAIL = 1;

  BRAINS.forEach((L, i) => {
    const k = ease((local - 0.3 - i * 0.35) / 0.6);
    if (k <= 0) return;
    ctx.save(); ctx.globalAlpha = k;
    ctx.fillStyle = "#0e0e10"; rrect(ctx, 0, bandY(i), 1000, bandH, 24); ctx.fill();
    ctx.strokeStyle = "rgba(245,245,247,0.16)"; ctx.lineWidth = 2; ctx.stroke();
    label(ctx, L.n, 32, bandY(i) + 68, 42, COL.text, 700, "left");
    label(ctx, L.w, 32, bandY(i) + 108, 22, COL.grey, 500, "left");
    const lines = L.d.split(" | ").flatMap(part => wrapText(ctx, part, 490, 30, 700)), y0 = bandY(i) + 91 - (lines.length - 1) * 19;
    lines.forEach((ln, j) => label(ctx, ln, 480, y0 + j * 38, 30, COL.text, 700, "left"));
    ctx.setLineDash([3, 9]); ctx.strokeStyle = "rgba(245,245,247,0.2)";
    ctx.beginPath(); ctx.moveTo(1012, mid(i)); ctx.lineTo(1100, mid(i)); ctx.stroke(); ctx.setLineDash([]);
    ctx.restore();
  });

  const kN = ease((local - 1.2) / 0.8);
  if (kN <= 0) return;
  ctx.save(); ctx.globalAlpha = kN;
  ctx.strokeStyle = "rgba(245,245,247,0.16)"; ctx.lineWidth = 2;
  xs.forEach(x => { ctx.beginPath(); ctx.moveTo(top.x, top.y + 44); ctx.lineTo(x, mid(1) - 18); ctx.moveTo(x, mid(1) + 18); ctx.lineTo(x, base - 80); ctx.stroke(); });
  ctx.fillStyle = "#0e0e10"; ctx.beginPath(); ctx.arc(top.x, top.y, 44, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = "rgba(245,245,247,0.42)"; ctx.stroke();
  for (let a = 0; a < 3; a++) { ctx.fillStyle = COL.text; ctx.beginPath(); ctx.arc(top.x - 14 + a * 14, top.y, 4, 0, Math.PI * 2); ctx.fill(); }
  xs.forEach((x, i) => {
    ctx.fillStyle = "#0e0e10"; ctx.beginPath(); ctx.arc(x, mid(1), 18, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = "rgba(245,245,247,0.42)"; ctx.stroke();
    const failed = b > 0 && i === FAIL, fixed = b > 3.4;
    miniRobot(ctx, x, base, failed && !fixed ? COL.red : COL.amber);
  });
  ctx.restore();
  if (b <= 0) return;

  // one robot fails
  const kR = ease(b / 0.4);
  ctx.save(); ctx.globalAlpha = kR * (b > 3.4 ? clamp(1 - (b - 3.4) / 0.5) : 1);
  ctx.strokeStyle = COL.red; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(xs[FAIL], base - 40, 42 + 5 * pulse, 0, Math.PI * 2); ctx.stroke();
  label(ctx, "one fails", xs[FAIL], base + 44, 24, COL.red, 700);
  ctx.restore();

  // the failure goes up: robot → its edge brain → collective brain
  const up = [[xs[FAIL], base - 90], [xs[FAIL], mid(1)], [top.x, top.y]];
  const kU = easeIO((b - 0.5) / 1.2);
  if (kU > 0) {
    const segL = up.slice(1).map((p, i) => Math.hypot(p[0] - up[i][0], p[1] - up[i][1])), tot = segL[0] + segL[1];
    let rem = tot * kU;
    ctx.save(); ctx.strokeStyle = COL.red; ctx.lineWidth = 4; ctx.setLineDash([2, 12]); ctx.lineCap = "round"; ctx.lineDashOffset = amb * 30;
    ctx.beginPath(); ctx.moveTo(...up[0]);
    for (let i = 0; i < 2 && rem > 0; i++) {
      const f = Math.min(1, rem / segL[i]); rem -= segL[i];
      ctx.lineTo(lerp(up[i][0], up[i + 1][0], f), lerp(up[i][1], up[i + 1][1], f));
    }
    ctx.stroke(); ctx.restore();
  }
  // fixed and tested once, at the top
  const kF = ease((b - 1.8) / 0.5);
  if (kF > 0) {
    ctx.save(); ctx.globalAlpha = kF; ctx.strokeStyle = COL.green; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(top.x, top.y, 52 + 4 * pulse, 0, Math.PI * 2); ctx.stroke();
    label(ctx, "fixed and tested once", top.x, 18, 24, COL.green, 700);
    ctx.restore();
  }
  // and sent down to every robot
  const kD = easeIO((b - 2.4) / 1.0);
  if (kD > 0) {
    ctx.save(); ctx.strokeStyle = COL.green; ctx.lineWidth = 3;
    xs.forEach(x => {
      const ex = lerp(top.x, x, kD), ey = lerp(top.y + 44, mid(1) - 18, kD);
      ctx.beginPath(); ctx.moveTo(top.x, top.y + 44); ctx.lineTo(ex, ey); ctx.stroke();
      if (kD >= 1) { ctx.beginPath(); ctx.moveTo(x, mid(1) + 18); ctx.lineTo(x, lerp(mid(1) + 18, base - 80, clamp((b - 3.4) / 0.4))); ctx.stroke(); }
    });
    const kA = ease((b - 3.6) / 0.5);
    ctx.globalAlpha = kA; label(ctx, "every robot updated", 1475, base + 44, 24, COL.green, 700);
    ctx.restore();
  }
}

const BUSINESS_DRAW = { nest: drawNest, library: drawLibrary, teach: drawTeach, fleet: drawFleet };
