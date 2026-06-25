/* Menu bar: open/close dropdowns */
function closeMenus(){ document.querySelectorAll('.menu.open').forEach(m => m.classList.remove('open')); }
document.querySelectorAll('.menu').forEach(m => {
  m.addEventListener('click', e => {
    if (e.target.closest('.mi')) return;          // item clicks handled separately
    e.stopPropagation();
    const wasOpen = m.classList.contains('open');
    closeMenus();
    if (!wasOpen && m.querySelector('.dropdown')) m.classList.add('open');
  });
  // hover-to-switch once a menu is already open (native menu-bar feel)
  m.addEventListener('mouseenter', () => {
    if (document.querySelector('.menu.open') && m.querySelector('.dropdown')) {
      closeMenus(); m.classList.add('open');
    }
  });
});
document.addEventListener('click', closeMenus);

/* Navigation — switch page + sync the active marker across menus */
function go(page){
  const el = document.getElementById('page-' + page);
  if (!el) return;
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  el.classList.add('active');
  document.querySelectorAll('.mi[data-page]').forEach(mi => mi.classList.toggle('active', mi.dataset.page === page));
  try { localStorage.setItem('vayu.page', page); } catch(e) {}
  mruPush(page);                       // record the visit for the Ctrl+` recent-views switcher
  if(axOn && typeof axRefresh === 'function') axRefresh();   // re-tag the newly-visible page in analysis mode
  closeMenus();
}
/* Most-recently-used view stack — newest first; the Ctrl+` switcher cycles it. */
let mru = [];
function mruPush(page){ const i = mru.indexOf(page); if(i >= 0) mru.splice(i,1); mru.unshift(page); }
/* Implementation-analysis mode (Ctrl+D) — state declared early so go()'s init call is safe. */
let axOn = false;
/* Restore the last-viewed page across refreshes (mirrors the Qt app's
   restoreUiState last-page behaviour). Falls back to the home dashboard. */
(function(){
  let saved; try { saved = localStorage.getItem('vayu.page'); } catch(e) {}
  go(saved && document.getElementById('page-' + saved) ? saved : 'dash');
})();

/* ===== Whole-GCS log replay (with a draggable crop/loop region) ===== */
let replayMode=false, rpPlaying=false, rpPos=0, rpDragging=false; const rpTotal=458;   // 07:38 recorded session
let cropA=0.15, cropB=0.80;   // crop region (fractions of total) — playback loops within it
function rpFmt(sec){ sec=Math.max(0,Math.round(sec)); return String(Math.floor(sec/60)).padStart(2,'0')+':'+String(sec%60).padStart(2,'0'); }
function rpUpdate(){
  const g=document.getElementById.bind(document);
  const a=cropA*rpTotal, b=cropB*rpTotal, span=Math.max(0.001,b-a);
  const pf=Math.max(0,Math.min(1,(rpPos-a)/span));     // playhead position WITHIN the crop window (0..1)
  // ── Row 1: playback scrubber — zoomed to just the crop window ──
  const fill=g('rpFill'), knob=g('rpKnob');
  if(fill){ fill.style.left='0'; fill.style.width=(pf*100)+'%'; }
  if(knob) knob.style.left=(pf*100)+'%';
  // ── Row 2: crop overview — full timeline ──
  const crop=g('rpCrop'), hs=g('rpHStart'), he=g('rpHEnd'), ph=g('rpOvPh');
  if(crop){ crop.style.left=(cropA*100)+'%'; crop.style.width=((cropB-cropA)*100)+'%'; }
  if(hs) hs.style.left=(cropA*100)+'%';
  if(he) he.style.left=(cropB*100)+'%';
  if(ph) ph.style.left=((rpTotal?rpPos/rpTotal:0)*100)+'%';   // playhead tick on the overview
  setTxt('rpCur', rpFmt(rpPos)); setTxt('rpCropStart', rpFmt(a)); setTxt('rpCropEnd', rpFmt(b)); setTxt('rpTot', rpFmt(rpTotal));
}
function enterReplay(){
  replayMode=true; rpPlaying=true; rpPos=cropA*rpTotal; closeMenus();
  document.querySelector('.window').classList.add('replay');   // read-only lock (except Simulator)
  document.getElementById('replayBar').classList.remove('hidden');
  const li=document.getElementById('liveInd'); li.classList.remove('on'); li.classList.add('replay'); li.lastChild.textContent='REPLAY';
  document.getElementById('rpPlay').textContent='⏸';
  const cs=document.getElementById('connSeg'); if(cs) cs.innerHTML='<span class="dot" style="background:var(--warn)"></span>Replaying log';
  showToast('info','Log replay — flight_2026-06-13_0412.vlog'); rpUpdate(); axRefresh();
}
function exitReplay(){
  replayMode=false; rpPlaying=false;
  document.querySelector('.window').classList.remove('replay');
  document.getElementById('replayBar').classList.add('hidden');
  const li=document.getElementById('liveInd'); li.classList.remove('replay'); li.lastChild.textContent='LIVE';
  const cs=document.getElementById('connSeg'); if(cs) cs.innerHTML = connected ? '<span class="dot green"></span>Connected: /dev/ttyUSB0' : '<span class="dot red"></span>Disconnected';
  axRefresh();
}
function rpToggle(){ if(!replayMode) return; rpPlaying=!rpPlaying; document.getElementById('rpPlay').textContent = rpPlaying?'⏸':'▶'; }
function rpSkip(which){ rpPos = (which ? cropB : cropA)*rpTotal; rpUpdate(); }   // start/end = crop bounds
function rpStep(ds){ rpPos = Math.max(cropA*rpTotal, Math.min(cropB*rpTotal, rpPos+ds)); rpUpdate(); }
// Click the zoomed playback bar → seek the playhead within the crop window (fine control).
function rpSeekPlay(ev){
  if(ev.target.id==='rpKnob') return;
  const r=document.getElementById('rpPlayScrub').getBoundingClientRect();
  const f=Math.max(0,Math.min(1,(ev.clientX-r.left)/r.width));
  rpPos = (cropA + f*(cropB-cropA))*rpTotal; rpUpdate();
}
// Click the full-timeline overview → seek the playhead (clamped into the crop).
function rpSeekCrop(ev){
  if(ev.target.classList.contains('rp-h')) return;   // handles drag, not seek
  const r=document.getElementById('rpCropScrub').getBoundingClientRect();
  const f=Math.max(0,Math.min(1,(ev.clientX-r.left)/r.width));
  rpPos = Math.max(cropA,Math.min(cropB,f))*rpTotal; rpUpdate();
}
// Drag the playhead (on the zoomed bar) or a crop boundary (on the overview) — works while playing.
function rpGrab(ev, which){
  ev.preventDefault(); ev.stopPropagation(); rpDragging=true;
  const playR=document.getElementById('rpPlayScrub').getBoundingClientRect();
  const cropR=document.getElementById('rpCropScrub').getBoundingClientRect();
  const move=e=>{
    if(which==='knob'){
      const f=Math.max(0,Math.min(1,(e.clientX-playR.left)/playR.width));
      rpPos=(cropA + f*(cropB-cropA))*rpTotal;                    // map into the crop window
    } else {
      const f=Math.max(0,Math.min(1,(e.clientX-cropR.left)/cropR.width));
      if(which==='start'){ cropA=Math.max(0,Math.min(f, cropB-0.03)); }
      else { cropB=Math.min(1,Math.max(f, cropA+0.03)); }
      rpPos=Math.max(cropA*rpTotal, Math.min(cropB*rpTotal, rpPos));   // keep playhead inside the crop
    }
    rpUpdate();
  };
  const up=()=>{ rpDragging=false; document.removeEventListener('mousemove',move); document.removeEventListener('mouseup',up); };
  document.addEventListener('mousemove',move); document.addEventListener('mouseup',up);
}
function rpTick(){
  if(!replayMode || !rpPlaying || rpDragging) return;
  const sp={'1×':1,'0.5×':0.5,'2×':2,'4×':4,'Max':12}[(document.getElementById('rpSpeed')||{}).value]||1;
  rpPos += 0.12*sp;
  const a=cropA*rpTotal, b=cropB*rpTotal;
  if(rpPos>=b) rpPos=a;        // loop within the crop region
  if(rpPos<a) rpPos=a;
  rpUpdate();
}

/* Transport selector — Serial link vs UDP (mirrors the Qt MainToolbar link/transport combo) */
function setTransport(v){
  const lbl = document.getElementById('portLbl'), sel = document.getElementById('portSel');
  if (v === 'udp'){ lbl.textContent = 'Host'; sel.innerHTML = '<option>127.0.0.1:14550</option><option>0.0.0.0:14550</option><option>(custom…)</option>'; }
  else { lbl.textContent = 'Port'; sel.innerHTML = '<option>/dev/ttyUSB0</option><option>/dev/pts/3 (SITL UART2)</option><option>(custom…)</option>'; }
}

/* Connect / ARM */
let connected = false, armed = false;
function toggleConn() {
  if (replayMode) { showToast('warn','Read-only in replay — exit replay to connect'); return; }
  connected = !connected;
  const b = document.getElementById('connBtn');
  b.className = 'tb-btn ' + (connected ? 'danger' : 'success');
  b.querySelector('span').textContent = connected ? 'Disconnect' : 'Connect';
  document.getElementById('connSeg').innerHTML = connected
    ? '<span class="dot green"></span>Connected: /dev/ttyUSB0' : '<span class="dot red"></span>Disconnected';
  document.getElementById('liveInd').classList.toggle('on', connected);
  showToast(connected ? 'ok' : 'info', connected ? 'Connected to /dev/ttyUSB0' : 'Link closed');
}
function toggleArm() {
  if (replayMode) { showToast('warn','Read-only in replay — cannot arm'); return; }
  armed = !armed;
  document.getElementById('armBtn').querySelector('span').textContent = armed ? 'DISARM' : 'ARM';
  const p = document.getElementById('statePill');
  p.textContent = armed ? 'ARMED' : 'STANDBY';
  p.style.color = armed ? 'var(--danger)' : 'var(--ok)';
  p.style.background = armed ? '#3a1a1a' : '#1a2d23';
  p.style.borderColor = armed ? '#783a42' : '#3a7050';
  showToast(armed ? 'warn' : 'info', armed ? 'ARMED — propellers live' : 'Disarmed');
}

/* Attitude — rectangular ADI */
const PX_PER_DEG = 4;   // pitch ladder scale
// build pitch ladder (±30° every 10°)
(function(){
  const pl = document.getElementById('pitchLadder');
  for (let deg = -30; deg <= 30; deg += 10) {
    if (deg === 0) continue;
    const m = document.createElement('div');
    m.className = 'pitch-mark';
    // inner is 320% tall; its center is the horizon. position relative to that.
    m.style.top = `calc(50% - ${deg * PX_PER_DEG}px)`;
    m.style.transform = `translate(-50%,-50%)`;
    const a = Math.abs(deg);
    m.innerHTML = `<span class="num">${a}</span><span class="bar"></span><span class="num">${a}</span>`;
    pl.appendChild(m);
  }
  // roll arc tick marks at 0,±10,±20,±30,±45,±60
  const arc = document.getElementById('rollArc');
  const cx = 100, cy = 90, r = 78;
  [-60,-45,-30,-20,-10,0,10,20,30,45,60].forEach(d => {
    const big = (d % 30 === 0);
    const rad = (-90 + d) * Math.PI / 180;
    const x1 = cx + Math.cos(rad)*r, y1 = cy + Math.sin(rad)*r;
    const x2 = cx + Math.cos(rad)*(r - (big?9:5)), y2 = cy + Math.sin(rad)*(r - (big?9:5));
    arc.innerHTML += `<line x1="${x1.toFixed(1)}" y1="${y1.toFixed(1)}" x2="${x2.toFixed(1)}" y2="${y2.toFixed(1)}" stroke="${d===0?'#FFE66D':'#cfd6e0'}" stroke-width="${big?1.6:1}"/>`;
  });
  // heading tape ticks every 30°
  const strip = document.getElementById('hdgStrip');
  const dirs = {0:'N',90:'E',180:'S',270:'W'};
  for (let h = -180; h <= 540; h += 30) {
    const v = ((h % 360) + 360) % 360;
    const s = document.createElement('span');
    s.className = 'htick';
    s.textContent = dirs[v] || v;
    s.dataset.h = h;
    strip.appendChild(s);
  }
})();

/* 2D / 3D attitude view toggle */
function setAttView(mode){
  const is3d = mode === '3d';
  document.getElementById('adi').classList.toggle('hidden', is3d);
  document.getElementById('adi3d').classList.toggle('hidden', !is3d);
  document.getElementById('btn2d').classList.toggle('on', !is3d);
  document.getElementById('btn3d').classList.toggle('on', is3d);
}

let t = 0;
(function animate(){
  t += 0.02;
  const roll = Math.sin(t)*18, pitch = Math.cos(t*0.7)*9, yaw = (Math.sin(t*0.3)*40+180+360)%360;
  document.getElementById('adiInner').style.transform =
    `translate(-50%,-50%) rotate(${-roll}deg) translateY(${pitch*PX_PER_DEG}px)`;
  document.getElementById('rollArc').setAttribute('transform', `rotate(${-roll} 100 90)`);
  // 3D airframe: bank with roll/pitch, spin with yaw
  document.getElementById('drone').style.transform =
    `rotateZ(${yaw}deg) rotateY(${roll}deg) rotateX(${pitch}deg)`;
  // heading tape: 44px per 30° tick → ~1.467px/deg; center current heading
  const strip = document.getElementById('hdgStrip');
  const w = strip.parentElement.clientWidth;
  strip.style.transform = `translateX(${w/2 - ((yaw+180)/30)*44 - 22}px)`;
  document.getElementById('rollV').textContent  = roll.toFixed(2)+'°';
  document.getElementById('pitchV').textContent = pitch.toFixed(2)+'°';
  document.getElementById('yawV').textContent   = yaw.toFixed(2)+'°';
  // RC stick gimbals: right = pitch/roll (spring-centered), left = throttle/yaw
  const sR = document.getElementById('stickR'), sL = document.getElementById('stickL');
  if (sR){ sR.style.left = (50 + Math.sin(t*1.3)*36) + '%'; sR.style.top = (50 - Math.cos(t*0.9)*36) + '%'; }
  if (sL){ sL.style.left = (50 + Math.sin(t*0.6)*30) + '%'; sL.style.top = (62 - Math.sin(t*0.45)*26) + '%'; }
  requestAnimationFrame(animate);
})();

/* Sparklines */
function spark(id, colors, phase, stdMax, dottedFrom){
  const svg = document.getElementById(id); if(!svg) return;
  const W=200,H=60,N=60; let html='';
  // value traces (solid; indices >= dottedFrom drawn dotted)
  colors.forEach((c,ci)=>{ let d='';
    for(let i=0;i<N;i++){const x=i/(N-1)*W; const y=H/2+Math.sin(i*0.3+phase+ci*1.7)*(H/3)*Math.sin(i*0.05+ci);
      d+=(i?'L':'M')+x.toFixed(1)+' '+y.toFixed(1)+' ';}
    const dot = (dottedFrom != null && ci >= dottedFrom);
    html+=`<path d="${d}" fill="none" stroke="${c}" stroke-width="${dot?1.1:1.4}" vector-effect="non-scaling-stroke"${dot?' stroke-linecap="round" stroke-dasharray="1 5"':''}/>`; });
  // rolling std σ traces (dotted, right axis 0..stdMax mapped bottom→top)
  if (stdMax){
    colors.forEach((c,ci)=>{ let d='';
      for(let i=0;i<N;i++){const x=i/(N-1)*W;
        const sd = stdMax*(0.38 + 0.16*Math.sin(i*0.11+phase*0.4+ci*2.1));  // synthetic σ
        const y = H - (sd/stdMax)*H;
        d+=(i?'L':'M')+x.toFixed(1)+' '+y.toFixed(1)+' ';}
      html+=`<path d="${d}" fill="none" stroke="${c}" stroke-width="1.1" vector-effect="non-scaling-stroke" stroke-linecap="round" stroke-dasharray="1 5" opacity=".55"/>`; });
  }
  svg.innerHTML=html;
}
/* Vehicle-state band: rolling window of status colours behind the traces */
const STATUS_N = 60, TICK_MS = 120, WINDOW_S = STATUS_N * TICK_MS / 1000;
const SCOL = { init:'#61AFEF', standby:'#98C379', armed:'#E06C75', failsafe:'#E0822E' };
let statusHistory = [];
for (let i = 0; i < STATUS_N; i++)              // seed a plausible timeline
  statusHistory.push(i < 10 ? SCOL.init : i < 34 ? SCOL.standby : i < 50 ? SCOL.armed : SCOL.failsafe);
function curStatusColor(){ return armed ? SCOL.armed : SCOL.standby; }
function applyStatusBands(){
  statusHistory.push(curStatusColor()); statusHistory.shift();
  const stops = [];
  for (let i = 0; i < STATUS_N; i++){
    const a = (i / STATUS_N * 100).toFixed(2), b = ((i + 1) / STATUS_N * 100).toFixed(2);
    stops.push(`${statusHistory[i]} ${a}%`, `${statusHistory[i]} ${b}%`);
  }
  const bg = `linear-gradient(to right, ${stops.join(',')})`;
  document.querySelectorAll('.g-status').forEach(el => el.style.background = bg);
}

/* Build legends, axis units, Y-ticks, status band + X-axis time labels */
(function(){
  const axisColor = {X:'var(--roll)', Y:'var(--pitch)', Z:'var(--yaw)', ALT:'var(--accent)'};
  document.querySelectorAll('.graph[data-axes]').forEach(g => {
    const leg = g.querySelector('.legend'), grid = g.querySelector('.g-grid'), plot = g.querySelector('.g-plot');
    const unit = g.querySelector('.unit') ? g.querySelector('.unit').textContent : '';
    const cols = (g.dataset.colors || '').split(',').filter(Boolean);
    const dottedFrom = g.dataset.dottedfrom != null ? +g.dataset.dottedfrom : Infinity;
    if (leg) g.dataset.axes.split(',').forEach((a, idx) => {
      const i = document.createElement('i');
      i.style.setProperty('--c', cols[idx] || axisColor[a] || 'var(--accent)');
      if (idx >= dottedFrom) i.classList.add('leg-dot');
      i.textContent = a; leg.appendChild(i);
    });
    const stdMax = +g.dataset.stdmax || 0;
    if (leg && stdMax) {            // dotted-line σ legend entry
      const i = document.createElement('i'); i.className = 'leg-std';
      i.style.setProperty('--c', 'var(--text-dim)'); i.textContent = 'σ'; leg.appendChild(i);
    }
    // status band — behind gridlines + trace (unless opted out)
    if (plot && !g.hasAttribute('data-noband')) { const sb = document.createElement('div'); sb.className = 'g-status'; plot.insertBefore(sb, plot.firstChild); }
    if (grid) {
      const max = +g.dataset.max, min = +g.dataset.min;
      for (let k = 0; k <= 4; k++) {
        const top = k / 4 * 100;
        const val = max - (max - min) * (k / 4);
        const gl = document.createElement('div'); gl.className = 'gl'; gl.style.top = top + '%'; grid.appendChild(gl);
        const gv = document.createElement('div'); gv.className = 'gv mono'; gv.style.top = top + '%';
        const num = Number.isInteger(val) ? val : val.toFixed(1);
        gv.textContent = k === 0 ? `${num} ${unit}` : num;   // unit on the top Y-tick
        grid.appendChild(gv);
      }
      [25,50,75].forEach(p => { const gx = document.createElement('div'); gx.className = 'gx'; gx.style.left = p + '%'; grid.appendChild(gx); });
      // right-hand σ axis (0 at bottom → stdMax at top)
      if (stdMax) for (let k = 0; k <= 4; k++) {
        const top = k / 4 * 100;
        const sval = stdMax - stdMax * (k / 4);
        const gvR = document.createElement('div'); gvR.className = 'gv gvR mono'; gvR.style.top = top + '%';
        const snum = Number.isInteger(sval) ? sval : sval.toFixed(1);
        gvR.textContent = k === 0 ? `σ ${snum}` : snum;
        grid.appendChild(gvR);
      }
    }
    // X-axis time scale (now on the right → past on the left)
    const xax = document.createElement('div'); xax.className = 'g-xaxis mono';
    xax.innerHTML = `<span>−${WINDOW_S.toFixed(1)}s</span><span>−${(WINDOW_S/2).toFixed(1)}s</span><span>0s</span>`;
    g.appendChild(xax);
  });
})();

let ph=0; const tri=['var(--roll)','var(--pitch)','var(--yaw)'];
setInterval(()=>{
  ph+=0.08;
  if(axOn) axPosition();   // keep analysis tags pinned as live content shifts the layout
  spark('g1',tri,ph,8); spark('g2',tri,ph*1.3,2); spark('g3',tri,ph*0.7,24); spark('g4',['var(--accent)'],ph*0.5,4);
  spark('gmotor', MCOLS, ph*0.6);
  spark('gRcHist', ['#61AFEF','#E06C75','#98C379','#D19A66','#C678DD','#56B6C2','#E5C07B','#ABB2BF'], ph*0.5, 0, 4);
  updateMotors();
  clTick(ph);
  if (simRunning) simTick(ph);
  atUpdate(ph);
  updateLinkStats(ph);
  calRecTick();
  rpTick();
  perfTick(ph);
  applyStatusBands();
  // status-bar sync-drift pill (green <20ms / amber <100 / red)
  const drift = 4 + Math.round(Math.sin(ph*0.2)*30 + Math.sin(ph)*3);
  const dv = document.getElementById('driftV');
  if (dv){ dv.textContent = (drift>=0?'+':'')+drift+'ms'; const ad = Math.abs(drift);
    dv.style.color = ad<20 ? 'var(--ok)' : ad<100 ? 'var(--warn)' : 'var(--danger)'; }
  const pk=document.getElementById('pktV'); pk.textContent=parseInt(pk.textContent)+7;
  // device temperature gauge (oscillate ~38°C, scale 0–80°C)
  const temp = 38 + Math.sin(ph*0.25)*4 + Math.sin(ph)*0.6;
  const fill = document.getElementById('tempFill'), read = document.getElementById('tempRead');
  if (fill){
    fill.style.height = Math.max(0, Math.min(100, temp/80*100)) + '%';
    const col = temp > 60 ? 'var(--danger)' : temp > 45 ? 'var(--warn)' : 'var(--ok)';
    read.textContent = temp.toFixed(1)+'°C';
    read.style.color = col;
  }
  // battery gauge (slow drain ~76%, scale 0–100%)
  const batt = 76 + Math.sin(ph*0.08)*3;
  const bf = document.getElementById('battFill'), br = document.getElementById('battRead');
  if (bf){
    bf.style.height = Math.max(0, Math.min(100, batt)) + '%';
    const bcol = batt > 50 ? 'var(--ok)' : batt > 20 ? 'var(--warn)' : 'var(--danger)';
    br.textContent = Math.round(batt)+'%';
    br.style.color = bcol;
  }
}, 120);

/* Log */
const logSamples=[
  ['[12:04:01] ','info','[GCS] Connected to /dev/ttyUSB0'],
  ['[12:04:01] ','ok','[FC] STANDBY — estimator converged'],
  ['[12:04:02] ','info','[GCS] Sync period updated to 5000 ms'],
  ['[12:04:03] ','warn','[FC] Magnetometer needs calibration'],
  ['[12:04:04] ','ok','[FC] Heartbeat from Device 42 · drift +4ms'],
  ['[12:04:05] ','info','[GCS] Sent CMD_ARM (lower throttle to arm)'],
  ['[12:04:06] ','err','[FC] FAILSAFE cleared — RC link restored'],
];
const logEl=document.getElementById('logLines');
logSamples.forEach(([ts,cls,msg])=>{const d=document.createElement('div');
  d.innerHTML=`<span class="t">${ts}</span><span class="${cls}">${msg}</span>`; logEl.appendChild(d);});

/* RC */
const rcDefs=[['Roll (Aileron)','stick'],['Pitch (Elevator)','stick'],['Throttle','thr'],['Yaw (Rudder)','stick'],
              ['AUX1 — Mode','aux'],['AUX2 — Arm','aux'],['AUX3','aux'],['AUX4','aux']];
const rcEl=document.getElementById('rcChannels');
rcDefs.forEach(([n,type],i)=>{const v=1100+Math.round(Math.random()*800); const pct=(v-1000)/1000*100;
  const col = type==='thr' ? '#7fae5a' : type==='aux' ? '#5c6f99' : '#3a5f8f';
  rcEl.innerHTML+=`<div class="rc-row"><div class="name">CH${i+1} · ${n}</div><div class="rc-bar"><div class="fill" style="width:${pct}%;background:${col}"></div><div class="mid"></div></div><div class="pwm mono">${v}</div></div>`;});

/* Motors — X-frame schematic with speed-arc rings, spinning props, cards + graph */
const MOTORS = [
  {n:1, x:315, y:85,  col:'#E06C75', dir:'CW'},
  {n:2, x:315, y:315, col:'#98C379', dir:'CCW'},
  {n:3, x:85,  y:315, col:'#61AFEF', dir:'CW'},
  {n:4, x:85,  y:85,  col:'#D19A66', dir:'CCW'},
];
const MCOLS = MOTORS.map(m => m.col);
const R = 40, C = 2 * Math.PI * R;
(function buildMotors(){
  const rings = document.getElementById('motorRings');
  const props = document.getElementById('motorProps');
  const labels = document.getElementById('motorLabels');
  const cards = document.getElementById('motorCards');
  MOTORS.forEach(m => {
    rings.innerHTML +=
      `<circle cx="${m.x}" cy="${m.y}" r="${R}" fill="#1A1D27" stroke="#3A4150" stroke-width="7"/>` +
      `<circle id="arc${m.n}" cx="${m.x}" cy="${m.y}" r="${R}" fill="none" stroke="${m.col}" stroke-width="7" stroke-linecap="round" transform="rotate(-90 ${m.x} ${m.y})" stroke-dasharray="${C.toFixed(1)}" stroke-dashoffset="${C.toFixed(1)}"/>` +
      `<circle cx="${m.x}" cy="${m.y}" r="23" fill="#2C313A" stroke="${m.col}" stroke-width="1.5"/>`;
    const px = m.x/400*100, py = m.y/400*100;
    props.innerHTML  += `<div class="mprop" id="mprop${m.n}" style="left:${px}%;top:${py}%"><div class="mblades"></div></div>`;
    const lblMt = m.y < 200 ? -62 : 62;   // top motors → label above ring, bottom → below
    labels.innerHTML += `<div class="ml-top" style="left:${px}%;top:${py}%;margin-top:${lblMt}px;color:${m.col}">M${m.n} · ${m.dir}</div>` +
                        `<div class="ml-pct mono" id="mpct${m.n}" style="left:${px}%;top:${py}%;color:${m.col}">0</div>`;
    cards.innerHTML  += `<div class="mcard"><span class="mdot" style="background:${m.col}"></span><span class="mname" style="color:${m.col}">M${m.n}</span><span class="mdir">${m.dir}</span><span class="mval mono" id="mval${m.n}">0%</span><span class="mstd mono" id="mstd${m.n}">±σ 0.00</span></div>`;
  });
})();
function updateMotors(){
  MOTORS.forEach((m,i) => {
    const frac = Math.max(0, Math.min(1, 0.42 + Math.sin(ph*0.6 + i*1.6)*0.16 + Math.sin(ph*2+i)*0.03));
    const arc = document.getElementById('arc'+m.n);
    if (arc) arc.setAttribute('stroke-dashoffset', (C*(1-frac)).toFixed(1));
    const pct = Math.round(frac*100);
    const pe = document.getElementById('mpct'+m.n); if (pe) pe.textContent = pct;
    const ve = document.getElementById('mval'+m.n); if (ve) ve.textContent = pct+'%';
    const se = document.getElementById('mstd'+m.n); if (se) se.textContent = '±σ '+(Math.abs(Math.sin(ph+i))*0.6).toFixed(2);
    const pr = document.getElementById('mprop'+m.n); if (pr){ const b = pr.firstChild; b.style.animationDuration = (0.7 - frac*0.5).toFixed(2)+'s'; }
  });
}

/* ===== Control Loop dashboard ===== */
// Each trace: name, colour, optional dash (setpoint), base/amp/speed/phase for
// the synthetic waveform. Current traces lag their setpoint to show tracking.
const CLSECS = [
  { key:'angle', title:'Outer Loop — Angle SP & Current', unit:'°', dec:1, min:-30, max:30, traces:[
    {nm:'R sp', c:'#D19A66', dash:1, amp:22, sp:0.6, ph:0.0},
    {nm:'P sp', c:'#C678DD', dash:1, amp:14, sp:0.5, ph:1.2},
    {nm:'Y sp', c:'#56B6C2', dash:1, amp:10, sp:0.3, ph:2.4},
    {nm:'Roll', c:'#E06C75', amp:22, sp:0.6, ph:-0.35},
    {nm:'Pitch',c:'#98C379', amp:14, sp:0.5, ph:0.85},
    {nm:'Yaw',  c:'#61AFEF', amp:10, sp:0.3, ph:2.05},
  ]},
  { key:'rate', title:'Inner Loop — Rate SP & Gyro', unit:'°/s', dec:0, min:-250, max:250, traces:[
    {nm:'R sp', c:'#D19A66', dash:1, amp:180, sp:0.9, ph:0.0},
    {nm:'P sp', c:'#C678DD', dash:1, amp:120, sp:0.8, ph:1.0},
    {nm:'Y sp', c:'#56B6C2', dash:1, amp:70,  sp:0.5, ph:2.0},
    {nm:'Roll', c:'#BE5046', amp:180, sp:0.9, ph:-0.3},
    {nm:'Pitch',c:'#7FB069', amp:120, sp:0.8, ph:0.75},
    {nm:'Yaw',  c:'#4078BF', amp:70,  sp:0.5, ph:1.8},
  ]},
  { key:'output', title:'Controller Outputs', unit:'', dec:3, min:-1, max:1, traces:[
    {nm:'Roll', c:'#E06C75', amp:0.5,  sp:0.9, ph:0},
    {nm:'Pitch',c:'#98C379', amp:0.35, sp:0.8, ph:1},
    {nm:'Yaw',  c:'#61AFEF', amp:0.2,  sp:0.5, ph:2},
    {nm:'Thr',  c:'#E5C07B', base:0.42, amp:0.05, sp:0.3, ph:0},
  ]},
  { key:'dt', title:'Loop Time (dt)', unit:'ms', dec:3, min:0, max:3, std:1, traces:[
    {nm:'Outer', c:'#98C379', base:1.0, amp:0.07, sp:1.3, ph:0},
    {nm:'Inner', c:'#61AFEF', base:0.5, amp:0.05, sp:1.7, ph:1},
  ]},
];
const clVal = (tr,i,ph) => (tr.base||0) + tr.amp*Math.sin(i*0.10 + ph*(tr.sp||1) + (tr.ph||0));
(function buildCL(){
  CLSECS.forEach(sec => {
    const host = document.getElementById('clsec-'+sec.key); if(!host) return;
    const stats = sec.traces.map((tr,idx) =>
      `<span class="cl-stat"><span class="nm"><i class="${tr.dash?'dash':''}" style="--c:${tr.c}"></i>${tr.nm}</span><b id="clv-${sec.key}-${idx}" style="color:${tr.c}">0</b></span>`).join('');
    host.innerHTML =
      `<div class="cl-head"><span class="cl-title">${sec.title}</span><span class="cl-stats">${stats}</span></div>` +
      `<div class="cl-plot"><div class="g-grid"></div><svg id="clsvg-${sec.key}" viewBox="0 0 200 60" preserveAspectRatio="none"></svg></div>`;
    // y-ticks (5) with unit on top + faint vertical gridlines
    const grid = host.querySelector('.g-grid');
    for (let k=0;k<=4;k++){
      const top=k/4*100, val=sec.max-(sec.max-sec.min)*(k/4);
      grid.innerHTML += `<div class="gl" style="top:${top}%"></div>`;
      const num = Number.isInteger(val)?val:val.toFixed(1);
      grid.innerHTML += `<div class="gv mono" style="top:${top}%">${k===0?num+' '+sec.unit:num}</div>`;
    }
    [25,50,75].forEach(p => grid.innerHTML += `<div class="gx" style="left:${p}%"></div>`);
  });
})();
function clTick(ph){
  CLSECS.forEach(sec => {
    const svg = document.getElementById('clsvg-'+sec.key); if(!svg) return;
    const W=200,H=60,N=60; let html='';
    sec.traces.forEach((tr,idx) => {
      let d='';
      for(let i=0;i<N;i++){ const x=i/(N-1)*W; const v=clVal(tr,i,ph);
        const y=H-((v-sec.min)/(sec.max-sec.min))*H;
        d+=(i?'L':'M')+x.toFixed(1)+' '+Math.max(-5,Math.min(65,y)).toFixed(1)+' '; }
      html += `<path d="${d}" fill="none" stroke="${tr.c}" stroke-width="${tr.dash?1.2:1.4}" vector-effect="non-scaling-stroke"${tr.dash?' stroke-dasharray="6 4"':''}/>`;
      const cur = clVal(tr,N-1,ph);
      const b = document.getElementById(`clv-${sec.key}-${idx}`);
      if (b){
        if (sec.std){ const sd = Math.abs(tr.amp)*0.5; b.textContent = cur.toFixed(sec.dec)+' ±'+sd.toFixed(3); }
        else b.textContent = cur.toFixed(sec.dec);
      }
    });
    svg.innerHTML = html;
  });
}

/* ===== Packet Analyzer ===== */
// Mirrors PacketDissector::typeName (protocol/PacketDissector.cpp): the real
// NavLink type nibbles 0x0..0xA, plus RAW for unknown.
const PKT_TYPES = [
  {name:'HEARTBEAT',      short:'HB',     col:'#FFE66D', hz:1},
  {name:'IMU_FULL',       short:'IMU',    col:'#4ECDC4', hz:200},
  {name:'IMU_COMPRESSED', short:'IMUΔ',   col:'#56B6C2', hz:200},
  {name:'COMMAND',        short:'Cmd',    col:'#E5C07B', hz:0},
  {name:'ATTITUDE',       short:'Att',    col:'#FF6B6B', hz:100},
  {name:'RC_CHANNELS',    short:'RC',     col:'#98C379', hz:50},
  {name:'SYSTEM_STATUS',  short:'Status', col:'#D19A66', hz:5},
  {name:'LOG',            short:'Log',    col:'#ABB2BF', hz:2},
  {name:'MOTOR',          short:'Motor',  col:'#E06C75', hz:50},
  {name:'PERF_STATS',     short:'Perf',   col:'#7FB069', hz:2},
  {name:'PERF_TASKNAME',  short:'Name',   col:'#5c6f99', hz:0.2},
  {name:'RAW',            short:'RAW',    col:'#C678DD', hz:0},
];
const hex2 = n => ('0'+(n&255).toString(16)).slice(-2);
const rnd = (a,b) => a + Math.random()*(b-a);
const colOf = t => (PKT_TYPES.find(p=>p.name===t)||{}).col || 'var(--text-muted)';

// Synthetic decoded payload per packet type
function payloadFor(type){
  switch(type){
    case 'IMU_FULL': return [['acc.x',rnd(-2,2).toFixed(3),'m/s²'],['acc.y',rnd(-2,2).toFixed(3),'m/s²'],['acc.z',rnd(9,10).toFixed(3),'m/s²'],['gyr.x',rnd(-1,1).toFixed(4),'rad/s'],['gyr.y',rnd(-1,1).toFixed(4),'rad/s'],['gyr.z',rnd(-1,1).toFixed(4),'rad/s']];
    case 'IMU_COMPRESSED': return [['acc.xyz (i16)',`${Math.round(rnd(-2000,2000))}, ${Math.round(rnd(-2000,2000))}, ${Math.round(rnd(8000,9800))}`,'mg'],['gyr.xyz (i16)',`${Math.round(rnd(-500,500))}, ${Math.round(rnd(-500,500))}, ${Math.round(rnd(-500,500))}`,'mrad/s']];
    case 'COMMAND': return [['cmd','SET_PID (0x000A)',''],['seq',Math.round(rnd(1,255))+'',''],['args','[roll.kp=4.5, …]','']];
    case 'ATTITUDE': return [['roll',rnd(-30,30).toFixed(2),'°'],['pitch',rnd(-20,20).toFixed(2),'°'],['yaw',rnd(0,360).toFixed(2),'°']];
    case 'SYSTEM_STATUS': return [['state','STANDBY (0x04)',''],['origin','HEALTH (0x02)',''],['battery',rnd(11,12.6).toFixed(2),'V'],['cpu_load',Math.round(rnd(20,60))+'',' %']];
    case 'RC_CHANNELS': return Array.from({length:8},(_,i)=>['ch'+(i+1),Math.round(rnd(1000,2000))+'','µs']);
    case 'MOTOR': return Array.from({length:4},(_,i)=>['m'+(i+1),(rnd(0.3,0.6)).toFixed(3),'']);
    case 'PERF_STATS': return [['cpu_load',Math.round(rnd(30,70))+'',' %'],['idle',Math.round(rnd(30,70))+'',' %'],['heap_free',Math.round(rnd(20,48))+'','KB'],['loop',Math.round(rnd(390,415))+'','Hz']];
    case 'PERF_TASKNAME': return [['task_id',Math.round(rnd(0,7))+'',''],['name','"ctrl_loop"',''],['state','RUNNING','']];
    case 'LOG': return [['level','INFO',''],['message','"estimator converged"','']];
    case 'HEARTBEAT': return [['(no payload)','—','']];
    default: return [['raw','…','']];
  }
}

const PKTS = [];
(function genPackets(){
  for(let i=0;i<22;i++){
    const t = PKT_TYPES[Math.floor(Math.random()*PKT_TYPES.length)];
    const rx = t.name==='HEARTBEAT' ? Math.random()>0.5 : t.name==='LOG' ? Math.random()>0.5 : Math.random()>0.3;
    const payload = payloadFor(t.name);
    const plen = Math.max(0, payload.length*4 + Math.floor(rnd(0,6)));
    const ts = (1700000000000 + i*137) >>> 0;
    const bytes = [];
    bytes.push('56','31',hex2(plen),'2a',hex2(ts),hex2(ts>>8),hex2(ts>>16),hex2(ts>>24));  // 8-byte header
    for(let b=0;b<plen;b++) bytes.push(hex2(Math.floor(Math.random()*256)));                // payload
    for(let b=0;b<4;b++) bytes.push(hex2(Math.floor(Math.random()*256)));                    // crc32
    const info = payload.map(x=>x[0]+'='+x[1]).join(' ').slice(0,46);   // one-line summary (for `info` filter field)
    const origin = t.name==='SYSTEM_STATUS' ? 'health' : '';
    PKTS.push({ i, time:`12:04:${10+i}.${Math.floor(rnd(100,999))}`, dir: rx?'RX':'TX',
                type:t.name, dev: t.name==='COMMAND'?1:42, len:plen, crc:'OK', ts, bytes, payload, info, origin });
  }
})();

// Frequency ribbon
(function buildFreq(){
  const fr = document.getElementById('freqRibbon');
  fr.innerHTML = PKT_TYPES.map(t =>
    `<span class="freq-pill"><span class="fp-dot" style="background:${t.col}"></span>${t.short} <span class="fp-hz" id="fhz-${t.name}">${t.hz}</span> Hz</span>`).join('')
    + `<span class="freq-pill" style="border-color:#3a5278;"><span class="fp-dot" style="background:#61AFEF"></span>TOTAL RX <span class="fp-hz">410</span> Hz</span>`;
})();

// Type filter chips
const enabledTypes = new Set(PKT_TYPES.map(t=>t.name));
let dirFilterVal = 'all';
(function buildChips(){
  const tc = document.getElementById('typeChips');
  tc.innerHTML = PKT_TYPES.map(t =>
    `<span class="tbtn on" data-type="${t.name}"><span class="tc-dot" style="background:${t.col}"></span>${t.short}</span>`).join('');
  tc.querySelectorAll('.tbtn').forEach(ch => ch.onclick = () => {
    const ty = ch.dataset.type;
    if (enabledTypes.has(ty)) { enabledTypes.delete(ty); ch.classList.remove('on'); }
    else { enabledTypes.add(ty); ch.classList.add('on'); }
    applyPktFilter();
  });
  // direction segmented control
  document.querySelectorAll('#dirFilter span').forEach(s => s.onclick = () => {
    document.querySelectorAll('#dirFilter span').forEach(x=>x.classList.remove('on'));
    s.classList.add('on'); dirFilterVal = s.dataset.dir; applyPktFilter();
  });
})();

// Render table
const ptbody = document.querySelector('#packetTable tbody');
(function renderPkts(){
  ptbody.innerHTML = PKTS.map(p =>
    `<tr data-i="${p.i}" data-type="${p.type}" data-dir="${p.dir}">
      <td class="mono">${p.time}</td>
      <td><span class="tag ${p.dir==='RX'?'rx':'tx'}">${p.dir}</span></td>
      <td style="color:${colOf(p.type)}">${p.type}</td>
      <td class="mono">${p.dev}</td><td class="mono">${p.len}</td>
      <td class="mono" style="color:var(--ok)">OK</td>
      <td class="mono" style="color:var(--text-dim)">${p.bytes.slice(0,6).join(' ')}…</td>
    </tr>`).join('');
  ptbody.querySelectorAll('tr').forEach(tr => tr.onclick = () => {
    ptbody.querySelectorAll('tr').forEach(x=>x.classList.remove('sel'));
    tr.classList.add('sel');
    showDecoder(PKTS[+tr.dataset.i]);
  });
})();

function applyPktFilter(){
  const q = (document.getElementById('pktSearch').value || '').toLowerCase();
  const devRaw = (document.getElementById('pktDev').value || '').trim();
  const devF = devRaw === '' ? -1 : parseInt(devRaw, 10);
  ptbody.querySelectorAll('tr').forEach(tr => {
    const p = PKTS[+tr.dataset.i];
    const okType = enabledTypes.has(p.type);
    const okDir  = dirFilterVal === 'all' || p.dir === dirFilterVal;
    const okDev  = devF < 0 || isNaN(devF) || p.dev === devF;
    const okSearch = !q || p.type.toLowerCase().includes(q) || p.bytes.join(' ').includes(q);
    let okExpr = true;
    if (pktExprPred) {
      const ctx = { type:p.type.toLowerCase(), dir:p.dir.toLowerCase(), dev:p.dev,
                    size:p.len, len:p.len, origin:(p.origin||'').toLowerCase(), info:(p.info||'').toLowerCase() };
      try { okExpr = pktExprPred(ctx); } catch(e) { okExpr = true; }   // fail-safe: invalid → inert
    }
    tr.style.display = (okType && okDir && okDev && okSearch && okExpr) ? '' : 'none';
  });
}

/* Wireshark-style display-filter — mirrors PacketFilterExpr (ui/widgets/PacketFilterExpr.*).
   Fields: type · dir · dev · size/len · origin · info.  Ops: == != < <= > >= contains, && || ! ( ).
   Compiles to a predicate; an invalid filter is inert (not fatal), exactly like the Qt class. */
let pktExprPred = null, pktExprOk = true;
function compileExpr(src){
  src = (src || '').trim();
  if (!src){ pktExprPred = null; pktExprOk = true; return; }
  const re = /\s*(\(|\)|&&|\|\||!=|!|==|<=|>=|<|>|contains|[A-Za-z_][A-Za-z_0-9]*|[0-9]+(?:\.[0-9]+)?|"[^"]*")\s*/y;
  const toks = []; let m, pos = 0;
  while (pos < src.length){ re.lastIndex = pos; m = re.exec(src); if(!m){ pktExprPred = null; pktExprOk = false; return; } toks.push(m[1]); pos = re.lastIndex; }
  let i = 0; const peek = () => toks[i], next = () => toks[i++];
  const parseOr = () => { let l = parseAnd(); while(peek()==='||'){ next(); const r = parseAnd(), a = l; l = c => a(c) || r(c); } return l; };
  const parseAnd = () => { let l = parseUnary(); while(peek()==='&&'){ next(); const r = parseUnary(), a = l; l = c => a(c) && r(c); } return l; };
  const parseUnary = () => { if(peek()==='!'){ next(); const e = parseUnary(); return c => !e(c); } return parsePrimary(); };
  function parsePrimary(){
    if (peek()==='('){ next(); const e = parseOr(); if(next()!==')') throw 0; return e; }
    const field = next(), op = next(); let val = next();
    if (field===undefined || op===undefined || val===undefined) throw 0;
    if (val[0]==='"') val = val.slice(1,-1);
    const f = field.toLowerCase();
    return c => {
      let lv = ({type:c.type, dir:c.dir, dev:c.dev, size:c.size, len:c.len, origin:c.origin, info:c.info})[f];
      if (lv===undefined) lv = '';
      if (op==='contains') return String(lv).toLowerCase().includes(String(val).toLowerCase());
      const ln = +lv, vn = +val, numeric = isFinite(ln) && isFinite(vn) && val !== '';
      if (op==='==') return numeric ? ln===vn : String(lv).toLowerCase()===String(val).toLowerCase();
      if (op==='!=') return numeric ? ln!==vn : String(lv).toLowerCase()!==String(val).toLowerCase();
      if (op==='<')  return ln <  vn; if (op==='>')  return ln >  vn;
      if (op==='<=') return ln <= vn; if (op==='>=') return ln >= vn;
      return true;
    };
  }
  try { const p = parseOr(); if(i!==toks.length) throw 0; pktExprPred = p; pktExprOk = true; }
  catch(e){ pktExprPred = null; pktExprOk = false; }
}
function applyPktExpr(){
  const v = (document.getElementById('pktExpr').value || '').trim();
  compileExpr(v);
  const msg = document.getElementById('pktExprMsg');
  if (msg){
    msg.style.color = !v ? 'var(--text-dim)' : pktExprOk ? 'var(--ok)' : 'var(--danger)';
    msg.textContent = !v ? 'fields: type · dir · dev · size · origin · info'
                         : pktExprOk ? '✓ filter active' : '✗ invalid filter (inert)';
  }
  applyPktFilter();
}
/* Streaming toggle — Qt writes live packets to CSV while armed (Start/Stop Streaming). */
function togglePktStream(){
  const b = document.getElementById('pktStream');
  const on = b.classList.toggle('on');
  b.textContent = on ? '■ Stop Streaming' : '▶ Start Streaming';
}

/* Link stats panel — link quality %, up/down throughput, cumulative bytes. */
let lsRx = 1.23*1024*1024, lsTx = 4.5*1024;
function humanBytes(b){ const u=['B','KB','MB','GB','TB']; let i=0; while(b>=1024&&i<4){ b/=1024; i++; } return b.toFixed(i===0?0:2)+' '+u[i]; }
function lsPlot(id, series, min, max){
  const svg=document.getElementById(id); if(!svg) return;
  const W=200,H=60,N=60; let html='';
  series.forEach(s=>{ let d='';
    for(let i=0;i<N;i++){ const x=i/(N-1)*W; const y=H-((s.fn(i)-min)/(max-min))*H;
      d+=(i?'L':'M')+x.toFixed(1)+' '+Math.max(-2,Math.min(62,y)).toFixed(1)+' '; }
    html+=`<path d="${d}" fill="none" stroke="${s.c}" stroke-width="1.4" vector-effect="non-scaling-stroke"/>`; });
  svg.innerHTML=html;
}
function updateLinkStats(ph){
  const qual = i => Math.min(100, 97 + 2.4*Math.sin(i*0.07+ph*0.3) - (Math.sin(i*0.5+ph)>1.7?7:0));
  const down = i => 11.5 + 2.2*Math.sin(i*0.13+ph*0.5) + Math.sin(i*0.6+ph)*0.6;
  const up   = i => 0.45 + 0.18*Math.sin(i*0.09+ph*0.4);
  lsPlot('gLinkQual', [{c:'#4ECDC4', fn:qual}], 0, 100);
  lsPlot('gLinkSpeed', [{c:'#4ECDC4', fn:down}, {c:'#E5A95C', fn:up}], 0, 24);
  const qv=qual(59), dv=down(59), uv=up(59);
  const q=document.getElementById('lsQual'), d=document.getElementById('lsDown'),
        u=document.getElementById('lsUp'), tot=document.getElementById('lsTotal');
  if(q){ q.textContent='Link '+Math.round(qv)+'%'; q.style.color = qv>90?'var(--ok)':qv>70?'var(--warn)':'var(--danger)'; }
  if(d) d.textContent='↓ '+dv.toFixed(1)+' KB/s';
  if(u) u.textContent='↑ '+uv.toFixed(2)+' KB/s';
  lsRx += dv*1024*0.12; lsTx += uv*1024*0.12;   // accumulate over the 120 ms tick
  if(tot) tot.textContent='RX '+humanBytes(lsRx)+'  ·  TX '+humanBytes(lsTx);
}

/* Live environment telemetry — wind speed/direction, air-relative airspeed,
   and ground-effect factor, all derived from the World config + drone state. */
const windSpdHist=[], windDirHist=[], airSpdHist=[], geHist=[];
function numv(id, d){ const e=document.getElementById(id); const v=e?parseFloat(e.value):NaN; return isFinite(v)?v:(d||0); }
function pushHist(a, v){ a.push(v); if(a.length>60) a.shift(); }
function updateEnv(ph, st){
  // --- wind vector: steady N/E/D + periodic gust + band-limited turbulence ---
  const wN=numv('windN',0), wE=numv('windE',0), wD=numv('windD',0);
  const gust=numv('windGust',0), period=numv('windPeriod',4)||4, turb=numv('windTurb',0);
  const gN = gust*Math.sin(2*Math.PI*simT/period);
  const tN = turb*1.6*(Math.sin(ph*1.7)+0.6*Math.sin(ph*3.1));
  const tE = turb*1.6*(Math.cos(ph*1.9)+0.6*Math.sin(ph*2.7));
  const cN=wN+gN+tN, cE=wE+tE, cD=wD;
  const spd=Math.hypot(cN,cE,cD), dir=((Math.atan2(cE,cN)*180/Math.PI)%360+360)%360;
  // --- air-relative airspeed: |v_drone − v_wind| (drone vel from speed·heading) ---
  const yaw=(st.yaw||0)*Math.PI/180;
  const vN=st.gs*Math.cos(yaw), vE=st.gs*Math.sin(yaw), vD=-(st.vs||0);
  const airspeed=Math.hypot(vN-cN, vE-cE, vD-cD);
  // --- ground-effect factor: thrust gain that grows as altitude AGL → 0 ---
  const geEl=document.getElementById('geOn'); const geOn = geEl ? geEl.classList.contains('on') : true;
  const h=Math.max(0.05, numv('geHeight',2)), z=Math.max(0.05, st.alt||0);
  const ge = geOn ? 1 + 0.35/(1+(z/h)*(z/h)) : 1.0;
  // --- history + plots ---
  pushHist(windSpdHist,spd); pushHist(windDirHist,dir); pushHist(airSpdHist,airspeed); pushHist(geHist,ge);
  lsPlot('gWindSpd', [{c:'#56B6C2', fn:i=> windSpdHist[i] ?? spd}], 0, 12);
  lsPlot('gWindDir', [{c:'#C678DD', fn:i=> windDirHist[i] ?? dir}], 0, 360);
  lsPlot('gAirSpd',  [{c:'#98C379', fn:i=> airSpdHist[i] ?? airspeed}], 0, 12);
  lsPlot('gGndEff',  [{c:'#E5C07B', fn:i=> geHist[i] ?? ge}], 1, 1.4);
  const set=(id,t)=>{ const e=document.getElementById(id); if(e) e.textContent=t; };
  set('windSpd', spd.toFixed(1)+' m/s');
  set('windDir', String(Math.round(dir)).padStart(3,'0')+'°');
  set('airSpd',  airspeed.toFixed(1)+' m/s');
  set('geFac',   ge.toFixed(2)+'×');
}

function showDecoder(p){
  const d = document.getElementById('decoder');
  const field = (k,v,u) => `<div class="dec-field"><span class="dk">${k}</span><span class="dv">${v}${u?`<span class="u">${u}</span>`:''}</span></div>`;
  const header =
    field('sync','0x56','') + field('type',`0x${hex2(PKT_TYPES.indexOf(PKT_TYPES.find(t=>t.name===p.type)))} (${p.type})`,'') +
    field('version','1','') + field('length',p.len,' B') + field('device_id',p.dev,'') + field('timestamp',p.ts>>>0,' ms');
  const payload = p.payload.map(([k,v,u]) => field(k,v,u)).join('');
  const crc = field('crc32','0x'+p.bytes.slice(-4).join('').toUpperCase(),'✓ OK');
  // raw hex, colour-coded by region
  const hh = p.bytes.slice(0,8).map(b=>`<span class="hh">${b}</span>`).join(' ');
  const hp = p.bytes.slice(8,8+p.len).map(b=>`<span class="hp">${b}</span>`).join(' ');
  const hc = p.bytes.slice(-4).map(b=>`<span class="hc">${b}</span>`).join(' ');
  d.innerHTML =
    `<div class="dec-h">Header</div>${header}` +
    `<div class="dec-h">Payload — ${p.type}</div>${payload}` +
    `<div class="dec-h">Footer</div>${crc}` +
    `<div class="dec-h">Raw Bytes</div><div class="dec-hex">${hh} ${hp} ${hc}</div>` +
    `<div class="dec-legend"><i style="color:var(--accent)">■ header</i><i style="color:var(--text-muted)">■ payload</i><i style="color:var(--warn)">■ crc</i></div>`;
}

/* ===== Simulator FPV ===== */
function simTab(which){
  document.querySelectorAll('#page-sim .tab').forEach(t=>t.classList.toggle('on', t.dataset.tab===which));
  document.querySelectorAll('#page-sim .tab-page').forEach(p=>p.classList.toggle('on', p.id==='simtab-'+which));
  // Autotune reflows the whole sim dock into a 3-pane layout (vehicle view top-left,
  // plots bottom-left, settings right) so the live plots are big and readable.
  const dock=document.getElementById('simDock'); if(dock) dock.classList.toggle('at-mode', which==='autotune');
}
let simRunning = true, showDown = true, showAdi = true;
function togglePip(which){
  if (which==='down'){ showDown=!showDown; document.getElementById('chipDown').classList.toggle('on',showDown); document.getElementById('pipDown').classList.toggle('hidden',!showDown); }
  else { showAdi=!showAdi; document.getElementById('chipAdi').classList.toggle('on',showAdi); document.getElementById('pipAdi').classList.toggle('hidden',!showAdi); }
}
function setSimRun(on){
  simRunning = on;
  const chip = document.getElementById('simRunChip');
  chip.textContent = on ? '● Running' : '● Stopped';
  chip.classList.toggle('on', on);
  chip.style.color = on ? '' : 'var(--text-dim)';
  applyRunningLocks();
}
// HUD palette — matches SimHudWidget.cpp exactly
const HUD = 'rgb(90,255,160)', DIM = 'rgba(90,255,160,0.43)', WARN = 'rgb(255,95,95)', HBOX = 'rgba(0,0,0,0.51)';
const HIST_N = 128;
const accHist = [[],[],[]], gyrHist = [[],[],[]];
let simT = 0;

// canvas line/text helpers
function pen(x,col,w,dash){ x.strokeStyle=col; x.lineWidth=w; x.setLineDash(dash||[]); }
function ln(x,x1,y1,x2,y2){ x.beginPath(); x.moveTo(x1,y1); x.lineTo(x2,y2); x.stroke(); }
function txt(x,s,px,py,al,col){ x.fillStyle=col; x.textAlign=al||'left'; x.fillText(s,px,py); }

function simTick(ph){
  // synthetic vehicle state
  const roll=Math.sin(ph*0.5)*16, pitch=Math.cos(ph*0.4)*7, yaw=(Math.sin(ph*0.2)*50+90+360)%360;
  const alt=6.5+5.3*Math.sin(ph*0.16), gs=3.1+Math.sin(ph*0.6)*1.2, vs=Math.cos(ph*0.16)*0.85;  // altitude sweeps low→high so ground-effect is visible
  const motor=[0,1,2,3].map(i=>0.42+Math.sin(ph*0.6+i*1.6)*0.14);
  // push IMU history
  const acc=[Math.sin(ph*1.1)*1.5, Math.cos(ph*0.9)*1.4, 9.8+Math.sin(ph*1.7)*0.4];
  const gyr=[Math.sin(ph*1.3)*18, Math.cos(ph*1.1)*14, Math.sin(ph*0.7)*9];
  for(let i=0;i<3;i++){ accHist[i].push(acc[i]); if(accHist[i].length>HIST_N)accHist[i].shift();
                        gyrHist[i].push(gyr[i]); if(gyrHist[i].length>HIST_N)gyrHist[i].shift(); }
  simT += 0.12;
  drawHud({roll,pitch,yaw,alt,gs,vs,motor,status: armed?'ARMED':'STANDBY'});
  if (showDown) drawDownCam(yaw);
  if (showAdi)  drawAdi(roll,pitch);
  // right-panel + toolbar readouts
  const set=(id,t)=>{const e=document.getElementById(id); if(e) e.textContent=t;};
  const mm=String(Math.floor(simT/60)).padStart(2,'0'), ss=String(Math.floor(simT%60)).padStart(2,'0');
  set('simAlt',alt.toFixed(1)+' m'); set('simSpd',gs.toFixed(1)+' m/s');
  set('simVs',(vs>=0?'+':'')+vs.toFixed(1)+' m/s'); set('simClk',`${mm}:${ss}`);
  set('simPose',`pos 0.0, 0.0, −${alt.toFixed(1)} m · vel ${gs.toFixed(1)} m/s`);
  // RC bridge bars
  const rcv=[1500+Math.sin(ph*0.7)*300,1500+Math.cos(ph*0.6)*250,1100+(Math.sin(ph*0.3)*0.5+0.5)*500,1500+Math.sin(ph*0.4)*200];
  rcv.forEach((v,i)=>{const f=document.getElementById('rcm'+i),t=document.getElementById('rcv'+i);
    if(f)f.style.width=((v-1000)/1000*100)+'%'; if(t)t.textContent=Math.round(v);});
  updateEnv(ph, {gs, yaw, vs, alt});
}

// Port of SimHudWidget::paintEvent — green FPV HUD over a synthetic world.
function drawHud(s){
  const cv=document.getElementById('fpvCanvas'); if(!cv) return;
  const host=cv.parentElement, dpr=window.devicePixelRatio||1;
  const W=host.clientWidth, H=host.clientHeight;
  if(cv.width!==W*dpr||cv.height!==H*dpr){ cv.width=W*dpr; cv.height=H*dpr; }
  const x=cv.getContext('2d'); x.setTransform(dpr,0,0,dpr,0,0);
  x.clearRect(0,0,W,H);
  x.font='bold 11px monospace'; x.textBaseline='middle';
  const cx=W/2, cy=H/2, ppd=H/45;

  // --- synthetic world (sky/ground) behind the HUD ---
  x.save(); x.translate(cx,cy); x.rotate(-s.roll*Math.PI/180); x.translate(0,s.pitch*ppd);
  x.fillStyle='#33597f'; x.fillRect(-W*2,-H*2,W*4,H*2);
  x.fillStyle='#4a3a26'; x.fillRect(-W*2,0,W*4,H*2);
  pen(x,'rgba(255,255,255,0.05)',1);
  for(let k=1;k<=12;k++){ ln(x,-W*2,k*20,W*2,k*20); }
  x.restore();

  // ===== artificial horizon + pitch ladder (clipped to centre column) =====
  x.save();
  const sm=110; x.beginPath(); x.rect(sm,0,W-2*sm,H); x.clip();
  x.translate(cx,cy); x.rotate(-s.roll*Math.PI/180); x.translate(0,s.pitch*ppd);
  pen(x,HUD,2); const hl=W*0.5-115;
  ln(x,-hl,0,-W*0.05,0); ln(x,W*0.05,0,hl,0);
  for(let a=-90;a<=90;a+=10){ if(a===0)continue; const y=-a*ppd; if(Math.abs(y)>H*0.55)continue;
    const half=(a%20===0)?W*0.10:W*0.06;
    pen(x,DIM,1.3, a<0?[5,4]:[]); ln(x,-half,y,-W*0.03,y); ln(x,W*0.03,y,half,y); x.setLineDash([]);
    pen(x,HUD,1); txt(x,String(a),-half-4,y,'right',HUD); txt(x,String(a),half+4,y,'left',HUD); }
  x.restore();

  // ===== center boresight =====
  pen(x,HUD,2);
  ln(x,cx-30,cy,cx-10,cy); ln(x,cx+10,cy,cx+30,cy); ln(x,cx,cy-8,cx,cy);
  x.fillStyle=HUD; x.beginPath(); x.arc(cx,cy,2.5,0,7); x.fill();

  // ===== roll arc + pointer =====
  const R=Math.min(W,H)*0.34;
  pen(x,DIM,1.4); x.beginPath();
  for(let a=-60;a<=60;a+=2){ const r=a*Math.PI/180; const px=cx+R*Math.sin(r), py=cy-R*Math.cos(r); a===-60?x.moveTo(px,py):x.lineTo(px,py); } x.stroke();
  pen(x,HUD,1.4);
  [-60,-45,-30,-20,-10,0,10,20,30,45,60].forEach(a=>{ const r=a*Math.PI/180, ins=(a%30===0)?12:7;
    ln(x,cx+R*Math.sin(r),cy-R*Math.cos(r),cx+(R-ins)*Math.sin(r),cy-(R-ins)*Math.cos(r)); });
  { const rr=Math.max(-60,Math.min(60,s.roll))*Math.PI/180, sn=Math.sin(rr), cs=Math.cos(rr);
    const tx=cx+R*sn, ty=cy-R*cs, dx=sn, dy=-cs, px=-dy, py=dx;
    x.fillStyle=HUD; x.beginPath(); x.moveTo(tx,ty); x.lineTo(tx-dx*11+px*6,ty-dy*11+py*6); x.lineTo(tx-dx*11-px*6,ty-dy*11-py*6); x.closePath(); x.fill(); }

  // ===== heading / compass tape (top) =====
  { const hdg=((s.yaw%360)+360)%360, tapeW=W*0.5, hppd=tapeW/80, ty=16;
    x.save(); x.beginPath(); x.rect(cx-tapeW/2,ty-4,tapeW,34); x.clip();
    const base=Math.floor(hdg)-45;
    for(let iv=base; iv<=base+90; iv++){ const h=((iv%360)+360)%360; if(h%5)continue;
      const hx=cx+(iv-hdg)*hppd;
      if(h%10===0){ pen(x,HUD,1.3); ln(x,hx,ty,hx,ty+9);
        let lbl=h===0?'N':h===90?'E':h===180?'S':h===270?'W':(h%30===0?String(h):'');
        if(lbl) txt(x,lbl,hx,ty+17,'center',HUD); }
      else { pen(x,DIM,1.1); ln(x,hx,ty,hx,ty+5); } }
    x.restore();
    pen(x,HUD,1.4); x.fillStyle=HUD; x.beginPath(); x.moveTo(cx,ty+11); x.lineTo(cx-6,ty+2); x.lineTo(cx+6,ty+2); x.closePath(); x.fill();
    const bw=48,bh=17; x.fillStyle=HBOX; x.fillRect(cx-24,ty-18,bw,bh); pen(x,HUD,1.4); x.strokeRect(cx-24,ty-18,bw,bh);
    txt(x,Math.round(hdg)+'°',cx,ty-18+bh/2,'center',HUD); }

  // ===== vertical tapes (speed left, altitude right) =====
  function tape(tx,side,value,ppu,major,title,dec){
    const top=H*0.20, bot=H*0.80, mid=(top+bot)/2;
    x.save(); x.beginPath(); x.rect(tx-60,top-2,120,bot-top+4); x.clip();
    pen(x,DIM,1.2); ln(x,tx,top,tx,bot);
    const lo=Math.floor(value-(mid-top)/ppu)-1, hi=Math.ceil(value+(bot-mid)/ppu)+1;
    for(let v=lo;v<=hi;v++){ const y=mid-(v-value)*ppu; if(y<top||y>bot)continue;
      const maj=(v%major===0), len=maj?12:6; pen(x,maj?HUD:DIM,maj?1.3:1);
      ln(x,tx,y,tx+side*len,y);
      if(maj&&v>=0) txt(x,String(v),tx+side*16,y,side<0?'right':'left',HUD); }
    x.restore();
    const vbx=side<0?tx-56:tx+4; x.fillStyle=HBOX; x.fillRect(vbx,mid-10,52,20);
    pen(x,HUD,1.4); x.strokeRect(vbx,mid-10,52,20);
    txt(x,value.toFixed(dec),vbx+26,mid,'center',HUD);
    x.fillStyle=HUD; x.beginPath(); x.moveTo(tx,mid); x.lineTo(tx+side*8,mid-6); x.lineTo(tx+side*8,mid+6); x.closePath(); x.fill();
    txt(x,title,tx,top-11,'center',HUD);
  }
  tape(60,-1,s.gs,12,5,'SPD m/s',1);
  tape(W-60,1,s.alt,14,5,'ALT m',1);
  // vertical speed under the altitude box
  txt(x,'VS '+s.vs.toFixed(1), W-6, H*0.80+14,'right', s.vs>=0?HUD:WARN);

  // ===== per-motor throttle bars (bottom-left) =====
  { const bw=11,gap=6,bh=64,bx=22,by=H-26;
    for(let i=0;i<4;i++){ const mx=bx+i*(bw+gap); pen(x,DIM,1); x.strokeRect(mx,by-bh,bw,bh);
      const f=Math.max(0,Math.min(1,s.motor[i])); x.fillStyle=f>0.92?WARN:HUD; x.fillRect(mx,by-bh*f,bw,bh*f);
      txt(x,'M'+(i+1),mx+bw/2,by+8,'center',HUD); } }

  // ===== system status (top-left) =====
  { const st=s.status||'—'; const danger=/FAILSAFE|ARMED|IN_AIR/.test(st);
    const scol=danger?WARN:HUD, label='STATE '+st; const w=x.measureText(label).width+16;
    x.fillStyle=HBOX; x.fillRect(12,12,w,18); pen(x,scol,1.3); x.strokeRect(12,12,w,18);
    txt(x,label,12+w/2,21,'center',scol); }

  // ===== accel / gyro mini-plots (bottom-right) =====
  function plot(rx,ry,rw,rh,title,hist,minRange){
    x.fillStyle=HBOX; x.fillRect(rx,ry,rw,rh); pen(x,DIM,1); x.strokeRect(rx,ry,rw,rh);
    let mx=minRange; for(let i=0;i<3;i++) for(const v of hist[i]) mx=Math.max(mx,Math.abs(v));
    const midY=ry+rh*0.58, plotH=rh*0.38;
    pen(x,DIM,0.8,[2,3]); ln(x,rx+3,midY,rx+rw-3,midY); x.setLineDash([]);
    const axc=['rgb(255,95,95)','rgb(90,255,160)','rgb(90,180,255)'];
    for(let i=0;i<3;i++){ const h=hist[i]; if(h.length<2)continue; pen(x,axc[i],1.2); x.beginPath();
      for(let k=0;k<h.length;k++){ const px=rx+4+(rw-8)*k/(HIST_N-1), py=midY-Math.max(-1,Math.min(1,h[k]/mx))*plotH; k?x.lineTo(px,py):x.moveTo(px,py); } x.stroke(); }
    txt(x,title,rx+4,ry+7,'left',HUD); txt(x,'±'+mx.toFixed(mx<10?1:0),rx+rw-4,ry+7,'right',DIM);
  }
  const pw=210,ph2=46,px=W-pw-14;
  plot(px,H-118,pw,ph2,'GYR °/s',gyrHist,30);
  plot(px,H-66,pw,ph2,'ACC m/s²',accHist,2);
}

// size a canvas to its CSS box (DPR-aware); returns 2d context + CSS size
function ctx2d(id){
  const cv=document.getElementById(id); if(!cv) return null;
  const dpr=window.devicePixelRatio||1, w=cv.clientWidth, h=cv.clientHeight;
  if(!w||!h) return null;
  if(cv.width!==w*dpr||cv.height!==h*dpr){ cv.width=w*dpr; cv.height=h*dpr; }
  const x=cv.getContext('2d'); x.setTransform(dpr,0,0,dpr,0,0);
  return {x,w,h};
}

// Down-facing camera PiP: body-fixed top-down view — ground scrolls/rotates,
// the quad silhouette stays centred.
function drawDownCam(yawDeg){
  const c=ctx2d('downCanvas'); if(!c) return; const {x,w,h}=c;
  x.clearRect(0,0,w,h); x.fillStyle='#0c1018'; x.fillRect(0,0,w,h);
  const cx=w/2, cy=h/2, yaw=yawDeg*Math.PI/180, off=(simT*16)%28;
  x.save(); x.translate(cx,cy); x.rotate(-yaw);
  x.strokeStyle='rgba(90,255,160,0.18)'; x.lineWidth=1; x.beginPath();
  for(let gx=-2*w; gx<=2*w; gx+=28){ x.moveTo(gx,-2*h); x.lineTo(gx,2*h); }
  for(let gy=-2*h+off; gy<=2*h; gy+=28){ x.moveTo(-2*w,gy); x.lineTo(2*w,gy); }
  x.stroke();
  // world-north marker
  x.fillStyle='rgba(90,255,160,0.7)'; x.font='bold 9px monospace'; x.textAlign='center';
  x.fillText('N', 0, -Math.min(w,h)*0.42);
  x.restore();
  // quad silhouette (fixed; front = top, red)
  const arm=Math.min(w,h)*0.30;
  x.strokeStyle='rgba(90,255,160,0.85)'; x.lineWidth=3; x.setLineDash([]);
  ln(x,cx-arm,cy-arm,cx+arm,cy+arm); ln(x,cx-arm,cy+arm,cx+arm,cy-arm);
  [[cx+arm,cy-arm,'#FF5F5F'],[cx-arm,cy-arm,'#FF5F5F'],[cx+arm,cy+arm,'#5AFFA0'],[cx-arm,cy+arm,'#5AFFA0']].forEach(m=>{
    x.fillStyle='#11151c'; x.strokeStyle=m[2]; x.lineWidth=2; x.beginPath(); x.arc(m[0],m[1],6,0,7); x.fill(); x.stroke(); });
  x.fillStyle='#5AFFA0'; x.beginPath(); x.arc(cx,cy,2.5,0,7); x.fill();
}

// Homepage-style artificial horizon PiP (blue sky / brown ground, yellow ref).
function drawAdi(roll,pitch){
  const c=ctx2d('adiCanvas'); if(!c) return; const {x,w,h}=c;
  x.clearRect(0,0,w,h);
  const cx=w/2, cy=h/2, ppd=h/30;
  x.save(); x.beginPath(); x.rect(0,0,w,h); x.clip();
  x.translate(cx,cy); x.rotate(-roll*Math.PI/180); x.translate(0,pitch*ppd);
  x.fillStyle='#4a7ab5'; x.fillRect(-w*2,-h*2,w*4,h*2);
  x.fillStyle='#6b5135'; x.fillRect(-w*2,0,w*4,h*2);
  x.strokeStyle='#fff'; x.lineWidth=2; ln(x,-w*2,0,w*2,0);
  x.font='8px monospace';
  x.strokeStyle='rgba(255,255,255,0.85)'; x.lineWidth=1.2;
  [-20,-10,10,20].forEach(a=>{ const y=-a*ppd, half=w*0.18; ln(x,-half,y,half,y);
    x.fillStyle='#fff'; x.textAlign='right'; x.fillText(Math.abs(a),-half-3,y); });
  x.restore();
  // center reference + roll pointer (yellow)
  x.strokeStyle='#FFE66D'; x.lineWidth=2.5; ln(x,cx-14,cy,cx-5,cy); ln(x,cx+5,cy,cx+14,cy);
  x.fillStyle='#FFE66D'; x.beginPath(); x.arc(cx,cy,2.5,0,7); x.fill();
  x.beginPath(); x.moveTo(cx,5); x.lineTo(cx-5,13); x.lineTo(cx+5,13); x.closePath(); x.fill();
}

/* ===== PID Autotune — mirrors the headless optimizer (tools/autotune) ===== */
let atRunning=false, atIter=0, atDone=false; const AT_MAX=40;
const AT_BASE={roll:[4.5,0.08,0.12], pitch:[4.2,0.075,0.11], yaw:[3.0,0.05,0.0]};
const AT_TGT ={roll:[5.2,0.095,0.145], pitch:[4.9,0.088,0.132], yaw:[3.4,0.06,0.0]};
function atStart(){ atRunning=true; atDone=false; atIter=0; }
function atStop(){ atRunning=false; }
function atUpdate(ph){
  if(atRunning){ atIter=Math.min(AT_MAX, atIter+0.4); if(atIter>=AT_MAX){ atRunning=false; atDone=true; } }
  const prog = atIter/AT_MAX;
  const st=document.getElementById('atStatus');
  if(st) st.textContent = atRunning ? `Running — eval ${Math.floor(atIter)}/${AT_MAX}`
                        : atDone ? `Converged · cost ${(0.06*(1-prog)+0.012).toFixed(3)}` : 'Idle';
  atDash();
  // Gains: best-so-far (converges to the proposed tune) + current eval (explores,
  // collapsing onto best as it converges) — mirrors the matplotlib current/best columns.
  const atActive = atRunning || atDone;
  ['roll','pitch','yaw'].forEach((ax,ai)=>{ for(let j=0;j<3;j++){
    const best = AT_BASE[ax][j] + (AT_TGT[ax][j]-AT_BASE[ax][j])*prog;
    const eb=document.getElementById(`ag-${ax}-${j}`);
    if(eb){ eb.textContent = j===0 ? best.toFixed(2) : best.toFixed(3); eb.style.color = atDone ? 'var(--ok)' : ''; }
    const ec=document.getElementById(`agc-${ax}-${j}`);
    if(ec){
      if(!atActive){ ec.textContent='—'; }
      else { const cur = best + Math.abs(best)*0.14*(1-prog)*Math.sin(ph*1.3 + j*1.7 + ai*2.1);
        ec.textContent = j===0 ? cur.toFixed(2) : cur.toFixed(3); }
    }
  }});
  const svg=document.getElementById('gAuto'); if(!svg) return;
  const N=120,W=200,H=60, Y=v=>H-((v+0.2)/1.6)*H;
  // cost per evaluation (noisy, decaying) + best-so-far staircase, revealed up
  // to the current eval — the autotuner's live cost-convergence window.
  const shown = atDone ? N : Math.max(2, Math.floor(prog*N));
  let cst='', bst='', best=1.4;
  for(let i=0;i<shown;i++){ const x=i/(N-1)*W, t=i/N;
    const cost=0.12+1.05*Math.exp(-t*3.2)*(1+0.32*Math.sin(i*0.7+ph));
    best=Math.min(best,cost);
    cst+=(i?'L':'M')+x.toFixed(1)+' '+Y(cost).toFixed(1)+' ';
    bst+=(i?'L':'M')+x.toFixed(1)+' '+Y(best).toFixed(1)+' '; }
  svg.innerHTML=`<path d="${cst}" fill="none" stroke="#E5C07B" stroke-width="1.2" vector-effect="non-scaling-stroke" opacity=".7"/>`+
                `<path d="${bst}" fill="none" stroke="#98C379" stroke-width="1.6" vector-effect="non-scaling-stroke"/>`;
}

/* ===== Autotune live dashboard — mirrors the matplotlib plotter ===== */
const AT_AX=[{c:'#E06C75', amp:18, axph:0.0}, {c:'#98C379', amp:12, axph:1.0}, {c:'#61AFEF', amp:8, axph:2.0}];
function atSq(x){ return Math.tanh(2.4*Math.sin(x)); }   // smoothed doublet excitation
function atClampY(y){ return Math.max(-3, Math.min(63, y)); }
// per-axis setpoint (dashed) vs measured (solid); tracking error shrinks as it converges
function atTrack(id, min, max, ampScale, lagBase){
  const svg=document.getElementById(id); if(!svg) return;
  const W=200,H=60,N=60, prog=atIter/AT_MAX, active=(atRunning||atDone), w=2*Math.PI/2.4;
  let html='';
  AT_AX.forEach(a=>{ const amp=a.amp*ampScale; let dsp='', dme='';
    for(let i=0;i<N;i++){ const x=i/(N-1)*W, t=i/N*5; let sp=0, me=0;
      if(active){ sp=amp*atSq(w*t + a.axph + ph*0.5);
        const lag=lagBase*(1-prog), over=amp*0.4*(1-prog)*Math.sin(w*3.4*t + a.axph);  // overshoot ripple
        me=amp*atSq(w*(t-lag) + a.axph + ph*0.5) + over; }
      dsp+=(i?'L':'M')+x.toFixed(1)+' '+atClampY(H-((sp-min)/(max-min))*H).toFixed(1)+' ';
      dme+=(i?'L':'M')+x.toFixed(1)+' '+atClampY(H-((me-min)/(max-min))*H).toFixed(1)+' '; }
    html+=`<path d="${dsp}" fill="none" stroke="${a.c}" stroke-width="1" stroke-dasharray="5 4" opacity=".6" vector-effect="non-scaling-stroke"/>`;
    html+=`<path d="${dme}" fill="none" stroke="${a.c}" stroke-width="1.5" vector-effect="non-scaling-stroke"/>`; });
  svg.innerHTML=html;
}
// controller outputs R/P/Y (solid) + throttle (amber); effort settles as gains converge
function atOut(id){
  const svg=document.getElementById(id); if(!svg) return;
  const W=200,H=60,N=60, prog=atIter/AT_MAX, active=(atRunning||atDone), w=2*Math.PI/2.4, min=-1.1, max=1.1;
  let html='';
  AT_AX.forEach((a,k)=>{ const amp=[0.6,0.45,0.3][k]; let d='';
    for(let i=0;i<N;i++){ const x=i/(N-1)*W, t=i/N*5;
      const o=active ? amp*Math.cos(w*t + a.axph + ph*0.5)*(0.6+0.4*(1-prog)) : 0;
      d+=(i?'L':'M')+x.toFixed(1)+' '+atClampY(H-((o-min)/(max-min))*H).toFixed(1)+' '; }
    html+=`<path d="${d}" fill="none" stroke="${a.c}" stroke-width="1.3" vector-effect="non-scaling-stroke"/>`; });
  let dt='';
  for(let i=0;i<N;i++){ const x=i/(N-1)*W; const thr=active?0.42+0.03*Math.sin(ph+i*0.1):0;
    dt+=(i?'L':'M')+x.toFixed(1)+' '+atClampY(H-((thr-min)/(max-min))*H).toFixed(1)+' '; }
  html+=`<path d="${dt}" fill="none" stroke="#E5C07B" stroke-width="1.2" vector-effect="non-scaling-stroke"/>`;
  svg.innerHTML=html;
}
function atDash(){
  atTrack('gAtAng', -25, 25, 1.0, 0.25);     // attitude (deg)
  atTrack('gAtRate', -250, 250, 12, 0.18);   // body rates (deg/s)
  atOut('gAtOut');
  const ds=document.getElementById('atDashStatus'); if(!ds) return;
  const opt=(document.querySelector('#simtab-autotune select')||{}).value||'hybrid';
  const prog=atIter/AT_MAX, baseline=0.42, best=0.06*(1-prog)+0.012, imp=Math.round((baseline-best)/baseline*100);
  ds.textContent = atRunning ? `running: ${opt} · eval ${Math.floor(atIter)}/${AT_MAX} · baseline ${baseline.toFixed(2)} · best ${best.toFixed(3)} (−${imp}%)`
                 : atDone ? `DONE: ${opt} · ${AT_MAX} evals · baseline ${baseline.toFixed(2)} → best ${best.toFixed(3)} (−${imp}%)`
                 : 'idle — press Start';
}

// Drag the PiP boxes by their header, clamped within the viewport.
function makeDraggable(pip){
  const head = pip.querySelector('.pip-h'); if(!head) return;
  head.addEventListener('mousedown', e => {
    e.preventDefault();
    const cont = pip.parentElement.getBoundingClientRect();
    const box = pip.getBoundingClientRect();
    const offX = e.clientX - box.left, offY = e.clientY - box.top;
    pip.style.right = 'auto';
    const mv = ev => {
      let nx = ev.clientX - cont.left - offX, ny = ev.clientY - cont.top - offY;
      nx = Math.max(0, Math.min(cont.width  - box.width,  nx));
      ny = Math.max(0, Math.min(cont.height - box.height, ny));
      pip.style.left = nx + 'px'; pip.style.top = ny + 'px';
    };
    const up = () => { document.removeEventListener('mousemove', mv); document.removeEventListener('mouseup', up); };
    document.addEventListener('mousemove', mv); document.addEventListener('mouseup', up);
  });
}
['pipDown','pipAdi'].forEach(id => { const el = document.getElementById(id); if(el) makeDraggable(el); });

/* ===== Toast notifications — mirrors core/Notify ===== */
function showToast(kind, msg, ms){
  const host = document.getElementById('toastHost'); if(!host) return;
  const t = document.createElement('div'); t.className = 'toast ' + kind;
  const ic = {info:'', ok:'✓ ', warn:'⚠ ', err:'✕ '}[kind] || '';
  t.textContent = ic + msg; host.appendChild(t);
  setTimeout(() => { t.style.transition = 'opacity .3s'; t.style.opacity = '0'; setTimeout(() => t.remove(), 300); },
             ms || (kind==='err' ? 7000 : kind==='warn' ? 5000 : 3000));
}

/* ===== Calibration wizard — step-gated (each orientation/rotation confirmed by a click) ===== */
let calSensor='acc', calModeSel='bias', calRunning=false, calStep=0, CAL_RUN=[], calStepProg=0, calStepRec=false;
const SENSOR_NAME={acc:'ACCELEROMETER', gyr:'GYROSCOPE', mag:'MAGNETOMETER'};
// Accel full = 6 discrete orientations; each pill maps to one captured step.
const ACCEL6=[
  {ax:'X',  txt:'PLACE DRONE NOSE UP'},        {ax:'-X', txt:'PLACE DRONE NOSE DOWN'},
  {ax:'Y',  txt:'PLACE DRONE RIGHT SIDE DOWN'},{ax:'-Y', txt:'PLACE DRONE LEFT SIDE DOWN'},
  {ax:'Z',  txt:'PLACE DRONE LEVEL (UPRIGHT)'},{ax:'-Z', txt:'PLACE DRONE UPSIDE DOWN'},
];
function setTxt(id,t){ const e=document.getElementById(id); if(e) e.textContent=t; }
function segPick(span){ [...span.parentElement.children].forEach(s=>s.classList.remove("on")); span.classList.add("on"); }
function setSettingsPane(id){
  document.querySelectorAll('#settingsNav .snav').forEach(n=>n.classList.toggle('on', n.dataset.sp===id));
  document.querySelectorAll('#page-settings .settings-pane').forEach(p=>p.classList.toggle('on', p.id==='sp-'+id));
}
function resetCalAxes(){ document.querySelectorAll('#calAxes .axis-pill').forEach(p=>p.classList.remove('done','active')); const pb=document.getElementById('calProgress'); if(pb) pb.style.width='0%'; }
// Reflect sensor capabilities: gyro = bias-only (no full); mag = rotation (no mode); accel = both.
function calApplyMode(){
  const sect=document.getElementById('calModeSection'), full=document.getElementById('calModeFull'), note=document.getElementById('calModeNote');
  sect.classList.toggle('hidden', calSensor==='mag');
  if(calSensor==='gyr'){ calModeSel='bias'; full.classList.add('disabled'); full.classList.remove('on'); document.getElementById('calModeBias').classList.add('on'); note.textContent='Gyroscope supports bias zeroing only — no scale/offset.'; }
  else { full.classList.remove('disabled'); note.textContent=''; }
  document.getElementById('calAxisSection').classList.toggle('hidden', !(calSensor==='acc' && calModeSel==='full'));
}
function calSelect(s){ if(calRunning) return; calSensor=s;
  document.querySelectorAll('.sensor-card').forEach(c=>c.classList.toggle('on', c.dataset.cal===s));
  if(s==='gyr') calModeSel='bias';
  calApplyMode();
  setTxt('calStatusChip','SENSOR SELECTED: '+(s==='acc'?'ACCEL':s==='gyr'?'GYRO':'MAG'));
  setTxt('calInstruction','PRESS START TO CALIBRATE '+SENSOR_NAME[s]); resetCalAxes();
}
function calMode(m){ if(calRunning) return; if(calSensor==='gyr') m='bias'; calModeSel=m;
  document.getElementById('calModeBias').classList.toggle('on', m==='bias');
  const full=document.getElementById('calModeFull'); if(!full.classList.contains('disabled')) full.classList.toggle('on', m==='full');
  calApplyMode();
}
function calBuildSteps(){
  if(calSensor==='gyr') return [{txt:'PLACE DRONE LEVEL AND KEEP IT COMPLETELY STILL', cap:'Capture bias'}];
  if(calSensor==='mag') return [{txt:'ROTATE THE DRONE IN A FIGURE-8 THROUGH ALL AXES', cap:'✓ Done — I have rotated it', fig8:true}];
  if(calSensor==='acc' && calModeSel==='full') return ACCEL6.slice();
  return [{txt:'PLACE DRONE LEVEL AND KEEP IT STILL', cap:'Capture'}];
}
function calStart(){ if(calRunning) return; CAL_RUN=calBuildSteps(); calStep=0; calRunning=true; resetCalAxes();
  document.getElementById('calStartBtn').classList.add('hidden');
  document.getElementById('calNextBtn').classList.remove('hidden');
  document.getElementById('calFig8').classList.toggle('hidden', calSensor!=='mag');
  setTxt('calStatusChip','CALIBRATING — '+SENSOR_NAME[calSensor]);
  showToast('info','Calibration started — '+SENSOR_NAME[calSensor]);
  calShowStep();
}
function calShowStep(){
  const step=CAL_RUN[calStep]; if(!step) return;
  setTxt('calInstruction', step.txt);
  const btn=document.getElementById('calNextBtn');
  const last=calStep===CAL_RUN.length-1;
  btn.textContent = step.cap || (last ? '✓ Capture & Finish' : '✓ Capture — Next orientation');
  if(calSensor==='acc' && calModeSel==='full'){
    document.querySelectorAll('#calAxes .axis-pill').forEach((p,i)=>{ p.classList.toggle('done', i<calStep); p.classList.toggle('active', i===calStep); });
  }
  if(step.fig8){
    // Mag: the operator decides when enough axes are covered → Done is available immediately.
    calStepRec=false; calStepProg=100;
    document.getElementById('calProgress').style.width='100%';
    btn.disabled=false; setTxt('calCoverage','rotate to cover all axes');
  } else {
    // Orientation must be held + sampled before Capture unlocks.
    calStepRec=true; calStepProg=0;
    document.getElementById('calProgress').style.width='0%';
    btn.disabled=true;
    setTxt('calCoverage', (calSensor==='acc'&&calModeSel==='full') ? `ORIENTATION ${calStep+1} / ${CAL_RUN.length} · recording…` : 'hold still — recording…');
  }
}
// per-orientation recording: fills while the drone is held, then unlocks Capture
function calRecTick(){
  if(!calRunning || !calStepRec) return;
  calStepProg = Math.min(100, calStepProg + 5);
  const pb=document.getElementById('calProgress'); if(pb) pb.style.width=calStepProg+'%';
  if(calStepProg>=100){
    calStepRec=false;
    document.getElementById('calNextBtn').disabled=false;
    setTxt('calInstruction', CAL_RUN[calStep].txt + '  —  ✓ recorded');
    setTxt('calCoverage', (calSensor==='acc'&&calModeSel==='full') ? `ORIENTATION ${calStep+1} / ${CAL_RUN.length} · ready` : 'recorded — press Capture');
  }
}
function calNext(){ if(!calRunning || calStepRec) return;   // gated: present orientation must finish recording first
  if(calSensor==='acc' && calModeSel==='full'){ const p=document.querySelectorAll('#calAxes .axis-pill')[calStep]; if(p){ p.classList.remove('active'); p.classList.add('done'); } }
  calStep++;
  if(calStep>=CAL_RUN.length){ calComplete(); return; }
  calShowStep();
}
function calComplete(){ calRunning=false;
  setTxt('calInstruction','CALIBRATION COMPLETE!'); setTxt('calStatusChip','SUCCESSFUL'); setTxt('calCoverage','done');
  const pb=document.getElementById('calProgress'); if(pb) pb.style.width='100%';
  document.getElementById('calNextBtn').classList.add('hidden');
  document.getElementById('calStartBtn').classList.remove('hidden');
  document.getElementById('calFig8').classList.add('hidden');
  document.querySelectorAll('#calAxes .axis-pill').forEach(p=>{ p.classList.remove('active'); p.classList.add('done'); });
  showToast('ok', SENSOR_NAME[calSensor]+' calibration complete');
}
function calCancel(){ if(!calRunning) return; calRunning=false; resetCalAxes();
  setTxt('calInstruction','CALIBRATION CANCELED'); setTxt('calStatusChip','READY'); setTxt('calCoverage','SYSTEM READY');
  document.getElementById('calNextBtn').classList.add('hidden');
  document.getElementById('calStartBtn').classList.remove('hidden');
  document.getElementById('calFig8').classList.add('hidden');
  showToast('warn','Calibration canceled');
}
calApplyMode();   // initial sensor capabilities (accel + bias → 6-axis hidden)

/* ===== Sim World obstacles — mirrors WorldEditorWidget ===== */
let OBS=[{type:'Box', pos:[2,0,-1], size:[1,1,1], rot:[0,0,0], rest:0.30}], obsSel=0;
const OBS_COL={Box:'#61AFEF', Sphere:'#98C379', Cyl:'#D19A66'};
function renderObs(){
  const el=document.getElementById('obsList'); if(!el) return;
  el.innerHTML = OBS.length ? OBS.map((o,i)=>
    `<div class="obs-item${i===obsSel?' sel':''}" onclick="selObs(${i})"><span class="oc" style="background:${OBS_COL[o.type]}"></span>${o.type} #${i+1} · (${o.pos.map(v=>v.toFixed(1)).join(', ')})</div>`).join('')
    : '<div class="obs-item" style="color:var(--text-dim); cursor:default">no obstacles</div>';
  const o=OBS[obsSel]; if(!o) return;
  const set=(id,v)=>{ const e=document.getElementById(id); if(e) e.value=v; };
  set('obsPx',o.pos[0].toFixed(3)); set('obsPy',o.pos[1].toFixed(3)); set('obsPz',o.pos[2].toFixed(3));
  set('obsSx',o.size[0].toFixed(3)); set('obsSy',o.size[1].toFixed(3)); set('obsSz',o.size[2].toFixed(3));
  set('obsRx',o.rot[0].toFixed(3)); set('obsRy',o.rot[1].toFixed(3)); set('obsRz',o.rot[2].toFixed(3));
  set('obsRest',o.rest.toFixed(2));
}
function selObs(i){ obsSel=i; renderObs(); }
function addObs(type){ const size = type==='Sphere' ? [0.5,0.5,0.5] : type==='Cyl' ? [0.4,0.4,1.0] : [1,1,1];
  OBS.push({type, pos:[2,0,-1], size, rot:[0,0,0], rest:0.30}); obsSel=OBS.length-1; renderObs(); }
function removeObs(){ if(!OBS.length) return; OBS.splice(obsSel,1); obsSel=Math.max(0,obsSel-1); renderObs(); }
renderObs();

/* ===== Per-motor coefficients (foldable, copy-from, editable thrust axis) ===== */
const MCOEF=[
  {kt:'1.522e-5', km:'2.44e-7', mo:'1200', tau:'0.020', ax:'0', ay:'0', az:'-1'},
  {kt:'1.510e-5', km:'2.41e-7', mo:'1195', tau:'0.021', ax:'0', ay:'0', az:'-1'},
  {kt:'1.528e-5', km:'2.45e-7', mo:'1205', tau:'0.019', ax:'0', ay:'0', az:'-1'},
  {kt:'1.519e-5', km:'2.43e-7', mo:'1198', tau:'0.020', ax:'0', ay:'0', az:'-1'},
];
const MCOEF_FIELDS=[
  ['kt','k_thrust','thrust per ω² (N·s²)'],
  ['km','k_moment','reaction torque per ω²'],
  ['mo','max_omega','max rotor speed (rad/s)'],
  ['tau','time constant τ','first-order spin-up lag (s)'],
];
function mcGet(m,f){ const e=document.getElementById(`mc-${m}-${f}`); return e?e.value:''; }
function mcSet(m,f,v){ const e=document.getElementById(`mc-${m}-${f}`); if(e) e.value=v; }
function refreshMcoefSum(m){ const el=document.getElementById('mc-sum-'+m); if(!el) return;
  el.textContent = `k_t ${mcGet(m,'kt')} · ω ${mcGet(m,'mo')} · τ ${mcGet(m,'tau')} · axis ${mcGet(m,'ax')},${mcGet(m,'ay')},${mcGet(m,'az')}`; }
function toggleMcoef(m){ document.getElementById('mcoef-'+m).classList.toggle('open'); }
function copyMotorCoef(dst, src){ if(!src) return; ['kt','km','mo','tau','ax','ay','az'].forEach(f=>mcSet(dst, f, mcGet(src, f))); refreshMcoefSum(dst);
  showToast('info', `M${dst} coefficients copied from M${src}`); }
function buildMcoef(){
  const host=document.getElementById('mcoefList'); if(!host) return;
  host.innerHTML = MCOEF.map((c,idx)=>{ const m=idx+1;
    const copyOpts = MCOEF.map((_,j)=> j!==idx ? `<option value="${j+1}">M${j+1}</option>` : '').join('');
    const rows = MCOEF_FIELDS.map(([f,name,desc])=>
      `<div class="setting"><div><div class="name">${name}</div><div class="desc">${desc}</div></div><div class="spin"><input class="txt" id="mc-${m}-${f}" value="${c[f]}" oninput="refreshMcoefSum(${m})"><div class="sb">▴▾</div></div></div>`).join('');
    const axis = `<div class="setting"><div><div class="name">Thrust axis</div><div class="desc">body-frame unit vector (x, y, z)</div></div>`+
      `<div class="axis-edit"><input class="mono" id="mc-${m}-ax" value="${c.ax}" oninput="refreshMcoefSum(${m})"><input class="mono" id="mc-${m}-ay" value="${c.ay}" oninput="refreshMcoefSum(${m})"><input class="mono" id="mc-${m}-az" value="${c.az}" oninput="refreshMcoefSum(${m})"></div></div>`;
    return `<div class="mcoef" id="mcoef-${m}">
        <div class="mcoef-head" onclick="toggleMcoef(${m})">
          <span class="mcoef-caret">▸</span><span class="mcoef-name">M${m}</span>
          <span class="mcoef-sum mono" id="mc-sum-${m}"></span>
          <select class="mcoef-copy" onclick="event.stopPropagation()" onchange="copyMotorCoef(${m}, this.value); this.value='';" title="Copy coefficients from another motor">
            <option value="">copy from…</option>${copyOpts}
          </select>
        </div>
        <div class="mcoef-body">${rows}${axis}</div>
      </div>`;
  }).join('');
  for(let m=1;m<=4;m++) refreshMcoefSum(m);
}
buildMcoef();

/* ===== Sensor models — per-sensor foldable noise + error models ===== */
// comps: extra error models offered (toggle each on, with inline params).
const SENS_COMPS={
  scale: {label:'Scale-factor error', desc:'per-axis multiplicative gain', fields:['1.000','1.000','1.000']},
  sat:   {label:'Saturation', desc:'± full-scale clip', fields:['160']},
  spike: {label:'Spikes / outliers', desc:'rate Hz · magnitude', fields:['0.5','5.0']},
  vib:   {label:'Vibration', desc:'blade-pass ∝ throttle · amplitude', fields:['1.0','0.40']},
  lat:   {label:'Latency', desc:'transport delay (ms)', fields:['2.0']},
  temp:  {label:'Temp-dependent bias', desc:'bias drift per °C', fields:['0.002']},
};
const SENSORS=[
  {key:'acc', name:'Accelerometer', unit:'m/s²', rate:'1000', sigma:'0.030', biasWalk:'2e-4', biasClip:'0.08', enabled:true,  comps:['scale','sat','spike','vib','lat','temp']},
  {key:'gyr', name:'Gyroscope',     unit:'rad/s', rate:'1000', sigma:'0.0014', biasWalk:'5e-6', biasClip:'0.012', enabled:true, comps:['scale','sat','spike','vib','lat','temp']},
  {key:'mag', name:'Magnetometer',  unit:'µT',   rate:'100',  sigma:'0.30', biasWalk:'1e-3', biasClip:'5.0',  enabled:true,  comps:['scale','spike','lat','temp']},
  {key:'baro',name:'Barometer',     unit:'m',    rate:'50',   sigma:'0.10', biasWalk:'1e-3', biasClip:'2.0',  enabled:true,  comps:['spike','lat','temp']},
  {key:'gps', name:'GPS',           unit:'m',    rate:'10',   sigma:'0.50', biasWalk:'0',    biasClip:'0',    enabled:false, comps:['spike','lat']},
];
function buildSensors(){
  const host=document.getElementById('sensorList'); if(!host) return;
  host.innerHTML = SENSORS.map(s=>{
    const gauss = `
        <div class="setting"><div><div class="name">White σ</div><div class="desc">RMS, ${s.unit}</div></div><div class="spin"><input class="txt" value="${s.sigma}"><div class="sb">▴▾</div></div></div>
        <div class="setting"><div><div class="name">Bias random-walk</div><div class="desc">drift rate, ${s.unit}/√s</div></div><div class="spin"><input class="txt" value="${s.biasWalk}"><div class="sb">▴▾</div></div></div>
        <div class="setting"><div><div class="name">Bias clip</div><div class="desc">max drift, ${s.unit}</div></div><div class="spin"><input class="txt" value="${s.biasClip}"><div class="sb">▴▾</div></div></div>`;
    const csv = `<div class="setting"><div><div class="name">Noise CSV</div><div class="desc">recorded samples replayed circularly — use a <b>seamless</b> file (ends where it starts)</div></div><button class="tb-btn" style="border-color:var(--border-strong)">Load .csv…</button></div>`;
    const comps = s.comps.map(c=>{ const d=SENS_COMPS[c];
      const inputs = d.fields.map(v=>`<input class="mono" value="${v}">`).join('');
      return `<div class="setting comp-row"><div style="display:flex; align-items:center; gap:8px;"><div class="chk" onclick="this.classList.toggle('on'); this.closest('.comp-row').classList.toggle('comp-on')"></div><div><div class="name">${d.label}</div><div class="desc">${d.desc}</div></div></div><div class="axis-edit">${inputs}</div></div>`;
    }).join('');
    return `<div class="mcoef" id="sensor-${s.key}">
        <div class="mcoef-head" onclick="toggleSensor('${s.key}')">
          <span class="mcoef-caret">▸</span>
          <span class="chk live-toggle${s.enabled?' on':''}" onclick="event.stopPropagation(); this.classList.toggle('on')" title="Enable sensor (off = dropout fault — applies live)"></span>
          <span class="mcoef-name">${s.name}</span>
          <span class="mcoef-sum mono">${s.rate} Hz · σ ${s.sigma} ${s.unit}</span>
        </div>
        <div class="mcoef-body">
          <div class="setting"><div><div class="name">Rate</div><div class="desc">structural — locked while running</div></div><div class="spin"><input class="txt struct-field" value="${s.rate}"><div class="sb">▴▾</div></div></div>
          <div class="setting"><div class="name">Noise source</div><div class="sw2" id="nsrc-${s.key}"><span class="on" onclick="setNoiseSrc('${s.key}','gauss')">Gaussian</span><span onclick="setNoiseSrc('${s.key}','csv')">CSV replay</span></div></div>
          <div id="gauss-${s.key}">${gauss}</div>
          <div id="csv-${s.key}" class="hidden">${csv}</div>
          <div style="margin:6px 0 2px; font-size:9px; color:var(--text-dim); text-transform:uppercase; letter-spacing:.4px;">Additional error models</div>
          ${comps}
        </div>
      </div>`;
  }).join('');
}
function toggleSensor(k){ document.getElementById('sensor-'+k).classList.toggle('open'); }
function setNoiseSrc(k, src){
  const sw=document.getElementById('nsrc-'+k);
  sw.querySelectorAll('span').forEach((s,i)=> s.classList.toggle('on', (i===0)===(src==='gauss')));
  document.getElementById('gauss-'+k).classList.toggle('hidden', src!=='gauss');
  document.getElementById('csv-'+k).classList.toggle('hidden', src!=='csv');
}
buildSensors();

/* ===== World magnetic field — N/E/D from intensity · declination · inclination ===== */
function updateMagVec(){
  const mag=parseFloat((document.getElementById('magMag')||{}).value)||0;
  const dec=((parseFloat((document.getElementById('magDec')||{}).value)||0))*Math.PI/180;
  const inc=((parseFloat((document.getElementById('magInc')||{}).value)||0))*Math.PI/180;
  const h=mag*Math.cos(inc);
  const N=h*Math.cos(dec), E=h*Math.sin(dec), D=mag*Math.sin(inc);
  const el=document.getElementById('magVec'); if(el) el.textContent=`${N.toFixed(1)}, ${E.toFixed(1)}, ${D.toFixed(1)}`;
}
updateMagVec();

/* ===== RC UART connect / disconnect toggle ===== */
function toggleRcUart(){
  const b=document.getElementById('rcUartBtn'); if(!b) return;
  const connected = b.textContent.trim()==='Disconnect';
  if(connected){ b.textContent='Connect'; b.className='tb-btn'; b.style.borderColor='var(--border-strong)'; showToast('info','RC UART disconnected'); }
  else { b.textContent='Disconnect'; b.className='tb-btn danger'; b.style.borderColor=''; showToast('ok','RC UART connected'); }
}

/* ===== GPS glitch fault ===== */
function toggleGpsGlitch(){
  const c=document.getElementById('gpsGlitchEn'); c.classList.toggle('on');
  document.getElementById('gpsGlitchOpts').classList.toggle('hidden', !c.classList.contains('on'));
}
function setGpsGlitchMode(m){
  const sel=document.getElementById('gpsGlitchMode'); if(sel && sel.value!==m) sel.value=m;
  document.querySelectorAll('#gpsGlitchOpts .gg-mode').forEach(e=> e.classList.toggle('hidden', e.dataset.gg!==m));
}

/* ===== Apply UX — when does the GCS push a change to the sim? =====
   Tier rule: settings ADDED ON TOP of the live state each tick (noise, env
   forces, obstacles) hot-apply via a per-group "Apply to Sim" button; settings
   that DEFINE THE PLANT (mass, geometry, motor coeffs, meshes, sensor rate) are
   structural — locked while the sim runs, applied on the next Start/Reset.
   Faults are immediate (the toggle itself is the apply). */
const HOT_GROUPS    = ['Environment', 'Wind & Turbulence', 'Air & Thrust', 'Magnetic Field', 'Sensor Models', 'Obstacles'];
const STRUCT_GROUPS = ['3D Model', 'Mass & Inertia', 'Motors — Quad X (positions in m)', 'Motor Coefficients — per motor', 'Terrain Mesh'];
function updateTabBadges(){
  ['vehicle','world','autotune'].forEach(t=>{
    const page=document.getElementById('simtab-'+t), tab=document.querySelector('#page-sim .tab[data-tab="'+t+'"]');
    if(page && tab) tab.classList.toggle('dirty', !!page.querySelector('.gframe.dirty'));
  });
}
function setGroupDirty(g, dirty){
  g.classList.toggle('dirty', dirty);
  const btn=g.querySelector('.apply-btn'); if(btn) btn.disabled=!dirty;
  const msg=g.querySelector('.apply-msg'); if(msg) msg.textContent = dirty ? '● unsaved changes' : 'in sync with sim';
  updateTabBadges();
}
function applyGroup(g){ setGroupDirty(g, false); showToast('ok', (g.getAttribute('data-apply')||'Settings')+' applied to sim'); }
function applyRunningLocks(){
  document.querySelectorAll('#page-sim .gframe[data-structural]').forEach(g=> g.classList.toggle('locked', simRunning));
  document.querySelectorAll('.struct-field').forEach(i=> i.disabled = simRunning);
}
function setupApplyUX(){
  document.querySelectorAll('#page-sim .gframe').forEach(g=>{
    const t=g.querySelector(':scope > .gtitle'); if(!t) return;
    const title=t.textContent.trim();
    if(STRUCT_GROUPS.includes(title)){
      g.setAttribute('data-structural','1');
      const h=document.createElement('div'); h.className='lock-hint';
      h.textContent='🔒 Sim running — stop to edit (applies on Start/Reset)';
      g.insertBefore(h, t.nextSibling);
    } else if(HOT_GROUPS.includes(title)){
      g.setAttribute('data-apply', title);
      const bar=document.createElement('div'); bar.className='apply-bar';
      bar.innerHTML='<span class="apply-dot"></span><span class="apply-msg">in sync with sim</span>'
        + '<button class="tb-btn apply-btn" disabled onclick="applyGroup(this.closest(\'.gframe\'))">Apply to Sim</button>';
      g.appendChild(bar);
    }
  });
  const dirtyFromEvent = e => { const g=e.target.closest && e.target.closest('.gframe[data-apply]'); if(g) setGroupDirty(g, true); };
  document.addEventListener('input', dirtyFromEvent);
  document.addEventListener('change', dirtyFromEvent);
  document.addEventListener('click', e=>{
    const t=e.target.closest && e.target.closest('.chk,.tbtn,.tb-btn,.sw2 span,.sw3 span,.spin .sb');
    if(!t || t.classList.contains('live-toggle') || (t.closest && t.closest('.apply-bar'))) return;  // live/Apply itself: not staged
    const g=t.closest('.gframe[data-apply]'); if(g) setGroupDirty(g, true);
  });
  applyRunningLocks();
}
setupApplyUX();

/* ===== Kernel Perf — mirrors PerfWidget ===== */
const ST_COL={READY:'#61AFEF', RUNNING:'#98C379', BLOCKED:'#E06C75', DELAYED:'#D19A66'};
const PERF_TASKS=[
  {name:'control_loop', state:'RUNNING', base:34, stackPeak:1180, stackSize:2048},
  {name:'estimator',    state:'READY',   base:18, stackPeak:1460, stackSize:2048},
  {name:'imu_driver',   state:'READY',   base:12, stackPeak:760,  stackSize:1024},
  {name:'rc_input',     state:'BLOCKED', base:4,  stackPeak:512,  stackSize:1024},
  {name:'telemetry',    state:'DELAYED', base:7,  stackPeak:980,  stackSize:1536},
  {name:'comms_tx',     state:'READY',   base:5,  stackPeak:640,  stackSize:1024},
  {name:'logger',       state:'BLOCKED', base:2,  stackPeak:420,  stackSize:768},
  {name:'idle',         state:'READY',   base:18, stackPeak:128,  stackSize:512},
];
const PERF_FIFOS=['imu.raw','imu.telemetry','imu.control','imu.calib','imu.calib_telem','attitude.telemetry','attitude.control','rc.telemetry','rc.control','control.telemetry']
  .map((n,i)=>({name:n, cap:[64,128,64,32,32,128,64,64,64,128][i], base:[40,55,30,8,8,60,35,20,18,50][i], drops:i===4?1240:0}));
let perfSeq=0, perfSelTask=0, perfSelQueue=-1; const perfHist={};
function pctBar(pct,col){ return `<span class="cellbar"><span class="cb-fill" style="width:${Math.min(100,pct)}%;background:${col}"></span><span>${Math.round(pct)}%</span></span>`; }
function stackCol(p){ return p>85?'#E06C75':p>65?'#D19A66':'#98C379'; }
function fillCol(p){ return p>85?'#E06C75':p>60?'#D19A66':'#4ECDC4'; }
function perfSelect(kind,idx){ if(kind==='t') perfSelTask=idx; else perfSelQueue=idx; }
function perfTick(ph){
  const page=document.getElementById('page-perf'); if(!page || !page.classList.contains('active')) return;
  perfSeq++; setTxt('perfSeq', perfSeq);
  const cpu=Math.max(0, 38 + Math.sin(ph*0.3)*6 + Math.sin(ph)*1.5), heap=63 + Math.sin(ph*0.1)*3, ipc=Math.max(0, 3 + Math.sin(ph*0.5)*2);
  const up=640+Math.floor(perfSeq/5);
  // --- CPU · Scheduler resource box: headline + grouped scheduler stats + history graph ---
  const ch=perfHist['cpu']=(perfHist['cpu']||[]); ch.push(cpu); if(ch.length>60) ch.shift();
  lsPlot('gCpuHist', [{c: cpu>80?'#E06C75':'#61AFEF', fn:i=> ch[i] ?? cpu}], 0, 100);
  setTxt('pCpuPct', cpu.toFixed(1)+'%'); setTxt('pCpuSub', `idle ${(100-cpu).toFixed(1)}%`);
  setTxt('pCpuStats', `ctx ${(184000+perfSeq*47).toLocaleString()} · systick 1820cyc/10.9µs · ipc ${ipc.toFixed(0)}% · uptime ${up}s`);
  // --- Heap · RAM box: Total/Used/Peak/Free meters + history graph ---
  const hh=perfHist['heap']=(perfHist['heap']||[]); hh.push(heap); if(hh.length>60) hh.shift();
  lsPlot('gHeapHist', [{c:'#C678DD', fn:i=> hh[i] ?? heap}], 0, 100);
  const usedKB=Math.round(heap/100*48), freeKB=48-usedKB, freePct=Math.max(0,100-heap);
  const hb=document.getElementById('pHeapBox');
  if(hb) hb.innerHTML =
    `<div class="mem-line"><span class="mk">Total</span><span></span><span class="mv">48 KB</span></div>`+
    `<div class="mem-line"><span class="mk">Used</span>${gbar(heap)}<span class="mv">${usedKB} KB · ${heap.toFixed(0)}%</span></div>`+
    `<div class="mem-line"><span class="mk">Peak</span><span></span><span class="mv">31 KB</span></div>`+
    `<div class="mem-line"><span class="mk">Free</span>${gbar(freePct)}<span class="mv">${freeKB} KB · ${freePct.toFixed(0)}%</span></div>`+
    `<div class="mem-extra mono">1240 alloc · 1198 free · 0 oom</div>`;
  // --- Tasks (proc list) ---
  const tb=document.querySelector('#perfTaskTable tbody');
  tb.innerHTML=PERF_TASKS.map((t,i)=>{ const c=Math.max(0, t.base + Math.sin(ph*0.6+i)*3), sp=Math.round(t.stackPeak/t.stackSize*100);
    const h=perfHist['t'+i]=(perfHist['t'+i]||[]); h.push(c); if(h.length>60) h.shift();
    const hs=perfHist['ts'+i]=(perfHist['ts'+i]||[]); hs.push(sp + Math.sin(ph*0.3+i)*0.6); if(hs.length>60) hs.shift();  // stack high-water (gentle drift)
    return `<tr onclick="perfSelect('t',${i})" class="${i===perfSelTask?'sel':''}"><td class="mono">${t.name}</td><td><span class="st-pill" style="color:${ST_COL[t.state]}">● ${t.state}</span></td><td style="width:116px">${pctCell(c)}</td><td style="width:116px">${pctCell(sp)}</td></tr>`;
  }).join('');
  // --- Queues · FIFOs (disks box): per-ring fill bar + drops ---
  const pq=document.getElementById('pQueues');
  if(pq) pq.innerHTML=PERF_FIFOS.map((f,i)=>{ const fill=Math.max(0,Math.min(100, f.base + Math.sin(ph*0.7+i*1.3)*12));
    const h=perfHist['f'+i]=(perfHist['f'+i]||[]); h.push(fill); if(h.length>60) h.shift();
    return `<div class="q-row ${i===perfSelQueue?'sel':''}" onclick="perfSelect('f',${i})"><span class="q-name">${f.name}</span>${pctCell(fill)}<span class="q-drops" style="color:${f.drops?'var(--danger)':'var(--text-dim)'}">${f.drops?f.drops:'0'}</span></div>`;
  }).join('');
  renderSelGraphs();
}
// btop-style gradient meter bar (green→amber→red, anchored to the full 0–100 scale)
function gbar(pct){ pct=Math.max(0,Math.min(100,pct)); const bs=pct>1?(10000/pct).toFixed(0):100;
  return `<div class="gbar"><i style="width:${pct}%; background-size:${bs}% 100%"></i></div>`; }
// shared bar+% cell so Tasks CPU/Stack and Queues Fill render identically
function pctCell(pct){ return `<div class="pc-cell">${gbar(pct)}<span class="pc-n">${Math.round(pct)}%</span></div>`; }
// Click a task / queue → plot its recorded history in the free space below the list.
function renderSelGraphs(){
  // Selected queue → fill-% history (centre column)
  if(perfSelQueue>=0){ const f=PERF_FIFOS[perfSelQueue], fh=perfHist['f'+perfSelQueue]||[];
    setTxt('qSelCap', `${f.name} — fill % · cap ${f.cap} · drops ${f.drops}`);
    lsPlot('gQueueSel', [{c:'#4ECDC4', fn:i=> fh[i] ?? (fh[fh.length-1]||0)}], 0, 100);
  } else { setTxt('qSelCap','Click a queue to plot its fill history'); lsPlot('gQueueSel', [{c:'#4ECDC4', fn:()=>0}], 0, 100); }
  // Selected task → CPU-% history (right column)
  if(perfSelTask>=0){ const t=PERF_TASKS[perfSelTask], th=perfHist['t'+perfSelTask]||[], ts=perfHist['ts'+perfSelTask]||[];
    setTxt('tSelCap', `${t.name} — CPU % (blue) vs Stack % (amber)`);
    lsPlot('gTaskSel', [{c:'#61AFEF', fn:i=> th[i] ?? (th[th.length-1]||0)}, {c:'#E5C07B', fn:i=> ts[i] ?? (ts[ts.length-1]||0)}], 0, 100);
  } else { setTxt('tSelCap','Click a task to plot its CPU + stack history'); lsPlot('gTaskSel', [{c:'#61AFEF', fn:()=>0}], 0, 100); }
}

/* ===================================================================
   Keyboard Shortcuts — rich, GCS-standard set + a VS Code–style
   editable keybindings panel. Bindings persist to localStorage; the
   global dispatcher reads the *effective* (possibly remapped) keymap.
   Conventions drawn from QGroundControl / Mission Planner / Auterion
   plus media-standard transport keys for log replay.
   =================================================================== */

const KB_INREPLAY = () => replayMode;             // guard: replay-only commands
const KB_DEFAULTS = [
  // ── Views ──────────────────────────────────────────────────────
  { id:'view.dash',     cat:'Views',                cmd:'Open Flight Dashboard',      keys:'Ctrl+1',           when:'always' },
  { id:'view.rc',       cat:'Views',                cmd:'Open RC Channels',           keys:'Ctrl+2',           when:'always' },
  { id:'view.motors',   cat:'Views',                cmd:'Open Motors',                keys:'Ctrl+3',           when:'always' },
  { id:'view.control',  cat:'Views',                cmd:'Open Control Loop',          keys:'Ctrl+4',           when:'always' },
  { id:'view.packets',  cat:'Views',                cmd:'Open Packet Analyzer',       keys:'Ctrl+5',           when:'always' },
  { id:'view.sim',      cat:'Views',                cmd:'Open Simulator',             keys:'Ctrl+6',           when:'always' },
  { id:'view.calib',    cat:'Views',                cmd:'Open Calibration',           keys:'Ctrl+7',           when:'always' },
  { id:'view.perf',     cat:'Views',                cmd:'Open Kernel Perf',           keys:'Ctrl+8',           when:'always' },
  { id:'view.settings', cat:'Views',                cmd:'Open Settings',              keys:'Ctrl+,',           when:'always' },
  { id:'view.mru',      cat:'Views',                cmd:'Switch to Recent View (hold to cycle)', keys:'Ctrl+`',    when:'always' },
  { id:'view.next',     cat:'Views',                cmd:'Next View',                  keys:'Ctrl+]',           when:'always' },
  { id:'view.prev',     cat:'Views',                cmd:'Previous View',              keys:'Ctrl+[',           when:'always' },
  { id:'view.palette',  cat:'Views',                cmd:'Command Palette',            keys:'Ctrl+Shift+P',     when:'always' },
  { id:'view.fullscreen',cat:'Views',               cmd:'Toggle Full Screen',         keys:'F11',              when:'always' },
  { id:'view.resetLayout',cat:'Views',              cmd:'Reset Panel Layout',         keys:'Ctrl+Shift+0',     when:'always' },
  // ── Connection ─────────────────────────────────────────────────
  { id:'link.toggle',   cat:'Connection',           cmd:'Connect / Disconnect Link',  keys:'Ctrl+K',           when:'always' },
  { id:'link.transport',cat:'Connection',           cmd:'Cycle Link Transport',       keys:'Ctrl+Alt+L',       when:'always' },
  { id:'link.refresh',  cat:'Connection',           cmd:'Refresh Serial Ports',       keys:'Ctrl+R',           when:'always' },
  // ── Vehicle commands ───────────────────────────────────────────
  { id:'veh.arm',       cat:'Vehicle',              cmd:'Arm / Disarm',               keys:'Ctrl+Shift+A',     when:'connected && !replay' },
  { id:'veh.takeoff',   cat:'Vehicle',              cmd:'Takeoff',                    keys:'Ctrl+Shift+T',     when:'armed' },
  { id:'veh.land',      cat:'Vehicle',              cmd:'Land',                       keys:'Ctrl+Shift+L',     when:'flying' },
  { id:'veh.rtl',       cat:'Vehicle',              cmd:'Return to Launch (RTL)',     keys:'Ctrl+Shift+R',     when:'flying' },
  { id:'veh.hold',      cat:'Vehicle',              cmd:'Hold / Loiter',              keys:'Ctrl+Shift+H',     when:'flying' },
  { id:'veh.pause',     cat:'Vehicle',              cmd:'Pause / Brake',              keys:'Ctrl+.',           when:'flying' },
  { id:'veh.mode',      cat:'Vehicle',              cmd:'Cycle Flight Mode',          keys:'Ctrl+Shift+M',     when:'connected' },
  { id:'veh.mission',   cat:'Vehicle',              cmd:'Start Mission',              keys:'Ctrl+Enter',       when:'armed' },
  { id:'veh.estop',     cat:'Vehicle',              cmd:'Emergency Stop (Kill)',      keys:'Ctrl+Shift+Backspace', when:'connected' },
  // ── Simulation & Tuning ────────────────────────────────────────
  { id:'tool.calibStart',cat:'Simulation & Tuning', cmd:'Run Calibration Wizard',     keys:'Ctrl+Shift+C',     when:'always' },
  { id:'sim.start',     cat:'Simulation & Tuning',  cmd:'Start Simulator (SITL)',     keys:'F5',               when:'always' },
  { id:'sim.stop',      cat:'Simulation & Tuning',  cmd:'Stop Simulator',             keys:'Shift+F5',         when:'sim running' },
  { id:'tune.start',    cat:'Simulation & Tuning',  cmd:'Start Autotune',             keys:'Ctrl+Shift+U',     when:'sim running' },
  { id:'tune.stop',     cat:'Simulation & Tuning',  cmd:'Stop Autotune',              keys:'Ctrl+Alt+U',       when:'tuning' },
  { id:'tune.apply',    cat:'Simulation & Tuning',  cmd:'Apply Proposed Gains',       keys:'Ctrl+Shift+G',     when:'tune complete' },
  // ── Log Replay (media-standard transport) ──────────────────────
  { id:'replay.open',   cat:'Log Replay',           cmd:'Open Log for Replay…',       keys:'Ctrl+O',           when:'always' },
  { id:'replay.exit',   cat:'Log Replay',           cmd:'Exit Replay → Live',         keys:'Esc',     when:'replay', guard:KB_INREPLAY },
  { id:'replay.play',   cat:'Log Replay',           cmd:'Play / Pause',               keys:'Space',   when:'replay', guard:KB_INREPLAY },
  { id:'replay.stepB',  cat:'Log Replay',           cmd:'Step Back 5 s',              keys:'Left',    when:'replay', guard:KB_INREPLAY },
  { id:'replay.stepF',  cat:'Log Replay',           cmd:'Step Forward 5 s',           keys:'Right',   when:'replay', guard:KB_INREPLAY },
  { id:'replay.toStart',cat:'Log Replay',           cmd:'Jump to Crop Start',         keys:'Home',    when:'replay', guard:KB_INREPLAY },
  { id:'replay.toEnd',  cat:'Log Replay',           cmd:'Jump to Crop End',           keys:'End',     when:'replay', guard:KB_INREPLAY },
  { id:'replay.cropIn', cat:'Log Replay',           cmd:'Set Crop In at Playhead',    keys:'I',       when:'replay', guard:KB_INREPLAY },
  { id:'replay.cropOut',cat:'Log Replay',           cmd:'Set Crop Out at Playhead',   keys:'O',       when:'replay', guard:KB_INREPLAY },
  { id:'replay.slower', cat:'Log Replay',           cmd:'Decrease Replay Speed',      keys:'[',       when:'replay', guard:KB_INREPLAY },
  { id:'replay.faster', cat:'Log Replay',           cmd:'Increase Replay Speed',      keys:']',       when:'replay', guard:KB_INREPLAY },
  // ── Map / View ─────────────────────────────────────────────────
  { id:'map.center',    cat:'Map',                  cmd:'Center on Vehicle',          keys:'Ctrl+Home',        when:'always' },
  { id:'map.zoomIn',    cat:'Map',                  cmd:'Zoom In',                    keys:'Ctrl+=',           when:'always' },
  { id:'map.zoomOut',   cat:'Map',                  cmd:'Zoom Out',                   keys:'Ctrl+-',           when:'always' },
  // ── Application ────────────────────────────────────────────────
  { id:'app.docs',      cat:'Application',          cmd:'Documentation',              keys:'F1',               when:'always' },
  { id:'app.keys',      cat:'Application',          cmd:'Keyboard Shortcuts',         keys:'Ctrl+Alt+K',       when:'always' },
  { id:'app.about',     cat:'Application',          cmd:'About Navigator',            keys:'',                 when:'always' },
  { id:'dev.analysis',  cat:'Application',          cmd:'Implementation-Analysis Mode', keys:'Ctrl+D',         when:'always' },
  { id:'app.quit',      cat:'Application',          cmd:'Quit',                       keys:'Ctrl+Q',           when:'always' },
];

/* effective keymap = defaults with any saved overrides applied */
function kbLoad(){ try { return JSON.parse(localStorage.getItem('vayu.keys')||'{}'); } catch(e){ return {}; } }
function kbSave(){ try { localStorage.setItem('vayu.keys', JSON.stringify(kbOverrides)); } catch(e){} }
let kbOverrides = kbLoad();
function kbDef(id){ return KB_DEFAULTS.find(d => d.id === id); }
function effKeys(d){ return (d.id in kbOverrides) ? kbOverrides[d.id] : d.keys; }
function currentKeymap(){ return KB_DEFAULTS.map(d => ({ ...d, keys: effKeys(d) })); }

/* commands that actually do something in the mock; the rest toast their name */
const KB_PAGES = ['dash','rc','motors','control','packets','sim','calib','perf','settings'];
function kbCurPage(){ const el = document.querySelector('.page.active'); return el ? el.id.replace('page-','') : 'dash'; }
function rpSpeedStep(d){ const s = document.getElementById('rpSpeed'); if(!s) return; s.selectedIndex = Math.max(0, Math.min(s.options.length-1, s.selectedIndex + d)); showToast('info','Replay speed → '+s.value); }
const KB_ACTIONS = {
  'view.dash':()=>go('dash'),   'view.rc':()=>go('rc'),       'view.motors':()=>go('motors'),
  'view.control':()=>go('control'),'view.packets':()=>go('packets'),'view.sim':()=>go('sim'),
  'view.calib':()=>go('calib'), 'view.perf':()=>go('perf'),   'view.settings':()=>go('settings'),
  'view.next':()=>{ const i=KB_PAGES.indexOf(kbCurPage()); go(KB_PAGES[(i+1)%KB_PAGES.length]); },
  'view.prev':()=>{ const i=KB_PAGES.indexOf(kbCurPage()); go(KB_PAGES[(i-1+KB_PAGES.length)%KB_PAGES.length]); },
  'link.toggle':()=>toggleConn(),
  'veh.arm':()=>toggleArm(),
  'tool.calibStart':()=>go('calib'),
  'app.keys':()=>openShortcuts(),
  'app.docs':()=>openModal('docsModal'),
  'app.about':()=>openModal('aboutModal'),
  'dev.analysis':()=>toggleAnalysis(),
  'replay.open':()=>{ if(!replayMode) enterReplay(); else showToast('info','Already replaying'); },
  'replay.exit':()=>exitReplay(),
  'replay.play':()=>rpToggle(),
  'replay.stepB':()=>rpStep(-5),
  'replay.stepF':()=>rpStep(5),
  'replay.toStart':()=>rpSkip(0),
  'replay.toEnd':()=>rpSkip(1),
  'replay.cropIn':()=>{ cropA=Math.max(0,Math.min(rpPos/rpTotal, cropB-0.03)); rpUpdate(); showToast('info','Crop-in set at playhead'); },
  'replay.cropOut':()=>{ cropB=Math.min(1,Math.max(rpPos/rpTotal, cropA+0.03)); rpUpdate(); showToast('info','Crop-out set at playhead'); },
  'replay.slower':()=>rpSpeedStep(-1),
  'replay.faster':()=>rpSpeedStep(1),
};
function runCommand(b){ const fn = KB_ACTIONS[b.id]; if(fn){ fn(); } else { showToast('info','⌘ '+b.cmd); } }

/* KeyboardEvent → canonical "Ctrl+Shift+K" string (matches binding format) */
const KB_KEYMAP = { ' ':'Space','Escape':'Esc','ArrowLeft':'Left','ArrowRight':'Right','ArrowUp':'Up','ArrowDown':'Down','Enter':'Enter','Tab':'Tab','Backspace':'Backspace','Home':'Home','End':'End','Delete':'Delete' };
function keyEventToStr(e){
  let k = e.key;
  if(k==='Control'||k==='Alt'||k==='Shift'||k==='Meta') return '';   // modifier alone
  const parts = [];
  if(e.ctrlKey)  parts.push('Ctrl');
  if(e.altKey)   parts.push('Alt');
  if(e.shiftKey) parts.push('Shift');
  if(KB_KEYMAP[k]) k = KB_KEYMAP[k]; else if(k.length===1) k = k.toUpperCase();
  parts.push(k);
  return parts.join('+');
}
function kbModPrefix(e){ const p=[]; if(e.ctrlKey)p.push('Ctrl'); if(e.altKey)p.push('Alt'); if(e.shiftKey)p.push('Shift'); return p.length ? p.join('+')+'+' : ''; }

/* ── editor rendering ── */
function keysToKbd(s){ return s.split('+').map(p => '<kbd class="kbk">'+p+'</kbd>').join('+'); }
function openShortcuts(){ closeMenus(); const m=document.getElementById('kbModal'); m.classList.remove('hidden'); const s=document.getElementById('kbSearch'); s.value=''; renderKb(); setTimeout(()=>s.focus(),0); axRefresh(); }
function closeShortcuts(){ if(kbCancelRec) kbCancelRec(); document.getElementById('kbModal').classList.add('hidden'); axRefresh(); }
function renderKb(){
  const q = (document.getElementById('kbSearch').value||'').trim().toLowerCase();
  const km = currentKeymap();
  const modN = km.filter(k => k.keys !== kbDef(k.id).keys).length;
  document.getElementById('kbModCount').textContent = modN ? (modN+' customized — saved to this browser') : 'All defaults';
  const cats = []; const byCat = {};
  km.forEach(k => { if(!byCat[k.cat]){ byCat[k.cat]=[]; cats.push(k.cat); } byCat[k.cat].push(k); });
  let html = '', shown = 0;
  cats.forEach(cat => {
    const rows = byCat[cat].filter(k => !q || k.cmd.toLowerCase().includes(q) || (k.keys||'').toLowerCase().includes(q) || cat.toLowerCase().includes(q) || (k.when||'').toLowerCase().includes(q));
    if(!rows.length) return;
    html += '<div class="kb-cat">'+cat+'</div>';
    rows.forEach(k => {
      shown++;
      const def = kbDef(k.id), isMod = k.keys !== def.keys, rec = kbRecording === k.id;
      let chip;
      if(rec)         chip = '<span class="kb-keys recording" id="kbk-'+k.id+'">Press keys…</span>';
      else if(!k.keys)chip = '<span class="kb-keys empty" onclick="kbRecord(\''+k.id+'\')">Unassigned</span>';
      else            chip = '<span class="kb-keys" onclick="kbRecord(\''+k.id+'\')" title="Click to change">'+keysToKbd(k.keys)+'</span>';
      html += '<div class="kb-row'+(isMod?' mod':'')+'">'
            +   '<span class="kb-cmd">'+k.cmd+'</span>'
            +   '<span class="kb-when">'+(k.when||'')+'</span>'
            +   chip
            +   '<button class="kb-reset" title="Reset to default ('+(def.keys||'Unassigned')+')" onclick="kbResetOne(\''+k.id+'\')">↺</button>'
            + '</div>';
    });
  });
  if(!shown) html = '<div class="kb-empty-msg">No commands match “'+q+'”.</div>';
  document.getElementById('kbBody').innerHTML = html;
}

/* ── recording a new binding (VS Code: click → press combo) ── */
let kbRecording = null, kbCancelRec = null;
function kbRecord(id){
  if(kbCancelRec) kbCancelRec();
  kbRecording = id;
  const onKey = e => {
    e.preventDefault(); e.stopPropagation();
    if(e.key === 'Escape'){ finish(null); return; }
    if(['Control','Alt','Shift','Meta'].includes(e.key)){               // live-preview modifiers
      const el = document.getElementById('kbk-'+id); if(el) el.textContent = kbModPrefix(e)+'…'; return;
    }
    const str = keyEventToStr(e); if(str) finish(str);
  };
  const finish = str => { document.removeEventListener('keydown', onKey, true); kbRecording=null; kbCancelRec=null; if(str) setBinding(id, str); renderKb(); };
  kbCancelRec = () => finish(null);
  document.addEventListener('keydown', onKey, true);
  renderKb();
}
function setBinding(id, keys){
  const clash = currentKeymap().find(k => k.id !== id && k.keys && k.keys === keys);
  if(clash){ kbOverrides[clash.id] = ''; showToast('warn', keys+' reassigned from “'+clash.cmd+'”'); }
  const def = kbDef(id);
  if(keys === def.keys) delete kbOverrides[id]; else kbOverrides[id] = keys;
  kbSave(); syncMenuHints();
}
function kbResetOne(id){ delete kbOverrides[id]; kbSave(); renderKb(); syncMenuHints(); }
function kbResetAll(){ kbOverrides = {}; kbSave(); renderKb(); syncMenuHints(); showToast('info','All shortcuts reset to defaults'); }

/* keep the menus' shortcut hints in sync with the (remapped) keymap */
function syncMenuHints(){
  const map = { dash:'view.dash', rc:'view.rc', motors:'view.motors', control:'view.control', packets:'view.packets', sim:'view.sim', calib:'view.calib', perf:'view.perf' };
  const km = currentKeymap();
  document.querySelectorAll('.mi[data-page]').forEach(mi => {
    const id = map[mi.dataset.page]; if(!id) return;
    const k = km.find(x => x.id === id), sc = mi.querySelector('.sc');
    if(sc && k) sc.textContent = k.keys || '';
  });
}

/* ── global dispatcher ── */
document.addEventListener('keydown', e => {
  if(kbRecording) return;                                   // capture handler owns the key
  const modal = document.getElementById('kbModal');
  const modalOpen = modal && !modal.classList.contains('hidden');
  const str = keyEventToStr(e); if(!str) return;
  if(modalOpen){ if(str==='Esc'){ e.preventDefault(); closeShortcuts(); } return; }
  const openDlg = document.querySelector('.modal:not(.hidden)');   // About / Documentation
  if(openDlg){ if(e.key==='Escape'){ e.preventDefault(); openDlg.classList.add('hidden'); } return; }
  // MRU recent-views switcher: hold Ctrl, tap ` to cycle (Shift reverses), Esc cancels.
  const mruB = currentKeymap().find(k => k.id === 'view.mru'), mruKeys = mruB ? mruB.keys : '';
  const mruRev = mruKeys ? (mruKeys.includes('Shift+') ? mruKeys.replace('Shift+','') : mruKeys.replace(/^((?:Ctrl\+)?(?:Alt\+)?)/,'$1Shift+')) : '';
  if(mruActive && e.key === 'Escape'){ e.preventDefault(); mruCancel(); return; }   // Ctrl is held → str is "Ctrl+Esc", match the raw key
  if(mruKeys && (str === mruKeys || (mruRev && str === mruRev))){ e.preventDefault(); if(!e.repeat) mruStep(str === mruRev ? -1 : 1); return; }
  const tag = (e.target.tagName||''), inField = /^(INPUT|TEXTAREA|SELECT)$/.test(tag) || e.target.isContentEditable;
  const hasMod = e.ctrlKey || e.altKey || e.metaKey;
  const b = currentKeymap().find(k => k.keys && k.keys === str);
  if(!b) return;
  if(b.id === 'view.fullscreen') return;                    // let the browser handle native F11 fullscreen
  if(inField && !hasMod && str !== 'Esc') return;           // don't steal plain typing
  if(b.guard && !b.guard()) return;                         // context gate (e.g. replay-only)
  e.preventDefault();
  runCommand(b);
});

/* ===================================================================
   MRU recent-views switcher — Firefox/VS Code style.
   • Ctrl+`           → switch to the most-recently-used view (tap = toggle)
   • hold Ctrl, tap ` → cycle through the last N views (overlay appears)
   • Shift            → cycle backwards
   • release Ctrl     → commit · Esc → cancel
   N is configurable in Settings → Units & Display (vayu.mruCount, default 5).
   =================================================================== */
const MRU_META = {
  dash:    { label:'Flight Dashboard', icon:'<circle cx="12" cy="12" r="9"/><path d="M3 12h18"/>' },
  rc:      { label:'RC Channels',      icon:'<circle cx="12" cy="12" r="3"/><path d="M4 12h4M16 12h4M12 4v4M12 16v4"/>' },
  motors:  { label:'Motors',           icon:'<circle cx="7" cy="7" r="3"/><circle cx="17" cy="7" r="3"/><circle cx="7" cy="17" r="3"/><circle cx="17" cy="17" r="3"/>' },
  control: { label:'Control Loop',     icon:'<path d="M3 18l5-9 4 5 3-7 6 11"/>' },
  packets: { label:'Packet Analyzer',  icon:'<path d="M4 6h16M4 12h16M4 18h10"/>' },
  sim:     { label:'Simulator',        icon:'<path d="M2 12s3-7 10-7 10 7 10 7-3 7-10 7-10-7-10-7z"/><circle cx="12" cy="12" r="3"/>' },
  calib:   { label:'Calibration',      icon:'<circle cx="12" cy="12" r="5"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>' },
  perf:    { label:'Kernel Perf',      icon:'<path d="M3 12h3l2-7 4 14 2-7h7"/>' },
  settings:{ label:'Settings',         icon:'<path d="M4 21v-7M4 10V3M12 21v-9M12 8V3M20 21v-5M20 12V3M1 14h6M9 8h6M17 16h6"/>' },
};
function mruCount(){ let v = parseInt(localStorage.getItem('vayu.mruCount')||'5', 10); if(!v || isNaN(v)) v = 5; return Math.max(2, Math.min(9, v)); }
function mruSetCount(v){ const n = Math.max(2, Math.min(9, parseInt(v,10)||5)); try { localStorage.setItem('vayu.mruCount', n); } catch(e){} const el = document.getElementById('setMruCount'); if(el) el.value = n; showToast('info','Recent-views switcher → '+n+' views'); }
function mruStepCount(){ mruSetCount(mruCount() >= 9 ? 2 : mruCount()+1); }   // the ▴▾ control bumps +1, wraps 9→2

let mruActive = false, mruSnap = [], mruSel = 0, mruShown = false, mruShowTimer = null;
function mruStep(dir){
  if(!mruActive){
    mruActive = true; mruSnap = mru.slice(0, mruCount()); mruSel = 0; mruShown = false;
    clearTimeout(mruShowTimer); mruShowTimer = setTimeout(() => { if(mruActive) mruShow(); }, 240);  // slight delay, like Firefox
  } else {
    clearTimeout(mruShowTimer); mruShow();    // a second tap = clearly cycling → reveal now
  }
  const n = Math.max(1, mruSnap.length);
  mruSel = (mruSel + dir + n) % n;
  if(mruShown) mruRender();
}
function mruShow(){ if(mruShown) return; mruShown = true; mruRender(); document.getElementById('mruSwitch').classList.remove('hidden'); }
function mruRender(){
  document.getElementById('mruList').innerHTML = mruSnap.map((p,i) => {
    const m = MRU_META[p] || { label:p, icon:'' };
    const tag = i===0 ? '<span class="mru-tag">current</span>' : '';
    return '<div class="mru-item'+(i===mruSel?' sel':'')+'"><svg viewBox="0 0 24 24">'+m.icon+'</svg><span class="mru-label">'+m.label+'</span>'+tag+'</div>';
  }).join('');
}
function mruCommit(){
  clearTimeout(mruShowTimer);
  const target = mruSnap[mruSel];
  document.getElementById('mruSwitch').classList.add('hidden');
  mruActive = false; mruShown = false;
  if(target) go(target);   // go() reorders the MRU so the chosen view becomes most-recent
}
function mruCancel(){ clearTimeout(mruShowTimer); document.getElementById('mruSwitch').classList.add('hidden'); mruActive = false; mruShown = false; }
window.addEventListener('keyup', e => { if(mruActive && (e.key === 'Control' || !e.ctrlKey)) mruCommit(); });
window.addEventListener('blur', () => { if(mruActive) mruCancel(); });

/* ===== Generic modals — About Navigator & Documentation ===== */
function openModal(id){ closeMenus(); if(id==='docsModal') renderDocs(); document.getElementById(id).classList.remove('hidden'); axRefresh(); }
function closeModal(id){ document.getElementById(id).classList.add('hidden'); axRefresh(); }

const DOC_TOPICS = [
  { group:'Getting Started', items:[
    ['Quick Start',          'Install, launch, and connect to your first vehicle'],
    ['Connecting a Vehicle', 'Serial / UDP links, baud rates, auto-reconnect'],
    ['Interface Tour',       'Views, docks, the toolbar and status bar'],
  ]},
  { group:'Flight Operations', items:[
    ['Flight Dashboard',     'Attitude, map, battery, GPS and live telemetry'],
    ['Arming & Flight Modes','Pre-arm checks, mode switching and failsafes'],
    ['RC Channels',          'Calibrating and monitoring transmitter input'],
  ]},
  { group:'Setup & Tuning', items:[
    ['Calibration Wizard',   'Accelerometer, gyro and magnetometer routines'],
    ['Motors & Mixer',       'Layout, per-motor coefficients and thrust axis'],
    ['Control Loop',         'PID structure and live loop telemetry'],
    ['Autotune',             'In-flight gain search and applying results'],
  ]},
  { group:'Simulation', items:[
    ['Simulator (SITL)',     'Running software-in-the-loop without hardware'],
    ['World & Vehicle Config','Wind, terrain, sensors, noise and fault injection'],
  ]},
  { group:'Diagnostics', items:[
    ['Packet Analyzer',      'Decoding and inspecting the NavLink stream'],
    ['Kernel Perf',          'Task CPU, stack, queues and heap on the FC'],
    ['Log Replay',           'Replaying a recorded flight with the crop tool'],
  ]},
  { group:'Reference', items:[
    ['Keyboard Shortcuts',   'Full key map — editable, with search'],
    ['Settings Reference',   'Every option in the Settings panel'],
    ['NavLink Protocol',     'Packet types, stream rates and framing'],
    ['Troubleshooting',      'Common link, calibration and SITL issues'],
  ]},
];
function renderDocs(){
  const ico = '<svg class="di-ico" viewBox="0 0 24 24"><path d="M6 2h9l5 5v15H6z"/><path d="M14 2v6h6"/></svg>';
  document.getElementById('docsBody').innerHTML = DOC_TOPICS.map(g =>
    '<div class="docs-group">'+g.group+'</div>' +
    g.items.map(([t,d]) =>
      '<div class="docs-item" onclick="docOpen(\''+t.replace(/'/g,"\\'")+'\')">'+ico+
        '<div><div class="di-t">'+t+'</div><div class="di-d">'+d+'</div></div>'+
        '<span class="di-arrow">→</span></div>'
    ).join('')
  ).join('');
}
function docOpen(t){
  if(t==='Keyboard Shortcuts'){ closeModal('docsModal'); openShortcuts(); return; }
  showToast('info','Opening documentation → '+t);
}

/* ===================================================================
   Implementation-analysis mode (Ctrl+D)
   A pure read-only overlay that classifies every meaningful piece of
   the mockup by how hard it is to make real in the production Qt GCS:
     • struct (red)  — needs an architectural rewrite of how the app works
     • add   (amber) — long but additive: new code that fits the architecture
     • easy  (green) — trivial to wire up in the real app
   Each tag (top-right corner) shows the three colour codes with the
   verdict lit; hovering reveals the reasoning, which hides on hover-out.
   =================================================================== */
const AX_RULES = [
  /* ── Whole-GCS log replay — the deepest piece ── */
  { sel:'#replayBar', lv:'struct', name:'Whole-GCS log replay', why:'Replay has to feed every widget from a recorded stream instead of the live link. The app binds widgets directly to the link, so this needs a swappable telemetry data-source, a seekable time-indexed playback engine, and a global read-only state that gates every command path. That is an architectural layer, not a widget.' },
  { sel:'#rpCropScrub,#rpCrop,#rpHStart,#rpHEnd,#rpOvPh', lv:'struct', name:'Replay crop / loop region', why:'Selecting and looping a sub-range requires the playback clock to be seekable and decoupled from wall-clock, with the log indexed by time. It rides entirely on the replay engine above.' },
  { sel:'#rpPlayScrub,#rpKnob,#rpFill,#rpSpeed,#rpCur,#rpCropEnd,#rpCropStart,#rpTot,.rp-btn', lv:'struct', name:'Replay transport', why:'Play / scrub / step / speed all drive a variable-rate playback clock over the indexed log — part of the replay engine, not standalone controls.' },

  /* ── In-GCS autotune (Python → C++ port) ── */
  { sel:'#ag-roll-0,#ag-roll-1,#ag-roll-2,#agc-roll-0,#agc-roll-1,#agc-roll-2,#ag-pitch-0,#ag-pitch-1,#ag-pitch-2,#agc-pitch-0,#agc-pitch-1,#agc-pitch-2,#ag-yaw-0,#ag-yaw-1,#ag-yaw-2,#agc-yaw-0,#agc-yaw-1,#agc-yaw-2', lv:'struct', name:'Autotune — current vs best gains', why:'Live current/best per gain means an in-GCS C++ optimizer running a sim-eval loop and streaming the candidate + running best every evaluation. Porting the Python optimizer into the GCS as a real-time subsystem is the ~weeks-scale rewrite you flagged.' },
  { sel:'#gAuto,#gAtAng,#gAtRate,#gAtOut,#atPlotTitle,#atDashStatus,#atStatus', lv:'add', name:'Autotune live plots', why:'Once the optimizer streams data, the matplotlib-style plots are additive QtCharts / custom-paint widgets fed per evaluation.' },
  { sel:'#atApplyBtn', lv:'easy', name:'Apply gains to firmware', why:'A single explicit CMD_SET_PID with the best vector — trivial. Per AT-1 this is the only path that writes gains.' },

  /* ── SITL world / fault injection (cross-process) ── */
  { sel:'#fpvCanvas,#adiCanvas,#downCanvas,#pipAdi,#pipDown,#simDock', lv:'struct', name:'SITL FPV + camera render', why:'Rendering an FPV / down-cam view of the simulated world needs an OpenGL or Qt3D pipeline fed by the sim — a whole rendering subsystem, not a panel.' },
  { sel:'#sensorList,#gpsGlitchEn,#gpsGlitchOpts,#gpsGlitchMode', lv:'struct', name:'Sensor fault / noise injection', why:'Per-sensor noise models, CSV-driven noise, scale/latency/temp-bias and GPS glitch must be injected inside the sim. That needs new runtime-injection messages in the vsimd protocol plus hooks in the sensor model — it spans the GCS↔simulator process boundary.' },
  { sel:'#rcUartBtn,#rcm0,#rcv0,#rcm1,#rcv1,#rcm2,#rcv2,#rcm3,#rcv3', lv:'struct', name:'RC bridge into SITL', why:'Bridging a real transmitter into the running sim means reading a live RC link and injecting it as sim input — a new cross-process input path.' },
  { sel:'#windN,#windE,#windD,#windGust,#windPeriod,#windTurb,#geOn,#geHeight,#geFac,#magMag,#magDec,#magInc,#magVec,#obsList,#obsRest,#airSpd', lv:'add', name:'World configuration', why:'Wind, ground-effect, magnetic field, obstacles, restitution and air density are new fields in the sim-init/config message plus a config model and save/load. Additive once the schema is extended.' },
  { sel:'#gWindSpd,#gWindDir,#gAirSpd,#gGndEff,#windSpd,#windDir', lv:'add', name:'Environment — live', why:'Live environment readouts/plots are additive once the sim echoes the active world state back over telemetry.' },
  { sel:'#mcoefList,#simPose,#simAlt,#simSpd,#simVs,#simClk', lv:'add', name:'Vehicle model / sim state', why:'Per-motor dissimilarity, thrust-axis and mass/inertia editing extend the vehicle/mixer model and its serialization — additive fields and UI.' },

  /* ── Kernel perf (firmware-dependent) ── */
  { sel:'#pCpuPct,#pCpuSub,#pCpuStats,#gCpuHist,#pHeapBox,#gHeapHist,#pQueues,#qSelCap,#gQueueSel,#perfTaskTable,#tSelCap,#gTaskSel', lv:'add', name:'Kernel-perf observability', why:'A PerfWidget exists, but per-task CPU/stack, queue fill and heap need the RTOS to emit perf counters in a perf packet at rate. Additive (new packet + binding) — and structural if the kernel has no counters to instrument.' },
  { sel:'#lsQual,#lsDown,#lsUp,#lsTotal,#gLinkQual,#gLinkSpeed,#perfBadge,#perfSeq', lv:'easy', name:'Network · NavLink stats', why:'Link quality and throughput already exist in LinkStatsPanel — bind and plot.' },

  /* ── Calibration wizard ── */
  { sel:'#calFig8,#f8p', lv:'add', name:'Mag figure-8 animation', why:'A guided figure-8 motion animation (QPropertyAnimation / SVG path) — additive guidance UI.' },
  { sel:'#calStatusChip,#calStartBtn,#calNextBtn,#calProgress,#calCoverage,#calAxes,#calInstruction,#calModeSection,#calAxisSection', lv:'add', name:'Calibration wizard flow', why:'A CalibrationWidget exists, but the gated multi-step wizard (proceed only on confirm, per-step recording, 6-axis coverage) is a new state-machine flow on top of it.' },
  { sel:'#calModeBias,#calModeFull', lv:'easy', name:'Calibration mode gating', why:'Restricting gyro to offset-only vs full calibration is a simple option gate.' },

  /* ── Packet analyzer ── */
  { sel:'#pktExpr,#pktExprMsg', lv:'add', name:'Packet filter expression', why:'A free-text expression filter needs a small query parser/evaluator over decoded fields — additive.' },
  { sel:'#freqRibbon', lv:'add', name:'Per-type frequency ribbon', why:'Aggregating per-packet-type rates into a live ribbon is a new small visualisation over the existing stream.' },
  { sel:'#packetTable,#decoder,#typeChips,#dirFilter,#pktSearch,#pktStream,#pktAutoScroll,#pktDev', lv:'easy', name:'Packet analyzer', why:'PacketAnalyzerWidget already decodes and tables the stream; type/direction filters and the decoder pane are existing/easy.' },

  /* ── Dashboard ── */
  { sel:'#adi3d,#drone', lv:'struct', name:'3D attitude / airframe', why:'A real-time 3D airframe render needs an OpenGL / Qt3D scene plus model assets — a new rendering subsystem.' },
  { sel:'#adi,#adiInner,#pitchLadder,#rollArc,#hdgStrip', lv:'add', name:'Attitude indicator (PFD)', why:'A custom-painted ADI/PFD with pitch ladder, roll arc and heading tape — additive QPainter widget (the app has a PFD to extend).' },
  { sel:'#g1,#g2,#g3,#g4,#tempFill,#tempRead,#battFill,#battRead,#statePill,#logLines,#rollV,#pitchV,#yawV', lv:'easy', name:'Telemetry readouts', why:'Sparklines, battery/temp gauges, the state pill and the system log bind to telemetry already flowing.' },
  { sel:'#btn2d,#btn3d', lv:'easy', name:'2D / 3D toggle', why:'Switches which attitude widget is shown — easy once both exist.' },

  /* ── RC / control ── */
  { sel:'#stickL,#stickR', lv:'add', name:'RC stick visualiser', why:'Custom-painted stick-crosshair widgets — additive.' },
  { sel:'#rcChannels,#gRcHist,#rssiV', lv:'easy', name:'RC channels', why:'Channel bars and history bind to existing RC telemetry.' },
  { sel:'#clsec-dt', lv:'add', name:'Loop dt / jitter', why:'Loop timing/jitter may need a new telemetry field from the controller — additive.' },
  { sel:'#clsec-angle,#clsec-rate,#clsec-output', lv:'easy', name:'Control-loop telemetry', why:'PID setpoint / measured / output plots bind to the control-loop packet.' },

  /* ── Settings ── */
  { sel:'#sp-streams', lv:'add', name:'Telemetry stream config', why:'Per-stream enable + rate must emit SET_STREAM_RATE and track the negotiated rate — additive wiring to the link.' },
  { sel:'#sp-link,#sp-display,#sp-plots,#sp-logging,#sp-alerts,#sp-advanced,#settingsNav,#setMruCount', lv:'easy', name:'Settings forms', why:'QSettings-backed forms (units, theme, plots, logging, alerts) — standard and easy.' },

  /* ── Convenience UI we mocked ── */
  { sel:'#kbModal', lv:'struct', name:'Editable keyboard shortcuts', why:'An editable keymap with searchable commands and when-contexts needs a central command/action registry the dispatcher resolves. Qt apps usually scatter QShortcuts per-widget, so this is a cross-cutting architectural addition.' },
  { sel:'#mruSwitch', lv:'add', name:'Recent-views (Ctrl+`) switcher', why:'An MRU overlay plus visit tracking — additive new widget (no GCS ships this).' },
  { sel:'#aboutModal', lv:'easy', name:'About dialog', why:'A standard about box — trivial.' },
  { sel:'#docsModal', lv:'easy', name:'Documentation', why:'Open external docs (easy) or embed a help browser (light additive).' },

  /* ── Toolbar / status / chrome ── */
  { sel:'#liveInd', lv:'add', name:'LIVE / REPLAY indicator', why:'The LIVE label is trivial; its REPLAY state is driven by the replay data-source — structural, see the replay bar.' },
  { sel:'#armBtn', lv:'easy', name:'ARM / Disarm', why:'Maps to the existing CMD_ARM command path with pre-arm gating.' },
  { sel:'#connBtn,#transport,#portSel,#portLbl', lv:'easy', name:'Link connect', why:'Transport / port / baud + connect already drive the serial/UDP backend.' },
  { sel:'#connSeg,#driftV,#pktV', lv:'easy', name:'Status bar', why:'Status fields bind to the existing link/clock model.' },
];
/* Catch-alls so every remaining control/panel still gets a verdict ("and all"). */
const AX_AUTO = [
  { sel:'.menubar .menu', lv:'easy', name:'Menu',          why:'A standard QMenu action — wires to an existing slot or view switch.' },
  { sel:'.mi,.snav,.tab', lv:'easy', name:'Navigation item', why:'Switches a view/pane in an existing QStackedWidget / QTabWidget.' },
  { sel:'.tb-btn,.rp-btn,button', lv:'easy', name:'Button', why:'A control that maps to an existing command/slot in the app.' },
  { sel:'select,input,.chk,.spin,.sw2,.sw3', lv:'easy', name:'Form control', why:'Bound to a setting or model field.' },
  { sel:'.gframe,.graph', lv:'add', name:'Plot',           why:'A live chart — additive QtCharts / custom-paint bound to a telemetry series.' },
  { sel:'.panel', lv:'add', name:'Panel',                  why:'A dockable QWidget that must be built and laid out.' },
];
const AX_LVNAME = { struct:'Deep structural rewrite', add:'Long additive work', easy:'Easy in the real app' };

let axTags = [], axRebuildTimer = null;
function toggleAnalysis(){
  axOn = !axOn;
  document.getElementById('axLayer').classList.toggle('hidden', !axOn);
  document.getElementById('axLegend').classList.toggle('hidden', !axOn);
  if(axOn){ axBuild(); showToast('info','Implementation-analysis mode ON — hover any tag'); }
  else { axClear(); axHideTip(); showToast('info','Analysis mode off'); }
}
function axClear(){ const L = document.getElementById('axLayer'); if(L) L.innerHTML = ''; axTags = []; }
function axMatch(el){ for(const r of AX_RULES) if(el.matches(r.sel)) return r; for(const r of AX_AUTO) if(el.matches(r.sel)) return r; return null; }
function axBuild(){
  axClear();
  const L = document.getElementById('axLayer');
  const sel = AX_RULES.concat(AX_AUTO).map(r => r.sel).join(',');
  const counts = { struct:0, add:0, easy:0 };
  const seen = new Set();
  Array.from(document.querySelectorAll(sel)).forEach(el => {
    if(seen.has(el)) return; const r = axMatch(el); if(!r) return; seen.add(el);
    const tag = document.createElement('div');
    tag.className = 'ax-tag lv-' + r.lv;
    tag.innerHTML = '<i class="ax-d struct"></i><i class="ax-d add"></i><i class="ax-d easy"></i>';
    const entry = { el, tag, lv:r.lv, name:r.name, why:r.why };
    tag.addEventListener('mouseenter', () => axShowTip(entry));
    tag.addEventListener('mouseleave', axHideTip);
    L.appendChild(tag); axTags.push(entry); counts[r.lv]++;
  });
  setTxt('axCStruct', counts.struct); setTxt('axCAdd', counts.add); setTxt('axCEasy', counts.easy);
  axPosition();
}
function axPosition(){
  const vw = innerWidth, vh = innerHeight;
  for(const { el, tag } of axTags){
    const r = el.getBoundingClientRect();
    if(r.width < 2 || r.height < 2 || r.bottom < 4 || r.top > vh - 2 || r.right < 4 || r.left > vw){ tag.style.display = 'none'; continue; }
    tag.style.display = '';
    tag.style.left = Math.min(vw - 2, r.right - 2) + 'px';
    tag.style.top  = Math.max(2, r.top + 2) + 'px';
  }
}
function axShowTip(entry){
  const t = document.getElementById('axTip');
  t.innerHTML = '<div class="ax-tip-lv '+entry.lv+'">'+AX_LVNAME[entry.lv]+'</div>'
              + '<div class="ax-tip-name">'+entry.name+'</div>'
              + '<div class="ax-tip-why">'+entry.why+'</div>';
  t.classList.remove('hidden');
  const r = entry.tag.getBoundingClientRect(), tw = t.offsetWidth, th = t.offsetHeight;
  let left = r.right - tw; if(left < 6) left = 6; if(left + tw > innerWidth - 6) left = innerWidth - 6 - tw;
  let top = r.bottom + 6; if(top + th > innerHeight - 6) top = r.top - th - 6;
  t.style.left = left + 'px'; t.style.top = Math.max(6, top) + 'px';
}
function axHideTip(){ const t = document.getElementById('axTip'); if(t) t.classList.add('hidden'); }
function axRefresh(){ if(!axOn) return; clearTimeout(axRebuildTimer); axRebuildTimer = setTimeout(axBuild, 40); }
window.addEventListener('scroll', () => { if(axOn) axPosition(); }, true);
window.addEventListener('resize', () => { if(axOn) axPosition(); });
document.addEventListener('click', () => { if(axOn) axRefresh(); }, true);   // rebuild after click-driven UI changes

syncMenuHints();   // apply any saved overrides to the menu hints on load
(function(){ const el = document.getElementById('setMruCount'); if(el) el.value = mruCount(); })();
