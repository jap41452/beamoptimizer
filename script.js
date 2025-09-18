// Keep canvases crisp when CSS width changes
    function syncCanvasSizes() {
      const ids = ['beamCanvas','dispPlot','momentPlot','shearPlot','stressPlot'];
      let changed = false;
      for (const id of ids) {
        const cv = document.getElementById(id);
        if (!cv) continue;
        const w = Math.round(cv.getBoundingClientRect().width);
        const h = Math.round(cv.getBoundingClientRect().height);
        if (w && h && (cv.width !== w || cv.height !== h)) {
          cv.width = w; cv.height = h; changed = true;
        }
      }
      if (changed) {
        try { drawBeam(); } catch(_) {}
        try {
          if (window.lastU && lastU.u && lastU.map) {
            plotDisplacement(lastU.u, lastU.map);
            plotMoment(); plotShearFromMoment(); plotBendingStress();
          }
        } catch(_) {}
      }
    }
    window.addEventListener('load', syncCanvasSizes);
    window.addEventListener('resize', syncCanvasSizes);

// -------------------- State --------------------
    let K_global = null;
    let segments = [];
    let nodes = [];
    let elementForces = null; // [{V1,M1,V2,M2,L}, ...]
    let lastU = null;         // {u, map}
    let unitSystem = 'US';

    // Discrete Sections Catalog (up to 10 rows) — name, I, St, Sb, optional A, wd
    let sectionsCatalog = [
      { name: 'S-1', I: 20, St: 5,  Sb: 5,  A: 0, wd: 0 },
      { name: 'S-2', I: 40, St: 8,  Sb: 8,  A: 0, wd: 0 },
      { name: 'S-3', I: 80, St: 12, Sb: 12, A: 0, wd: 0 }
    ];

    // -------------------- Helpers --------------------
    function isHinge(node){ return !!node.pinJoint; }

    function fmtSig3(n){
      if (n === 0) return '0';
      if (!isFinite(n)) return String(n);
      const abs = Math.abs(n);
      if (abs >= 1e-3 && abs < 1e4) {
        const s = Number(n).toPrecision(3);
        return (s.includes('e') || s.includes('E')) ? Number(n).toString() : String(parseFloat(s));
      } else {
        return Number(n).toExponential(2);
      }
    }

    function normalizeSegments(){
      // Back-compat: migrate S -> St/Sb and q -> qL/qR; default wd/A
      for (const seg of segments){
        if (seg.St === undefined && seg.S !== undefined) seg.St = seg.S;
        if (seg.Sb === undefined && seg.S !== undefined) seg.Sb = seg.S;
        if (seg.St === undefined) seg.St = 5.3;
        if (seg.Sb === undefined) seg.Sb = 5.3;
        if (seg.qL === undefined && seg.q !== undefined) seg.qL = seg.q;
        if (seg.qR === undefined && seg.q !== undefined) seg.qR = seg.q;
        if (seg.qL === undefined) seg.qL = 0;
        if (seg.qR === undefined) seg.qR = 0;
        if (seg.wd === undefined) seg.wd = 0;
        if (seg.A  === undefined) seg.A  = 0;
        delete seg.S; delete seg.q;
      }
    }

    // DOF map with hinge (pin = moment release; shear still transferred)
    function buildDOFMap(){
      const n = nodes.length; const idxW = new Array(n); const thLeft = new Array(n); const thRight = new Array(n); let dof=0;
      for (let i=0;i<n;i++) idxW[i] = dof++;
      for (let i=0;i<n;i++){
        const hinge = isHinge(nodes[i]); const hasLeft = (i>0); const hasRight = (i<n-1);
        if (!hinge){
          const r = dof++; thLeft[i] = hasLeft? r : r; thRight[i] = hasRight? r : r;
        } else {
          if (hasLeft)  thLeft[i]  = dof++;
          if (hasRight) thRight[i] = dof++;
        }
      }
      return { idxW, thLeft, thRight, ndof: dof };
    }

    // -------------------- Editors --------------------
    function editNode(i) {
      const node = nodes[i];
      Swal.fire({
        title: `Node ${i} Properties`,
        html:
          `<label><span>BC:</span><select id="bc" class="swal2-select">
            <option value="free" ${node.bc === 'free' ? 'selected' : ''}>Free</option>
            <option value="pinned" ${node.bc === 'pinned' ? 'selected' : ''}>Simple (w=0, θ free)</option>
            <option value="fixed" ${node.bc === 'fixed' ? 'selected' : ''}>Fixed</option>
          </select></label>
          <label><span>F:</span><input id="F" class="swal2-input" value="${node.F}"></label>
          <label><span>M:</span><input id="M" class="swal2-input" value="${node.M}"></label>
          <label><span>Kv:</span><input id="Kv" class="swal2-input" value="${node.Kv}"></label>
          <label><span>Km:</span><input id="Km" class="swal2-input" value="${node.Km}"></label>
          <hr style='margin:6px 0;'>
          <label><span>Pin joint (moment release; shear-only transfer):</span><input type="checkbox" id="pinJoint" ${node.pinJoint? 'checked':''}></label>
          <label><span>Prescribed w:</span><input id="w0" class="swal2-input" placeholder="(blank = none)" value="${Number.isFinite(node.w0)? node.w0 : ''}"></label>
          <label><span>Prescribed θ:</span><input id="th0" class="swal2-input" placeholder="(blank = none)" value="${Number.isFinite(node.th0)? node.th0 : ''}"></label>`,
        focusConfirm: false,
        preConfirm: () => {
          const bcVal = document.getElementById('bc').value;
          const Fv = parseFloat(document.getElementById('F').value);
          const Mv = parseFloat(document.getElementById('M').value);
          const Kv = parseFloat(document.getElementById('Kv').value);
          const Km = parseFloat(document.getElementById('Km').value);
          const pin = document.getElementById('pinJoint').checked;
          const w0raw = document.getElementById('w0').value.trim();
          const th0raw = document.getElementById('th0').value.trim();
          const pinOK = !(bcVal === 'fixed' || Km > 0);
          return [ bcVal, Fv, Mv, Kv, Km, pin && pinOK, (w0raw==='')? null: parseFloat(w0raw), (th0raw==='')? null: parseFloat(th0raw) ];
        }
      }).then((res)=>{
        const data = res.value;
        if (data) nodes[i] = { bc: data[0], F: data[1], M: data[2], Kv: data[3], Km: data[4], pinJoint: data[5], w0: data[6], th0: data[7] };
        drawBeam();
      });
    }

    function editSegment(i) {
      const seg = segments[i];
      Swal.fire({
        title: `Segment ${i} Properties`,
        html:
          `<div id="segForm">
            <label><span>E:</span><input id="E" class="swal2-input" value="${seg.E}"></label>
            <label><span>I:</span><input id="I" class="swal2-input" value="${seg.I}"></label>
            <label><span>A (area):</span><input id="A" class="swal2-input" value="${seg.A ?? 0}"></label>
            <label><span>L:</span><input id="L" class="swal2-input" value="${seg.L}"></label>
            <label><span>St (top):</span><input id="St" class="swal2-input" value="${seg.St ?? seg.S ?? ''}"></label>
            <label><span>Sb (bottom):</span><input id="Sb" class="swal2-input" value="${seg.Sb ?? seg.S ?? ''}"></label>
            <label><span>qL (left, + up):</span><input id="qL" class="swal2-input" value="${seg.qL ?? seg.q ?? 0}"></label>
            <label><span>qR (right, + up):</span><input id="qR" class="swal2-input" value="${seg.qR ?? seg.q ?? 0}"></label>
            <label><span>wd (weight dens.):</span><input id="wd" class="swal2-input" value="${seg.wd ?? 0}"></label>
            <div id="matRow" style="margin-top:6px; display:flex; gap:6px; flex-wrap:wrap; align-items:center;">
              <span style="font-size:12px; color:#555; margin-right:4px;">Material:</span>
              <button type="button" class="matBtn" data-mat="steel" style="height:26px; padding:0 10px; border:1px solid #bbb; border-radius:6px; background:#f5f5f5;">Steel</button>
              <button type="button" class="matBtn" data-mat="aluminum" style="height:26px; padding:0 10px; border:1px solid #bbb; border-radius:6px; background:#f5f5f5;">Aluminum</button>
              <button type="button" class="matBtn" data-mat="hardwood" style="height:26px; padding:0 10px; border:1px solid #bbb; border-radius:6px; background:#f5f5f5;">Hard wood</button>
              <button type="button" class="matBtn" data-mat="softwood" style="height:26px; padding:0 10px; border:1px solid #bbb; border-radius:6px; background:#f5f5f5;">Soft wood</button>
            </div>
            <div style="margin-top:6px;">
              <button id="secCalcToggle" type="button" style="height:28px; padding:0 10px; border-radius:6px; border:1px solid #99c; background:#eef; color:#223;">Section Calculator…</button>
            </div>
            <div id="secCalc" style="display:none; margin-top:8px; padding-top:8px; border-top:1px solid #ddd;">
              <div id="shapeGrid" style="display:grid; grid-template-columns:repeat(3, auto); gap:6px; align-items:center;">
                <button type="button" class="shapeBtn" data-shape="rect" title="Rectangle" style="display:flex; align-items:center; gap:6px; padding:4px 6px; border:1px solid #ccc; border-radius:6px; background:#fafafa;">
                  <svg width="28" height="20"><rect x="2" y="2" width="24" height="16" fill="#ddd" stroke="#666"/></svg>
                  <span>Rectangle</span>
                </button>
                <button type="button" class="shapeBtn" data-shape="circ" title="Circle / Tube" style="display:flex; align-items:center; gap:6px; padding:4px 6px; border:1px solid #ccc; border-radius:6px; background:#fafafa;">
                  <svg width="28" height="28"><circle cx="14" cy="14" r="12" fill="#ddd" stroke="#666"/><circle cx="14" cy="14" r="6" fill="#fff" stroke="#666"/></svg>
                  <span>Circle</span>
                </button>
                <button type="button" class="shapeBtn" data-shape="tube" title="Hollow Box" style="display:flex; align-items:center; gap:6px; padding:4px 6px; border:1px solid #ccc; border-radius:6px; background:#fafafa;">
                  <svg width="28" height="20"><rect x="2" y="2" width="24" height="16" fill="#ddd" stroke="#666"/><rect x="7" y="6" width="14" height="8" fill="#fff" stroke="#666"/></svg>
                  <span>Hollow Box</span>
                </button>
                <button type="button" class="shapeBtn" data-shape="ibeam" title="I-Beam" style="display:flex; align-items:center; gap:6px; padding:4px 6px; border:1px solid #ccc; border-radius:6px; background:#fafafa;">
                  <svg width="32" height="20"><rect x="2" y="2" width="28" height="4" fill="#ddd" stroke="#666"/><rect x="14" y="6" width="4" height="8" fill="#ddd" stroke="#666"/><rect x="2" y="14" width="28" height="4" fill="#ddd" stroke="#666"/></svg>
                  <span>I-Beam</span>
                </button>
                <button type="button" class="shapeBtn" data-shape="channel" title="Channel" style="display:flex; align-items:center; gap:6px; padding:4px 6px; border:1px solid #ccc; border-radius:6px; background:#fafafa;">
                  <svg width="32" height="20"><rect x="2" y="2" width="6" height="16" fill="#ddd" stroke="#666"/><rect x="2" y="2" width="22" height="4" fill="#ddd" stroke="#666"/><rect x="2" y="14" width="22" height="4" fill="#ddd" stroke="#666"/></svg>
                  <span>Channel</span>
                </button>
                <button type="button" class="shapeBtn" data-shape="tbeam" title="T-Beam" style="display:flex; align-items:center; gap:6px; padding:4px 6px; border:1px solid #ccc; border-radius:6px; background:#fafafa;">
                  <svg width="32" height="20"><rect x="2" y="2" width="28" height="4" fill="#ddd" stroke="#666"/><rect x="14" y="6" width="4" height="12" fill="#ddd" stroke="#666"/></svg>
                  <span>T-Beam</span>
                </button>
              </div>
              <div id="secInputs" style="margin-top:8px; font-size:12px;"></div>
            </div>
          </div>`,
        focusConfirm: false,
        didOpen: (el)=>{ setupSectionCalc(el, i); setupMaterialPresets(el, i); },
        preConfirm: () => [
          parseFloat(document.getElementById('E').value),
          parseFloat(document.getElementById('I').value),
          parseFloat(document.getElementById('A').value),
          parseFloat(document.getElementById('L').value),
          parseFloat(document.getElementById('St').value),
          parseFloat(document.getElementById('Sb').value),
          parseFloat(document.getElementById('qL').value),
          parseFloat(document.getElementById('qR').value),
          parseFloat(document.getElementById('wd').value)
        ]
      }).then((res)=>{
        const data = res.value;
        if (data) segments[i] = { E: data[0], I: data[1], A: data[2], L: data[3], St: data[4], Sb: data[5], qL: data[6], qR: data[7], wd: data[8], secIndex: segments[i].secIndex };
        drawBeam();
      });
    }

    // ---- Section Calculator helpers ----
    function setupSectionCalc(el, segIndex){
      const toggleBtn = el.querySelector('#secCalcToggle');
      const panel = el.querySelector('#secCalc');
      if (toggleBtn && panel){ toggleBtn.addEventListener('click', ()=>{ panel.style.display = panel.style.display==='none' ? 'block' : 'none'; }); }
      const grid = el.querySelector('#shapeGrid');
      const inputs = el.querySelector('#secInputs');
      function renderForm(shape){
        let html='';
        if (shape==='rect'){
          html = `
            <div style="display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
              <label><span style="width:80px; display:inline-block;">b (width)</span><input id="sh_b" class="swal2-input" style="width:100px"></label>
              <label><span style="width:80px; display:inline-block;">h (height)</span><input id="sh_h" class="swal2-input" style="width:100px"></label>
              <button id="applySec" type="button" style="height:28px; padding:0 12px; border:1px solid #7aa; border-radius:6px; background:#e6f3ff;">Use</button>
            </div>`;
        } else if (shape==='circ'){
          html = `
            <div style="display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
              <label><span style="width:80px; display:inline-block;">OD</span><input id="sh_od" class="swal2-input" style="width:100px"></label>
              <label><span style="width:80px; display:inline-block;">ID</span><input id="sh_id" class="swal2-input" style="width:100px" value="0"></label>
              <button id="applySec" type="button" style="height:28px; padding:0 12px; border:1px solid #7aa; border-radius:6px; background:#e6f3ff;">Use</button>
            </div>`;
        } else if (shape==='tube'){
          html = `
            <div style="display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
              <label><span style="width:80px; display:inline-block;">b<sub>o</sub></span><input id="sh_bo" class="swal2-input" style="width:100px"></label>
              <label><span style="width:80px; display:inline-block;">h<sub>o</sub></span><input id="sh_ho" class="swal2-input" style="width:100px"></label>
              <label><span style="width:80px; display:inline-block;">b<sub>i</sub></span><input id="sh_bi" class="swal2-input" style="width:100px"></label>
              <label><span style="width:80px; display:inline-block;">h<sub>i</sub></span><input id="sh_hi" class="swal2-input" style="width:100px"></label>
              <button id="applySec" type="button" style="height:28px; padding:0 12px; border:1px solid #7aa; border-radius:6px; background:#e6f3ff;">Use</button>
            </div>`;
        } else if (shape==='ibeam' || shape==='channel'){
          html = `
            <div style="display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
              <label><span style="width:120px; display:inline-block;">h (depth)</span><input id="sh_h" class="swal2-input" style="width:100px"></label>
              <label><span style="width:120px; display:inline-block;">b<sub>f</sub> (flange)</span><input id="sh_bf" class="swal2-input" style="width:100px"></label>
              <label><span style="width:120px; display:inline-block;">t<sub>f</sub> (flange)</span><input id="sh_tf" class="swal2-input" style="width:100px"></label>
              <label><span style="width:120px; display:inline-block;">t<sub>w</sub> (web)</span><input id="sh_tw" class="swal2-input" style="width:100px"></label>
              <button id="applySec" type="button" style="height:28px; padding:0 12px; border:1px solid #7aa; border-radius:6px; background:#e6f3ff;">Use</button>
            </div>`;
        } else if (shape==='tbeam'){
          html = `
            <div style="display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
              <label><span style="width:120px; display:inline-block;">h (total)</span><input id="sh_h" class="swal2-input" style="width:100px"></label>
              <label><span style="width:120px; display:inline-block;">b<sub>f</sub> (flange)</span><input id="sh_bf" class="swal2-input" style="width:100px"></label>
              <label><span style="width:120px; display:inline-block;">t<sub>f</sub> (flange)</span><input id="sh_tf" class="swal2-input" style="width:100px"></label>
              <label><span style="width:120px; display:inline-block;">t<sub>w</sub> (web)</span><input id="sh_tw" class="swal2-input" style="width:100px"></label>
              <button id="applySec" type="button" style="height:28px; padding:0 12px; border:1px solid #7aa; border-radius:6px; background:#e6f3ff;">Use</button>
              <div style="font-size:11px; color:#555;">Assumes flange at the <b>top</b>.</div>
            </div>`;
        }
        inputs.innerHTML = html;
        const apply = el.querySelector('#applySec');
        if (apply){
          apply.addEventListener('click', ()=>{
            try{
              const out = (function(){
                if (shape==='rect'){
                  const b = parseFloat(el.querySelector('#sh_b').value), h = parseFloat(el.querySelector('#sh_h').value);
                  if (!(b>0 && h>0)) throw new Error('Enter b,h > 0');
                  const A = b*h; const I = b*Math.pow(h,3)/12; const c=h/2; const S = I/c; return {A,I,St:S,Sb:S};
                }
                if (shape==='circ'){
                  const od = parseFloat(el.querySelector('#sh_od').value), id = parseFloat(el.querySelector('#sh_id').value||'0');
                  if (!(od>0) || id<0 || id>=od) throw new Error('Require OD>0 and 0≤ID<OD');
                  const A = Math.PI*(od*od - id*id)/4; const I = Math.PI*(Math.pow(od,4) - Math.pow(id,4))/64; const c=od/2; const S=I/c; return {A,I,St:S,Sb:S};
                }
                if (shape==='tube'){
                  const bo=parseFloat(el.querySelector('#sh_bo').value), ho=parseFloat(el.querySelector('#sh_ho').value), bi=parseFloat(el.querySelector('#sh_bi').value), hi=parseFloat(el.querySelector('#sh_hi').value);
                  if (!(bo>0 && ho>0 && bi>=0 && hi>=0 && bo>bi && ho>hi)) throw new Error('Require bo>bi≥0 and ho>hi≥0');
                  const A = bo*ho - bi*hi; const I = (bo*Math.pow(ho,3) - bi*Math.pow(hi,3))/12; const c=ho/2; const S=I/c; return {A,I,St:S,Sb:S};
                }
                if (shape==='ibeam' || shape==='channel'){
                  const h=parseFloat(el.querySelector('#sh_h').value), bf=parseFloat(el.querySelector('#sh_bf').value), tf=parseFloat(el.querySelector('#sh_tf').value), tw=parseFloat(el.querySelector('#sh_tw').value);
                  if (!(h>0 && bf>0 && tf>0 && tw>0 && h>2*tf)) throw new Error('Check dimensions: h>2tf and positive');
                  const hw = h - 2*tf; const A = 2*bf*tf + tw*hw; const d = h/2 - tf/2;
                  const I = 2*(bf*Math.pow(tf,3)/12 + bf*tf*Math.pow(d,2)) + (tw*Math.pow(hw,3))/12;
                  const c = h/2; const S = I/c; return {A,I,St:S,Sb:S};
                }
                if (shape==='tbeam'){
                  const h=parseFloat(el.querySelector('#sh_h').value), bf=parseFloat(el.querySelector('#sh_bf').value), tf=parseFloat(el.querySelector('#sh_tf').value), tw=parseFloat(el.querySelector('#sh_tw').value);
                  if (!(h>0 && bf>0 && tf>0 && tw>0 && h>tf)) throw new Error('Check dimensions: h>tf and positive');
                  const hw = h - tf; // web height
                  const A_f = bf*tf, y_f = h - tf/2; const I_f = bf*Math.pow(tf,3)/12;
                  const A_w = tw*hw, y_w = hw/2;     const I_w = tw*Math.pow(hw,3)/12;
                  const A = A_f + A_w; const ybar = (A_f*y_f + A_w*y_w)/A;
                  const I = I_f + A_f*Math.pow(y_f - ybar,2) + I_w + A_w*Math.pow(y_w - ybar,2);
                  const St = I/(h - ybar), Sb = I/(ybar - 0);
                  return {A,I,St,Sb};
                }
                throw new Error('Unknown shape');
              })();
              const set = (id,val)=>{ const inp = document.getElementById(id); if (inp){ inp.value = val; inp.dispatchEvent(new Event('input', {bubbles:true})); } };
              const to3 = (v)=> Number(v).toPrecision(3);
              set('A', to3(out.A)); set('I', to3(out.I)); set('St', to3(out.St)); set('Sb', to3(out.Sb));
              try {
                const tgt = segments[segIndex];
                if (tgt) {
                  tgt.A  = parseFloat(to3(out.A));
                  tgt.I  = parseFloat(to3(out.I));
                  tgt.St = parseFloat(to3(out.St));
                  tgt.Sb = parseFloat(to3(out.Sb));
                }
                drawBeam();
              } catch(_) {}
              Swal.fire({icon:'success', title:'Section properties applied', timer:900, showConfirmButton:false});
            }catch(err){ Swal.fire('Invalid input', err.message || String(err), 'error'); }
          });
        }
      }
      if (grid){ grid.querySelectorAll('button[data-shape]').forEach(btn=> btn.addEventListener('click', ()=> renderForm(btn.dataset.shape)) ); }
    }

    // ---- Material presets (E & wd) ----
    function setupMaterialPresets(el, segIndex){
      const row = el.querySelector('#matRow');
      if (row && !row.querySelector('.unitBtn')){
        const spacer = document.createElement('span'); spacer.style.cssText = 'font-size:12px; color:#555; margin-left:8px;'; spacer.textContent = 'Units:';
        const us = document.createElement('button'); us.type='button'; us.className='unitBtn'; us.dataset.unit='US'; us.textContent='US'; us.style.cssText='height:26px; padding:0 10px; border:1px solid #bbb; border-radius:6px; background:#f5f5f5;';
        const si = document.createElement('button'); si.type='button'; si.className='unitBtn'; si.dataset.unit='SI'; si.textContent='Metric (SI)'; si.style.cssText='height:26px; padding:0 10px; border:1px solid #bbb; border-radius:6px; background:#f5f5f5;';
        row.appendChild(spacer); row.appendChild(us); row.appendChild(si);
      }

      const presets = {
        US: { steel: {E:29e6, wd:0.283}, aluminum:{E:10e6, wd:0.098}, hardwood:{E:1.6e6, wd:0.025}, softwood:{E:1.2e6, wd:0.018} },
        SI: { steel: {E:200e9, wd:77000}, aluminum:{E:69e9, wd:26500}, hardwood:{E:11e9, wd:7000}, softwood:{E:8.3e9, wd:5000} }
      };

      function refreshUnitButtons(){
        el.querySelectorAll('.unitBtn').forEach(b=>{
          const active = (b.dataset.unit === unitSystem);
          b.style.background  = active ? '#e6f3ff' : '#f5f5f5';
          b.style.borderColor = active ? '#5a8' : '#bbb';
          b.style.fontWeight  = active ? '600' : '400';
        });
      }
      el.querySelectorAll('.unitBtn').forEach(btn=>{
        btn.addEventListener('click', ()=>{ unitSystem = btn.dataset.unit; refreshUnitButtons(); });
      });
      refreshUnitButtons();

      el.querySelectorAll('.matBtn').forEach(btn=>{
        btn.addEventListener('click', ()=>{
          const key = btn.dataset.mat; const pset = presets[unitSystem] || presets.US; const p = pset[key]; if (!p) return;
          const eIn = el.querySelector('#E'); const wdIn = el.querySelector('#wd');
          if (eIn)  eIn.value  = p.E;
          if (wdIn) wdIn.value = p.wd;
          try { const tgt = segments[segIndex]; if (tgt){ tgt.E = p.E; tgt.wd = p.wd; } drawBeam(); } catch(_) {}
        });
      });
    }

    // -------------------- Build & Draw --------------------
    function buildBeam() {
      const numSegments = parseInt(document.getElementById("numSegments").value);
      const totalLen = parseFloat(document.getElementById("totalLength").value);
      if (isNaN(numSegments) || numSegments < 1 || numSegments > 10) { alert("Please enter a valid number of segments (1-10)."); return; }
      if (!isFinite(totalLen) || totalLen <= 0) { alert("Please enter a valid Total Length (> 0)."); return; }

      const segLen = totalLen / numSegments;

      // Initialize/evenly space lengths on Build
      const seed = segments.length ? segments[0] : { E: 12e6, I: 21.3, St: 5.3, Sb: 5.3, qL: 0, qR: 0, wd: 0, A: 0 };
      const arr = [];
      for (let i=0;i<numSegments;i++){
        const prev = (i && arr[i-1]) ? arr[i-1] : seed;
        arr.push({ ...prev, L: segLen });
      }
      segments = arr;
      normalizeSegments();

      // Rebuild nodes to match
      nodes = Array(numSegments + 1).fill().map(() => ({ bc: 'free', F: 0, M: 0, Kv: 0, Km: 0, pinJoint: false, w0: null, th0: null }));

      elementForces = null; K_global = null; lastU = null; drawBeam();
    }

    function drawBeam() {
      const canvas = document.getElementById("beamCanvas");
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      if (segments.length === 0) return;

      const margin = parseInt(getComputedStyle(document.documentElement).getPropertyValue('--pad')) || 50;
      const totalLength = segments.reduce((sum, seg) => sum + seg.L, 0);
      const scaleX = (canvas.width - 2 * margin) / totalLength;
      const maxEI = Math.max(...segments.map(seg => seg.E * seg.I));
      const maxHeight = 0.40 * canvas.height * 0.25;
      const eiScale = parseFloat(document.getElementById('eiScale').value || '1');
      const maxAbsQ = Math.max(...segments.flatMap(seg => [Math.abs(seg.qL || 0), Math.abs(seg.qR || 0)]), 0);
      const maxAbsF = Math.max(...nodes.map(n => Math.abs(n.F || 0)), 0);
      const maxAbsW = Math.max(...segments.map(s => Math.abs((s.wd||0)*(s.A||0))), 0);

      const midY = Math.round(canvas.height * 0.35);
      ctx.strokeStyle = '#aaa'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(margin, midY); ctx.lineTo(canvas.width - margin, midY); ctx.stroke();

      const approxTicks = 10, tickPx = (canvas.width - 2*margin) / approxTicks;
      ctx.fillStyle = '#666'; ctx.textAlign = 'center'; ctx.font = '11px sans-serif';
      const beamHalf = (maxHeight * eiScale) / 2;
      const tickLabelY = midY + Math.max(beamHalf + 26, 36);
      const MIN_GAP_BELOW_TICKS = 14;
      for (let i=0;i<=approxTicks;i++){
        const xx = margin + i*tickPx;
        ctx.beginPath(); ctx.moveTo(xx, midY-6); ctx.lineTo(xx, midY+6); ctx.strokeStyle='#ddd'; ctx.stroke();
        const xVal=(totalLength*i/approxTicks).toFixed(0);
        ctx.fillText(xVal, xx, tickLabelY);
      }

      let x = margin;
      for (let i = 0; i < segments.length; i++) {
        const seg = segments[i];
        const width = seg.L * scaleX;
        const height = ((seg.E * seg.I) / maxEI) * maxHeight * eiScale;

        // Darker gray shading (≈60%→30% lightness)
        const wmag = Math.abs((seg.wd||0)*(seg.A||0));
        const tGray = maxAbsW > 0 ? Math.min(1, wmag / maxAbsW) : 0;
        const light = Math.round(60 - tGray * 30);
        ctx.fillStyle = `hsl(0 0% ${light}%)`;
        ctx.fillRect(x, midY - height/2, width, height);

        drawLoadBlanket(ctx, x, midY, width, height, (seg.qL || 0), (seg.qR || 0), maxAbsQ);

        const secName = (typeof seg.secIndex === 'number' && sectionsCatalog[seg.secIndex])
          ? `Section: ${sectionsCatalog[seg.secIndex].name}` : null;

        const props = [
          ...(secName ? [secName] : []),
          `E: ${fmtSig3(seg.E)}`, `I: ${fmtSig3(seg.I)}`, `A: ${fmtSig3(seg.A ?? 0)}`, `L: ${fmtSig3(seg.L)}`,
          `St: ${fmtSig3(seg.St)}`, `Sb: ${fmtSig3(seg.Sb)}`, `qL: ${fmtSig3(seg.qL ?? 0)}`, `qR: ${fmtSig3(seg.qR ?? 0)}`, `wd: ${fmtSig3(seg.wd ?? 0)}`
        ];
        ctx.fillStyle = '#111'; ctx.textAlign = 'center'; ctx.font = '12px sans-serif';
        const startY = Math.max(midY + height/2 + 14, tickLabelY + MIN_GAP_BELOW_TICKS);
        for (let j=0;j<props.length;j++) ctx.fillText(props[j], x + width/2, startY + j*14);

        drawNodeFeatures(ctx, x, midY, nodes[i], maxAbsF);
        drawNodeDot(ctx, x, midY);
        x += width;
      }
      drawNodeFeatures(ctx, x, midY, nodes[segments.length], maxAbsF);
      drawNodeDot(ctx, x, midY);
    }

    function drawNodeDot(ctx, x, y){ ctx.beginPath(); ctx.fillStyle = '#06f'; ctx.arc(x, y, 6, 0, Math.PI*2); ctx.fill(); }

    // Visualize linear distributed load q(x)
    function drawLoadBlanket(ctx, x, midY, width, height, qL, qR, maxAbsQ){
      if ((!qL && !qR) || !isFinite(maxAbsQ) || maxAbsQ <= 0) return;
      const count = Math.max(6, Math.floor(width / 40));
      const spacing = width / (count + 1);
      ctx.save(); ctx.strokeStyle = '#0a8'; ctx.lineWidth = 1.5;
      for (let k=1;k<=count;k++){
        const xi = x + k*spacing; const r = (xi - x)/width;
        const qx = qL + (qR - qL) * r; if (!qx) continue;
        const up = qx > 0; const baseY = up ? (midY - height/2 - 6) : (midY + height/2 + 6);
        const maxLen = 48;
        const len = Math.max(10, (maxLen * Math.abs(qx)) / maxAbsQ);
        const tipY = up ? (baseY - len) : (baseY + len);
        ctx.beginPath(); ctx.moveTo(xi, baseY); ctx.lineTo(xi, tipY); ctx.stroke();
        ctx.beginPath(); if (up){ ctx.moveTo(xi-6, tipY+8); ctx.lineTo(xi, tipY); ctx.lineTo(xi+6, tipY+8); }
                         else  { ctx.moveTo(xi-6, tipY-8); ctx.lineTo(xi, tipY); ctx.lineTo(xi+6, tipY-8); }
        ctx.stroke();
      }
      ctx.restore();
    }

    function drawNodeFeatures(ctx, x, y, node, maxAbsF){
      ctx.strokeStyle = '#000'; ctx.lineWidth = 2;
      if (node.bc === 'pinned') { ctx.beginPath(); ctx.moveTo(x-16, y+18); ctx.lineTo(x, y); ctx.lineTo(x+16, y+18); ctx.closePath(); ctx.stroke(); }
      else if (node.bc === 'fixed') { ctx.beginPath(); ctx.moveTo(x-6, y-24); ctx.lineTo(x-6, y+24); ctx.stroke(); for (let yy=y-24; yy<=y+24; yy+=8) { ctx.beginPath(); ctx.moveTo(x-6, yy); ctx.lineTo(x-18, yy+8); ctx.stroke(); } }
      if (node.pinJoint) { ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI*2); ctx.stroke(); }
      if (node.Kv !== 0) { ctx.beginPath(); let sY=y+10; ctx.moveTo(x, sY); for (let i=0;i<5;i++){ ctx.lineTo(x+(i%2===0?6:-6), sY+6); sY+=6; } ctx.lineTo(x, sY+6); ctx.stroke(); }
      if (node.Km !== 0) { ctx.beginPath(); ctx.arc(x-24, y, 10, 0, Math.PI*2); ctx.stroke(); ctx.beginPath(); ctx.moveTo(x-14, y); ctx.lineTo(x-6, y); ctx.stroke(); }

      // Nodal moment glyph
      if (Math.abs(node.M) > 1e-12) {
        const isPos = node.M > 0;
        const r = 22;
        ctx.beginPath();
        if (isPos) ctx.arc(x, y, r, -0.5*Math.PI, 0.5*Math.PI, false);
        else       ctx.arc(x, y, r,  0.5*Math.PI, -0.5*Math.PI, false);
        ctx.stroke();
        const theta = isPos ? 0.5*Math.PI : -0.5*Math.PI;
        const tipX = x + r * Math.cos(theta);
        const tipY = y + r * Math.sin(theta);
        const shaft = 10, head = 12, phi = Math.PI/6, baseA = Math.PI;
        ctx.beginPath(); ctx.moveTo(tipX, tipY); ctx.lineTo(tipX - shaft, tipY);
        ctx.moveTo(tipX, tipY); ctx.lineTo(tipX + head*Math.cos(baseA + phi), tipY + head*Math.sin(baseA + phi));
        ctx.moveTo(tipX, tipY); ctx.lineTo(tipX + head*Math.cos(baseA - phi), tipY + head*Math.sin(baseA - phi));
        ctx.stroke();
      }

      if (Math.abs(node.F) > 1e-12) {
        const up = node.F > 0;
        const base = 16, extra = 34;
        const len = (maxAbsF && maxAbsF > 0) ? base + extra * (Math.abs(node.F) / maxAbsF) : base + extra * 0.6;
        const tail = y + (up ? 10 : -10);
        const tip  = y + (up ? -len :  len);
        ctx.beginPath(); ctx.moveTo(x, tail); ctx.lineTo(x, tip); ctx.stroke();
        ctx.beginPath();
        if (up) { ctx.moveTo(x - 7, tip + 8); ctx.lineTo(x, tip); ctx.lineTo(x + 7, tip + 8); }
        else    { ctx.moveTo(x - 7, tip - 8); ctx.lineTo(x, tip); ctx.lineTo(x + 7, tip - 8); }
        ctx.closePath(); ctx.fill();
      }
      if (Number.isFinite(node.w0) && Math.abs(node.w0) > 1e-12) { const above = node.w0 > 0; const yy = y + (above ? -26 : 26); ctx.beginPath(); ctx.moveTo(x-10, yy); ctx.lineTo(x+10, yy); ctx.stroke(); }
      if (Number.isFinite(node.th0) && Math.abs(node.th0) > 1e-12) { const cw = node.th0 > 0; ctx.beginPath(); ctx.arc(x+26, y, 10, 0.1*Math.PI, 1.4*Math.PI, !cw); ctx.stroke(); }
      ctx.fillStyle = '#111'; ctx.textAlign = 'center'; ctx.font = '12px sans-serif'; let offsetY = y + 46;
      if (Math.abs(node.F) > 1e-6) { ctx.fillText(`F: ${node.F}`, x, offsetY); offsetY += 14; }
      if (Math.abs(node.Kv) > 1e-6){ ctx.fillText(`Kv: ${node.Kv}`, x, offsetY); offsetY += 14; }
      if (Math.abs(node.Km) > 1e-6){ ctx.fillText(`Km: ${node.Km}`, x, offsetY); offsetY += 14; }
      if (Math.abs(node.M) > 1e-6) { ctx.fillText(`M: ${node.M}`, x, offsetY); offsetY += 14; }
      if (Number.isFinite(node.w0)) { ctx.fillText(`w₀: ${node.w0}`, x, offsetY); offsetY += 14; }
      if (Number.isFinite(node.th0)) { ctx.fillText(`θ₀: ${node.th0}`, x, offsetY); offsetY += 14; }
    }

    // Canvas picking
    const beamCanvas = document.getElementById('beamCanvas');
    beamCanvas.addEventListener('click', (e)=>{
      if (segments.length===0) return;
      const rect = beamCanvas.getBoundingClientRect(); const xClick = e.clientX - rect.left; const yClick = e.clientY - rect.top;
      const margin = 50; const totalLength = segments.reduce((s,seg)=>s+seg.L,0); const scaleX = (beamCanvas.width - 2*margin) / totalLength; const midY = Math.round(beamCanvas.height*0.35);
      const nodeXs = [margin]; let accX = margin; for (let i=0;i<segments.length;i++){ accX += segments[i].L*scaleX; nodeXs.push(accX); }
      for (let i=0;i<nodes.length;i++){ const dx = xClick - nodeXs[i]; const dy = yClick - midY; if (Math.hypot(dx,dy) < 20){ editNode(i); return; } }
      accX = margin; for (let i=0;i<segments.length;i++){ const w = segments[i].L*scaleX; if (xClick>=accX && xClick<=accX+w && Math.abs(yClick - midY) <= 18){ editSegment(i); return; } accX += w; }
    });

    // -------------------- Analysis --------------------
    function beamElementStiffness(E, I, L) {
      const EI = E * I, L2 = L * L, L3 = L2 * L;
      return [
        [ 12*EI/L3,   6*EI/L2, -12*EI/L3,   6*EI/L2 ],
        [  6*EI/L2,   4*EI/L,   -6*EI/L2,   2*EI/L  ],
        [-12*EI/L3,  -6*EI/L2,  12*EI/L3,  -6*EI/L2 ],
        [  6*EI/L2,   2*EI/L,   -6*EI/L2,   4*EI/L  ]
      ];
    }

    let dispChart=null, momentChart=null, shearChart=null, stressChart=null;
    function clearPlot(chartRef, canvasId){ if (chartRef) { try{ chartRef.destroy(); } catch(_){} } const cv = document.getElementById(canvasId); if (cv){ const c=cv.getContext('2d'); c.clearRect(0,0,cv.width,cv.height);} return null; }
    function clearAllPlots(){ dispChart=clearPlot(dispChart,'dispPlot'); momentChart=clearPlot(momentChart,'momentPlot'); shearChart=clearPlot(shearChart,'shearPlot'); stressChart=clearPlot(stressChart,'stressPlot'); }

    function solveBeam(){
      clearAllPlots(); lastU = null; elementForces = null; K_global = null;
      const n = nodes.length; if (n<2) { Swal.fire('Error','Build the beam first.','error'); return; }

      const map = buildDOFMap(); const dof = map.ndof;
      const K = Array(dof).fill().map(()=>Array(dof).fill(0));
      const F = Array(dof).fill(0);

      // Assemble
      for (let e=0; e<segments.length; e++){
        const {E,I,L} = segments[e]; const k = beamElementStiffness(E,I,L);
        const i = e, j = e+1;
        const wi = map.idxW[i]; const wj = map.idxW[j];
        const thi = map.thRight[i]; const thj = map.thLeft[j];
        const dofs = [wi, thi, wj, thj];
        for (let r=0;r<4;r++) for (let c=0;c<4;c++) K[dofs[r]][dofs[c]] += k[r][c];
      }

      // Nodal loads & springs
      for (let i=0;i<n;i++){
        const wi = map.idxW[i]; F[wi] += nodes[i].F || 0; if (nodes[i].Kv) K[wi][wi] += nodes[i].Kv;
        const hinge = isHinge(nodes[i]);
        if (!hinge && (nodes[i].Km||0) > 0){
          const r = (typeof map.thLeft[i]==='number') ? map.thLeft[i] : map.thRight[i]; if (typeof r==='number') K[r][r] += nodes[i].Km;
        }
        const M = nodes[i].M || 0; if (Math.abs(M)>0){
          if (!hinge){ const r = (typeof map.thLeft[i]==='number') ? map.thLeft[i] : map.thRight[i]; if (typeof r==='number') F[r] += M; }
          else { if (typeof map.thLeft[i] === 'number') F[map.thLeft[i]] += M/2; if (typeof map.thRight[i] === 'number') F[map.thRight[i]] += M/2; }
        }
      }

      // Distributed loads -> consistent nodal loads
      for (let e = 0; e < segments.length; e++){
        const feq = consistentLoadVector(e);
        const i = e, j = e+1;
        const wi = map.idxW[i], wj = map.idxW[j];
        const thi = map.thRight[i], thj = map.thLeft[j];
        F[wi] += feq[0]; if (typeof thi === 'number') F[thi] += feq[1];
        F[wj] += feq[2]; if (typeof thj === 'number') F[thj] += feq[3];
      }

      // Dirichlet constraints
      const knownIdx = []; const knownVal = [];
      for (let i=0;i<n;i++){
        const bc = nodes[i].bc;
        if (bc === 'fixed'){
          knownIdx.push(map.idxW[i]); knownVal.push(0);
          const rL = map.thLeft[i]; const rR = map.thRight[i]; if (typeof rL==='number'){ knownIdx.push(rL); knownVal.push(0);} if (typeof rR==='number' && rR!==rL){ knownIdx.push(rR); knownVal.push(0);} 
        } else if (bc === 'pinned'){
          knownIdx.push(map.idxW[i]); knownVal.push(0); // θ free
        }
        if (Number.isFinite(nodes[i].w0)) { const idx = map.idxW[i]; const p = knownIdx.indexOf(idx); if (p>=0) knownVal[p]=nodes[i].w0; else { knownIdx.push(idx); knownVal.push(nodes[i].w0);} }
        if (Number.isFinite(nodes[i].th0)) {
          const rL = map.thLeft[i]; const rR = map.thRight[i];
          if (typeof rL==='number'){ const p=knownIdx.indexOf(rL); if (p>=0) knownVal[p]=nodes[i].th0; else { knownIdx.push(rL); knownVal.push(nodes[i].th0);} }
          if (typeof rR==='number' && rR!==rL){ const p=knownIdx.indexOf(rR); if (p>=0) knownVal[p]=nodes[i].th0; else { knownIdx.push(rR); knownVal.push(nodes[i].th0);} }
        }
      }
      const mapKV = new Map(); for (let k=0;k<knownIdx.length;k++) mapKV.set(knownIdx[k], knownVal[k]);
      const kIdx = Array.from(mapKV.keys()).sort((a,b)=>a-b); const kVal = kIdx.map(i=>mapKV.get(i));
      const allIdx = [...Array(K.length).keys()]; const uIdx = allIdx.filter(i=>!kIdx.includes(i)); if (uIdx.length===0){ Swal.fire('Error','All DOFs are constrained — system is singular (mechanism).','error'); return; }

      const Kuu = uIdx.map(r=>uIdx.map(c=>K[r][c])); const Kuk = uIdx.map(r=>kIdx.map(c=>K[r][c])); const Fu = uIdx.map(r=>F[r]); const rhs = numeric.sub(Fu, numeric.dot(Kuk, kVal));
      try{
        const uu = numeric.solve(Kuu, rhs); const u = Array(K.length).fill(0); uIdx.forEach((d,i)=>u[d]=uu[i]); kIdx.forEach((d,i)=>u[d]=kVal[i]);
        lastU = { u, map }; K_global = K; plotDisplacement(u, map); elementForces = computeElementForces(u, map); drawBeam();
        plotMoment(); plotShearFromMoment(); plotBendingStress();
      }catch(e){
        Swal.fire('Mechanism or insufficient restraints', 'The system matrix is singular. This often happens with simply supported ends and an internal hinge with only nodal loading.', 'warning');
      }
    }

    // Internal element forces for plotting (equilibrium sign)
    function computeElementForces(u, map){
      const res = [];
      for (let e=0; e<segments.length; e++){
        const {E,I,L} = segments[e];
        const k = beamElementStiffness(E,I,L);
        const wi = map.idxW[e], thi = map.thRight[e], wj = map.idxW[e+1], thj = map.thLeft[e+1];
        const ue = [ u[wi], u[thi], u[wj], u[thj] ];
        const fe = numeric.dot(k, ue); // from stiffness
        const feq = consistentLoadVector(e); // distributed load contribution (linear q)
        const ftotal = numeric.sub(fe, feq); // internal end forces
        res.push({ V1: -ftotal[0], M1: -ftotal[1], V2: -ftotal[2], M2: -ftotal[3], L });
      }
      return res;
    }

    // -------------------- Plotting --------------------
    function plotDisplacement(u, map){
      const pts=[]; let xTot=0;
      for (let e=0;e<segments.length;e++){
        const L = segments[e].L; const wi=map.idxW[e], thi=map.thRight[e], wj=map.idxW[e+1], thj=map.thLeft[e+1];
        const u1=u[wi], th1=u[thi], u2=u[wj], th2=u[thj];
        const numPts=20; const start=(e===0)?0:1;
        for (let j=start;j<=numPts;j++){
          const xi=(j/numPts)*L, r=xi/L;
          const h1=1-3*r*r+2*r*r*r, h2=xi*(1-2*r+r*r), h3=3*r*r-2*r*r*r, h4=xi*(r*r-r);
          const w = h1*u1 + h2*th1 + h3*u2 + h4*th2;
          pts.push({ x: xTot+xi, y: w });
        }
        xTot += L;
      }
      const ctx = document.getElementById('dispPlot').getContext('2d'); if (dispChart) dispChart.destroy();
      dispChart = new Chart(ctx, {
        type:'line',
        data:{ datasets:[{ label:'Deflection', data:pts, parsing:false, borderWidth:2, fill:false, pointRadius:0 }]},
        options:{
          responsive:false,
          scales:{
            x:{ type:'linear', title:{display:true, text:'x-position'}, ticks:{ callback:(v)=>fmtSig3(v) } },
            y:{ title:{display:true, text:'Deflection (up +)'}, ticks:{ callback:(v)=>fmtSig3(v) } }
          }
        }
      });
    }

    function plotMoment(){
      if (!elementForces) { Swal.fire('Error','Please solve the beam first.'); return; }
      const pts=[]; let xBase=0;
      for (let i=0;i<elementForces.length;i++){
        const ef = elementForces[i]; const L = ef.L; const seg = segments[i]; const qw = - (seg.wd || 0) * (seg.A || 0); const qL = (seg.qL || 0) + qw; const qR = (seg.qR || 0) + qw; const dq = qR - qL;
        const samples = 48; const start=(i===0)?0:1; // avoid double-plot at shared nodes
        for (let j=start;j<=samples;j++){
          const xi = (j/samples)*L;
          const M_right = -ef.M2;
          const integral = ef.V1*(L - xi) - 0.5*qL*(L*L - xi*xi) - (dq/(6*L))*(L*L*L - xi*xi*xi);
          const M = M_right + integral;
          pts.push({ x:xBase + xi, y:M });
        }
        xBase += L;
      }
      const ctx = document.getElementById('momentPlot').getContext('2d'); if (momentChart) momentChart.destroy();
      momentChart = new Chart(ctx,{
        type:'line',
        data:{ datasets:[{ label:'Bending Moment (internal)', data:pts, parsing:false, borderWidth:2, fill:false, pointRadius:2, tension:0 }]},
        options:{
          responsive:false,
          scales:{
            x:{ type:'linear', title:{display:true, text:'x-position'}, ticks:{ callback:(v)=>fmtSig3(v) } },
            y:{ title:{display:true, text:'Moment'}, ticks:{ callback:(v)=>fmtSig3(v) } }
          }
        }
      });
    }

    function plotShearFromMoment(){
      if (!elementForces) { Swal.fire('Error','Please solve the beam first.'); return; }
      const pts=[]; let xBase=0;
      for (let i=0;i<elementForces.length;i++){
        const ef = elementForces[i]; const L = ef.L; const seg = segments[i]; const qw = - (seg.wd || 0) * (seg.A || 0); const qL = (seg.qL || 0) + qw; const qR = (seg.qR || 0) + qw; const dq = qR - qL;
        const samples = 24; const start=(i===0)?0:1;
        for (let j=start;j<=samples;j++){
          const xi = (j/samples)*L;
          const V  = ef.V1 - ( qL*xi + (dq*xi*xi)/(2*L) );
          pts.push({ x:xBase + xi, y:V });
        }
        xBase += L;
      }
      const ctx = document.getElementById('shearPlot').getContext('2d'); if (shearChart) shearChart.destroy();  <!-- FIXED -->
      shearChart = new Chart(ctx,{
        type:'line',
        data:{ datasets:[{ label:'Shear Force (internal)', data:pts, parsing:false, borderWidth:2, fill:false, pointRadius:2, tension:0 }]},
        options:{
          responsive:false,
          scales:{
            x:{ type:'linear', title:{display:true, text:'x-position'}, ticks:{ callback:(v)=>fmtSig3(v) } },
            y:{ title:{display:true, text:'Shear'}, ticks:{ callback:(v)=>fmtSig3(v) } }
          }
        }
      });
    }

    function plotBendingStress(){
      if (!elementForces) { Swal.fire('Error','Please solve the beam first.'); return; }
      const ptsTop=[]; const ptsBot=[]; let xBase=0;
      for (let i=0;i<elementForces.length;i++){
        const ef = elementForces[i]; const L = ef.L; const { St, Sb } = segments[i]; const seg = segments[i]; const qw = - (seg.wd || 0) * (seg.A || 0); const qL = (seg.qL || 0) + qw; const qR = (seg.qR || 0) + qw; const dq = qR - qL;
        const hasTop = Number.isFinite(St) && Math.abs(St) > 1e-12;
        const hasBot = Number.isFinite(Sb) && Math.abs(Sb) > 1e-12;
        const samples = 48; const start=(i===0)?0:1;
        for (let j=start;j<=samples;j++){
          const xi = (j/samples)*L;
          const M_right = -ef.M2; const integral = ef.V1*(L - xi) - 0.5*qL*(L*L - xi*xi) - (dq/(6*L))*(L*L*L - xi*xi*xi);
          const M = M_right + integral;
          if (hasTop) ptsTop.push({ x:xBase + xi, y: -M / St });
          if (hasBot) ptsBot.push({ x:xBase + xi, y:  M / Sb });
        }
        xBase += L;
      }
      const ctx = document.getElementById('stressPlot').getContext('2d'); if (stressChart) stressChart.destroy();
      const datasets = [];
      if (ptsTop.length) datasets.push({ label:'Top stress', data: ptsTop, parsing:false, borderWidth:2, fill:false, pointRadius:2, tension:0 });
      if (ptsBot.length) datasets.push({ label:'Bottom stress', data: ptsBot, parsing:false, borderWidth:2, fill:false, pointRadius:2, tension:0 });
      stressChart = new Chart(ctx,{
        type:'line',
        data:{ datasets },
        options:{
          responsive:false,
          scales:{
            x:{ type:'linear', title:{display:true, text:'x-position'}, ticks:{ callback:(v)=>fmtSig3(v) } },
            y:{ title:{display:true, text:'Stress (tension positive)'}, ticks:{ callback:(v)=>fmtSig3(v) } }
          }
        }
      });
    }

    // Consistent load vector for linear q(x) using 3-pt Gauss integration
    function consistentLoadVector(e){
      const seg = segments[e]; const L = seg.L;
      const qw = - (seg.wd || 0) * (seg.A || 0); // self-weight (down)
      const qL = (seg.qL || 0) + qw; // effective linear q(x)
      const qR = (seg.qR || 0) + qw;
      const gauss = [ {t:-Math.sqrt(3/5), w:5/9}, {t:0, w:8/9}, {t:Math.sqrt(3/5), w:5/9} ];
      const feq = [0,0,0,0];
      for (const g of gauss){
        const t=g.t, w=g.w; const x=(t+1)*L/2; const r=x/L; const J=L/2;
        const h1=1-3*r*r+2*r*r*r, h2=x*(1-2*r+r*r), h3=3*r*r-2*r*r*r, h4=x*(r*r-r);
        const qx = qL + (qR - qL) * r; // combined q
        feq[0] += w*J*h1*qx; feq[1] += w*J*h2*qx; feq[2] += w*J*h3*qx; feq[3] += w*J*h4*qx;
      }
      return feq;
    }

    // ------------- Discrete Catalog + Interpolation + Optimizer -------------
    // Helper: build N linearly interpolated catalog rows between two endpoints
    function interpolateCatalog(endpoints, N){
      const [r0, r1] = endpoints;
      const lerp = (a,b,t)=> a + (b - a)*t;
      const safeLerp = (a,b,t)=>{
        const aOK = Number.isFinite(a), bOK = Number.isFinite(b);
        if (!aOK && !bOK) return undefined;
        if (!aOK) return b;
        if (!bOK) return a;
        return lerp(a,b,t);
      };
      const out = [];
      for (let k=0; k<N; k++){
        const t = (N===1)? 0 : (k/(N-1)); // include endpoints
        out.push({
          name: `${r0.name}-${r1.name}-${k+1}`,
          I:   lerp(r0.I,  r1.I,  t),
          St:  lerp(r0.St, r1.St, t),
          Sb:  lerp(r0.Sb, r1.Sb, t),
          A:   safeLerp(r0.A,  r1.A,  t),
          wd:  safeLerp(r0.wd, r1.wd, t),
        });
      }
      return out;
    }

    // Helper: parse CSV from the Catalog textarea
    function parseCatalogCSV(raw){
      const lines = raw.split(/\r?\n/).filter(s=>s.trim().length>0);
      if (!lines.length) throw new Error('Empty input.');
      const header = lines[0].split(',').map(s=>s.trim().toLowerCase());
      const need = ['name','i','st','sb'];
      for (const k of need) if (!header.includes(k)) throw new Error('Header must include name,I,St,Sb');
      const idx = Object.fromEntries(header.map((h,i)=>[h,i]));
      const rows=[];
      for (let i=1;i<lines.length;i++){
        const cols = lines[i].split(',').map(s=>s.trim());
        if (!cols.length || !cols[0]) continue;
        const name = cols[idx['name']];
        const I  = parseFloat(cols[idx['i']]);
        const St = parseFloat(cols[idx['st']]);
        const Sb = parseFloat(cols[idx['sb']]);
        const A  = idx['a']  !== undefined && cols[idx['a']]  !== '' ? parseFloat(cols[idx['a']])  : undefined;
        const wd = idx['wd'] !== undefined && cols[idx['wd']] !== '' ? parseFloat(cols[idx['wd']]) : undefined;
        if (!(name && Number.isFinite(I) && Number.isFinite(St) && Number.isFinite(Sb))) {
          throw new Error(`Bad row ${i}: need name,I,St,Sb`);
        }
        rows.push({ name, I, St, Sb, ...(A!==undefined? {A}:{}), ...(wd!==undefined? {wd}:{} ) });
      }
      if (!rows.length) throw new Error('No valid rows found.');
      return rows;
    }

    function openSectionsCatalog(){
      const toCSV = (rows)=>[
        'name,I,St,Sb,A,wd',
        ...rows.map(r => [r.name, r.I, r.St, r.Sb, r.A ?? '', r.wd ?? ''].join(','))
      ].join('\n');

      Swal.fire({
        title: 'Sections Catalog (up to 10)',
        html: `
          <div class="hint" style="margin-bottom:6px;">
            CSV columns: <code>name,I,St,Sb,A,wd</code> — <b>A</b> & <b>wd</b> optional.
          </div>
          <textarea id="catText" style="width:520px;height:220px; font-family: ui-monospace, SFMono-Regular, Menlo, monospace;"></textarea>
          <div style="margin-top:8px; display:flex; align-items:center; gap:12px; flex-wrap:wrap;">
            <label style="display:inline-flex; align-items:center; gap:8px;">
              <input id="interpToggle" type="checkbox">
              <span>Generate N interpolated sets from 2 endpoints</span>
            </label>
            <label style="display:inline-flex; align-items:center; gap:6px;">
              N: <input id="interpN" type="number" min="2" max="10" value="10" style="width:70px; height:28px; padding:0 8px;">
            </label>
            <button id="previewBtn" type="button" style="height:28px; padding:0 10px; border-radius:6px; border:1px solid #99c; background:#eef;">Preview</button>
          </div>
          <div class="hint" style="margin-top:4px;">
            With the option checked and exactly two rows, the app linearly interpolates <code>I, St, Sb</code> (and <code>A, wd</code> when both endpoints define them).
          </div>
        `,
        didOpen: ()=>{
          document.getElementById('catText').value = toCSV(sectionsCatalog);

          // Preview handler
          document.getElementById('previewBtn').addEventListener('click', ()=>{
            const raw = document.getElementById('catText').value.trim();
            const useInterp = !!document.getElementById('interpToggle').checked;
            const N = Math.max(2, Math.min(10, parseInt(document.getElementById('interpN').value || '10', 10)));
            try{
              let rows = parseCatalogCSV(raw);
              if (useInterp){
                if (rows.length !== 2) throw new Error('Interpolation preview requires exactly 2 rows.');
                rows = interpolateCatalog(rows, N);
              } else {
                rows = rows.slice(0, 10);
              }

              // Sort by capacity (min St,Sb) for monotonic optimizer steps
              rows.sort((a,b)=> Math.min(a.St,a.Sb) - Math.min(b.St,b.Sb));

              // Show charts
              Swal.fire({
                title: 'Interpolated Catalog Preview',
                html: `
                  <div style="display:grid; grid-template-columns:1fr; gap:10px; width:min(680px, 90vw);">
                    <div><canvas id="chartS" width="640" height="240"></canvas></div>
                    <div><canvas id="chartI" width="640" height="240"></canvas></div>
                  </div>
                `,
                didOpen: ()=>{
                  const labels = rows.map((r,i)=> (r.name || `S${i+1}`));
                  const Sts = rows.map(r=>r.St);
                  const Sbs = rows.map(r=>r.Sb);
                  const Is  = rows.map(r=>r.I);

                  const ctxS = document.getElementById('chartS').getContext('2d');
                  new Chart(ctxS, {
                    type:'line',
                    data:{ labels, datasets:[
                      { label:'St (top)', data:Sts, borderWidth:2, fill:false, pointRadius:2, tension:0 },
                      { label:'Sb (bottom)', data:Sbs, borderWidth:2, fill:false, pointRadius:2, tension:0 }
                    ]},
                    options:{ responsive:false, scales:{ y:{ title:{display:true, text:'Section modulus S'} } } }
                  });

                  const ctxI = document.getElementById('chartI').getContext('2d');
                  new Chart(ctxI, {
                    type:'line',
                    data:{ labels, datasets:[
                      { label:'I', data:Is, borderWidth:2, fill:false, pointRadius:2, tension:0 }
                    ]},
                    options:{ responsive:false, scales:{ y:{ title:{display:true, text:'Moment of inertia I'} } } }
                  });
                },
                showConfirmButton: true,
                confirmButtonText: 'Close'
              });
            }catch(err){
              Swal.fire('Preview error', err.message || String(err), 'error');
            }
          });
        },
        preConfirm: ()=>{
          const raw = document.getElementById('catText').value.trim();
          const useInterp = !!document.getElementById('interpToggle').checked;
          const N = Math.max(2, Math.min(10, parseInt(document.getElementById('interpN').value || '10', 10)));
          try{
            let rowsOut = parseCatalogCSV(raw);
            if (useInterp){
              if (rowsOut.length !== 2) throw new Error('Interpolation mode requires exactly 2 rows.');
              rowsOut = interpolateCatalog(rowsOut, N);
            }
            rowsOut = rowsOut.slice(0, 10);
            rowsOut.sort((a,b)=> Math.min(a.St,a.Sb) - Math.min(b.St,b.Sb));
            return rowsOut;
          }catch(err){
            Swal.showValidationMessage(err.message || String(err));
            return false;
          }
        },
        showCancelButton: true,
        confirmButtonText: 'Save'
      }).then(res=>{
        if (!res.value) return;
        sectionsCatalog = res.value;
        Swal.fire({icon:'success', title:'Catalog saved', timer:900, showConfirmButton:false});
      });
    }

    function applySectionToSegment(seg, catIdx){
      const s = sectionsCatalog[catIdx];
      if (!s) return;
      seg.I  = s.I;
      seg.St = s.St;
      seg.Sb = s.Sb;
      if (s.A  !== undefined) seg.A  = s.A;
      if (s.wd !== undefined) seg.wd = s.wd;
      seg.secIndex = catIdx;
    }

    function measureSegmentMomentExtrema(){
      if (!elementForces) return [];
      const out=[];
      for (let i=0;i<elementForces.length;i++){
        const ef=elementForces[i], L=ef.L, seg=segments[i];
        const qw = - (seg.wd || 0) * (seg.A || 0);
        const qL = (seg.qL || 0) + qw, qR = (seg.qR || 0) + qw, dq = qR - qL;
        let MposMax = -Infinity, MnegMin = Infinity;
        const samples=96;
        for (let j=0;j<=samples;j++){
          const xi=(j/samples)*L;
          const M_right=-ef.M2;
          const M = M_right + ( ef.V1*(L - xi) - 0.5*qL*(L*L - xi*xi) - (dq/(6*L))*(L*L*L - xi*xi*xi) );
          if (M > MposMax) MposMax = M;
          if (M < MnegMin) MnegMin = M;
        }
        out.push({MposMax, MnegMin});
      }
      return out;
    }

    function measureSegmentStressExtrema(){
      if (!elementForces) return [];
      const out = [];
      for (let i=0;i<segments.length;i++){
        const ef = elementForces[i], L = ef.L, seg = segments[i];
        const qw = - (seg.wd || 0) * (seg.A || 0);
        const qL = (seg.qL || 0) + qw, qR = (seg.qR || 0) + qw, dq = qR - qL;
        let sigMaxAbs = 0, sigTMax = 0, sigBMax = 0;
        const St = seg.St, Sb = seg.Sb;
        const samples = 96;
        for (let j=0;j<=samples;j++){
          const xi = (j/samples)*L;
          const M_right = -ef.M2;
          const M = M_right + ( ef.V1*(L - xi) - 0.5*qL*(L*L - xi*xi) - (dq/(6*L))*(L*L*L - xi*xi*xi) );
          const sigTop = (Number.isFinite(St) && Math.abs(St)>1e-12) ? (-M/ St) : 0;
          const sigBot = (Number.isFinite(Sb) && Math.abs(Sb)>1e-12) ? ( M/ Sb) : 0;
          const m = Math.max(Math.abs(sigTop), Math.abs(sigBot));
          if (m > sigMaxAbs) sigMaxAbs = m;
          if (Math.abs(sigTop) > Math.abs(sigTMax)) sigTMax = sigTop;
          if (Math.abs(sigBot) > Math.abs(sigBMax)) sigBMax = sigBot;
        }
        out.push({ sigMaxAbs, sigTMax, sigBMax });
      }
      return out;
    }

    function optimizeSectionsDiscrete(){
      if (!sectionsCatalog || sectionsCatalog.length === 0){
        Swal.fire('Catalog needed','Please create a sections catalog first.','info'); return;
      }
      if (!lastU || !elementForces) { try { solveBeam(); } catch(_){} }
      if (!lastU || !elementForces){ Swal.fire('Error','Solve the beam first.','error'); return; }

      Swal.fire({
        title: 'Optimize Sections (Discrete)',
        html: `
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;">
            <label><span>σ_allow (absolute)</span><input id="sigAllowD" class="swal2-input" value="20000"></label>
            <label><span>Max iterations</span><input id="maxIterD" class="swal2-input" value="40"></label>
            <label><span>Over-limit tolerance</span><input id="tolOver" class="swal2-input" value="0.01"></label>
            <label><span>Under-limit margin</span><input id="tolUnder" class="swal2-input" value="0.10"></label>
          </div>
          <hr style="margin:6px 0;">
          <label style="gap:8px;"><span>Start with single catalog section for all segments</span>
            <input id="singleStartD" type="checkbox" checked>
          </label>
          <div class="hint" style="margin-top:6px;">Heuristic: bump up if |σ| > σ_allow·(1+tolOver); try bumping down if |σ| < σ_allow·(1−tolUnder). Re-solves each step.</div>
        `,
        preConfirm: ()=>{
          const v = id => parseFloat(document.getElementById(id).value);
          const sigAllow = v('sigAllowD');
          const maxIters = parseInt(document.getElementById('maxIterD').value);
          const tolOver  = v('tolOver');
          const tolUnder = v('tolUnder');
          const singleStart = document.getElementById('singleStartD').checked;
          if (!(sigAllow>0)) { Swal.showValidationMessage('σ_allow must be > 0'); return false; }
          if (!(maxIters>=1)) { Swal.showValidationMessage('Max iterations must be ≥ 1'); return false; }
          if (!(tolOver>=0 && tolUnder>=0)) { Swal.showValidationMessage('Tolerances must be ≥ 0'); return false; }
          return { sigAllow, maxIters, tolOver, tolUnder, singleStart };
        },
        showCancelButton: true,
        confirmButtonText: 'Run'
      }).then(res=>{
        if (!res.value) return;
        runDiscreteOptimization(res.value);
      });
    }

    function runDiscreteOptimization(cfg){
      // Initial assignment
      if (cfg.singleStart){
        const ext = measureSegmentMomentExtrema();
        let globalReqS = 0;
        for (const e of ext){
          globalReqS = Math.max(globalReqS, Math.abs(e.MposMax)/cfg.sigAllow, Math.abs(e.MnegMin)/cfg.sigAllow);
        }
        let k0 = sectionsCatalog.findIndex(s => (s.St >= globalReqS && s.Sb >= globalReqS));
        if (k0 < 0) k0 = sectionsCatalog.length - 1;
        for (let i=0;i<segments.length;i++) applySectionToSegment(segments[i], k0);
      } else {
        for (let i=0;i<segments.length;i++){
          if (typeof segments[i].secIndex !== 'number'){
            let best=0, bestErr=Infinity;
            for (let k=0;k<sectionsCatalog.length;k++){
              const s = sectionsCatalog[k];
              const err = Math.abs(Math.log(((segments[i].St||1))/(s.St||1))) + Math.abs(Math.log(((segments[i].Sb||1))/(s.Sb||1)));
              if (err < bestErr){ bestErr=err; best=k; }
            }
            applySectionToSegment(segments[i], best);
          }
        }
      }

      let iter = 0;
      for (; iter < cfg.maxIters; iter++){
        try { solveBeam(); } catch(_) {}
        if (!elementForces) { Swal.fire('Error','Solve failed during optimization.','error'); return; }

        const sigs = measureSegmentStressExtrema();
        let anyChange = false;

        for (let i=0;i<segments.length;i++){
          const seg = segments[i], sInfo = sigs[i];
          const over  = sInfo.sigMaxAbs > cfg.sigAllow * (1 + cfg.tolOver);
          const under = sInfo.sigMaxAbs < cfg.sigAllow * (1 - cfg.tolUnder);

          let k = (typeof seg.secIndex === 'number') ? seg.secIndex : 0;

          if (over && k < sectionsCatalog.length - 1){
            k++;
            applySectionToSegment(seg, k);
            anyChange = true;
          } else if (under && k > 0){
            k--;
            applySectionToSegment(seg, k);
            anyChange = true;
          }
        }

        if (!anyChange) break;
      }

      try { solveBeam(); } catch(_) {}
      drawBeam();
      plotBendingStress();

      Swal.fire({
        icon:'success',
        title:'Discrete optimization complete',
        html:`Iterations: <b>${iter}</b><br>Catalog size: <b>${sectionsCatalog.length}</b>`,
        timer: 1300,
        showConfirmButton: false
      });
    }

    // -------------------- CSV Export --------------------
    function exportCSV(){
      if (!lastU || !elementForces){ Swal.fire('Note','Solve first to export results.','info'); return; }
      const { u, map } = lastU; const rows = [['x','w(up+)','V','M','sigma_top(tension+)','sigma_bottom(tension+)']]; let xTot=0;
      for (let i=0;i<segments.length;i++){
        const seg = segments[i]; const L=seg.L; const wi=map.idxW[i], thi=map.thRight[i], wj=map.idxW[i+1], thj=map.thLeft[i+1];
        const u1=u[wi], th1=u[thi], u2=u[wj], th2=u[thj]; const ef = elementForces[i];
        const numPts=20; const start=(i===0)?0:1;
        for (let j=start;j<=numPts;j++){
          const xi=(j/numPts)*L, r=xi/L; const h1=1-3*r*r+2*r*r*r, h2=xi*(1-2*r+r*r), h3=3*r*r-2*r*r*r, h4=xi*(r*r-r);
          const w=h1*u1+h2*th1+h3*u2+h4*th2;
          const qw = - (seg.wd || 0) * (seg.A || 0), qL = (seg.qL || 0) + qw, qR = (seg.qR || 0) + qw, dq = qR - qL;
          const V = ef.V1 - ( qL*xi + (dq*xi*xi)/(2*L) );
          const M_right = -ef.M2; const M = M_right + ( ef.V1*(L - xi) - 0.5*qL*(L*L - xi*xi) - (dq/(6*L))*(L*L*L - xi*xi*xi) );
          const sigT = (Number.isFinite(seg.St) && Math.abs(seg.St)>1e-12) ? (-M/seg.St) : '';
          const sigB = (Number.isFinite(seg.Sb) && Math.abs(seg.Sb)>1e-12) ? ( M/seg.Sb) : '';
          rows.push([ (xTot+xi).toFixed(6), w, V, M, sigT, sigB ]);
        }
        xTot += L;
      }
      const csv = rows.map(r=>r.join(','));
      const blob = new Blob([csv.join('\n')], {type:'text/csv'});
      const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = 'beam_results.csv'; a.click(); URL.revokeObjectURL(a.href);
    }

    // Expose API
    window.buildBeam = buildBeam;
    window.solveBeam = solveBeam;
    window.plotMoment = plotMoment;
    window.plotShearFromMoment = plotShearFromMoment;
    window.plotBendingStress = plotBendingStress;
    window.clearAllPlots = clearAllPlots;
    window.exportCSV = exportCSV;
    window.openSectionsCatalog = openSectionsCatalog;
    window.optimizeSectionsDiscrete = optimizeSectionsDiscrete;

    // Safe initial build
    try { buildBeam(); } catch(e) { console.error(e); }
