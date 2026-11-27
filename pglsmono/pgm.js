/* PGLS-mono tab of the loop-closure dashboard.
   For every VINS keyframe k: P, F, V_min = min(V_kj, V_jk) against every earlier keyframe j.
   Sources: VINS poses (pose graph / VIO) x Depth Anything AC depth (metric / raw), and the ground truth
   of the v7 run (GT poses + LiDAR depth) at the same keyframe times.
   Data: pglsmono/data/*.js written by build_pglsmono_web.py. */
(function(){
"use strict";
const $=id=>document.getElementById(id);
const IDX=window.PGM_INDEX||{seqs:[],info:{}};
const SRC={
  pg_metric:{name:"VINS pose graph + Depth Anything, metric", pose:"pg", depth:"metric",
    help:"Estimated poses (pose graph, i.e. with any loop corrections) and Depth Anything AC depth scaled to metres with the VINS landmarks of each keyframe (d = a/z + b). This is what a robot would have online."},
  vio_metric:{name:"VINS VIO + Depth Anything, metric", pose:"vio", depth:"metric",
    help:"Like the first, but each keyframe keeps its own VIO pose (no loop corrections)."},
  pg_raw:{name:"VINS pose graph + Depth Anything, raw", pose:"pg", depth:"raw",
    help:"Depth Anything AC relative depth without any scale (median of each image = 1). Distances are in metres but this depth is not, so P (distance / scene depth) becomes much stricter and V compares unscaled points. A no-scale baseline only."},
  vio_raw:{name:"VINS VIO + Depth Anything, raw", pose:"vio", depth:"raw",
    help:"VIO poses with the unscaled relative depth (see raw above)."},
  gt:{name:"Ground truth: GT poses + LiDAR depth (v7)", pose:"gt", depth:"lidar",
    help:"The v7 run: ground-truth poses and LiDAR depth (Ouster, 120 m), for the RGB camera frame nearest in time to each VINS keyframe. Use it as the reference for the estimated values."},
};
const CH=[["P","P (pose)"],["F","F (frustum)"],["V","V_min (occlusion)"]];
const CHCOL={P:"#58a6ff",F:"#3fb950",V:"#d2a8ff"};
const SHORT={pg_metric:"PG + metric",vio_metric:"VIO + metric",pg_raw:"PG + raw",vio_raw:"VIO + raw",gt:"GT"};
const DTICKS=[0.5,1,2,5,10,20,50,120];
const fx=(v,d=2)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d);
const G={seq:null,src:"pg_metric",cmp:true,k:0,t:0,playing:false,speed:2,last:0,plane:"xy",colorBy:"P",mode:"image",
         align:"100",klScope:"all",klCh:"P",gap:10,pickMode:"fixed",picks:[],pickK:-1,slot:0,hover:null,view:null,bounds:null,theta:0,D:null,M:null,MG:null};
const loaded={}, mats={};

/* ------------------------------------------------------------------ layout */
function build(){
  const el=$("pgm"); if(!el) return false;
  el.innerHTML=`
  <div class="col">
    <div class="card" id="gLeftTop">
      <div class="gsel">
        <label>Sequence</label><select id="gSeq"></select>
        <label>Values from</label><select id="gSrc">${Object.keys(SRC).map(s=>`<option value="${s}">${SRC[s].name}</option>`).join("")}</select>
      </div>
      <div class="ghelp" id="gHelp"></div>
      <label class="k" id="gCmpBox" style="display:block;margin:4px 0 6px"><input type="checkbox" id="gCmp" checked> also show the ground truth (white line on the plots, "GT" in the captions)</label>
      <h2>Sequence</h2><table id="gTSeq"></table><div id="gWarn"></div>
      <h2 style="margin-top:10px">Agreement between versions <span class="k">(KL divergence)</span></h2>
      <div class="ghelp" id="gKlHelp"></div><table id="gTKl" class="gkl"></table>
      <details id="gKlMore"><summary class="k">every pair of versions</summary>
        <div class="row" style="margin:4px 0"><span class="gseg" id="gKlCh"><button data-v="P" class="on">P</button><button data-v="F">F</button><button data-v="V">V_min</button></span></div>
        <div style="overflow-x:auto"><table id="gTKl2" class="gkl" style="font-size:11px"></table></div><div class="ghelp">row = reference p, column = q: KL(p ‖ q). Not symmetric.</div></details>
      <h2 style="margin-top:10px">Current keyframe <span class="k" id="gResNote"></span></h2><table id="gTRes"></table>
    </div>
    <div class="card" id="gCur">
      <div class="ghead"><h2>Current keyframe</h2><span class="k" id="gCurCap"></span>
        <span style="flex:1"></span><span class="gseg" id="gMode"><button data-v="image" class="on">image</button><button data-v="depth">depth</button></span></div>
      <div class="gimg" id="gCurBox"></div>
      <div id="gCbar" style="display:none"><img class="gbar" src="pglsmono/colorbar.png" style="width:100%" alt="">
        <div class="gticks">${DTICKS.map(m=>`<span style="left:${(100*Math.log(m/0.5)/Math.log(240)).toFixed(1)}%;transform:translateX(${m===0.5?"0":m===120?"-100%":"-50%"})">${m} m</span>`).join("")}</div></div>
    </div>
  </div>
  <div class="col">
    <div class="card" id="gTraj">
      <div class="ctl">
        <button id="gPlay" class="primary" title="start / pause (space)">&#9654; Play</button>
        <button id="gEnd" title="jump to the last keyframe">End</button>
        <input type="range" id="gSlider" min="0" max="1" value="0" step="1">
        <span id="gTl" class="k"></span>
        <label>speed <select id="gSpeed"><option>1</option><option selected>2</option><option>5</option><option>10</option><option>20</option></select>&times;</label>
        <label>view <select id="gPlane"><option value="xy" selected>x-y (top)</option><option value="xz">x-z</option><option value="yz">y-z</option></select></label>
        <label>dots <select id="gColor"><option value="P" selected>P</option><option value="F">F</option><option value="V">V_min</option><option value="none">none</option></select></label>
        <label id="gOtherBox"><input type="checkbox" id="gOther" checked> <span id="gOtherLab">ground truth (dashed)</span></label>
        <span id="gAlBox" class="k" title="Umeyama (SE3, no scale) alignment of the ground truth to the estimate, fitted on the first N keyframes">align on first
          <span class="gseg" id="gAlign"><button data-v="100" class="on">100</button><button data-v="150">150</button><button data-v="200">200</button><button data-v="250">250</button><button data-v="all">all</button></span></span>
        <label title="arrows show where the camera looks: current keyframe (white), the picks (their colours) and every few keyframes along the path (grey)"><input type="checkbox" id="gArrows" checked> view arrows</label>
        <label><input type="checkbox" id="gRot" checked> rotate to fit</label>
        <button id="gReset" title="reset zoom and pan">Reset view</button>
      </div>
      <div class="cv"><canvas id="gCvTraj"></canvas></div>
      <div class="leg" style="margin-top:6px" id="gTrajLeg"></div>
    </div>
    <div class="card" id="gBot">
      <div class="ghead"><h2>Keyframe k vs every earlier keyframe j</h2>
        <span style="flex:1"></span>
        <label class="k">picks</label>
        <span class="gseg" id="gPickMode" title="how the 4 coloured keyframes are chosen">
          <button data-v="fixed" class="on" title="the same 4 keyframes while k moves, so you can follow their values">fixed</button>
          <button data-v="random" title="4 new random earlier keyframes at every keyframe k">random each k</button>
          <button data-v="top" title="the 4 earlier keyframes with the highest P at every keyframe k (at least 2 s apart)">top-4 P each k</button></span>
        <button id="gRoll" title="draw 4 new random keyframes">&#x21bb; re-roll</button>
        <label class="k">min time gap <input type="number" id="gGap" value="10" min="0" step="1"> s</label>
      </div>
      <div id="gRead"></div>
      <div id="gBotGrid">
        <div id="gPlots">${CH.map(([c])=>`<div class="cv"><canvas id="gCv${c}"></canvas></div>`).join("")}</div>
        <div id="gImgs">${[0,1,2,3].map(i=>`<div class="gimg" id="gImg${i}"></div>`).join("")}</div>
      </div>
    </div>
  </div>`;
  if(!$("gTip")){const t=document.createElement("div"); t.id="gTip"; document.body.appendChild(t);}
  return true;
}

/* ------------------------------------------------------------------ data */
function loadJS(src,test){
  if(test()) return Promise.resolve();
  if(loaded[src]) return loaded[src];
  return loaded[src]=new Promise((res,rej)=>{const s=document.createElement("script"); s.src=src+"?v="+(IDX.built||"").replace(/\D/g,"");
    s.onload=()=>res(); s.onerror=()=>{delete loaded[src]; rej(new Error("cannot load "+src));}; document.head.appendChild(s);});
}
function b64u8(s){const b=atob(s), u=new Uint8Array(b.length); for(let i=0;i<b.length;i++) u[i]=b.charCodeAt(i); return u;}
async function loadMat(seq,ver){
  const key=seq+"|"+ver; if(mats[key]) return mats[key];
  await loadJS(`pglsmono/data/${seq}/${ver}.js`,()=>window.PGMV&&window.PGMV[key]);
  const raw=window.PGMV[key]; mats[key]={P:b64u8(raw.P),F:b64u8(raw.F),V:b64u8(raw.V)}; delete window.PGMV[key];
  return mats[key];
}
const off=k=>k*(k-1)/2;
const hasGT=()=>!!(G.D&&G.D.versions&&G.D.versions.indexOf("gt")>=0);
const showGT=()=>G.cmp&&G.src!=="gt"&&G.MG;
function val(ch,k,j,M){ M=M||G.M; if(!M||j===k||j<0) return NaN; const q=j<k?M[ch][off(k)+j]:M[ch][off(j)+k]; return q===255?NaN:q/250; }
const gval=(ch,k,j)=>G.MG?val(ch,k,j,G.MG):NaN;
const posePath=()=>G.D[SRC[G.src].pose]||G.D.pg;
function dist(k,j){ const p=posePath(); const a=p.x[k]-p.x[j], b=p.y[k]-p.y[j], c=p.z[k]-p.z[j]; return Math.sqrt(a*a+b*b+c*c); }
const old=(k,j)=>G.D.t[k]-G.D.t[j]>=G.gap;
const rel=(k,j)=>j<k?`${fx(G.D.t[k]-G.D.t[j],1)} s before k`:`${fx(G.D.t[j]-G.D.t[k],1)} s after k`;

/* ------------------------------------------------------------------ picks */
function eligible(k,strict){ const out=[]; for(let j=0;j<k;j++) if(old(k,j)) out.push(j); if(!out.length&&!strict) for(let j=0;j<k;j++) out.push(j); return out; }
function colours(){ const h0=Math.random()*360; return [0,1,2,3].map(i=>`hsl(${Math.round(h0+i*90+(Math.random()*30-15))%360},85%,62%)`); }
function ensureSlots(){ if(G.picks.length<4){ const c=colours(); while(G.picks.length<4) G.picks.push({j:-1,c:c[G.picks.length]}); } }
function sortPicks(){ G.picks.sort((a,b)=>(a.j<0)-(b.j<0)||a.j-b.j); }
const live=()=>G.picks.filter(p=>p.j>=0);
function drawRandom(strict){ ensureSlots(); const el=eligible(G.k,strict); G.picks.forEach(p=>{ p.j=el.length?el.splice(Math.floor(Math.random()*el.length),1)[0]:-1; }); sortPicks(); }
function drawTop(){
  ensureSlots(); const k=G.k, L=[]; for(let j=0;j<k;j++){ if(!old(k,j)) continue; const v=val("P",k,j); if(isFinite(v)&&v>0) L.push([v,j]); }
  L.sort((a,b)=>b[0]-a[0]); const S=[];                                  // highest P, at least 2 s apart
  for(const x of L){ if(S.length>=4) break; if(S.every(j=>Math.abs(G.D.t[j]-G.D.t[x[1]])>=2)) S.push(x[1]); }
  G.picks.forEach((p,i)=>{p.j=S[i]!==undefined?S[i]:-1;}); sortPicks();
}
function topWhy(){
  const k=G.k; let n=0,p=0; for(let j=0;j<k;j++){ if(!old(k,j)) continue; n++; if(val("P",k,j)>0) p++; }
  return `only ${p} of the ${n} keyframes ≥ ${G.gap} s before k have P &gt; 0<br><span class="k">(the top-4 are also kept ≥ 2 s apart from each other)</span>`;
}
function updatePicks(){
  ensureSlots();
  if(G.pickMode==="random"){ if(G.pickK!==G.k){ drawRandom(G.playing); G.pickK=G.k; } return; }
  if(G.pickMode==="top"){ drawTop(); G.pickK=G.k; return; }
  // fixed: picks stay where they are, also ahead of k (their values then come from keyframe j's own row);
  // only empty slots are filled, with random earlier keyframes
  if(!G.picks.some(p=>p.j<0)) return;
  const el=eligible(G.k,G.playing).filter(j=>!G.picks.some(p=>p.j===j));
  G.picks.forEach(p=>{ if(p.j<0) p.j=el.length?el.splice(Math.floor(Math.random()*el.length),1)[0]:-1; });
  sortPicks();
}
function setPickMode(m){ G.pickMode=m; G.pickK=-1; $("gPickMode").querySelectorAll("button").forEach(b=>b.classList.toggle("on",b.dataset.v===m)); render(); }
function reroll(){ if(G.pickMode==="top") setPickModeQuiet("fixed"); G.picks=[]; ensureSlots(); drawRandom(false); G.pickK=G.k; G.slot=0; render(); }
function setPickModeQuiet(m){ G.pickMode=m; $("gPickMode").querySelectorAll("button").forEach(b=>b.classList.toggle("on",b.dataset.v===m)); }
function replacePick(j){
  if(j<0||j===G.k||j>=G.D.t.length||G.picks.some(p=>p.j===j)) return;
  setPickModeQuiet("fixed");                                              // a hand-picked keyframe stays
  const empty=G.picks.find(p=>p.j<0);
  if(empty) empty.j=j; else { G.picks[G.slot%4].j=j; G.slot=(G.slot+1)%4; }
  render();
}

/* ------------------------------------------------------------------ tables + images */
function fillSeq(){
  const I=G.D.info;
  $("gTSeq").innerHTML=
   `<tr><td class="k">Keyframes</td><td>${I.n} <span class="k">over ${fx(I.duration,1)} s</span></td></tr>
    <tr><td class="k">Estimated path / end–start gap</td><td>${fx(I.path_len,1)} m / ${fx(I.end_gap,1)} m</td></tr>
    <tr><td class="k">Median scene depth D</td><td>${fx(I.D_med,1)} m <span class="k">(Depth Anything, VINS-scaled)</span></td></tr>
    <tr><td class="k">Own scale fit</td><td>${fx(100*I.own_fit,0)} % <span class="k">of keyframes (else previous fit)</span></td></tr>
    <tr><td class="k">Processing</td><td>${fx(I.ms_med,0)} ms <span class="k">median per keyframe</span></td></tr>
    <tr><td class="k">Loop corrections</td><td>${I.loop_diff>1e-3?`max |P<sub>pg</sub> − P<sub>vio</sub>| = ${fx(I.loop_diff,3)}`:`<span class="k">none (pose graph = VIO)</span>`}</td></tr>
    <tr><td class="k">Ground truth</td><td>${I.gt?`v7 frame within ${fx(1000*I.gt_match_dt,0)} ms <span class="k">(median; max ${fx(1000*I.gt_match_dt_max,0)} ms)</span>`:`<span class="k">not available</span>`}</td></tr>`;
  $("gWarn").innerHTML=(G.D.warn||[]).map(w=>`<div class="warn">⚠ ${w}</div>`).join("");
  fillKl();
}
function fillKl(){
  const K=G.D.kl; if(!K){ $("gTKl").innerHTML=`<tr><td class="k">not computed – rebuild the dashboard</td></tr>`; $("gKlHelp").textContent=""; return; }
  const S=K[G.klScope], names=K.names, est=names.filter(n=>n!=="gt"), ref=names.indexOf("gt")>=0;
  const score=(ch,v)=>{ if(ref) return S[ch].gt[v];                         // KL(ground truth || version)
    const o=est.filter(x=>x!==v).map(x=>S[ch][x][v]).filter(x=>x!==null); return o.length?o.reduce((a,b)=>a+b,0)/o.length:null; };
  $("gKlHelp").innerHTML=(ref?`Each version's P, F and V<sub>min</sub> matrices are compared with the ground truth as 2-D distributions: KL(GT ‖ version), over all ${S.pairs.toLocaleString()} keyframe pairs. KL ≥ 0, 0 = identical: lower is better. Ranked from most to least coherent with the ground truth; all sequences: second tab.`
      :`No ground truth for this sequence: each version is scored by its mean KL divergence to the other three (0 = identical).`)+
      ` Pairs where a version has no value are left out (coverage: ${est.map(v=>SHORT[v]+" "+fx(100*S.cover[v],0)+"%").join(", ")}).`;
  const cols=["P","F","V"].map(ch=>est.map(v=>[v,score(ch,v)]).sort((a,b)=>(a[1]===null)-(b[1]===null)||a[1]-b[1]));
  const cell=x=>x[1]===null?`<td class="k">${SHORT[x[0]]} –</td>`:`<td><span class="gkn" style="color:${VCOL[x[0]]}">${SHORT[x[0]]}</span> ${fx(x[1],3)}</td>`;
  $("gTKl").innerHTML=`<tr><th></th><th>P</th><th>F</th><th>V<sub>min</sub></th></tr>`+est.map((_,i)=>`<tr><td class="k">${i+1}.</td>${cols.map(c=>cell(c[i])).join("")}</tr>`).join("");
  const C=S[G.klCh];
  $("gTKl2").innerHTML=`<tr><th>p \ q</th>${names.map(n=>`<th>${SHORT[n]}</th>`).join("")}</tr>`+
    names.map(a=>`<tr><td class="k">${SHORT[a]}</td>${names.map(b=>`<td>${a===b?"·":fx(C[a][b],3)}</td>`).join("")}</tr>`).join("");
}
const VCOL={pg_metric:"#58a6ff",vio_metric:"#79c0ff",pg_raw:"#f0b429",vio_raw:"#f78166",gt:"#e6e9ee"};
function sceneD(k){ const s=SRC[G.src]; return s.depth==="lidar"?`${fx(G.D.gt_D&&G.D.gt_D[k],2)} m (LiDAR)`:s.depth==="raw"?`${fx(G.D.Draw[k],2)} (raw, no unit)`:`${fx(G.D.D[k],2)} m`; }
function fillRes(){
  const k=G.k, D=G.D; let n=0,nP=0,nF=0,nV=0,n3=0,best=-1,bv=-1;
  for(let j=0;j<k;j++){ if(!old(k,j)) continue; n++;
    const p=val("P",k,j), f=val("F",k,j), v=val("V",k,j);
    if(p>=0.5) nP++; if(f>=0.5) nF++; if(v>=0.3) nV++; if(p>=0.5&&f>=0.5&&v>=0.3) n3++; if(p>bv){bv=p;best=j;} }
  $("gResNote").textContent=`(k = ${k})`;
  $("gTRes").innerHTML=
   `<tr><td class="k">Time / scene depth D<sub>k</sub></td><td>${fx(D.t[k],1)} s / ${sceneD(k)}</td></tr>
    <tr><td class="k">Scale fit</td><td>${D.fit[k]} <span class="k">· ${D.nlm[k]} landmarks · AbsRel ${fx(D.absrel[k],3)}</span></td></tr>
    <tr><td class="k">Earlier keyframes ≥ ${G.gap} s older</td><td>${n}</td></tr>
    <tr><td class="k">P ≥ 0.5 / F ≥ 0.5 / V<sub>min</sub> ≥ 0.3</td><td>${nP} / ${nF} / ${nV}</td></tr>
    <tr><td class="k">All three</td><td><b>${n3}</b></td></tr>
    <tr><td class="k">Highest P</td><td>${best>=0&&bv>0?`${fx(bv,3)} <span class="k">with j = ${best} (${fx(dist(k,best),1)} m, ${fx(D.t[k]-D.t[best],1)} s earlier)</span>`:"–"}</td></tr>`;
}
const imgSrc=(k)=>`pglsmono/${G.mode==="depth"?"depth":"img"}/${G.seq}/${k}.jpg`;
function setImg(box,k){
  const want=imgSrc(k); let im=box.querySelector("img");
  if(!G.D.img[k]&&G.mode==="image"){ if(im) im.remove(); if(!box.querySelector(".empty")) box.insertAdjacentHTML("afterbegin",`<div class="empty">no image for this keyframe</div>`); return; }
  const e=box.querySelector(".empty"); if(e) e.remove();
  if(!im){ im=document.createElement("img"); im.alt=""; im.onerror=()=>{im.style.visibility="hidden";}; im.onload=()=>{im.style.visibility="visible";}; box.insertBefore(im,box.firstChild); }
  if(im.getAttribute("src")!==want) im.setAttribute("src",want);
}
function valChip(c,k,j,col){
  const v=val(c,k,j), g=showGT()?gval(c,k,j):null, nm=c==="V"?"V<sub>min</sub>":c;
  return `<span>${nm} <b style="color:${col}">${fx(v,2)}</b>${g!==null?` <span class="k">GT ${fx(g,2)}</span>`:""}</span>`;
}
/* GTA-style stars in the top-right corner of each pick: always in the order P, F, V_min;
   a star is lit when P >= 0.5, F >= 0.3, V_min >= 0.3 */
const STAR_RULE=[["P",0.5],["F",0.3],["V",0.3]];
const STAR_PATH="M12 1.8l3.1 6.6 7.1.8-5.3 4.9 1.5 7.1L12 17.6l-6.4 3.6 1.5-7.1L1.8 9.2l7.1-.8z";
function stars(k,j){
  let n=0; const s=STAR_RULE.map(([c,th])=>{ const v=val(c,k,j), on=isFinite(v)&&v>=th; if(on) n++;
    return `<span class="gstar${on?" on":""}" title="${c==="V"?"V_min":c} ${fx(v,2)} ${on?"≥":"<"} ${th}"><svg viewBox="0 0 24 24"><path d="${STAR_PATH}"/></svg><i>${c==="V"?"V":c}</i></span>`; }).join("");
  return {html:s, n};
}
function showImages(){
  const k=G.k, cur=$("gCurBox");
  setImg(cur,k);
  let cc=cur.querySelector(".cap"); if(!cc){ cc=document.createElement("div"); cc.className="cap"; cur.appendChild(cc); }
  cc.innerHTML=`<b>k = ${k}</b> · t = ${fx(G.D.t[k],1)} s <span class="k">/ ${fx(G.D.t[G.D.t.length-1],1)} s</span>`;
  $("gCurCap").textContent=`k = ${k} · t = ${fx(G.D.t[k],1)} s`;
  $("gCbar").style.display=G.mode==="depth"?"block":"none";
  for(let i=0;i<4;i++){
    const box=$("gImg"+i), p=G.picks[i];
    if(!p||p.j<0){ box.style.borderColor=p?p.c:""; box.dataset.j="";
      box.innerHTML=`<div class="empty">${!G.k?"no earlier keyframe yet":G.pickMode==="top"?topWhy():"waiting for a keyframe ≥ "+G.gap+" s older"}</div>`; continue; }
    if(box.dataset.j!==String(p.j)||!box.querySelector(".cap")){ box.innerHTML=`<div class="cap"></div><div class="gstars"></div><div class="vals"></div>`; box.dataset.j=p.j; }
    const st=stars(k,p.j), sb=box.querySelector(".gstars"); sb.innerHTML=st.html; sb.classList.toggle("all",st.n===3);
    setImg(box,p.j); box.style.borderColor=p.c;
    box.querySelector(".cap").innerHTML=`<span class="chip" style="background:${p.c}"></span><b>j = ${p.j}</b> · t = ${fx(G.D.t[p.j],1)} s <span class="k">(${rel(k,p.j)})</span> · ${fx(dist(k,p.j),1)} m${p.j>k?' <span class="k">· ahead of k</span>':""}`;
    box.querySelector(".vals").innerHTML=CH.map(([c])=>valChip(c,k,p.j,p.c)).join("");
  }
}

/* ------------------------------------------------------------------ trajectory */
function setupCanvas(cv){
  const r=cv.getBoundingClientRect(), d=window.devicePixelRatio||1;
  if(cv.width!==Math.round(r.width*d)||cv.height!==Math.round(r.height*d)){cv.width=Math.round(r.width*d);cv.height=Math.round(r.height*d);}
  const c=cv.getContext("2d"); c.setTransform(d,0,0,d,0,0); return [c,r.width,r.height];
}
function viridis(v){
  const S=[[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]]; v=Math.min(1,Math.max(0,v))*4; const i=Math.min(3,Math.floor(v)), f=v-i;
  const a=S[i], b=S[i+1]; return `rgb(${Math.round(a[0]+f*(b[0]-a[0]))},${Math.round(a[1]+f*(b[1]-a[1]))},${Math.round(a[2]+f*(b[2]-a[2]))})`;
}
/* the other path, dashed: the ground truth aligned onto the estimate (rigid fit), or with "ground truth"
   selected the pose-graph estimate aligned onto the ground truth */
const ocache={};
function otherPath(){
  if(!G.D||!G.D.gt||!G.D.gt_align||!$("gOther").checked) return null;
  const isGT=SRC[G.src].pose==="gt", pose=isGT?"pg":SRC[G.src].pose, key=G.seq+"|"+pose+"|"+isGT+"|"+G.align;
  if(ocache[key]) return ocache[key];
  const AL=G.D.gt_align[G.align]||G.D.gt_align, A=AL[pose], R=A.R, t=A.t, s=isGT?G.D[pose]:G.D.gt, n=s.x.length, o={x:[],y:[],z:[],gt:!isGT,rmse:A.rmse,n:A.n,look:null};
  const L=G.D.look&&G.D.look[isGT?pose:"gt"];
  if(L){ o.look={x:[],y:[],z:[]}; for(let i=0;i<n;i++){ const a=L.x[i], b=L.y[i], c=L.z[i];
    if(!isGT){ o.look.x.push(R[0]*a+R[1]*b+R[2]*c); o.look.y.push(R[3]*a+R[4]*b+R[5]*c); o.look.z.push(R[6]*a+R[7]*b+R[8]*c); }
    else { o.look.x.push(R[0]*a+R[3]*b+R[6]*c); o.look.y.push(R[1]*a+R[4]*b+R[7]*c); o.look.z.push(R[2]*a+R[5]*b+R[8]*c); } } }
  for(let i=0;i<n;i++){ const a=s.x[i], b=s.y[i], c=s.z[i];
    if(!isGT){ o.x.push(R[0]*a+R[1]*b+R[2]*c+t[0]); o.y.push(R[3]*a+R[4]*b+R[5]*c+t[1]); o.z.push(R[6]*a+R[7]*b+R[8]*c+t[2]); }
    else { const u=a-t[0], v=b-t[1], w=c-t[2]; o.x.push(R[0]*u+R[3]*v+R[6]*w); o.y.push(R[1]*u+R[4]*v+R[7]*w); o.z.push(R[2]*u+R[5]*v+R[8]*w); } }
  return ocache[key]=o;
}
function computeBounds(){
  const p0=posePath(), op=otherPath(), ax=G.plane[0], ay=G.plane[1];
  const p=op?{x:p0.x.concat(op.x),y:p0.y.concat(op.y),z:p0.z.concat(op.z)}:p0, n=p.x.length; let th=0;
  if($("gRot").checked&&n>3){ let mx=0,my=0; for(let i=0;i<n;i++){mx+=p[ax][i];my+=p[ay][i];} mx/=n;my/=n;
    let sxx=0,syy=0,sxy=0; for(let i=0;i<n;i++){const a=p[ax][i]-mx,b=p[ay][i]-my;sxx+=a*a;syy+=b*b;sxy+=a*b;} th=0.5*Math.atan2(2*sxy,sxx-syy); }
  G.theta=th; const ct=Math.cos(th), st=Math.sin(th); let u0=1e18,u1=-1e18,v0=1e18,v1=-1e18;
  for(let i=0;i<n;i++){const u=p[ax][i]*ct+p[ay][i]*st, v=-p[ax][i]*st+p[ay][i]*ct; u0=Math.min(u0,u);u1=Math.max(u1,u);v0=Math.min(v0,v);v1=Math.max(v1,v);}
  const mu=(u1-u0)*0.05+0.5, mv=(v1-v0)*0.05+0.5; G.bounds=[u0-mu,u1+mu,v0-mv,v1+mv]; G.view={z:1,px:0,py:0};
}
/* an arrow from screen point q along the viewing direction d (world) projected into the drawing plane */
function arrow(c,q,L,i,ax,ay,ct,st,len,col,w,dash){
  if(!L) return; const dx=L[ax][i], dy=L[ay][i]; const u=dx*ct+dy*st, v=-dx*st+dy*ct, m=Math.hypot(u,v);
  if(m<1e-3) return;                                      // looking straight along the view axis
  const sx=u/m, sy=-v/m, l=len*Math.max(0.35,Math.min(1,m)), ex=q[0]+sx*l, ey=q[1]+sy*l, h=Math.max(5,len*0.32);
  c.strokeStyle=col; c.fillStyle=col; c.lineWidth=w; c.setLineDash(dash||[]);
  c.beginPath(); c.moveTo(q[0],q[1]); c.lineTo(ex-sx*h*0.6,ey-sy*h*0.6); c.stroke(); c.setLineDash([]);
  c.beginPath(); c.moveTo(ex,ey); c.lineTo(ex-sx*h-sy*h*0.5,ey-sy*h+sx*h*0.5); c.lineTo(ex-sx*h+sy*h*0.5,ey-sy*h-sx*h*0.5); c.closePath(); c.fill();
}
function niceLen(m){const e=Math.pow(10,Math.floor(Math.log10(m))), f=m/e; return (f>=5?5:f>=2?2:1)*e;}
function drawTraj(){
  const [c,W,H]=setupCanvas($("gCvTraj")); c.clearRect(0,0,W,H); if(!G.D) return; if(!G.bounds) computeBounds();
  const p=posePath(), ax=G.plane[0], ay=G.plane[1], k=G.k, n=p.x.length, pad=14, ct=Math.cos(G.theta), st=Math.sin(G.theta), B=G.bounds;
  const s=Math.min((W-2*pad)/(B[1]-B[0]),(H-2*pad)/(B[3]-B[2])), V=G.view, mu=(B[0]+B[1])/2, mv=(B[2]+B[3])/2;
  const SC=i=>{const x=p[ax][i], y=p[ay][i], u=x*ct+y*st, v=-x*st+y*ct; return [W/2+V.px+V.z*s*(u-mu), H/2+V.py-V.z*s*(v-mv)];};
  G.sc=SC;
  const isGT=SRC[G.src].pose==="gt", pc=isGT?"#c9d1d9":"#58a6ff";
  const line=(a,b,col,w)=>{c.beginPath(); for(let i=a;i<=b;i++){const q=SC(i); i===a?c.moveTo(q[0],q[1]):c.lineTo(q[0],q[1]);} c.strokeStyle=col; c.lineWidth=w; c.stroke();};
  line(0,n-1,"#3a4553",1.4);
  line(0,k,pc,2.4);
  for(let j=0;j<k;j++){ const q=SC(j), o=old(k,j);
    let col="rgba(140,151,166,.55)"; if(G.colorBy!=="none"&&o){ const v=val(G.colorBy,k,j); col=isFinite(v)?viridis(v):"rgba(140,151,166,.35)"; }
    c.fillStyle=col; c.beginPath(); c.arc(q[0],q[1],o&&G.colorBy!=="none"?3.2:2,0,7); c.fill(); }
  // the other path (dashed) with the same picks, dashed in the same colours
  const op=otherPath(), arrowsOn=$("gArrows").checked, LK=G.D.look&&G.D.look[SRC[G.src].pose];
  if(arrowsOn&&LK){ let last=null;                                        // on the keyframe markers, thinned to >= 16 px apart
    for(let j=0;j<k;j++){ const q=SC(j); if(last&&Math.hypot(q[0]-last[0],q[1]-last[1])<16) continue; last=q;
      let col="rgba(200,205,212,.5)"; if(G.colorBy!=="none"&&old(k,j)){ const v=val(G.colorBy,k,j); if(isFinite(v)) col=viridis(v); }
      arrow(c,q,LK,j,ax,ay,ct,st,12,col,1.2); } }
  if(op){ const SO=i=>{const x=op[ax][i], y=op[ay][i], u=x*ct+y*st, v=-x*st+y*ct; return [W/2+V.px+V.z*s*(u-mu), H/2+V.py-V.z*s*(v-mv)];};
    const oc=op.gt?"rgba(201,209,217,":"rgba(88,166,255,";
    const dl=(a,b,col,w)=>{c.beginPath(); for(let i=a;i<=b;i++){const q=SO(i); i===a?c.moveTo(q[0],q[1]):c.lineTo(q[0],q[1]);} c.strokeStyle=col; c.lineWidth=w; c.setLineDash([5,4]); c.stroke(); c.setLineDash([]);};
    dl(0,n-1,oc+".25)",1.2); dl(0,k,oc+".85)",1.8);
    const qo=SO(k);
    live().forEach(pk=>{ const q=SO(pk.j); c.strokeStyle=pk.c; c.lineWidth=2; c.globalAlpha=0.9; c.setLineDash([5,4]);
      c.beginPath(); c.moveTo(qo[0],qo[1]); c.lineTo(q[0],q[1]); c.stroke(); c.setLineDash([]);
      c.lineWidth=2; c.beginPath(); c.arc(q[0],q[1],4.5,0,7); c.stroke(); c.globalAlpha=1;
      if(arrowsOn) arrow(c,q,op.look,pk.j,ax,ay,ct,st,18,pk.c,1.6,[3,2]); });
    c.strokeStyle=op.gt?"#c9d1d9":"#58a6ff"; c.lineWidth=2; c.beginPath(); c.arc(qo[0],qo[1],6,0,7); c.stroke();
    if(arrowsOn) arrow(c,qo,op.look,k,ax,ay,ct,st,24,op.gt?"#c9d1d9":"#58a6ff",1.8,[3,2]); }
  const qk=SC(k);
  live().forEach(pk=>{ const q=SC(pk.j), fut=pk.j>k;
    c.strokeStyle="rgba(0,0,0,.6)"; c.lineWidth=5; c.beginPath(); c.moveTo(qk[0],qk[1]); c.lineTo(q[0],q[1]); c.stroke();
    c.strokeStyle=pk.c; c.lineWidth=2.6; c.setLineDash(fut?[6,4]:[]); c.beginPath(); c.moveTo(qk[0],qk[1]); c.lineTo(q[0],q[1]); c.stroke(); c.setLineDash([]);
    c.fillStyle=pk.c; c.strokeStyle="#0f1318"; c.lineWidth=1.5; c.beginPath(); c.arc(q[0],q[1],6,0,7); c.fill(); c.stroke();
    if(arrowsOn&&LK) arrow(c,q,LK,pk.j,ax,ay,ct,st,26,pk.c,2.4);
    c.font="bold 11px system-ui"; c.fillStyle=pk.c; c.fillText("j="+pk.j,q[0]+8,q[1]-7); });
  if(arrowsOn&&LK){ arrow(c,qk,LK,k,ax,ay,ct,st,34,"rgba(0,0,0,.7)",5); arrow(c,qk,LK,k,ax,ay,ct,st,32,"#ffffff",2.6); }
  c.fillStyle=pc; c.strokeStyle="#fff"; c.lineWidth=2; c.beginPath(); c.arc(qk[0],qk[1],7,0,7); c.fill(); c.stroke();
  c.font="bold 12px system-ui"; c.fillStyle="#fff"; c.fillText("k="+k,qk[0]+10,qk[1]+14);
  const q0=SC(0); if(arrowsOn&&LK) arrow(c,q0,LK,0,ax,ay,ct,st,20,"#3fb950",2); c.fillStyle="#3fb950"; c.fillRect(q0[0]-4,q0[1]-4,8,8); c.fillStyle="#8c97a6"; c.font="11px system-ui"; c.fillText("start",q0[0]+7,q0[1]-6);
  const mpp=1/(V.z*s), L=niceLen(mpp*90), px=L/mpp;
  c.strokeStyle="#c9d1d9"; c.lineWidth=2; c.beginPath(); c.moveTo(12,H-14); c.lineTo(12+px,H-14); c.moveTo(12,H-18); c.lineTo(12,H-10); c.moveTo(12+px,H-18); c.lineTo(12+px,H-10); c.stroke();
  c.fillStyle="#c9d1d9"; c.fillText((L>=1?L.toFixed(0):L.toFixed(2))+" m",16+px,H-10);
  c.fillStyle="#6b7684"; c.fillText(`${isGT?"ground-truth":SRC[G.src].pose==="vio"?"VIO":"pose-graph"} poses · plane ${G.plane}${$("gRot").checked?" · rotated "+(G.theta*57.296).toFixed(0)+"°":""} · wheel = zoom, drag = pan, double-click = reset, click = pick`,12,14);
  $("gTrajLeg").innerHTML=`<span><i style="border-color:${pc}"></i>${isGT?"ground-truth":"estimated"} path up to keyframe k</span><span><i style="border-color:#3a4553"></i>rest of the run</span>
    <span>dots: earlier keyframes ≥ ${G.gap} s older, coloured by ${G.colorBy==="V"?"V_min":G.colorBy} (dark purple = 0 … yellow = 1)</span><span>coloured lines: the 4 picks</span>${$("gArrows").checked?"<span>arrows on the markers: viewing direction (white = k, coloured = picks, green = start, small = earlier keyframes in their dot colour; dashed = on the ground truth; shorter = looking up/down out of the plane)</span>":""}${op?`<span><i style="border-color:${op.gt?"#c9d1d9":"#58a6ff"};border-top-style:dashed"></i>${op.gt?"ground truth":"pose-graph estimate"} (dashed; Umeyama alignment on the first ${op.n} keyframes${op.n>=G.D.t.length?" = all":""}, RMSE ${fx(op.rmse,2)} m there), same picks dashed</span>`:""}`;
}

/* ------------------------------------------------------------------ 1-D plots (time axis, whole sequence) */
const PL={L:34,R:12,T:20,B:18};
function niceStep(x){ const e=Math.pow(10,Math.floor(Math.log10(Math.max(x,1e-9)))), f=x/e; return (f>=5?5:f>=2?2:1)*e; }
function rowStats(ch){ const k=G.k; let fin=0,pos=0; for(let j=0;j<k;j++){ if(!old(k,j)) continue; const v=val(ch,k,j); if(isFinite(v)){fin++; if(v>0) pos++;} } return {fin,pos}; }
function jAtTime(t,k){ const T=G.D.t; let lo=0,hi=k-1,r=0; if(k<=0) return null; while(lo<=hi){const m=(lo+hi)>>1; if(T[m]<=t){r=m;lo=m+1}else hi=m-1;}
  if(r+1<k&&Math.abs(T[r+1]-t)<Math.abs(T[r]-t)) r++; return r; }
function drawPlot(ch,label){
  const cv=$("gCv"+ch), [c,W,H]=setupCanvas(cv); c.clearRect(0,0,W,H); if(!G.D) return;
  const T=G.D.t, k=G.k, Tend=Math.max(T[T.length-1],1), PW=W-PL.L-PL.R, PH=H-PL.T-PL.B;
  const X=t=>PL.L+t/Tend*PW, Y=v=>H-PL.B-v*PH, bw=Math.max(1.2,PW*(Tend/T.length)/Tend*0.85), tk=T[k];
  c.fillStyle="#141a21"; c.fillRect(PL.L,PL.T,PW,PH);
  // after k: not computed yet (future)  |  last gap seconds before k: too recent to be a loop
  c.fillStyle="rgba(0,0,0,.35)"; c.fillRect(X(tk),PL.T,PW-(X(tk)-PL.L),PH);
  const g0=Math.max(0,tk-G.gap); c.fillStyle="rgba(140,151,166,.13)"; c.fillRect(X(g0),PL.T,X(tk)-X(g0),PH);
  c.font="10px system-ui"; c.fillStyle="#6b7684";
  if(X(tk)-X(g0)>46) c.fillText(`< ${G.gap} s`,X(g0)+3,PL.T+11);
  if(PL.L+PW-X(tk)>60) c.fillText("not reached yet",X(tk)+5,PL.T+11);
  c.lineWidth=1;
  [0,0.25,0.5,0.75,1].forEach(v=>{c.strokeStyle=v%0.5?"#1c232c":"#2a323d"; c.beginPath();c.moveTo(PL.L,Y(v));c.lineTo(W-PL.R,Y(v));c.stroke();
    if(v%0.5===0){c.fillStyle="#8c97a6"; c.fillText(v.toFixed(1),PL.L-22,Y(v)+3);} });
  const st=niceStep(Tend/8); c.fillStyle="#8c97a6";
  for(let t=0;t<=Tend+1e-6;t+=st){ const x=X(t); c.strokeStyle="#2a323d"; c.beginPath(); c.moveTo(x,H-PL.B); c.lineTo(x,H-PL.B+4); c.stroke(); const s=t+" s"; c.fillText(s,Math.min(x-c.measureText(s).width/2,W-PL.R-c.measureText(s).width),H-PL.B+14); }
  const col=CHCOL[ch];
  for(let j=0;j<k;j++){ const v=val(ch,k,j), x=X(T[j]);
    if(!isFinite(v)){ c.fillStyle="rgba(140,151,166,.6)"; c.fillRect(x-bw/2,Y(0)+1,bw,2); continue; }
    if(v<=0) continue; c.fillStyle=col; c.globalAlpha=old(k,j)?0.9:0.45; c.fillRect(x-bw/2,Y(v),bw,Y(0)-Y(v)); c.globalAlpha=1; }
  if(showGT()){                                                            // ground truth as a white line
    c.strokeStyle="rgba(255,255,255,.85)"; c.lineWidth=1.2; c.beginPath(); let pen=false;
    for(let j=0;j<k;j++){ const g=gval(ch,k,j); if(!isFinite(g)){pen=false;continue;} const x=X(T[j]), y=Y(g); pen?c.lineTo(x,y):c.moveTo(x,y); pen=true; }
    c.stroke(); }
  c.strokeStyle="rgba(255,255,255,.7)"; c.setLineDash([3,3]); c.beginPath(); c.moveTo(X(tk),PL.T); c.lineTo(X(tk),H-PL.B); c.stroke(); c.setLineDash([]);
  c.fillStyle="#e6e9ee"; c.font="bold 10px system-ui"; c.fillText("k",X(tk)+3,H-PL.B-3);
  const rs=rowStats(ch);
  if(k>0&&rs.pos===0){ c.fillStyle="#8c97a6"; c.font="12px system-ui";
    const m=rs.fin===0?(rs.fin===0&&!k?"":`no value ≥ ${G.gap} s before k`+(SRC[G.src].depth==="metric"&&ch!=="F"?" (no metric depth here – try raw or ground truth)":""))
                     :`all ${rs.fin} keyframes ≥ ${G.gap} s older have ${ch==="V"?"V_min":ch} = 0`;
    c.fillText(m,PL.L+PW/2-c.measureText(m).width/2,PL.T+PH/2); }
  if(G.hover!==null&&G.hover!==k){ const x=X(T[G.hover]), v=val(ch,k,G.hover);
    c.strokeStyle="#e6e9ee"; c.lineWidth=1; c.beginPath(); c.moveTo(x,PL.T); c.lineTo(x,H-PL.B); c.stroke();
    c.font="bold 11px system-ui"; const s=isFinite(v)?v.toFixed(2):"n/a", tw=c.measureText(s).width, tx=Math.min(x+6,W-PL.R-tw-4), ty=isFinite(v)?Math.max(Y(v)-4,PL.T+12):Y(0)-4;
    c.fillStyle="rgba(15,19,24,.92)"; c.fillRect(tx-3,ty-11,tw+6,14); c.fillStyle="#e6e9ee"; c.fillText(s,tx,ty); }
  live().forEach(pk=>{ const v=val(ch,k,pk.j), x=X(T[pk.j]), fut=pk.j>k;
    c.strokeStyle=pk.c; c.lineWidth=1.5; c.globalAlpha=0.6; c.setLineDash(fut?[4,4]:[]); c.beginPath(); c.moveTo(x,PL.T); c.lineTo(x,H-PL.B); c.stroke(); c.setLineDash([]); c.globalAlpha=1;
    c.beginPath(); c.arc(x,isFinite(v)?Y(v):Y(0),5,0,7);
    if(fut){ c.strokeStyle=pk.c; c.lineWidth=2; c.stroke(); } else { c.fillStyle=pk.c; c.strokeStyle="#0f1318"; c.lineWidth=1.5; c.fill(); c.stroke(); } });
  c.font="600 11px system-ui"; c.fillStyle=col; c.fillText(label,PL.L,PL.T-6); let tx=PL.L+c.measureText(label).width+14;
  c.font="11px system-ui";
  live().forEach(pk=>{ const v=val(ch,k,pk.j), g=showGT()?gval(ch,k,pk.j):null, s=`j${pk.j}  ${fx(v,2)}${g!==null?" (GT "+fx(g,2)+")":""}`;
    c.fillStyle=pk.c; c.beginPath(); c.arc(tx+4,PL.T-10,4,0,7); c.fill(); c.fillText(s,tx+11,PL.T-6); tx+=c.measureText(s).width+24; });
  cv._J=x=>jAtTime((x-PL.L)/PW*Tend,T.length);
}
function drawPlots(){ CH.forEach(([ch,l])=>drawPlot(ch,l)); readout(); }
function readout(){
  const el=$("gRead"); if(!el) return; const j=G.hover, k=G.k;
  if(j===null||j===k){ el.innerHTML=`<span class="k">x axis: time of keyframe j (whole sequence, fixed) · in "fixed" you can also pick keyframes after k (dashed, hollow dot: value from that keyframe's own row) · bar height = value · faded bars and grey zone: less than ${G.gap} s before k (too recent to count as a loop) · grey tick under the axis: no value · dark area: after k${showGT()?" · white line: ground truth":""} · hover to read, click to make it a pick</span>`; return; }
  el.innerHTML=`<b>j = ${j}</b> <span class="k">t = ${fx(G.D.t[j],1)} s · ${rel(k,j)} · ${fx(dist(k,j),1)} m away${j>k?" · computed later, at keyframe "+j:""}</span> &nbsp; `+
    CH.map(([c])=>{const v=val(c,k,j), g=showGT()?gval(c,k,j):null; return `<span style="color:${CHCOL[c]}">${c==="V"?"V<sub>min</sub>":c} <b>${isFinite(v)?v.toFixed(3):"n/a"}</b></span>${g!==null?` <span class="k">(GT ${fx(g,3)})</span>`:""}`;}).join(" &nbsp; ")+
    ` &nbsp; <span class="k">click to make it a pick</span>`;
}

/* ------------------------------------------------------------------ render, play */
function render(){
  if(!G.D||!G.M||$("pgm").style.display==="none") return;
  updatePicks(); fillRes(); showImages(); drawTraj(); drawPlots();
  $("gSlider").value=G.k; $("gTl").textContent=`keyframe ${G.k} / ${G.D.t.length-1} · ${fx(G.D.t[G.k],1)} / ${fx(G.D.t[G.D.t.length-1],1)} s`;
}
function setK(k){ k=Math.max(0,Math.min(G.D.t.length-1,k)); const ch=k!==G.k; G.k=k; if(ch) render(); }
function kAt(t){ const T=G.D.t; let lo=0,hi=T.length-1,r=0; while(lo<=hi){const m=(lo+hi)>>1; if(T[m]<=t){r=m;lo=m+1}else hi=m-1;} return r; }
function setPlaying(on){
  if(on){ if(G.k>=G.D.t.length-1){G.k=0; G.t=0;} else G.t=G.D.t[G.k]; G.playing=true; G.last=performance.now(); $("gPlay").innerHTML="&#10073;&#10073; Pause"; }
  else { G.playing=false; $("gPlay").innerHTML="&#9654; Play"; }
}
function tick(ts){
  if(G.playing&&G.D){ const dt=(ts-G.last)/1000; G.last=ts; G.t+=dt*G.speed; const T=G.D.t;
    if(G.t>=T[T.length-1]){ G.t=T[T.length-1]; setK(T.length-1); setPlaying(false); } else setK(kAt(G.t)); }
  requestAnimationFrame(tick);
}
function srcOptions(){
  const sel=$("gSrc"); Array.from(sel.options).forEach(o=>{ o.disabled=(o.value==="gt"&&!hasGT()); });
  if(G.src==="gt"&&!hasGT()) G.src="pg_metric"; sel.value=G.src;
  $("gHelp").textContent=SRC[G.src].help+(G.src==="gt"?"":hasGT()?"":" (No ground truth exported for this sequence.)");
  $("gCmpBox").style.display=(hasGT()&&G.src!=="gt")?"block":"none";
  $("gOtherBox").style.display=$("gAlBox").style.display=(G.D&&G.D.gt_align)?"":"none";
  $("gOtherLab").textContent=G.src==="gt"?"estimate (dashed)":"ground truth (dashed)";
}
async function selectSeq(seq){
  G.seq=seq; $("status").textContent="loading "+seq+" …";
  try{ await loadJS(`pglsmono/data/${seq}.js`,()=>window.PGM&&window.PGM[seq]); G.D=window.PGM[seq]; srcOptions(); await selectVer(); }
  catch(e){ $("status").textContent=e.message; return; }
  $("status").textContent=""; setPlaying(false);
  $("gSlider").max=G.D.t.length-1; G.k=G.D.t.length-1; G.bounds=null;
  fillSeq(); G.picks=[]; G.pickK=-1; if(G.pickMode==="fixed") drawTop(); render();       // start on the best pairs of the last keyframe
}
async function selectVer(){
  $("status").textContent="loading …";
  G.M=await loadMat(G.seq,G.src);
  G.MG=(hasGT()&&G.cmp&&G.src!=="gt")?await loadMat(G.seq,"gt"):null;
  $("status").textContent="";
}

/* ------------------------------------------------------------------ tabs + events */
function showTab(on){
  $("pgm").style.display=on?"grid":"none"; $("tab-pgm").classList.toggle("on",on);
  const q=$("q22-btn"), qq=$("q22"); if(q) q.style.display=on?"none":""; if(qq&&on) qq.hidden=true;
  if(on){ $("app").style.display="none"; $("cmp").style.display="none"; $("tab-ex").classList.remove("on"); $("tab-cmp").classList.remove("on");
    if(G.D) render(); else if(IDX.seqs.length) selectSeq($("gSeq").value); }
}
function seg(id,cb){ $(id).querySelectorAll("button").forEach(b=>b.onclick=()=>{ $(id).querySelectorAll("button").forEach(x=>x.classList.toggle("on",x===b)); cb(b.dataset.v); }); }
function hookCanvasPick(cv,toJ,tip){
  let down=null;
  cv.addEventListener("mousedown",e=>{down=[e.clientX,e.clientY];});
  cv.addEventListener("mouseup",e=>{ if(down&&Math.abs(e.clientX-down[0])+Math.abs(e.clientY-down[1])<5){ const j=toJ(e); if(j!==null) replacePick(j);} down=null; });
  if(tip){ cv.addEventListener("mousemove",e=>{ const s=tip(e), t=$("gTip"); if(!s){t.style.display="none";return;} t.innerHTML=s; t.style.display="block"; t.style.left=(e.clientX+12)+"px"; t.style.top=(e.clientY+12)+"px"; });
    cv.addEventListener("mouseleave",()=>{$("gTip").style.display="none";}); }
}
function nearestKf(e){
  if(!G.sc) return null; const r=e.target.getBoundingClientRect(), x=e.clientX-r.left, y=e.clientY-r.top; let b=-1,bd=1e9;
  for(let j=0;j<G.D.t.length;j++){ if(j===G.k) continue; const q=G.sc(j), d=(q[0]-x)**2+(q[1]-y)**2; if(d<bd){bd=d;b=j;} } return bd<400?b:null;
}
function init(){
  if(!build()) return;
  const tabs=document.querySelector(".tabs");
  if(!$("tab-pgm")){ const b=document.createElement("button"); b.className="tab"; b.id="tab-pgm"; b.textContent="PGLS-mono"; tabs.insertBefore(b,tabs.firstChild); }
  $("tab-pgm").onclick=()=>showTab(true);
  $("tab-ex").addEventListener("click",()=>showTab(false)); $("tab-cmp").addEventListener("click",()=>showTab(false));
  $("gSeq").innerHTML=IDX.seqs.map(s=>`<option value="${s}">${s}${(IDX.info[s]&&(IDX.info[s].diverge_k!==undefined?IDX.info[s].diverge_k!==null:IDX.info[s].max_step>10))?" ⚠ VINS diverged":""}</option>`).join("");
  $("gSeq").onchange=e=>selectSeq(e.target.value);
  $("gSrc").onchange=async e=>{ const pose0=SRC[G.src].pose; G.src=e.target.value; srcOptions(); await selectVer(); if(SRC[G.src].pose!==pose0) G.bounds=null; G.pickK=-1; render(); };
  $("gCmp").onchange=async e=>{ G.cmp=e.target.checked; await selectVer(); render(); };
  seg("gMode",v=>{G.mode=v; showImages();});
  seg("gPickMode",v=>setPickMode(v));
  seg("gKlCh",v=>{G.klCh=v; fillKl();});
  $("gPlay").onclick=()=>setPlaying(!G.playing);
  $("gEnd").onclick=()=>{setPlaying(false); setK(G.D.t.length-1);};
  $("gSlider").oninput=e=>{setPlaying(false); setK(+e.target.value);};
  $("gSpeed").onchange=e=>{G.speed=+e.target.value;};
  $("gPlane").onchange=e=>{G.plane=e.target.value; G.bounds=null; render();};
  $("gColor").onchange=e=>{G.colorBy=e.target.value; drawTraj();};
  $("gRot").onchange=()=>{G.bounds=null; drawTraj();};
  $("gOther").onchange=()=>{G.bounds=null; drawTraj();};
  $("gArrows").onchange=()=>drawTraj();
  seg("gAlign",v=>{G.align=v; const vw=G.view; G.bounds=null; computeBounds(); if(vw) G.view=vw; drawTraj();});
  $("gReset").onclick=()=>{G.bounds=null; drawTraj();};
  $("gGap").onchange=e=>{G.gap=Math.max(0,+e.target.value||0); G.pickK=-1; render();};
  $("gRoll").onclick=reroll;
  const cv=$("gCvTraj"); let drag=null;
  cv.addEventListener("wheel",e=>{e.preventDefault(); if(!G.view) return; const r=cv.getBoundingClientRect(), mx=e.clientX-r.left-r.width/2, my=e.clientY-r.top-r.height/2;
    const f=Math.exp(-e.deltaY*0.0015), z0=G.view.z, z1=Math.min(200,Math.max(0.5,z0*f)), q=z1/z0; G.view.px=mx-(mx-G.view.px)*q; G.view.py=my-(my-G.view.py)*q; G.view.z=z1; drawTraj();},{passive:false});
  cv.addEventListener("mousedown",e=>{ if(G.view) drag=[e.clientX,e.clientY,G.view.px,G.view.py]; cv.style.cursor="grabbing";});
  window.addEventListener("mouseup",()=>{drag=null; cv.style.cursor="grab";});
  window.addEventListener("mousemove",e=>{ if(!drag) return; G.view.px=drag[2]+e.clientX-drag[0]; G.view.py=drag[3]+e.clientY-drag[1]; drawTraj();});
  cv.addEventListener("dblclick",()=>{G.view={z:1,px:0,py:0}; drawTraj();}); cv.style.cursor="grab";
  hookCanvasPick(cv,nearestKf,e=>{const j=nearestKf(e); return j===null?null:`j = ${j} · t = ${fx(G.D.t[j],1)} s · P ${fx(val("P",G.k,j))} · F ${fx(val("F",G.k,j))} · V<sub>min</sub> ${fx(val("V",G.k,j))}`;});
  CH.forEach(([ch])=>{ const p=$("gCv"+ch), J=e=>{const r=p.getBoundingClientRect(); const j=p._J?p._J(e.clientX-r.left):null; return (j!==null&&j>=0&&j!==G.k)?j:null;};
    hookCanvasPick(p,J,null);
    p.addEventListener("mousemove",e=>{ const j=J(e); if(j!==G.hover){ G.hover=j; drawPlots(); } });
    p.addEventListener("mouseleave",()=>{ G.hover=null; drawPlots(); }); });
  window.addEventListener("resize",()=>{ if($("pgm").style.display!=="none") render(); });
  document.addEventListener("keydown",e=>{ if($("pgm").style.display==="none"||!G.D||/INPUT|SELECT/.test(document.activeElement.tagName)) return;
    if(e.key==="ArrowRight"){setPlaying(false);setK(G.k+1);} else if(e.key==="ArrowLeft"){setPlaying(false);setK(G.k-1);} else if(e.key===" "){e.preventDefault();setPlaying(!G.playing);} });
  if(!IDX.seqs.length){ $("pgm").innerHTML=`<div class="card">no PGLS-mono data – run build_pglsmono_web.py</div>`; return; }
  requestAnimationFrame(tick);
  showTab(true);
}
if(document.readyState==="loading") document.addEventListener("DOMContentLoaded",init); else init();
})();
