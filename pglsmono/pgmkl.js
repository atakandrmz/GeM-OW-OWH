/* "PGLS-mono agreement" tab: KL divergence between the P, F and V_min matrices of the four estimated versions and
   the ground truth, summarised over all sequences and per sequence. Reads PGM_INDEX.info[seq].kl from
   pglsmono/data/index.js (build_pglsmono_web.py). */
(function(){
"use strict";
const $=id=>document.getElementById(id);
const IDX=window.PGM_INDEX||{seqs:[],info:{}};
const EST=["pg_metric","vio_metric","pg_raw","vio_raw"];
const SHORT={pg_metric:"PG + metric",vio_metric:"VIO + metric",pg_raw:"PG + raw",vio_raw:"VIO + raw",gt:"GT"};
const LONG={pg_metric:"VINS pose graph + Depth Anything, metric",vio_metric:"VINS VIO + Depth Anything, metric",
            pg_raw:"VINS pose graph + Depth Anything, raw",vio_raw:"VINS VIO + Depth Anything, raw",gt:"Ground truth (GT poses + LiDAR)"};
const VCOL={pg_metric:"#58a6ff",vio_metric:"#79c0ff",pg_raw:"#f0b429",vio_raw:"#f78166",gt:"#e6e9ee"};
const CH=[["P","P (pose)"],["F","F (frustum)"],["V","V<sub>min</sub> (occlusion)"]];
const fx=(v,d=3)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d);
const K={excl:true,seq:null,ch:"P"};
const diverged=s=>{const i=IDX.info[s]||{}; return i.diverge_k!==undefined?i.diverge_k!==null:(i.max_step>10);};
const klOf=s=>(IDX.info[s]||{}).kl;
const hasGT=s=>{const k=klOf(s); return !!(k&&k.names.indexOf("gt")>=0);};
function score(s,ch,v){ const k=klOf(s); if(!k||!k[ch]) return null;
  if(hasGT(s)) return k[ch].gt[v];
  const o=EST.filter(x=>x!==v&&k[ch][x]).map(x=>k[ch][x][v]).filter(x=>x!==null); return o.length?o.reduce((a,b)=>a+b,0)/o.length:null; }
const seqs=()=>IDX.seqs.filter(s=>klOf(s)&&!(K.excl&&diverged(s)));

function build(){
  const d=document.createElement("div"); d.id="pgk"; d.style.display="none";
  const app=$("pgm")||$("app"); app.parentNode.insertBefore(d,app.nextSibling);
  d.innerHTML=`
  <div class="card" style="margin-bottom:10px">
    <h2>How the four PGLS-mono versions agree with the ground truth &mdash; KL divergence</h2>
    <div class="kh">For every sequence, each version's P, F and V<sub>min</sub> matrix (all keyframe pairs j &lt; k) is turned into a 2-D distribution
      (value + 0.001, divided by the sum) and compared with the ground-truth matrix (GT poses + LiDAR depth, v7 run, same keyframe times):
      <b>KL(GT ‖ version)</b>. KL is always ≥ 0 and 0 means the two matrices have the same pattern, so <b>lower is better</b>.
      Because each matrix is normalised, KL measures <i>where</i> the high and low values are (which pairs score high), not their absolute level.
      It is not symmetric; KL(GT ‖ version) grows most where the ground truth is high and the version is low (missed overlaps).
      Pairs where a version has no value (no metric depth) are left out; "coverage" says how many were kept.</div>
    <div class="row" style="margin-top:6px"><label class="k"><input type="checkbox" id="kExcl" checked> leave out sequences where VINS diverged</label>
      <span class="k" id="kNote"></span></div>
  </div>
  <div class="card" style="margin-bottom:10px"><h2>Summary over sequences</h2><div style="overflow:auto"><table class="ktab" id="kSum"></table></div>
    <div class="kh" id="kSumNote"></div></div>
  <div class="card" style="margin-bottom:10px"><h2>Per sequence &mdash; KL(GT ‖ version), best of each row in green, worst in red</h2>
    <div class="kgrid">${CH.map(([c,l])=>`<div><h3 class="kh3" style="color:${({P:"#58a6ff",F:"#3fb950",V:"#d2a8ff"})[c]}">${l}</h3><div style="overflow:auto"><table class="ktab" id="kSeq${c}"></table></div></div>`).join("")}</div></div>
  <div class="card"><h2>Every pair of versions for one sequence &mdash; KL(row ‖ column)</h2>
    <div class="row" style="margin-bottom:6px"><label class="k">sequence</label><select id="kSeqSel"></select>
      <span class="gseg" id="kCh">${CH.map(([c],i)=>`<button data-v="${c}" class="${i?"":"on"}">${c==="V"?"V_min":c}</button>`).join("")}</span></div>
    <div style="overflow:auto"><table class="ktab" id="kPair"></table></div>
    <div class="kh">Diagonal = 0. Small values off the diagonal between "PG" and "VIO" mean the loop corrections barely changed anything.</div></div>`;
}

function fill(){
  const S=seqs(), all=IDX.seqs.filter(klOf), out=all.filter(s=>!S.includes(s));
  $("kNote").textContent=`${S.length} of ${all.length} sequences used`+(out.length?` (left out: ${out.map(s=>s.replace(/^spot_(outdoor_day_|indoor_)?/,"")).join(", ")})`:"");
  // summary: mean KL per version and channel, mean rank, number of first places
  const rows=EST.map(v=>{ const r={v}; CH.forEach(([c])=>{ const xs=S.map(s=>score(s,c,v)).filter(x=>x!==null);
      r[c]=xs.length?xs.reduce((a,b)=>a+b,0)/xs.length:null;
      const rk=S.map(s=>{ const sc=EST.map(w=>[w,score(s,c,w)]).filter(x=>x[1]!==null).sort((a,b)=>a[1]-b[1]); return sc.findIndex(x=>x[0]===v)+1; }).filter(x=>x>0);
      r[c+"rank"]=rk.length?rk.reduce((a,b)=>a+b,0)/rk.length:null; r[c+"win"]=rk.filter(x=>x===1).length; });
    const rs=CH.map(([c])=>r[c+"rank"]).filter(x=>x!==null); r.overall=rs.length?rs.reduce((a,b)=>a+b,0)/rs.length:null; return r; });
  rows.sort((a,b)=>(a.overall===null)-(b.overall===null)||a.overall-b.overall);
  const mx={}; CH.forEach(([c])=>{ mx[c]=Math.max(1e-9,...rows.map(r=>r[c]||0)); });
  const best={}; CH.forEach(([c])=>{ best[c]=rows.filter(r=>r[c]!==null).reduce((a,b)=>a===null||b[c]<a[c]?b:a,null); });
  $("kSum").innerHTML=`<tr><th>rank</th><th>version</th>${CH.map(([c,l])=>`<th>${l}<br><span class="k">mean KL · mean rank · 1st</span></th>`).join("")}<th>overall<br><span class="k">mean rank</span></th></tr>`+
    rows.map((r,i)=>`<tr><td>${i+1}.</td><td><span class="kdot" style="background:${VCOL[r.v]}"></span><b>${SHORT[r.v]}</b><br><span class="k">${LONG[r.v]}</span></td>`+
      CH.map(([c])=>`<td class="${best[c]===r?"kbest":""}"><div class="kbar"><i style="width:${r[c]===null?0:Math.round(100*r[c]/mx[c])}%;background:${VCOL[r.v]}"></i></div>
        <b>${fx(r[c])}</b> <span class="k">· ${fx(r[c+"rank"],2)} · ${r[c+"win"]}/${S.length}</span></td>`).join("")+
      `<td><b>${fx(r.overall,2)}</b></td></tr>`).join("");
  $("kSumNote").innerHTML=`mean KL = average of KL(GT ‖ version) over the sequences used; mean rank = average place among the four versions (1 = closest to the ground truth); 1st = in how many sequences it was the closest.`+
    (S.some(s=>!hasGT(s))?` Sequences without ground truth are scored by the mean KL to the other three versions.`:"");
  // per sequence
  CH.forEach(([c])=>{
    $("kSeq"+c).innerHTML=`<tr><th>sequence</th>${EST.map(v=>`<th style="color:${VCOL[v]}">${SHORT[v]}</th>`).join("")}<th>coverage<br><span class="k">metric</span></th></tr>`+
      IDX.seqs.filter(klOf).map(s=>{ const sc=EST.map(v=>score(s,c,v)), ok=sc.filter(x=>x!==null), lo=Math.min(...ok), hi=Math.max(...ok), k=klOf(s), dv=diverged(s);
        return `<tr class="${dv?"kdiv":""}"><td title="${s}">${s.replace(/^spot_/,"")}${dv?" ⚠":""}${hasGT(s)?"":" <span class='k'>(no GT)</span>"}</td>`+
          sc.map(x=>`<td class="${x===null?"":x===lo?"kbest":x===hi?"kworst":""}">${fx(x)}</td>`).join("")+
          `<td class="k">${k.cover?fx(100*k.cover.pg_metric,0)+" %":"–"}</td></tr>`; }).join("");
  });
  fillPair();
}
function fillPair(){
  const s=$("kSeqSel").value, k=klOf(s); if(!k){ $("kPair").innerHTML=""; return; }
  const C=k[K.ch], n=k.names;
  $("kPair").innerHTML=`<tr><th>row ‖ column</th>${n.map(v=>`<th style="color:${VCOL[v]}">${SHORT[v]}</th>`).join("")}</tr>`+
    n.map(a=>`<tr><td style="color:${VCOL[a]}">${SHORT[a]}</td>${n.map(b=>`<td>${a===b?"0":fx(C[a][b])}</td>`).join("")}</tr>`).join("");
}

function show(on){
  const d=$("pgk"); if(!d) return; d.style.display=on?"block":"none"; $("tab-pgk").classList.toggle("on",on);
  if(on){ ["app","cmp","pgm"].forEach(id=>{ if($(id)) $(id).style.display="none"; }); ["tab-ex","tab-cmp","tab-pgm"].forEach(id=>{ if($(id)) $(id).classList.remove("on"); });
    const q=$("q22-btn"); if(q) q.style.display="none"; fill(); }
}
function init(){
  if(!IDX.seqs.length||$("pgk")) return;
  build();
  const tabs=document.querySelector(".tabs"), b=document.createElement("button"); b.className="tab"; b.id="tab-pgk"; b.textContent="PGLS-mono agreement";
  const after=$("tab-pgm"); tabs.insertBefore(b,after?after.nextSibling:tabs.firstChild);
  b.onclick=()=>show(true);
  ["tab-pgm","tab-ex","tab-cmp"].forEach(id=>{ if($(id)) $(id).addEventListener("click",()=>show(false)); });
  $("kExcl").onchange=e=>{K.excl=e.target.checked; fill();};
  $("kSeqSel").innerHTML=IDX.seqs.filter(klOf).map(s=>`<option value="${s}">${s}${diverged(s)?" ⚠":""}</option>`).join("");
  $("kSeqSel").onchange=fillPair;
  $("kCh").querySelectorAll("button").forEach(x=>x.onclick=()=>{ $("kCh").querySelectorAll("button").forEach(y=>y.classList.toggle("on",y===x)); K.ch=x.dataset.v; fillPair(); });
}
if(document.readyState==="loading") document.addEventListener("DOMContentLoaded",init); else init();
})();
