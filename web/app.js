'use strict';
const $ = id => document.getElementById(id);
const colors = ['#ff987c','#9ee0af','#87bcff'];
const panels = [...document.querySelectorAll('.signal')];
let latest = null, displayed = null, paused = false, history = [], windowSeconds = 10;
let poseMode = 'imu';
let lastDeviceNs = null, live = false, latestStatus = {}, lastFrame = 0;
const format = (n,d=2) => Number.isFinite(n) ? n.toFixed(d) : '—';
function status(s) {
  latestStatus=s; live=s.state==='live';
  $('status-dot').className=live?'live':(s.state==='disconnected'?'disconnected':'');
  $('connection-label').textContent=live?(paused?'表示停止中':'LIVE · 取得中'):(s.state==='disconnected'?'未接続':'接続中');
  $('notice').hidden=live&&!paused;
  $('notice').className='notice'+(s.state==='disconnected'?' error':'');
  $('notice').textContent=live&&paused?'表示を一時停止しています。実機のデータ取得とCSV用バッファは継続中です。':s.message;
  $('center').disabled=!live;
  $('reset-translation').disabled=!live;
  $('zupt').disabled=!live;
  $('rate').textContent=live&&s.rate_hz?Number(s.rate_hz).toLocaleString():'—';
  $('rate-fill').style.width=live?Math.min(100,(s.rate_hz||0)/10)+'%':'0';
  $('sample-count').textContent=(s.samples||0).toLocaleString();
  $('gaps').textContent=s.gaps||0;
  $('device-info').textContent=s.serial?`${s.serial} · FW ${s.firmware}`:'AIR · USB HID';
}
const source=new EventSource('/events');
source.onmessage=e=>{
  const data=JSON.parse(e.data); status(data.status);
  if(!data.sample) { if(!live)latest=null; return; }
  latest=data.sample;
  if(document.activeElement!==$('zupt'))$('zupt').checked=!!latest.pose.zupt_enabled;
  if(lastDeviceNs!==null && (latest.device_ns<lastDeviceNs || data.status.samples<2))history=[];
  if(latest.device_ns!==lastDeviceNs && !paused) {
    history.push(latest); if(history.length>1600)history.shift();
  }
  lastDeviceNs=latest.device_ns;
  if(!paused)displayed=latest;
};
source.onerror=()=>status({...latestStatus,state:'disconnected',message:'サーバーに接続できません。起動状態を確認してください。'});
$('pause').onclick=()=>{
  paused=!paused; $('pause').textContent=paused?'▶ 表示を再開':'Ⅱ 表示を止める';
  if(!paused) { history=[]; displayed=latest; }
  status(latestStatus);
};
$('fusion-mode').onchange=e=>{poseMode=e.target.value;};
$('window').onchange=e=>{windowSeconds=Number(e.target.value);};
$('center').onclick=async()=>{
  const button=$('center');button.disabled=true;
  try {
    const response=await fetch('/api/center',{method:'POST',headers:{'X-Air-Viewer':'1'}});
    if(!response.ok)throw new Error('正面リセットに失敗しました');
  }catch(error){$('notice').hidden=false;$('notice').textContent=error.message;}
  finally{button.disabled=!live;}
};
async function translationAction(path,body){
  const response=await fetch(path,{method:'POST',headers:{'X-Air-Viewer':'1','Content-Type':'application/json'},body:JSON.stringify(body||{})});
  if(!response.ok)throw new Error('並進の設定に失敗しました');
}
$('reset-translation').onclick=async()=>{
  try{await translationAction('/api/translation/reset');}
  catch(error){$('notice').hidden=false;$('notice').textContent=error.message;}
};
$('zupt').onchange=async e=>{
  const enabled=e.target.checked;
  try{await translationAction('/api/translation/zupt',{enabled});}
  catch(error){e.target.checked=!enabled;$('notice').hidden=false;$('notice').textContent=error.message;}
};
function values(sample,panel){
  return panel.dataset.translation?sample.pose.variants[poseMode].translation[panel.dataset.signal]:sample[panel.dataset.signal];
}
function surface(canvas){
  const box=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
  const width=Math.round(box.width*dpr),height=Math.round(box.height*dpr);
  if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}
  const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,box.width,box.height);
  return [c,box.width,box.height];
}
function chart(panel){
  const [c,w,h]=surface(panel.querySelector('canvas')); const key=panel.dataset.signal;
  const end=history.length?history[history.length-1].device_ns/1e9:0;
  const start=end-windowSeconds;
  const points=history.filter(s=>s.device_ns/1e9>=start);
  let extent=key==='gyro_dps'?5:key==='accel_m_s2'?12:.2;
  for(const s of points)for(const v of values(s,panel))extent=Math.max(extent,Math.abs(v)*1.15);
  const step=10**Math.floor(Math.log10(extent));extent=Math.ceil(extent/step)*step;
  const left=39,right=w-9,top=14,bottom=h-27;
  const x=t=>left+(t-start)/windowSeconds*(right-left),y=v=>bottom-(v+extent)/(2*extent)*(bottom-top);
  c.font='9px ui-monospace,monospace';c.textAlign='right';
  for(const v of [-extent,0,extent]){
    c.strokeStyle=v===0?'#3a4b57':'#27343e';c.lineWidth=.7;c.beginPath();c.moveTo(left,y(v));c.lineTo(right,y(v));c.stroke();
    c.fillStyle='#7c919f';c.fillText(format(v,extent<1?2:0),left-7,y(v)+3);
  }
  c.textAlign='center';
  for(let i=0;i<=4;i++){
    const t=start+windowSeconds*i/4;
    c.strokeStyle='#25323b';c.beginPath();c.moveTo(x(t),top);c.lineTo(x(t),bottom);c.stroke();
    c.fillStyle='#7c919f';c.fillText(i===4?'今':`${format(t-end,Number.isInteger(windowSeconds/4)?0:1)}s`,x(t),h-8);
  }
  c.save();c.beginPath();c.rect(left,top,right-left,bottom-top);c.clip();
  colors.forEach((color,axis)=>{
    c.strokeStyle=color;c.lineWidth=1.3;c.beginPath();let previous=null,epoch=null;
    for(const s of points){const t=s.device_ns/1e9;
      if(panel.dataset.translation&&epoch!==s.pose.translation_generation)previous=null;
      epoch=s.pose.translation_generation;
      if(previous===null||t-previous>.15)c.moveTo(x(t),y(values(s,panel)[axis]));else c.lineTo(x(t),y(values(s,panel)[axis]));previous=t;
    }c.stroke();
  });c.restore();
  if(!points.length){c.fillStyle='#8496a2';c.textAlign='center';c.fillText('センサーデータを待っています',w/2,h/2);}
  if(displayed)panel.querySelectorAll('.readings b').forEach((b,i)=>{b.textContent=format(values(displayed,panel)[i],key==='mag_scaled'?4:3);});
}
function rotate(v,q){
  const [w,x,y,z]=q,[a,b,c]=v;
  const tx=2*(y*c-z*b),ty=2*(z*a-x*c),tz=2*(x*b-y*a);
  return [a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,c+w*tz+x*ty-y*tx];
}
function scene(){
  const [c,w,h]=surface($('pose'));
  const q=displayed?.pose.variants?.[poseMode]?.quaternion_wxyz||[1,0,0,0];
  const translation=displayed?.pose.variants?.[poseMode]?.translation;
  const position=translation?.position_view_m||[0,0,0];
  const trail=history.filter(s=>s.pose.translation_generation===displayed?.pose.translation_generation&&s.pose.reference_generation===displayed?.pose.reference_generation).slice(-500);
  let range=Number($('scene-range').value);
  if(!range){range=.25;for(const s of trail)for(const value of s.pose.variants[poseMode].translation.position_view_m)range=Math.max(range,Math.abs(value)*1.2);}
  const space=p=>p.map(v=>v/range*.8);
  const shift=space(position);
  // モデルは回転＋並進。参照グリッドと軌跡は同じ原点の座標系。

  const project=(p,apply=true)=>{
    let [x,y,z]=apply?rotate(p,q).map((v,i)=>v+shift[i]):p;
    const cy=Math.cos(-.32),sy=Math.sin(-.32),cx=Math.cos(.23),sx=Math.sin(.23);
    [x,z]=[cy*x+sy*z,-sy*x+cy*z];[y,z]=[cx*y-sx*z,sx*y+cx*z];
    const scale=Math.min(w*.22,h*.36)*4.6/Math.max(.5,4.6+z);
    return [w/2+x*scale,h*.49-y*scale];
  };
  function path(points,color,width=1,fill=null,apply=true){
    c.beginPath();points.forEach((p,i)=>{const [x,y]=project(p,apply);i?c.lineTo(x,y):c.moveTo(x,y);});
    if(fill){c.closePath();c.fillStyle=fill;c.fill();}c.strokeStyle=color;c.lineWidth=width;c.lineJoin='round';c.lineCap='round';c.stroke();
  }
  // 水平基準グリッド。
  for(let n=-5;n<=5;n++){
    path([[n*.4,-.7,-1.5],[n*.4,-.7,2]],'#24323c',.6,null,false);
    path([[-2,-.7,n*.35],[2,-.7,n*.35]],'#24323c',.6,null,false);
  }
  if(trail.length>1)path(trail.map(s=>space(s.pose.variants[poseMode].translation.position_view_m)),'#8ce1b0',1.5,null,false);
  const origin=project([0,0,0],false);c.fillStyle='#8ce1b0';c.beginPath();c.arc(origin[0],origin[1],3,0,Math.PI*2);c.fill();
  c.fillStyle='#8899a3';c.font='10px monospace';c.textAlign='left';c.fillText('±'+format(range,2)+' m',20,25);
  c.save();c.globalAlpha=.12;
  c.fillStyle='#8ce1b0';c.beginPath();c.ellipse(w/2,h*.76,w*.19,10,0,0,Math.PI*2);c.fill();c.restore();
  // 一体型のブローフレーム、左右レンズ、ブリッジ、テンプル。
  for(const side of [-1,1]){
    const points=[[side*.14,.21,-.33],[side*1.04,.24,-.26],[side*.98,-.28,-.24],[side*.25,-.3,-.32]];
    path(points,'#9fe3c2',1.8,'#31595166');
    path([[side*.12,.25,-.34],[side*1.09,.28,-.25],[side*1.10,.19,1.03],[side*.94,.04,1.27]],'#b2c6c7',5);
    path([[side*.28,-.23,-.33],[side*.25,-.08,-.23]],'#87a4a6',2);
    path([[side*.23,.13,-.34],[side*.88,.15,-.28]],'#c4f6e222',2);
  }
  path([[-.16,.18,-.34],[0,.24,-.4],[.16,.18,-.34]],'#d2e8e2',4);
  const axes=[[[.5,0,0],colors[0],'X'],[[0,.5,0],colors[1],'Y'],[[0,0,-.5],colors[2],'Z']];
  for(const [end,color,label] of axes){
    const p=rotate(end,q);const base=[w-66,51];c.strokeStyle=color;c.lineWidth=1.6;c.beginPath();c.moveTo(...base);c.lineTo(base[0]+p[0]*55,base[1]-p[1]*55+p[2]*20);c.stroke();c.fillStyle=color;c.font='9px monospace';c.fillText(label,base[0]+p[0]*65,base[1]-p[1]*65+p[2]*25);
  }
  if(!live&&!paused){c.fillStyle='#8899a3';c.font='11px sans-serif';c.textAlign='center';c.fillText('実機データの接続待ち',w/2,30);}
}
function draw(now){
  if(now-lastFrame>30){
    lastFrame=now;scene();panels.forEach(chart);
    if(displayed){
      const p=displayed.pose, selected=p.variants?.[poseMode]||p;
      for(const angle of ['yaw','pitch','roll'])$(angle).textContent=format(selected[angle],1)+'°';
      $('motion-state').textContent=p.rest_detected?'静止検出 · バイアス補正中':'動作中';
      $('bias').textContent=(p.bias_dps||[]).map(v=>format(v,3)).join(' / ');
      const delta=p.variants?((p.variants.mag.yaw-p.variants.imu.yaw+540)%360)-180:0;
      $('yaw-difference').textContent=format(delta,1)+'°';
      $('mag-state').textContent=!p.magnetic_reference_ready?'地磁気参照 学習中':p.magnetic_disturbance?'地磁気 外乱検出':'地磁気 正常';
      $('mag-state').className=p.magnetic_reference_ready&&p.magnetic_disturbance?'warning':'';
      $('fusion-hint').textContent=poseMode==='imu'?'VQFで回転、重力除去した加速度の二重積分で並進を推定。軌跡は直近約10秒。':'VQFが地磁気で方位を補正し、磁気外乱を検出します。磁気校正の物理検証は未完了のため比較用です。';
      const translation=selected.translation;
      $('integration-time').textContent=format(translation.elapsed_seconds,1);
      $('zupt-state').textContent=p.zupt_enabled?(translation.zupt_active?'静止 · 速度ゼロ補正中':'速度ゼロ補正 ON'):'速度補正 OFF';
      $('temperature').textContent=format(displayed.temperature_c,1);
    }else $('temperature').textContent='—';
  }
  requestAnimationFrame(draw);
}
status({state:'connecting',message:'本体情報と校正データを読み取っています。',samples:0});
requestAnimationFrame(draw);
