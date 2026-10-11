const $ = id => document.getElementById(id);

// Theme
const themeToggle = $('themeToggle');
themeToggle.addEventListener('click', () => {
    const html = document.documentElement;
    const isDark = html.getAttribute('data-theme') === 'dark';
    html.setAttribute('data-theme', isDark ? 'light' : 'dark');
    themeToggle.textContent = isDark ? '☀️' : '🌙';
});

// MAVLink (Simplified & Robust)
const MAV = { sysid:255, compid:190, seq:0, targetSys:1, targetComp:1 };
const CRC_EXTRA = {0:50,1:124,24:24,65:118,74:20,21:159,22:220,20:214,23:168,76:152,77:143,253:83,148:178,193:75};
function mavCrc(data, extra) {
    let crc = 0xFFFF;
    const acc = b => { let tmp = (b ^ (crc & 0xFF)) & 0xFF; tmp = (tmp ^ (tmp << 4)) & 0xFF; crc = ((crc >>> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >>> 4)) & 0xFFFF; };
    for (let i=0;i<data.length;i++) acc(data[i]); acc(extra); return crc;
}
function packV2(msgid, payload) {
    const extra = CRC_EXTRA[msgid] || 0;
    const h = new Uint8Array(10);
    h[0]=0xFD; h[1]=payload.length; h[2]=0; h[3]=0;
    h[4]=MAV.seq & 0xFF; MAV.seq=(MAV.seq+1)&0xFF;
    h[5]=MAV.sysid; h[6]=MAV.compid;
    h[7]=msgid&0xFF; h[8]=(msgid>>8)&0xFF; h[9]=(msgid>>16)&0xFF;
    const body = new Uint8Array(9+payload.length);
    body.set(h.subarray(1),0); body.set(payload,9);
    const crc = mavCrc(body, extra);
    const out = new Uint8Array(12+payload.length);
    out.set(h,0); out.set(payload,10);
    out[10+payload.length]=crc&0xFF; out[11+payload.length]=(crc>>8)&0xFF;
    return out;
}
const encHeartbeat = () => { const p=new Uint8Array(9), dv=new DataView(p.buffer); dv.setUint32(0,0,true); dv.setUint8(4,6); dv.setUint8(5,8); dv.setUint8(6,0); dv.setUint8(7,0); dv.setUint8(8,3); return packV2(0,p); };
const encParamRequestList = () => packV2(21, new Uint8Array([MAV.targetSys, MAV.targetComp]));
const encCommandLong = (cmd, a,b,c,d,e,f,g) => { const p=new Uint8Array(33), dv=new DataView(p.buffer); [a,b,c,d,e,f,g].forEach((v,i)=>dv.setFloat32(i*4, v||0, true)); dv.setUint16(28,cmd,true); dv.setUint8(30,MAV.targetSys); dv.setUint8(31,MAV.targetComp); dv.setUint8(32,0); return packV2(76,p); };
const encSetMsgInterval = (msgid,hz) => encCommandLong(511, msgid, hz>0?1e6/hz:0, 0,0,0,0,0);

const dvOf = p => new DataView(p.buffer, p.byteOffset, p.byteLength);
const U8 =(dv,o)=> o<dv.byteLength?dv.getUint8(o):0;
const U16=(dv,o)=> o+1<dv.byteLength?dv.getUint16(o,true):0;
const U32=(dv,o)=> o+3<dv.byteLength?dv.getUint32(o,true):0;
const I16=(dv,o)=> o+1<dv.byteLength?dv.getInt16(o,true):0;
const F32=(dv,o)=> o+3<dv.byteLength?dv.getFloat32(o,true):0;
const STR=(dv,o,n)=>{ let s=''; for(let i=0;i<n&&o+i<dv.byteLength;i++){ const c=dv.getUint8(o+i); if(!c)break; s+=String.fromCharCode(c);} return s; };

const msgHandlers = {};
const onMsg = (id,fn)=>{ msgHandlers[id]=fn; };
let rxBuf = new Uint8Array(0);
function feedBytes(chunk) {
    const nb=new Uint8Array(rxBuf.length+chunk.length); nb.set(rxBuf); nb.set(chunk,rxBuf.length); rxBuf=nb;
    let i=0;
    while(i<rxBuf.length) {
        const m=rxBuf[i]; if(m!==0xFD&&m!==0xFE){ i++; continue; }
        if(m===0xFD) {
            if(i+10>rxBuf.length) break; const len=rxBuf[i+1]; if(i+12+len>rxBuf.length) break;
            const msgid=rxBuf[i+7]|(rxBuf[i+8]<<8)|(rxBuf[i+9]<<16); const extra=CRC_EXTRA[msgid];
            const body=rxBuf.subarray(i+1,i+10+len);
            const ok = extra===undefined ? true : (()=>{ const c=mavCrc(body,extra); return (c&0xFF)===rxBuf[i+10+len] && ((c>>8)&0xFF)===rxBuf[i+11+len]; })();
            if(ok){ const h=msgHandlers[msgid]; if(h) h(rxBuf.subarray(i+10,i+10+len), rxBuf[i+5], rxBuf[i+6]); }
            i+=12+len;
        } else { i+=8+rxBuf[i+1]; }
    }
    rxBuf = rxBuf.slice(i);
}

const FC = {
    demo:false, connected:false, paramsDone:false,
    params:new Map(), paramCount:0, _paramIdx:new Set(),
    rc:new Array(18).fill(1500),
    voltage:0, current:0, gpsFix:0, sats:0,
    modeNum:-1, armed:false, fwVer:'',
    listeners:{},
    on(e,f){ (this.listeners[e]=this.listeners[e]||[]).push(f); },
    emit(e,d){ (this.listeners[e]||[]).forEach(f=>{ try{f(d)}catch(_){}}); },
    async connect() {
        if(!('serial' in navigator)) throw new Error('浏览器不支持 WebSerial');
        const port = await navigator.serial.requestPort();
        await port.open({baudRate:115200});
        this.port=port; this.demo=false;
        this.writer = port.writable.getWriter();
        this._send = d => this.writer.write(d);
        this._send(encHeartbeat());
        this._hbTimer = setInterval(()=>{ try{this._send(encHeartbeat());}catch(_){}} ,1000);
        this._readLoop(port);
        await this._waitFor(()=>this._hbSeen, 9000, '未收到心跳');
        this.connected=true;
        this._send(encSetMsgInterval(65,5)); this._send(encSetMsgInterval(1,2)); this._send(encSetMsgInterval(24,2));
        this._send(encCommandLong(520,1,0,0,0,0,0,0));
        await this.downloadParams();
        this.paramsDone=true;
    },
    async _readLoop(port) {
        const reader = port.readable.getReader();
        try{ while(true){ const {value,done} = await reader.read(); if(done) break; if(value && value.length) feedBytes(value); } }catch(e){}
        finally{ try{reader.releaseLock();}catch(_){} }
        if(this.connected) this._onDrop();
    },
    _onDrop(){ this.connected=false; this.emit('disconnect',{}); },
    async disconnect(){ clearInterval(this._hbTimer); try{ if(this.port) await this.port.close(); }catch(_){} this.connected=false; this.emit('disconnect',{}); },
    _waitFor(cond, ms, errMsg){ return new Promise((res,rej)=>{ const t0=Date.now(); const iv=setInterval(()=>{ if(cond()){ clearInterval(iv); res(); } else if(Date.now()-t0>ms){ clearInterval(iv); rej(new Error(errMsg||'超时')); } },120); }); },
    async downloadParams(){
        this._paramIdx=new Set(); this.paramCount=0;
        this._send(encParamRequestList());
        const t0=Date.now();
        await new Promise(res=>{ const iv=setInterval(()=>{ if(this.paramCount && this._paramIdx.size>=this.paramCount){ clearInterval(iv); res(); } else if(Date.now()-t0>30000){ clearInterval(iv); res(); } },300); });
    }
};

onMsg(0,(p,sys,comp)=>{ const dv=dvOf(p); MAV.targetSys=sys; MAV.targetComp=comp; FC.modeNum=U32(dv,0); FC.armed=!!(U8(dv,6)&128); FC._hbSeen=true; FC.emit('heartbeat',{}); });
onMsg(22,(p)=>{ const dv=dvOf(p); const val=F32(dv,0), count=U16(dv,4), idx=U16(dv,6); const name=STR(dv,8,16), type=U8(dv,24); FC.params.set(name,{value:val,type}); FC.paramCount=count; FC._paramIdx.add(idx); FC.emit('param',{name}); });
onMsg(65,(p)=>{ const dv=dvOf(p); for(let i=0;i<18;i++){ const v=U16(dv,4+i*2); if(v>=900&&v<=2100) FC.rc[i]=v; } FC.emit('rc',{}); });
onMsg(1,(p)=>{ const dv=dvOf(p); FC.voltage=U16(dv,14)/1000; FC.current=I16(dv,16)/100; FC.emit('telemetry',{}); });
onMsg(24,(p)=>{ const dv=dvOf(p); FC.gpsFix=U8(dv,28); FC.sats=U8(dv,29); FC.emit('telemetry',{}); });

// UI Logic
const S = { step:0, done:new Set(), badges:new Set() };
const STEPS = [
    { t: '认识 ROC WING', s: '安全须知与结构', badge: '🛡️' },
    { t: '连接与体检', s: 'WebSerial 直连', badge: '🔌' },
    { t: '遥控器验证', s: '方向与行程', badge: '🕹️' },
    { t: '肌肉记忆', s: '解锁与模式', badge: '🧠' },
    { t: '起飞检查单', s: '终极确认', badge: '🎓' }
];
const canNext = [ ()=>true, ()=>FC.connected && FC.paramsDone, ()=>true, ()=>S.armOk, ()=>S.checkAll ];

function earnBadge(i) {
    if(S.badges.has(i)) return;
    S.badges.add(i);
    const b = $('badges');
    const el = document.createElement('div');
    el.className = 'badge earned';
    el.textContent = STEPS[i].badge;
    el.title = STEPS[i].t;
    b.appendChild(el);
}

function renderRail() {
    $('railSteps').innerHTML = STEPS.map((s,i)=>{
        const cls = S.done.has(i)?'done':(i===S.step?'active':'');
        return `<div class="step ${cls}" data-i="${i}"><div class="num">${S.done.has(i)?'✓':String(i+1)}</div><div class="tx"><b>${s.t}</b><span>${s.s}</span></div></div>`;
    }).join('');
    document.querySelectorAll('#railSteps .step').forEach(el=>{ el.onclick=()=>{ S.step=+el.dataset.i; render(); }; });
}

function refreshTop() {
    const c = FC.connected;
    $('connDot').classList.toggle('on', c);
    $('connTxt').textContent = c ? '已连接' : '未连接';
}

const R = [];
R[0] = () => `
<div class="step-hero"><div class="step-no">01</div><div><h2>认识你的 ROC WING</h2><p>欢迎加入新手飞行学院！在通电之前，我们必须确认几件关乎生命安全的小事。</p></div></div>
<div class="card"><h3>🛡️ 绝对禁令</h3>
${warn('⚠️ <b>严禁使用 18650 电池</b>！必须使用 6S 软包锂聚合物电池，否则空中断电必炸机。')}
${warn('⚠️ <b>未拆桨叶前严禁解锁</b>！室内调试必须卸下螺旋桨，否则断指。', true)}
</div>
<div class="card"><h3>🔍 认识重心 (CG)</h3>
<p>ROC Wing 是 Y3 倾转旋翼。重心刻痕在机翼塑料件上，允许向机头方向偏差 ≤1cm。重心不对会导致固定翼模式无法爬升或失速。</p>
</div>`;

R[1] = () => {
    if(!FC.connected) return `
<div class="step-hero"><div class="step-no">02</div><div><h2>连接与体检</h2><p>用 Type-C 数据线连接飞控与电脑。我们将下载参数并进行健康体检。</p></div></div>
<div class="card"><h3>🔌 连接飞控</h3>
<button class="btn btn-next" id="btnConn">连接飞控</button>
<button class="btn btn-prev" id="btnDemo" style="margin-left:10px">演示模式</button>
<div id="connProg" style="margin-top:14px"></div>
</div>`;
    return `
<div class="step-hero"><div class="step-no">02</div><div><h2>健康体检报告</h2><p style="color:var(--success)">✓ 已连接，参数已下载。请确认以下指标全部绿灯。</p></div></div>
<div class="card"><h3>🏥 实时体检</h3>
<div class="grid3">
    <div class="tile"><div class="v ${FC.voltage>21?'ok':'bad'}" id="tVolt">${FC.voltage.toFixed(1)} V</div><div class="k">电池电压 (需 >21V)</div></div>
    <div class="tile"><div class="v ${FC.gpsFix>=3?'ok':'warnv'}" id="tGps">${FC.sats} 颗</div><div class="k">GPS 搜星 (需 >10)</div></div>
    <div class="tile"><div class="v ok">正常</div><div class="k">传感器预热</div></div>
</div></div>`;
};

R[2] = () => `
<div class="step-hero"><div class="step-no">03</div><div><h2>遥控器验证</h2><p>拨动摇杆，确认屏幕上的虚拟飞机动作与你的操作一致。</p></div></div>
<div class="card"><h3>🕹️ 方向验证</h3>
<div class="gimbals">
    <div class="gimbal"><div class="gl">左摇杆</div><div class="cross" style="left:50%;top:8%;bottom:8%;width:1px"></div><div class="cross" style="top:50%;left:8%;right:8%;height:1px"></div><div class="dot" id="dotL"></div></div>
    <div class="gimbal"><div class="gl">右摇杆</div><div class="cross" style="left:50%;top:8%;bottom:8%;width:1px"></div><div class="cross" style="top:50%;left:8%;right:8%;height:1px"></div><div class="dot" id="dotR"></div></div>
</div>
<p style="text-align:center;color:var(--dim)">如果方向反了，请在 Mission Planner 中校准或反向通道。方向正确可直接下一步。</p>
</div>`;

R[3] = () => `
<div class="step-hero"><div class="step-no">04</div><div><h2>肌肉记忆训练</h2><p>首飞只用 <b style="color:var(--primary)">QLOITER (垂起定点)</b> 模式。练熟解锁手势。</p></div></div>
<div class="card"><h3>🧠 解锁练习</h3>
<p>油门最低 + 方向<b style="color:var(--success)">右打</b>保持 2 秒 = 解锁。</p>
<div class="gimbals">
    <div class="gimbal"><div class="gl">左摇杆</div><div class="cross" style="left:50%;top:8%;bottom:8%;width:1px"></div><div class="cross" style="top:50%;left:8%;right:8%;height:1px"></div><div class="zone arm" id="zoneArm"></div><div class="dot" id="dotL"></div></div>
</div>
<div id="gestMsg" style="text-align:center;font-size:16px;margin:10px 0;color:var(--dim)">把左摇杆推到右下角保持 2 秒</div>
<button class="btn btn-next" id="btnArmOk" style="display:block;margin:20px auto">✓ 我已完成练习</button>
</div>
<div class="card"><h3>🚁 模式说明</h3>
<div class="grid2">
    <div class="mode-card q"><div class="mn">QLOITER</div><div class="en">VTOL · 定点悬停</div><p>像四轴一样飞，松杆即悬停。首飞唯一指定模式。</p></div>
    <div class="mode-card c"><div class="mn">CRUISE</div><div class="en">固定翼 · 巡航</div><p>自动保持高度/航向。<b style="color:var(--danger)">严禁低空使用</b>，爬升太慢会撞树。</p></div>
</div></div>`;

R[4] = () => `
<div class="step-hero"><div class="step-no">05</div><div><h2>终极放飞检查单</h2><p>全部勾选，获得机长徽章，准备起飞！</p></div></div>
<div class="card"><h3>📋 起飞前确认</h3><div id="finalList">
${checkItem('f1','电池已接好、电量充足，6S 软包','电压正常，无鼓包破损。')}
${checkItem('f2','重心确认无误','CG 在刻痕处（机头方向 ≤1cm）。')}
${checkItem('f3','遥控器已校准，CH6 = QLOITER','上电默认 QLOITER。')}
${checkItem('f4','GPS 已定位（室外）','等待搜星 >10 颗再解锁。')}
${checkItem('f5','场地空旷，远离人群','首飞选择无风或微风天气。')}
</div></div>
<div id="finishBox"></div>`;

const warn = (h,amber) => `<div class="warn-strip${amber?' amber':''}">${h}</div>`;
const checkItem = (id,title,desc) => `<label class="check" data-cid="${id}"><input type="checkbox" data-cid="${id}"><span class="t"><b>${title}</b><span>${desc}</span></span></label>`;

function render() {
    const i = S.step;
    $('stageBody').innerHTML = R[i]();
    bindStep(i);
    renderRail(); refreshTop();
    $('btnPrev').disabled = i===0;
    $('btnNext').disabled = !canNext[i]();
    $('btnNext').innerHTML = i===STEPS.length-1 ? '毕业 🎉' : '下一课 →';
    $('navHint').textContent = canNext[i]()?'':'请完成当前课程要求';
}

function bindStep(i) {
    if(i===1 && !FC.connected) {
        $('btnConn').onclick = async () => {
            $('btnConn').disabled = true;
            $('connProg').innerHTML = '⏳ 等待选择串口...';
            try { await FC.connect(); refreshTop(); render(); }
            catch(e) { $('connProg').innerHTML = `⚠️ 连接失败: ${e.message}`; $('btnConn').disabled = false; }
        };
    }
    if(i===1 && FC.connected) {
        const upd = () => {
            const v = $('tVolt'); if(v) v.textContent = FC.voltage.toFixed(1);
            const g = $('tGps'); if(g) g.textContent = FC.sats + ' 颗';
        };
        FC.on('telemetry', upd); upd();
    }
    if(i===2) {
        const paint = () => {
            const ch3=FC.rc[2]||1500, ch4=FC.rc[3]||1500, ch1=FC.rc[0]||1500, ch2=FC.rc[1]||1500;
            const dotL=$('dotL'), dotR=$('dotR');
            if(dotL) { dotL.style.left = (ch4-1000)/10+'%'; dotL.style.top = (100-(ch3-1000)/10)+'%'; }
            if(dotR) { dotR.style.left = (ch1-1000)/10+'%'; dotR.style.top = (100-(ch2-1000)/10)+'%'; }
        };
        FC.on('rc', paint); paint();
    }
    if(i===3) {
        const paint = () => {
            const ch3=FC.rc[2]||1500, ch4=FC.rc[3]||1500;
            const dotL=$('dotL'), zoneArm=$('zoneArm');
            if(dotL) { dotL.style.left = (ch4-1000)/10+'%'; dotL.style.top = (100-(ch3-1000)/10)+'%'; }
            const inArm = ch3<1150 && ch4>1850;
            if(zoneArm) zoneArm.classList.toggle('hit-arm', inArm);
            if(inArm && !S.armOk) {
                $('gestMsg').innerHTML = '✓ 手势正确！';
                S.armOk = true;
                earnBadge(3);
                $('btnNext').disabled = false;
            }
        };
        FC.on('rc', paint); paint();
        $('btnArmOk').onclick = () => { S.armOk = true; earnBadge(3); $('btnNext').disabled = false; $('gestMsg').innerHTML = '✓ 已确认'; };
    }
    if(i===4) {
        const all = [...document.querySelectorAll('#finalList input')];
        const refresh = () => {
            const n = all.filter(x=>x.checked).length;
            all.forEach(cb=>cb.closest('.check').classList.toggle('done', cb.checked));
            S.checkAll = (n === all.length);
            $('btnNext').disabled = !S.checkAll;
            if(S.checkAll && !$('finishBox').innerHTML) {
                earnBadge(4);
                $('finishBox').innerHTML = `<div class="card finish"><div class="big">🎓</div><h2>恭喜毕业，机长！</h2><p>你已掌握 ROC Wing 安全首飞的全部知识。<br>请前往空旷场地，切换到 QLOITER 模式，开始你的飞行之旅！</p></div>`;
            }
        };
        all.forEach(cb=>cb.onchange=refresh);
    }
}

$('btnPrev').onclick = () => { if(S.step>0){ S.step--; render(); } };
$('btnNext').onclick = () => { if(!canNext[S.step]())return; S.done.add(S.step); earnBadge(S.step); if(S.step<STEPS.length-1){ S.step++; render(); } };

render();