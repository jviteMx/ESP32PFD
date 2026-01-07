#pragma once

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <pgmspace.h>

struct MirrorPayload {
  float pitch_deg;
  float roll_deg;
  float slip_px;
  float turnRate_dps;
  float speed_disp;
  float altitude_ft;
  uint8_t state;
  uint8_t gps_ok;
  uint8_t imu_ok;
  const char* unit;
};

class PhoneMirror {
public:
  void beginAP(const char* ssid, const char* pass) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, pass);
    ws_.onEvent(PhoneMirror::onWsEvent);
    server_.addHandler(&ws_);

    server_.on("/", HTTP_GET, [&](AsyncWebServerRequest* req) {
      req->send_P(200, "text/html", INDEX_HTML);
    });

    server_.on("/health", HTTP_GET, [&](AsyncWebServerRequest* req) {
      req->send(200, "text/plain", "ok");
    });

    server_.begin();
  }

  void tick() { ws_.cleanupClients(); }

  void broadcast(uint32_t now_ms, uint32_t period_ms, const MirrorPayload& p) {
    if ((now_ms - last_tx_ms_) < period_ms) return;
    last_tx_ms_ = now_ms;

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"pitch\":%.2f,\"roll\":%.2f,\"slip\":%.2f,\"turn\":%.2f,"
             "\"spd\":%.2f,\"alt\":%.1f,\"state\":%u,\"gps\":%u,\"imu\":%u,\"unit\":\"%s\"}",
             p.pitch_deg, p.roll_deg, p.slip_px, p.turnRate_dps,
             p.speed_disp, p.altitude_ft,
             (unsigned)p.state, (unsigned)p.gps_ok, (unsigned)p.imu_ok, p.unit);

    ws_.textAll(buf);
  }

  IPAddress apIP() const { return WiFi.softAPIP(); }

private:
  static void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
                        AwsEventType type, void*, uint8_t*, size_t) {
    if (type == WS_EVT_CONNECT) {
      Serial.printf("WS connect: %u\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("WS disconnect: %u\n", client->id());
    }
  }

  AsyncWebServer server_{80};
  AsyncWebSocket ws_{"/ws"};
  uint32_t last_tx_ms_ = 0;

  static constexpr const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>PFD Mirror</title>
  <style>
    body{margin:0;background:#000;color:#fff;font-family:Arial;}
    #wrap{display:flex;flex-direction:column;align-items:center;gap:8px;padding:10px;}
    canvas{background:#111;border:1px solid #444;max-width:98vw;height:auto;}
    .row{display:flex;gap:14px;flex-wrap:wrap;justify-content:center}
    .badge{padding:6px 10px;border:1px solid #555;border-radius:8px;background:#111}
  </style>
</head>
<body>
<div id="wrap">
  <div class="row">
    <div class="badge" id="st">STATE: --</div>
    <div class="badge" id="sp">SPD: --</div>
    <div class="badge" id="al">ALT: --</div>
  </div>
  <canvas id="c" width="360" height="270"></canvas>
  <div style="opacity:.7;font-size:12px">Open: http://192.168.4.1</div>
</div>
<script>
const c=document.getElementById('c'), ctx=c.getContext('2d');
const st=document.getElementById('st'), sp=document.getElementById('sp'), al=document.getElementById('al');

let s={pitch:0,roll:0,slip:0,turn:0,spd:0,alt:0,state:0,gps:0,imu:0,unit:"km/h"};

function draw(){
  const W=c.width,H=c.height;
  ctx.clearRect(0,0,W,H);

  const sky="#007a8c", ground="#6b3b1f", fg="#ffffff";
  const cx=W/2, cy=H/2;
  const pitch_px = s.pitch * 3.6;
  const r = s.roll * Math.PI/180;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(r);

  ctx.fillStyle=sky;
  ctx.fillRect(-W, -H*2 + pitch_px, W*2, H*2);

  ctx.fillStyle=ground;
  ctx.fillRect(-W, pitch_px, W*2, H*2);

  ctx.strokeStyle=fg;
  ctx.lineWidth=2;
  ctx.beginPath();
  ctx.moveTo(-W, pitch_px);
  ctx.lineTo(W, pitch_px);
  ctx.stroke();

  ctx.font="12px Arial";
  ctx.fillStyle=fg;
  ctx.strokeStyle=fg;
  ctx.lineWidth=1;
  for(let p=-30;p<=30;p+=5){
    if(p===0) continue;
    const y=(s.pitch-p)*3.6;
    if(y<-140||y>140) continue;
    const L=(p%10===0)?45:25;
    ctx.beginPath();
    ctx.moveTo(-L,y); ctx.lineTo(L,y); ctx.stroke();
    if(p%10===0){
      ctx.fillText(Math.abs(p).toString(), L+6, y+4);
      ctx.fillText(Math.abs(p).toString(), -L-22, y+4);
    }
  }
  ctx.restore();

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(r);
  ctx.fillStyle=fg;
  ctx.fillRect(-45,-3,90,6);
  ctx.fillRect(-4,-32,8,64);
  ctx.restore();

  ctx.strokeStyle=fg; ctx.lineWidth=2;
  ctx.beginPath();
  ctx.arc(cx,55,70,(Math.PI*200)/180,(Math.PI*340)/180);
  ctx.stroke();

  const rr=Math.max(-60,Math.min(60,s.roll))*Math.PI/180;
  const pr=70;
  const px=cx+Math.sin(rr)*pr;
  const py=55-Math.cos(rr)*pr;
  ctx.fillStyle="#ffd400";
  ctx.beginPath();
  ctx.moveTo(px,py); ctx.lineTo(px+10,py+12); ctx.lineTo(px-10,py+12);
  ctx.closePath(); ctx.fill();

  ctx.strokeStyle=fg; ctx.lineWidth=2;
  ctx.beginPath();
  ctx.moveTo(cx-70,H-55); ctx.lineTo(cx+70,H-55); ctx.stroke();
  const t=Math.max(-40,Math.min(40,s.turn*3));
  ctx.beginPath();
  ctx.moveTo(cx+t,H-62); ctx.lineTo(cx+t,H-48); ctx.stroke();

  ctx.strokeRect(cx-60,H-35,120,20);
  ctx.beginPath();
  ctx.moveTo(cx,H-33); ctx.lineTo(cx,H-17); ctx.stroke();
  ctx.fillStyle=fg;
  ctx.beginPath();
  ctx.arc(cx+s.slip,H-25,6,0,Math.PI*2); ctx.fill();
}

function upd(){
  st.textContent="STATE: "+(s.state===3?"FAULT":s.state===2?"DEGRADED":"RUN")+(s.gps?"":" | GPS STALE");
  sp.textContent=`SPD: ${s.spd.toFixed(0)} ${s.unit}`;
  al.textContent=`ALT: ${s.alt.toFixed(0)} ft`;
}

function connect(){
  const w=new WebSocket(`ws://${location.host}/ws`);
  w.onclose=()=>setTimeout(connect,1000);
  w.onmessage=(ev)=>{ try{ s=JSON.parse(ev.data); upd(); draw(); }catch(e){} };
}
connect();
</script>
</body>
</html>
)HTML";
};
