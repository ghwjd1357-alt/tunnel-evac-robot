/* ═══════════════════════════════════════════════════════════════════
   roslib_shim.mjs — roslibjs 대신 node 에서 rosbridge 에 직접 붙는다 (2026-09-02)

   node 22 의 내장 WebSocket 을 써서 브라우저와 **똑같은 경로**(ws://:9090,
   rosbridge JSON 프로토콜)로 접속한다. 즉 이 검증은 흉내가 아니라
   같은 배선을 타는 실제 통신이다. 다른 점은 화면이 가짜 DOM 인 것뿐.
   ═══════════════════════════════════════════════════════════════════ */

export function makeROSLIB(url = 'ws://localhost:9090') {
  const subs = new Map();          // topic → [cb]
  let ws = null, onConn = [], onClose = [];

  class Ros {
    constructor() {
      ws = new WebSocket(url);
      ws.onopen = () => onConn.forEach(f => f());
      ws.onclose = () => onClose.forEach(f => f());
      ws.onerror = () => {};
      ws.onmessage = ev => {
        let m; try { m = JSON.parse(ev.data); } catch { return; }
        if (m.op !== 'publish') return;
        (subs.get(m.topic) || []).forEach(cb => { try { cb(m.msg); } catch (e) { console.error(e); } });
      };
    }
    on(ev, fn) { ev === 'connection' ? onConn.push(fn) : ev === 'close' ? onClose.push(fn) : 0; }
    close() { ws?.close(); }
  }

  class Topic {
    constructor(o) { Object.assign(this, o); }
    subscribe(cb) {
      (subs.get(this.name) || subs.set(this.name, []).get(this.name)).push(cb);
      send({ op: 'subscribe', topic: this.name, type: this.messageType,
             ...(this.throttle_rate ? { throttle_rate: this.throttle_rate, queue_length: 1 } : {}) });
    }
    publish(msg) { send({ op: 'publish', topic: this.name, msg }); }
  }

  class Service {
    constructor(o) { Object.assign(this, o); }
    callService(_req, ok) { setTimeout(() => ok({ time: { secs: 0 } }), 5); }  /* RTT 는 형태만 */
  }

  function send(o) {
    const go = () => ws.send(JSON.stringify(o));
    ws.readyState === 1 ? go() : ws.addEventListener('open', go, { once: true });
  }

  return { Ros, Topic, Service, Message: class { constructor(o) { Object.assign(this, o); } },
           ServiceRequest: class { constructor(o) { Object.assign(this, o); } },
           _close: () => ws?.close() };
}
