import dgram from 'node:dgram';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { decode, encode } from './bridge/mavlink.mjs';

const gcs = dgram.createSocket('udp4');
await new Promise((r) => gcs.bind(0, '127.0.0.1', r));
const sse = [];
let bridgeAddr = null;
gcs.on('message', (buf, rinfo) => { bridgeAddr = rinfo; });

const bridge = spawn(process.execPath, [fileURLToPath(new URL('./bridge/server.mjs', import.meta.url))], {
  env: { ...process.env, BRIDGE_HTTP_PORT: '0', GCS_PORT: String(gcs.address().port) }, stdio: ['ignore','pipe','inherit'],
});
const httpPort = await new Promise((res) => bridge.stdout.on('data', (d) => { const m=String(d).match(/http=(\d+)/); if(m) res(+m[1]); }));
await new Promise(r=>setTimeout(r,300));
// open SSE
const s = await fetch(`http://127.0.0.1:${httpPort}/commands`);
const rd = s.body.getReader(); const dec = new TextDecoder(); let buf='';
(async()=>{ for(;;){ const {value,done}=await rd.read(); if(done)break; buf+=dec.decode(value,{stream:true}); let i; while((i=buf.indexOf('\n\n'))>=0){ const c=buf.slice(0,i); buf=buf.slice(i+2); const d=c.split('\n').find(l=>l.startsWith('data: ')); if(d) sse.push(JSON.parse(d.slice(6))); } } })().catch(()=>{});
const send=(n,f)=>new Promise(r=>gcs.send(encode(n,f),bridgeAddr.port,bridgeAddr.address,r));
await new Promise(r=>setTimeout(r,200));

// Simulate QGC writing FENCE_ENABLE (as it might when the fence page is used)
await send('PARAM_SET',{param_value:1,target_system:1,target_component:1,param_id:'FENCE_ENABLE',param_type:9});
await new Promise(r=>setTimeout(r,200));
console.log('after FENCE_ENABLE set, SSE fence cmds:', JSON.stringify(sse.filter(e=>e.type==='fence')));
bridge.kill(); gcs.close(); process.exit(0);
