// Revalidate an actual RGB/Flyvis browser run against the shared scene geometry.
import {readFile,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {WorldGeometry} from '../environments/dangjin/world-geometry.js';

const root=new URL('../environments/dangjin/',import.meta.url);
const input=process.argv[2];
if(!input)throw new Error('Usage: node tools/validate_dangjin_patrol.mjs telemetry.json [report.json]');
const run=JSON.parse(await readFile(input,'utf8'));
const world=new WorldGeometry(JSON.parse(await readFile(new URL('site.json',root))));
assert.equal(run.model.backend,'flyvis');
assert.equal(run.model.bundle_sha256,'97f853183f79d2e987532ca2716ce3c33c2f41f875af05396e0cd8b14a7bc3c2');
assert.equal(run.model.flow_decoder_loaded,true);
assert.equal(run.perception.backend,'flyvis-road-readout');
assert.equal(run.perception.encoder_frozen,true);
assert.equal(run.perception.raw_image_skip_connection,false);
assert.equal(run.perception.views,4);
assert.equal(run.layout_revision,world.site.layout_revision);
assert.equal(run.perception.uses_ground_truth_at_runtime,false);
assert.equal(run.errors,0);
assert.equal(run.mission.complete,true);
assert.equal(run.mission.fraction,1);
assert.ok(run.mission.checkpoints>=49);
assert.equal(run.trace.at(-1).command.reason,'lap_complete');
assert.equal(run.trace.at(-1).command.vx,0);
let distance=0,clearance=Infinity,squared=0,maxError=0,maxStep=0;
for(let i=0;i<run.trace.length;i++){
  const frame=run.trace[i],p=frame.pose;
  assert.equal(world.blocked(p.x,p.y,.65),null,`blocked at frame ${i}`);
  assert.ok(frame.perception.ms>0&&frame.flow_ms>0,`missing model inference at ${i}`);
  assert.equal(frame.perception.views,4);assert.ok(frame.perception.circuit_ms>0);
  assert.ok(frame.command.vx>=0&&frame.command.vx<=.8);
  assert.ok(Math.abs(frame.command.yaw_rate)<=.35);
  if(i){
    const prev=run.trace[i-1].pose,step=Math.hypot(p.x-prev.x,p.y-prev.y);
    assert.ok(step<=.40001,`position jump at ${i}`);
    assert.equal(world.swept(prev,p,.65),null,`swept collision at ${i}`);
    distance+=step;maxStep=Math.max(maxStep,step);
  }
  const edge=world.nearestRoad(p.x,p.y).clearance;
  const building=Math.min(...world.obstacles.map(r=>Math.hypot(Math.max(r.x-p.x,0,p.x-r.x-r.length),Math.max(r.y-p.y,0,p.y-r.y-r.width))));
  const zone=Math.min(...world.zones.map(r=>Math.hypot(Math.max(r.x-p.x,0,p.x-r.x-r.length),Math.max(r.y-p.y,0,p.y-r.y-r.width))));
  clearance=Math.min(clearance,edge,building,zone);
  const error=frame.mission.cross_track_m;squared+=error**2;maxError=Math.max(maxError,error);
}
const report={schema:'dangjin.patrol.validation.v1',source:input,frames:run.trace.length,
  layout_revision:run.layout_revision,road_backend:run.perception.backend,detected_camera_views:4,
  distance_m:distance,route_length_m:run.mission.length_m,simulation_seconds:run.trace.at(-1).stamp_sec-run.trace[0].stamp_sec,
  checkpoints:run.mission.checkpoints,complete:true,automatic_stop:run.trace.at(-1).command,
  unexpected_stops:run.trace.filter(t=>t.command.vx===0&&t.command.reason!=='lap_complete').length,
  swept_collisions:0,minimum_center_clearance_m:clearance,minimum_footprint_clearance_m:clearance-.65,
  route_center_rmse_m:Math.sqrt(squared/run.trace.length),route_center_max_error_m:maxError,max_step_m:maxStep,
  flyvis_sha256:run.model.bundle_sha256,road_model_sha256:run.perception.weights_sha256,
  runtime_ground_truth:false,localization:'known simulation pose',physics:'kinematic',
  scope:'One synthetic RGB closed-loop lap; not a real-world robot performance claim'};
report.source_sha256={};
for(const path of ['site.json','world.json','world-geometry.js','scene-details.js','robot-sim.js','app.js',
  '../../tools/dangjin_sim_server.py','../../tools/dangjin_patrol.py','../../tools/connectome_road.py','../../tools/train_connectome_road.py']){
  report.source_sha256[path]=createHash('sha256').update(await readFile(new URL(path,root))).digest('hex');
}
assert.equal(report.unexpected_stops,0);
if(process.argv[3])await writeFile(process.argv[3],JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
