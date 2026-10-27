// Evaluate an actual browser RGB closed-loop run. No rollout or pose correction.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {WorldGeometry,stepPose,rectangleDistance} from '../environments/dangjin/world-geometry.js';

const [input,output]=process.argv.slice(2);
if(!input)throw Error('Usage: node tools/validate_dangjin_neural_drive.mjs run.json [report.json]');
const root=new URL('../environments/dangjin/',import.meta.url);
const run=JSON.parse(await readFile(input));
const world=new WorldGeometry(JSON.parse(await readFile(new URL('site.json',root))));
assert.equal(run.layout_revision,world.site.layout_revision);
assert.equal(run.policy.backend,'flyvis-drive-readout');
assert.equal(run.policy.map_assistance,false);assert.equal(run.policy.goal_input,false);assert.equal(run.policy.pose_input,false);
assert.equal(run.policy.runtime_planner,false);assert.equal(run.policy.runtime_pure_pursuit,false);
assert.equal(run.policy.emergency_stop_override,false);assert.equal(run.policy.fault_flag_input,false);
assert.equal(run.perception.backend,'flyvis-road-readout');assert.equal(run.perception.uses_ground_truth_at_runtime,false);
assert.equal(run.errors,0);assert.equal(run.mission.complete,true);
assert.equal(run.trace.at(-1).command.reason,'lap_complete');
assert.ok(run.sensor_faults.enabled);
assert.ok(run.sensor_faults.seed<1000||run.sensor_faults.seed>=3000,'evaluation seed must be outside train/validation range');
let distance=0,stopFrames=0,resumes=0,collisions=0,holes=0,dropFrames=0,maxIntegrationError=0,sumSquare=0,maxError=0,minClearance=Infinity;
const episodes=new Set();
for(let i=0;i<run.trace.length;i++){
  const t=run.trace[i],f=t.sensor_faults;
  assert.ok(t.perception.circuit_ms>0&&t.policy.ms>0);assert.equal(t.perception.views,4);
  assert.equal(t.camera_yaw,0);assert.equal(t.surface_ms,0);assert.equal(t.flow_ms,0);
  assert.equal(t.policy.override,false);assert.ok(!('local_goal' in t.policy));assert.ok(!('target_site' in t.mission));
  assert.equal(t.environment?.oncoming_car??null,null,'full-lap static-scene report does not cover moving obstacles');
  if(i<run.trace.length-1)assert.deepEqual(t.command,t.policy.model_command,'model command was overwritten');
  assert.ok(t.command.vx>=0&&t.command.vx<=.8&&Math.abs(t.command.yaw_rate)<=.35);
  if(f.kind==='holes')holes++;
  if(f.kind==='dropout'){dropFrames++;episodes.add(f.event_id);assert.ok(f.remaining_road_pixels.every(v=>v===0));}
  if(t.command.reason==='neural_wait'){stopFrames++;assert.ok(t.policy.stop_probability>.5);}
  sumSquare+=t.mission.cross_track_m**2;maxError=Math.max(maxError,t.mission.cross_track_m);
  minClearance=Math.min(minClearance,world.nearestRoad(t.pose.x,t.pose.y).clearance,
    ...world.obstacles.map(r=>rectangleDistance(t.pose.x,t.pose.y,r)),...world.zones.map(r=>rectangleDistance(t.pose.x,t.pose.y,r)));
  if(i+1<run.trace.length){
    const next=run.trace[i+1];assert.equal(next.stamp_sec-t.stamp_sec,.5);
    let integrated={...t.pose},violated=false;
    for(let j=0;j<10;j++){
      const substep=stepPose(integrated,t.command,.05);
      if(world.swept(integrated,substep,.65))violated=true;
      integrated=substep;
    }
    const integrationError=Math.hypot(integrated.x-next.pose.x,integrated.y-next.pose.y);
    maxIntegrationError=Math.max(maxIntegrationError,integrationError);
    assert.ok(integrationError<1e-8,'motion differs from the model output');
    assert.ok(Math.abs(integrated.heading-next.pose.heading)<1e-8);
    if(violated)collisions++;
    distance+=Math.hypot(t.pose.x-next.pose.x,t.pose.y-next.pose.y);
    if(t.command.reason==='neural_wait'&&next.command.vx>0)resumes++;
  }
}
assert.ok(holes>0&&episodes.size>0&&resumes>0);
assert.equal(collisions,0,'road departure, forbidden-zone entry or collision observed');
assert.equal(run.trace.at(-1).evaluation.violation_frames,0);
assert.ok(distance>.94*run.mission.length_m&&run.mission.checkpoints>=49);
const report={schema:'dangjin.neural_drive.validation.v1',source:input,complete:true,
  seed:run.sensor_faults.seed,profile:run.sensor_faults.profile,frames:run.trace.length,
  distance_m:distance,route_length_m:run.mission.length_m,simulation_seconds:run.trace.at(-1).stamp_sec-run.trace[0].stamp_sec,
  hole_frames:holes,dropout_frames:dropFrames,dropout_episodes:episodes.size,
  model_stop_frames:stopFrames,model_resumptions:resumes,model_wait_seconds:stopFrames*.5,
  swept_violations:collisions,emergency_stop_interventions:0,planner_calls:0,pure_pursuit_calls:0,
  route_center_rmse_m:Math.sqrt(sumSquare/run.trace.length),route_center_max_error_m:maxError,
  minimum_footprint_clearance_m:minClearance-.65,max_command_integration_error_m:maxIntegrationError,
  policy_sha256:run.policy.weights_sha256,road_sha256:run.perception.weights_sha256,
  encoder_sha256:run.policy.encoder_sha256,map_assistance:run.policy.map_assistance,goal_input:run.policy.goal_input,oncoming_car_enabled:false,
  scope:'One unseen-fault-seed synthetic RGB closed-loop lap; not real-robot validation'};
report.source_sha256={};
for(const path of ['robot-sim.js','site.json','world.json','world-geometry.js','../../tools/connectome_policy.py',
  '../../tools/train_connectome_policy.py','../../tools/dangjin_sim_server.py','../../tools/dangjin_sensor_faults.py',
  '../../tools/validate_dangjin_neural_drive.mjs']){
  report.source_sha256[path]=createHash('sha256').update(await readFile(new URL(path,root))).digest('hex');
}
if(output)await writeFile(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
