// Audit browser camera -> actual model -> uncorrected kinematic motion.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {WorldGeometry,stepPose,rectangleDistance} from '../environments/dangjin/world-geometry.js';

const [input,output]=process.argv.slice(2);
if(!input)throw Error('Usage: node tools/validate_front_drive.mjs run.json report.json');
const root=new URL('../environments/dangjin/',import.meta.url);
const run=JSON.parse(await readFile(input));
const world=new WorldGeometry(JSON.parse(await readFile(new URL('site.json',root))));
assert.equal(run.layout_revision,world.site.layout_revision);
assert.equal(run.policy.backend,'flyvis-front-drive');
for(const key of ['map_assistance','goal_input','pose_input','runtime_planner','runtime_pure_pursuit','fault_flag_input','emergency_stop_override'])assert.equal(run.policy[key],false,key);
assert.equal(run.perception.backend,'flyvis-front-road');assert.equal(run.perception.uses_ground_truth_at_runtime,false);
assert.ok(run.trace.length>1);
if(run.sensor_faults.enabled)assert.ok(run.sensor_faults.seed<1000||run.sensor_faults.seed>=3000,'unseen evaluation fault seed required');
let distance=0,stops=0,resumes=0,violations=0,holes=0,drops=0,maxIntegrationError=0,sumSquare=0,maxError=0,minClearance=Infinity;
const episodes=new Set();let firstViolation=null;
for(let i=0;i<run.trace.length;i++){
  const t=run.trace[i],f=t.sensor_faults;
  assert.equal(t.perception.views,1);assert.ok(t.perception.circuit_ms>0&&t.policy.ms>0);
  assert.equal(t.camera_yaw,0);assert.equal(t.surface_ms,0);assert.equal(t.flow_ms,0);assert.equal(t.policy.override,false);
  if(t.command.reason!=='lap_complete')assert.deepEqual(t.command,t.policy.model_command,'model command overwritten');
  assert.ok(t.command.vx>=0&&t.command.vx<=.8&&Math.abs(t.command.yaw_rate)<=.35);
  if(f.kind==='holes')holes++;
  if(f.kind==='dropout'){drops++;episodes.add(f.event_id);assert.deepEqual(f.remaining_road_pixels,[0]);}
  if(t.command.reason==='neural_wait'){stops++;assert.ok(t.policy.stop_probability>=.5);}
  if(t.mission){sumSquare+=t.mission.cross_track_m**2;maxError=Math.max(maxError,t.mission.cross_track_m);}
  minClearance=Math.min(minClearance,world.nearestRoad(t.pose.x,t.pose.y).clearance,
    ...world.obstacles.map(r=>rectangleDistance(t.pose.x,t.pose.y,r)),...world.zones.map(r=>rectangleDistance(t.pose.x,t.pose.y,r)));
  if(i+1<run.trace.length){
    const next=run.trace[i+1],dt=t.environment?.simulation_step_sec??.5;
    assert.ok([.1,.5].includes(dt)&&Math.abs(next.stamp_sec-t.stamp_sec-dt)<1e-8);
    const substeps=Math.round(dt/.05);
    let integrated={...t.pose},violation=null;
    for(let j=0;j<substeps;j++){
      const step=stepPose(integrated,t.command,.05);
      violation=violation||world.swept(integrated,step,.65);integrated=step;
    }
    const error=Math.hypot(integrated.x-next.pose.x,integrated.y-next.pose.y);
    maxIntegrationError=Math.max(maxIntegrationError,error);
    assert.ok(error<1e-8&&Math.abs(integrated.heading-next.pose.heading)<1e-8,'position correction detected');
    if(violation){violations++;firstViolation||={frame:i,reason:violation,pose:t.pose};}
    distance+=Math.hypot(t.pose.x-next.pose.x,t.pose.y-next.pose.y);
    if(t.command.reason==='neural_wait'&&next.command.vx>0)resumes++;
  }
}
const complete=run.mission?.complete===true;
const faultRecovery=!run.sensor_faults.enabled||(holes>0&&episodes.size>0&&resumes>0);
const coverage=complete&&distance>.94*run.mission.length_m&&run.mission.checkpoints>=49&&run.trace.at(-1).command.reason==='lap_complete';
const passed=coverage&&violations===0&&run.errors===0&&faultRecovery&&run.trace.at(-1).evaluation.violation_frames===0;
const report={schema:'connectum.front.closed_loop.v1',passed,complete,source:input,
  seed:run.sensor_faults.seed,profile:run.sensor_faults.profile,faults_enabled:run.sensor_faults.enabled,
  frames:run.trace.length,distance_m:distance,route_length_m:run.mission?.length_m,
  progress_fraction:run.mission?.fraction,simulation_seconds:run.trace.at(-1).stamp_sec-run.trace[0].stamp_sec,
  hole_frames:holes,dropout_frames:drops,dropout_episodes:episodes.size,model_stop_frames:stops,model_resumptions:resumes,
  swept_violation_frames:violations,first_violation:firstViolation,errors:run.errors,
  route_center_rmse_m:Math.sqrt(sumSquare/run.trace.length),route_center_max_error_m:maxError,
  minimum_footprint_clearance_m:minClearance-.65,max_command_integration_error_m:maxIntegrationError,
  model_sha256:run.policy.weights_sha256,encoder_sha256:run.policy.encoder_sha256,
  camera_sway:run.trace[0].environment?.camera_sway??{enabled:false},
  camera_pitch_range_deg:[Math.min(...run.trace.map(t=>t.environment?.camera_sway?.pitch_deg??0)),Math.max(...run.trace.map(t=>t.environment?.camera_sway?.pitch_deg??0))],
  input:'one rendered front RGB image; optional walking pitch and detection holes/dropout',
  control:'actual learned speed/yaw; no map, goal, pose or planner input; episode end only',
  scope:'Same-site synthetic browser RGB and kinematics. Not quadruped dynamics, new-site or real-robot certification.'};
report.source_sha256={};
for(const path of ['robot-sim.js','camera-sway.js','site.json','world-geometry.js','../../tools/front_connectome.py',
  '../../tools/dangjin_sim_server.py','../../tools/dangjin_sensor_faults.py','../../tools/validate_front_drive.mjs']){
  report.source_sha256[path]=createHash('sha256').update(await readFile(new URL(path,root))).digest('hex');
}
if(output)await writeFile(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
if(!passed)process.exitCode=1;
