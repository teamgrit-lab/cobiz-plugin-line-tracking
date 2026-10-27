// Replay the environment actor and integrate logged model commands, never drive.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {WorldGeometry,rectangleDistance,stepPose} from '../environments/dangjin/world-geometry.js';
import {OncomingCar} from '../environments/dangjin/oncoming-car.js';
const [input,output]=process.argv.slice(2),run=JSON.parse(await readFile(input));
const world=new WorldGeometry(JSON.parse(await readFile(new URL('../environments/dangjin/site.json',import.meta.url))));
const car=new OncomingCar(world.perimeterRoute(),run.trace[0].pose,{loop:true});
let minClearance=Infinity,waitUpdates=0,violations=0,passed=false;
assert.equal(run.policy.map_assistance,false);assert.equal(run.policy.goal_input,false);assert.equal(run.errors,0);
const forward=(p,q)=>(q.x-p.x)*Math.cos(p.heading)+(q.y-p.y)*Math.sin(p.heading);
assert.ok(forward(run.trace[0].pose,car.pose)>10);
for(let i=0;i<run.trace.length;i++){
  const t=run.trace[i],actual=t.environment.oncoming_car;
  assert.ok(Math.hypot(actual.pose.x-car.pose.x,actual.pose.y-car.pose.y)<1e-8);
  assert.equal(t.camera_yaw,0);assert.ok(t.perception.circuit_ms>0);assert.equal(t.policy.override,false);
  assert.deepEqual(t.command,t.policy.model_command);assert.ok(!('target_site' in t.mission));
  assert.ok(Math.abs(actual.elapsed_s-i*.5)<1e-8);
  passed ||= forward(t.pose,actual.pose)<-5;
  if(i===run.trace.length-1)break;
  let robot={...t.pose};
  for(let j=0;j<10;j++){
    car.update(.05);world.extraObstacles=[car.footprint()];
    const next=stepPose(robot,t.command,.05);
    if(world.swept(robot,next,.65))violations++;
    minClearance=Math.min(minClearance,rectangleDistance(next.x,next.y,car.footprint())-.65);
    robot=next;
  }
  const next=run.trace[i+1].pose;
  assert.ok(Math.hypot(next.x-robot.x,next.y-robot.y)<1e-8);
  if(t.command.reason==='neural_wait')waitUpdates++;
}
assert.ok(passed);assert.ok(waitUpdates>0);assert.equal(violations,0);assert.ok(minClearance>0);
assert.equal(run.trace.at(-1).evaluation.violation_frames,0);
const report={schema:'dangjin.oncoming.validation.v1',source:input,frames:run.trace.length,
  duration_s:run.trace.at(-1).stamp_sec-run.trace[0].stamp_sec,car_speed_mps:car.speed,car_size_m:[4.3,1.8],
  passed,minimum_robot_to_car_clearance_m:minClearance,swept_violations:violations,
  car_advanced_during_model_wait_frames:waitUpdates,seed:run.sensor_faults.seed,profile:run.sensor_faults.profile,
  policy_sha256:run.policy.weights_sha256,map_assistance:false,
  scope:'One synthetic low-speed oncoming encounter; not all obstacle layouts or real-robot avoidance'};
if(output)await writeFile(output,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
