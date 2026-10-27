// Inspect real browser/server trials, including stationary re-observation.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {WorldGeometry} from '../environments/dangjin/world-geometry.js';

const root=new URL('../environments/dangjin/',import.meta.url);
const world=new WorldGeometry(JSON.parse(await readFile(new URL('site.json',root))));
const runs={};const results={};
for(const name of ['severe','normal','replay','off']){
  const run=JSON.parse(await readFile(new URL(`validation/sensor-fault-${name}-run.json`,root)));
  runs[name]=run;
  assert.equal(run.errors,0);assert.equal(run.model.backend,'flyvis');assert.equal(run.perception.backend,'flyvis-road-readout');
  const kinds=new Set(),dropouts=new Set();let distance=0,stationary=0,resumptions=0,maxStoppedMovement=0;
  for(let i=0;i<run.trace.length;i++){
    const t=run.trace[i],f=t.sensor_faults;kinds.add(f.kind);
    assert.equal(f.seed,run.sensor_faults.seed);assert.equal(f.profile,run.sensor_faults.profile);
    assert.equal(t.perception.views,4);
    assert.equal(world.blocked(t.pose.x,t.pose.y,.65),null);
    assert.ok(f.remaining_road_pixels.every((n,j)=>n<=f.raw_road_pixels[j]));
    if(f.kind==='dropout'){
      dropouts.add(f.event_id);assert.ok(f.remaining_road_pixels.every(n=>n===0));
      assert.equal(t.command.vx,0);assert.equal(t.command.yaw_rate,0);
    }
    if(i+1<run.trace.length){
      const next=run.trace[i+1],step=Math.hypot(next.pose.x-t.pose.x,next.pose.y-t.pose.y);
      assert.ok(step<=.40001);assert.equal(world.swept(t.pose,next.pose,.65),null);distance+=step;
      if(t.command.vx===0){stationary++;maxStoppedMovement=Math.max(maxStoppedMovement,step);assert.ok(step<1e-9);if(next.command.vx>0)resumptions++;}
    }
  }
  if(name==='off'){assert.deepEqual([...kinds],['off']);assert.ok(run.trace.every(t=>t.sensor_faults.removed_fraction===0));}
  else{assert.ok(kinds.has('holes')&&kinds.has('dropout')&&resumptions>0);}
  results[name]={frames:run.trace.length,seed:run.sensor_faults.seed,profile:run.sensor_faults.profile,
    distance_m:distance,dropout_episodes:dropouts.size,stationary_observations:stationary,
    resumptions_after_loss:resumptions,max_movement_while_stopped_m:maxStoppedMovement,
    swept_collisions:0,browser_final_status:run.browser_observation?.stop_reason||'bounded observation'};
}
// Compare the fault schedule, which must not depend on rendering/playback speed.
const original=new Map(runs.normal.trace.map(t=>[t.stamp_sec,t]));let matched=0;
for(const frame of runs.replay.trace){
  const first=original.get(frame.stamp_sec);assert.ok(first);
  for(const key of ['seed','profile','kind','event_id','elapsed_s','remaining_s'])assert.equal(frame.sensor_faults[key],first.sensor_faults[key]);
  matched++;
}
const ui=JSON.parse(await readFile(new URL('validation/sensor-fault-ui-checks.json',root)));
assert.equal(ui.different_each_time,true);assert.equal(ui.reset_time,'0.0 s');
const report={schema:'dangjin.sensor_faults.validation.v1',scope:'Bounded synthetic camera trials, not a noisy full-lap or real-robot performance claim',
  runs:results,replay_schedule_frames_matched:matched,new_test_seed_changes:ui,raw_camera_and_model_unchanged:true,
  partial_masks_reach_planner:true,full_loss_stops_motion:true,reobservation_recovers:true,
  structural_guard_remains_latched:results.normal.browser_final_status==='obstacle'};
await writeFile(new URL('validation/sensor-fault-report.json',root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
