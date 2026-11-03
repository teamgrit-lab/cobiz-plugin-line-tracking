import test from 'node:test';
import assert from 'node:assert/strict';
import {newFaultSeed,shouldReobserve,roadWaitState} from '../environments/dangjin/sensor-fault-control.js';

test('only perception-loss zero commands are retried; safety and user stops remain latched',()=>{
  const command=reason=>({vx:0,vy:0,yaw_rate:0,reason});
  for(const reason of ['detection_dropout','path_unavailable','fresh_mask_violation'])assert.ok(shouldReobserve(command(reason),{enabled:true}));
  for(const reason of ['no_go_zone','obstacle','road_edge','planner_error','paused','lap_complete'])assert.ok(!shouldReobserve(command(reason),{enabled:true}));
  assert.ok(!shouldReobserve(command('path_unavailable'),{enabled:false}));
  assert.ok(!shouldReobserve({...command('detection_dropout'),vx:.1},{enabled:true}));
  assert.ok(!shouldReobserve({...command('detection_dropout'),yaw_rate:.1},{enabled:true}));
});
test('a new test gets a different uint32 seed',()=>{
  let seed=0;
  for(let i=0;i<20;i++){const next=newFaultSeed(seed);assert.notEqual(seed,next);assert.ok(Number.isInteger(next)&&next>=0&&next<2**32);seed=next;}
});
test('Road confirmation, missing road and dropout have distinct waiting states',()=>{
  const command=reason=>({vx:0,vy:0,yaw_rate:0,reason});
  const pending=roadWaitState(command('turn_confirmation'),{maneuver:'confirm_left',turn_confirmation_frames:2,turn_confirmation_target:3},{enabled:false});
  assert.equal(pending.reason,'turn_confirmation');assert.match(pending.detail,/2\/3/);
  assert.equal(roadWaitState(command('path_unavailable'),{path_status:'no_connected_corridor'}).reason,'path_unavailable');
  assert.equal(roadWaitState(command('path_unavailable'),{}, {kind:'dropout'}).reason,'detection_dropout');
  assert.equal(roadWaitState(command('road_reobserve')).reason,'road_reobserve');
  assert.equal(roadWaitState(command('path_unavailable'),{recovery_status:'confirming',recovery_direction:'left',recovery_confirmation_frames:1}).reason,'side_recovery_confirmation');
  assert.equal(roadWaitState(command('path_unavailable'),{recovery_status:'timeout'}).reason,'side_recovery_timeout');
  assert.equal(roadWaitState({vx:0,vy:0,yaw_rate:.18,reason:'road_side_recovery'}),null);
  assert.equal(roadWaitState({...command('turn_confirmation'),vx:.4}),null);
  for(const reason of ['paused','lap_complete','road_edge','obstacle','planner_error'])assert.equal(roadWaitState(command(reason)),null);
});
