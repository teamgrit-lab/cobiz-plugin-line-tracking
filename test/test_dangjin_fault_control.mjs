import test from 'node:test';
import assert from 'node:assert/strict';
import {newFaultSeed,shouldReobserve} from '../environments/dangjin/sensor-fault-control.js';

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
