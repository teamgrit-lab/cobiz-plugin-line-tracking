import test from 'node:test';
import assert from 'node:assert/strict';
import {OncomingCar} from '../environments/dangjin/oncoming-car.js';
import {rectangleDistance} from '../environments/dangjin/world-geometry.js';

test('passenger car approaches at 0.35 m/s on its side, independent of robot motion',()=>{
  const car=new OncomingCar([[0,0],[100,0]],{x:20,y:0,heading:0});
  assert.equal(car.pose.x,38);assert.equal(car.pose.y,1.8);assert.equal(car.pose.heading,Math.PI);
  const before=car.snapshot();car.update(10);
  assert.equal(car.pose.x,34.5);assert.equal(before.pose.x,38);
  assert.equal(car.footprint().length,4.3);assert.equal(car.footprint().width,1.8);
  // Time advances even while the robot waits for a fresh detection.
  car.update(.5);assert.ok(Math.abs(car.pose.x-34.325)<1e-10);
});
test('opposite robot heading reverses NPC direction and lane; no end teleport',()=>{
  const car=new OncomingCar([[0,0],[100,0]],{x:80,y:0,heading:Math.PI});
  assert.equal(car.pose.x,62);assert.equal(car.pose.y,-1.8);assert.equal(car.pose.heading,0);
  car.update(1000);assert.equal(car.pose.x,100);
});
test('collision footprint rotates with vehicle rather than using a square box',()=>{
  const r={x:-2.15,y:-.9,length:4.3,width:1.8,heading:Math.PI/2};
  assert.equal(rectangleDistance(0,2,r),0);assert.ok(Math.abs(rectangleDistance(2,0,r)-1.1)<1e-10);
});
