// Keep retry policy explicit: a zero command never advances the robot pose.
export function shouldReobserve(command,faults){
  return command.vx===0&&command.vy===0&&command.yaw_rate===0&&faults?.enabled===true&&
    ['detection_dropout','path_unavailable','fresh_mask_violation','path_low_confidence'].includes(command.reason);
}

export function newFaultSeed(previous){
  let seed;
  do{seed=crypto.getRandomValues(new Uint32Array(1))[0];}while(seed===previous);
  return seed;
}
