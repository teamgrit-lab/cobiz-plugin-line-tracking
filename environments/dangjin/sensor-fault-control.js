// Keep retry policy explicit: a zero command never advances the robot pose.
export function shouldReobserve(command,faults){
  return command.vx===0&&command.vy===0&&command.yaw_rate===0&&faults?.enabled===true&&
    ['detection_dropout','path_unavailable','fresh_mask_violation','path_low_confidence'].includes(command.reason);
}

// Road retries keep only sensor time moving; the zero command stays zero.
export function roadWaitState(command,metrics={},faults={}){
  if(command.vx!==0||command.vy!==0||command.yaw_rate!==0||
     !['path_unavailable','turn_confirmation','road_reobserve'].includes(command.reason))return null;
  if(faults.kind==='dropout')return {reason:'detection_dropout',detail:'전체 검출 누락 · 새 영상에서 통로를 다시 확인합니다.'};
  if(metrics.recovery_status==='confirming')return {reason:'side_recovery_confirmation',detail:`${metrics.recovery_direction==='left'?'왼쪽':'오른쪽'} 차도 근거 ${metrics.recovery_confirmation_frames}/2 프레임 확인 중입니다.`};
  if(metrics.recovery_status==='timeout')return {reason:'side_recovery_timeout',detail:'10초 동안 경로를 재탐색했습니다. 새 시험 또는 작업을 시작해 다시 확인할 수 있습니다.'};
  if(command.reason==='turn_confirmation'){
    const direction=metrics.maneuver==='confirm_left'?'왼쪽':'오른쪽';
    return {reason:'turn_confirmation',detail:`${direction} 회전 확인 ${metrics.turn_confirmation_frames??0}/${metrics.turn_confirmation_target??3} · 현재 유지 가능한 진행 통로가 없어 정지합니다.`};
  }
  if(command.reason==='road_reobserve')return {reason:'road_reobserve',detail:'회전 후보 앞의 미관측 영역을 다시 확인합니다.'};
  const details={insufficient_side_road:'좌우 가까운 차도 근거가 부족합니다.',no_connected_corridor:'현재 검출에서 폭과 연결 조건을 만족하는 통로가 없습니다.',no_forward_samples:'전방 관측 범위가 부족합니다.'};
  return {reason:'path_unavailable',detail:details[metrics.path_status]||'현재 검출에서 유효한 진행 통로가 없습니다.'};
}

export function newFaultSeed(previous){
  let seed;
  do{seed=crypto.getRandomValues(new Uint32Array(1))[0];}while(seed===previous);
  return seed;
}
