// Negotiate the native model resolution before allocating capture buffers.
export function fcnCameraFormat(status){
  const {model,perception}=status;
  if(!['fcn512-360','fcn1024-360'].includes(model?.backend))return null;
  const width=perception?.input_width??512,height=perception?.input_height??256;
  if(![512,1024].includes(width)||height!==width/2||model.backend!==`fcn${width}-360`||
     perception.backend!==model.backend||model.bundle_sha256!==perception.weights_sha256||
     (model.input_width!==undefined&&model.input_width!==width)||
     (model.input_height!==undefined&&model.input_height!==height))throw Error('FCN 모델과 입력 해상도가 일치하지 않습니다.');
  const grid=[4,height/32,width/32];
  if(perception.class_grid_shape!==undefined&&JSON.stringify(perception.class_grid_shape)!==JSON.stringify(grid))
    throw Error('FCN 원래 검출 격자의 크기가 일치하지 않습니다.');
  if(status.camera_transport!==`fcn-atlas-${width}x${height}`)throw Error('카메라 입력이 변경되었습니다. 서버를 다시 시작하세요.');
  return {width,height,grid};
}
