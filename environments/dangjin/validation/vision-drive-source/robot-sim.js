import * as THREE from 'three';
import {localToSite,stepPose} from './world-geometry.js';
import {OncomingCar} from './oncoming-car.js';
import {newFaultSeed,shouldReobserve} from './sensor-fault-control.js';

const $=id=>document.getElementById(id);
const W=480,H=270,DT=.5;
const zero={vx:0,vy:0,yaw_rate:0};
const reasonText={lap_complete:'우사 한 바퀴 완료 · 자동 정지',collecting:'인식 학습 표본 수집 중',no_go_zone:'중앙도로 통행 금지',obstacle:'구조물·장비 충돌 방지',road_edge:'도로 경계 도달',path_unavailable:'관측된 중앙 경로 없음',path_low_confidence:'경로 관측 범위 부족',planner_error:'추론 오류',tracking:'도로 중앙 추종 중',tracking_slow_turn:'커브 감속 중',paused:'일시 정지',idle:'주행 준비',backend:'실제 Flyvis 서버 연결 필요'};
Object.assign(reasonText,{neural_drive:'커넥톰이 속도·조향 결정 중',neural_wait:'커넥톰이 정지 결정 · 다음 관측 대기',detection_dropout:'검출 전체 누락 · 정지 후 재검출',reobserving:'경로 재검출 중 · 정지 유지'});

function canvas(width,height){const c=document.createElement('canvas');c.width=width;c.height=height;return c;}
function rgbaCanvas(bytes,c){const ctx=c.getContext('2d');const image=ctx.createImageData(c.width,c.height);const stride=c.width*4;for(let y=0;y<c.height;y++)image.data.set(bytes.subarray((c.height-1-y)*stride,(c.height-y)*stride),y*stride);ctx.putImageData(image,0,0);}
const png=c=>c.toDataURL('image/png').split(',')[1];

export class RobotSimulation {
  constructor({scene,renderer,camera,controls,model,roofs,vegetation,world,onOpen}){
    Object.assign(this,{scene,renderer,camera,controls,model,roofs,vegetation,world,onOpen});
    this.active=false;this.running=false;this.busy=false;this.generation=0;this.sequence=0;this.stamp=1;this.next=0;this.distance=0;this.samples=[];this.events=[];this.session=crypto.randomUUID();this.missionKey=crypto.randomUUID();this.scenario='patrol';this.viewYaw=0;this.mission=null;this.navTarget=null;this.pose={...world.site.simulation.start};this.command={...zero};this.lastStatus='idle';
    this.group=new THREE.Group();this.visuals=new THREE.Group();scene.add(this.group,this.visuals);this.group.visible=false;this.visuals.visible=false;
    this.makeRobot();
    this.cubeTarget=new THREE.WebGLCubeRenderTarget(256,{generateMipmaps:false,minFilter:THREE.LinearFilter});
    this.cubeCamera=new THREE.CubeCamera(.05,650,this.cubeTarget);scene.add(this.cubeCamera);
    this.frontCamera=new THREE.PerspectiveCamera(90,W/H,.05,650);
    this.frontTarget=new THREE.WebGLRenderTarget(W,H,{depthBuffer:true});this.panoTarget=new THREE.WebGLRenderTarget(768,384,{depthBuffer:false});
    this.frontTarget.texture.colorSpace=THREE.SRGBColorSpace;this.panoTarget.texture.colorSpace=THREE.SRGBColorSpace;
    this.quadScene=new THREE.Scene();this.quadCamera=new THREE.OrthographicCamera(-1,1,1,-1,0,2);this.quadCamera.position.z=1;
    this.shader=new THREE.ShaderMaterial({uniforms:{environment:{value:this.cubeTarget.texture},panorama:{value:0},basis:{value:new THREE.Matrix3()},aspect:{value:W/H}},
      vertexShader:'varying vec2 vUv; void main(){vUv=uv;gl_Position=vec4(position.xy,0.,1.);}',
      fragmentShader:`precision highp float; varying vec2 vUv; uniform samplerCube environment; uniform int panorama; uniform mat3 basis; uniform float aspect;
        void main(){vec3 d;if(panorama==1){float lon=(vUv.x-.5)*6.283185307;float lat=(vUv.y-.5)*3.141592654;d=vec3(sin(lon)*cos(lat),sin(lat),-cos(lon)*cos(lat));}else{d=normalize(vec3((vUv.x*2.-1.)*aspect,vUv.y*2.-1.,-1.));}
        gl_FragColor=textureCube(environment,basis*d);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
        }`});
    this.quadScene.add(new THREE.Mesh(new THREE.PlaneGeometry(2,2),this.shader));
    this.rgb=canvas(W,H);this.labels=canvas(W,H);this.hazard=canvas(W,H);this.rawBytes=new Uint8Array(W*H*4);this.panoBytes=new Uint8Array(768*384*4);
    this.views=[this.rgb,...Array.from({length:3},()=>canvas(W,H))];
    this.black=new THREE.MeshBasicMaterial({color:0,side:THREE.DoubleSide});this.white=new THREE.MeshBasicMaterial({color:0xffffff,side:THREE.DoubleSide});this.red=new THREE.MeshBasicMaterial({color:0xff0000,side:THREE.DoubleSide});
    this.pathLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:'#66e1c5',depthTest:false}));this.pathLine.renderOrder=10;this.visuals.add(this.pathLine);
    this.trailLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:'#d38b42'}));this.visuals.add(this.trailLine);this.trail=[];
    this.bind();this.reset('patrol');
  }
  makeRobot(){
    const steel=new THREE.MeshStandardMaterial({color:'#283f40',roughness:.6}),orange=new THREE.MeshStandardMaterial({color:'#ddaa58',roughness:.5});
    const add=(geometry,material,x,y,z)=>{const mesh=new THREE.Mesh(geometry,material);mesh.position.set(x,y,z);mesh.castShadow=true;this.group.add(mesh);return mesh;};
    add(new THREE.BoxGeometry(1.1,.27,.52),steel,0,.62,0);add(new THREE.BoxGeometry(.85,.06,.44),orange,0,.79,0);
    this.legs=[];for(const x of [-.39,.39])for(const z of [-.26,.26]){const leg=add(new THREE.BoxGeometry(.12,.5,.12),steel,x,.3,z);this.legs.push(leg);add(new THREE.BoxGeometry(.21,.1,.16),steel,x,.08,z);}
    add(new THREE.CylinderGeometry(.035,.035,.72,8),steel,0,1.15,0);
    add(new THREE.SphereGeometry(.14,12,8),orange,0,1.6,0);
    add(new THREE.SphereGeometry(.065,10,8),steel,.12,1.6,0);add(new THREE.SphereGeometry(.065,10,8),steel,-.12,1.6,0);
  }
  bind(){
    $('sim-panel-toggle').onclick=()=>{
      const collapsed=$('sim-panel-toggle').getAttribute('aria-expanded')==='true';
      $('sim-panel-toggle').setAttribute('aria-expanded',String(!collapsed));
      $('sim-panel-toggle').textContent=collapsed?'로봇 화면 펼치기 ▴':'로봇 화면 접기 ▾';
      $('robot-panel-body').hidden=collapsed;$('robot-panel').classList.toggle('collapsed',collapsed);
    };
    $('robot-mode').onclick=()=>this.open();$('sim-start').onclick=()=>this.running?this.pause():this.start();
    $('sim-reset').onclick=()=>this.reset($('sim-scenario').value);
    $('sim-scenario').onchange=()=>this.reset($('sim-scenario').value);
    for(const id of ['sim-fault-enabled','sim-fault-profile'])$(id).onchange=()=>this.reset($('sim-scenario').value);
    $('sim-fault-replay').onclick=()=>{const input=$('sim-fault-seed');if(input.value!==''&&input.reportValidity())this.reset($('sim-scenario').value,Number(input.value));};
    $('sim-nudge').onclick=()=>{this.pause();const point=localToSite([0,.6],this.pose);const next={...this.pose,x:point[0],y:point[1]};if(!this.world.swept(this.pose,next,.65)){this.pose=next;this.generation++;this.events.push({t:this.stamp,pose:{...next},reason:'manual_offset'});this.place();this.capture();}};
    $('sim-collect').onclick=()=>this.collect();
    $('sim-policy-collect').onclick=()=>this.collectPolicy();
    $('sim-obstacle').onchange=e=>this.placeObstacle(e.target.checked);
    $('sim-export').onclick=()=>{const blob=new Blob([JSON.stringify({schema:'dangjin.simulation.trace.v1',model:this.health?.model,perception:this.health?.perception,policy:this.health?.policy,evaluation:{violations:this.violations,emergency_stop_override:!this.health?.policy},mission:this.mission,physics:'kinematic',sensor_faults:{...this.faultConfig,schema:'dangjin.sensor_faults.v1'},wait_seconds:this.waitSeconds,inference_count:this.inferenceCount,samples:this.samples,events:this.events},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download='dangjin-simulation-trace.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
    document.addEventListener('visibilitychange',()=>{if(document.hidden&&this.running)this.pause();});
  }
  open(){this.onOpen();document.body.classList.add('robot-active');this.active=true;this.group.visible=true;this.visuals.visible=true;$('robot-panel').hidden=false;$('sim-controls').hidden=false;$('photo-card').hidden=true;$('model-mode').setAttribute('aria-pressed','false');$('robot-mode').setAttribute('aria-pressed','true');this.place();this.capture();this.follow(true);this.checkBackend();}
  close(){this.pause();document.body.classList.remove('robot-active');this.active=false;this.group.visible=false;this.visuals.visible=false;$('robot-panel').hidden=true;$('sim-controls').hidden=true;$('robot-mode').setAttribute('aria-pressed','false');}
  status(reason,detail=''){this.lastStatus=reason;$('sim-status').textContent=reasonText[reason]||reason;$('sim-detail').textContent=detail;$('sim-status').dataset.reason=reason;}
  async checkBackend(){
    try{const response=await fetch('/api/status',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error('서버 미연결');const value=await response.json();if(value.model?.backend!=='flyvis'||!value.model.flow_decoder_loaded||!value.perception||value.perception.uses_ground_truth_at_runtime)throw Error('학습된 회로 필요');if(value.layout_revision!==this.world.site.layout_revision)throw Error('지도 변경 후 서버를 다시 시작하세요.');if(value.sensor_faults_version!=='dangjin.sensor_faults.v1')throw Error('검출 누락 시험 서버를 다시 시작하세요.');this.health=value;this.fixedCamera=!!value.policy;$('sim-backend').textContent=value.policy?'Flyvis · 학습된 속도·조향':value.perception.backend==='flyvis-road-readout'?`Flyvis ${value.model.version} · 360° 차도 검출`:`RGB SegFormer + Flyvis ${value.model.version}`;$('sim-start').disabled=false;return true;}
    catch{this.health=null;$('sim-backend').textContent='시뮬레이션 서버: localhost:8766';$('sim-start').disabled=true;this.status('backend','README의 시뮬레이션 실행 명령을 사용하세요.');return false;}
  }
  async start(){if(!this.active)this.open();if(this.lastStatus==='lap_complete')this.reset(this.scenario);const generation=this.generation;if(!await this.checkBackend()||generation!==this.generation)return;this.running=true;this.next=0;$('sim-start').textContent='Ⅱ 일시 정지';this.status('tracking');}
  pause(reason='paused',detail=''){if(this.active)this.events.push({t:this.stamp,pose:{...this.pose},reason,detail});this.running=false;this.command={...zero};this.generation++;$('sim-start').textContent='▶ 추종 시작';this.status(reason,detail);this.updateMetrics();}
  reset(scenario,seed){
    this.pause('idle');this.scenario=scenario;this.missionKey=crypto.randomUUID();this.mission=null;this.navTarget=null;this.viewYaw=0;this.stamp=1;this.distance=0;this.samples=[];this.events=[];this.violations=[];this.trail=[];this.sequence=0;this.pathLine.geometry.setFromPoints([]);this.trailLine.geometry.setFromPoints([]);
    this.faultConfig={enabled:$('sim-fault-enabled').checked,profile:$('sim-fault-profile').value,seed:seed??newFaultSeed(this.faultConfig?.seed)};
    $('sim-fault-seed').value=String(this.faultConfig.seed);this.lastFault=null;this.faultDropouts=new Set();this.waitSeconds=0;this.inferenceCount=0;
    $('sim-fault-live').dataset.kind='clear';$('sim-fault-live').textContent=this.faultConfig.enabled?'랜덤 검출 누락 시험 · 새 패턴 준비':'검출 누락 효과 꺼짐';
    $('sim-mask').nextElementSibling.textContent=this.faultConfig.enabled?'누락 적용 후 검출 · 행동 모델 입력':'360° 차도 검출 · 초록: 통행 추정 / 회색: 미관측';
    $('sim-mask').getContext('2d').clearRect(0,0,$('sim-mask').width,$('sim-mask').height);
    this.oncomingCar=null;this.world.extraObstacles=[];if(this.equipment){this.visuals.remove(this.equipment);this.equipment=null;}$('sim-obstacle').checked=false;
    const names={bend:['compost-outer',.30],east:['south-east',.92],west:['west-north',.13]};
    if(scenario==='patrol'){const points=this.world.perimeterRoute(),p=points[0],q=points[12];this.pose={x:p[0],y:p[1],heading:Math.atan2(q[1]-p[1],q[0]-p[0])};this.navTarget=null;}
    else if(scenario==='blocked')this.pose={x:145.65,y:184,heading:-Math.PI/2};
    else{const [name,t]=names[scenario]||names.bend,road=this.world.roads.find(r=>r.id===name),p=road.curve.getPointAt(t),v=road.curve.getTangentAt(t);this.pose={x:p.x,y:-p.z,heading:Math.atan2(-v.z,v.x)};}
    this.place();this.updateMetrics();if(this.active){this.capture();this.follow(true);}
  }
  placeObstacle(enabled){
    this.pause();this.oncomingCar=null;this.world.extraObstacles=[];
    if(this.equipment){this.visuals.remove(this.equipment);this.equipment=null;}
    if(enabled){
      const road=this.world.roads.find(r=>r.id===this.world.nearestRoad(this.pose.x,this.pose.y).id);
      this.oncomingCar=new OncomingCar(this.scenario==='patrol'?this.world.perimeterRoute():road.samples,this.pose,{loop:this.scenario==='patrol'});
      const g=new THREE.Group();g.userData.testObstacle=true;
      const paint=new THREE.MeshStandardMaterial({color:'#b9d6e5',metalness:.35,roughness:.4});
      const glass=new THREE.MeshStandardMaterial({color:'#233f51',roughness:.2});
      const box=(size,position,material)=>{const mesh=new THREE.Mesh(new THREE.BoxGeometry(...size),material);mesh.position.set(...position);mesh.userData.semantic='hazard';g.add(mesh);};
      box([4.3,.68,1.8],[0,.68,0],paint);box([2.25,.6,1.65],[-.2,1.28,0],glass);box([2.05,.08,1.6],[-.2,1.61,0],paint);
      const tyre=new THREE.MeshStandardMaterial({color:'#182124'});
      for(const x of [-1.35,1.35])for(const z of [-.88,.88]){const wheel=new THREE.Mesh(new THREE.CylinderGeometry(.32,.32,.18,16),tyre);wheel.rotation.x=Math.PI/2;wheel.position.set(x,.34,z);wheel.userData.semantic='hazard';g.add(wheel);}
      const headlight=new THREE.MeshStandardMaterial({color:'#ffedb0',emissive:'#ffc96f',emissiveIntensity:.65});
      const tail=new THREE.MeshStandardMaterial({color:'#ad2424',emissive:'#8c0909',emissiveIntensity:.35});
      for(const z of [-.6,.6]){box([.03,.16,.34],[2.16,.85,z],headlight);box([.03,.16,.34],[-2.16,.85,z],tail);}
      this.visuals.add(g);this.equipment=g;this.advanceObstacle(0);
    }
    if(this.active)this.capture();
  }
  advanceObstacle(dt){
    if(!this.oncomingCar)return;
    const p=this.oncomingCar.update(dt);this.equipment.position.set(p.x,0,-p.y);this.equipment.rotation.y=p.heading;
    this.world.extraObstacles=[this.oncomingCar.footprint()];
  }
  place(){this.group.position.set(this.pose.x,0,-this.pose.y);this.group.rotation.y=this.pose.heading;}
  follow(instant=false){if(!$('sim-follow').checked)return;const p=localToSite([-7,0],this.pose),behind=localToSite([3,0],this.pose),target=new THREE.Vector3(behind[0],1,-behind[1]),eye=new THREE.Vector3(p[0],3.4,-p[1]);if(instant){this.camera.position.copy(eye);this.controls.target.copy(target);}else{this.camera.position.lerp(eye,.1);this.controls.target.lerp(target,.1);}}
  capture(truth=false){
    const {renderer,scene}=this;const saved={target:renderer.getRenderTarget(),tone:renderer.toneMapping,color:renderer.outputColorSpace,bg:scene.background,fog:scene.fog,model:this.model.visible,roof:this.roofs.visible,veg:this.vegetation.visible,robot:this.group.visible,visuals:this.visuals.visible,shadow:renderer.shadowMap.autoUpdate};
    const x=this.pose.x,y=this.pose.y,h=this.world.site.simulation.camera_height_m;
    this.viewYaw=this.fixedCamera?0:this.navTarget?Math.max(-1.25,Math.min(1.25,Math.atan2(Math.sin(Math.atan2(this.navTarget[1]-y,this.navTarget[0]-x)-this.pose.heading),Math.cos(Math.atan2(this.navTarget[1]-y,this.navTarget[0]-x)-this.pose.heading)))):0;
    this.cameraPose={...this.pose,heading:this.pose.heading+this.viewYaw};
    this.frontCamera.position.set(x,h,-y);const f=localToSite([10,0],this.cameraPose);this.frontCamera.lookAt(f[0],h,-f[1]);this.frontCamera.updateMatrixWorld();
    this.cubeCamera.position.copy(this.frontCamera.position);
    this.model.visible=true;this.roofs.visible=true;this.vegetation.visible=true;this.group.visible=false;
    // Only planned lines/trail are hidden. A test obstruction is physical evidence.
    this.pathLine.visible=false;this.trailLine.visible=false;this.visuals.visible=true;
    const annotations=[];scene.traverse(obj=>{if(obj.userData.sensorHidden){annotations.push([obj,obj.visible]);obj.visible=false;}});
    try{
      scene.background=new THREE.Color('#c9d9df');scene.fog=new THREE.Fog('#c9d9df',250,600);renderer.shadowMap.autoUpdate=false;
      this.cubeCamera.update(renderer,scene);
      for(let sector=0;sector<(truth?1:4);sector++){
        const aim=localToSite([10,0],{...this.cameraPose,heading:this.cameraPose.heading+sector*Math.PI/2});
        this.frontCamera.lookAt(aim[0],h,-aim[1]);this.frontCamera.updateMatrixWorld();
        this.shader.uniforms.basis.value.setFromMatrix4(this.frontCamera.matrixWorld);
        this.shader.uniforms.panorama.value=0;renderer.setRenderTarget(this.frontTarget);renderer.render(this.quadScene,this.quadCamera);renderer.readRenderTargetPixels(this.frontTarget,0,0,W,H,this.rawBytes);rgbaCanvas(this.rawBytes,this.views[sector]);
      }
      this.frontCamera.lookAt(f[0],h,-f[1]);this.frontCamera.updateMatrixWorld();
      this.shader.uniforms.basis.value.setFromMatrix4(this.frontCamera.matrixWorld);
      this.shader.uniforms.panorama.value=1;renderer.setRenderTarget(this.panoTarget);renderer.render(this.quadScene,this.quadCamera);renderer.readRenderTargetPixels(this.panoTarget,0,0,768,384,this.panoBytes);rgbaCanvas(this.panoBytes,$('sim-panorama'));
      if(truth){
      const restore=[];scene.background=new THREE.Color(0);scene.fog=null;renderer.toneMapping=THREE.NoToneMapping;renderer.outputColorSpace=THREE.LinearSRGBColorSpace;
      scene.traverse(obj=>{
        if(obj.isMesh){restore.push([obj,obj.material,obj.visible]);if(obj.userData.semanticOnly)obj.visible=true;obj.material=obj.userData.semantic==='road'?this.white:obj.userData.semantic==='hazard'?this.red:this.black;}
        else if(obj.isLine||obj.isPoints||obj.isSprite){restore.push([obj,null,obj.visible]);obj.visible=false;}
      });
      try{renderer.setRenderTarget(this.frontTarget);renderer.render(scene,this.frontCamera);renderer.readRenderTargetPixels(this.frontTarget,0,0,W,H,this.rawBytes);}
      finally{for(const [obj,material,visible] of restore){if(material)obj.material=material;obj.visible=visible;}}
      const labelImage=this.labels.getContext('2d').createImageData(W,H),hazardImage=this.hazard.getContext('2d').createImageData(W,H);
      const preview=$('sim-mask').getContext('2d'),previewImage=preview.createImageData(W,H);
      this.maskRoad=new Uint8Array(W*H);this.maskHazard=new Uint8Array(W*H);
      for(let i=0;i<W*H;i++){
        const src=((H-1-Math.floor(i/W))*W+i%W)*4,r=this.rawBytes[src],g=this.rawBytes[src+1];const road=r>180&&g>180?1:0,hazard=r>180&&g<80?1:0;this.maskRoad[i]=road;this.maskHazard[i]=hazard;
        for(let ch=0;ch<3;ch++){labelImage.data[i*4+ch]=road;hazardImage.data[i*4+ch]=hazard;}
        labelImage.data[i*4+3]=hazardImage.data[i*4+3]=255;
        previewImage.data.set(road?[94,167,140,255]:hazard?[209,105,73,255]:[27,48,50,255],i*4);
      }
      this.labels.getContext('2d').putImageData(labelImage,0,0);this.hazard.getContext('2d').putImageData(hazardImage,0,0);preview.putImageData(previewImage,0,0);
      }
      $('sim-forward').getContext('2d').drawImage(this.rgb,0,0);
      this.localConfig=this.calibration();
      return truth?{image_png:png(this.rgb),selected_png:png(this.labels)}:{image_png:png(this.rgb),views_png:this.views.map(png)};
    }finally{
      for(const [obj,visible] of annotations)obj.visible=visible;
      renderer.setRenderTarget(saved.target);renderer.toneMapping=saved.tone;renderer.outputColorSpace=saved.color;renderer.shadowMap.autoUpdate=saved.shadow;scene.background=saved.bg;scene.fog=saved.fog;this.model.visible=saved.model;this.roofs.visible=saved.roof;this.vegetation.visible=saved.veg;this.group.visible=saved.robot;this.visuals.visible=saved.visuals;this.pathLine.visible=true;this.trailLine.visible=true;
    }
  }
  pixel(point){const p=localToSite(point,this.cameraPose||this.pose),v=new THREE.Vector3(p[0],.08,-p[1]).project(this.frontCamera);return [(v.x*.5+.5)*W-.5,(-v.y*.5+.5)*H-.5];}
  calibration(){
    const near=2.5,far=10,half=4.2;
    // Intrinsics in the robot frame stay identical across movement; avoid
    // floating-point world projections changing the server geometry signature.
    const f=H/2,height=this.world.site.simulation.camera_height_m-.08;
    const roi=[[near,half],[near,-half],[far,-half],[far,half]].flatMap(([forward,left])=>[(W/2-.5-f*left/forward)/(W-1),(H/2-.5+f*height/forward)/(H-1)]);
    return {near_distance_m:near,far_distance_m:far,ground_half_width_m:half,search_half_width_m:half,roi_polygon:roi,roi_exclusion_polygon:[],bev_width_px:168,bev_height_px:80};
  }
  async plan(now){
    this.busy=true;const generation=this.generation,pose={...this.pose},stamp=this.stamp,id=`${this.session}:${generation}:${++this.sequence}`;
    const started=performance.now();
    try{
      const pixels=this.capture();
      const request={schema:'line_tracking.connectome.v1',stream_id:`${this.session}:${generation}`,request_id:id,stamp_sec:stamp,requested_modes:[1],local_config:this.localConfig,planner_config:{half_width_m:.3,margin_m:.35,min_length_m:1,max_center_offset_m:.2,flow_weight:.02,road_corridor_smoothing_m:1,road_building_cost_weight:0,road_building_distance_m:2},pose,speed_mps:.8,camera_yaw:this.viewYaw,scenario:this.scenario,mission_id:this.missionKey,sensor_faults:this.faultConfig,environment:{oncoming_car:this.oncomingCar?.snapshot()||null},evaluation:{violation_frames:this.violations?.length||0,latest_violation:this.violations?.at(-1)||null},...pixels};
      const response=await fetch('/api/step',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(request),signal:AbortSignal.timeout(10000)});
      const reply=await response.json();if(generation!==this.generation||!this.running)return;
      if(!response.ok)throw Error(reply.error||'추론 요청 실패');
      if(reply.request_id!==id||reply.stream_id!==request.stream_id||reply.stamp_sec!==stamp||reply.model?.bundle_sha256!==this.health?.model.bundle_sha256||reply.model?.backend!=='flyvis'||reply.perception?.weights_sha256!==this.health?.perception.weights_sha256)throw Error('프레임 또는 모델 불일치');
      const command=reply.command;const neural=!!reply.policy;
      if(neural&&reply.policy.weights_sha256!==this.health.policy?.weights_sha256)throw Error('행동 모델 불일치');
      const faults=reply.sensor_faults;
      if(!faults||faults.schema!=='dangjin.sensor_faults.v1'||faults.seed!==this.faultConfig.seed||faults.profile!==this.faultConfig.profile||faults.enabled!==this.faultConfig.enabled)throw Error('검출 누락 시험 설정 불일치');
      this.inferenceCount++;
      if(faults.kind==='dropout')this.faultDropouts.add(faults.event_id);
      if(faults.event_id!==this.lastFault?.event_id||faults.kind!==this.lastFault?.kind)this.events.push({t:stamp,pose:{...pose},reason:'sensor_fault_change',sensor_faults:faults});
      this.lastFault=faults;
      $('sim-fault-live').dataset.kind=faults.kind;
      $('sim-fault-live').textContent=faults.kind==='dropout'?`전체 검출 누락 · 남은 ${faults.remaining_s.toFixed(1)} s`:faults.kind==='holes'?`랜덤 구멍 · 차도 픽셀 ${(faults.removed_fraction*100).toFixed(1)}% 누락`:faults.enabled?'검출 유지 구간':'검출 누락 효과 꺼짐';
      if(!command||!Number.isFinite(command.vx)||!Number.isFinite(command.yaw_rate)||command.vx<0||command.vx>.800001||Math.abs(command.yaw_rate)>.350001)throw Error('제어 한계 위반');
      let points=command.vx>0||command.reason==='lap_complete'?(reply.paths?.['1']?.points_xy||[]):[];
      if(neural&&command.vx>0){let arc={x:0,y:0,heading:0};points=[[0,0]];for(let i=0;i<30;i++){arc=stepPose(arc,command,.1);points.push([arc.x,arc.y]);}}
      const worldPoints=points.map(p=>localToSite(p,this.cameraPose));
      this.mission=reply.mission;this.navTarget=neural?null:reply.mission?.target_site||null;
      const prediction=new Image();prediction.src='data:image/png;base64,'+(reply.surround_prediction_png||reply.prediction_png);await prediction.decode();if(generation!==this.generation)return;
      const mask=$('sim-mask');mask.width=prediction.width;mask.height=prediction.height;mask.getContext('2d').drawImage(prediction,0,0);
      mask.nextElementSibling.textContent=faults.enabled?'누락 적용 후 검출 · 행동 모델 입력':reply.surround_prediction_png?'360° 차도 검출 · 초록: 통행 추정 / 회색: 미관측':'RGB 차도 추정 · 정답 마스크 미사용';
      this.updateMetrics();
      for(let i=1;!neural&&i<worldPoints.length;i++){
        const a={x:worldPoints[i-1][0],y:worldPoints[i-1][1]},b={x:worldPoints[i][0],y:worldPoints[i][1]};
        const blocked=this.world.swept(a,b,.65);if(blocked){this.pause(blocked);return;}
      }
      this.pathLine.geometry.dispose();this.pathLine.geometry=new THREE.BufferGeometry().setFromPoints(worldPoints.map(p=>new THREE.Vector3(p[0],.25,-p[1])));
      const overlay=$('sim-forward').getContext('2d');overlay.strokeStyle='#50f6c1';overlay.lineWidth=3;overlay.beginPath();points.forEach((p,i)=>{const [x,y]=this.pixel(p);i?overlay.lineTo(x,y):overlay.moveTo(x,y);});overlay.stroke();
      if(command.vx===0){
        if(neural&&command.reason==='neural_wait'||!neural&&shouldReobserve(command,faults)){
          // A stopped robot still receives new sensor frames. Advancing only
          // sensor time lets a transient dropout end without blind motion.
          this.command={...zero};
          for(let i=0;i<10;i++){this.advanceObstacle(DT/10);const reason=this.world.blocked(pose.x,pose.y,.65);if(reason&&!this.violations.some(v=>v.t===stamp))this.violations.push({t:stamp,reason,pose:{...pose}});}
          this.stamp+=DT;this.waitSeconds+=DT;
          this.samples.push({t:stamp,pose,next_pose:{...pose},command,environment:request.environment,waiting:true,policy:reply.policy_metrics,center_error_m:this.world.nearestRoad(pose.x,pose.y).distance,perception:reply.perception_metrics,sensor_faults:faults,mission:reply.mission});
          this.status(neural?'neural_wait':faults.kind==='dropout'?'detection_dropout':'reobserving','정지 상태에서 새 영상을 확인합니다.');this.updateMetrics();this.scheduleNext(started);return;
        }
        this.pause(command.reason);return;
      }
      let next=pose;
      for(let i=0;i<10;i++){this.advanceObstacle(DT/10);const step=stepPose(next,command,DT/10);const reason=this.world.swept(next,step,.65);if(reason){if(!neural){this.pause(reason);return;}if(!this.violations.some(v=>v.t===stamp))this.violations.push({t:stamp,reason,pose:{...step}});}next=step;}
      this.command=command;this.distance+=Math.hypot(next.x-pose.x,next.y-pose.y);this.pose=next;this.stamp+=DT;
      const nearest=this.world.nearestRoad(next.x,next.y);const duration=performance.now()-started;
      this.samples.push({t:stamp,pose,next_pose:next,command,environment:request.environment,center_error_m:nearest.distance,planner_ms:duration,policy:reply.policy_metrics,evaluation:{violation_frames:this.violations.length},flow_ms:reply.timing.flow_ms,model_inferred_frames:reply.model.inferred_frames,perception:reply.perception_metrics,sensor_faults:faults,mission:reply.mission});
      this.trail.push(new THREE.Vector3(next.x,.22,-next.y));if(this.trail.length>5000)this.trail.shift();this.trailLine.geometry.dispose();this.trailLine.geometry=new THREE.BufferGeometry().setFromPoints(this.trail);
      this.place();this.status(command.reason,neural?`행동 추론 ${reply.policy_metrics.ms.toFixed(1)} ms · 회전 ${command.yaw_rate.toFixed(3)} rad/s · 이탈/접촉 ${this.violations.length}회`:`${reply.perception_metrics.views===4?'360° 4방향 검출':'차도 인식'} ${reply.perception_metrics.ms.toFixed(0)} ms · 광류 ${reply.timing.flow_ms.toFixed(0)} ms`);this.updateMetrics();
      this.scheduleNext(started);
    }catch(error){if(generation===this.generation)this.pause('planner_error',error.message);}
    finally{this.busy=false;}
  }
  scheduleNext(started){const rate=Number($('sim-rate').value);this.next=performance.now()+Math.max(0,DT*1000/rate-(performance.now()-started));}
  updateMetrics(){
    $('sim-progress').textContent=this.mission?`${(this.mission.fraction*100).toFixed(1)}% · ${this.mission.progress_m.toFixed(0)} / ${this.mission.length_m.toFixed(0)} m`:this.scenario==='patrol'?'0.0% · 순환 시작점':'구간 시험';
    $('sim-distance').textContent=`${this.distance.toFixed(1)} m`;$('sim-speed').textContent=`${this.command.vx.toFixed(2)} m/s`;
    const moving=this.samples.filter(s=>!s.waiting),n=moving.length;const rms=n?Math.sqrt(moving.reduce((sum,s)=>sum+s.center_error_m**2,0)/n):0;
    $('sim-error').textContent=`${rms.toFixed(2)} m`;$('sim-frames').textContent=String(this.inferenceCount||0);$('sim-time').textContent=`${((this.stamp-1)).toFixed(1)} s`;
    $('sim-fault-counts').textContent=`전체 누락 ${this.faultDropouts?.size||0}회 · 재검출 대기 ${(this.waitSeconds||0).toFixed(1)} s`;
  }
  async collect(){
    this.pause();const saved={...this.pose},nav=this.navTarget;this.navTarget=null;
    const route=this.world.perimeterRoute();$('sim-start').disabled=true;$('sim-collect').disabled=true;
    $('sim-mask').width=W;$('sim-mask').height=H;
    const caption=$('sim-mask').nextElementSibling;caption.textContent='학습 표본용 정답 · 주행 입력과 분리';
    try{
      for(let i=0;i<128;i++){
        const index=Math.floor((i+.37)*route.length/128)%route.length,p=route[index],q=route[(index+8)%route.length],heading=Math.atan2(q[1]-p[1],q[0]-p[0]);
        const offset=((i*7)%11-5)*.17;
        for(let sector=0;sector<4;sector++){
          this.pose={x:p[0]-Math.sin(heading)*offset,y:p[1]+Math.cos(heading)*offset,heading:heading+((i*3)%9-4)*.04+sector*Math.PI/2};
          const pixels=this.capture(true),sample=i*4+sector;
          const response=await fetch('/api/training-frame',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({index:sample,dataset:'connectome360',pose:this.pose,...pixels})});
          if(!response.ok)throw Error('표본 저장 실패');this.status('collecting',`${sample+1} / 512 · 4방향`);
          await new Promise(resolve=>requestAnimationFrame(resolve));
        }
      }
      this.status('idle','4방향 RGB 학습 표본 512개 저장 · 주행 중에는 정답 마스크를 사용하지 않습니다.');
    }catch(error){this.status('planner_error',error.message);}
    finally{this.pose=saved;this.navTarget=nav;this.place();this.capture();caption.textContent='360° 차도 검출 · 초록: 통행 추정 / 회색: 미관측';$('sim-mask').width=768;$('sim-mask').height=384;$('sim-start').disabled=false;$('sim-collect').disabled=false;}
  }
  async collectPolicy(){
    this.pause();const saved={...this.pose},nav=this.navTarget,fixed=this.fixedCamera;
    this.navTarget=null;this.fixedCamera=true;
    const route=this.world.perimeterRoute();$('sim-start').disabled=true;$('sim-policy-collect').disabled=true;
    try{
      for(let i=0;i<256;i++){
        const index=Math.floor((i+.23)*route.length/256)%route.length,p=route[index],q=route[(index+5)%route.length],heading=Math.atan2(q[1]-p[1],q[0]-p[0]);
        for(let variant=0;variant<3;variant++){
          const offset=(variant-1)*1.1,angle=variant===1?0:((i*13+variant*7)%17-8)*.045;
          this.pose={x:p[0]-Math.sin(heading)*offset,y:p[1]+Math.cos(heading)*offset,heading:heading+angle};
          const pixels=this.capture(),sample=i*3+variant;
          const response=await fetch('/api/policy-training-frame',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({index:sample,pose_group:i,route_index:index,pose:this.pose,local_config:this.localConfig,...pixels})});
          if(!response.ok)throw Error('주행 정책 영상 저장 실패');this.status('collecting',`${sample+1} / 768 · 차도 이탈·방향 변화 포함`);
          await new Promise(resolve=>requestAnimationFrame(resolve));
        }
      }
      this.status('idle','주행 정책 영상 768 × 4방향 저장 완료');
    }catch(error){this.status('planner_error',error.message);}
    finally{this.pose=saved;this.navTarget=nav;this.fixedCamera=fixed;this.place();this.capture();$('sim-start').disabled=false;$('sim-policy-collect').disabled=false;}
  }
  tick(now){if(!this.active)return;this.follow();if(this.running&&!this.busy&&now>=this.next)this.plan(now);}
}
