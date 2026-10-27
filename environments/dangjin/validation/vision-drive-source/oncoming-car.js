// Environment actor only. Never supplies features or commands to the robot.
export class OncomingCar {
  constructor(points,robotPose,{loop=false,speed=.35,ahead=18,lateral=1.8}={}){
    this.points=points;this.loop=loop;this.speed=speed;this.elapsed=0;
    this.s=[0];for(let i=1;i<points.length;i++)this.s.push(this.s.at(-1)+Math.hypot(points[i][0]-points[i-1][0],points[i][1]-points[i-1][1]));
    this.length=this.s.at(-1);let closest={distance:Infinity};
    for(let i=1;i<points.length;i++){
      const a=points[i-1],b=points[i],dx=b[0]-a[0],dy=b[1]-a[1],length=this.s[i]-this.s[i-1];
      if(length<1e-8)continue;
      const t=Math.max(0,Math.min(1,((robotPose.x-a[0])*dx+(robotPose.y-a[1])*dy)/(length*length)));
      const distance=Math.hypot(robotPose.x-a[0]-t*dx,robotPose.y-a[1]-t*dy);
      if(distance<closest.distance)closest={distance,s:this.s[i-1]+t*length,heading:Math.atan2(dy,dx)};
    }
    const forward=Math.cos(closest.heading-robotPose.heading)>=0?1:-1;
    this.direction=-forward;this.offset=lateral*forward;this.origin=closest.s+forward*ahead;
    this.update(0);
  }
  update(dt){
    this.elapsed+=dt;let s=this.origin+this.direction*this.speed*this.elapsed;
    s=this.loop?(s%this.length+this.length)%this.length:Math.max(0,Math.min(this.length,s));
    let i=1;while(i<this.s.length-1&&this.s[i]<s)i++;
    const a=this.points[i-1],b=this.points[i],t=(s-this.s[i-1])/(this.s[i]-this.s[i-1]||1),h=Math.atan2(b[1]-a[1],b[0]-a[0]);
    this.pose={x:a[0]+t*(b[0]-a[0])-Math.sin(h)*this.offset,y:a[1]+t*(b[1]-a[1])+Math.cos(h)*this.offset,heading:h+(this.direction<0?Math.PI:0)};
    return this.pose;
  }
  footprint(){return {id:'oncoming-car',x:this.pose.x-2.15,y:this.pose.y-.9,length:4.3,width:1.8,heading:this.pose.heading};}
  snapshot(){return {pose:{...this.pose},footprint:this.footprint(),speed_mps:this.speed,elapsed_s:this.elapsed};}
}
