"""Seeded, simulation-only false negatives AFTER road inference and BEFORE planning.

Burst schedules use simulated seconds, so pausing or changing playback speed
does not draw a new pattern. This is a stress test, not a measured noise model.
"""
import math
import cv2
import numpy as np

PROFILES={
    'mild': {'holes_gap':(3,7),'holes_duration':(.5,1.5),'holes_count':(3,7),'radius_scale':.65,
             'drop_first':(12,22),'drop_gap':(22,40),'drop_duration':(.5,1)},
    'normal': {'holes_gap':(1.5,4.5),'holes_duration':(.5,2.5),'holes_count':(5,13),'radius_scale':1.,
               'drop_first':(6,12),'drop_gap':(10,22),'drop_duration':(.5,2)},
    'severe': {'holes_gap':(.5,2),'holes_duration':(1,3),'holes_count':(9,20),'radius_scale':1.5,
               'drop_first':(3,7),'drop_gap':(5,11),'drop_duration':(1,3)},
}


def fault_config(value=None):
    if value is None:return {'enabled':False,'profile':'normal','seed':0}
    if not isinstance(value,dict) or set(value)!={'enabled','profile','seed'}:
        raise ValueError('sensor_faults requires enabled, profile and seed')
    if type(value['enabled']) is not bool or value['profile'] not in PROFILES:
        raise ValueError('invalid sensor fault profile')
    if type(value['seed']) is not int or not 0<=value['seed']<=2**32-1:
        raise ValueError('sensor fault seed must be a uint32')
    return dict(value)


class SensorFaults:
    version='dangjin.sensor_faults.v1'

    def __init__(self,config=None):
        self.config=fault_config(config);p=PROFILES[self.config['profile']]
        self.profile=p;self.events={'holes':[],'dropout':[]}
        self.rng={kind:np.random.default_rng(np.random.SeedSequence([self.config['seed'],i]))
                  for i,kind in enumerate(self.events)}
        self.next_start={'holes':float(self.rng['holes'].uniform(1,3)),
                         'dropout':float(self.rng['dropout'].uniform(*p['drop_first']))}

    def active_event(self,kind,t):
        p=self.profile;rng=self.rng[kind];events=self.events[kind]
        while self.next_start[kind]<=t:
            start=self.next_start[kind]
            duration=float(rng.uniform(*(p['holes_duration'] if kind=='holes' else p['drop_duration'])))
            event={'id':len(events)+1,'start':start,'end':start+duration}
            if kind=='holes':
                event['ellipses']=[]
                for _ in range(4):
                    ellipses=[]
                    for _ in range(int(rng.integers(*p['holes_count']))):
                        ellipses.append([float(rng.uniform(.15,.85)),float(rng.uniform(.56,.88)),
                            float(rng.uniform(.018,.065)*p['radius_scale']),
                            float(rng.uniform(.008,.028)*p['radius_scale']),float(rng.uniform(-25,25))])
                    event['ellipses'].append(ellipses)
            events.append(event)
            self.next_start[kind]=event['end']+float(rng.uniform(*(p['holes_gap'] if kind=='holes' else p['drop_gap'])))
        return next((event for event in reversed(events) if event['start']<=t<event['end']),None)

    def apply(self,masks,elapsed):
        if not math.isfinite(elapsed) or not 0<=elapsed<=100000:raise ValueError('bounded fault simulation time required')
        source=np.asarray(masks)
        if source.ndim!=3 or not 1<=len(source)<=4 or not np.isin(source,[0,1]).all():
            raise ValueError('one to four binary road masks required')
        output=source.astype(np.uint8,copy=True);holes=dropout=None
        kind='off'
        if self.config['enabled']:
            holes=self.active_event('holes',elapsed);dropout=self.active_event('dropout',elapsed)
            kind='dropout' if dropout else 'holes' if holes else 'clear'
            if dropout:output.fill(0)
            elif holes:
                h,w=output.shape[1:]
                for i,ellipses in enumerate(holes['ellipses'][:len(output)]):
                    for cx,cy,rx,ry,angle in ellipses:
                        cv2.ellipse(output[i],(round(cx*w),round(cy*h)),(max(1,round(rx*w)),max(1,round(ry*h))),angle,0,360,0,-1)
        before=np.count_nonzero(source,axis=(1,2));after=np.count_nonzero(output,axis=(1,2))
        active=dropout or holes
        details={'schema':self.version,**self.config,'kind':kind,'elapsed_s':elapsed,
            'event_id':f'{kind}-{active["id"]}' if active else None,
            'remaining_s':max(0.,active['end']-elapsed) if active else 0.,
            'removed_fraction':float((before.sum()-after.sum())/max(1,int(before.sum()))),
            'front_removed_fraction':float((before[0]-after[0])/max(1,int(before[0]))),
            'raw_road_pixels':before.tolist(),'remaining_road_pixels':after.tolist()}
        return output,details
