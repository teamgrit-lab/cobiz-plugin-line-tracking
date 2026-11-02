import fs from 'node:fs';
import { WorldGeometry } from './world-geometry.js';
const site=JSON.parse(fs.readFileSync(new URL('./site.json',import.meta.url)));
const world=new WorldGeometry(site);
fs.writeFileSync(new URL('./world.json',import.meta.url),JSON.stringify(world.export(),null,2)+'\n');
console.log(`Exported ${world.roads.length} roads, ${world.obstacles.length} solid footprints, ${world.zones.length} exclusion zone`);
