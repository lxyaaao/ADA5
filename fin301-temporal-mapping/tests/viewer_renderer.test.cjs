'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const modulePath = path.join(__dirname, '../viewer/renderer.js');
const R = fs.existsSync(modulePath) ? require(modulePath) : {};
const close = (a,b,t=1e-7) => assert.ok(Math.abs(a-b)<t, `${a} != ${b}`);
const closeArray = (a,b,t=1e-7) => a.forEach((v,i)=>close(v,b[i],t));
const camera = (extra={}) => ({center:[2,-3,0],scale:20,width:800,height:600,azimuth:0,elevation:Math.PI/2,...extra});

test('top view maps positive x right, positive y up with equal metric scale',()=>{
  assert.equal(typeof R.projectPoint,'function');
  closeArray(R.projectPoint([2,-3,0],camera()),[400,300,0]);
  closeArray(R.projectPoint([3,-2,0],camera()).slice(0,2),[420,280]);
});
test('orthographic screen-to-plane round trips at arbitrary camera angles and heights',()=>{
  assert.equal(typeof R.screenToPlane,'function');
  for(const azimuth of [-2.1,0,.6]) for(const elevation of [.15,.8,Math.PI/2]) {
    const c = camera({azimuth,elevation,center:[3,2,.7]});
    for(const p of [[1,2,0],[-4,3,1.2],[0,0,-2]]) {
      const s=R.projectPoint(p,c);
      closeArray(R.screenToPlane(s[0],s[1],p[2],c),p);
    }
  }
  assert.equal(R.screenToPlane(400,300,0,camera({elevation:0})),null);
});
test('GPU orthographic matrix agrees with CSS-pixel projection and nearer depth',()=>{
  assert.equal(typeof R.orthographicMatrix,'function');
  const c=camera({azimuth:.7,elevation:.6}); const p=[7,-2,1];
  const m=R.orthographicMatrix(c,200); const clip=[0,1,2].map(i=>m[i]*p[0]+m[4+i]*p[1]+m[8+i]*p[2]+m[12+i]);
  const screen=R.projectPoint(p,c);
  close((clip[0]+1)*c.width/2,screen[0],1e-5);
  close((1-clip[1])*c.height/2,screen[1],1e-5);
  close(clip[2],-screen[2]/200,1e-7);
});
test('grid corners apply resolution, origin, and yaw without flipping world Y',()=>{
  assert.equal(typeof R.gridCorners,'function');
  const corners=R.gridCorners({width:3,height:2,resolution:.5,origin:[4,-2,Math.PI/2]});
  [[4,-2,0],[4,-.5,0],[3,-.5,0],[3,-2,0]].forEach((p,i)=>closeArray(corners[i],p));
  assert.deepEqual(R.GRID_UVS,[0,1,1,1,1,0,0,0]); // image row zero is upper local Y
});
test('fit frames all bounds in equal-scale top and tilted views',()=>{
  assert.equal(typeof R.fitCamera,'function');
  for(const elevation of [.5,Math.PI/2]) {
    const c=R.fitCamera({min:[-15,-4,-2],max:[12,3,4]},camera({elevation,azimuth:.7}),.85);
    for(const x of [-15,12])for(const y of [-4,3])for(const z of [-2,4]){
      const p=R.projectPoint([x,y,z],c);
      assert.ok(p[0]>=0 && p[0]<=c.width && p[1]>=0 && p[1]<=c.height);
    }
  }
});
test('zoom preserves cursor-plane anchor and pan follows screen displacement',()=>{
  assert.equal(typeof R.zoomCamera,'function');
  const c=camera({azimuth:.7,elevation:.6}); const anchor=R.screenToPlane(123,456,0,c);
  const zoomed=R.zoomCamera(c,2,123,456);
  closeArray(R.projectPoint(anchor,zoomed).slice(0,2),[123,456]);
  const panned=R.panCamera(c,25,-18);
  const before=R.projectPoint([1,1,0],c), after=R.projectPoint([1,1,0],panned);
  close(after[0]-before[0],25); close(after[1]-before[1],-18);
});
test('occupancy appearance honors negate and thresholds and preserves unknown',()=>{
  assert.equal(typeof R.occupancyGray,'function');
  const m={negate:0,occupied_thresh:.65,free_thresh:.196,mode:'trinary'};
  assert.equal(R.occupancyGray(0,255,m),0);
  assert.equal(R.occupancyGray(254,255,m),254);
  assert.equal(R.occupancyGray(205,255,m),205);
  assert.equal(R.occupancyGray(255,255,{...m,negate:1}),0);
  assert.equal(R.occupancyGray(0,0,m),205);
  assert.equal(R.occupancyGray(100,255,{...m,mode:'raw'}),0);
  assert.equal(R.occupancyGray(255,255,{...m,mode:'raw'}),205);
});
test('metric helpers reject nonfinite and invalid dimensions',()=>{
  assert.equal(typeof R.gridCorners,'function');
  assert.throws(()=>R.gridCorners({width:3,height:2,resolution:0,origin:[0,0,0]}),/resolution/i);
  assert.throws(()=>R.gridCorners({width:3,height:2,resolution:1,origin:[NaN,0,0]}),/origin/i);
  assert.throws(()=>R.zoomCamera(camera(),-1),/zoom/i);
});
test('missing WebGL reports a useful recoverable error rather than fake 3D',()=>{
  assert.equal(typeof R.ViewerRenderer,'function');
  assert.throws(()=>new R.ViewerRenderer({getContext:()=>null},{}),/WebGL.*hardware acceleration/);
});
