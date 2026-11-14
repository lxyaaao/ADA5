/* Offline metric map renderer. No network, dependencies, or pointer listeners.
 * Screen coordinates are CSS pixels. World coordinates are metres.
 * Camera: center XYZ, scale CSS pixels/metre, azimuth/elevation in radians.
 * Positive projected depth points towards the eye. Top view is +X right/+Y up.
 */
(function (root, factory) {
  'use strict';
  const api = factory(root);
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.MapRenderer = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function (root) {
  'use strict';
  const PI = Math.PI;
  const GRID_UVS = Object.freeze([0, 1, 1, 1, 1, 0, 0, 0]);
  const COLORS = Object.freeze({lidar: [0.09, 0.36, 0.48], visual: [0.72, 0.19, 0.55], trajectory: [0.89, 0.42, 0.09]});
  const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const finite = (v) => typeof v === 'number' && Number.isFinite(v);
  const vector = (v, n) => v && v.length === n && Array.from(v).every(finite);
  const copyCamera = (camera) => ({...camera, center: camera.center.slice()});

  function cameraBasis(camera) {
    const ca = Math.cos(camera.azimuth), sa = Math.sin(camera.azimuth);
    const ce = Math.cos(camera.elevation), se = Math.sin(camera.elevation);
    return {right: [ca, sa, 0], up: [-sa*se, ca*se, ce], eye: [sa*ce, -ca*ce, se]};
  }
  function projectPoint(point, camera) {
    const b = cameraBasis(camera), d = point.map((x, i) => x-camera.center[i]);
    return [camera.width/2 + dot(d,b.right)*camera.scale,
      camera.height/2 - dot(d,b.up)*camera.scale, dot(d,b.eye)];
  }
  function screenToPlane(x, y, z, camera) {
    const b = cameraBasis(camera);
    if (!finite(x) || !finite(y) || !finite(z) || Math.abs(b.eye[2]) < 1e-8) return null;
    const u = (x-camera.width/2)/camera.scale, v = (camera.height/2-y)/camera.scale;
    const base = camera.center.map((c,i) => c+u*b.right[i]+v*b.up[i]);
    const t = (z-base[2])/b.eye[2];
    return [base[0]+t*b.eye[0], base[1]+t*b.eye[1], z];
  }
  function orthographicMatrix(camera, depthRange) {
    const b = cameraBasis(camera), sx = 2*camera.scale/camera.width, sy = 2*camera.scale/camera.height;
    const d = depthRange || 1000;
    return new Float32Array([
      b.right[0]*sx,b.up[0]*sy,-b.eye[0]/d,0,
      b.right[1]*sx,b.up[1]*sy,-b.eye[1]/d,0,
      b.right[2]*sx,b.up[2]*sy,-b.eye[2]/d,0,
      -dot(camera.center,b.right)*sx,-dot(camera.center,b.up)*sy,dot(camera.center,b.eye)/d,1
    ]);
  }
  function gridCorners(meta) {
    if (!meta || !finite(meta.resolution) || meta.resolution <= 0) throw new Error('Invalid grid resolution');
    if (!Number.isInteger(meta.width) || !Number.isInteger(meta.height) || meta.width < 1 || meta.height < 1) throw new Error('Invalid grid dimensions');
    if (!vector(meta.origin,3)) throw new Error('Invalid grid origin');
    const [x,y,yaw] = meta.origin, c = Math.cos(yaw), s = Math.sin(yaw);
    const w = meta.width*meta.resolution, h = meta.height*meta.resolution;
    return [[0,0],[w,0],[w,h],[0,h]].map(([u,v]) => [x+c*u-s*v,y+s*u+c*v,0]);
  }
  function boundsCorners(bounds) {
    const result = [];
    for (const x of [bounds.min[0],bounds.max[0]]) for (const y of [bounds.min[1],bounds.max[1]]) for (const z of [bounds.min[2],bounds.max[2]]) result.push([x,y,z]);
    return result;
  }
  function fitCamera(bounds, camera, padding) {
    const next = copyCamera(camera), pad = padding === undefined ? 0.86 : padding;
    if (!bounds) return next;
    next.center = bounds.min.map((v,i) => (v+bounds.max[i])/2);
    const basis = cameraBasis(next), corners = boundsCorners(bounds);
    const horizontal = corners.map(p => dot(p,basis.right)), vertical = corners.map(p => dot(p,basis.up));
    const dx = Math.max(1,Math.max(...horizontal)-Math.min(...horizontal));
    const dy = Math.max(1,Math.max(...vertical)-Math.min(...vertical));
    next.scale = clamp(pad*Math.min(next.width/dx,next.height/dy),0.001,100000);
    return next;
  }
  function zoomCamera(camera, factor, x, y) {
    if (!finite(factor) || factor <= 0) throw new Error('Invalid zoom factor');
    const next = copyCamera(camera), sx = x === undefined ? camera.width/2 : x, sy = y === undefined ? camera.height/2 : y;
    const before = screenToPlane(sx,sy,0,camera);
    next.scale = clamp(camera.scale*factor,0.001,100000);
    const after = screenToPlane(sx,sy,0,next);
    if (before && after) next.center = next.center.map((v,i) => v+before[i]-after[i]);
    return next;
  }
  function panCamera(camera, dx, dy) {
    const next = copyCamera(camera), b = cameraBasis(camera);
    // Move the focal point in the view plane so the rendered map follows the pointer.
    next.center = next.center.map((v,i) => v-dx/camera.scale*b.right[i]+dy/camera.scale*b.up[i]);
    return next;
  }
  function occupancyGray(gray, alpha, meta) {
    if (alpha < 255) return 205;
    const mode = meta.mode || 'trinary';
    const value = meta.negate ? 255-gray : gray;
    if (mode === 'raw') return value <= 100 ? Math.round(254*(1-value/100)) : 205;
    const occupied = meta.occupied_thresh === undefined ? 0.65 : meta.occupied_thresh;
    const free = meta.free_thresh === undefined ? 0.196 : meta.free_thresh;
    const probability = 1-value/255;
    if (probability > occupied) return 0;
    if (probability < free) return 254;
    if (mode === 'scale') return Math.round(254*(1-(probability-free)/(occupied-free)));
    return 205;
  }
  function niceStep(value) {
    const magnitude = 10**Math.floor(Math.log10(Math.max(value,1e-12)));
    const n = value/magnitude;
    return (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10)*magnitude;
  }
  function formatMetres(value) {
    if (Math.abs(value)<1e-9) return '0';
    return Number(value.toFixed(Math.abs(value)<1 ? 3 : 2)).toString();
  }
  function arrayBounds(positions) {
    if (!positions.length) return null;
    const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
    for(let i=0;i<positions.length;i++) {
      const v=positions[i], j=i%3;
      if(!finite(v)) throw new Error('Point coordinates must be finite');
      min[j]=Math.min(min[j],v); max[j]=Math.max(max[j],v);
    }
    return {min,max};
  }
  function compileProgram(gl, vertex, fragment) {
    const shaders=[];
    for(const [kind,source] of [[gl.VERTEX_SHADER,vertex],[gl.FRAGMENT_SHADER,fragment]]) {
      const shader=gl.createShader(kind); gl.shaderSource(shader,source); gl.compileShader(shader);
      if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS)) {
        const message=gl.getShaderInfoLog(shader); gl.deleteShader(shader);
        shaders.forEach(s=>gl.deleteShader(s));
        throw new Error('WebGL shader compilation failed: '+message);
      }
      shaders.push(shader);
    }
    const program=gl.createProgram(); shaders.forEach(s=>gl.attachShader(program,s)); gl.linkProgram(program);
    shaders.forEach(s=>gl.deleteShader(s));
    if(!gl.getProgramParameter(program,gl.LINK_STATUS)) {
      const message=gl.getProgramInfoLog(program); gl.deleteProgram(program);
      throw new Error('WebGL program linking failed: '+message);
    }
    return program;
  }

  class ViewerRenderer {
    constructor(canvas, overlay) {
      if(!canvas || !overlay) throw new Error('Map and overlay canvases are required');
      this.canvas=canvas; this.overlay=overlay;
      this.gl=canvas.getContext('webgl',{alpha:false,antialias:true,preserveDrawingBuffer:true});
      if(!this.gl) throw new Error('WebGL is unavailable. Open this viewer in a browser that supports WebGL and enable hardware acceleration. Your map files have not been modified.');
      this.ctx=overlay.getContext('2d');
      if(!this.ctx) throw new Error('Could not create the 2D annotation canvas.');
      const gl=this.gl;
      this.pointProgram=compileProgram(gl,
        'attribute vec3 a_position; attribute vec3 a_color; uniform mat4 u_matrix; uniform float u_size; varying vec3 v_color; void main(){gl_Position=u_matrix*vec4(a_position,1.0);gl_PointSize=u_size;v_color=a_color;}',
        'precision mediump float; varying vec3 v_color; uniform float u_round; void main(){if(u_round>0.5 && distance(gl_PointCoord,vec2(0.5))>0.5)discard;gl_FragColor=vec4(v_color,1.0);}');
      this.pointLocations={position:gl.getAttribLocation(this.pointProgram,'a_position'),color:gl.getAttribLocation(this.pointProgram,'a_color'),matrix:gl.getUniformLocation(this.pointProgram,'u_matrix'),size:gl.getUniformLocation(this.pointProgram,'u_size'),round:gl.getUniformLocation(this.pointProgram,'u_round')};
      this.gridProgram=compileProgram(gl,
        'attribute vec3 a_position; attribute vec2 a_uv; uniform mat4 u_matrix; varying vec2 v_uv; void main(){gl_Position=u_matrix*vec4(a_position,1.0);v_uv=a_uv;}',
        'precision mediump float; varying vec2 v_uv; uniform sampler2D u_texture; void main(){gl_FragColor=texture2D(u_texture,v_uv);}');
      this.gridLocations={position:gl.getAttribLocation(this.gridProgram,'a_position'),uv:gl.getAttribLocation(this.gridProgram,'a_uv'),matrix:gl.getUniformLocation(this.gridProgram,'u_matrix'),texture:gl.getUniformLocation(this.gridProgram,'u_texture')};
      this.layers=Object.create(null); this.visibility={lidar:true,visual:false,trajectory:true,grid:false};
      this.grid=null; this.measurement=[]; this.vehicle=null; this.pointSize=2;
      this.camera={center:[0,0,0],scale:20,azimuth:0,elevation:PI/2,width:1,height:1};
      this.view='top'; this.orbitAngles={azimuth:-0.45,elevation:0.8}; this.framePending=false;
      this.dpr=1; this.maxPointSize=gl.getParameter(gl.ALIASED_POINT_SIZE_RANGE)[1];
      gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LEQUAL); gl.clearColor(0.956,0.969,0.965,1);
      this.resize();
    }
    _schedule() {
      if(this.framePending) return;
      this.framePending=true;
      const schedule=typeof root.requestAnimationFrame==='function' ? root.requestAnimationFrame.bind(root) : (cb)=>root.setTimeout(cb,16);
      schedule(()=>{this.framePending=false;this.render();});
    }
    _deleteLayer(layer) {
      if(!layer) return;
      this.gl.deleteBuffer(layer.buffer);
      if(layer.colorBuffer) this.gl.deleteBuffer(layer.colorBuffer);
    }
    setPointLayer(name, positions, colorArrayRGB01) {
      if(!positions || positions.length%3) throw new Error('XYZ point array length must be a multiple of three');
      const data=positions instanceof Float32Array ? positions : new Float32Array(positions);
      const bounds=arrayBounds(data), color=colorArrayRGB01 || COLORS[name] || COLORS.lidar;
      if(color.length!==3 && color.length!==data.length) throw new Error('RGB colors must be uniform or one per point');
      for(const c of color) if(!finite(c)||c<0||c>1) throw new Error('RGB colors must lie in [0,1]');
      const gl=this.gl, layer={count:data.length/3,bounds,color:Array.from(color.length===3?color:[1,1,1]),buffer:gl.createBuffer(),colorBuffer:null};
      gl.bindBuffer(gl.ARRAY_BUFFER,layer.buffer);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);
      if(color.length!==3 && data.length) {
        layer.colorBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,layer.colorBuffer);
        gl.bufferData(gl.ARRAY_BUFFER,color instanceof Float32Array?color:new Float32Array(color),gl.STATIC_DRAW);
      }
      this._deleteLayer(this.layers[name]);this.layers[name]=layer;
      if(!(name in this.visibility)) this.visibility[name]=true;
      this._schedule();return this;
    }
    setTrajectory(positions) {return this.setPointLayer('trajectory',positions,COLORS.trajectory);}
    setGrid(imageCanvas, metadata) {
      if(!imageCanvas) {
        this._deleteGrid();this.grid=null;this._schedule();return this;
      }
      const meta={mode:'trinary',negate:0,occupied_thresh:0.65,free_thresh:0.196,...metadata};
      const corners=gridCorners(meta);
      if(!['trinary','scale','raw'].includes(meta.mode)) throw new Error('Unsupported occupancy mode: '+meta.mode);
      if(!(finite(meta.free_thresh)&&finite(meta.occupied_thresh)&&meta.free_thresh>=0&&meta.occupied_thresh<=1&&meta.free_thresh<meta.occupied_thresh)) throw new Error('Invalid occupancy thresholds');
      if(imageCanvas.width!==meta.width || imageCanvas.height!==meta.height) throw new Error('Grid image dimensions disagree with metadata');
      const gl=this.gl;
      if(meta.width>gl.getParameter(gl.MAX_TEXTURE_SIZE)||meta.height>gl.getParameter(gl.MAX_TEXTURE_SIZE)) throw new Error('The grid image exceeds this device\'s texture size limit. Use a smaller grid image.');
      const temp=this.canvas.ownerDocument.createElement('canvas');temp.width=meta.width;temp.height=meta.height;
      const ctx=temp.getContext('2d');ctx.drawImage(imageCanvas,0,0);
      const image=ctx.getImageData(0,0,meta.width,meta.height),pixels=image.data;
      for(let i=0;i<pixels.length;i+=4) {
        const gray=occupancyGray((pixels[i]+pixels[i+1]+pixels[i+2])/3,pixels[i+3],meta);
        pixels[i]=gray;pixels[i+1]=gray;pixels[i+2]=gray;pixels[i+3]=255;
      }
      ctx.putImageData(image,0,0);
      const texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);
      gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,false);
      gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
      gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,gl.RGBA,gl.UNSIGNED_BYTE,temp);
      const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);
      const interleaved=[];
      for(let i=0;i<4;i++) interleaved.push(corners[i][0],corners[i][1],-0.01,GRID_UVS[i*2],GRID_UVS[i*2+1]);
      gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(interleaved),gl.STATIC_DRAW);
      this._deleteGrid();this.grid={texture,buffer,meta,corners,bounds:arrayBounds(corners.flat())};
      this._schedule();return this;
    }
    _deleteGrid() {
      if(this.grid){this.gl.deleteTexture(this.grid.texture);this.gl.deleteBuffer(this.grid.buffer);}
    }
    setVisible(name, visible) {this.visibility[name]=Boolean(visible);this._schedule();return this;}
    setPointSize(px) {
      if(!finite(px)||px<=0)throw new Error('Point size must be positive');
      this.pointSize=clamp(px,0.5,30);this._schedule();return this;
    }
    setView(view) {
      if(!['top','3d'].includes(view))throw new Error('View must be top or 3d');
      if(this.view==='3d')this.orbitAngles={azimuth:this.camera.azimuth,elevation:this.camera.elevation};
      this.view=view;
      Object.assign(this.camera,view==='top'?{azimuth:0,elevation:PI/2}:this.orbitAngles);
      this._schedule();return this;
    }
    _bounds() {
      const bounds=[];
      for(const name of Object.keys(this.layers)) if(this.visibility[name]&&this.layers[name].bounds)bounds.push(this.layers[name].bounds);
      if(this.grid&&this.visibility.grid)bounds.push(this.grid.bounds);
      if(!bounds.length)return null;
      return {min:[0,1,2].map(i=>Math.min(...bounds.map(b=>b.min[i]))),max:[0,1,2].map(i=>Math.max(...bounds.map(b=>b.max[i])))};
    }
    fit() {this.camera=fitCamera(this._bounds(),this.camera);this._schedule();return this;}
    zoom(factor,x,y) {this.camera=zoomCamera(this.camera,factor,x,y);this._schedule();return this;}
    pan(dx,dy) {
      if(!finite(dx)||!finite(dy))return this;
      this.camera=panCamera(this.camera,dx,dy);this._schedule();return this;
    }
    orbit(dx,dy) {
      if(!finite(dx)||!finite(dy))return this;
      if(this.view!=='3d')this.setView('3d');
      this.camera.azimuth+=dx*0.006;this.camera.elevation=clamp(this.camera.elevation+dy*0.006,0.12,PI/2);
      this.orbitAngles={azimuth:this.camera.azimuth,elevation:this.camera.elevation};this._schedule();return this;
    }
    project(point) {return projectPoint(point,this.camera);}
    screenToPlane(x,y,z=0) {return screenToPlane(x,y,z,this.camera);}
    setMeasurement(points) {
      if(!Array.isArray(points)||points.length>2||points.some(p=>!vector(p,3)))throw new Error('Measurement requires zero, one, or two finite XYZ points');
      this.measurement=points.map(p=>[p[0],p[1],0]);this._schedule();return this;
    }
    setVehicle(pose) {
      if(pose!==null&&(!pose||![pose.x,pose.y,pose.yaw].every(finite)))throw new Error('Vehicle pose must contain finite x,y,yaw');
      this.vehicle=pose?{x:pose.x,y:pose.y,yaw:pose.yaw}:null;this._schedule();return this;
    }
    resize() {
      const rect=this.canvas.getBoundingClientRect();
      this.camera.width=Math.max(1,rect.width||this.canvas.clientWidth||800);
      this.camera.height=Math.max(1,rect.height||this.canvas.clientHeight||600);
      this.dpr=clamp(root.devicePixelRatio||1,1,3);
      for(const canvas of [this.canvas,this.overlay]) {
        const w=Math.round(this.camera.width*this.dpr),h=Math.round(this.camera.height*this.dpr);
        if(canvas.width!==w)canvas.width=w;if(canvas.height!==h)canvas.height=h;
      }
      this._schedule();return this;
    }
    _drawGrid(matrix) {
      if(!this.grid||!this.visibility.grid)return;
      const gl=this.gl,l=this.gridLocations;gl.useProgram(this.gridProgram);
      gl.uniformMatrix4fv(l.matrix,false,matrix);gl.bindBuffer(gl.ARRAY_BUFFER,this.grid.buffer);
      gl.enableVertexAttribArray(l.position);gl.vertexAttribPointer(l.position,3,gl.FLOAT,false,20,0);
      gl.enableVertexAttribArray(l.uv);gl.vertexAttribPointer(l.uv,2,gl.FLOAT,false,20,12);
      gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.grid.texture);gl.uniform1i(l.texture,0);
      gl.drawArrays(gl.TRIANGLE_FAN,0,4);gl.disableVertexAttribArray(l.position);gl.disableVertexAttribArray(l.uv);
    }
    _drawLayer(name,matrix) {
      const layer=this.layers[name];if(!layer||!layer.count||!this.visibility[name])return;
      const gl=this.gl,l=this.pointLocations,isTrajectory=name==='trajectory';gl.useProgram(this.pointProgram);
      gl.uniformMatrix4fv(l.matrix,false,matrix);
      const size=name==='visual'?Math.max(5,this.pointSize+2):this.pointSize;
      gl.uniform1f(l.size,Math.min(this.maxPointSize,size*this.dpr));gl.uniform1f(l.round,isTrajectory?0:1);
      gl.bindBuffer(gl.ARRAY_BUFFER,layer.buffer);gl.enableVertexAttribArray(l.position);gl.vertexAttribPointer(l.position,3,gl.FLOAT,false,0,0);
      if(layer.colorBuffer){gl.bindBuffer(gl.ARRAY_BUFFER,layer.colorBuffer);gl.enableVertexAttribArray(l.color);gl.vertexAttribPointer(l.color,3,gl.FLOAT,false,0,0);}
      else {gl.disableVertexAttribArray(l.color);gl.vertexAttrib3fv(l.color,layer.color);}
      // The trajectory is an annotation: keep it legible through the point layer.
      if(isTrajectory){gl.disable(gl.DEPTH_TEST);gl.lineWidth(1);}
      gl.drawArrays(isTrajectory?gl.LINE_STRIP:gl.POINTS,0,layer.count);
      if(isTrajectory)gl.enable(gl.DEPTH_TEST);
      gl.disableVertexAttribArray(l.position);gl.disableVertexAttribArray(l.color);
    }
    _line(a,b,color,width=1,dash=[]) {
      const c=this.ctx;c.beginPath();c.moveTo(a[0],a[1]);c.lineTo(b[0],b[1]);c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);c.stroke();c.setLineDash([]);
    }
    _label(text,x,y,color='#344c51',align='left') {
      const c=this.ctx;c.font='12px system-ui, sans-serif';c.textAlign=align;c.textBaseline='middle';
      const width=c.measureText(text).width;
      const left=align==='center'?x-width/2:align==='right'?x-width:x;
      c.fillStyle='rgba(250,252,250,0.90)';c.fillRect(left-4,y-9,width+8,18);c.fillStyle=color;c.fillText(text,x,y);
    }
    _drawMetricGrid() {
      const w=this.camera.width,h=this.camera.height;
      const plane=[[0,0],[w,0],[w,h],[0,h]].map(p=>this.screenToPlane(p[0],p[1]));
      if(plane.some(p=>!p))return;
      const minX=Math.min(...plane.map(p=>p[0])),maxX=Math.max(...plane.map(p=>p[0]));
      const minY=Math.min(...plane.map(p=>p[1])),maxY=Math.max(...plane.map(p=>p[1]));
      let step=niceStep(90/this.camera.scale);
      while((maxX-minX+maxY-minY)/step>90)step=niceStep(step*1.1);
      for(let x=Math.ceil(minX/step)*step;x<=maxX;x+=step){
        this._line(this.project([x,minY,0]),this.project([x,maxY,0]),'rgba(45,80,85,0.09)');
        if(this.view==='top')this._label(formatMetres(x),this.project([x,0,0])[0],h-14,'#708589','center');
      }
      for(let y=Math.ceil(minY/step)*step;y<=maxY;y+=step){
        this._line(this.project([minX,y,0]),this.project([maxX,y,0]),'rgba(45,80,85,0.09)');
        if(this.view==='top')this._label(formatMetres(y),8,this.project([0,y,0])[1],'#708589');
      }
      const origin=this.project([0,0,0]),length=38/this.camera.scale;
      if(origin[0]>-50&&origin[0]<w+50&&origin[1]>-50&&origin[1]<h+50){
        const x=this.project([length,0,0]),y=this.project([0,length,0]);
        this._line(origin,x,'#b24738',2);this._line(origin,y,'#25856b',2);
        this._label('X',x[0]+7,x[1],'#b24738');this._label('Y',y[0]+5,y[1]-7,'#25856b');
        this._label('O',origin[0]-9,origin[1]+11,'#536970');
      }
      const scaleLength=niceStep(90/this.camera.scale)/2, pixels=scaleLength*this.camera.scale;
      const sx=24,sy=h-42;
      this._line([sx,sy],[sx+pixels,sy],'#2f5157',2);
      this._line([sx,sy-4],[sx,sy+4],'#2f5157',2);this._line([sx+pixels,sy-4],[sx+pixels,sy+4],'#2f5157',2);
      this._label(formatMetres(scaleLength)+' m',sx+pixels/2,sy-12,'#2f5157','center');
    }
    _drawMeasurement() {
      const c=this.ctx,points=this.measurement.map(p=>this.project(p));
      if(points.length===2){
        this._line(points[0],points[1],'#a33677',2,[6,4]);
        const distance=Math.hypot(this.measurement[1][0]-this.measurement[0][0],this.measurement[1][1]-this.measurement[0][1]);
        this._label(distance.toFixed(3)+' m · Planar distance',(points[0][0]+points[1][0])/2,(points[0][1]+points[1][1])/2-16,'#94316c','center');
      }
      points.forEach((p,i)=>{
        c.beginPath();c.arc(p[0],p[1],5,0,PI*2);c.fillStyle='#a33677';c.fill();c.strokeStyle='#fff';c.lineWidth=2;c.stroke();
        const world=this.measurement[i];this._label((i?'B':'A')+' ('+world[0].toFixed(2)+', '+world[1].toFixed(2)+') m',p[0]+10,p[1]+15,'#94316c');
      });
    }
    _drawVehicle() {
      if(!this.vehicle)return;
      const p=this.vehicle,c=this.ctx,a=this.project([p.x,p.y,0]);
      const b=this.project([p.x+Math.cos(p.yaw),p.y+Math.sin(p.yaw),0]);
      const angle=Math.atan2(b[1]-a[1],b[0]-a[0]);
      c.save();c.translate(a[0],a[1]);c.rotate(angle);c.beginPath();c.moveTo(14,0);c.lineTo(-9,8);c.lineTo(-5,0);c.lineTo(-9,-8);c.closePath();
      c.fillStyle='#f99a37';c.fill();c.strokeStyle='#693a15';c.lineWidth=1.6;c.stroke();c.restore();
      this._label('Trajectory position',a[0]+17,a[1]-14,'#7b481d');
    }
    render() {
      const gl=this.gl;if(gl.isContextLost())return;
      gl.viewport(0,0,this.canvas.width,this.canvas.height);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
      const bounds=this._bounds();
      const range=bounds?Math.max(100,...bounds.min.map((v,i)=>Math.abs(v-this.camera.center[i])),...bounds.max.map((v,i)=>Math.abs(v-this.camera.center[i])))*4:1000;
      const matrix=orthographicMatrix(this.camera,range);this._drawGrid(matrix);
      for(const name of Object.keys(this.layers))if(name!=='trajectory')this._drawLayer(name,matrix);
      this._drawLayer('trajectory',matrix);
      const c=this.ctx;c.setTransform(this.dpr,0,0,this.dpr,0,0);c.clearRect(0,0,this.camera.width,this.camera.height);
      this._drawMetricGrid();this._drawMeasurement();this._drawVehicle();
    }
    exportPNG() {
      this.render();
      const canvas=this.canvas.ownerDocument.createElement('canvas');canvas.width=this.canvas.width;canvas.height=this.canvas.height;
      const ctx=canvas.getContext('2d');ctx.drawImage(this.canvas,0,0);ctx.drawImage(this.overlay,0,0);
      return canvas.toDataURL('image/png');
    }
    getState() {
      return {...copyCamera(this.camera),view:this.view,pixelsPerMetre:this.camera.scale,pointSize:this.pointSize,dpr:this.dpr,visible:{...this.visibility},bounds:this._bounds(),counts:Object.fromEntries(Object.entries(this.layers).map(([k,v])=>[k,v.count]))};
    }
  }
  return {ViewerRenderer,cameraBasis,projectPoint,screenToPlane,orthographicMatrix,gridCorners,GRID_UVS,fitCamera,zoomCamera,panCamera,occupancyGray,COLORS};
});
