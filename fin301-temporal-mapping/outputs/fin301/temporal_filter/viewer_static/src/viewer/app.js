(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const elements = Object.fromEntries(['viewport','mapCanvas','overlayCanvas','loading','fatalError','datasetTitle','sourceName',
    'pointCount','mapExtent','heightInfo','lidarToggle','gridToggle','trajectoryToggle','visualToggle','lidarCount','gridLabel',
    'trajectoryCount','visualCount','pointSize','pointSizeValue','measureBtn','measurementBox','distanceValue','measurementHint',
    'measurementCoords','measurePrompt','coordinateReadout','viewModeLabel','statusText','fileList','fileCount','mapInfo',
    'playBtn','timelineSlider','poseLabel','timeLabel','speedSelect','fileInput','pointFileLabel','pointFileSelect','toast'].map(id=>[id,$(id)]));
  const colors = {lidar:[.14,.27,.33],visual:[.91,.47,.16]};
  const empty = new Float32Array(0);
  const decoder = new TextDecoder('utf-8', {fatal:true});
  let renderer, scene=null, bundled, viewMode='top', measuring=false, measurements=[], pointer=null;
  let playing=false, playTime=0, lastTick=0, playbackFrame=0, currentIndex=0, toastTimer;
  let loadGeneration=0;

  function status(text) { elements.statusText.textContent=text; }
  function toast(text,error=false) {
    clearTimeout(toastTimer); elements.toast.textContent=text; elements.toast.classList.toggle('error',error);
    elements.toast.hidden=false; toastTimer=setTimeout(()=>{elements.toast.hidden=true;},error?7000:3500);
  }
  function fromBase64(value) {const text=atob(value),bytes=new Uint8Array(text.length);for(let i=0;i<text.length;i++)bytes[i]=text.charCodeAt(i);return bytes.buffer;}
  function extension(name) {return name.split('.').pop().toLowerCase();}
  function basename(name) {return name.replace(/\\/g,'/').split('/').pop();}
  function fileByName(files,name) {return files.find(f=>f.name.toLowerCase()===basename(name).toLowerCase());}
  function number(value,digits=2) {return Number(value).toLocaleString('en-US',{minimumFractionDigits:digits,maximumFractionDigits:digits});}
  function duration(seconds) {const t=Math.max(0,seconds||0);return Math.floor(t/60)+':'+String(Math.floor(t%60)).padStart(2,'0');}
  function download(bytes,name,type='application/octet-stream') {
    const url=URL.createObjectURL(new Blob([bytes],{type})); const a=document.createElement('a');a.href=url;a.download=name;
    document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),10000);
  }
  function decodeImage(file) {
    if(extension(file.name)==='pgm') {
      const image=MapIO.parsePGM(file.bytes),canvas=document.createElement('canvas');canvas.width=image.width;canvas.height=image.height;
      const context=canvas.getContext('2d'),rgba=context.createImageData(image.width,image.height);
      for(let i=0;i<image.data.length;i++){const n=i*4;rgba.data[n]=rgba.data[n+1]=rgba.data[n+2]=image.data[i];rgba.data[n+3]=255;}
      context.putImageData(rgba,0,0);return Promise.resolve(canvas);
    }
    return new Promise((resolve,reject)=>{
      const image=new Image(),url=URL.createObjectURL(new Blob([file.bytes]));
      image.onload=()=>{URL.revokeObjectURL(url);if(image.width*image.height>25000000){reject(new Error('The map image exceeds 25 million pixels.'));return;}
        const canvas=document.createElement('canvas');canvas.width=image.width;canvas.height=image.height;canvas.getContext('2d').drawImage(image,0,0);resolve(canvas);};
      image.onerror=()=>{URL.revokeObjectURL(url);reject(new Error('Cannot read map image: '+file.name));}; image.src=url;
    });
  }
  async function parseScene(files,title,isBundled) {
    if(!files.length)throw new Error('No supported map files were selected.');
    const names=new Set();for(const file of files){const key=file.name.toLowerCase();if(names.has(key))throw new Error('Duplicate filename: '+file.name);names.add(key);}
    const pcdfiles=files.filter(f=>extension(f.name)==='pcd'&&f.name.toLowerCase()!=='visual_landmarks.pcd');
    const primary=fileByName(pcdfiles,'lidar_scan_map.pcd')||pcdfiles[0]||null;
    const visualFile=fileByName(files,'visual_landmarks.pcd');
    const yamlFiles=files.filter(f=>['yaml','yml'].includes(extension(f.name)));
    if(yamlFiles.length>1)throw new Error('Select one YAML map configuration at a time.');
    let grid=null;
    if(yamlFiles.length){const config=MapIO.parseMapYaml(decoder.decode(yamlFiles[0].bytes));const imageFile=fileByName(files,config.image);
      if(!imageFile)throw new Error('The YAML file requires image '+config.image+'; select both the YAML file and this image.');
      if(!['pgm','png','jpg','jpeg'].includes(extension(imageFile.name)))throw new Error('The grid image must be PGM, PNG or JPEG.');
      const canvas=await decodeImage(imageFile);grid={canvas,config:{...config,width:canvas.width,height:canvas.height},file:yamlFiles[0]};}
    const csvFiles=files.filter(f=>extension(f.name)==='csv');if(csvFiles.length>1)throw new Error('Select one trajectory CSV at a time.');
    const trajectory=csvFiles.length?MapIO.parseTrajectory(decoder.decode(csvFiles[0].bytes)):null;
    if(!primary&&!visualFile&&!grid&&!trajectory)throw new Error('Import a PCD, a YAML file with its image, or a trajectory CSV.');
    return {files,title,isBundled,pcdfiles,primary,cloud:primary?MapIO.parsePCD(primary.bytes):null,
      visual:visualFile?MapIO.parsePCD(visualFile.bytes):null,grid,trajectory};
  }
  function primaryHasVisual() {return !!(scene&&scene.primary&&scene.primary.name.toLowerCase()==='map.pcd'&&scene.visual&&fileByName(scene.files,'lidar_scan_map.pcd'));}
  function cloudVariant(name) {return scene&&scene.isBundled&&bundled.metadata&&bundled.metadata.pointcloud_variants?bundled.metadata.pointcloud_variants[name]:null;}
  function updatePrimaryInformation() {
    const cloud=scene.cloud,grid=scene.grid;
    elements.sourceName.textContent=scene.primary?scene.primary.name:(grid?grid.file.name:(scene.trajectory?'trajectory.csv':'visual_landmarks.pcd'));
    elements.pointCount.textContent=cloud?cloud.count.toLocaleString('en-US'):'—'; elements.lidarCount.textContent=cloud?cloud.count.toLocaleString('en-US'):'No cloud';
    if(cloud){const min=cloud.bounds.min,max=cloud.bounds.max;elements.mapExtent.textContent=number(max[0]-min[0],1)+' × '+number(max[1]-min[1],1);
      elements.heightInfo.textContent=Math.max(Math.abs(min[2]),Math.abs(max[2]))<1e-6?'LiDAR scan plane · z = 0 m':'Cloud height: '+number(min[2])+' to '+number(max[2])+' m';}
    else {elements.mapExtent.textContent=grid?number(grid.config.width*grid.config.resolution,1)+' × '+number(grid.config.height*grid.config.resolution,1):'—';elements.heightInfo.textContent=grid?'Grid placement uses the YAML origin, yaw and resolution.':'No main point cloud loaded.';}
    const variant=scene.primary?cloudVariant(scene.primary.name):null;
    if(variant){elements.heightInfo.textContent+=' · '+number(variant.removed_percent,1)+'% removed. Original grid retained; localization accuracy is unverified.';}
    elements.lidarToggle.disabled=!cloud;elements.visualToggle.disabled=!scene.visual||primaryHasVisual();
    if(primaryHasVisual()){elements.visualToggle.checked=false;elements.visualCount.textContent='Included in cloud';renderer.setVisible('visual',false);}
    else elements.visualCount.textContent=scene.visual?scene.visual.count.toLocaleString('en-US'):'No landmarks';
  }
  function updateFiles() {
    elements.fileList.replaceChildren();elements.fileCount.textContent=scene.files.length+' files';
    for(const file of scene.files){const button=document.createElement('button');button.className='file-item';button.title='Download '+file.name;
      const badge=document.createElement('strong');badge.textContent=extension(file.name).toUpperCase();const name=document.createElement('span');name.textContent=file.name;
      const icon=document.createElement('i');icon.textContent='↓';button.append(badge,name,icon);button.addEventListener('click',()=>download(file.bytes,file.name));elements.fileList.append(button);}
    elements.pointFileSelect.replaceChildren();for(const file of scene.pcdfiles){const option=document.createElement('option');option.value=file.name;const variant=cloudVariant(file.name);option.textContent=variant?variant.label+' — '+file.name:file.name;option.selected=scene.primary===file;elements.pointFileSelect.append(option);}
    elements.pointFileLabel.hidden=scene.pcdfiles.length<2;
  }
  function installScene(next) {
    stopPlayback();scene=next;clearMeasurement();setMeasuring(false);currentIndex=0;playTime=0;
    elements.datasetTitle.textContent=scene.title;
    document.title=scene.title+' · Offline Viewer';
    renderer.setPointLayer('lidar',scene.cloud?scene.cloud.positions:empty,colors.lidar);
    renderer.setPointLayer('visual',scene.visual?scene.visual.positions:empty,colors.visual);
    const poses=scene.trajectory?scene.trajectory.poses:[];
    renderer.setTrajectory(new Float32Array(poses.flatMap(p=>[p.x,p.y,0])));
    renderer.setGrid(scene.grid?scene.grid.canvas:null,scene.grid?scene.grid.config:null);
    elements.lidarToggle.checked=!!scene.cloud;elements.gridToggle.checked=!scene.cloud&&!!scene.grid;
    elements.visualToggle.checked=!scene.cloud&&!scene.grid&&!!scene.visual;elements.trajectoryToggle.checked=poses.length>0;
    elements.gridToggle.disabled=!scene.grid;elements.trajectoryToggle.disabled=!poses.length;
    for(const name of ['lidar','grid','trajectory','visual'])renderer.setVisible(name,elements[name+'Toggle'].checked);
    elements.gridLabel.textContent=scene.grid?number(scene.grid.config.resolution,2)+' m/cell':'No grid';
    elements.trajectoryCount.textContent=poses.length?poses.length.toLocaleString('en-US')+' poses':'No trajectory';
    elements.playBtn.disabled=poses.length<2;elements.timelineSlider.disabled=!poses.length;
    elements.timelineSlider.max=String(Math.max(0,poses.length-1));elements.timelineSlider.value='0';
    updatePrimaryInformation();updateFiles();setPose(0);setView('top');renderer.fit();
    elements.mapInfo.textContent=scene.grid?'Grid '+scene.grid.config.width+' × '+scene.grid.config.height+'; origin ('+scene.grid.config.origin.map(v=>number(v,2)).join(', ')+'); resolution '+scene.grid.config.resolution+' m/cell.':'No occupancy grid loaded. Select the image referenced by the YAML file when importing it.';
    status((scene.isBundled?'fin301 loaded':'Local files loaded')+' · All points shown in their original coordinates');
  }
  async function loadFiles(files,title,isBundled) {
    const generation=++loadGeneration;elements.loading.hidden=false;
    try{const next=await parseScene(files,title,isBundled);if(generation!==loadGeneration)return;installScene(next);}
    catch(error){if(generation!==loadGeneration)return;toast(error.message||String(error),true);status('Import failed; the current map is retained.');if(!scene){elements.fatalError.textContent=error.message;elements.fatalError.hidden=false;}}
    finally{if(generation===loadGeneration)elements.loading.hidden=true;}
  }
  async function restore() {
    const files=bundled.files.map(file=>({name:file.name,bytes:fromBase64(file.base64)}));
    await loadFiles(files,bundled.metadata&&bundled.metadata.viewer_title||'fin301 Localization Map',true);
  }
  async function importFiles(list) {
    const chosen=Array.from(list).filter(file=>['pcd','yaml','yml','pgm','png','jpg','jpeg','csv'].includes(extension(file.name)));
    if(chosen.length>32||chosen.reduce((sum,f)=>sum+f.size,0)>100*1024*1024){toast('Import at most 32 files with a combined size of 100 MB or less.',true);return;}
    try{const files=await Promise.all(chosen.map(async file=>({name:file.name,bytes:await file.arrayBuffer()})));
      await loadFiles(files,'Local Map',false);}
    catch(error){toast('Cannot read local files: '+error.message,true);}
    elements.fileInput.value='';
  }
  function setView(mode) {
    viewMode=mode;renderer.setView(mode);
    $('topViewBtn').classList.toggle('selected',mode==='top');$('threeViewBtn').classList.toggle('selected',mode==='3d');
    $('topViewBtn').setAttribute('aria-pressed',String(mode==='top'));$('threeViewBtn').setAttribute('aria-pressed',String(mode==='3d'));
    $('viewHint').textContent=mode==='top'?'Drag to pan · Scroll to zoom':'Drag to orbit · Right-drag / Shift to pan';elements.viewModeLabel.textContent=mode==='top'?'Top view · Metres':'3D · Orthographic';
  }
  function setMeasuring(value) {measuring=value;elements.measureBtn.classList.toggle('active',value);elements.measureBtn.setAttribute('aria-pressed',String(value));
    elements.viewport.classList.toggle('measuring',value);elements.measurePrompt.hidden=!value;
    elements.measurePrompt.textContent=measurements.length===1?'Select the second point · Esc to exit':'Select the first point · Esc to exit';
    elements.measureBtn.textContent=value?'⌁ Measuring · Click to exit':'⌁ Measure two points';}
  function clearMeasurement() {measurements=[];if(renderer)renderer.setMeasurement([]);elements.distanceValue.innerHTML='— <span>m</span>';
    elements.measurementHint.textContent='Enable measurement, then select a start and end point on the map.';elements.measurementCoords.textContent='';if(measuring)setMeasuring(true);}
  function measurementPoint(x,y) {
    let point=renderer.screenToPlane(x,y,0);if(!point)return null;
    // Snap only to visible scan-plane returns; visual landmarks do not become 2D obstacles.
    if(scene&&scene.cloud&&elements.lidarToggle.checked){let best=64;const p=scene.cloud.positions;
      for(let i=0;i<p.length;i+=3){if(Math.abs(p[i+2])>1e-5)continue;const s=renderer.project([p[i],p[i+1],0]);if(!s)continue;
        const d=(s[0]-x)**2+(s[1]-y)**2;if(d<best){best=d;point=[p[i],p[i+1],0];}}}
    return point;
  }
  function addMeasurement(x,y) {
    const point=measurementPoint(x,y);if(!point){toast('The current view does not intersect the z=0 plane. Adjust the view or switch to Top.',true);return;}
    if(measurements.length===2)clearMeasurement();measurements.push(point);renderer.setMeasurement(measurements);
    elements.measurementCoords.textContent=measurements.map((p,i)=>(i?'B':'A')+' ('+number(p[0],3)+', '+number(p[1],3)+') m').join('\n');
    elements.measurementCoords.style.whiteSpace='pre-line';
    if(measurements.length===2){const d=Math.hypot(measurements[1][0]-measurements[0][0],measurements[1][1]-measurements[0][1]);elements.distanceValue.innerHTML=number(d,3)+' <span>m</span>';
      elements.measurementHint.textContent='Map distance in the z=0 plane. Click again to start a new measurement.';status('Measured distance: '+number(d,3)+' m (map estimate)');}
    else elements.measurementHint.textContent='Point A selected. Choose endpoint B.';setMeasuring(true);
  }
  function setPose(index) {
    const poses=scene&&scene.trajectory?scene.trajectory.poses:[];currentIndex=Math.max(0,Math.min(index,poses.length-1));
    const pose=poses[currentIndex];renderer.setVehicle(pose&&elements.trajectoryToggle.checked?pose:null);
    if(pose){elements.timelineSlider.value=String(currentIndex);elements.timeLabel.textContent=duration(pose.time)+' / '+duration(poses[poses.length-1].time);
      elements.poseLabel.textContent=(currentIndex+1)+' / '+poses.length;elements.timelineSlider.title='x='+number(pose.x,3)+' m, y='+number(pose.y,3)+' m, yaw='+number(pose.yaw*180/Math.PI,1)+'°; t='+pose.timestampNs+' ns';}
    else{elements.timeLabel.textContent='0:00 / 0:00';elements.poseLabel.textContent='No trajectory loaded';}
  }
  function stopPlayback() {playing=false;cancelAnimationFrame(playbackFrame);elements.playBtn.textContent='▶';elements.playBtn.setAttribute('aria-label','Play trajectory');}
  function tick(now) {
    if(!playing||!scene.trajectory)return;const poses=scene.trajectory.poses;playTime+=(now-lastTick)/1000*Number(elements.speedSelect.value);lastTick=now;
    let lo=0,hi=poses.length-1;while(lo<hi){const m=Math.ceil((lo+hi)/2);if(poses[m].time<=playTime)lo=m;else hi=m-1;}setPose(lo);
    if(playTime>=poses[poses.length-1].time){setPose(poses.length-1);stopPlayback();return;}playbackFrame=requestAnimationFrame(tick);
  }
  function togglePlayback() {
    if(!scene||!scene.trajectory||scene.trajectory.poses.length<2)return;if(playing){stopPlayback();return;}
    const poses=scene.trajectory.poses;if(currentIndex>=poses.length-1)setPose(0);playTime=poses[currentIndex].time;playing=true;lastTick=performance.now();
    elements.playBtn.textContent='Ⅱ';elements.playBtn.setAttribute('aria-label','Pause trajectory');playbackFrame=requestAnimationFrame(tick);
  }
  function pointerPosition(event) {const rect=elements.viewport.getBoundingClientRect();return [event.clientX-rect.left,event.clientY-rect.top];}
  function attachEvents() {
    $('restoreBtn').addEventListener('click',restore);$('importBtn').addEventListener('click',()=>elements.fileInput.click());
    elements.fileInput.addEventListener('change',event=>importFiles(event.target.files));
    $('helpBtn').addEventListener('click',()=>$('helpDialog').showModal());$('closeHelpBtn').addEventListener('click',()=>$('helpDialog').close());
    $('helpDialog').addEventListener('click',event=>{if(event.target===$('helpDialog')){const r=$('helpDialog').getBoundingClientRect();if(event.clientX<r.left||event.clientX>r.right||event.clientY<r.top||event.clientY>r.bottom)$('helpDialog').close();}});
    $('topViewBtn').addEventListener('click',()=>setView('top'));$('threeViewBtn').addEventListener('click',()=>setView('3d'));
    $('fitBtn').addEventListener('click',()=>renderer.fit());$('resetBtn').addEventListener('click',()=>{setView('top');renderer.fit();});
    $('zoomInBtn').addEventListener('click',()=>renderer.zoom(1.25));$('zoomOutBtn').addEventListener('click',()=>renderer.zoom(.8));
    elements.pointSize.addEventListener('input',()=>{renderer.setPointSize(Number(elements.pointSize.value));elements.pointSizeValue.textContent=Number(elements.pointSize.value).toFixed(1)+' px';});
    for(const name of ['lidar','grid','trajectory','visual'])elements[name+'Toggle'].addEventListener('change',()=>{renderer.setVisible(name,elements[name+'Toggle'].checked);if(name==='trajectory')setPose(currentIndex);});
    elements.pointFileSelect.addEventListener('change',()=>{try{const file=fileByName(scene.files,elements.pointFileSelect.value);const cloud=MapIO.parsePCD(file.bytes);
      scene.primary=file;scene.cloud=cloud;renderer.setPointLayer('lidar',cloud.positions,colors.lidar);elements.lidarToggle.checked=true;renderer.setVisible('lidar',true);updatePrimaryInformation();if(!scene.isBundled)renderer.fit();clearMeasurement();status('Current point cloud: '+file.name);}
      catch(error){toast(error.message,true);elements.pointFileSelect.value=scene.primary.name;}});
    elements.measureBtn.addEventListener('click',()=>setMeasuring(!measuring));$('clearMeasureBtn').addEventListener('click',clearMeasurement);
    elements.playBtn.addEventListener('click',togglePlayback);elements.timelineSlider.addEventListener('input',()=>{stopPlayback();setPose(Number(elements.timelineSlider.value));});
    $('exportBtn').addEventListener('click',()=>{try{const url=renderer.exportPNG();const a=document.createElement('a');a.href=url;a.download='fin301-map-view.png';document.body.appendChild(a);a.click();a.remove();toast('Current map view exported.');}catch(error){toast('Image export failed: '+error.message,true);}});
    elements.viewport.addEventListener('contextmenu',event=>event.preventDefault());
    elements.viewport.addEventListener('wheel',event=>{event.preventDefault();const [x,y]=pointerPosition(event);renderer.zoom(Math.exp(-Math.max(-200,Math.min(200,event.deltaY))*.0018),x,y);},{passive:false});
    elements.viewport.addEventListener('pointerdown',event=>{if(event.target.closest('button')||!scene)return;if(event.button!==0&&event.button!==2)return;
      const p=pointerPosition(event);pointer={id:event.pointerId,start:p,last:p,button:event.button,moved:false,shift:event.shiftKey};elements.viewport.setPointerCapture(event.pointerId);elements.viewport.focus({preventScroll:true});elements.viewport.classList.add('dragging');});
    elements.viewport.addEventListener('pointermove',event=>{const [x,y]=pointerPosition(event);const world=renderer.screenToPlane(x,y,0);elements.coordinateReadout.textContent=world?'X '+number(world[0],2)+'　Y '+number(world[1],2)+' m':'View is parallel to the z=0 plane';
      if(!pointer||pointer.id!==event.pointerId)return;const dx=x-pointer.last[0],dy=y-pointer.last[1];if(Math.hypot(x-pointer.start[0],y-pointer.start[1])>4)pointer.moved=true;
      if(pointer.moved){if(viewMode==='3d'&&pointer.button===0&&!pointer.shift&&!event.shiftKey&&!measuring)renderer.orbit(dx,dy);else renderer.pan(dx,dy);}pointer.last=[x,y];});
    const endPointer=event=>{if(!pointer||pointer.id!==event.pointerId)return;const click=!pointer.moved&&pointer.button===0;const p=pointerPosition(event);pointer=null;elements.viewport.classList.remove('dragging');
      if(elements.viewport.hasPointerCapture(event.pointerId))elements.viewport.releasePointerCapture(event.pointerId);if(click&&measuring&&event.type==='pointerup')addMeasurement(...p);};
    elements.viewport.addEventListener('pointerup',endPointer);elements.viewport.addEventListener('pointercancel',endPointer);
    for(const name of ['dragenter','dragover'])elements.viewport.addEventListener(name,event=>{event.preventDefault();$('dropHint').hidden=false;});
    elements.viewport.addEventListener('dragleave',event=>{if(!elements.viewport.contains(event.relatedTarget))$('dropHint').hidden=true;});
    elements.viewport.addEventListener('drop',event=>{event.preventDefault();$('dropHint').hidden=true;importFiles(event.dataTransfer.files);});
    document.addEventListener('keydown',event=>{if($('helpDialog').open||event.ctrlKey||event.metaKey||event.altKey||['INPUT','SELECT','TEXTAREA'].includes(event.target.tagName))return;
      const key=event.key.toLowerCase();if(key==='f'){renderer.fit();event.preventDefault();}else if(key==='2')setView('top');else if(key==='3')setView('3d');
      else if(key==='m')setMeasuring(!measuring);else if(key==='escape'){setMeasuring(false);stopPlayback();}else if(event.code==='Space'&&event.target.tagName!=='BUTTON'){event.preventDefault();togglePlayback();}
      else if(key==='+'||key==='=')renderer.zoom(1.25);else if(key==='-')renderer.zoom(.8);});
    new ResizeObserver(()=>renderer.resize()).observe(elements.viewport);
  }
  async function main() {
    try{bundled=JSON.parse($('bundled-files').textContent);renderer=new MapRenderer.ViewerRenderer(elements.mapCanvas,elements.overlayCanvas);
      attachEvents();renderer.setPointSize(Number(elements.pointSize.value));await restore();
      // Read-only diagnostic snapshot used by the browser verification harness.
      window.viewerDiagnostics=()=>({ready:!!scene,dataset:scene&&scene.title,pointCount:scene&&scene.cloud?scene.cloud.count:0,
        visualCount:scene&&scene.visual?scene.visual.count:0,trajectoryCount:scene&&scene.trajectory?scene.trajectory.poses.length:0,
        grid:scene&&scene.grid?{width:scene.grid.config.width,height:scene.grid.config.height,origin:scene.grid.config.origin,resolution:scene.grid.config.resolution}:null,
        viewMode,measuring,measurements:measurements.map(p=>p.slice()),currentIndex,playing,camera:renderer.getState()});
    }catch(error){elements.loading.hidden=true;elements.fatalError.hidden=false;elements.fatalError.textContent='Viewer could not start: '+error.message+'. Use a current WebGL-capable version of Edge, Chrome or Firefox.';status('Initialization failed');console.error(error);}
  }
  main();
})();
