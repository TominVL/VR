"use strict";

function getRotationMatrixFromVector(rotationVector) {
    let q1 = rotationVector[0];
    let q2 = rotationVector[1];
    let q3 = rotationVector[2];
    let q0;

    if (rotationVector.length >= 4) {
        q0 = rotationVector[3];
    } else {
        q0 = 1 - q1*q1 - q2*q2 - q3*q3;
        q0 = (q0 > 0) ? Math.sqrt(q0) : 0;
    }

    let sq_q1 = 2 * q1 * q1;
    let sq_q2 = 2 * q2 * q2;
    let sq_q3 = 2 * q3 * q3;
    let q1_q2 = 2 * q1 * q2;
    let q3_q0 = 2 * q3 * q0;
    let q1_q3 = 2 * q1 * q3;
    let q2_q0 = 2 * q2 * q0;
    let q2_q3 = 2 * q2 * q3;
    let q1_q0 = 2 * q1 * q0;


    let r00 = 1 - sq_q2 - sq_q3;
    let r01 = q1_q2 - q3_q0;
    let r02 = q1_q3 + q2_q0;

    let r10 = q1_q2 + q3_q0;
    let r11 = 1 - sq_q1 - sq_q3;
    let r12 = q2_q3 - q1_q0;

    let r20 = q1_q3 - q2_q0;
    let r21 = q2_q3 + q1_q0;
    let r22 = 1 - sq_q1 - sq_q2;

    return new Float32Array([
        r00, r10, r20, 0,
        r01, r11, r21, 0,
        r02, r12, r22, 0,
        0,   0,   0,   1
    ]);
}

/* ═══════════════════════════════════════════════
   TRACKBALL ROTATOR  (mouse fallback)
═══════════════════════════════════════════════ */
function TrackballRotator(canvas, drawCallback) {
    let quat = [0, 0, 0, 1];
    let dragging = false, prev = [0, 0];

    function norm(x, y) {
        let r = Math.sqrt(x*x + y*y);
        if (r > 1) { x /= r; y /= r; r = 1; }
        return [x, y, Math.sqrt(1 - r*r)];
    }
    function mulQ(a, b) {
        return [
            a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
            a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
            a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
            a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]
        ];
    }
    function quatToMat(q) {
        let [x, y, z, w] = q;
        return new Float32Array([
            1-2*(y*y+z*z), 2*(x*y+z*w),   2*(x*z-y*w),   0,
            2*(x*y-z*w),   1-2*(x*x+z*z), 2*(y*z+x*w),   0,
            2*(x*z+y*w),   2*(y*z-x*w),   1-2*(x*x+y*y), 0,
            0,             0,             0,             1
        ]);
    }
    function clientXY(e, rect) {
        let ex = e.touches ? e.touches[0].clientX : e.clientX;
        let ey = e.touches ? e.touches[0].clientY : e.clientY;
        return [
            (ex - rect.left) / rect.width  * 2 - 1,
           -((ey - rect.top) / rect.height * 2 - 1)
        ];
    }

    canvas.addEventListener('mousedown', e => {
        dragging = true;
        prev = clientXY(e, canvas.getBoundingClientRect());
    });
    window.addEventListener('mouseup', () => dragging = false);
    window.addEventListener('mousemove', e => {
        if (!dragging) return;
        let cur = clientXY(e, canvas.getBoundingClientRect());
        let p1 = norm(prev[0], prev[1]);
        let p2 = norm(cur[0],  cur[1]);
        let axis = [
            p1[1]*p2[2] - p1[2]*p2[1],
            p1[2]*p2[0] - p1[0]*p2[2],
            p1[0]*p2[1] - p1[1]*p2[0]
        ];
        let dot   = Math.min(1, p1[0]*p2[0] + p1[1]*p2[1] + p1[2]*p2[2]);
        let angle = Math.acos(dot);
        let s     = Math.sin(angle / 2);
        let L     = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]) || 1;
        let dq    = [axis[0]/L*s, axis[1]/L*s, axis[2]/L*s, Math.cos(angle/2)];
        quat = mulQ(dq, quat);
        prev = cur;
        if (drawCallback) drawCallback();
    });

    this.getViewMatrix = () => quatToMat(quat);
}

/* ═══════════════════════════════════════════════
   STEREO CAMERA  (ported from OpenGL)
═══════════════════════════════════════════════ */
function StereoCamera(Convergence, EyeSeparation, AspectRatio, FOV, NearClipping, FarClipping) {
    this.mConvergence          = Convergence;
    this.mEyeSeparation        = EyeSeparation;
    this.mAspectRatio          = AspectRatio;
    this.mFOV                  = FOV * Math.PI / 180.0;
    this.mNearClippingDistance = NearClipping;
    this.mFarClippingDistance  = FarClipping;

    this._frustumMat = function(left, right, bottom, top, near, far) {
        let rl = right - left, tb = top - bottom, fn = far - near;
        return new Float32Array([
            2*near/rl,       0,               0,                    0,
            0,               2*near/tb,       0,                    0,
            (right+left)/rl, (top+bottom)/tb, -(far+near)/fn,      -1,
            0,               0,               -2*far*near/fn,       0
        ]);
    };

    this.getLeftFrustum = function() {
        let top    =  this.mNearClippingDistance * Math.tan(this.mFOV / 2);
        let bottom = -top;
        let a      =  this.mAspectRatio * Math.tan(this.mFOV / 2) * this.mConvergence;
        let b      =  a - this.mEyeSeparation / 2;
        let c      =  a + this.mEyeSeparation / 2;
        let left   = -b * this.mNearClippingDistance / this.mConvergence;
        let right  =  c * this.mNearClippingDistance / this.mConvergence;
        return {
            proj:         this._frustumMat(left, right, bottom, top, this.mNearClippingDistance, this.mFarClippingDistance),
            eyeTranslate: this.mEyeSeparation / 2
        };
    };

    this.getRightFrustum = function() {
        let top    =  this.mNearClippingDistance * Math.tan(this.mFOV / 2);
        let bottom = -top;
        let a      =  this.mAspectRatio * Math.tan(this.mFOV / 2) * this.mConvergence;
        let b      =  a - this.mEyeSeparation / 2;
        let c      =  a + this.mEyeSeparation / 2;
        let left   = -c * this.mNearClippingDistance / this.mConvergence;
        let right  =  b * this.mNearClippingDistance / this.mConvergence;
        return {
            proj:         this._frustumMat(left, right, bottom, top, this.mNearClippingDistance, this.mFarClippingDistance),
            eyeTranslate: -this.mEyeSeparation / 2
        };
    };
}

/* ═══════════════════════════════════════════════
   MATRIX HELPERS
═══════════════════════════════════════════════ */
function mat4mul(a, b) {
    let r = new Float32Array(16);
    for (let i = 0; i < 4; i++)
        for (let j = 0; j < 4; j++) {
            let s = 0;
            for (let k = 0; k < 4; k++) s += a[j + k*4] * b[k + i*4];
            r[j + i*4] = s;
        }
    return r;
}
function mat4trans(x, y, z) {
    let m = new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
    m[12] = x; m[13] = y; m[14] = z;
    return m;
}

/* ═══════════════════════════════════════════════
   SENSOR SERVER WebSocket
   Connects to Sensor Server app on Android.
   Reads game_rotation_vector sensor data.
═══════════════════════════════════════════════ */
let sensorMatrix = null;   // Float32Array(16) from phone, null = use mouse
let ws = null;

function connectSensor(ip, port) {
    // Samsung devices use different sensor type name
    // We try standard first; if disconnects, Samsung variant is used
    const sensorType = document.getElementById('sensor-type').value;
    const url = `ws://${ip}:${port}/sensor/connect?type=${sensorType}`;
    console.log("Connecting to:", url);
    setWsStatus('Connecting...', '#fa0');

    ws = new WebSocket(url);

    ws.onopen = () => {
        setWsStatus('Connected ✓', '#4f4');
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            // Sensor Server v7 sends: {"values":[x,y,z,w,accuracy],"accuracy":N,"timestamp":N}
            let v = null;
            if (Array.isArray(data)) {
                v = data;                    // old format: raw array
            } else if (data.values) {
                v = data.values;             // new format: {values:[...]}
            }
            if (!v || v.length < 3) return;
            sensorMatrix = getRotationMatrixFromVector(v);
            // Show incoming data for debug
            const dbg = document.getElementById('ws-status');
            if (dbg) dbg.textContent = 'LIVE ● x:' + v[0].toFixed(2) + ' y:' + v[1].toFixed(2) + ' z:' + v[2].toFixed(2);
        } catch(e) {
            console.warn('Sensor parse error:', e, event.data);
        }
    };

    ws.onerror = (e) => {
        console.error('WS error:', e);
        setWsStatus('Error (see console)', '#f44');
    };

    ws.onclose = (e) => {
        // code 1006 = abnormal closure (connection refused / wrong URL)
        // code 1000 = normal close
        setWsStatus('Disconnected (code ' + e.code + ')', '#f88');
        console.warn('WS closed:', e.code, e.reason);
        sensorMatrix = null;
    };
}

function setWsStatus(text, color) {
    const el = document.getElementById('ws-status');
    if (el) { el.textContent = text; el.style.color = color; }
}

/* ═══════════════════════════════════════════════
   MAIN APP
═══════════════════════════════════════════════ */
(function() {
    const canvas = document.getElementById('glcanvas');
    const gl = canvas.getContext('webgl', { preserveDrawingBuffer: false });
    if (!gl) { alert('WebGL not supported'); return; }

    function makeProgram(vsSource, fsSource) {
        function comp(type, src) {
            let s = gl.createShader(type);
            gl.shaderSource(s, src);
            gl.compileShader(s);
            if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
                console.error('Shader error:', gl.getShaderInfoLog(s));
                return null;
            }
            return s;
        }
        let p = gl.createProgram();
        gl.attachShader(p, comp(gl.VERTEX_SHADER,   vsSource));
        gl.attachShader(p, comp(gl.FRAGMENT_SHADER, fsSource));
        gl.linkProgram(p);
        if (!gl.getProgramParameter(p, gl.LINK_STATUS))
            console.error('Program link error:', gl.getProgramInfoLog(p));
        return p;
    }

    const prog    = makeProgram(vertexShaderSource,    fragmentShaderSource);
    const progCam = makeProgram(camVertexShaderSource, camFragmentShaderSource);

    const loc = {
        aVertex:   gl.getAttribLocation (prog, 'aVertex'),
        aNormal:   gl.getAttribLocation (prog, 'aNormal'),
        uMVP:      gl.getUniformLocation(prog, 'uMVP'),
        uMV:       gl.getUniformLocation(prog, 'uMV'),
        uLightPos: gl.getUniformLocation(prog, 'uLightPos'),
        uColor:    gl.getUniformLocation(prog, 'uColor'),
    };
    const camLoc = {
        aPos: gl.getAttribLocation (progCam, 'aPos'),
        uTex: gl.getUniformLocation(progCam, 'uTex'),
    };

    // ── Surface buffers ──
    const vBuf = gl.createBuffer();
    const nBuf = gl.createBuffer();
    const iBuf = gl.createBuffer();
    const wBuf = gl.createBuffer();
    let idxCount = 0, wireCount = 0;

    function uploadSurface(U, V) {
        let d = buildSurface(U, V);
        idxCount  = d.idx.length;
        wireCount = d.wires.length;
        gl.bindBuffer(gl.ARRAY_BUFFER,         vBuf); gl.bufferData(gl.ARRAY_BUFFER,         d.verts, gl.STATIC_DRAW);
        gl.bindBuffer(gl.ARRAY_BUFFER,         nBuf); gl.bufferData(gl.ARRAY_BUFFER,         d.norms, gl.STATIC_DRAW);
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, iBuf); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, d.idx,   gl.STATIC_DRAW);
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, wBuf); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, d.wires, gl.STATIC_DRAW);
    }

    // ── Webcam quad ──
    const quadBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, quadBuf);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1,-1,  1,-1,  -1,1,  1,1]), gl.STATIC_DRAW);

    const video = document.getElementById('webcam');
    let camTex = null;
    let webcamReady = false;

    function initCamTex() {
        camTex = gl.createTexture();
        gl.activeTexture(gl.TEXTURE3);
        gl.bindTexture(gl.TEXTURE_2D, camTex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 1, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE,
                      new Uint8Array([60, 60, 60, 255]));
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    }

    function updateCamTex() {
        if (!webcamReady || !camTex) return;
        if (video.videoWidth === 0 || video.readyState < 2) return;
        gl.activeTexture(gl.TEXTURE3);
        gl.bindTexture(gl.TEXTURE_2D, camTex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, video);
    }

    function drawWebcamPlane() {
        if (!webcamReady || !camTex) return;
        gl.useProgram(progCam);
        gl.disable(gl.DEPTH_TEST);
        gl.depthMask(false);
        gl.bindBuffer(gl.ARRAY_BUFFER, quadBuf);
        gl.vertexAttribPointer(camLoc.aPos, 2, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(camLoc.aPos);
        gl.activeTexture(gl.TEXTURE3);
        gl.bindTexture(gl.TEXTURE_2D, camTex);
        gl.uniform1i(camLoc.uTex, 3);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
        gl.depthMask(true);
        gl.enable(gl.DEPTH_TEST);
    }

    // ── App state ──
    let eyeSep = 0.5, convergence = 8, fov = 40, nearClip = 1;
    const farClip = 100;
    let wireframe = true;
    let Usteps = 30, Vsteps = 30;

    const trackball = new TrackballRotator(canvas, () => {});

    // ── Sliders ──
    function bindSlider(sliderId, valId, sbId, parser, onchange) {
        let sl = document.getElementById(sliderId);
        let fn = () => {
            let v = parser(sl.value);
            document.getElementById(valId).textContent = v;
            if (sbId) document.getElementById(sbId).textContent = v;
            onchange(parseFloat(sl.value));
        };
        sl.addEventListener('input', fn);
        fn();
    }
    bindSlider('s-eye',  'v-eye',  'sb-eye',  v => parseFloat(v).toFixed(2), v => { eyeSep      = v; });
    bindSlider('s-conv', 'v-conv', 'sb-conv', v => parseFloat(v).toFixed(1), v => { convergence = v; });
    bindSlider('s-fov',  'v-fov',  'sb-fov',  v => parseInt(v),              v => { fov         = v; });
    bindSlider('s-near', 'v-near', 'sb-near', v => parseFloat(v).toFixed(1), v => { nearClip    = v; });
    bindSlider('s-u', 'v-u', null, v => parseInt(v), v => { Usteps = parseInt(v); uploadSurface(Usteps, Vsteps); });
    bindSlider('s-v', 'v-v', null, v => parseInt(v), v => { Vsteps = parseInt(v); uploadSurface(Usteps, Vsteps); });

    document.getElementById('wireframe-btn').addEventListener('click', function() {
        wireframe = !wireframe;
        this.textContent = wireframe ? 'Wireframe: ON' : 'Wireframe: OFF';
        this.classList.toggle('active', wireframe);
    });

    // ── Sensor connect button ──
    document.getElementById('sensor-btn').addEventListener('click', () => {
        const ip   = document.getElementById('sensor-ip').value.trim();
        const port = document.getElementById('sensor-port').value.trim() || '8080';
        if (!ip) { alert('Enter phone IP address'); return; }
        if (ws) ws.close();
        connectSensor(ip, port);
    });

    document.getElementById('sensor-disconnect').addEventListener('click', () => {
        if (ws) ws.close();
        sensorMatrix = null;
        setWsStatus('Disconnected', '#888');
    });

    // ── Webcam ──
    document.getElementById('cam-btn').addEventListener('click', () => {
        navigator.mediaDevices.getUserMedia({ video: true })
        .then(stream => {
            video.srcObject = stream;
            document.getElementById('webcam-status').textContent = 'Starting...';
            initCamTex();

            function onReady() {
                webcamReady = true;
                document.getElementById('webcam-status').textContent = 'LIVE ●';
                document.getElementById('webcam-wrap').style.display = 'block';
            }

            video.addEventListener('playing', onReady, { once: true });
            video.play().catch(err => console.warn('video.play():', err));
            setTimeout(() => { if (!webcamReady && video.readyState >= 2) onReady(); }, 1500);
        })
        .catch(err => {
            console.error('Webcam error:', err);
            document.getElementById('webcam-status').textContent = 'Access denied';
        });
    });

    uploadSurface(Usteps, Vsteps);

    // ── Draw surface for one eye ──
    function drawSurface(proj, mv) {
        gl.useProgram(prog);

        gl.bindBuffer(gl.ARRAY_BUFFER, vBuf);
        gl.vertexAttribPointer(loc.aVertex, 3, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(loc.aVertex);

        gl.bindBuffer(gl.ARRAY_BUFFER, nBuf);
        gl.vertexAttribPointer(loc.aNormal, 3, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(loc.aNormal);

        let mvp = mat4mul(proj, mv);
        gl.uniformMatrix4fv(loc.uMVP, false, mvp);
        gl.uniformMatrix4fv(loc.uMV,  false, mv);

        let t = performance.now() * 0.001;
        gl.uniform3f(loc.uLightPos, 4*Math.cos(t), 4*Math.sin(t), 3);

        gl.uniform3f(loc.uColor, 0.3, 0.55, 0.85);
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, iBuf);
        gl.drawElements(gl.TRIANGLES, idxCount, gl.UNSIGNED_SHORT, 0);

        if (wireframe) {
            gl.uniform3f(loc.uColor, 0.05, 0.8, 1.0);
            gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, wBuf);
            gl.drawElements(gl.LINES, wireCount, gl.UNSIGNED_SHORT, 0);
        }
    }

    // ── FPS ──
    let lastTime = performance.now(), frames = 0;

    // ── MAIN RENDER LOOP ──
    function frame() {
        requestAnimationFrame(frame);

        frames++;
        let now = performance.now();
        if (now - lastTime > 500) {
            let fps = Math.round(frames * 1000 / (now - lastTime));
            frames = 0; lastTime = now;
            let el = document.getElementById('fps-label');
            if (el) el.textContent = fps + ' FPS';
        }

        gl.viewport(0, 0, canvas.width, canvas.height);

        let cam = new StereoCamera(convergence, eyeSep, canvas.width / canvas.height, fov, nearClip, farClip);
        let L   = cam.getLeftFrustum();
        let R   = cam.getRightFrustum();

        // Use phone sensor matrix if connected, otherwise mouse trackball
        let rotM = sensorMatrix ? sensorMatrix : trackball.getViewMatrix();

        let transM = mat4trans(0, 0, -convergence);
        let MV_L   = mat4mul(transM, mat4mul(mat4trans( L.eyeTranslate, 0, 0), rotM));
        let MV_R   = mat4mul(transM, mat4mul(mat4trans( R.eyeTranslate, 0, 0), rotM));

        updateCamTex();

        gl.colorMask(true, true, true, true);
        gl.clearColor(0, 0, 0, 1);
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

        // LEFT EYE — Red channel
        gl.colorMask(true, false, false, false);
        drawWebcamPlane();
        gl.clear(gl.DEPTH_BUFFER_BIT);
        drawSurface(L.proj, MV_L);

        // RIGHT EYE — Cyan channel
        gl.clear(gl.DEPTH_BUFFER_BIT);
        gl.colorMask(false, true, true, false);
        drawWebcamPlane();
        gl.clear(gl.DEPTH_BUFFER_BIT);
        drawSurface(R.proj, MV_R);

        gl.colorMask(true, true, true, true);
    }

    requestAnimationFrame(frame);
})();
