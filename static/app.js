/**
 * WebKin Application - Main entry point
 */

let scene, camera, renderer, controls;
let kinematicScene;
let ws;
let jointNames = [];
let manualMode = false;  // false = server control, true = local manual control
let offsetOverrides = {};  // Track which joints have offset overrides

function init() {
    // Scene setup
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);

    // Camera
    camera = new THREE.PerspectiveCamera(
        60,
        window.innerWidth / window.innerHeight,
        0.1,
        10000
    );
    camera.position.set(200, 200, 200);
    camera.lookAt(0, 0, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    document.getElementById('container').appendChild(renderer.domElement);

    // Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Lights
    const ambientLight = new THREE.AmbientLight(0x404040, 0.5);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(100, 200, 100);
    scene.add(directionalLight);

    const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight2.position.set(-100, -100, -100);
    scene.add(directionalLight2);

    // Grid helper
    const gridHelper = new THREE.GridHelper(500, 50, 0x444444, 0x333333);
    scene.add(gridHelper);

    // Axes helper
    const axesHelper = new THREE.AxesHelper(100);
    scene.add(axesHelper);

    // Create scene manager
    kinematicScene = new KinematicScene(scene);

    // Setup mode toggle
    setupModeToggle();

    // Setup clear overrides button
    setupClearOverridesButton();

    // Load offset overrides
    loadOffsetOverrides();

    // Connect WebSocket
    connectWebSocket();

    // Load tree data for local calculations
    loadTreeData();

    // Handle resize
    window.addEventListener('resize', onWindowResize);

    // Start animation loop
    animate();
}

function setupModeToggle() {
    const checkbox = document.getElementById('manual-mode');
    const label = document.getElementById('mode-label');
    const sliders = document.getElementById('joint-sliders');

    checkbox.addEventListener('change', (e) => {
        manualMode = e.target.checked;

        if (manualMode) {
            label.textContent = 'Manual control';
            label.className = 'mode-local';
            sliders.classList.remove('disabled');
        } else {
            label.textContent = 'Server control';
            label.className = 'mode-server';
            sliders.classList.add('disabled');
        }

        console.log(`Mode: ${manualMode ? 'manual' : 'server'}`);
    });
}

function setupClearOverridesButton() {
    const btn = document.getElementById('clear-overrides');
    btn.addEventListener('click', async () => {
        try {
            const response = await fetch('/api/offset/overrides', { method: 'DELETE' });
            if (response.ok) {
                offsetOverrides = {};
                updateOverrideButtons();
                console.log('Cleared all offset overrides');
            }
        } catch (error) {
            console.error('Failed to clear overrides:', error);
        }
    });
}

async function loadOffsetOverrides() {
    try {
        const response = await fetch('/api/offset/overrides');
        const data = await response.json();
        offsetOverrides = data.overrides || {};
        updateOverrideButtons();
    } catch (error) {
        console.error('Failed to load offset overrides:', error);
    }
}

function updateOverrideButtons() {
    // Update all zero buttons to show override status
    for (const jointName of jointNames) {
        const btn = document.getElementById(`zero-${jointName}`);
        if (btn) {
            if (offsetOverrides[jointName] !== undefined) {
                btn.classList.add('has-override');
                btn.textContent = '✓ Zero';
            } else {
                btn.classList.remove('has-override');
                btn.textContent = 'Set Zero';
            }
        }
    }

    // Update clear button state
    const clearBtn = document.getElementById('clear-overrides');
    clearBtn.disabled = Object.keys(offsetOverrides).length === 0;
}

async function setZeroOffset(jointName) {
    try {
        const response = await fetch('/api/offset/set_zero', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ joint_name: jointName })
        });

        if (response.ok) {
            const data = await response.json();
            offsetOverrides[jointName] = data.offset;
            updateOverrideButtons();
            console.log(`Set zero for ${jointName}: offset = ${data.offset}`);
        }
    } catch (error) {
        console.error('Failed to set zero:', error);
    }
}

async function loadTreeData() {
    try {
        const response = await fetch('/api/tree');
        const data = await response.json();
        if (!data.error) {
            kinematicScene.setTreeData(data);
            console.log('Tree data loaded for local calculations');
        }
    } catch (error) {
        console.error('Failed to load tree data:', error);
    }
}

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        console.log('WebSocket connected');
        document.getElementById('ws-status').textContent = 'connected';
        document.getElementById('ws-status').className = 'status-connected';
    };

    ws.onclose = () => {
        console.log('WebSocket disconnected');
        document.getElementById('ws-status').textContent = 'disconnected';
        document.getElementById('ws-status').className = 'status-disconnected';
        setTimeout(connectWebSocket, 3000);
    };

    ws.onmessage = (event) => {
        try {
            const message = JSON.parse(event.data);
            handleMessage(message);
        } catch (error) {
            console.error('Failed to parse message:', error);
        }
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
    };
}

function handleMessage(message) {
    console.log(`Received message: ${message.type}`);

    switch (message.type) {
        case 'scene_init':
            console.log('=== SCENE_INIT received ===');
            console.log('Joints:', message.joints);
            console.log('Nodes:', Object.keys(message.nodes));
            console.log('Z-up:', message.zUp);

            // Clear existing scene (for dynamic tree reload)
            kinematicScene.clear();

            // Set Z-up mode if specified by server
            kinematicScene.setZUp(message.zUp || false);

            // Initial scene setup
            kinematicScene.initFromSceneData(message.nodes);
            jointNames = message.joints || [];
            createJointSliders(jointNames);

            // Reload tree data for local calculations
            loadTreeData();

            // Reload offset overrides (tree may have changed)
            loadOffsetOverrides();

            console.log('=== SCENE_INIT complete ===');
            break;

        case 'scene_update':
            // Update poses from server (only if not in manual mode)
            if (!manualMode) {
                kinematicScene.updateFromSceneData(message.nodes);
            }
            break;
    }
}

function createJointSliders(joints) {
    const container = document.getElementById('joint-sliders');
    container.innerHTML = '';

    for (const jointName of joints) {
        const div = document.createElement('div');
        div.className = 'slider-group';

        // Assume rotator by default, range +-180 degrees in radians
        const min = -Math.PI;
        const max = Math.PI;
        const step = 0.01;

        const hasOverride = offsetOverrides[jointName] !== undefined;

        div.innerHTML = `
            <label>${jointName}</label>
            <div class="slider-row">
                <input type="range"
                       id="slider-${jointName}"
                       min="${min}"
                       max="${max}"
                       step="${step}"
                       value="0">
                <button class="btn-zero ${hasOverride ? 'has-override' : ''}"
                        id="zero-${jointName}">${hasOverride ? '✓ Zero' : 'Set Zero'}</button>
            </div>
            <span class="slider-value" id="value-${jointName}">0°</span>
        `;

        container.appendChild(div);

        const slider = div.querySelector('input');
        const valueSpan = div.querySelector('.slider-value');
        const zeroBtn = div.querySelector('.btn-zero');

        slider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            valueSpan.textContent = (value * 180 / Math.PI).toFixed(1) + '°';

            if (manualMode) {
                // Local update
                kinematicScene.setLocalJointCoord(jointName, value);
                kinematicScene.updateLocal();
            }
        });

        zeroBtn.addEventListener('click', () => {
            setZeroOffset(jointName);
        });
    }

    updateOverrideButtons();
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// Start
init();
