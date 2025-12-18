/**
 * WebKin Application - Main entry point
 */

let scene, camera, renderer, controls;
let kinematicScene;
let ws;
let jointNames = [];
let jointsInfo = {};  // Joint metadata: {name: {type, slider_min, slider_max, axis_scale, axis_offset}}
let manualMode = false;  // false = server control, true = local manual control
let axisOverrides = {};  // Track which joints have axis overrides

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

    // Load axis overrides
    loadAxisOverrides();

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
            const response = await fetch('/api/axis/overrides', { method: 'DELETE' });
            if (response.ok) {
                axisOverrides = {};
                updateOverrideButtons();
                console.log('Cleared all axis overrides');
            }
        } catch (error) {
            console.error('Failed to clear overrides:', error);
        }
    });
}

async function loadAxisOverrides() {
    try {
        const response = await fetch('/api/axis/overrides');
        const data = await response.json();
        axisOverrides = data.overrides || {};
        updateOverrideButtons();
    } catch (error) {
        console.error('Failed to load axis overrides:', error);
    }
}

function updateOverrideButtons() {
    // Update all zero buttons to show override status
    for (const jointName of jointNames) {
        const btn = document.getElementById(`zero-${jointName}`);
        if (btn) {
            const hasOffset = axisOverrides[jointName]?.axis_offset !== undefined;
            if (hasOffset) {
                btn.classList.add('has-override');
                btn.textContent = '✓ Zero';
            } else {
                btn.classList.remove('has-override');
                btn.textContent = 'Set Zero';
            }
        }

        // Update settings button to show if there are any overrides
        const settingsBtn = document.getElementById(`settings-${jointName}`);
        if (settingsBtn) {
            const hasOverrides = axisOverrides[jointName] && Object.keys(axisOverrides[jointName]).length > 0;
            if (hasOverrides) {
                settingsBtn.classList.add('active');
            } else {
                settingsBtn.classList.remove('active');
            }
        }
    }

    // Update clear button state
    const clearBtn = document.getElementById('clear-overrides');
    clearBtn.disabled = Object.keys(axisOverrides).length === 0;
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
            if (!axisOverrides[jointName]) axisOverrides[jointName] = {};
            axisOverrides[jointName].axis_offset = data.offset;
            updateOverrideButtons();
            // Update the offset input field
            const offsetInput = document.getElementById(`offset-${jointName}`);
            if (offsetInput) offsetInput.value = data.offset;
            console.log(`Set zero for ${jointName}: offset = ${data.offset}`);
        }
    } catch (error) {
        console.error('Failed to set zero:', error);
    }
}

function updateSliderAttributes() {
    // Update slider min/max/step and settings inputs from jointsInfo
    for (const jointName of jointNames) {
        const info = jointsInfo[jointName] || {};
        const sliderMin = info.slider_min !== undefined ? info.slider_min : -180;
        const sliderMax = info.slider_max !== undefined ? info.slider_max : 180;
        const axisScale = info.axis_scale !== undefined ? info.axis_scale : 1.0;
        const axisOffset = info.axis_offset !== undefined ? info.axis_offset : 0.0;

        const slider = document.getElementById(`slider-${jointName}`);
        if (slider) {
            slider.min = sliderMin;
            slider.max = sliderMax;
            slider.step = (sliderMax - sliderMin) / 1000;

            // Clamp current value to new range
            const currentValue = parseFloat(slider.value);
            if (currentValue < sliderMin) slider.value = sliderMin;
            if (currentValue > sliderMax) slider.value = sliderMax;
        }

        // Update settings inputs
        const minInput = document.getElementById(`min-${jointName}`);
        const maxInput = document.getElementById(`max-${jointName}`);
        const scaleInput = document.getElementById(`scale-${jointName}`);
        const offsetInput = document.getElementById(`offset-${jointName}`);

        if (minInput) minInput.value = sliderMin;
        if (maxInput) maxInput.value = sliderMax;
        if (scaleInput) scaleInput.value = axisScale;
        if (offsetInput) offsetInput.value = axisOffset;
    }
}

async function applyAxisSettings(jointName) {
    const minInput = document.getElementById(`min-${jointName}`);
    const maxInput = document.getElementById(`max-${jointName}`);
    const scaleInput = document.getElementById(`scale-${jointName}`);
    const offsetInput = document.getElementById(`offset-${jointName}`);

    const params = { joint_name: jointName };

    if (minInput && minInput.value !== '') {
        params.slider_min = parseFloat(minInput.value);
    }
    if (maxInput && maxInput.value !== '') {
        params.slider_max = parseFloat(maxInput.value);
    }
    if (scaleInput && scaleInput.value !== '') {
        params.axis_scale = parseFloat(scaleInput.value);
    }
    if (offsetInput && offsetInput.value !== '') {
        params.axis_offset = parseFloat(offsetInput.value);
    }

    try {
        const response = await fetch('/api/axis/override', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(params)
        });

        if (response.ok) {
            const data = await response.json();
            axisOverrides[jointName] = data.overrides;

            // Update local jointsInfo with new values
            if (!jointsInfo[jointName]) jointsInfo[jointName] = {};
            if (params.slider_min !== undefined) jointsInfo[jointName].slider_min = params.slider_min;
            if (params.slider_max !== undefined) jointsInfo[jointName].slider_max = params.slider_max;
            if (params.axis_scale !== undefined) jointsInfo[jointName].axis_scale = params.axis_scale;
            if (params.axis_offset !== undefined) jointsInfo[jointName].axis_offset = params.axis_offset;

            // Update slider min/max attributes
            const slider = document.getElementById(`slider-${jointName}`);
            if (slider) {
                const info = jointsInfo[jointName];
                const sliderMin = info.slider_min !== undefined ? info.slider_min : -180;
                const sliderMax = info.slider_max !== undefined ? info.slider_max : 180;
                slider.min = sliderMin;
                slider.max = sliderMax;
                slider.step = (sliderMax - sliderMin) / 1000;

                // Clamp current value to new range
                const currentValue = parseFloat(slider.value);
                if (currentValue < sliderMin) slider.value = sliderMin;
                if (currentValue > sliderMax) slider.value = sliderMax;

                // In manual mode, trigger update with new scale
                if (manualMode) {
                    const currentScale = info.axis_scale !== undefined ? info.axis_scale : 1.0;
                    const physicalValue = parseFloat(slider.value) * currentScale;
                    kinematicScene.setLocalJointCoord(jointName, physicalValue);
                    kinematicScene.updateLocal();
                }
            }

            updateOverrideButtons();
            console.log(`Applied settings for ${jointName}:`, params);
        }
    } catch (error) {
        console.error('Failed to apply axis settings:', error);
    }
}

function toggleAxisSettings(jointName) {
    const settings = document.getElementById(`axis-settings-${jointName}`);
    if (settings) {
        settings.classList.toggle('visible');
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
            console.log('JointsInfo:', message.jointsInfo);
            console.log('Nodes:', Object.keys(message.nodes));
            console.log('Z-up:', message.zUp);

            // Clear existing scene (for dynamic tree reload)
            kinematicScene.clear();

            // Set Z-up mode if specified by server
            kinematicScene.setZUp(message.zUp || false);

            // Initial scene setup
            kinematicScene.initFromSceneData(message.nodes);
            jointNames = message.joints || [];
            jointsInfo = message.jointsInfo || {};
            createJointSliders(jointNames);

            // Reload tree data for local calculations
            loadTreeData();

            // Reload axis overrides (tree may have changed)
            loadAxisOverrides();

            console.log('=== SCENE_INIT complete ===');
            break;

        case 'scene_update':
            // Update poses from server (only if not in manual mode)
            if (!manualMode) {
                kinematicScene.updateFromSceneData(message.nodes);
            }
            // Update jointsInfo if provided (e.g., after axis override change)
            if (message.jointsInfo) {
                const jointsInfoChanged = JSON.stringify(jointsInfo) !== JSON.stringify(message.jointsInfo);
                jointsInfo = message.jointsInfo;
                updateSliderAttributes();
                // If axis params changed, force update even in manual mode
                if (jointsInfoChanged && manualMode) {
                    kinematicScene.updateFromSceneData(message.nodes);
                }
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

        // Get joint info from server, use defaults if not available
        const info = jointsInfo[jointName] || {};
        const sliderMin = info.slider_min !== undefined ? info.slider_min : -180;
        const sliderMax = info.slider_max !== undefined ? info.slider_max : 180;
        const axisScale = info.axis_scale !== undefined ? info.axis_scale : 1.0;
        const axisOffset = info.axis_offset !== undefined ? info.axis_offset : 0.0;

        // Calculate step based on range
        const range = sliderMax - sliderMin;
        const step = range / 1000;  // 1000 steps across the range

        const hasOffset = axisOverrides[jointName]?.axis_offset !== undefined;
        const hasOverrides = axisOverrides[jointName] && Object.keys(axisOverrides[jointName]).length > 0;

        div.innerHTML = `
            <label>${jointName}</label>
            <div class="slider-row">
                <input type="range"
                       id="slider-${jointName}"
                       min="${sliderMin}"
                       max="${sliderMax}"
                       step="${step}"
                       value="0">
                <button class="btn-zero ${hasOffset ? 'has-override' : ''}"
                        id="zero-${jointName}">${hasOffset ? '✓ Zero' : 'Set Zero'}</button>
                <button class="btn-settings ${hasOverrides ? 'active' : ''}"
                        id="settings-${jointName}">⚙</button>
            </div>
            <span class="slider-value" id="value-${jointName}">0</span>
            <div class="axis-settings" id="axis-settings-${jointName}">
                <div class="axis-settings-row">
                    <label>Min:</label>
                    <input type="number" id="min-${jointName}" value="${sliderMin}" step="any">
                    <label>Max:</label>
                    <input type="number" id="max-${jointName}" value="${sliderMax}" step="any">
                </div>
                <div class="axis-settings-row">
                    <label>Scale:</label>
                    <input type="number" id="scale-${jointName}" value="${axisScale}" step="any">
                    <label>Offset:</label>
                    <input type="number" id="offset-${jointName}" value="${axisOffset}" step="any">
                </div>
                <button class="btn-apply" id="apply-${jointName}">Apply</button>
            </div>
        `;

        container.appendChild(div);

        const slider = div.querySelector(`#slider-${jointName}`);
        const valueSpan = div.querySelector(`#value-${jointName}`);
        const zeroBtn = div.querySelector(`#zero-${jointName}`);
        const settingsBtn = div.querySelector(`#settings-${jointName}`);
        const applyBtn = div.querySelector(`#apply-${jointName}`);

        slider.addEventListener('input', (e) => {
            const userValue = parseFloat(e.target.value);
            // Display value in user units (no degree symbol - units may vary)
            valueSpan.textContent = userValue.toFixed(2);

            if (manualMode) {
                // Get current axis_scale from jointsInfo (may have been updated)
                const info = jointsInfo[jointName] || {};
                const currentScale = info.axis_scale !== undefined ? info.axis_scale : 1.0;
                // Transform to physical coordinate: userValue * scale
                // Note: offset is applied in kinematic calculation
                const physicalValue = userValue * currentScale;
                kinematicScene.setLocalJointCoord(jointName, physicalValue);
                kinematicScene.updateLocal();
            }
        });

        zeroBtn.addEventListener('click', () => {
            setZeroOffset(jointName);
        });

        settingsBtn.addEventListener('click', () => {
            toggleAxisSettings(jointName);
        });

        applyBtn.addEventListener('click', () => {
            applyAxisSettings(jointName);
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
