/**
 * Kinematic Scene Renderer for Three.js
 * Only renders - all calculations done on server
 */

class SceneNode {
    constructor(name, modelData) {
        this.name = name;
        this.group = new THREE.Group();
        this.group.name = name;
        this.mesh = null;

        if (modelData) {
            this.createMesh(modelData);
        }
    }

    createMesh(modelData) {
        const type = modelData.type || 'cube';
        let geometry;
        const material = new THREE.MeshPhongMaterial({
            color: modelData.color || 0x9e9e9e,
            flatShading: false
        });

        switch (type) {
            case 'cube':
                const size = modelData.size || [10, 10, 10];
                geometry = new THREE.BoxGeometry(size[0], size[1], size[2]);
                break;

            case 'sphere':
                const radius = modelData.radius || 10;
                geometry = new THREE.SphereGeometry(radius, 32, 32);
                break;

            case 'cylinder':
                const cylRadius = modelData.radius || 10;
                const cylHeight = modelData.height || 20;
                geometry = new THREE.CylinderGeometry(cylRadius, cylRadius, cylHeight, 32);
                break;

            case 'cone':
                const r1 = modelData.radius1 || 10;
                const r2 = modelData.radius2 || 5;
                const coneHeight = modelData.height || 20;
                geometry = new THREE.CylinderGeometry(r2, r1, coneHeight, 32);
                break;

            case 'file':
            case 'stl':
                this.loadExternalModel(modelData);
                // Placeholder while loading
                geometry = new THREE.BoxGeometry(1, 1, 1);
                material.visible = false;
                break;

            default:
                geometry = new THREE.BoxGeometry(10, 10, 10);
        }

        this.mesh = new THREE.Mesh(geometry, material);
        this.group.add(this.mesh);
    }

    loadExternalModel(modelData) {
        const path = modelData.path;
        if (!path) return;

        const url = path.startsWith('/') ? path : `/static/models/${path}`;
        const extension = path.split('.').pop().toLowerCase();

        if (extension === 'stl') {
            const loader = new THREE.STLLoader();
            loader.load(
                url,
                (geometry) => {
                    const material = new THREE.MeshPhongMaterial({
                        color: modelData.color || 0x9e9e9e,
                        flatShading: false
                    });
                    const mesh = new THREE.Mesh(geometry, material);

                    if (modelData.scale) {
                        if (Array.isArray(modelData.scale)) {
                            mesh.scale.set(modelData.scale[0], modelData.scale[1], modelData.scale[2]);
                        } else {
                            mesh.scale.setScalar(modelData.scale);
                        }
                    }

                    if (modelData.rotation) {
                        const r = modelData.rotation;
                        mesh.rotation.set(
                            r[0] * Math.PI / 180,
                            r[1] * Math.PI / 180,
                            r[2] * Math.PI / 180
                        );
                    }

                    if (modelData.offset) {
                        mesh.position.set(modelData.offset[0], modelData.offset[1], modelData.offset[2]);
                    }

                    if (this.mesh) {
                        this.group.remove(this.mesh);
                    }
                    this.mesh = mesh;
                    this.group.add(mesh);

                    console.log(`Loaded STL: ${path}`);
                },
                undefined,
                (error) => console.error(`Failed to load STL ${path}:`, error)
            );
        }
    }

    setPose(pose) {
        if (pose.position) {
            this.group.position.set(pose.position[0], pose.position[1], pose.position[2]);
        }
        if (pose.orientation) {
            this.group.quaternion.set(
                pose.orientation[0],
                pose.orientation[1],
                pose.orientation[2],
                pose.orientation[3]
            );
        }
    }
}


class KinematicScene {
    constructor(scene) {
        this.scene = scene;
        this.nodes = {};
        this.treeData = null;  // Original tree structure for local calculations
    }

    initFromSceneData(sceneData) {
        // Create nodes from scene data
        for (const [name, data] of Object.entries(sceneData)) {
            if (!this.nodes[name]) {
                const node = new SceneNode(name, data.model);
                this.nodes[name] = node;
                this.scene.add(node.group);
            }
            // Apply initial pose
            if (data.pose) {
                this.nodes[name].setPose(data.pose);
            }
        }

        console.log(`Scene initialized with ${Object.keys(this.nodes).length} nodes`);
    }

    setTreeData(data) {
        this.treeData = data;
        this.jointCoords = {};
        // Initialize joint coords to 0
        this._collectJoints(data);
    }

    _collectJoints(node) {
        if (node.type === 'rotator' || node.type === 'actuator') {
            this.jointCoords[node.name] = 0;
        }
        for (const child of (node.children || [])) {
            this._collectJoints(child);
        }
    }

    updateFromSceneData(sceneData) {
        for (const [name, data] of Object.entries(sceneData)) {
            if (this.nodes[name] && data.pose) {
                this.nodes[name].setPose(data.pose);
            }
        }
    }

    // Local calculation methods
    setLocalJointCoord(name, value) {
        if (this.jointCoords.hasOwnProperty(name)) {
            this.jointCoords[name] = value;
        }
    }

    updateLocal() {
        if (!this.treeData) return;
        this._computePoses(this.treeData, { position: [0,0,0], orientation: [0,0,0,1] });
    }

    _computePoses(node, parentPose) {
        // Compose parent * local
        const localPose = node.pose || { position: [0,0,0], orientation: [0,0,0,1] };
        const globalPose = this._composePoses(parentPose, localPose);

        // Apply to scene node
        if (this.nodes[node.name]) {
            this.nodes[node.name].setPose(globalPose);
        }

        // Compute child parent pose (with joint transform)
        let childParentPose = globalPose;
        if (node.type === 'rotator' || node.type === 'actuator') {
            const jointTransform = this._getJointTransform(node);
            childParentPose = this._composePoses(globalPose, jointTransform);
        }

        // Recurse children
        for (const child of (node.children || [])) {
            this._computePoses(child, childParentPose);
        }
    }

    _getJointTransform(node) {
        const coord = this.jointCoords[node.name] || 0;
        const axis = node.axis || [0, 0, 1];

        if (node.type === 'rotator') {
            const q = this._quatFromAxisAngle(axis, coord);
            return { position: [0, 0, 0], orientation: q };
        } else if (node.type === 'actuator') {
            return {
                position: [axis[0] * coord, axis[1] * coord, axis[2] * coord],
                orientation: [0, 0, 0, 1]
            };
        }
        return { position: [0, 0, 0], orientation: [0, 0, 0, 1] };
    }

    _quatFromAxisAngle(axis, angle) {
        const half = angle / 2;
        const s = Math.sin(half);
        return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(half)];
    }

    _composePoses(p1, p2) {
        // p1 * p2
        const pos = this._addVec(p1.position, this._rotateVec(p1.orientation, p2.position));
        const ori = this._mulQuat(p1.orientation, p2.orientation);
        return { position: pos, orientation: ori };
    }

    _addVec(a, b) {
        return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
    }

    _rotateVec(q, v) {
        // Rotate vector v by quaternion q
        const qv = [v[0], v[1], v[2], 0];
        const qConj = [-q[0], -q[1], -q[2], q[3]];
        const result = this._mulQuat(this._mulQuat(q, qv), qConj);
        return [result[0], result[1], result[2]];
    }

    _mulQuat(a, b) {
        return [
            a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
            a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
            a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
            a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]
        ];
    }

    clear() {
        for (const node of Object.values(this.nodes)) {
            this.scene.remove(node.group);
        }
        this.nodes = {};
    }
}
