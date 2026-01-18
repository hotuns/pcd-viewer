import * as THREE from "three";

type Vec3 = { x: number; y: number; z: number };
type QuaternionLike = { x: number; y: number; z: number; w: number };

// Three.js 默认 Y 轴朝上、Z 轴朝向观察者，ROS 机体系为 X(前)/Y(左)/Z(上)
// 将 ROS 向量转换为 Three.js 视图：Xv = Xr, Yv = Zr, Zv = -Yr
const BODY_TO_VIEWER_MATRIX = new THREE.Matrix4().set(
  1,  0,  0, 0,
  0,  0,  1, 0,
  0, -1,  0, 0,
  0,  0,  0, 1
);
const VIEWER_TO_BODY_MATRIX = BODY_TO_VIEWER_MATRIX.clone().invert();
const MODEL_ALIGNMENT = new THREE.Quaternion();

const tempVec = new THREE.Vector3();
const tempMatrix = new THREE.Matrix4();

export const convertBodyPositionToViewer = (pos: Vec3) => {
  tempVec.set(pos.x, pos.y, pos.z).applyMatrix4(BODY_TO_VIEWER_MATRIX);
  return { x: tempVec.x, y: tempVec.y, z: tempVec.z };
};

export const convertViewerPositionToBody = (pos: Vec3) => {
  tempVec.set(pos.x, pos.y, pos.z).applyMatrix4(VIEWER_TO_BODY_MATRIX);
  return { x: tempVec.x, y: tempVec.y, z: tempVec.z };
};

export const convertBodyOrientationToViewer = (orientation?: QuaternionLike | null) => {
  if (!orientation) return null;
  const bodyQuat = new THREE.Quaternion(
    orientation.x ?? 0,
    orientation.y ?? 0,
    orientation.z ?? 0,
    orientation.w ?? 1
  ).normalize();
  const bodyMatrix = tempMatrix.makeRotationFromQuaternion(bodyQuat);
  const viewerMatrix = BODY_TO_VIEWER_MATRIX.clone().multiply(bodyMatrix).multiply(VIEWER_TO_BODY_MATRIX);
  const viewerQuat = new THREE.Quaternion().setFromRotationMatrix(viewerMatrix);
  return {
    x: viewerQuat.x,
    y: viewerQuat.y,
    z: viewerQuat.z,
    w: viewerQuat.w,
  };
};

export const getModelAlignmentQuaternion = () => new THREE.Quaternion();
