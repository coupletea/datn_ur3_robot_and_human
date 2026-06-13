#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import mediapipe as mp
import numpy as np
from geometry_msgs.msg import Pose, PoseArray

try:
    from pylibfreenect2 import (
        CpuPacketPipeline,
        Frame,
        FrameType,
        Freenect2,
        OpenGLPacketPipeline,
        Registration,
        SyncMultiFrameListener,
    )
    _PYLIBFREENECT2_IMPORT_ERROR = None
except ImportError as exc:
    Frame = None
    FrameType = None
    Freenect2 = None
    CpuPacketPipeline = None
    OpenGLPacketPipeline = None
    Registration = None
    SyncMultiFrameListener = None
    _PYLIBFREENECT2_IMPORT_ERROR = exc

try:
    from pylibfreenect2.libfreenect2 import CudaPacketPipeline as _CudaPacketPipeline
except ImportError:
    _CudaPacketPipeline = None

try:
    from ultralytics import YOLO
    _ULTRALYTICS_IMPORT_ERROR = None
except ImportError as exc:
    YOLO = None
    _ULTRALYTICS_IMPORT_ERROR = exc


Point3D = Tuple[float, float, float]
Pixel2D = Tuple[int, int]
SkeletonDict = Dict[str, Point3D]
SkeletonPixels = Dict[str, Pixel2D]
ConnectionPair = Tuple[str, str]

POSE_LANDMARK_IDS = [0, 11, 12, 13, 14, 15, 16, 23, 24]
LEFT_HAND_JOINT_IDS = [100 + index for index in range(21)]
RIGHT_HAND_JOINT_IDS = [200 + index for index in range(21)]
POSE_ARRAY_JOINT_IDS = POSE_LANDMARK_IDS + LEFT_HAND_JOINT_IDS + RIGHT_HAND_JOINT_IDS
BODY_LANDMARK_ORDER = ["pose_%d" % index for index in POSE_LANDMARK_IDS]
LEFT_HAND_LANDMARK_ORDER = ["left_hand_%d" % index for index in range(21)]
RIGHT_HAND_LANDMARK_ORDER = ["right_hand_%d" % index for index in range(21)]
LANDMARK_ORDER = (
    BODY_LANDMARK_ORDER
    + LEFT_HAND_LANDMARK_ORDER
    + RIGHT_HAND_LANDMARK_ORDER
)

HAND_CONNECTION_INDEXES = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
]
BODY_CONNECTIONS: List[ConnectionPair] = [
    ("pose_11", "pose_13"),
    ("pose_13", "pose_15"),
    ("pose_12", "pose_14"),
    ("pose_14", "pose_16"),
    ("pose_11", "pose_12"),
    ("pose_11", "pose_23"),
    ("pose_12", "pose_24"),
    ("pose_23", "pose_24"),
]
LEFT_HAND_CONNECTIONS: List[ConnectionPair] = [
    ("left_hand_%d" % a, "left_hand_%d" % b)
    for a, b in HAND_CONNECTION_INDEXES
]
RIGHT_HAND_CONNECTIONS: List[ConnectionPair] = [
    ("right_hand_%d" % a, "right_hand_%d" % b)
    for a, b in HAND_CONNECTION_INDEXES
]
HAND_TO_BODY_CONNECTIONS: List[ConnectionPair] = [
    ("pose_15", "left_hand_0"),
    ("pose_16", "right_hand_0"),
]
DEFAULT_CONNECTION_PAIRS: List[ConnectionPair] = (
    BODY_CONNECTIONS
    + HAND_TO_BODY_CONNECTIONS
    + LEFT_HAND_CONNECTIONS
    + RIGHT_HAND_CONNECTIONS
)

# Compatibility aliases for older code/config. New pipeline uses LANDMARK_ORDER.
DEFAULT_TRACKED_JOINT_IDS = POSE_LANDMARK_IDS
INVALID_COORD = float("nan")


class KinectFrameTimeout(RuntimeError):
    """Bao loi khi Kinect khong tra frame moi trong timeout."""


def _kinect_serial_to_text(serial: Any) -> str:
    """Doi serial Kinect tu bytes/str ve chuoi de so khop config."""
    if isinstance(serial, bytes):
        return serial.decode("utf-8", errors="ignore")
    return str(serial)


def _normalize_requested_serial(serial: Optional[str]) -> str:
    text = _kinect_serial_to_text(serial).strip() if serial is not None else ""
    if text.lower() in ("none", "null", "serial", "serial_front", "serial_back", "front", "back"):
        return ""
    return text


def _make_packet_pipeline(packet_pipeline: Optional[str]):
    pipeline_name = str(packet_pipeline or "").strip().lower()
    if not pipeline_name or pipeline_name in ("default", "auto"):
        return None
    if pipeline_name == "cpu":
        if CpuPacketPipeline is None:
            raise RuntimeError("KINECT_PACKET_PIPELINE_UNAVAILABLE pipeline=cpu")
        return CpuPacketPipeline()
    if pipeline_name == "opengl":
        if OpenGLPacketPipeline is None:
            raise RuntimeError("KINECT_PACKET_PIPELINE_UNAVAILABLE pipeline=opengl")
        return OpenGLPacketPipeline()
    if pipeline_name in ("cuda", "cuda_kde"):
        if _CudaPacketPipeline is not None:
            return _CudaPacketPipeline()
        # pylibfreenect2 not compiled with CUDA bindings — fall back to default
        # (libfreenect2 C++ auto-selects CUDA when built with CUDA support)
        import warnings
        warnings.warn(
            "CudaPacketPipeline not available in Python bindings; "
            "falling back to default (C++ auto-select). "
            "Rebuild pylibfreenect2 with CUDA for explicit control.",
            RuntimeWarning,
        )
        return None
    raise RuntimeError("KINECT_PACKET_PIPELINE_UNKNOWN pipeline=%s" % pipeline_name)


def is_hand_landmark(name: Any) -> bool:
    """Tra ve True neu landmark name thuoc ban tay."""
    text = str(name)
    return text.startswith("left_hand_") or text.startswith("right_hand_")


def is_body_landmark(name: Any) -> bool:
    """Tra ve True neu landmark name thuoc body pose."""
    return str(name).startswith("pose_")


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


def _is_finite_point(point: Sequence[float]) -> bool:
    return len(point) == 3 and all(math.isfinite(float(value)) for value in point)


def pose_is_valid(pose_msg: Pose) -> bool:
    """Kiem tra Pose skeleton co toa do thuc hay la landmark NaN."""
    return (
        math.isfinite(pose_msg.position.x)
        and math.isfinite(pose_msg.position.y)
        and math.isfinite(pose_msg.position.z)
    )


def initialize_camera(
    device_index: int = 0,
    serial: Optional[str] = None,
    packet_pipeline: Optional[str] = None,
    color_only: bool = False,
):
    """Khoi tao Kinect v2 va listener theo sensor mode.

    Args:
        device_index: Thu tu Kinect khi khong chon theo serial.
        serial: Serial Kinect can mo. Neu co thi uu tien hon device_index.
        packet_pipeline: `default`, `cpu`, `opengl`, hoac `cuda`.
        color_only: Neu True chi dang ky color listener, khong tao registration.

    Return:
        Tuple `(freenect, device, listener, registration)`. `registration` la None
        trong color-only mode.

    Raises:
        RuntimeError khi thieu pylibfreenect2 hoac khong tim thay Kinect.
    """
    if Freenect2 is None:
        raise RuntimeError(
            "pylibfreenect2 is required for Kinect skeleton tracking: %s"
            % _PYLIBFREENECT2_IMPORT_ERROR
        )

    freenect = Freenect2()
    num_devices = freenect.enumerateDevices()
    if num_devices == 0:
        raise RuntimeError("KINECT_NOT_FOUND No Kinect v2 device found")

    raw_serials = [
        freenect.getDeviceSerialNumber(index)
        for index in range(num_devices)
    ]
    available_serials = [_kinect_serial_to_text(item) for item in raw_serials]
    requested_serial = _normalize_requested_serial(serial)
    if requested_serial:
        if requested_serial not in available_serials:
            raise RuntimeError(
                "KINECT_SERIAL_NOT_FOUND serial=%s available=%s"
                % (requested_serial, ",".join(available_serials))
            )
        selected_index = available_serials.index(requested_serial)
    else:
        selected_index = int(device_index)
        if selected_index < 0 or selected_index >= num_devices:
            raise RuntimeError(
                "KINECT_DEVICE_INDEX_OUT_OF_RANGE index=%d count=%d available=%s"
                % (selected_index, num_devices, ",".join(available_serials))
            )
    selected_serial = raw_serials[selected_index]
    selected_serial_text = available_serials[selected_index]

    try:
        pipeline = _make_packet_pipeline(packet_pipeline)
        if pipeline is None:
            device = freenect.openDevice(selected_serial)
        else:
            device = freenect.openDevice(selected_serial, pipeline)
    except Exception as exc:
        raise RuntimeError(
            "KINECT_OPEN_FAILED serial=%s error=%s" % (selected_serial_text, exc)
        )

    if color_only:
        listener = SyncMultiFrameListener(FrameType.Color)
        device.setColorFrameListener(listener)
        device.start()
        return freenect, device, listener, None

    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Depth)
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)
    device.start()

    registration = Registration(
        device.getIrCameraParams(),
        device.getColorCameraParams(),
    )
    return freenect, device, listener, registration


def _registered_bgr_image(registered: Any) -> np.ndarray:
    """Doi Kinect registered frame thanh anh BGR cho OpenCV."""
    image = registered.asarray(dtype=np.uint8)

    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    if image.ndim == 3 and image.shape[2] == 3:
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def _color_bgr_image(color_frame: Any) -> np.ndarray:
    """Doi Kinect color frame thanh anh BGR cho OpenCV."""
    image = color_frame.asarray(dtype=np.uint8)

    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    if image.ndim == 3 and image.shape[2] == 3:
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def read_color_frame_only(listener, frame_timeout_ms: Optional[int]):
    """Doc color frame Kinect khong kem depth.

    Return:
        `(color_bgr, color_rgb)`.

    Raises:
        KinectFrameTimeout neu khong co color frame trong timeout.
        RuntimeError neu pylibfreenect2 khong kha dung.
    """
    if FrameType is None:
        raise RuntimeError(
            "pylibfreenect2 is required for Kinect skeleton tracking: %s"
            % _PYLIBFREENECT2_IMPORT_ERROR
        )

    frames = None
    try:
        if frame_timeout_ms is None or int(frame_timeout_ms) < 0:
            frames = listener.waitForNewFrame()
        else:
            frames = listener.waitForNewFrame(milliseconds=int(frame_timeout_ms))

        if frames is None:
            raise KinectFrameTimeout(
                "No Kinect color frame received within %d ms" % int(frame_timeout_ms)
            )

        color_frame = frames[FrameType.Color]
        color_bgr = _color_bgr_image(color_frame)
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        return color_bgr, color_rgb
    finally:
        if frames is not None:
            listener.release(frames)


def read_registered_frames(listener, registration, frame_timeout_ms: Optional[int]):
    """Doc frame Kinect va tao anh registered BGR/RGB kem depth undistorted.

    Return:
        `(registered_bgr, registered_rgb, undistorted, registered)`.

    Raises:
        KinectFrameTimeout neu khong co frame trong timeout.
        RuntimeError neu pylibfreenect2 khong kha dung.
    """
    if Frame is None or FrameType is None:
        raise RuntimeError(
            "pylibfreenect2 is required for Kinect skeleton tracking: %s"
            % _PYLIBFREENECT2_IMPORT_ERROR
        )

    frames = None
    try:
        if frame_timeout_ms is None or int(frame_timeout_ms) < 0:
            frames = listener.waitForNewFrame()
        else:
            frames = listener.waitForNewFrame(milliseconds=int(frame_timeout_ms))

        if frames is None:
            raise KinectFrameTimeout(
                "No Kinect frame received within %d ms" % int(frame_timeout_ms)
            )

        color_frame = frames[FrameType.Color]
        depth_frame = frames[FrameType.Depth]
        undistorted = Frame(512, 424, 4)
        registered = Frame(512, 424, 4)
        registration.apply(color_frame, depth_frame, undistorted, registered)

        registered_bgr = _registered_bgr_image(registered)
        registered_rgb = cv2.cvtColor(registered_bgr, cv2.COLOR_BGR2RGB)
        return registered_bgr, registered_rgb, undistorted, registered
    finally:
        if frames is not None:
            listener.release(frames)


def load_yolo_segment_model(
    yolo_model_path: str,
    device: str = "cpu",
    gpu_memory_fraction: Optional[float] = 0.45,
):
    """Load YOLO segmentation model.

    Raises:
        RuntimeError neu chua cai `ultralytics` hoac model load that bai.
    """
    if YOLO is None:
        raise RuntimeError(
            "ultralytics is required when enable_yolo_person_segmentation=true: %s"
            % _ULTRALYTICS_IMPORT_ERROR
        )

    model = YOLO(yolo_model_path)
    if device:
        try:
            model.to(device)
        except Exception:
            # Some ultralytics versions only accept device in predict().
            pass
    if device and str(device).startswith("cuda") and gpu_memory_fraction is not None:
        try:
            import torch
            torch.cuda.set_per_process_memory_fraction(float(gpu_memory_fraction), device=0)
        except Exception:
            pass
    setattr(model, "_ur3_hrc_device", device)
    return model


def _to_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def run_yolo_person_segmentation(
    model,
    rgb_image: np.ndarray,
    conf_threshold: float,
    iou_threshold: float,
    person_class_id: int = 0,
) -> List[Dict[str, Any]]:
    """Chay YOLO segmentation va tra ve mask/bbox person."""
    height, width = rgb_image.shape[:2]
    device = getattr(model, "_ur3_hrc_device", None)
    predict_kwargs = {
        "source": rgb_image,
        "conf": float(conf_threshold),
        "iou": float(iou_threshold),
        "verbose": False,
    }
    if device:
        predict_kwargs["device"] = device

    results = model.predict(**predict_kwargs)
    if not results:
        return []

    result = results[0]
    if result.masks is None or result.boxes is None:
        return []

    masks_data = _to_numpy(result.masks.data)
    classes = _to_numpy(result.boxes.cls).astype(int)
    scores = _to_numpy(result.boxes.conf).astype(float)
    boxes = _to_numpy(result.boxes.xyxy).astype(float)

    detections: List[Dict[str, Any]] = []
    for index, class_id in enumerate(classes):
        if int(class_id) != int(person_class_id):
            continue
        if index >= len(masks_data):
            continue

        mask = masks_data[index]
        if mask.shape[:2] != (height, width):
            mask = cv2.resize(mask.astype(np.float32), (width, height), interpolation=cv2.INTER_LINEAR)
        binary_mask = mask > 0.5
        detections.append(
            {
                "mask": binary_mask.astype(np.uint8),
                "box": tuple(float(v) for v in boxes[index]),
                "score": float(scores[index]),
            }
        )
    return detections


def select_best_person_mask(
    person_detections: Sequence[Dict[str, Any]],
    image_shape: Sequence[int],
    min_area_ratio: float,
    prefer_center: bool = True,
    select_mode: str = "legacy",
    depth_map: Optional[np.ndarray] = None,
    min_depth_px: int = 50,
):
    """Chon person mask theo legacy score hoac median depth gan nhat."""
    if not person_detections:
        return None, None

    height, width = image_shape[:2]
    image_area = float(max(1, height * width))

    if str(select_mode).strip().lower() == "nearest":
        valid_depth_map = (
            isinstance(depth_map, np.ndarray)
            and depth_map.shape[:2] == (height, width)
        )
        if valid_depth_map:
            nearest_candidates = []
            required_depth_px = max(1, int(min_depth_px))
            for detection in person_detections:
                mask = detection["mask"].astype(bool)
                if mask.shape[:2] != (height, width):
                    continue
                area_ratio = float(np.count_nonzero(mask)) / image_area
                if area_ratio < min_area_ratio:
                    continue

                values = depth_map[mask]
                values = values[np.isfinite(values) & (values > 0)]
                if values.size < required_depth_px:
                    continue
                nearest_candidates.append((float(np.median(values)), detection))

            if nearest_candidates:
                min_depth = min(candidate[0] for candidate in nearest_candidates)
                closest = [
                    detection
                    for depth, detection in nearest_candidates
                    if math.isclose(depth, min_depth, rel_tol=1e-9, abs_tol=1e-6)
                ]
                if len(closest) == 1:
                    best_detection = closest[0]
                    return best_detection["mask"].astype(np.uint8), best_detection

    # Legacy selector and nearest fallback. Keep scoring unchanged for compatibility.
    center_x = width / 2.0
    center_y = height / 2.0
    max_center_distance = math.sqrt(center_x * center_x + center_y * center_y)

    best_detection = None
    best_score = -float("inf")
    for detection in person_detections:
        mask = detection["mask"].astype(bool)
        area_ratio = float(np.count_nonzero(mask)) / image_area
        if area_ratio < min_area_ratio:
            continue

        score = area_ratio
        if prefer_center:
            x1, y1, x2, y2 = detection["box"]
            box_center_x = (x1 + x2) / 2.0
            box_center_y = (y1 + y2) / 2.0
            center_distance = math.sqrt((box_center_x - center_x) ** 2 + (box_center_y - center_y) ** 2)
            center_bonus = 1.0 - min(1.0, center_distance / max_center_distance)
            score = area_ratio + 0.25 * center_bonus + 0.10 * float(detection.get("score", 0.0))

        if score > best_score:
            best_score = score
            best_detection = detection

    if best_detection is None:
        return None, None
    return best_detection["mask"].astype(np.uint8), best_detection


def dilate_mask(mask: Optional[np.ndarray], dilate_px: int) -> Optional[np.ndarray]:
    """Noi person mask de landmark sat bien khong bi loai oan."""
    if mask is None:
        return None
    if dilate_px <= 0:
        return mask.astype(np.uint8)
    size = int(dilate_px) * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.dilate(mask.astype(np.uint8), kernel)


def landmark_inside_mask(
    x_px: int,
    y_px: int,
    mask: Optional[np.ndarray],
    margin_px: int = 4,
    min_ratio: float = 0.20,
) -> bool:
    """Kiem tra pixel landmark co nam trong person mask khong."""
    if mask is None:
        return True

    height, width = mask.shape[:2]
    if x_px < 0 or y_px < 0 or x_px >= width or y_px >= height:
        return False

    margin = max(0, int(margin_px))
    x1 = max(0, x_px - margin)
    x2 = min(width, x_px + margin + 1)
    y1 = max(0, y_px - margin)
    y2 = min(height, y_px + margin + 1)
    patch = mask[y1:y2, x1:x2]
    if patch.size <= 0:
        return False
    return float(np.count_nonzero(patch)) / float(patch.size) >= float(min_ratio)


def load_mediapipe_holistic(
    model_complexity: int = 0,
    min_detection_confidence: float = 0.5,
    min_tracking_confidence: float = 0.5,
    enable_segmentation: bool = False,
):
    """Khoi tao MediaPipe Holistic cho body va hands."""
    return mp.solutions.holistic.Holistic(
        static_image_mode=False,
        model_complexity=int(model_complexity),
        smooth_landmarks=True,
        enable_segmentation=bool(enable_segmentation),
        refine_face_landmarks=False,
        min_detection_confidence=float(min_detection_confidence),
        min_tracking_confidence=float(min_tracking_confidence),
    )


def run_holistic(holistic, rgb_image: np.ndarray):
    """Chay MediaPipe Holistic tren anh RGB."""
    return holistic.process(rgb_image)


def normalized_landmark_to_pixel(lm, image_width: int, image_height: int) -> Optional[Pixel2D]:
    """Doi landmark normalized `(x,y)` sang pixel da clamp."""
    if not math.isfinite(float(lm.x)) or not math.isfinite(float(lm.y)):
        return None
    x_px = int(np.clip(float(lm.x) * image_width, 0, image_width - 1))
    y_px = int(np.clip(float(lm.y) * image_height, 0, image_height - 1))
    return x_px, y_px


def camera_point_from_registration(registration, undistorted, x_px: int, y_px: int) -> Optional[Point3D]:
    """Lay diem 3D tu depth registration theo quy uoc camera frame cua project."""
    point_3d = registration.getPointXYZ(undistorted, int(y_px), int(x_px))
    raw_x, raw_y, raw_z = point_3d
    if not _is_finite_point((raw_x, raw_y, raw_z)):
        return None

    # Keep the coordinate convention from the original project:
    # Kinect registered point (x, y, z) -> camera frame point (-x, z, -y).
    return -float(raw_x), float(raw_z), -float(raw_y)


def _camera_point_from_registration(registration, undistorted, x_px: int, y_px: int) -> Optional[Point3D]:
    return camera_point_from_registration(registration, undistorted, x_px, y_px)


def extract_pose_landmarks_3d(
    results,
    registration,
    undistorted,
    person_mask: Optional[np.ndarray],
    image_width: int,
    image_height: int,
    min_visibility: float,
    pose_ids: Optional[Iterable[int]] = None,
    mask_margin_px: int = 4,
    mask_min_ratio: float = 0.20,
) -> Tuple[SkeletonDict, SkeletonPixels]:
    """Lay pose landmarks 3D da loc visibility va person mask."""
    if not getattr(results, "pose_landmarks", None):
        return {}, {}

    wanted_ids = list(pose_ids or POSE_LANDMARK_IDS)
    points_3d: SkeletonDict = {}
    pixels: SkeletonPixels = {}
    landmarks = results.pose_landmarks.landmark

    for index in wanted_ids:
        if index >= len(landmarks):
            continue
        lm = landmarks[index]
        if getattr(lm, "visibility", 1.0) < min_visibility:
            continue
        pixel = normalized_landmark_to_pixel(lm, image_width, image_height)
        if pixel is None:
            continue
        x_px, y_px = pixel
        if not landmark_inside_mask(x_px, y_px, person_mask, mask_margin_px, mask_min_ratio):
            continue
        point = camera_point_from_registration(registration, undistorted, x_px, y_px)
        if point is None:
            continue
        name = "pose_%d" % index
        points_3d[name] = point
        pixels[name] = pixel

    return points_3d, pixels


def extract_hand_landmarks_3d(
    hand_landmarks,
    hand_prefix: str,
    registration,
    undistorted,
    person_mask: Optional[np.ndarray],
    image_width: int,
    image_height: int,
    min_hand_presence: int,
    mask_margin_px: int = 3,
    mask_min_ratio: float = 0.15,
) -> Tuple[SkeletonDict, SkeletonPixels]:
    """Lay 21 hand landmarks 3D va reject neu qua it diem hop le."""
    if hand_landmarks is None:
        return {}, {}

    points_3d: SkeletonDict = {}
    pixels: SkeletonPixels = {}
    for index, lm in enumerate(hand_landmarks.landmark):
        pixel = normalized_landmark_to_pixel(lm, image_width, image_height)
        if pixel is None:
            continue
        x_px, y_px = pixel
        if not landmark_inside_mask(x_px, y_px, person_mask, mask_margin_px, mask_min_ratio):
            continue
        point = camera_point_from_registration(registration, undistorted, x_px, y_px)
        if point is None:
            continue
        name = "%s_%d" % (hand_prefix, index)
        points_3d[name] = point
        pixels[name] = pixel

    if len(points_3d) < int(min_hand_presence):
        return {}, {}
    return points_3d, pixels


def validate_body_geometry(
    pose_points_3d: SkeletonDict,
    min_core_points: int = 5,
    shoulder_width_range: Tuple[float, float] = (0.20, 0.75),
    torso_length_range: Tuple[float, float] = (0.25, 1.00),
) -> Tuple[bool, str]:
    """Kiem tra body skeleton co hinh hoc nguoi hop ly khong."""
    core_names = ["pose_11", "pose_12", "pose_13", "pose_14", "pose_15", "pose_16", "pose_23", "pose_24"]
    core_count = sum(1 for name in core_names if name in pose_points_3d)
    if core_count < int(min_core_points):
        return False, "BODY_TOO_FEW_CORE_POINTS count=%d" % core_count

    if "pose_11" in pose_points_3d and "pose_12" in pose_points_3d:
        shoulder_width = _dist(pose_points_3d["pose_11"], pose_points_3d["pose_12"])
        if not (shoulder_width_range[0] <= shoulder_width <= shoulder_width_range[1]):
            return False, "BODY_BAD_SHOULDER_WIDTH %.3f" % shoulder_width

    if all(name in pose_points_3d for name in ("pose_11", "pose_12", "pose_23", "pose_24")):
        shoulder_center = tuple(
            (pose_points_3d["pose_11"][axis] + pose_points_3d["pose_12"][axis]) / 2.0
            for axis in range(3)
        )
        hip_center = tuple(
            (pose_points_3d["pose_23"][axis] + pose_points_3d["pose_24"][axis]) / 2.0
            for axis in range(3)
        )
        torso_length = _dist(shoulder_center, hip_center)
        if not (torso_length_range[0] <= torso_length <= torso_length_range[1]):
            return False, "BODY_BAD_TORSO_LENGTH %.3f" % torso_length

    return True, "BODY_OK count=%d" % core_count


def validate_hand_geometry(
    hand_points_3d: SkeletonDict,
    min_valid_points: int = 8,
    max_finger_span: float = 0.30,
    min_palm_size: float = 0.025,
    max_palm_size: float = 0.18,
) -> Tuple[bool, str]:
    """Kiem tra hand landmarks co kich thuoc ban tay hop ly khong."""
    if len(hand_points_3d) < int(min_valid_points):
        return False, "HAND_TOO_FEW_POINTS count=%d" % len(hand_points_3d)

    prefix = "left_hand" if any(str(name).startswith("left_hand_") for name in hand_points_3d) else "right_hand"
    tips = ["%s_%d" % (prefix, index) for index in (4, 8, 12, 16, 20)]
    available_tips = [hand_points_3d[name] for name in tips if name in hand_points_3d]
    if len(available_tips) >= 2:
        max_span = 0.0
        for idx, point_a in enumerate(available_tips):
            for point_b in available_tips[idx + 1:]:
                max_span = max(max_span, _dist(point_a, point_b))
        if max_span > max_finger_span:
            return False, "HAND_BAD_FINGER_SPAN %.3f" % max_span

    palm_names = ["%s_%d" % (prefix, index) for index in (0, 5, 9, 13, 17)]
    palm_points = [hand_points_3d[name] for name in palm_names if name in hand_points_3d]
    if len(palm_points) >= 3:
        palm_span = 0.0
        for idx, point_a in enumerate(palm_points):
            for point_b in palm_points[idx + 1:]:
                palm_span = max(palm_span, _dist(point_a, point_b))
        if palm_span < min_palm_size or palm_span > max_palm_size:
            return False, "HAND_BAD_PALM_SIZE %.3f" % palm_span

    return True, "HAND_OK count=%d" % len(hand_points_3d)


def _rename_hand_keys(points: SkeletonDict, pixels: SkeletonPixels, target_side: str):
    old_prefix = "left_hand" if any(str(name).startswith("left_hand_") for name in points) else "right_hand"
    new_prefix = "%s_hand" % target_side

    renamed_points: SkeletonDict = {}
    renamed_pixels: SkeletonPixels = {}
    for name, point in points.items():
        suffix = str(name).split("_")[-1]
        renamed_points["%s_%s" % (new_prefix, suffix)] = point
    for name, pixel in pixels.items():
        suffix = str(name).split("_")[-1]
        renamed_pixels["%s_%s" % (new_prefix, suffix)] = pixel
    return renamed_points, renamed_pixels


def attach_hands_to_pose_wrists(
    pose_points_3d: SkeletonDict,
    hand_points_by_side: Dict[str, SkeletonDict],
    hand_pixels_by_side: Dict[str, SkeletonPixels],
    pose_pixels: Optional[SkeletonPixels] = None,
    max_attach_distance_m: float = 0.15,
    max_attach_distance_px: float = 80.0,
    allow_swap: bool = True,
):
    """Gan hand_0 vao pose wrist gan nhat va reject tay khong noi duoc."""
    pose_pixels = pose_pixels or {}
    wrist_names = {"left": "pose_15", "right": "pose_16"}
    candidates: List[Tuple[float, str, str, float, float]] = []

    for detected_side, hand_points in hand_points_by_side.items():
        if not hand_points:
            continue
        detected_prefix = "left_hand" if detected_side == "left" else "right_hand"
        hand_wrist_name = "%s_0" % detected_prefix
        if hand_wrist_name not in hand_points:
            continue
        target_sides = ["left", "right"] if allow_swap else [detected_side]
        for target_side in target_sides:
            pose_wrist_name = wrist_names[target_side]
            if pose_wrist_name not in pose_points_3d:
                continue
            distance_m = _dist(hand_points[hand_wrist_name], pose_points_3d[pose_wrist_name])
            if distance_m > max_attach_distance_m:
                continue

            distance_px = 0.0
            hand_pixels = hand_pixels_by_side.get(detected_side, {})
            if (
                max_attach_distance_px > 0.0
                and hand_wrist_name in hand_pixels
                and pose_wrist_name in pose_pixels
            ):
                hx, hy = hand_pixels[hand_wrist_name]
                px, py = pose_pixels[pose_wrist_name]
                distance_px = math.sqrt((hx - px) ** 2 + (hy - py) ** 2)
                if distance_px > max_attach_distance_px:
                    continue

            score = distance_m + distance_px / 1000.0
            candidates.append((score, detected_side, target_side, distance_m, distance_px))

    candidates.sort(key=lambda item: item[0])
    used_detected = set()
    used_target = set()
    attached_points: Dict[str, SkeletonDict] = {"left": {}, "right": {}}
    attached_pixels: Dict[str, SkeletonPixels] = {"left": {}, "right": {}}
    status: List[str] = []

    for _score, detected_side, target_side, distance_m, distance_px in candidates:
        if detected_side in used_detected or target_side in used_target:
            continue
        renamed_points, renamed_pixels = _rename_hand_keys(
            hand_points_by_side.get(detected_side, {}),
            hand_pixels_by_side.get(detected_side, {}),
            target_side,
        )
        attached_points[target_side] = renamed_points
        attached_pixels[target_side] = renamed_pixels
        used_detected.add(detected_side)
        used_target.add(target_side)
        status.append(
            "%s->%s d_m=%.3f d_px=%.1f" % (detected_side, target_side, distance_m, distance_px)
        )

    for side, hand_points in hand_points_by_side.items():
        if hand_points and side not in used_detected:
            status.append("%s_rejected_no_wrist_attach" % side)

    return attached_points, attached_pixels, ";".join(status) if status else "NO_HANDS_ATTACHED"


def merge_pose_and_hands(
    pose_points_3d: SkeletonDict,
    left_hand_3d: Optional[SkeletonDict] = None,
    right_hand_3d: Optional[SkeletonDict] = None,
) -> SkeletonDict:
    """Gop body va hand landmarks thanh mot skeleton dict."""
    skeleton: SkeletonDict = {}
    skeleton.update(pose_points_3d or {})
    skeleton.update(left_hand_3d or {})
    skeleton.update(right_hand_3d or {})
    return skeleton


def _default_landmark_order_for_msg(pose_array: PoseArray, landmark_order: Optional[Iterable[Any]]):
    if landmark_order is not None:
        return list(landmark_order)
    if len(pose_array.poses) == len(LANDMARK_ORDER):
        return list(LANDMARK_ORDER)
    return list(DEFAULT_TRACKED_JOINT_IDS)


def skeleton_dict_to_pose_array(
    skeleton: Dict[Any, Point3D],
    frame_id: str,
    stamp,
    landmark_order: Optional[Iterable[Any]] = None,
) -> PoseArray:
    """Encode skeleton dict thanh PoseArray theo landmark order co dinh."""
    order = list(landmark_order or LANDMARK_ORDER)
    pose_array = PoseArray()
    pose_array.header.frame_id = frame_id
    pose_array.header.stamp = stamp

    for name in order:
        pose_msg = Pose()
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0

        if name in skeleton and _is_finite_point(skeleton[name]):
            x, y, z = skeleton[name]
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
        else:
            pose_msg.position.x = INVALID_COORD
            pose_msg.position.y = INVALID_COORD
            pose_msg.position.z = INVALID_COORD
        pose_array.poses.append(pose_msg)

    return pose_array


def pose_array_to_skeleton_dict(
    pose_array: PoseArray,
    landmark_order: Optional[Iterable[Any]] = None,
) -> Dict[Any, Point3D]:
    """Decode PoseArray ve skeleton dict, bo landmark NaN."""
    order = _default_landmark_order_for_msg(pose_array, landmark_order)
    skeleton: Dict[Any, Point3D] = {}

    for name, pose_msg in zip(order, pose_array.poses):
        if pose_msg.orientation.w <= 0.0:
            continue
        if not pose_is_valid(pose_msg):
            continue
        skeleton[name] = (
            pose_msg.position.x,
            pose_msg.position.y,
            pose_msg.position.z,
        )

    return skeleton


def numeric_joint_order_for_pose_array(
    pose_array: PoseArray,
    fallback_order: Optional[Iterable[int]] = None,
) -> List[int]:
    """Return numeric joint schema matching tracker or fused PoseArray."""
    pose_count = len(pose_array.poses)
    if pose_count == len(POSE_ARRAY_JOINT_IDS):
        return list(POSE_ARRAY_JOINT_IDS)
    if pose_count == len(POSE_LANDMARK_IDS):
        return list(POSE_LANDMARK_IDS)
    return list(fallback_order or POSE_LANDMARK_IDS)


def pose_array_to_numeric_joint_dict(
    pose_array: PoseArray,
    fallback_order: Optional[Iterable[int]] = None,
) -> Dict[int, Point3D]:
    """Decode canonical tracker/fusion PoseArray to numeric joint IDs."""
    order = numeric_joint_order_for_pose_array(pose_array, fallback_order)
    skeleton: Dict[int, Point3D] = {}
    for joint_id, pose_msg in zip(order, pose_array.poses):
        if pose_msg.orientation.w <= 0.0 or not pose_is_valid(pose_msg):
            continue
        skeleton[int(joint_id)] = (
            float(pose_msg.position.x),
            float(pose_msg.position.y),
            float(pose_msg.position.z),
        )
    return skeleton


def _draw_connection(image: np.ndarray, pixels: SkeletonPixels, pair: ConnectionPair, color: Tuple[int, int, int], thickness: int) -> None:
    name_a, name_b = pair
    if name_a not in pixels or name_b not in pixels:
        return
    cv2.line(image, pixels[name_a], pixels[name_b], color, thickness, cv2.LINE_AA)


def draw_filtered_skeleton(
    image_bgr: np.ndarray,
    skeleton_pixels: SkeletonPixels,
    person_mask: Optional[np.ndarray] = None,
    draw_person_mask: bool = True,
    draw_body: bool = True,
    draw_hands: bool = True,
    draw_reject_regions: bool = False,
    status_text: str = "",
    fps: Optional[float] = None,
) -> np.ndarray:
    """Ve debug image chi tu skeleton da qua filter."""
    output = image_bgr.copy()

    if draw_person_mask and person_mask is not None:
        mask = person_mask.astype(bool)
        overlay = output.copy()
        overlay[mask] = (0, 120, 70)
        output = cv2.addWeighted(overlay, 0.25, output, 0.75, 0.0)

    if draw_body:
        for pair in BODY_CONNECTIONS:
            _draw_connection(output, skeleton_pixels, pair, (0, 220, 0), 2)
        for pair in HAND_TO_BODY_CONNECTIONS:
            _draw_connection(output, skeleton_pixels, pair, (255, 220, 0), 2)

    if draw_hands:
        for pair in LEFT_HAND_CONNECTIONS:
            _draw_connection(output, skeleton_pixels, pair, (255, 120, 0), 1)
        for pair in RIGHT_HAND_CONNECTIONS:
            _draw_connection(output, skeleton_pixels, pair, (180, 0, 255), 1)

    for name, pixel in skeleton_pixels.items():
        if is_hand_landmark(name):
            color = (255, 120, 0) if str(name).startswith("left_hand_") else (180, 0, 255)
            radius = 2
        else:
            color = (0, 255, 0)
            radius = 4
        cv2.circle(output, pixel, radius, color, -1, cv2.LINE_AA)

    if draw_reject_regions:
        cv2.rectangle(output, (0, 0), (output.shape[1] - 1, output.shape[0] - 1), (0, 255, 255), 1)

    lines = []
    if status_text:
        lines.append(status_text)
    if fps is not None and math.isfinite(float(fps)):
        lines.append("FPS %.1f" % float(fps))
    lines.append("landmarks %d" % len(skeleton_pixels))

    y = 24
    for line in lines:
        cv2.putText(output, line[:120], (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(output, line[:120], (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y += 22

    return output
