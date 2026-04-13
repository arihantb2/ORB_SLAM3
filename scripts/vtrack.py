import cv2
import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple, List, Optional

# --- Base Classes for Modular Design ---

class FeatureDetector(ABC):
    """Abstract base class for feature detectors and descriptors."""
    @abstractmethod
    def detect_and_compute(self, image: np.ndarray, mask: Optional[np.ndarray] = None) -> Tuple[List[cv2.KeyPoint], np.ndarray]:
        pass

class Tracker(ABC):
    """Abstract base class for tracking algorithms."""
    @abstractmethod
    def track(self, prev_img: np.ndarray, curr_img: np.ndarray, prev_pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Returns (tracked_points, status_mask)."""
        pass

# --- Implementations ---

class ORBDetector(FeatureDetector):
    def __init__(self, n_features: int = 1000):
        self.orb = cv2.ORB_create(nfeatures=n_features)

    def detect_and_compute(self, image: np.ndarray, mask: Optional[np.ndarray] = None):
        return self.orb.detectAndCompute(image, mask)

class SIFTDetector(FeatureDetector):
    def __init__(self):
        self.sift = cv2.SIFT_create()

    def detect_and_compute(self, image: np.ndarray, mask: Optional[np.ndarray] = None):
        return self.sift.detectAndCompute(image, mask)

class KLTTracker(Tracker):
    """Lucas-Kanade Optical Flow Tracker (KLT)."""
    def __init__(self, win_size=(21, 21), max_level=3):
        self.lk_params = dict(
            winSize=win_size,
            maxLevel=max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

    def track(self, prev_img: np.ndarray, curr_img: np.ndarray, prev_pts: np.ndarray):
        # Ensure points are float32
        prev_pts = np.float32(prev_pts).reshape(-1, 1, 2)
        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(prev_img, curr_img, prev_pts, None, **self.lk_params)
        return curr_pts.reshape(-1, 2), status.reshape(-1)

class DescriptorMatcher:
    """Handles matching between descriptors with optional windowed search."""
    def __init__(self, method: str = 'BF'):
        if method == 'BF':
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        elif method == 'FLANN':
            FLANN_INDEX_LSH = 6
            index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
            search_params = dict(checks=50)
            self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

    def match(self, desc1: np.ndarray, desc2: np.ndarray):
        return self.matcher.match(desc1, desc2)

# --- Hybrid Pipeline Experiment ---

class HybridTracker:
    """
    Example of a modular pipeline:
    1. Estimate motion using KLT (Optical Flow).
    2. Use that motion as a prior to center a windowed search for Descriptor Matching.
    """
    def __init__(self, detector: FeatureDetector, window_size: int = 50):
        self.detector = detector
        self.motion_model = KLTTracker()
        self.window_size = window_size
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING) # Typical for ORB

    def track_step(self, frame1: np.ndarray, frame2: np.ndarray, pts1: np.ndarray):
        # 1. KLT Motion Prior
        pts2_klt, status = self.motion_model.track(frame1, frame2, pts1)
        
        # 2. Extract descriptors around predicted locations in frame2
        # (For this playground, we simplify by detecting in whole image but filtering by distance)
        kp2, desc2 = self.detector.detect_and_compute(frame2)
        kp1, desc1 = self.detector.detect_and_compute(frame1) # Should ideally be cached

        # Logic for "Windowed Search" using the KLT prior
        # We find descriptors in frame2 that are near the KLT predicted points
        final_pts = []
        for i, pt_prior in enumerate(pts2_klt):
            if status[i] == 0:
                final_pts.append(pt_prior) # Fallback to KLT if match fails
                continue
            
            # Find closest keypoint in frame2 to our KLT prediction
            # This is a simplified 'windowed' logic
            dists = np.linalg.norm(np.array([k.pt for k in kp2]) - pt_prior, axis=1)
            candidates = np.where(dists < self.window_size)[0]
            
            if len(candidates) > 0:
                # Of candidates, find the best descriptor match for desc1[i]
                # (Implementation details for actual matching omitted for brevity)
                best_idx = candidates[np.argmin(dists[candidates])]
                final_pts.append(kp2[best_idx].pt)
            else:
                final_pts.append(pt_prior)

        return np.array(final_pts)

# --- Visualizer Utility ---

def run_demo():
    """Simple demo using camera feed or synthetic motion."""
    cap = cv2.VideoCapture(0)
    ret, old_frame = cap.read()
    if not ret:
        print("Failed to capture video. Please check your camera.")
        return

    detector = ORBDetector()
    tracker = KLTTracker()

    def detect_points(gray: np.ndarray) -> np.ndarray:
        kp, _ = detector.detect_and_compute(gray)
        if not kp:
            return np.empty((0, 2), dtype=np.float32)
        return np.array([k.pt for k in kp[:50]], dtype=np.float32)

    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    p0 = detect_points(old_gray)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture video. Please check your camera.")
            break
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Re-detect if too few tracked points remain
        if len(p0) < 10:
            p0 = detect_points(frame_gray)
            old_gray = frame_gray.copy()
            cv2.imshow('vtrack Playground', frame)
            if cv2.waitKey(30) & 0xFF == 27:
                break
            continue

        # Track
        p1, st = tracker.track(old_gray, frame_gray, p0)

        # Draw
        for i, (new, old) in enumerate(zip(p1, p0)):
            if st[i]:
                a, b = new.ravel()
                c, d = old.ravel()
                cv2.line(frame, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                cv2.circle(frame, (int(a), int(b)), 5, (0, 0, 255), -1)

        cv2.imshow('vtrack Playground', frame)
        if cv2.waitKey(30) & 0xFF == 27:
            break

        # Update — keep only successfully tracked points
        old_gray = frame_gray.copy()
        p0 = p1[st == 1]

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("Starting vtrack Demo (KLT)... Press ESC to exit.")
    run_demo()