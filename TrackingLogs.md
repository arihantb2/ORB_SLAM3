# Tracking introspection logs (pinhole stereo / monocular)

This document describes the **VERBOSITY_QUIET** tracking and frame-construction logs added for debugging and tuning. It is intended for humans and AI agents interpreting log output from a real run (e.g. pinhole stereo or monocular, non-inertial).

## Log format

- Every line is prefixed by **frame id** as `[<mnId>]`.
- All messages below are printed at **Verbose::VERBOSITY_QUIET** (visible when verbosity is QUIET or higher).
- Order of lines reflects execution order within each frame.

---

## 1. Stereo initialization (stereo only)

| Log | When | Meaning |
|-----|------|--------|
| `[id] STEREO_INIT failed: keypoints=N < MinKeypoints=K.` | Before init | Frame has fewer than `Tracking.StereoInit.MinKeypoints` left keypoints; init is skipped. |
| `[id] STEREO_INIT ok: keypoints=N map_points=M.` | After first keyframe is created | Stereo init succeeded; first keyframe has N keypoints and the map has M points. |

---

## 2. Stereo pinhole frame construction (stereo only)

| Log | When | Meaning |
|-----|------|--------|
| `[id] STEREO_PINHOLE_FRAME: stereo_inlier_matches=N (keypoints=K ratio=R).` | After left-right matching in the stereo pinhole constructor | N left keypoints got a valid stereo match (depth); K is total left keypoints; R = N/K. Low R or declining N/K over time can indicate matching or calibration issues. |

*(Fisheye / two-camera stereo paths do not emit these logs.)*

---

## 3. Pose estimation (which step was used)

Each frame, pose is estimated by one of two paths, then local map is applied.

**Path A – Reference keyframe first** (no velocity or within 2 frames after reloc):

- `[id] TRACK_REF_KF failed.` — Reference keyframe tracking was tried and failed (see detailed failure below if present).
- `[id] TRACK_REF_KF ok: nmatches=N nmatchesMap=M.` — Reference keyframe tracking succeeded; N BoW matches, M inliers after pose optimization.

**Path B – Motion model first** (velocity available, not in post-reloc window):

- `[id] TRACK_WITH_MOTION_MODEL failed.` — Motion model was tried and failed (see detailed failure below if present).
- `[id] TRACK_REF_KF failed (fallback).` — After motion model failed, reference keyframe fallback was tried and failed.
- `[id] TRACK_REF_KF ok: nmatches=N nmatchesMap=M.` — Fallback reference keyframe tracking succeeded.
- `[id] TRACK_WITH_MOTION_MODEL ok: nmatchesMap=M.` — Motion model succeeded; M inliers after pose optimization.

**Detailed failure reasons (same step may log one of these then the summary “failed” line):**

- **TRACK_REF_KF:**  
  - `nmatches=N < MinBoWMatches=K` — Too few BoW matches with reference keyframe.  
  - `nmatchesMap=N < MinOptimizedMapMatches=K` — Enough BoW matches but too few inliers after pose optimization.
- **TRACK_WITH_MOTION_MODEL:**  
  - `Not enough matches [N] < MinInitialMatches=K.` — Too few matches from initial projection search.  
  - `Not enough matches [N] with wider search < MinRetryMatches=K.` — Retry with wider search still below threshold.  
  - `Not enough matches after pose optimization [N] < MinOptimizedMapMatches=K.` — Too few inliers after optimization.

---

## 4. Local map tracking

Only run when pose estimation (ref KF or motion model) already succeeded.

| Log | When | Meaning |
|-----|------|--------|
| `[id] TRACK_LOCAL_MAP failed.` | TrackLocalMap() was called and returned false | Local map tracking was attempted and failed (e.g. too few inliers; see detailed logs from TrackLocalMap if present). |
| `[id] TRACK_LOCAL_MAP ok: inliers=N.` | TrackLocalMap() succeeded | N inliers after local map matching and pose optimization. |

If you see `TRACK_REF_KF failed` or `TRACK_WITH_MOTION_MODEL failed` but never `TRACK_LOCAL_MAP failed`, tracking was lost at pose estimation; local map was never attempted.

**Detailed local map failure reasons:**

- `TRACK_LOCAL_MAP failed: recent reloc, inliers=N < 50.` — Within a short window after relocalization, inliers are below the fixed threshold (50).
- `TRACK_LOCAL_MAP failed: inliers=N < VisualMinInliers=K.` — Inliers below `Tracking.LocalMap.VisualMinInliers` (visual, non-IMU path).

---

## 5. Tracking lost (stereo / monocular only)

| Log | When | Meaning |
|-----|------|--------|
| `[id] Tracking LOST (frames_since_last_kf=N).` | State transitions from OK to LOST | Tracking declared lost; N frames since the last keyframe. Emitted only for `STEREO` and `MONOCULAR` (not IMU). |

---

## 6. Other

| Log | When | Meaning |
|-----|------|--------|
| `[id] Tracking: Created new keyframe` | A new keyframe is inserted | Normal operation; keyframe creation. |

---

## Typical sequence (stereo pinhole)

1. **Before init:** Repeated `STEREO_INIT failed: keypoints=...` until keypoints ≥ MinKeypoints.
2. **First valid frame:** `STEREO_INIT ok: keypoints=... map_points=...`.
3. **Every frame:** One `STEREO_PINHOLE_FRAME: stereo_inlier_matches=...` for the new frame.
4. **Pose:** Either `TRACK_REF_KF ok` or `TRACK_WITH_MOTION_MODEL ok` (or their failure + optional fallback).
5. **Local map:** If pose ok, then `TRACK_LOCAL_MAP ok` or `TRACK_LOCAL_MAP failed`.
6. **If lost:** `Tracking LOST (frames_since_last_kf=...)`.

## Config parameters (YAML)

Relevant keys for tuning (see sample configs and `Settings.cc` / `Tracking.cc`):

- **Stereo init:** `Tracking.StereoInit.MinKeypoints`
- **Reference keyframe:** `Tracking.ReferenceKeyframe.NNRatio`, `MinBoWMatches`, `MinOptimizedMapMatches`
- **Motion model:** `Tracking.MotionModel.NNRatio`, `ProjectionSearchThStereo` / `ProjectionSearchThMono`, `MinInitialMatches`, `RetryProjectionSearchThStereo` / `RetryProjectionSearchThMono`, `MinRetryMatches`, `MinOptimizedMapMatches`
- **Local map:** `Tracking.LocalMap.GenericMinInliers`, `Tracking.LocalMap.VisualMinInliers`

Relocalization and IMU-only branches are not covered by this log set; the “recent reloc” inlier threshold (50) is hard-coded and not exposed.
