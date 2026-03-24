# ORB-SLAM3: LocalMappingResult Rich Data

## Overview

`LocalMappingResult` is the per-iteration output of the LocalMapping thread.
It is delivered asynchronously to the client via a callback registered with
`System::SetLocalMappingCallback()`.

```cpp
slam.SetLocalMappingCallback([](const ORB_SLAM3::LocalMappingResult& r) {
    // Called from the LocalMapping thread. Copy and return quickly.
    myQueue.push(r);
});
```

The callback fires once per keyframe processed — not on idle iterations where
the thread finds an empty queue. The struct is fully self-contained: no raw
pointers into library internals, just numeric IDs, poses, and world-space
positions.

This document describes every field in the struct and shows how a client can
use the data to maintain a real-time sparse map and replicate the visualization
seen in the built-in ORB-SLAM3 Pangolin viewer.

---

## The built-in viewer and what drives it

The Pangolin viewer (`MapDrawer`) draws four kinds of visual elements on every
frame:

| Visual element | Color | Source (inside the library) |
|---|---|---|
| Map-point cloud | Black (non-reference) / Red (reference KFs) | All `MapPoint` objects in the active map |
| KeyFrame frustums | Red thick = map-origin, Green = LBA-optimised, Red = LBA-fixed, Blue = other | All `KeyFrame` objects + `Map::msOptKFs / msFixedKFs` |
| Covisibility graph edges | Semi-transparent green | `KeyFrame::GetCovisiblesByWeight(100)` |
| Spanning-tree edges | Same green, blended | `KeyFrame::GetParent()` per KF |

All of this data is now available in `LocalMappingResult` without accessing
any library internals.

---

## Struct reference

### Top-level `LocalMappingResult`

```
LocalMappingResult
├── iteration              — monotonic counter (1, 2, 3, …)
├── keyframe_id            — KeyFrame::mnId of the KF processed
├── frame_id               — Frame::mnFrameId (tracking frame index)
├── timestamp              — sensor timestamp in seconds
│
├── process_new_keyframe   — ProcessNewKeyFrameResult
├── map_point_culling      — MapPointCullingResult
├── create_new_map_points  — CreateNewMapPointsResult
├── search_in_neighbors    — SearchInNeighborsResult
├── lba                    — LocalBundleAdjustmentResult
├── keyframe_culling       — KeyFrameCullingResult
│
├── added_map_points       — convenience mirror of create_new_map_points.new_map_points
├── culled_map_point_ids   — convenience mirror of map_point_culling.culled_map_point_ids
└── lba_outlier_map_point_ids — convenience mirror of lba.outlier_map_point_ids
```

---

### `ProcessNewKeyFrameResult` — per-KF identity and initial pose

The first stage of every iteration. Pops the next keyframe from the queue,
computes its BoW descriptor, associates existing map points, and inserts the
frame into the Atlas.

| Field | Type | Meaning |
|---|---|---|
| `keyframe_id` | `unsigned long` | `KeyFrame::mnId` — unique numeric ID of the KF |
| `frame_id` | `unsigned long` | `Frame::mnFrameId` — sequential tracking-frame index |
| `timestamp` | `double` | Sensor timestamp in seconds |
| `pose` | `Sophus::SE3f` | Camera-in-world transform **Twc** (wTc) at insertion time, *before* any LBA refinement this iteration. Apply directly as the model matrix for rendering. |
| `num_kf_map_point_slots` | `int` | Total feature slots in the KF's observation vector (equals number of keypoints detected) |
| `num_associated_map_points` | `int` | How many slots hold a valid, non-bad map point after association |
| `num_stereo_map_points_registered` | `int` | Stereo points created by Tracking and promoted to the "recently added" list this call |
| `queue_size_after` | `int` | Remaining KFs waiting in the queue after this one was popped |
| `duration_ms` | `double` | Wall-clock time for this stage |

The pose is already in world frame (`Twc`), so it can be used directly as the model matrix:

```cpp
Eigen::Matrix4f Twc = r.process_new_keyframe.pose.matrix();
```

---

### `MapPointCullingResult` — map-point cleanup

Walks the list of recently-added map points and removes unreliable ones.

| Field | Type | Meaning |
|---|---|---|
| `num_recent_map_points_before` | `int` | Size of the "recently added" list before culling |
| `num_culled_already_bad` | `int` | Points already flagged bad by another thread — removed from list only |
| `num_culled_low_found_ratio` | `int` | Points where (found / visible) < threshold — `SetBadFlag()` called |
| `num_culled_too_few_observations` | `int` | Points with too few observations after the minimum age window — `SetBadFlag()` called |
| `num_graduated` | `int` | Points that aged out of the "recent" list normally — they graduate to the main map, `SetBadFlag()` NOT called |
| `num_recent_map_points_after` | `int` | Size of the list after culling |
| `culled_map_point_ids` | `vector<unsigned long>` | IDs of points on which `SetBadFlag()` was called — client should remove these |
| `duration_ms` | `double` | Wall-clock time for this stage |

---

### `CreateNewMapPointsResult` — triangulation

Searches for feature matches across neighbouring keyframes using the epipolar
constraint, triangulates valid pairs, and creates new map points.

| Field | Type | Meaning |
|---|---|---|
| `num_neighbour_kfs` | `int` | Neighbour KFs considered for triangulation |
| `num_epipolar_matches` | `int` | Total feature matches found across all neighbour pairs before geometric checks |
| `num_stereo_unproject_attempts` | `int` | Candidates that went through the stereo-unproject path |
| `num_created` | `int` | Map points successfully created |
| `num_created_from_stereo` | `int` | Subset of `num_created` from stereo unprojection rather than multi-view triangulation |
| `aborted_early` | `bool` | `true` when the loop was cut short because a new KF arrived |
| `new_map_points` | `vector<NewMappingMapPoint>` | The newly created map points — see below |
| `duration_ms` | `double` | Wall-clock time for this stage |

#### `NewMappingMapPoint`

| Field | Type | Meaning |
|---|---|---|
| `id` | `unsigned long` | `MapPoint::mnId` — globally unique |
| `pos_world` | `Eigen::Vector3f` | 3-D position in the world frame |
| `first_kf_id` | `unsigned long` | `mnId` of the keyframe that first observed (created) this point |

---

### `SearchInNeighborsResult` — map-point fusion

Fuses duplicate map points between the current KF and its neighbours via
bidirectional projection search.

| Field | Type | Meaning |
|---|---|---|
| `num_first_level_neighbours` | `int` | Direct covisible neighbours used as fusion targets |
| `num_second_level_neighbours` | `int` | Covisible-of-covisible neighbours also used |
| `num_target_kfs` | `int` | Total fusion targets (first + second level, deduplicated) |
| `aborted_early` | `bool` | Stage exited early because a new KF arrived |
| `duration_ms` | `double` | Wall-clock time for this stage |

Note: this stage can merge duplicate map points (one survives, one is marked
bad). The surviving point retains its ID; the bad point will appear in a
subsequent `culled_map_point_ids`.

---

### `LocalBundleAdjustmentResult` — graph structure and optimisation window

LBA jointly optimises the current KF and its local neighbourhood. It may be
skipped due to throttling or insufficient keyframes.

| Field | Type | Meaning |
|---|---|---|
| `skipped` | `bool` | `true` if LBA did not run this iteration |
| `skip_reason` | `string` | Why it was skipped: `"throttled"`, `"too_few_keyframes"`, `"stop_requested"`, `"new_kf_arrived"`, `"no_fixed_kfs"` |
| `num_fixed_kfs` | `int` | KFs held as anchors (poses not modified) |
| `fixed_keyframe_ids` | `vector<unsigned long>` | IDs of those anchor KFs |
| `num_optimised_kfs` | `int` | KFs whose poses were free variables in the optimisation |
| `optimised_keyframe_ids` | `vector<unsigned long>` | IDs of those optimised KFs (current KF is always first) |
| `num_map_points` | `int` | Map points included in the optimisation |
| `num_edges` | `int` | Reprojection edges in the GTSAM graph |
| `outlier_map_point_ids` | `vector<unsigned long>` | Points rejected by the post-optimisation reprojection-error pass — client should remove these |
| `num_outlier_map_points` | `int` | Convenience count |
| `covisibility_edges` | `vector<CovisibilityEdge>` | Covisibility graph edges in the LBA window — see below |
| `spanning_tree_edges` | `vector<SpanningTreeEdge>` | Spanning-tree edges for every KF in the window — see below |
| `duration_ms` | `double` | Wall-clock time. Zero when `skipped == true` |

All vector fields are empty when `skipped == true`.

#### The LBA window

The window contains two disjoint KF sets:

- **Optimised KFs** (`optimised_keyframe_ids`): the current KF plus its best
  covisible neighbours. Their poses are free variables.
- **Fixed KFs** (`fixed_keyframe_ids`): KFs just outside the window that are
  connected to it but whose poses are held constant as anchors. The map-origin
  KF is always in this set (or pinned via a tight prior factor).

#### `CovisibilityEdge`

| Field | Type | Meaning |
|---|---|---|
| `kf_id_a` | `unsigned long` | Lower of the two KF IDs |
| `kf_id_b` | `unsigned long` | Higher of the two KF IDs |
| `weight` | `int` | Number of shared map-point observations between the two KFs |

Covers all pairs in the window where both KFs have at least one shared
observation. Fixed × fixed pairs are omitted (they don't interact in the
graph).

The connection threshold in `UpdateConnections()` is **15** shared
observations — any pair with `weight >= 15` has an edge in the covisibility
graph. The LBA window naturally contains the densest region of the graph
around the current KF.

#### `SpanningTreeEdge`

| Field | Type | Meaning |
|---|---|---|
| `child_kf_id` | `unsigned long` | The KF whose parent is being described |
| `parent_kf_id` | `unsigned long` | Its parent in the spanning tree; `0` if this KF is the map-origin root |
| `parent_in_lba_window` | `bool` | `true` when the parent is also inside the LBA window (optimised or fixed) |

One entry per KF in the LBA window (all optimised KFs + all fixed KFs).
The spanning tree is a directed acyclic graph rooted at the map-origin KF;
each non-root KF has exactly one parent.

---

### `KeyFrameCullingResult` — redundant KF removal

Marks KFs bad when the majority of their map points are already well-observed
by other KFs.

| Field | Type | Meaning |
|---|---|---|
| `num_kfs_checked` | `int` | Local KFs examined |
| `num_kfs_culled` | `int` | KFs marked bad (`SetBadFlag()` called) |
| `aborted_early` | `bool` | Loop exited early |
| `culled_keyframe_ids` | `vector<unsigned long>` | IDs of culled KFs |
| `duration_ms` | `double` | Wall-clock time for this stage |

---

## The essential graph

The **essential graph** in ORB-SLAM3 is a sparse subgraph of the covisibility
graph used for loop-closure optimisation. In this codebase it is the union of:

1. **Spanning-tree edges** — the minimum connected skeleton, maintained
   automatically by `KeyFrame::UpdateConnections()`. Available in
   `lba.spanning_tree_edges`.

2. **Covisibility edges** — all KF pairs sharing ≥ 15 map-point observations.
   Available in `lba.covisibility_edges`.

3. **Loop-closure edges** — not implemented in this codebase.

Together, `covisibility_edges` and `spanning_tree_edges` give the complete
essential-graph slice for the active LBA window on every iteration.

---

## Replicating the Pangolin viewer

The following sections show how to reproduce each visual element from the
built-in viewer using only `LocalMappingResult` data.

### 1. Sparse map-point cloud

Maintain a client-side map of `{id → Eigen::Vector3f pos_world}`.

```cpp
// On each callback:
for (auto& mp : r.added_map_points)
    client_map[mp.id] = mp.pos_world;

for (auto id : r.culled_map_point_ids)
    client_map.erase(id);

for (auto id : r.lba_outlier_map_point_ids)
    client_map.erase(id);
```

The viewer draws all map-point positions as a black point cloud, with the
points visible from the current reference KF rendered in red. Because
`LocalMappingResult` does not yet expose which points belong to the reference
KF's local map, render all points in a neutral color as a first pass; add a
separate step using the current pose and a radius threshold to highlight nearby
points in red if desired.

### 2. KeyFrame frustums

Maintain a client-side map of `{kf_id → Sophus::SE3f Twc}`.

```cpp
auto& pkf = r.process_new_keyframe;
kf_poses[pkf.keyframe_id] = pkf.pose;  // already Twc

// Remove culled KFs
for (auto id : r.keyframe_culling.culled_keyframe_ids)
    kf_poses.erase(id);
```

Apply `pose.matrix()` directly as the model-view matrix. The viewer draws a
standard camera frustum (pyramid with a rectangular front face) at that transform.

**Coloring** (matches the viewer's LBA debug mode):

```cpp
// After receiving a result where lba.skipped == false:
set<unsigned long> opt_kfs(r.lba.optimised_keyframe_ids.begin(),
                           r.lba.optimised_keyframe_ids.end());
set<unsigned long> fixed_kfs(r.lba.fixed_keyframe_ids.begin(),
                              r.lba.fixed_keyframe_ids.end());

for (auto& [id, Tcw] : kf_poses)
{
    bool is_root = false;
    for (auto& e : r.lba.spanning_tree_edges)
        if (e.child_kf_id == id && e.parent_kf_id == 0) { is_root = true; break; }

    Color color;
    float line_width;
    if (is_root) {
        color = RED; line_width = 5 * kKeyFrameLineWidth;   // map-origin
    } else if (opt_kfs.count(id)) {
        color = GREEN; line_width = kKeyFrameLineWidth;      // being optimised
    } else if (fixed_kfs.count(id)) {
        color = RED; line_width = kKeyFrameLineWidth;        // anchor
    } else {
        color = BLUE; line_width = kKeyFrameLineWidth;       // regular
    }
    // draw frustum at Tcw.inverse()
}
```

Exact RGB values from the viewer:
- Map-origin (root): `(1, 0, 0)` thick
- LBA-optimised: `(0, 1, 0)`
- LBA-fixed: `(1, 0, 0)` normal weight
- Regular: `(0, 0, 1)`

### 3. Covisibility graph edges

The viewer draws a line between every pair of KFs that share enough map points,
using semi-transparent green `(0, 1, 0, 0.6)`.

```cpp
// After receiving a result where lba.skipped == false:
for (auto& e : r.lba.covisibility_edges)
{
    auto it_a = kf_poses.find(e.kf_id_a);
    auto it_b = kf_poses.find(e.kf_id_b);
    if (it_a == kf_poses.end() || it_b == kf_poses.end()) continue;

    Eigen::Vector3f ow_a = it_a->second.translation();  // Twc.t() is the camera origin
    Eigen::Vector3f ow_b = it_b->second.translation();
    draw_line(ow_a, ow_b, color_green_alpha);
}
```

The `weight` field on each edge can drive line thickness or transparency for a
richer visualization (higher weight = stronger covisibility = thicker line).

### 4. Spanning-tree edges

Drawn with the same green color as the covisibility edges. Every non-root KF
has exactly one parent; draw a line from the KF's camera center to its parent's
camera center.

```cpp
for (auto& e : r.lba.spanning_tree_edges)
{
    if (e.parent_kf_id == 0) continue;  // root has no parent

    auto it_child  = kf_poses.find(e.child_kf_id);
    auto it_parent = kf_poses.find(e.parent_kf_id);
    if (it_child == kf_poses.end() || it_parent == kf_poses.end()) continue;

    Eigen::Vector3f ow_c = it_child->second.translation();
    Eigen::Vector3f ow_p = it_parent->second.translation();
    draw_line(ow_c, ow_p, color_green_alpha);
}
```

`parent_in_lba_window` tells you whether the parent was part of the current
optimisation window. Edges where `parent_in_lba_window == false` connect
outward to older KFs — useful for distinguishing "local" versus "long-range"
tree edges in visualizations.

### 5. Live camera pose

The current camera pose is available from the `TrackingResult` returned by
`TrackStereo()` / `TrackMonocular()` on the main thread. Draw a green frustum
at `TrackingResult::pose.inverse()`.

---

## Client state machine

The minimal client state needed to replicate the viewer:

```
client_map_points: map<unsigned long, Eigen::Vector3f>
kf_poses:         map<unsigned long, Sophus::SE3f>   (Twc per KF)
lba_opt_kfs:      set<unsigned long>
lba_fixed_kfs:    set<unsigned long>
covisibility:     vector<CovisibilityEdge>
spanning_tree:    vector<SpanningTreeEdge>
```

Update rules on each `LocalMappingResult` callback:

```
1. kf_poses[pkf.keyframe_id] = pkf.pose

2. for id in keyframe_culling.culled_keyframe_ids:
       kf_poses.erase(id)

3. for mp in added_map_points:
       client_map_points[mp.id] = mp.pos_world

4. for id in culled_map_point_ids ∪ lba_outlier_map_point_ids:
       client_map_points.erase(id)

5. if not lba.skipped:
       lba_opt_kfs   = set(lba.optimised_keyframe_ids)
       lba_fixed_kfs = set(lba.fixed_keyframe_ids)
       covisibility  = lba.covisibility_edges    // window slice; merge with global graph
       spanning_tree = lba.spanning_tree_edges   // window slice; merge with global tree
```

The covisibility and spanning-tree data in each callback is a **window-local
slice** (the KFs currently in the LBA window). For a complete global graph,
maintain a persistent `map<pair<ul,ul>, int>` for covisibility and a
`map<ul, ul>` for parent pointers, updating entries from each callback:

```cpp
// Update covisibility weights for the window:
for (auto& e : r.lba.covisibility_edges)
    global_covisibility[{e.kf_id_a, e.kf_id_b}] = e.weight;

// Update parent pointers for every KF seen this iteration:
for (auto& e : r.lba.spanning_tree_edges)
    if (e.parent_kf_id != 0)
        parent_of[e.child_kf_id] = e.parent_kf_id;
    // else: e.child_kf_id is the root — no parent entry needed

// Remove culled KFs from the graph:
for (auto id : r.keyframe_culling.culled_keyframe_ids) {
    parent_of.erase(id);
    // erase covisibility edges referencing id
}
```

---

## Edge cases

| Situation | Behaviour |
|---|---|
| `lba.skipped == true` | `covisibility_edges` and `spanning_tree_edges` are empty. Graph state from the last non-skipped iteration remains valid until the next LBA runs. |
| Map-origin KF | `spanning_tree_edges` entry with `parent_kf_id == 0`. Always present in `fixed_keyframe_ids`. Draw its frustum with a thick red outline. |
| KF culled before LBA runs | Appears in `keyframe_culling.culled_keyframe_ids`. Remove from `kf_poses` and from the global graph. Its map points may still be valid if other KFs observe them. |
| `parent_in_lba_window == false` | The spanning-tree edge exits the window. The parent KF's pose is in `kf_poses` from a previous iteration — the edge can still be drawn as long as the parent hasn't been culled. |
| Reset / new map | All existing IDs become stale. Listen for a reset event (e.g. a gap in `iteration` values, or a new `keyframe_id = 0`) and flush all client state. |
