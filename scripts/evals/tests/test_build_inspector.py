"""Tests for build_inspector.py — build_data_payload."""
import math

import numpy as np
import pandas as pd
import pytest

from build_inspector import BOOL_FLAG_DEFS, Y_AXIS_DEFS, build_data_payload


# ---------------------------------------------------------------------------
# Fixture helpers
# ---------------------------------------------------------------------------


def _make_df(n=6, *, tracking_states=None, keyframes=None):
    """Minimal frame-stats DataFrame matching what load_frame_stats_csv produces."""
    t0 = 1_700_000_000.0
    if tracking_states is None:
        tracking_states = [1, 1, 0, 1, 1, 1][:n]
    if keyframes is None:
        keyframes = [0, 1, 0, 0, 1, 0][:n]

    df = pd.DataFrame(
        {
            "timestamp":         t0 + np.arange(n) * 0.1,
            "tracking_state":    tracking_states,
            "is_keyframe":       keyframes,
            "tracked_map_points": [100, 120, 0, 110, 130, 115][:n],
            "tracking_time_ms":   [10.0, 12.0, float("nan"), 11.0, 13.0, 11.5][:n],
        }
    )
    df["t"] = (df["timestamp"] - df["timestamp"].iloc[0]).round(3)
    return df


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestBuildDataPayload:
    def test_top_level_keys(self):
        payload = build_data_payload(_make_df())
        assert set(payload.keys()) == {"meta", "raw", "flags", "sigs", "color_defs"}

    def test_meta_n_frames(self):
        payload = build_data_payload(_make_df(n=6))
        assert payload["meta"]["n_frames"] == 6

    def test_meta_n_lost(self):
        """One frame has tracking_state==0 in the default fixture."""
        payload = build_data_payload(_make_df(n=6))
        assert payload["meta"]["n_lost"] == 1

    def test_meta_n_kf(self):
        """Two frames have is_keyframe==1 in the default fixture."""
        payload = build_data_payload(_make_df(n=6))
        assert payload["meta"]["n_kf"] == 2

    def test_meta_pct_lost(self):
        payload = build_data_payload(_make_df(n=6))
        assert payload["meta"]["pct_lost"] == pytest.approx(100 / 6, abs=0.1)

    def test_meta_pct_kf(self):
        payload = build_data_payload(_make_df(n=6))
        assert payload["meta"]["pct_kf"] == pytest.approx(200 / 6, abs=0.1)

    def test_meta_duration(self):
        payload = build_data_payload(_make_df(n=6))
        # 5 intervals × 0.1 s = 0.5 s
        assert payload["meta"]["duration_s"] == pytest.approx(0.5, abs=0.01)

    def test_meta_run_name_default(self):
        """DataFrame without attrs.run_name → 'unknown'."""
        payload = build_data_payload(_make_df())
        assert payload["meta"]["run_name"] == "unknown"

    def test_meta_run_name_from_attrs(self):
        df = _make_df()
        df.attrs["run_name"] = "test_run_42"
        payload = build_data_payload(df)
        assert payload["meta"]["run_name"] == "test_run_42"

    def test_nan_serialised_as_none(self):
        """NaN in a numeric column must appear as None, not float('nan')."""
        payload = build_data_payload(_make_df(n=6))
        raw = payload["raw"]
        assert "tracking_time_ms" in raw
        vals = raw["tracking_time_ms"]
        # The third entry (index 2) had NaN
        assert vals[2] is None
        # No bare NaN floats anywhere
        for v in vals:
            assert v is None or (isinstance(v, float) and math.isfinite(v))

    def test_sigs_have_required_keys(self):
        for sig in build_data_payload(_make_df())["sigs"]:
            assert {"col", "label", "color", "default", "group"} <= set(sig.keys())

    def test_at_least_one_sig_default(self):
        """Exactly one signal must be active (default=True) on first load."""
        sigs = build_data_payload(_make_df())["sigs"]
        assert sum(s["default"] for s in sigs) >= 1

    def test_flags_contain_only_present_columns(self):
        """Flags should only list columns that exist in the DataFrame."""
        df = _make_df()
        present_cols = set(df.columns)
        for flag in build_data_payload(df)["flags"]:
            assert flag["col"] in present_cols

    def test_color_defs_empty_without_trajectory(self):
        """No trajectory DataFrame → color_defs must be empty."""
        assert build_data_payload(_make_df())["color_defs"] == []

    def test_all_lost_frames(self):
        n = 4
        df = _make_df(n=n, tracking_states=[0] * n, keyframes=[0] * n)
        payload = build_data_payload(df)
        assert payload["meta"]["n_lost"] == n
        assert payload["meta"]["pct_lost"] == pytest.approx(100.0)

    def test_no_lost_frames(self):
        n = 4
        df = _make_df(n=n, tracking_states=[1] * n, keyframes=[0] * n)
        payload = build_data_payload(df)
        assert payload["meta"]["n_lost"] == 0
        assert payload["meta"]["pct_lost"] == pytest.approx(0.0)

    def test_raw_lists_same_length(self):
        """All columns in raw must have the same length as the DataFrame."""
        df = _make_df(n=5)
        raw = build_data_payload(df)["raw"]
        for col, vals in raw.items():
            assert len(vals) == 5, f"Column '{col}' has length {len(vals)}, expected 5"

    def test_t_column_present_in_raw(self):
        """The elapsed-time column 't' must be included in raw."""
        raw = build_data_payload(_make_df(n=4))["raw"]
        assert "t" in raw
        assert raw["t"][0] == pytest.approx(0.0)
