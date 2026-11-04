"""Summarize NDOB shadow-run CSV logs without implying flight acceptance."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
import pandas as pd


def _stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {
        "mean": float(np.mean(values)),
        "rms": float(np.sqrt(np.mean(values ** 2))),
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        "max_abs": float(np.max(np.abs(values))),
    }


def summarize(run):
    run = Path(run).resolve()
    flight = pd.read_csv(run / "flight.csv")
    research = pd.read_csv(run / "research.csv")
    ndob = pd.read_csv(run / "ndob.csv")
    events = Counter()
    for text in ndob["events"].dropna().astype(str):
        events.update(event for event in text.split("|") if event)
    recent = ndob[ndob["t"] >= ndob["t"].max() - 30.0]
    recent_flight = flight[flight["t"] >= flight["t"].max() - 30.0]
    recent_events = Counter()
    for text in recent["events"].dropna().astype(str):
        recent_events.update(event for event in text.split("|") if event)
    estimates = {f"axis_{i}": _stats(recent[f"estimated_wrench{i}"]) for i in range(6)}
    result = {
        "run": str(run),
        "acceptance": {
            "completed": bool((run / "events.json").exists() and not (run / "failure.txt").exists()),
            "failure": (run / "failure.txt").read_text().strip() if (run / "failure.txt").exists() else None,
            "note": "CSV summary only; a completed fixed-arm acceptance window is not inferred.",
        },
        "flight": {
            "samples": int(len(flight)),
            "last_time_s": float(flight["t"].iloc[-1]),
            "last_phase": str(flight["phase"].iloc[-1]),
            "last_30s_position_rms_m": float(np.sqrt(np.mean(
                recent_flight[["ex", "ey", "ez"]].to_numpy() ** 2))),
            "last_30s_attitude_rms_deg": float(np.degrees(np.sqrt(np.mean(
                recent_flight["angle"].to_numpy() ** 2)))),
            "last_30s_mean_attitude_deg": float(np.degrees(recent_flight["angle"].mean())),
        },
        "compute_s": {
            "median": float(research["compute_s"].median()),
            "p95": float(research["compute_s"].quantile(0.95)),
            "max": float(research["compute_s"].max()),
        },
        "ndob": {
            "samples": int(len(ndob)),
            "observer_valid_ratio": float(ndob["observer_valid"].mean()),
            "shadow_mode_ratio": float(ndob["shadow_mode"].mean()),
            "compensation_max_abs": float(ndob[[f"compensation_wrench{i}" for i in range(6)]].abs().max().max()),
            "input_age_s": _stats(ndob["input_age"]),
            "dt_s": _stats(ndob["dt"]),
            "abnormal_dt_count": int((ndob["dt"] > 0.05).sum()),
            "output_limited_count": int(ndob["output_limited"].sum()),
            "events": dict(events),
            "last_30s_observer_valid_ratio": float(recent["observer_valid"].mean()),
            "last_30s_abnormal_dt_count": int((recent["dt"] > 0.05).sum()),
            "last_30s_output_limited_count": int(recent["output_limited"].sum()),
            "last_30s_events": dict(recent_events),
            "last_30s_estimated_wrench": estimates,
        },
        "input_quality_caveat": (
            "input_age is computed from runtime receive times, not hardware sample timestamps; "
            "zero or small values do not prove sensor synchronization or low sensing latency."
        ),
    }
    output = run / "ndob_summary.json"
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+")
    args = parser.parse_args()
    print(json.dumps([summarize(run) for run in args.runs], indent=2))


if __name__ == "__main__":
    main()
