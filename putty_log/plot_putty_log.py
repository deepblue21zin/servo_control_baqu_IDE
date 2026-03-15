#!/usr/bin/env python3
import argparse
import math
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


CSV_RE = re.compile(
    r"^CSV,(?P<ms>\d+),(?P<mode>-?\d+),(?P<target>-?\d+(?:\.\d+)?),"
    r"(?P<current>-?\d+(?:\.\d+)?),(?P<error>-?\d+(?:\.\d+)?),"
    r"(?P<output>-?\d+(?:\.\d+)?),(?P<dir>-?\d+),(?P<enc>\d+)"
)

KB_RE = re.compile(
    r"^\[KB\]\[(?P<kind>target|snapshot)\] "
    r"T=(?P<target>-?\d+(?:\.\d+)?)deg "
    r"C=(?P<current>-?\d+(?:\.\d+)?)deg "
    r"E=(?P<error>-?\d+(?:\.\d+)?)deg "
    r"O=(?P<output>-?\d+(?:\.\d+)?) "
    r"DIR=(?P<dir>-?\d+) ENC=(?P<enc>\d+)"
)

LAT_RE = re.compile(
    r"^LATENCY_STAGE,seq=(?P<seq>\d+),name=(?P<name>[^,]+),count=(?P<count>\d+),"
    r"avg_cycles=(?P<avg_cycles>\d+),p99_cycles=(?P<p99_cycles>\d+),max_cycles=(?P<max_cycles>\d+),"
    r"avg_us=(?P<avg_us>-?\d+(?:\.\d+)?),p99_us=(?P<p99_us>-?\d+(?:\.\d+)?),max_us=(?P<max_us>-?\d+(?:\.\d+)?)"
)


def parse_logs(paths):
    control_rows = []
    latency_rows = []

    for path in paths:
        with path.open("r", encoding="utf-8", errors="ignore") as handle:
            for line_no, line in enumerate(handle, 1):
                line = line.strip()

                match = CSV_RE.match(line)
                if match:
                    control_rows.append(
                        {
                            "source": path.name,
                            "kind": "csv",
                            "x": int(match.group("ms")),
                            "x_label": "time_ms",
                            "target": float(match.group("target")),
                            "current": float(match.group("current")),
                            "error": float(match.group("error")),
                            "output": float(match.group("output")),
                            "dir": int(match.group("dir")),
                            "enc": int(match.group("enc")),
                        }
                    )
                    continue

                match = KB_RE.match(line)
                if match:
                    control_rows.append(
                        {
                            "source": path.name,
                            "kind": match.group("kind"),
                            "x": len(control_rows),
                            "x_label": "sample_index",
                            "target": float(match.group("target")),
                            "current": float(match.group("current")),
                            "error": float(match.group("error")),
                            "output": float(match.group("output")),
                            "dir": int(match.group("dir")),
                            "enc": int(match.group("enc")),
                        }
                    )
                    continue

                match = LAT_RE.match(line)
                if match:
                    latency_rows.append(
                        {
                            "source": path.name,
                            "seq": int(match.group("seq")),
                            "name": match.group("name"),
                            "avg_us": float(match.group("avg_us")),
                            "p99_us": float(match.group("p99_us")),
                            "max_us": float(match.group("max_us")),
                        }
                    )

    return control_rows, latency_rows


def plot_control(control_rows, out_dir):
    if not control_rows:
        return None

    x_label = control_rows[0]["x_label"]
    xs = [row["x"] for row in control_rows]
    target = [row["target"] for row in control_rows]
    current = [row["current"] for row in control_rows]
    error = [row["error"] for row in control_rows]
    output = [row["output"] for row in control_rows]
    enc = [row["enc"] for row in control_rows]

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(xs, target, label="target_deg", linewidth=1.5)
    axes[0].plot(xs, current, label="current_deg", linewidth=1.5)
    axes[0].plot(xs, error, label="error_deg", linewidth=1.2)
    axes[0].set_ylabel("deg")
    axes[0].set_title("Steering Control Trace")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(xs, output, label="output_cmd", linewidth=1.5)
    axes[1].plot(xs, enc, label="enc_raw", linewidth=1.2)
    axes[1].set_ylabel("output / enc")
    axes[1].set_xlabel(x_label)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    out_path = out_dir / "control_trace.png"
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return out_path


def plot_latency(latency_rows, out_dir):
    if not latency_rows:
        return None

    grouped = defaultdict(list)
    for row in latency_rows:
        grouped[row["name"]].append(row)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for stage_name in sorted(grouped.keys()):
        rows = sorted(grouped[stage_name], key=lambda item: item["seq"])
        seq = [row["seq"] for row in rows]
        avg_us = [row["avg_us"] for row in rows]
        p99_us = [row["p99_us"] for row in rows]
        axes[0].plot(seq, avg_us, label=f"{stage_name} avg_us", linewidth=1.2)
        axes[1].plot(seq, p99_us, label=f"{stage_name} p99_us", linewidth=1.2)

    axes[0].set_title("Latency Stage Average")
    axes[0].set_ylabel("avg_us")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].set_title("Latency Stage P99")
    axes[1].set_ylabel("p99_us")
    axes[1].set_xlabel("batch_seq")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    out_path = out_dir / "latency_trace.png"
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return out_path


def print_summary(control_rows, latency_rows):
    print(f"control_rows={len(control_rows)}")
    if control_rows:
        targets = {row['target'] for row in control_rows}
        enc_values = {row['enc'] for row in control_rows}
        print(f"control_unique_targets={len(targets)}")
        print(f"control_unique_enc_values={len(enc_values)}")

    print(f"latency_rows={len(latency_rows)}")
    if latency_rows:
        grouped = defaultdict(list)
        for row in latency_rows:
            grouped[row["name"]].append(row)
        for stage_name in sorted(grouped.keys()):
            rows = grouped[stage_name]
            avg_avg = sum(item["avg_us"] for item in rows) / len(rows)
            max_p99 = max(item["p99_us"] for item in rows)
            print(
                f"stage={stage_name} avg_of_avg_us={avg_avg:.3f} max_p99_us={max_p99:.3f}"
            )


def main():
    parser = argparse.ArgumentParser(description="Plot PuTTY log traces for control and latency.")
    parser.add_argument(
        "logs",
        nargs="*",
        default=["putty_log/putty.log"],
        help="PuTTY log files to parse",
    )
    parser.add_argument(
        "--out-dir",
        default="putty_log/plots",
        help="Output directory for generated PNG files",
    )
    args = parser.parse_args()

    paths = [Path(item) for item in args.logs]
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    control_rows, latency_rows = parse_logs(paths)
    print_summary(control_rows, latency_rows)

    control_path = plot_control(control_rows, out_dir)
    latency_path = plot_latency(latency_rows, out_dir)

    if control_path is not None:
        print(f"saved_control_plot={control_path}")
    if latency_path is not None:
        print(f"saved_latency_plot={latency_path}")


if __name__ == "__main__":
    main()
