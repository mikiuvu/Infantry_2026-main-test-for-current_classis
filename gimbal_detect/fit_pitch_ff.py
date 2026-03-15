from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CSV_CANDIDATES = [
    SCRIPT_DIR / "vofa+.csv",
    SCRIPT_DIR / "yaw_data.csv",
]

# Default channel order from application/gimbal/gimbal.c VOFA output (pitch line):
# CH0  pitch_actual_deg
# CH1  pitch_imu_w_rad
# CH2  pitch_imu_alpha_rad
# CH3  pitch_ref_speed_rad
# CH4  pitch_ref_acc_rad
# CH5  pitch_current_ff_raw
# CH6  pitch_real_current_raw
DEFAULT_ANGLE_COL = 0
DEFAULT_OMEGA_COL = 1
DEFAULT_ALPHA_COL = 2
DEFAULT_FF_COL = 5
DEFAULT_CURRENT_COL = 6

RAD_TO_DEG = 180.0 / np.pi
DEG_TO_RAD = np.pi / 180.0


def resolve_csv_path(csv_arg: str | None) -> Path:
    if csv_arg:
        csv_path = Path(csv_arg)
        if not csv_path.is_absolute():
            csv_path = Path.cwd() / csv_path
        return csv_path

    for csv_path in DEFAULT_CSV_CANDIDATES:
        if csv_path.exists():
            return csv_path

    raise FileNotFoundError("No CSV found. Pass --csv or place vofa+.csv / yaw_data.csv in gimbal_detect.")


def resolve_column(df: pd.DataFrame, spec: str | int) -> str:
    if isinstance(spec, int):
        if spec < 0 or spec >= len(df.columns):
            raise IndexError(f"Column index {spec} out of range. Available columns: {list(df.columns)}")
        return str(df.columns[spec])

    if spec.isdigit():
        return resolve_column(df, int(spec))

    if spec not in df.columns:
        raise KeyError(f"Column '{spec}' not found. Available columns: {list(df.columns)}")
    return spec


def load_data(
    csv_path: Path,
    angle_col: str | int,
    omega_col: str | int,
    alpha_col: str | int,
    current_col: str | int,
    ff_col: str | int | None,
    subtract_ff: bool,
    omega_min: float,
) -> tuple[pd.DataFrame, str, str, str, str, str | None]:
    df = pd.read_csv(csv_path, skip_blank_lines=True).dropna(how="all")

    angle_name = resolve_column(df, angle_col)
    omega_name = resolve_column(df, omega_col)
    alpha_name = resolve_column(df, alpha_col)
    current_name = resolve_column(df, current_col)
    ff_name = resolve_column(df, ff_col) if ff_col is not None else None

    use_columns = [angle_name, omega_name, alpha_name, current_name]
    if ff_name is not None:
        use_columns.append(ff_name)

    data = df[use_columns].copy()
    for col in use_columns:
        data[col] = pd.to_numeric(data[col], errors="coerce")
    data = data.dropna()

    if subtract_ff and ff_name is None:
        raise ValueError("--subtract-ff requires a valid ff column.")

    if subtract_ff:
        data["fit_current_raw"] = data[current_name] - data[ff_name]
    else:
        data["fit_current_raw"] = data[current_name]

    data = data[np.abs(data[omega_name].to_numpy()) >= omega_min]
    if len(data) < 30:
        raise ValueError("Not enough valid samples after filtering.")

    return data, angle_name, omega_name, alpha_name, current_name, ff_name


def fit_model(
    pitch_deg: np.ndarray,
    omega_rad: np.ndarray,
    alpha_rad: np.ndarray,
    current_raw: np.ndarray,
) -> dict[str, float | np.ndarray]:
    theta = pitch_deg * DEG_TO_RAD
    x = np.column_stack(
        [
            np.ones_like(theta),
            alpha_rad,
            omega_rad,
            np.cos(theta),
            np.sin(theta),
        ]
    )
    coeffs, *_ = np.linalg.lstsq(x, current_raw, rcond=None)
    pred = x @ coeffs

    bias, k_acc, k_w, g_cos, g_sin = coeffs
    gravity_max = float(np.hypot(g_cos, g_sin))
    horizontal_angle_deg = float(np.arctan2(g_sin, g_cos) * RAD_TO_DEG)

    ss_res = float(np.sum((current_raw - pred) ** 2))
    ss_tot = float(np.sum((current_raw - np.mean(current_raw)) ** 2))
    rmse = float(np.sqrt(np.mean((current_raw - pred) ** 2)))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0

    return {
        "bias": float(bias),
        "k_acc": float(k_acc),
        "k_w": float(k_w),
        "g_cos": float(g_cos),
        "g_sin": float(g_sin),
        "gravity_max": gravity_max,
        "horizontal_angle_deg": horizontal_angle_deg,
        "pred": pred,
        "rmse": rmse,
        "r2": float(r2),
    }


def fit_directional_model(
    pitch_deg: np.ndarray,
    omega_rad: np.ndarray,
    alpha_rad: np.ndarray,
    current_raw: np.ndarray,
) -> dict[str, dict[str, float | np.ndarray]]:
    pos_mask = omega_rad >= 0.0
    neg_mask = omega_rad < 0.0

    if np.count_nonzero(pos_mask) < 30 or np.count_nonzero(neg_mask) < 30:
        raise ValueError("Positive/negative samples are not enough for directional fitting.")

    return {
        "pos": fit_model(pitch_deg[pos_mask], omega_rad[pos_mask], alpha_rad[pos_mask], current_raw[pos_mask]),
        "neg": fit_model(pitch_deg[neg_mask], omega_rad[neg_mask], alpha_rad[neg_mask], current_raw[neg_mask]),
    }


def print_one_direction(tag: str, result: dict[str, float | np.ndarray]) -> None:
    print(f"{tag}:")
    print(f"bias                = {float(result['bias']):.6f} raw")
    print(f"k_acc               = {float(result['k_acc']):.6f} raw/(rad/s^2)")
    print(f"k_w                 = {float(result['k_w']):.6f} raw/(rad/s)")
    print(f"gravity_max         = {float(result['gravity_max']):.6f} raw")
    print(f"horizontal_angle    = {float(result['horizontal_angle_deg']):.6f} deg")
    print(f"RMSE                = {float(result['rmse']):.6f} raw")
    print(f"R^2                 = {float(result['r2']):.6f}")
    print()


def print_result(
    result: dict[str, dict[str, float | np.ndarray]],
    csv_path: Path,
    angle_name: str,
    omega_name: str,
    alpha_name: str,
    current_name: str,
    subtract_ff: bool,
) -> None:
    print(f"CSV          : {csv_path}")
    print(f"angle col    : {angle_name}")
    print(f"omega col    : {omega_name}")
    print(f"alpha col    : {alpha_name}")
    print(f"current col  : {current_name}")
    print(f"subtract ff  : {subtract_ff}")
    print()
    print("Identified pitch model by direction:")
    print("current_raw = bias_dir + k_acc_dir * alpha_actual + k_w_dir * omega_actual + g_cos_dir * cos(theta) + g_sin_dir * sin(theta)")
    print()
    print_one_direction("positive direction", result["pos"])
    print_one_direction("negative direction", result["neg"])
    print("robot_def.h suggestions:")
    print(f"#define PITCH_ACC_FF_COEF_RAW_POS   {float(result['pos']['k_acc']):.6f}f")
    print(f"#define PITCH_W_FF_COEF_RAW_POS     {float(result['pos']['k_w']):.6f}f")
    print(f"#define PITCH_BIAS_FF_RAW_POS       {float(result['pos']['bias']):.6f}f")
    print(f"#define PITCH_ACC_FF_COEF_RAW_NEG   {float(result['neg']['k_acc']):.6f}f")
    print(f"#define PITCH_W_FF_COEF_RAW_NEG     {float(result['neg']['k_w']):.6f}f")
    print(f"#define PITCH_BIAS_FF_RAW_NEG       {float(result['neg']['bias']):.6f}f")
    print(f"#define GRAVITY_COMP_MAX            {0.5 * (float(result['pos']['gravity_max']) + float(result['neg']['gravity_max'])):.6f}f")
    print(f"#define PITCH_HORIZONTAL_ANGLE      {0.5 * (float(result['pos']['horizontal_angle_deg']) + float(result['neg']['horizontal_angle_deg'])):.6f}f")
    print()
    print("Optional engineering extras:")
    print(f"pitch_bias_raw_pos = {float(result['pos']['bias']):.6f}")
    print(f"pitch_bias_raw_neg = {float(result['neg']['bias']):.6f}")


def plot_result(
    data: pd.DataFrame,
    angle_name: str,
    omega_name: str,
    alpha_name: str,
    result: dict[str, dict[str, float | np.ndarray]],
) -> None:
    t = np.arange(len(data))
    omega = data[omega_name].to_numpy()
    pos_mask = omega >= 0.0
    pred = np.zeros_like(omega)
    pred[pos_mask] = np.asarray(result["pos"]["pred"])
    pred[~pos_mask] = np.asarray(result["neg"]["pred"])

    plt.figure(figsize=(12, 10))

    plt.subplot(4, 1, 1)
    plt.plot(t, data[angle_name].to_numpy(), label="pitch_deg")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(t, data[omega_name].to_numpy(), label="omega_rad")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.plot(t, data[alpha_name].to_numpy(), label="alpha_rad")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 4)
    plt.plot(t, data["fit_current_raw"].to_numpy(), label="fit current raw")
    plt.plot(t, pred, label="pred current raw", alpha=0.85)
    plt.grid(True)
    plt.legend()

    plt.figure(figsize=(10, 6))
    plt.scatter(data[angle_name].to_numpy(), data["fit_current_raw"].to_numpy(), s=8, alpha=0.3, label="samples")
    order = np.argsort(data[angle_name].to_numpy())
    plt.plot(
        data[angle_name].to_numpy()[order],
        pred[order],
        color="red",
        linewidth=2,
        label="model",
    )
    plt.xlabel("pitch angle (deg)")
    plt.ylabel("current raw")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fit pitch feedforward model with gravity compensation from VOFA CSV.")
    parser.add_argument("--csv", type=str, default=None, help="CSV path. Defaults to gimbal_detect/vofa+.csv or yaw_data.csv.")
    parser.add_argument("--angle-col", type=str, default=str(DEFAULT_ANGLE_COL), help="Pitch angle column name or index. Default: CH0.")
    parser.add_argument("--omega-col", type=str, default=str(DEFAULT_OMEGA_COL), help="Pitch angular velocity column name or index. Default: CH1.")
    parser.add_argument("--alpha-col", type=str, default=str(DEFAULT_ALPHA_COL), help="Pitch angular acceleration column name or index. Default: CH2.")
    parser.add_argument("--ff-col", type=str, default=str(DEFAULT_FF_COL), help="Pitch current feedforward column name or index. Default: CH5.")
    parser.add_argument("--current-col", type=str, default=str(DEFAULT_CURRENT_COL), help="Pitch real current column name or index. Default: CH6.")
    parser.add_argument("--subtract-ff", action="store_true", help="Subtract existing pitch FF from measured current before fitting.")
    parser.add_argument("--omega-min", type=float, default=0.02, help="Drop samples with |omega| below this rad/s.")
    parser.add_argument("--no-plot", action="store_true", help="Disable matplotlib plots.")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    csv_path = resolve_csv_path(args.csv)

    data, angle_name, omega_name, alpha_name, current_name, _ = load_data(
        csv_path=csv_path,
        angle_col=args.angle_col,
        omega_col=args.omega_col,
        alpha_col=args.alpha_col,
        current_col=args.current_col,
        ff_col=args.ff_col,
        subtract_ff=args.subtract_ff,
        omega_min=args.omega_min,
    )

    result = fit_directional_model(
        pitch_deg=data[angle_name].to_numpy(),
        omega_rad=data[omega_name].to_numpy(),
        alpha_rad=data[alpha_name].to_numpy(),
        current_raw=data["fit_current_raw"].to_numpy(),
    )

    print_result(
        result=result,
        csv_path=csv_path,
        angle_name=angle_name,
        omega_name=omega_name,
        alpha_name=alpha_name,
        current_name=current_name,
        subtract_ff=args.subtract_ff,
    )

    if not args.no_plot:
        plot_result(data, angle_name, omega_name, alpha_name, result)


if __name__ == "__main__":
    main()
