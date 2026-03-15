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

# Default channel order from application/gimbal/gimbal.c VOFA output (yaw spin line):
# CH0  gimbal_actual_yaw_deg
# CH1  chassis_imu_wz_rad
# CH2  chassis_imu_wz_acc_rad
# CH3  yaw_spin_ff_raw
# CH4  yaw_real_current_raw
# CH5  yaw_total_ff_raw
DEFAULT_OMEGA_COL = 1
DEFAULT_CURRENT_COL = 4
DEFAULT_MOTION_FF_COL = None


def get_tau_c_scale(omega_rad: np.ndarray, v_s_rad: float) -> np.ndarray:
    v_s_rad = max(v_s_rad, 1e-6)
    abs_ratio = np.abs(omega_rad / v_s_rad)
    return (1.0 - np.exp(-(abs_ratio * abs_ratio))) * np.sign(omega_rad)


def get_tau_s_scale(omega_rad: np.ndarray, v_s_rad: float) -> np.ndarray:
    v_s_rad = max(v_s_rad, 1e-6)
    abs_ratio = np.abs(omega_rad / v_s_rad)
    return np.exp(-(abs_ratio * abs_ratio)) * np.sign(omega_rad)


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
    omega_col: str | int,
    current_col: str | int,
    motion_ff_col: str | int | None,
    subtract_motion_ff: bool,
    abs_omega_min: float,
) -> tuple[pd.DataFrame, str, str, str | None]:
    df = pd.read_csv(csv_path, skip_blank_lines=True).dropna(how="all")

    omega_name = resolve_column(df, omega_col)
    current_name = resolve_column(df, current_col)
    motion_ff_name = resolve_column(df, motion_ff_col) if motion_ff_col is not None else None

    use_columns = [omega_name, current_name]
    if motion_ff_name is not None:
        use_columns.append(motion_ff_name)

    data = df[use_columns].copy()
    for col in use_columns:
        data[col] = pd.to_numeric(data[col], errors="coerce")
    data = data.dropna()

    if subtract_motion_ff and motion_ff_name is None:
        raise ValueError("--subtract-motion-ff requires a valid motion_ff column.")

    if subtract_motion_ff:
        data["fit_current_raw"] = data[current_name] - data[motion_ff_name]
    else:
        data["fit_current_raw"] = data[current_name]

    data = data[np.abs(data[omega_name].to_numpy()) >= abs_omega_min]
    if len(data) < 30:
        raise ValueError("Not enough valid samples after filtering.")

    return data, omega_name, current_name, motion_ff_name


def fit_once(omega_rad: np.ndarray, current_raw: np.ndarray, v_s_rad: float) -> dict[str, float | np.ndarray]:
    x = np.column_stack(
        [
            np.ones_like(omega_rad),
            omega_rad,
            get_tau_c_scale(omega_rad, v_s_rad),
            get_tau_s_scale(omega_rad, v_s_rad),
        ]
    )
    coeffs, *_ = np.linalg.lstsq(x, current_raw, rcond=None)
    pred = x @ coeffs

    ss_res = float(np.sum((current_raw - pred) ** 2))
    ss_tot = float(np.sum((current_raw - np.mean(current_raw)) ** 2))
    rmse = float(np.sqrt(np.mean((current_raw - pred) ** 2)))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0

    return {
        "bias": float(coeffs[0]),
        "b": float(coeffs[1]),
        "tau_c": float(coeffs[2]),
        "tau_s": float(coeffs[3]),
        "pred": pred,
        "rmse": rmse,
        "r2": float(r2),
        "v_s_rad": float(v_s_rad),
    }


def fit_directional_model(
    omega_rad: np.ndarray,
    current_raw: np.ndarray,
    vs_min: float,
    vs_max: float,
    vs_steps: int,
    refine_rounds: int,
) -> dict[str, dict[str, float | np.ndarray]]:
    pos_mask = omega_rad >= 0.0
    neg_mask = omega_rad < 0.0

    if np.count_nonzero(pos_mask) < 30 or np.count_nonzero(neg_mask) < 30:
        raise ValueError("Positive/negative samples are not enough for directional fitting.")

    return {
        "pos": fit_best_model(omega_rad[pos_mask], current_raw[pos_mask], vs_min, vs_max, vs_steps, refine_rounds),
        "neg": fit_best_model(omega_rad[neg_mask], current_raw[neg_mask], vs_min, vs_max, vs_steps, refine_rounds),
    }


def fit_best_model(
    omega_rad: np.ndarray,
    current_raw: np.ndarray,
    vs_min: float,
    vs_max: float,
    vs_steps: int,
    refine_rounds: int,
) -> dict[str, float | np.ndarray]:
    best_result: dict[str, float | np.ndarray] | None = None
    low = max(vs_min, 1e-4)
    high = max(vs_max, low + 1e-4)

    for _ in range(max(refine_rounds, 1)):
        candidates = np.linspace(low, high, max(vs_steps, 3))
        for v_s_rad in candidates:
            result = fit_once(omega_rad, current_raw, float(v_s_rad))
            if best_result is None:
                best_result = result
                continue

            if float(result["r2"]) > float(best_result["r2"]):
                best_result = result
            elif float(result["r2"]) == float(best_result["r2"]) and float(result["rmse"]) < float(best_result["rmse"]):
                best_result = result

        span = (high - low) / max(vs_steps - 1, 1)
        center = float(best_result["v_s_rad"])
        low = max(vs_min, center - span)
        high = min(vs_max, center + span)

    if best_result is None:
        raise RuntimeError("Failed to fit spin feedforward model.")
    return best_result


def print_one_direction(tag: str, result: dict[str, float | np.ndarray]) -> None:
    print(f"{tag}:")
    print(f"bias      = {float(result['bias']):.6f} raw")
    print(f"b         = {float(result['b']):.6f} raw/(rad/s)")
    print(f"tau_c     = {float(result['tau_c']):.6f} raw")
    print(f"tau_s     = {float(result['tau_s']):.6f} raw")
    print(f"v_s       = {float(result['v_s_rad']):.6f} rad/s")
    print(f"RMSE      = {float(result['rmse']):.6f} raw")
    print(f"R^2       = {float(result['r2']):.6f}")
    print()


def print_result(
    result: dict[str, dict[str, float | np.ndarray]],
    csv_path: Path,
    omega_name: str,
    current_name: str,
    subtract_motion_ff: bool,
) -> None:
    print(f"CSV             : {csv_path}")
    print(f"omega col       : {omega_name}")
    print(f"current col     : {current_name}")
    print(f"subtract motion : {subtract_motion_ff}")
    print()
    print("Fitted spin model by direction:")
    print("current_raw = bias_dir + b_dir * omega + tau_c_dir * scale_c + tau_s_dir * scale_s")
    print()
    print_one_direction("positive direction", result["pos"])
    print_one_direction("negative direction", result["neg"])
    print("robot_def.h suggestions:")
    print(f"#define YAW_WZ_FF_COEF_RAW_POS      {float(result['pos']['b']):.6f}f")
    print(f"#define YAW_FRICTION_C_FF_RAW_POS   {float(result['pos']['tau_c']):.6f}f")
    print(f"#define YAW_FRICTION_S_FF_RAW_POS   {float(result['pos']['tau_s']):.6f}f")
    print(f"#define YAW_BIAS_FF_RAW_POS         {float(result['pos']['bias']):.6f}f")
    print(f"#define YAW_WZ_FF_COEF_RAW_NEG      {float(result['neg']['b']):.6f}f")
    print(f"#define YAW_FRICTION_C_FF_RAW_NEG   {float(result['neg']['tau_c']):.6f}f")
    print(f"#define YAW_FRICTION_S_FF_RAW_NEG   {float(result['neg']['tau_s']):.6f}f")
    print(f"#define YAW_BIAS_FF_RAW_NEG         {float(result['neg']['bias']):.6f}f")
    print(f"#define YAW_STRIBECK_VS_RAD         {max(float(result['pos']['v_s_rad']), float(result['neg']['v_s_rad'])):.6f}f")


def plot_result(
    data: pd.DataFrame,
    omega_name: str,
    result: dict[str, dict[str, float | np.ndarray]],
) -> None:
    t = np.arange(len(data))
    current_raw = data["fit_current_raw"].to_numpy()
    omega_rad = data[omega_name].to_numpy()
    pos_mask = omega_rad >= 0.0
    pred = np.zeros_like(omega_rad)
    pred[pos_mask] = np.asarray(result["pos"]["pred"])
    pred[~pos_mask] = np.asarray(result["neg"]["pred"])

    plt.figure(figsize=(12, 9))

    plt.subplot(3, 1, 1)
    plt.plot(t, omega_rad, label="omega_rad")
    plt.ylabel("rad/s")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, current_raw, label="fit current raw")
    plt.plot(t, pred, label="pred current raw", alpha=0.85)
    plt.ylabel("raw")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    order = np.argsort(omega_rad)
    plt.scatter(omega_rad, current_raw, s=8, alpha=0.35, label="samples")
    plt.plot(omega_rad[order], pred[order], color="red", linewidth=2, label="model")
    plt.xlabel("omega (rad/s)")
    plt.ylabel("current raw")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fit spin-only Stribeck+bias feedforward model from VOFA CSV.")
    parser.add_argument("--csv", type=str, default=None, help="CSV path. Defaults to gimbal_detect/vofa+.csv or yaw_data.csv.")
    parser.add_argument("--omega-col", type=str, default=str(DEFAULT_OMEGA_COL), help="Spin omega column name or index. Default: CH1.")
    parser.add_argument("--current-col", type=str, default=str(DEFAULT_CURRENT_COL), help="Measured current/raw column name or index. Default: CH4.")
    parser.add_argument("--motion-ff-col", type=str, default=DEFAULT_MOTION_FF_COL, help="Motion FF column name or index. Default: None (set when needed).")
    parser.add_argument("--subtract-motion-ff", action="store_true", help="Subtract motion_ff_raw from current before fitting.")
    parser.add_argument("--omega-min", type=float, default=0.05, help="Drop samples with |omega| below this rad/s.")
    parser.add_argument("--vs-min", type=float, default=0.01, help="Minimum Stribeck characteristic speed in rad/s.")
    parser.add_argument("--vs-max", type=float, default=1.20, help="Maximum Stribeck characteristic speed in rad/s.")
    parser.add_argument("--vs-steps", type=int, default=120, help="Grid points per refinement round.")
    parser.add_argument("--refine-rounds", type=int, default=3, help="Refinement rounds for v_s search.")
    parser.add_argument("--no-plot", action="store_true", help="Disable matplotlib plots.")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    csv_path = resolve_csv_path(args.csv)

    data, omega_name, current_name, _ = load_data(
        csv_path=csv_path,
        omega_col=args.omega_col,
        current_col=args.current_col,
        motion_ff_col=args.motion_ff_col,
        subtract_motion_ff=args.subtract_motion_ff,
        abs_omega_min=args.omega_min,
    )

    result = fit_directional_model(
        omega_rad=data[omega_name].to_numpy(),
        current_raw=data["fit_current_raw"].to_numpy(),
        vs_min=args.vs_min,
        vs_max=args.vs_max,
        vs_steps=args.vs_steps,
        refine_rounds=args.refine_rounds,
    )

    print_result(
        result=result,
        csv_path=csv_path,
        omega_name=omega_name,
        current_name=current_name,
        subtract_motion_ff=args.subtract_motion_ff,
    )

    if not args.no_plot:
        plot_result(data, omega_name, result)


if __name__ == "__main__":
    main()
