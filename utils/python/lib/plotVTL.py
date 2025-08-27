
"""
plotVTL.py

 Plotting module for GNSS-SDR Vector Tracking Loop (VTL) dumps.

 Pedro Pereira, 2025. pereirapedro@gmail.com

 Expected GNSS_vtl keys (from vtl_read_dump.py):
    nepoch, receiver_time_s, vtl_dt_s,
    VTL_RX_P_*_ECEF_m, VTL_RX_V_*_ECEF_ms, VTL_RX_CLK_B_GPS_m, VTL_RX_CLK_B_GAL_m, VTL_RX_CLK_D_ms,
    RTKL_RX_P_*_ECEF_m, RTKL_RX_V_*_ECEF_ms, RTKL_RX_CLK_B_GPS_m, RTKL_RX_CLK_B_GAL_m, RTKL_RX_CLK_D_ms,
    VTL_active_channels,
    EKF_prefit_PR_m, EKF_prefit_PRR_ms, EKF_postfit_PR_m, EKF_postfit_PRR_ms,
    EKF_meascov_PR, EKF_meascov_PRR, EKF_process_noise,
    RTKL_observed_PR_m, RTKL_observed_PRR_ms, VTL_computed_PR_m, VTL_computed_PRR_ms,
    VTL_code_freq_hz,
    RTKL_SV_P_*_ECEF_m, RTKL_SV_CLK_B_m, RTKL_SV_CLK_D_ms, RTKL_topo_delay_m, RTKL_iono_delay_m, RTKL_code_bias_m
    
 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

from __future__ import annotations

import os
from typing import Iterable, Dict, Any

import numpy as np
import matplotlib.pyplot as plt


SKIP_EPOCHS_DEFAULT = 10  # number of epochs to skip in plots


def _ensure_dir(path: str) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)


def _epochs(G: Dict[str, Any]) -> np.ndarray:
    if "receiver_time_s" in G and len(G["receiver_time_s"]):
        t = np.asarray(G["receiver_time_s"], dtype=float)
        return t - t[0]
    if "vtl_dt_s" in G and len(G["vtl_dt_s"]):
        dt = float(np.asarray(G["vtl_dt_s"]).ravel()[0])
        n = _infer_nepoch(G)
        return np.arange(n) * dt
    n = _infer_nepoch(G)
    return np.arange(n)


def _infer_nepoch(G: Dict[str, Any]) -> int:
    for k in ["receiver_time_s", "VTL_RX_P_X_ECEF_m", "RTKL_RX_P_X_ECEF_m"]:
        if k in G and hasattr(G[k], "__len__"):
            return len(G[k])
    if "nepoch" in G:
        try:
            return int(G["nepoch"])
        except Exception:
            pass
    return 0


def _reshape_ch(x, n_epoch: int, n_ch: int) -> np.ndarray:
    a = np.asarray(x, dtype=float).ravel()
    if a.size == n_epoch * n_ch:
        return a.reshape(n_epoch, n_ch)
    if n_ch > 0 and a.size % n_ch == 0:
        return a.reshape(a.size // n_ch, n_ch)
    if n_ch > 0 and a.size > 0:
        pad = (-a.size) % n_ch
        a = np.concatenate([a, np.full(pad, np.nan)])
        return a.reshape(-1, n_ch)
    return a.reshape(-1, 1)


def _active_mask(G: Dict[str, Any], n_epoch: int, n_ch: int) -> np.ndarray:
    if "VTL_active_channels" not in G:
        return np.ones((n_epoch, n_ch), dtype=bool)
    act = _reshape_ch(G["VTL_active_channels"], n_epoch, n_ch)
    return act.astype(bool)


def _sanitize(y: np.ndarray) -> np.ndarray:
    y = np.asarray(y, dtype=np.float64, order='C')
    y[~np.isfinite(y)] = np.nan
    return y


def plotVTL(G: Dict[str, Any],
            settings: Dict[str, Any],
            groups: Iterable[str] = ("pos","vel","clock","residuals","meas","sv","activity"),
            save: bool = True,
            show: bool = False,
            skip_epochs: int = SKIP_EPOCHS_DEFAULT) -> Dict[str, Any]:

    groups = set(groups)
    if "all" in groups:
        groups = {"pos","vel","clock","residuals","meas","sv","activity"}

    n_ch = int(settings.get("numberOfChannels", 0))
    n_epoch = _infer_nepoch(G)
    t_full = _epochs(G)

    # Skip initial epochs for plotting
    start = min(max(skip_epochs, 0), len(t_full)) if len(t_full) else 0
    t = t_full[start:] if len(t_full) else t_full

    outdir = settings.get("fig_path", "./figs")
    if save:
        _ensure_dir(outdir)

    figs: Dict[str, Any] = {}

    # Determine channels that were active at least once
    active_mask = _active_mask(G, n_epoch, n_ch) if (n_ch > 0 and n_epoch > 0) else np.zeros((0,0), dtype=bool)
    active_any = active_mask.any(axis=0) if active_mask.size else np.array([], dtype=bool)

    def _slice_1d(y):
        y = _sanitize(y)
        return y[start:start+len(t)] if len(y) >= start else y*0

    def _slice_2d(M):
        M = _reshape_ch(M, n_epoch, n_ch)
        M = _sanitize(M)
        return M[start:start+len(t), :] if M.shape[0] >= start else np.empty((0, M.shape[1]))

    def _maybe_close(fig, is_aggregate: bool):
        if not is_aggregate or not show:
            plt.close(fig)

    # ---------- Position (ECEF) & Errors ----------
    if "pos" in groups:
        has_gt = all(k in settings for k in ["gt_rx_p_x","gt_rx_p_y","gt_rx_p_z"])
        f, axs = plt.subplots(2 if has_gt else 1, 3, figsize=(13, 6 if has_gt else 4), constrained_layout=True)
        axs = np.atleast_2d(axs)
        pos_keys = ["VTL_RX_P_X_ECEF_m","VTL_RX_P_Y_ECEF_m","VTL_RX_P_Z_ECEF_m"]
        labels = ["X","Y","Z"]
        # Top row: positions for BOTH RTKL and VTL
        for i, k in enumerate(pos_keys):
            if f"RTKL_RX_P_{labels[i]}_ECEF_m" in G:
                axs[0, i].plot(t, _slice_1d(G.get(f"RTKL_RX_P_{labels[i]}_ECEF_m", [])), linestyle="--", label=f"RTKL {labels[i]}")
            axs[0, i].plot(t, _slice_1d(G.get(k, [])), label=f"VTL {labels[i]}")
            axs[0, i].set_title(f"ECEF {labels[i]}"); axs[0, i].set_xlabel("Time [s]"); axs[0, i].set_ylabel("m"); axs[0, i].grid(True, alpha=0.3)
            axs[0, i].legend(loc="best")
        # Bottom row: errors for BOTH RTKL and VTL
        if has_gt:
            gt = np.array([settings["gt_rx_p_x"], settings["gt_rx_p_y"], settings["gt_rx_p_z"]], dtype=float)
            vtl_xyz = np.vstack([_slice_1d(G.get(k, [])) for k in pos_keys]).T
            rtk_xyz = np.vstack([_slice_1d(G.get(f"RTKL_RX_P_{lab}_ECEF_m", [])) for lab in labels]).T \
                      if f"RTKL_RX_P_{labels[0]}_ECEF_m" in G else None
            if vtl_xyz.shape[0] == len(t):
                vtl_err = vtl_xyz - gt
                for i, lab in enumerate(labels):
                    if rtk_xyz is not None and rtk_xyz.shape[0] == len(t):
                        rtk_err = rtk_xyz - gt
                        axs[1, i].plot(t, rtk_err[:, i], linestyle="--", label=f"RTKL-gt {lab}", zorder=2)
                    axs[1, i].plot(t, vtl_err[:, i], label=f"VTL-gt {lab}", zorder=3)
                    axs[1, i].set_title(f"Error {lab}"); axs[1, i].set_xlabel("Time [s]"); axs[1, i].set_ylabel("m"); axs[1, i].grid(True, alpha=0.3)
                    axs[1, i].legend(loc="best")
        figs["pos"] = f
        if save: f.savefig(os.path.join(outdir, "pos_ecef_and_errors.png"), dpi=150)
        _maybe_close(f, is_aggregate=True)

    # ---------- Velocity (ECEF) ----------
    if "vel" in groups:
        f, axs = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
        vel_keys = ["VTL_RX_V_X_ECEF_ms","VTL_RX_V_Y_ECEF_ms","VTL_RX_V_Z_ECEF_ms"]
        labels = ["X","Y","Z"]
        for i, k in enumerate(vel_keys):
            if f"RTKL_RX_V_{labels[i]}_ECEF_ms" in G:
                axs[i].plot(t, _slice_1d(G.get(f"RTKL_RX_V_{labels[i]}_ECEF_ms", [])), linestyle="--", label=f"RTKL {labels[i]}")
            axs[i].plot(t, _slice_1d(G.get(k, [])), label=f"VTL {labels[i]}")
            axs[i].set_title(f"ECEF Velocity {labels[i]}"); axs[i].set_xlabel("Time [s]"); axs[i].set_ylabel("m/s")
            axs[i].grid(True, alpha=0.3); axs[i].legend(loc="best")
        figs["vel"] = f
        if save: f.savefig(os.path.join(outdir, "vel_ecef.png"), dpi=150)
        _maybe_close(f, is_aggregate=True)

    # ---------- Clock (bias & drift) ----------
    if "clock" in groups:
        f, axs = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
        if "RTKL_RX_CLK_B_GPS_m" in G:
            axs[0].plot(t, _slice_1d(G.get("RTKL_RX_CLK_B_GPS_m", [])), linestyle="--", label="RTKL GPS bias")
        axs[0].plot(t, _slice_1d(G.get("VTL_RX_CLK_B_GPS_m", [])), label="VTL GPS bias")
        axs[0].set_title("Clock Bias (GPS) [m]"); axs[0].grid(True, alpha=0.3); axs[0].set_xlabel("Time [s]"); axs[0].legend(loc="best")

        if "VTL_RX_CLK_B_GAL_m" in G and len(G["VTL_RX_CLK_B_GAL_m"]) == len(t_full):
            if "RTKL_RX_CLK_B_GAL_m" in G:
                axs[1].plot(t, _slice_1d(G.get("RTKL_RX_CLK_B_GAL_m", [])), linestyle="--", label="RTKL GAL bias")
            axs[1].plot(t, _slice_1d(G.get("VTL_RX_CLK_B_GAL_m", [])), label="VTL GAL bias")
            axs[1].set_title("Clock Bias (GAL) [m]"); axs[1].grid(True, alpha=0.3); axs[1].set_xlabel("Time [s]"); axs[1].legend(loc="best")
        else:
            axs[1].axis("off")

        if "RTKL_RX_CLK_D_ms" in G:
            axs[2].plot(t, _slice_1d(G.get("RTKL_RX_CLK_D_ms", [])), linestyle="--", label="RTKL drift")
        axs[2].plot(t, _slice_1d(G.get("VTL_RX_CLK_D_ms", [])), label="VTL drift")
        axs[2].set_title("Clock Drift [m/s]"); axs[2].grid(True, alpha=0.3); axs[2].set_xlabel("Time [s]"); axs[2].legend(loc="best")

        figs["clock"] = f
        if save: f.savefig(os.path.join(outdir, "clock.png"), dpi=150)
        _maybe_close(f, is_aggregate=True)

    # ---------- Satellite-related per-channel (active only) ----------
    if "sv" in groups and n_ch > 0 and len(t) > 0:
        svx = _slice_2d(G.get("RTKL_SV_P_X_ECEF_m", []))
        svy = _slice_2d(G.get("RTKL_SV_P_Y_ECEF_m", []))
        svz = _slice_2d(G.get("RTKL_SV_P_Z_ECEF_m", []))
        sv_cb = _slice_2d(G.get("RTKL_SV_CLK_B_m", []))
        cd_all = _slice_2d(G.get("RTKL_SV_CLK_D_ms", []))
        topo = _slice_2d(G.get("RTKL_topo_delay_m", []))
        iono = _slice_2d(G.get("RTKL_iono_delay_m", []))
        cbias = _slice_2d(G.get("RTKL_code_bias_m", []))

        sat_root = os.path.join(outdir, "satellites");  _ensure_dir(sat_root) if save else None
        for ch in range(n_ch):
            if ch >= len(active_any) or not active_any[ch]:
                continue

            # Coords
            fx, axx = plt.subplots(1, 3, figsize=(13,4), constrained_layout=True)
            for i, (arr, lab) in enumerate([(svx, "X"), (svy, "Y"), (svz, "Z")]):
                y = arr[:, ch] if arr.shape[1] > ch else np.array([])
                axx[i].plot(t, y); axx[i].set_title(f"SV {lab} (CH{ch:02d})")
                axx[i].set_xlabel("Time [s]"); axx[i].set_ylabel("m"); axx[i].grid(True, alpha=0.3)
            if save: fx.savefig(os.path.join(sat_root, f"sv_coords_CH{ch:02d}.png"), dpi=150)
            plt.close(fx)

            # Clock
            fclk, axc = plt.subplots(1, 2, figsize=(10,4), constrained_layout=True)
            cb = sv_cb[:, ch] if sv_cb.shape[1] > ch else np.array([])
            cd = cd_all[:, ch] if cd_all.shape[1] > ch else np.array([])
            axc[0].plot(t, cb, label="Bias"); axc[0].set_title(f"Clock bias (CH{ch:02d})"); axc[0].set_xlabel("Time [s]"); axc[0].set_ylabel("m"); axc[0].grid(True, alpha=0.3)
            axc[1].plot(t, cd, label="Drift"); axc[1].set_title(f"Clock drift (CH{ch:02d})"); axc[1].set_xlabel("Time [s]"); axc[1].set_ylabel("m/s"); axc[1].grid(True, alpha=0.3)
            if save: fclk.savefig(os.path.join(sat_root, f"sv_clock_CH{ch:02d}.png"), dpi=150)
            plt.close(fclk)

            # bias
            fe, axe = plt.subplots(1, 1, figsize=(10,4), constrained_layout=True)
            y_topo = topo[:, ch] if topo.shape[1] > ch else np.array([])
            y_iono = iono[:, ch] if iono.shape[1] > ch else np.array([])
            y_cb   = cbias[:, ch] if cbias.shape[1] > ch else np.array([])
            axe.plot(t, y_topo, label="Topo")
            axe.plot(t, y_iono, label="Iono")
            axe.plot(t, y_cb, label="Code bias", linestyle="--")
            axe.set_title(f"Propagation bias (CH{ch:02d})"); axe.set_xlabel("Time [s]"); axe.set_ylabel("m"); axe.grid(True, alpha=0.3); axe.legend(loc="best")
            if save: fe.savefig(os.path.join(sat_root, f"sv_bias_CH{ch:02d}.png"), dpi=150)
            plt.close(fe)

    # ---------- Channel activity ----------
    if "activity" in groups and n_ch > 0 and len(t) > 0:
        act_full = _reshape_ch(G.get("VTL_active_channels", []), n_epoch, n_ch).astype(int)
        act = act_full[start:start+len(t), :]
        f, ax = plt.subplots(figsize=(12, 6), constrained_layout=True)
        extent = (t[0], t[-1] if len(t) else 1, -0.5, n_ch - 0.5)
        ax.imshow(act.T, aspect="auto", interpolation="nearest", origin="lower", extent=extent)
        ax.set_xlabel("Time [s]"); ax.set_ylabel("Channel")
        ax.set_title("Channel Activity (1=active, 0=inactive)")
        ax.set_yticks(range(n_ch)); ax.grid(False)
        figs["activity"] = f
        if save: f.savefig(os.path.join(outdir, "channel_activity.png"), dpi=150)
        if not show: plt.close(f)

    # ---------- Per-channel measurements (active channels only) ----------
    if ("meas" in groups or "residuals" in groups) and n_ch > 0 and len(t) > 0:
        meas_root = os.path.join(outdir, "measurements")
        res_root  = os.path.join(meas_root, "residuals")
        if save:
            _ensure_dir(meas_root); _ensure_dir(res_root)

        PR_obs  = _slice_2d(G.get("RTKL_observed_PR_m", []))
        PR_comp = _slice_2d(G.get("VTL_computed_PR_m", []))
        PRR_obs  = _slice_2d(G.get("RTKL_observed_PRR_ms", []))
        PRR_comp = _slice_2d(G.get("VTL_computed_PRR_ms", []))

        PREF_PR   = _slice_2d(G.get("EKF_prefit_PR_m", []))
        POSTF_PR  = _slice_2d(G.get("EKF_postfit_PR_m", []))
        PREF_PRR  = _slice_2d(G.get("EKF_prefit_PRR_ms", []))
        POSTF_PRR = _slice_2d(G.get("EKF_postfit_PRR_ms", []))

        for ch in range(n_ch):
            if ch >= len(active_any) or not active_any[ch]:
                continue

            # observed vs computed
            fmc, axmc = plt.subplots(1, 2, figsize=(12,4), constrained_layout=True)
            y1 = PR_obs[:, ch]  if PR_obs.shape[1]  > ch else np.array([])
            y2 = PR_comp[:, ch] if PR_comp.shape[1] > ch else np.array([])
            axmc[0].plot(t, y1, label="Observed")
            axmc[0].plot(t, y2, linestyle="--", label="Computed")
            axmc[0].set_title(f"PR Observed vs Computed (CH{ch:02d})"); axmc[0].set_xlabel("Time [s]"); axmc[0].set_ylabel("m"); axmc[0].grid(True, alpha=0.3)

            y1r = PRR_obs[:, ch]  if PRR_obs.shape[1]  > ch else np.array([])
            y2r = PRR_comp[:, ch] if PRR_comp.shape[1] > ch else np.array([])
            axmc[1].plot(t, y1r, label="Observed")
            axmc[1].plot(t, y2r, linestyle="--", label="Computed")
            axmc[1].set_title(f"PRR Observed vs Computed (CH{ch:02d})"); axmc[1].set_xlabel("Time [s]"); axmc[1].set_ylabel("m/s"); axmc[1].grid(True, alpha=0.3)
            axmc[0].legend(loc="best")
            if save: fmc.savefig(os.path.join(meas_root, f"observed_vs_computed_CH{ch:02d}.png"), dpi=150)
            plt.close(fmc)

            # residuals
            fr, axr = plt.subplots(1, 2, figsize=(12,4), constrained_layout=True)
            yp1 = PREF_PR[:, ch]  if PREF_PR.shape[1]  > ch else np.array([])
            yp2 = POSTF_PR[:, ch] if POSTF_PR.shape[1] > ch else np.array([])
            axr[0].plot(t, yp1, label="Prefit")
            axr[0].plot(t, yp2, label="Postfit")
            axr[0].set_title(f"PR Residuals (CH{ch:02d})"); axr[0].set_xlabel("Time [s]"); axr[0].set_ylabel("m"); axr[0].grid(True, alpha=0.3)

            yrr1 = PREF_PRR[:, ch]  if PREF_PRR.shape[1]  > ch else np.array([])
            yrr2 = POSTF_PRR[:, ch] if POSTF_PRR.shape[1] > ch else np.array([])
            axr[1].plot(t, yrr1, label="Prefit")
            axr[1].plot(t, yrr2, label="Postfit")
            axr[1].set_title(f"PRR Residuals (CH{ch:02d})"); axr[1].set_xlabel("Time [s]"); axr[1].set_ylabel("m/s"); axr[1].grid(True, alpha=0.3)
            axr[0].legend(loc="best")
            if save: fr.savefig(os.path.join(res_root, f"residuals_CH{ch:02d}.png"), dpi=150)
            plt.close(fr)

    if show:
        plt.show()

    return figs