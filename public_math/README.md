# Public math reference

This directory exposes only the device-independent mathematical layer from the private calibration project. It is intended for review, teaching, and reproducible report calculations.

Included:

- `se3.py`: rigid transforms, exponential/log maps, Jacobian, inversion, and validation;
- `hand_eye.py`: Park-Martin motion-pair selection and `A_ij X = X B_ij` initialization;
- `robust_fit.py`: optional `SE(3)` joint refinement with one soft-L1 loss per pose;
- `metrics.py`: error summaries, effective coverage, group bootstrap confidence intervals, Wilson intervals, and first-order uncertainty budgets.

Device adapters, capture orchestration, private run data, credentials, and GUI code are intentionally excluded.

## Equations

The reference implementation follows these report-facing definitions:

$$
T_{BE}T_{EC}=T_{BW}T_{WC}, \qquad A_{ij}X=XB_{ij}
$$

$$
e_p=\left\|p_{\mathrm{Quest}}-p_{\mathrm{Robot}}\right\|_2, \qquad
e_R=\operatorname{angle}\!\left(T_{\mathrm{Robot}}^{-1}T_{\mathrm{Quest}}\right)
$$

$$
C(t)=\frac{N_{\mathrm{valid}}(t)}{N_{\mathrm{candidate}}(t)}, \qquad
v_k=\frac{\left\|p_{k+1}-p_{k-1}\right\|_2}{t_{k+1}-t_{k-1}}
$$

Confidence limits are computed by resampling independent pose or trajectory groups. A Wilson interval is used for a valid/candidate proportion; it is not presented as a precision guarantee.

Install the reference dependencies with:

```bash
python -m pip install -r requirements.txt
```
