"""Offline corner geometry and COM from static suspension; never opens hardware.

Motor-face coordinates are projections onto the outward normals of M1/M2/M3.
Their ordering need not be right-handed; this module uses Euclidean line
projections, then maps positions into the actual IMU frame before reporting
firmware Pitch/Roll. BDIR/encoder signs are not face-normal measurements.

At rest the firmware's normalized accelerometer vector points UP. For a freely
suspended body its COM lies BELOW the suspension point: C=P-lambda*g_up.
For corner balance the COM must lie ABOVE the supporting corner instead.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError('Expected a finite three-vector')
    return result


def unit(value):
    result = vector(value)
    length = np.linalg.norm(result)
    if length < 1e-9:
        raise ValueError('Direction has zero length')
    return result / length


def face_basis(normals):
    basis = np.column_stack([unit(value) for value in normals])
    if basis.shape != (3, 3) or not np.allclose(basis.T @ basis, np.eye(3), atol=1e-3):
        raise ValueError('Three measured motor-face normals must be orthogonal')
    return basis


def fit_suspension(points_mm, gravity_up):
    """Least-squares intersection of gravity lines; reports geometry quality.

    All positions and directions must use the SAME body frame. At least two
    nonparallel gravity lines are necessary; three or more check consistency.
    """
    if len(points_mm) != len(gravity_up) or len(points_mm) < 2:
        raise ValueError('At least two distinct suspension observations required')
    points = [vector(p) for p in points_mm]
    directions = [unit(g) for g in gravity_up]
    projections = [np.eye(3) - np.outer(g, g) for g in directions]
    lhs = sum(projections)
    condition = float(np.linalg.cond(lhs))
    if not math.isfinite(condition) or condition > 1000:
        raise ValueError('Gravity lines nearly parallel; COM is not identifiable')
    center = np.linalg.solve(lhs, sum(q @ p for q, p in zip(projections, points)))
    residuals = [float(np.linalg.norm(q @ (center - p))) for p, q in zip(points, projections)]
    downward = [float(-np.dot(center - p, g)) for p, g in zip(points, directions)]
    rms = math.sqrt(sum(r*r for r in residuals) / len(residuals))
    return {'center_mm': center.tolist(), 'condition_number': condition,
            'line_residuals_mm': residuals, 'rms_residual_mm': rms,
            'com_below_suspension_mm': downward,
            'geometry_quality_ok': rms <= 2 and all(d > 0 for d in downward)}


def firmware_angles(gravity_up):
    x, y, z = unit(gravity_up)
    return {'pitch_deg': math.degrees(math.atan2(y, math.hypot(x, z))),
            'roll_deg': math.degrees(math.atan2(-x, z))}


def calculate(data):
    side = float(data['cube_side_mm'])
    if not math.isfinite(side) or side <= 0:
        raise ValueError('Invalid cube size')
    support = vector(data['support_corner_motor_face_mm'])
    ideal = unit(-support)
    result = {'support_corner_motor_face_mm': support.tolist(),
              'geometric_center_reference_motor_face': ideal.tolist(),
              'geometric_case_only': True,
              'board_normal_to_vertical_deg_if_com_at_center':
                  math.degrees(math.acos(float(np.clip(-ideal[1], -1, 1)))),
              'actual_com_reference_available': False,
              'physical_balance_verified': False}
    normals = data.get('signed_motor_face_normals_imu')
    if normals is None:
        result['missing'] = ['signed motor-face normals in the IMU frame',
                             'two or more static free-suspension observations']
        return result
    basis = face_basis(normals)
    result['geometric_case_imu_angles'] = firmware_angles(basis @ ideal)
    observations = data.get('suspension_observations', [])
    if len(observations) < 2:
        result['missing'] = ['two or more static free-suspension observations']
        return result
    # Fit in the motor-face projection coordinates, whose metric is Euclidean.
    fit = fit_suspension([o['point_motor_face_mm'] for o in observations],
                         [basis.T @ unit(o['gravity_up_imu']) for o in observations])
    result['suspension_fit'] = fit
    if not fit['geometry_quality_ok']:
        result['missing'] = ['consistent freely suspended static measurements']
        return result
    gravity = unit(basis @ (vector(fit['center_mm']) - support))
    result.update(actual_com_reference_available=True,
                  center_motor_face_mm=fit['center_mm'],
                  reference_gravity_imu=gravity.tolist(),
                  reference_angles=firmware_angles(gravity))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = calculate(json.loads(args.input.read_text(encoding='utf-8-sig')))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
