import importlib.util
from pathlib import Path

import numpy as np

path = Path(__file__).resolve().parents[1] / 'scripts/d435_depth_processing.py'
spec = importlib.util.spec_from_file_location('d435_processing', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_invalid_and_out_of_range_geometry_stays_invalid():
    raw = np.array([[np.nan, np.inf, 0.1, 0.279, 0.5, 11]], np.float32)
    result = module.model_depth(raw, 674, .05, 0, .001, .28, 10, np.random.default_rng(0))
    assert np.isnan(result[0, [0, 1, 2, 3, 5]]).all()
    assert result[0, 4] == .5


def test_depth_is_quantized_in_millimetres():
    raw = np.array([[.5001, .5008, 1.0124]], np.float32)
    result = module.model_depth(raw, 674, .05, 0, .001, .28, 10, np.random.default_rng(0))
    np.testing.assert_allclose(result, [[.500, .501, 1.012]], atol=1e-6)


def test_stereo_depth_uncertainty_grows_quadratically():
    sigmas = []
    for distance in (1., 2.):
        result = module.model_depth(np.full((200, 500), distance, np.float32),
                                    674, .05, .08, .00001, .28, 10, np.random.default_rng(2))
        sigmas.append(np.std(result))
    assert 3.9 < sigmas[1] / sigmas[0] < 4.1
    assert abs(sigmas[0] - .08 / (674 * .05)) < .0001


def test_registration_transforms_coordinates_and_projects_rgb():
    depth = np.ones((1, 2), np.float32)
    k = np.array([[100, 0, 0], [0, 100, 0], [0, 0, 1]], np.float32)
    rgb = np.array([[[0, 0, 0], [0, 0, 0], [255, 1, 2], [3, 255, 4]]], np.uint8)
    points = module.registered_points(depth, k, k, rgb, np.eye(3), np.array([.02, 0, 0]))
    np.testing.assert_allclose(points['x'], [[.02, .03]])
    np.testing.assert_allclose(points['z'], [[1., 1.]])
    assert points['rgb'].tolist() == [[0xFF0102, 0x03FF04]]
    assert points.dtype.itemsize == 16


def test_invalid_depth_not_replaced_with_a_valid_point():
    k = np.eye(3, dtype=np.float32)
    result = module.registered_points(np.array([[np.nan]], np.float32), k, k,
                                      np.zeros((1, 1, 3), np.uint8), np.eye(3), np.zeros(3))
    assert np.isnan(result['z']).all()
    assert result['rgb'][0, 0] == 0
