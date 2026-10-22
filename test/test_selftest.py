"""The loopback check must reject a trigger shifted by whole periods."""

import importlib.util
import os

import numpy as np
import pytest

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'ad_selftest.py')


@pytest.fixture(scope='module')
def selftest():
    spec = importlib.util.spec_from_file_location('ad_selftest', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def square(n, edges, noise_v=0.005, seed=0):
    """-0.5 V, rising to +0.5 V at each index in edges (falling back 300 later)."""
    v = np.full(n, -0.5)
    for e in edges:
        v[e:e + 300] = 0.5
    return v + np.random.default_rng(seed).normal(0.0, noise_v, n)


def test_one_edge_at_the_trigger_passes(selftest):
    ok, edges = selftest.loopback_edge_ok(square(8192, [2000]), 2000)
    assert ok and edges == [2000]


def test_a_trigger_shifted_by_one_period_fails(selftest):
    # The edge landed 1000 samples (one 1 kHz period) after the reported trigger.
    assert not selftest.loopback_edge_ok(square(8192, [3000]), 2000)[0]


def test_two_edges_in_the_capture_fail(selftest):
    # A wave rising more than once per capture cannot prove the position.
    assert not selftest.loopback_edge_ok(square(8192, [1000, 2000]), 2000)[0]


def test_no_edge_fails(selftest):
    assert not selftest.loopback_edge_ok(np.full(8192, -0.5), 2000)[0]
