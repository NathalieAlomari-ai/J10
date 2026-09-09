import pytest

from cv_node.smoothing import EmaSmoother


def test_first_update_passes_through_unchanged():
    s = EmaSmoother(alpha=0.3)
    assert s.update(1.0) == pytest.approx(1.0)


def test_converges_toward_a_constant_target():
    s = EmaSmoother(alpha=0.3)
    s.update(0.0)
    for _ in range(50):
        value = s.update(1.0)
    assert value == pytest.approx(1.0, abs=1e-3)


def test_smooths_a_single_spike():
    s = EmaSmoother(alpha=0.3)
    s.update(0.0)
    spiked = s.update(10.0)
    assert 0.0 < spiked < 10.0  # damped, not fully passed through


def test_alpha_one_disables_smoothing():
    s = EmaSmoother(alpha=1.0)
    s.update(0.0)
    assert s.update(5.0) == pytest.approx(5.0)


@pytest.mark.parametrize("bad_alpha", [0.0, -0.1, 1.1])
def test_rejects_alpha_out_of_range(bad_alpha):
    with pytest.raises(ValueError):
        EmaSmoother(alpha=bad_alpha)


def test_reset_forgets_prior_state():
    s = EmaSmoother(alpha=0.3)
    s.update(10.0)
    s.update(10.0)
    s.reset()
    assert s.update(1.0) == pytest.approx(1.0)  # behaves like the first-ever update again
