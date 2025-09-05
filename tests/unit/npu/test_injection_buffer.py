import pytest

from feagi.npu.fcl_injection_service import FCLInjectionService


class DummyFCL:
    def __init__(self):
        self.updates = []

    def update_fcl(self, current_timestep, neurons_by_cortical):
        # Store a copy for assertions
        self.updates.append((int(current_timestep), {k: list(v) for k, v in neurons_by_cortical.items()}))


class DummySAH:
    def __init__(self):
        self.connectome_manager = type("C", (), {"cortical_areas": {"_power": type("A", (), {"cortical_idx": 1})()}})()

    def get_power_area_neurons(self):
        # two duplicates to test coalescing
        return [10, 11, 11, 12]

    def get_special_config(self, cid):
        return type("Cfg", (), {"enabled": True, "injection_timing": "pre_burst", "injection_probability": 1.0})()

    def record_injection(self):
        pass


def make_service(cap_total=16, cap_area=16, max_total=8, max_area=4):
    fcl = DummyFCL()
    sah = DummySAH()
    svc = FCLInjectionService(fcl_manager=fcl, special_area_handler=sah)
    # Override budgets deterministically
    svc._buffer_capacity_total = cap_total
    svc._buffer_capacity_per_area = cap_area
    svc._drain_max_total = max_total
    svc._drain_max_per_area = max_area
    svc._coalesce_duplicates = True
    svc._fairness = "round_robin"
    svc._drop_policy = "newest"
    return svc, fcl


def test_enqueue_and_drain_pre_burst_coalesce_and_budget():
    svc, fcl = make_service()

    # Enqueue power neurons
    count = svc.inject_candidates(current_timestep=0)
    # Should return accepted + drained; with budgets, drained <= max_total
    assert count > 0

    # FCL should have one update at t=0
    assert fcl.updates, "No FCL updates recorded"
    t, by_area = fcl.updates[-1]
    assert t == 0
    # Coalesced set {10,11,12}
    assert 1 in by_area
    assert sorted(by_area[1]) == [10, 11, 12]


def test_global_and_per_area_budgets_respected():
    svc, fcl = make_service(max_total=3, max_area=2)

    # Enqueue twice to build up buffer a bit
    svc.inject_candidates(current_timestep=0)
    # Inject again at t=1; drain occurs each injection
    svc.inject_candidates(current_timestep=1)

    # Check that each update drained no more than budgets
    for _, by_area in fcl.updates:
        drained = sum(len(v) for v in by_area.values())
        assert drained <= 3
        for v in by_area.values():
            assert len(v) <= 2


def test_buffer_status_reporting():
    svc, _ = make_service()
    status = svc.get_buffer_status()
    assert set(status.keys()) >= {"capacity_total", "capacity_per_area", "buffered_total", "areas", "last_drain"}


