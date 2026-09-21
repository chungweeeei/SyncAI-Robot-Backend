"""MapGateway with FAST-LIO2's ``interface`` package absent.

The counterpart to test_map_gateway.py, which ``importorskip``s ``interface``
and therefore says nothing about the configuration the runtime image actually
ships today: without that package the four clients it types -- pgo's
``save_maps`` / ``reset_mapping`` and the localizer's ``relocalize`` /
``relocalize_check`` -- are never registered, and the methods behind them
refuse instead of raising ``AttributeError`` on a ``None`` client.

Deliberately **not** skipped when ``interface`` is importable. The branch is
selected by the module's ``_INTERFACE_SRVS`` flag, so patching that flag tests
it in either image; gating this file on the package being *missing* would mean
the branch is only ever exercised in the one environment where nobody looks.

TEMPORARY, like the guard it covers: when ``interface`` gets its own repo and
interface.repos names it, the guard goes and this file goes with it.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")

from builtin_interfaces.msg import Time  # noqa: E402
from nav2_msgs.srv import LoadMap  # noqa: E402

from syncai_backend.gateways.map import map as map_module  # noqa: E402
from syncai_backend.gateways.map.map import MapGateway  # noqa: E402


@pytest.fixture
def gw(logger, monkeypatch):
    """A MapGateway constructed as if ``interface`` had failed to import.

    ``created`` records every ``create_client`` call rather than keying mocks by
    srv_name the way test_map_gateway.py does: here the point is which clients
    are *not* asked for, and a name-keyed dict would answer that with a KeyError
    from inside the constructor instead of a readable assertion.
    """
    monkeypatch.setattr(map_module, "_INTERFACE_SRVS", False)

    created = []
    load_map_client = MagicMock()

    def create_client(srv_type, srv_name):
        created.append(srv_name)
        return load_map_client

    node = MagicMock()
    node.create_client.side_effect = create_client
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()

    gateway = MapGateway(logger=logger, node=node)
    gateway._initial_pose_pub.get_subscription_count.return_value = 1

    return SimpleNamespace(gateway=gateway, created=created, load_map=load_map_client)


class TestRegistration:
    """What the constructor does, and does not, ask the node for."""

    def test_only_the_load_map_client_is_created(self, gw):
        # load_map is nav2_msgs and always available; the other four are typed
        # by `interface` and must not be requested at all -- create_client with
        # srv_type=None is a TypeError deep in rclpy, i.e. a failure to start.
        assert gw.created == ["map_server/load_map"]
        assert set(gw.gateway._service_clients) == {"load_map"}

    def test_the_missing_package_is_named_at_startup(self, logger, monkeypatch):
        # The one signal an operator gets before a route refuses. A MagicMock
        # logger rather than the structlog one from conftest, because the
        # assertion is about the call, not the rendered line.
        monkeypatch.setattr(map_module, "_INTERFACE_SRVS", False)
        mock_logger = MagicMock()
        node = MagicMock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()

        MapGateway(logger=mock_logger, node=node)

        mock_logger.warning.assert_called_once()
        assert "interface" in mock_logger.warning.call_args[0][0]


class TestTheRefusals:
    """Every route that needs a pgo or localizer service answers, not raises."""

    @pytest.mark.parametrize(
        "call",
        [
            pytest.param(lambda g: g.save_map("/home/x/map/full"), id="save_map"),
            pytest.param(lambda g: g.reset_mapping(), id="reset_mapping"),
            pytest.param(
                lambda g: g.swap_localizer_map("/home/x/map/full/map.pcd", 1.0, 2.0, 0.5),
                id="swap_localizer_map",
            ),
        ],
    )
    def test_it_refuses_with_the_shared_sentence(self, gw, call):
        success, message = call(gw.gateway)

        assert success is False
        # The shared constant, not a paraphrase: the router surfaces it
        # verbatim, and it names the cause rather than "service unavailable",
        # which an operator reads as the robot being in the wrong mode.
        assert message == map_module._NO_INTERFACE
        assert "interface" in message

    def test_nothing_is_dispatched_on_the_surviving_client(self, gw):
        # The guards return before any client is touched, so the one client
        # that does exist must be left alone -- a refusal that still called
        # load_map would be a far stranger bug than the missing package.
        gw.gateway.save_map("/home/x/map/full")
        gw.gateway.reset_mapping()
        gw.gateway.swap_localizer_map("/home/x/map/full/map.pcd", 0.0, 0.0, 0.0)

        gw.load_map.call_async.assert_not_called()
        gw.load_map.wait_for_service.assert_not_called()

    def test_a_map_switch_seeds_no_initial_pose(self, gw):
        # swap_localizer_map's second half publishes `initialpose`. Refusing
        # early has to skip it: a pose seeded for a map the localizer was never
        # pointed at is worse than no pose.
        gw.gateway.swap_localizer_map("/home/x/map/full/map.pcd", 1.0, 2.0, 0.5)

        gw.gateway._initial_pose_pub.publish.assert_not_called()

    def test_localization_converged_is_unknown_rather_than_false(self, gw):
        # None is "cannot be asked", which is exactly the state here. False
        # would claim the localizer answered and said no.
        assert gw.gateway.localization_converged() is None

    def test_nav_services_ready_is_false(self, gw):
        # `relocalize` is not in the dict at all, so the unguarded version of
        # this would be a KeyError rather than a False -- a 500 on the map
        # switch preflight instead of a refusal.
        assert gw.gateway.nav_services_ready() is False
        gw.load_map.wait_for_service.assert_not_called()


class TestWhatStillWorks:
    """The nav2 half of the gateway is untouched by the guard."""

    def test_reload_map_still_reaches_map_server(self, gw, monkeypatch):
        gw.load_map.wait_for_service.return_value = True
        gw.load_map.call_async.return_value = MagicMock(
            result=lambda: LoadMap.Response(result=LoadMap.Response.RESULT_SUCCESS)
        )
        monkeypatch.setattr(map_module, "_wait_for_future", lambda f, timeout: True)

        assert gw.gateway.reload_map(yaml_path="/home/x/map/full/gridmap.yaml") == (
            True,
            "",
        )
        gw.load_map.call_async.assert_called_once()
