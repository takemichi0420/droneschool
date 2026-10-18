import backend.main as main
from backend.main import DroneController, should_start_worker, resolve_command_target, send_velocity_command


def test_velocity_vector_uses_heading_for_forward():
    controller = DroneController()

    vector = controller.build_velocity_vector("moveForward", 0.0)
    assert vector == (0.5, 0.0, 0.0)

    vector = controller.build_velocity_vector("moveForward", 90.0)
    assert vector == (0.0, 0.5, 0.0)

    vector = controller.build_velocity_vector("moveRight", 90.0)
    assert vector == (0.5, 0.0, 0.0)


def test_control_commands_start_worker_when_needed():
    assert should_start_worker("takeoff", False) is True
    assert should_start_worker("connect", False) is False
    assert should_start_worker("takeoff", True) is False


def test_resolve_command_target_falls_back_to_vehicle():
    class DummyVehicle:
        target_system = 2
        target_component = 3

    assert resolve_command_target(DummyVehicle()) == (2, 3)


def test_velocity_command_is_sent_when_heading_is_zero():
    class DummyMAV:
        def __init__(self):
            self.calls = []

        def set_position_target_local_ned_send(self, *args):
            self.calls.append(args)

    class DummyVehicle:
        def __init__(self):
            self.mav = DummyMAV()
            self.mode = None

        def set_mode(self, mode):
            self.mode = mode

    main.current_target_system = 1
    main.current_target_component = 1
    main.get_state_snapshot = lambda: {"heading": 0}

    vehicle = DummyVehicle()
    send_velocity_command(vehicle, "moveForward")

    assert vehicle.mav.calls
    assert vehicle.mav.calls[0][3] == main.mavutil.mavlink.MAV_FRAME_BODY_NED
