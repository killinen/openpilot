from openpilot.selfdrive.car.i30.carcontroller import CarController as I30CarController


class CarController(I30CarController):
  def __init__(self, dbc_name, CP, VM):
    # State is decoded with the J1939 DBC, but SSC steering commands are packed
    # with the i30 actuator DBC and sent on bus 1 by the shared controller.
    super().__init__("hyundai_i30_2014", CP, VM)
