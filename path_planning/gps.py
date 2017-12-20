import gpsd


class GPS:
    def __init__(self):
        gpsd.connect()

    @staticmethod
    def query_gps_location(self):
        packet = gpsd.get_current()
        return packet.position()
