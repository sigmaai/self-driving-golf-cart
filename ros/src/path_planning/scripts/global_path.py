import googlemaps
from datetime import datetime
import json

class GlobalPathPlanner():

    def __init__(self):
        self.gmaps = googlemaps.Client(key="AIzaSyDDAfjrlHHk8murckGuT0mh4yU5QG7dH-g")
        print("Google Maps initialized")

    def direction(self, start, destination):

        now = datetime.now()
        direction = self.gmaps.directions(start, destination, mode="driving",
                                                units="metric",
                                                departure_time=now)[0]
        legs = direction['legs'][0]
        return legs['steps']

