#!/usr/bin/env python

PKG='osm_cartography'
import roslib; roslib.load_manifest(PKG)

import unittest

try:
    import unique_id
except ImportError:
    pass

from geographic_msgs.msg import GeographicMap

from geodesy import bounding_box
from osm_cartography.geo_map import *
from osm_cartography import xml_map

#def fromLatLong(lat, lon, alt=float('nan')):
#    """Generate WayPoint from latitude, longitude and (optional) altitude.
#
#    :returns: minimal WayPoint object just for test cases.
#    """
#    geo_pt = GeoPoint(latitude = lat, longitude = lon, altitude = alt)
#    return WayPoint(position = geo_pt)

class TestGeoMap(unittest.TestCase):
    """Unit tests for GeoMap class.
    """

    def test_empty_map_features(self):
        gm = GeoMap(GeographicMap())
        self.assertEqual(gm.n_features, 0)

        # test GeoMapFeatures iterator with empty list
        gf = GeoMapFeatures(gm)
        i = 0
        for f in gf:
            self.fail(msg='there are no features in this map')
            i += 1
        self.assertEqual(i, 0)
        self.assertEqual(len(gf), 0)
        uu = 'da7c242f-2efe-5175-9961-49cc621b80b9'
        with self.assertRaises(KeyError):
            x = gf[uu]

    def test_tiny_map_features(self):
        gm = GeoMap(xml_map.get_osm('package://osm_cartography/tests/tiny.osm',
                                    bounding_box.makeGlobal()))
        self.assertEqual(gm.n_features, 2)

        # expected feature IDs
        uuids = ['8e8b355f-f1e8-5d82-827d-91e688e807e4',
                 '199dd143-8309-5401-9728-6ca5e1c6e235']

        # test GeoMapFeatures iterator
        gf = GeoMapFeatures(gm)
        i = 0
        for f in gf:
            if type(f.id.uuid) == str(): # old-style message?
                self.assertEqual(f.id.uuid, uuids[i])
            else:
                self.assertEqual(unique_id.toHexString(f.id), uuids[i])
            i += 1
        self.assertEqual(i, 2)
        self.assertEqual(len(gf), 2)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_geo_map_py', TestGeoMap) 
