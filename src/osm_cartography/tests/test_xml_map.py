#!/usr/bin/env python

PKG='osm_cartography'
import roslib; roslib.load_manifest(PKG)

import unittest

from geodesy import bounding_box

from osm_cartography import xml_map

class TestXmlMap(unittest.TestCase):
    """Unit tests for OSM XML map parser.
    """

    def test_tiny_osm_file(self):
        # :todo: deeper results verification
        m = xml_map.get_osm('package://osm_cartography/tests/tiny.osm',
                            bounding_box.makeGlobal())
        self.assertEqual(len(m.points), 3)
        self.assertEqual(len(m.features), 2)

    def test_prc_osm_file(self):
        # :todo: deeper results verification
        m = xml_map.get_osm('package://osm_cartography/tests/prc.osm',
                            bounding_box.makeGlobal())
        self.assertEqual(len(m.points), 986)
        self.assertEqual(len(m.features), 84)

    def test_invalid_url(self):
        self.assertRaises(ValueError, xml_map.get_osm,
                          'ftp://osm_cartography/tests/prc.osm',
                          bounding_box.makeGlobal())

    def test_empty_osm_file(self):
        self.assertRaises(ValueError, xml_map.get_osm,
                          'package://osm_cartography/tests/empty.osm',
                          bounding_box.makeGlobal())

    def test_missing_osm_file(self):
        self.assertRaises(ValueError, xml_map.get_osm,
                          'package://osm_cartography/tests/missing.osm',
                          bounding_box.makeGlobal())

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_xml_map_py', TestXmlMap) 
