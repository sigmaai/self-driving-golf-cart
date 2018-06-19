#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/neil/Workspace/self-driving-golf-cart/src/osm"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/neil/Workspace/self-driving-golf-cart/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/neil/Workspace/self-driving-golf-cart/install/lib/python2.7/dist-packages:/home/neil/Workspace/self-driving-golf-cart/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/neil/Workspace/self-driving-golf-cart/build" \
    "/usr/bin/python" \
    "/home/neil/Workspace/self-driving-golf-cart/src/osm/setup.py" \
    build --build-base "/home/neil/Workspace/self-driving-golf-cart/build/osm" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/neil/Workspace/self-driving-golf-cart/install" --install-scripts="/home/neil/Workspace/self-driving-golf-cart/install/bin"
