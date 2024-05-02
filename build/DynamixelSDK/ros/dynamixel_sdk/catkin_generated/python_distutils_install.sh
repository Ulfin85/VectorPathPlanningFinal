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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ulfin/catkin_ws/src/DynamixelSDK/ros/dynamixel_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ulfin/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ulfin/catkin_ws/install/lib/python3/dist-packages:/home/ulfin/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ulfin/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/ulfin/catkin_ws/src/DynamixelSDK/ros/dynamixel_sdk/setup.py" \
     \
    build --build-base "/home/ulfin/catkin_ws/build/DynamixelSDK/ros/dynamixel_sdk" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ulfin/catkin_ws/install" --install-scripts="/home/ulfin/catkin_ws/install/bin"
