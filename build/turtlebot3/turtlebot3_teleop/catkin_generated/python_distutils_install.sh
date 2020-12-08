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

echo_and_run cd "/home/emilia/turtlebot_control/src/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/emilia/turtlebot_control/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/emilia/turtlebot_control/install/lib/python3/dist-packages:/home/emilia/turtlebot_control/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/emilia/turtlebot_control/build" \
    "/usr/bin/python3" \
    "/home/emilia/turtlebot_control/src/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/emilia/turtlebot_control/build/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/emilia/turtlebot_control/install" --install-scripts="/home/emilia/turtlebot_control/install/bin"
