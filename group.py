#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState, ParamValue
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome, ParamSet, ParamGet

instances_num = 12  # количество аппаратов
freq = 20  # Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
speed = 5

action_state = "takeoff"
formation_number = 0 # 0-3: C S P A
data = {}
lz = {}

current_corner = 2
# ==================== corners
# 3 ________________ 2
# =                  =
# =                  =
# 4 ________________ 1
# ====================
corners = {
    # [x, y]
    # относительно места взлёта
    # They're 100% wrong so debug first position please
    1: {"x": -35, "y": -72},
    2: {"x": 35, "y": -72},
    3: {"x": 35, "y": 72},
    4: {"x": -35, "y": 72},
}

formations = {
    0: [
        { "x": 0, "y": 0, "z": 16 },
        { "x": 0, "y": -1, "z": 4 },
        { "x": 0, "y": -0.5, "z": 6.5 },
        { "x": 0, "y": -0.5, "z": 18.5 },
        { "x": 0, "y": 1, "z": 4 },
        { "x": 0, "y": 0, "z": 16 }, # 6
        { "x": 2, "y": 0, "z": 13 },
        { "x": 2, "y": -1, "z": 1 },
        { "x": 2, "y": 0.5, "z": 3.5 },
        { "x": 2, "y": -0.5, "z": 15.5 },
        { "x": 2, "y": 1, "z": 1 },
        { "x": 2, "y": 0, "z": 13 },
    ]
}


#!! MAIN LOOP

def do_formation(pt: PositionTarget, drone_id: int, dt: float) -> str or None:
    global current_corner, formation_number

    if current_corner == 1 or current_corner == 3:
        return "go_next_corner"

    drone_pos = data[drone_id]["local_position/pose"].pose.position
    next_pos = formations[formation_number][drone_id-1]

    set_pos(
        pt,
        drone_pos.x + next_pos["x"],
        drone_pos.y + next_pos["y"],
        drone_pos.z + next_pos["z"]
    )

    return "go_next_corner"
    


def prepare_for_loop(pt: PositionTarget, drone_id: int, dt: float) -> str or None:
    distance = corners[2]["x"]
    estimated_arrival = distance / speed

    set_vel(pt, speed, 0, 0)

    if estimated_arrival < dt:
        drone_pos = data[drone_id]["local_position/pose"].pose.position
        set_vel(pt, 0, 0, 0)
        # Get in there Lewis
        if drone_id <= 6:
            set_pos(pt, corners[2]["x"], drone_pos.y, drone_pos.z)
        else:
            set_pos(pt, corners[2]["x"]-2, drone_pos.y, drone_pos.z)

        return "do_formation"


def go_next_corner(pt: PositionTarget, drone_id: int, dt: float) -> str or None:
    global current_corner
    next_corner = current_corner + 1 if current_corner != 4 else 1
    axis = "x" if next_corner == 2 or next_corner == 4 else "y"

    corner_pos = corners[current_corner]
    next_corner_pos = corners[next_corner]

    distance = next_corner_pos[axis] - corner_pos[axis]
    estimated_arrival = abs(distance / speed)
    velocity = speed if distance >= 0 else -speed

    if axis == "y":
        set_vel(pt, 0, velocity, 0)
    else:
        set_vel(pt, velocity, 0, 0)

    if estimated_arrival < dt:
        drone_pos = data[drone_id]["local_position/pose"].pose.position
        set_vel(pt, 0, 0, 0)
        if drone_id == instances_num:
            current_corner = next_corner
        if axis == "x":
            set_pos(pt, next_corner_pos["x"], drone_pos.y, drone_pos.z)

        return "do_formation"


def offboard_loop():
    global action_state
    pub_pt = {}
    # создаем топики, для публикации управляющих значений
    for n in range(1, instances_num + 1):
        pub_pt[n] = rospy.Publisher(
            f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

    pt = PositionTarget()
    # см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    # t0 = time.time()
    ts = time.time()
    _failsafe_state = action_state

    # цикл управления
    rate = rospy.Rate(freq)
    print(
        f"\x1b[31m Default Local Position of first copter {data[1]['local_position/pose'].pose.position} \x1b[0m")
    # print(adgsdf)
    while not rospy.is_shutdown():
        dt = time.time() - ts  # time since change of action state

        # управляем каждым аппаратом централизованно
        for n in range(1, instances_num + 1):
            set_mode(n, "OFFBOARD")
            action_did_finish = actions[action_state](pt, n, dt)
            pub_pt[n].publish(pt)
            if type(action_did_finish) == type("string"):
                _failsafe_state = action_did_finish
        else:
            # safety to ensure that every drone made its last action
            if _failsafe_state != action_state:
                print(
                    f"State changed from {action_state} to {_failsafe_state}")
                if _failsafe_state == "go_next_corner":
                    print(f"Current corner - {current_corner}")
                action_state = _failsafe_state
                # Maybe could sleep here
                if action_state == "do_formation":
                    time.sleep(10)
                else:
                    time.sleep(3)
                ts = time.time()

        rate.sleep()

#!! DRONE COMMON COMMANDS


def set_pos(pt, x, y, z):
    pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    pt.position.x = x
    pt.position.y = y
    pt.position.z = z


def set_vel(pt, vx, vy, vz):
    pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    pt.velocity.x = vx
    pt.velocity.y = vy
    pt.velocity.z = vz


def arming(n, to_arm):
    d = data[n].get("state")
    if d is not None and d.armed != to_arm:
        service_proxy(n, "cmd/arming", CommandBool, to_arm)


def mc_takeoff(pt, n, dt) -> str or None:
    set_vel(pt, 0, 0, 2)

    if dt > 3:
        arming(n, True)

    if dt > 8:
        return "prepare_for_loop"


# Actions list
actions = {
    "takeoff": mc_takeoff,
    "prepare_for_loop": prepare_for_loop,
    "do_formation": do_formation,
    "go_next_corner": go_next_corner,
}

#!! LOW LEVEL CODE. HIDE IT


def subscribe_on_mavros_topics(suff, data_class):
    # подписываемся на Mavros топики всех аппаратов
    for n in range(1, instances_num + 1):
        data[n] = {}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))


def topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg


def service_proxy(n, path, arg_type, *args, **kwds):
    service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
    ret = service(*args, **kwds)

    # rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")
    return ret


def set_mode(n, new_mode):
    d = data[n].get("state")
    if d is not None and d.mode != new_mode:
        service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)


def set_param(n, name, i=0, r=0.0):
    return service_proxy(n, "param/set", ParamSet, name, ParamValue(i, r)).success


def get_param(n, name):
    ret = service_proxy(n, "param/get", ParamGet, param_id=name)

    v = None
    if ret.success:
        i = ret.value.integer
        r = ret.value.real
        if i != 0:
            v = i
        elif r != 0.0:
            v = r
        else:
            v = 0

    return v


def subscribe_on_topics():
    # глобальная (GPS) система координат
    # высота задана в элипсоиде WGS-84 и не равна высоте над уровнем моря, см. https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
    subscribe_on_mavros_topics("global_position/global", NavSatFix)

    # локальная система координат, точка отсчета = место включения аппарата
    subscribe_on_mavros_topics("local_position/pose", PoseStamped)
    subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

    # состояние
    subscribe_on_mavros_topics("state", State)
    subscribe_on_mavros_topics("extended_state", ExtendedState)


def on_shutdown_cb():
    rospy.logfatal("shutdown")


if __name__ == '__main__':
    if len(sys.argv) > 1:
        instances_num = int(sys.argv[1])

    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    subscribe_on_topics()

    rospy.on_shutdown(on_shutdown_cb)

    try:
        offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
